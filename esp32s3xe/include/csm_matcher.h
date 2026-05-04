/**
 * AMR 2.0 — Correlative Scan Matching (CSM) for ESP32-S3
 * 
 * Ported from scanMatcher.js — Reference: Olson (2009) Real-Time CSM
 * 
 * Algorithm:
 *   1. Build a likelihood field from the current occupancy grid
 *      (BFS Gaussian distance transform from occupied cells)
 *   2. Coarse search: brute-force score all poses in (x±30cm, y±30cm, θ±10°)
 *      with 5cm/2° steps
 *   3. Fine search: refine around best coarse pose with 1cm/0.3° steps
 *   4. Score = avg likelihood at projected scan points → best pose = correction
 * 
 * Memory budget: ~16KB likelihood field (128×128 × 1 byte, quantized 0-255)
 *   Allocated in PSRAM if available, else SRAM.
 * 
 * Performance target: <30ms per match on ESP32-S3 @ 240MHz
 */

#ifndef CSM_MATCHER_H
#define CSM_MATCHER_H

#include <cmath>
#include <cstring>
#include "lidar_mapper.h"

struct CsmConfig {
    // Search window
    float searchWindowXY    = 0.3f;    // ±0.3m
    float searchWindowTheta = 0.175f;  // ±10°

    // Coarse resolution
    float coarseStepXY      = 0.05f;   // 5cm
    float coarseStepTheta   = 0.035f;  // ~2°

    // Fine resolution
    float fineStepXY        = 0.01f;   // 1cm
    float fineStepTheta     = 0.005f;  // ~0.3°
    float fineWindowXY      = 0.08f;   // ±8cm around coarse
    float fineWindowTheta   = 0.05f;   // ±3° around coarse

    // Likelihood field
    float likelihoodSigma   = 0.1f;    // Gaussian σ (meters)
    int   likelihoodMaxDist = 5;       // Max BFS distance (cells)

    // Thresholds
    float minTravelDist     = 0.15f;   // Min travel before matching
    float minTravelHeading  = 0.2f;    // Min heading change (rad)
    int   minScanPoints     = 30;      // Min valid points
    float minMatchScore     = 0.3f;    // Accept threshold (0-1)
    float maxCorrectionDist = 0.25f;   // Max position correction
    float maxCorrectionAngle= 0.12f;   // Max angle correction (~7°)
};

struct CsmResult {
    float dx     = 0.0f;    // Correction X (meters)
    float dy     = 0.0f;    // Correction Y (meters)
    float dTheta = 0.0f;    // Correction heading (radians)
    float score  = 0.0f;    // Match quality (0-1)
    bool  accepted = false; // Whether correction should be applied
};

class CsmMatcher {
public:
    CsmConfig config;

    // Statistics
    int   matchCount = 0;
    float lastScore  = 0.0f;
    float lastMatchMs = 0.0f;

    // Last matched pose (for travel distance check)
    float lastMatchedX     = 0.0f;
    float lastMatchedY     = 0.0f;
    float lastMatchedTheta = 0.0f;

    CsmMatcher() : _likelihoodField(nullptr) {}

    ~CsmMatcher() {
        if (_likelihoodField) free(_likelihoodField);
    }

    /**
     * Initialize the matcher. Call once in setup().
     * Returns false if memory allocation fails.
     */
    bool init() {
        int totalCells = GRID_SIZE * GRID_SIZE;
        _likelihoodField = (uint8_t*)heap_caps_malloc(
            totalCells, psramFound() ? MALLOC_CAP_SPIRAM : MALLOC_CAP_DEFAULT);
        if (!_likelihoodField) return false;
        memset(_likelihoodField, 0, totalCells);
        return true;
    }

    /**
     * Perform CSM: find best pose correction for current scan on existing grid.
     * 
     * @param mapper     Current occupancy grid mapper
     * @param odomX/Y/Theta  Current odometry pose (world frame)
     * @param scan       Array of LidarPoints (local frame, distance in meters)
     * @param scanLen    Number of valid points in scan
     * @param result     Output correction
     * @return true if a valid match was found
     */
    bool matchScan(
        const OccupancyGridMapper& mapper,
        float odomX, float odomY, float odomTheta,
        const LidarPoint* scan, int scanLen,
        CsmResult& result)
    {
        unsigned long t0 = millis();

        // Check minimum travel distance
        float travelDist = sqrtf(
            (odomX - lastMatchedX) * (odomX - lastMatchedX) +
            (odomY - lastMatchedY) * (odomY - lastMatchedY));
        float travelHeading = fabsf(_normalizeAngle(odomTheta - lastMatchedTheta));

        if (travelDist < config.minTravelDist && travelHeading < config.minTravelHeading) {
            result = CsmResult();
            return false;
        }

        // Convert scan to local Cartesian
        int localCount = 0;
        _scanToLocal(scan, scanLen, localCount);
        if (localCount < config.minScanPoints) {
            result = CsmResult();
            return false;
        }

        // Build likelihood field from current grid
        _buildLikelihoodField(mapper);

        // Phase 1: Coarse search
        float bestX, bestY, bestTheta, bestScore;
        _searchBestPose(mapper,
                        odomX, odomY, odomTheta,
                        config.searchWindowXY, config.searchWindowTheta,
                        config.coarseStepXY, config.coarseStepTheta,
                        localCount,
                        bestX, bestY, bestTheta, bestScore);

        // Phase 2: Fine search around coarse result
        float fineX, fineY, fineTheta, fineScore;
        _searchBestPose(mapper,
                        bestX, bestY, bestTheta,
                        config.fineWindowXY, config.fineWindowTheta,
                        config.fineStepXY, config.fineStepTheta,
                        localCount,
                        fineX, fineY, fineTheta, fineScore);

        // Compute correction
        float corrX = fineX - odomX;
        float corrY = fineY - odomY;
        float corrTheta = _normalizeAngle(fineTheta - odomTheta);

        // Safety clamp
        float corrDist = sqrtf(corrX * corrX + corrY * corrY);
        if (corrDist > config.maxCorrectionDist) {
            float scale = config.maxCorrectionDist / corrDist;
            corrX *= scale;
            corrY *= scale;
        }
        if (fabsf(corrTheta) > config.maxCorrectionAngle) {
            corrTheta = (corrTheta > 0) ? config.maxCorrectionAngle : -config.maxCorrectionAngle;
        }

        bool accepted = fineScore > config.minMatchScore;

        result.dx = corrX;
        result.dy = corrY;
        result.dTheta = corrTheta;
        result.score = fineScore;
        result.accepted = accepted;

        if (accepted) {
            lastMatchedX = odomX + corrX;
            lastMatchedY = odomY + corrY;
            lastMatchedTheta = _normalizeAngle(odomTheta + corrTheta);
            lastScore = fineScore;
            matchCount++;
        }

        lastMatchMs = (float)(millis() - t0);
        return accepted;
    }

    void reset() {
        lastMatchedX = lastMatchedY = lastMatchedTheta = 0;
        matchCount = 0;
        lastScore = 0;
        if (_likelihoodField) {
            memset(_likelihoodField, 0, GRID_SIZE * GRID_SIZE);
        }
    }

private:
    // Likelihood field: quantized 0-255 (0 = no likelihood, 255 = occupied cell)
    uint8_t* _likelihoodField;

    // Local scan buffer (pre-converted Cartesian)
    static const int MAX_LOCAL = 360;
    float _localX[MAX_LOCAL];
    float _localY[MAX_LOCAL];

    /**
     * Convert polar LidarPoints to local Cartesian coordinates
     */
    void _scanToLocal(const LidarPoint* scan, int scanLen, int& outCount) {
        outCount = 0;
        for (int i = 0; i < scanLen && outCount < MAX_LOCAL; i++) {
            float d = scan[i].distance;
            if (d < 0.05f || d > 3.0f) continue;
            if (!scan[i].quality) continue;

            float rad = scan[i].angle * M_PI / 180.0f;
            _localX[outCount] = cosf(rad) * d;
            _localY[outCount] = sinf(rad) * d;
            outCount++;
        }
    }

    /**
     * Build likelihood field via BFS distance transform from occupied cells.
     * Gaussian falloff: likelihood = exp(-dist² / (2σ²))
     * Quantized to uint8_t [0, 255] to save memory.
     */
    void _buildLikelihoodField(const OccupancyGridMapper& mapper) {
        const int W = GRID_SIZE;
        const int totalCells = W * W;

        // Clear: 0 = unknown/far
        memset(_likelihoodField, 0, totalCells);

        // Sigma in cells
        float sigmaC = config.likelihoodSigma / GRID_RESOLUTION;
        float sigma2 = 2.0f * sigmaC * sigmaC;
        int maxDist = config.likelihoodMaxDist;

        // BFS queue — capped to save RAM. With maxDist=5, typical environments
        // produce <5000 BFS entries. Cap at 4096 pairs = 8KB.
        static const int BFS_MAX = 4096;
        static int16_t bfsQx[BFS_MAX];
        static int16_t bfsQy[BFS_MAX];
        static uint8_t bfsDist[BFS_MAX]; // distance of each entry
        int qHead = 0, qTail = 0;

        // Precompute Gaussian LUT for distances 0..maxDist
        uint8_t gaussLut[16]; // maxDist capped at 15
        for (int d = 0; d <= maxDist && d < 16; d++) {
            gaussLut[d] = (uint8_t)(expf(-(float)(d * d) / sigma2) * 255.0f);
        }

        // Seed: all occupied cells → likelihood = 255
        for (int y = 0; y < W; y++) {
            for (int x = 0; x < W; x++) {
                if (mapper.grid[y][x] > 0) {
                    _likelihoodField[y * W + x] = 255;
                    if (qTail < BFS_MAX) {
                        bfsQx[qTail] = x;
                        bfsQy[qTail] = y;
                        bfsDist[qTail] = 0;
                        qTail++;
                    }
                }
            }
        }

        // BFS expand — only update cells with higher likelihood (closer to obstacle)
        while (qHead < qTail) {
            int gx = bfsQx[qHead];
            int gy = bfsQy[qHead];
            int curDist = bfsDist[qHead];
            qHead++;

            if (curDist >= maxDist) continue;

            int newDist = curDist + 1;
            uint8_t newLikelihood = gaussLut[newDist];

            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = gx + dx, ny = gy + dy;
                    if (nx < 0 || nx >= W || ny < 0 || ny >= W) continue;

                    int ni = ny * W + nx;
                    // Only update if new likelihood is higher (closer to obstacle)
                    if (newLikelihood > _likelihoodField[ni]) {
                        _likelihoodField[ni] = newLikelihood;
                        if (qTail < BFS_MAX) {
                            bfsQx[qTail] = nx;
                            bfsQy[qTail] = ny;
                            bfsDist[qTail] = (uint8_t)newDist;
                            qTail++;
                        }
                    }
                }
            }
        }
    }

    /**
     * Brute-force search: try all poses in window, score each,
     * return the best one.
     */
    void _searchBestPose(
        const OccupancyGridMapper& mapper,
        float centerX, float centerY, float centerTheta,
        float rangeXY, float rangeTheta,
        float stepXY, float stepTheta,
        int localCount,
        float& outX, float& outY, float& outTheta, float& outScore)
    {
        float bestScore = -1.0f;
        outX = centerX;
        outY = centerY;
        outTheta = centerTheta;
        outScore = 0.0f;

        const int W = GRID_SIZE;

        for (float dTheta = -rangeTheta; dTheta <= rangeTheta; dTheta += stepTheta) {
            float theta = centerTheta + dTheta;
            float cosT = cosf(theta);
            float sinT = sinf(theta);

            for (float dx = -rangeXY; dx <= rangeXY; dx += stepXY) {
                float px = centerX + dx;

                for (float dy = -rangeXY; dy <= rangeXY; dy += stepXY) {
                    float py = centerY + dy;

                    int totalLikelihood = 0;
                    int validCount = 0;

                    for (int i = 0; i < localCount; i++) {
                        // Transform local → world
                        float wx = px + _localX[i] * cosT - _localY[i] * sinT;
                        float wy = py + _localX[i] * sinT + _localY[i] * cosT;

                        // World → grid
                        int gx = mapper.world_to_grid_x(wx);
                        int gy = mapper.world_to_grid_y(wy);

                        if (gx >= 0 && gx < W && gy >= 0 && gy < W) {
                            totalLikelihood += _likelihoodField[gy * W + gx];
                            validCount++;
                        }
                    }

                    if (validCount > 0) {
                        float score = (float)totalLikelihood / (float)(validCount * 255);
                        if (score > bestScore) {
                            bestScore = score;
                            outX = px;
                            outY = py;
                            outTheta = theta;
                            outScore = score;
                        }
                    }
                }
            }
        }
    }

    static float _normalizeAngle(float a) {
        while (a > M_PI)  a -= 2.0f * M_PI;
        while (a < -M_PI) a += 2.0f * M_PI;
        return a;
    }
};

#endif // CSM_MATCHER_H
