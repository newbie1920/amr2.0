/**
 * AMR 2.0 — DWA Local Planner (ESP32-S3)
 * Ported from dwaPlanner.js (Web Dashboard)
 * 
 * Dynamic Window Approach with:
 *   - Pre-computed local costmap (BFS inflation) — O(1) cost lookup
 *   - Velocity sampling (vSamples × wSamples)
 *   - Trajectory simulation with collision detection
 *   - Circular footprint clearance scoring
 *   - Path alignment + goal heading + speed scoring
 *   - Rotate-in-place for large heading errors
 *   - Predictive braking near obstacles
 * 
 * All memory is statically allocated — zero malloc in real-time loop.
 * Designed to run at 10-20Hz inside controlTask (Core 1).
 * 
 * BUG #4 FIX: Replaced on-the-fly inflation (O(n²) per cell) with
 *   pre-computed local costmap using BFS flood-fill.
 *   Old: ~180ms/cycle on ESP32 → New: ~8ms/cycle
 */

#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <Arduino.h>
#include <cmath>
#include "config.h"
#include "navigator.h"       // Waypoint, MAX_WAYPOINTS
#include "occupancy_grid.h"  // OccupancyGridMapper, GRID_SIZE, GRID_RESOLUTION

// ============================================================
//   DWA CONFIGURATION
// ============================================================

struct DwaConfig {
    float maxSpeedTrans   = 0.30f;   // m/s (closer to NAV_MAX_LINEAR_VEL 0.40, conservative for DWA)
    float minSpeedTrans   = 0.0f;
    float maxSpeedRot     = 1.5f;    // rad/s
    float maxAccelTrans   = 0.8f;    // m/s²
    float maxAccelRot     = 2.5f;    // rad/s²
    float simTime         = 2.0f;    // Prediction horizon (seconds)
    float simGranularity  = 0.15f;   // seconds per step
    int   vSamples        = 7;       // linear velocity samples (reduced from 9 for ESP32)
    int   wSamples        = 15;      // angular velocity samples (reduced from 21 for ESP32)
    float robotRadius     = 0.08f;   // meters (8cm)
    float preferredClearance = 0.25f; // meters
    float stopOnClearance = 0.04f;   // meters (stop if closer than 4cm)
    float headingLookahead = 1.5f;   // meters — local goal pick distance
    float pathDistBias    = 2.0f;    // Cost weight: distance to path
    float goalDistBias    = 12.0f;   // Cost weight: distance to local goal
    float goalHeadingBias = 12.0f;   // Cost weight: heading alignment to goal
    float clearanceBias   = 15.0f;   // Cost weight: obstacle clearance
    float speedBias       = 4.0f;    // Cost weight: prefer faster trajectories
    float rotateInPlaceAngle = M_PI / 2.0f; // Threshold for pure rotation
};

// ============================================================
//   DWA OUTPUT
// ============================================================

struct DwaResult {
    float v;                // Chosen linear velocity (m/s)
    float w;                // Chosen angular velocity (rad/s)
    bool  ok;               // true = found a safe trajectory
    float score;            // Best trajectory score (lower = better)
    float clearance;        // Minimum clearance of best trajectory
    // Diagnostic counters
    int   collisionCount;
    int   clearanceRejectCount;
    int   okCount;
};

// ============================================================
//   PRE-COMPUTED LOCAL COSTMAP
// ============================================================
// Instead of scanning O(n²) neighbors per query, we BFS-inflate the
// occupancy grid ONCE per DWA cycle into a local costmap buffer.
// Then all collision/clearance checks are O(1) lookups.
//
// The costmap covers GRID_SIZE×GRID_SIZE and uses the same coordinate
// system as the OccupancyGridMapper. Stored as uint8_t:
//   254 = lethal (direct obstacle)
//   253 = inscribed (within robot radius)
//   1-252 = exponential decay (inflation zone)
//   0 = free

static constexpr int INFLATE_CELLS = 4;      // 0.4m inflation radius
static constexpr int INSCRIBED_CELLS = 2;     // 0.2m inscribed radius
static constexpr float COST_SCALING = 3.0f;   // Exponential decay factor
static constexpr int8_t OCC_THRESHOLD = 10;   // Log-odds occupied threshold

// DWA operates on a LOCAL window (256x256) centered on robot,
// not the full 1024x1024 grid. This keeps BFS fast and memory bounded.
static constexpr int DWA_LOCAL_SIZE = 256;
static_assert(DWA_LOCAL_SIZE <= 256, "DWA BFS packing requires DWA_LOCAL_SIZE <= 256");

// ============================================================
//   DWA PLANNER CLASS
// ============================================================

class DwaPlanner {
public:
    DwaConfig cfg;

    // Pre-computed costmap — allocated as static to avoid member bloat
    // Uses global static to avoid 16KB in class instance
    bool costmapValid = false;
    unsigned long lastCostmapBuildMs = 0;

    // Local window origin in global grid coords
    int _winOriginX = 0, _winOriginY = 0;

    DwaPlanner() {}

    /**
     * Rebuild the inflation costmap from the current occupancy grid.
     * Uses a LOCAL 256x256 window centered on the robot.
     * BFS flood-fill from all occupied cells within this window.
     * Cost: ~5-8ms on ESP32-S3 @ 240MHz.
     * 
     * Call this ONCE per DWA cycle, before computeVelocity().
     */
    void buildCostmap(const OccupancyGridMapper& mapper, float robotX, float robotY) {
        unsigned long t0 = micros();

        // Compute local window origin centered on robot
        int robotGX = mapper.world_to_grid_x(robotX);
        int robotGY = mapper.world_to_grid_y(robotY);
        int half = DWA_LOCAL_SIZE / 2;
        _winOriginX = constrain(robotGX - half, 0, GRID_SIZE - DWA_LOCAL_SIZE);
        _winOriginY = constrain(robotGY - half, 0, GRID_SIZE - DWA_LOCAL_SIZE);

        // PSRAM-backed arrays (256x256 = 64KB each + 128KB BFS queue)
        // Cannot fit 256KB in 320KB DRAM — must use PSRAM
        static uint8_t* costmapBuf = nullptr;
        static uint8_t* distMap = nullptr;
        static uint16_t* bfsQueue = nullptr;
        static const int CM_SIZE = DWA_LOCAL_SIZE * DWA_LOCAL_SIZE;
        static const int BFS_SIZE = DWA_LOCAL_SIZE * DWA_LOCAL_SIZE;
        
        if (!costmapBuf) {
            costmapBuf = (uint8_t*)heap_caps_malloc(CM_SIZE, MALLOC_CAP_SPIRAM);
            distMap    = (uint8_t*)heap_caps_malloc(CM_SIZE, MALLOC_CAP_SPIRAM);
            bfsQueue   = (uint16_t*)heap_caps_malloc(BFS_SIZE * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
            if (!costmapBuf || !distMap || !bfsQueue) {
                Serial.println("[DWA] PSRAM alloc failed!");
                return;
            }
        }

        memset(costmapBuf, 0, CM_SIZE);
        memset(distMap, 255, CM_SIZE); // 255 = infinity (unvisited)

        int qHead = 0, qTail = 0;
        const int BFS_CAP = DWA_LOCAL_SIZE * DWA_LOCAL_SIZE;

        // Seed BFS with all occupied cells in local window
        for (int ly = 0; ly < DWA_LOCAL_SIZE; ly++) {
            for (int lx = 0; lx < DWA_LOCAL_SIZE; lx++) {
                int gx = _winOriginX + lx;
                int gy = _winOriginY + ly;
                if (gx < 0 || gx >= GRID_SIZE || gy < 0 || gy >= GRID_SIZE) continue;
                if (mapper.grid_cell_const(gx, gy) >= OCC_THRESHOLD) {
                    costmapBuf[ly * DWA_LOCAL_SIZE + lx] = 254; // Lethal
                    distMap[ly * DWA_LOCAL_SIZE + lx] = 0;
                    if (qTail < BFS_CAP) {
                        bfsQueue[qTail++] = (uint16_t)((ly << 8) | lx);
                    }
                }
            }
        }

        // BFS expand
        const uint8_t INFLATE_DIST_MAX = (uint8_t)(INFLATE_CELLS * 20); // 4 * 20 = 80

        while (qHead < qTail) {
            uint16_t packed = bfsQueue[qHead++];
            int cy = (packed >> 8) & 0xFF;
            int cx = packed & 0xFF;
            uint8_t currentDist = distMap[cy * DWA_LOCAL_SIZE + cx];

            if (currentDist >= INFLATE_DIST_MAX) continue;

            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = cx + dx, ny = cy + dy;
                    if (nx < 0 || nx >= DWA_LOCAL_SIZE || ny < 0 || ny >= DWA_LOCAL_SIZE) continue;

                    // step = 20 for cardinal, 28 for diagonal (≈1.414 * 20)
                    uint8_t step = (dx != 0 && dy != 0) ? 28 : 20;
                    uint16_t newDistRaw = (uint16_t)currentDist + step;
                    uint8_t newDist = (newDistRaw > 255) ? 255 : (uint8_t)newDistRaw;

                    if (newDist < distMap[ny * DWA_LOCAL_SIZE + nx]) {
                        distMap[ny * DWA_LOCAL_SIZE + nx] = newDist;

                        // Compute cost from distance
                        float realDist = newDist / 20.0f; // Convert back to cells
                        uint8_t cost;
                        if (realDist <= (float)INSCRIBED_CELLS) {
                            cost = 253; // Inscribed zone
                        } else {
                            float distM = (realDist - (float)INSCRIBED_CELLS) * GRID_RESOLUTION;
                            float c = 252.0f * expf(-COST_SCALING * distM);
                            cost = (uint8_t)fmaxf(1.0f, fminf(252.0f, c));
                        }
                        costmapBuf[ny * DWA_LOCAL_SIZE + nx] = cost;

                        if (newDist < INFLATE_DIST_MAX && qTail < BFS_CAP) {
                            bfsQueue[qTail++] = (uint16_t)((ny << 8) | nx);
                        }
                    }
                }
            }
        }

        // Copy to accessible location
        _costmapPtr = costmapBuf;
        costmapValid = true;
        lastCostmapBuildMs = millis();

        unsigned long dt = micros() - t0;
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 5000) {
            lastLog = millis();
            Serial.printf("[DWA] Costmap built in %lu us, %d BFS entries (win %d,%d)\n", dt, qHead, _winOriginX, _winOriginY);
        }
    }

    /**
     * Self-clear costmap trong robot footprint.
     * Gọi sau buildCostmap(), truyền vị trí robot hiện tại.
     * Tránh phantom obstacle khi LiDAR phản xạ gần thân robot.
     * (Ported from navWorker.js self-clear logic)
     */
    void clearRobotFootprint(float robotX, float robotY, const OccupancyGridMapper& mapper) {
        if (!_costmapPtr) return;
        int rgx = mapper.world_to_grid_x(robotX);
        int rgy = mapper.world_to_grid_y(robotY);
        // Convert to local window coords
        int lrx = rgx - _winOriginX;
        int lry = rgy - _winOriginY;
        // Footprint = INSCRIBED_CELLS + 1 = 3 cells = 0.3m
        const int CLEAR_R = INSCRIBED_CELLS + 1;
        for (int dy = -CLEAR_R; dy <= CLEAR_R; dy++) {
            for (int dx = -CLEAR_R; dx <= CLEAR_R; dx++) {
                int nx = lrx + dx, ny = lry + dy;
                if (nx < 0 || nx >= DWA_LOCAL_SIZE || ny < 0 || ny >= DWA_LOCAL_SIZE) continue;
                if (dx*dx + dy*dy <= CLEAR_R*CLEAR_R) {
                    _costmapPtr[ny * DWA_LOCAL_SIZE + nx] = 0;
                }
            }
        }
    }

    /**
     * Compute the best (v, w) velocity command using DWA.
     * MUST call buildCostmap() before this each cycle.
     */
    DwaResult computeVelocity(
        float poseX, float poseY, float poseTheta,
        float curV, float curW,
        const Waypoint* path, int pathLen,
        const OccupancyGridMapper& mapper
    ) {
        DwaResult result = {0, 0, false, 1e9f, 0, 0, 0, 0};

        if (pathLen <= 0) return result;

        // ── Pick local goal (carrot) ─────────────────────────
        float goalX, goalY;
        pickLocalGoal(poseX, poseY, path, pathLen, goalX, goalY);

        float goalHeading = atan2f(goalY - poseY, goalX - poseX);
        float headingError = normalizeAngle(goalHeading - poseTheta);

        // ── ROTATE-IN-PLACE for large heading errors ─────────
        if (fabsf(headingError) > cfg.rotateInPlaceAngle) {
            if (checkCollisionAt(poseX, poseY, poseTheta, mapper)) {
                // Can't rotate — blocked
                return result;
            }
            float gain = 1.8f;
            float cmdW = (headingError > 0 ? 1.0f : -1.0f) * 
                         fminf(cfg.maxSpeedRot, fabsf(headingError) * gain);
            result.v = 0;
            result.w = cmdW;
            result.ok = true;
            result.clearance = cfg.preferredClearance;
            return result;
        }

        // ── Dynamic max speed (curve braking) ────────────────
        float dynamicVMax = cfg.maxSpeedTrans;
        float absHeadErr = fabsf(headingError);
        if (absHeadErr > 0.26f) {
            float brakeFactor = fmaxf(0.4f, 1.0f - (absHeadErr / 1.57f));
            dynamicVMax = cfg.maxSpeedTrans * brakeFactor;
        }

        // ── Predictive braking ───────────────────────────────
        if (curV > 0.05f) {
            float lt = 0.8f;
            float futX = poseX + curV * cosf(poseTheta) * lt;
            float futY = poseY + curV * sinf(poseTheta) * lt;
            float futClear = getCircularClearance(futX, futY, mapper);
            if (futClear < cfg.preferredClearance * 0.6f) {
                float urgency = 1.0f - (futClear / (cfg.preferredClearance * 0.6f));
                dynamicVMax = fminf(dynamicVMax, 
                    cfg.maxSpeedTrans * fmaxf(0.2f, 1.0f - urgency * 0.6f));
            }
        }

        // ── Dynamic Window ───────────────────────────────────
        float controlDt = 0.4f;
        float vMin = fmaxf(cfg.minSpeedTrans, curV - cfg.maxAccelTrans * controlDt);
        float vMax = fminf(dynamicVMax,        curV + cfg.maxAccelTrans * controlDt);
        float wMin = fmaxf(-cfg.maxSpeedRot,   curW - cfg.maxAccelRot * controlDt);
        float wMax = fminf(cfg.maxSpeedRot,    curW + cfg.maxAccelRot * controlDt);

        // ── Sample velocities ────────────────────────────────
        float bestScore = 1e9f;

        for (int iv = 0; iv < cfg.vSamples; iv++) {
            float v = sampleLinear(vMin, vMax, iv, cfg.vSamples);

            for (int iw = 0; iw < cfg.wSamples; iw++) {
                float w = sampleAngular(wMin, wMax, iw, cfg.wSamples);

                // ── Simulate trajectory ──────────────────────
                float score;
                float minClear;
                bool trajOk = scoreTrajectory(
                    v, w, poseX, poseY, poseTheta,
                    goalX, goalY, path, pathLen,
                    mapper, score, minClear
                );

                if (!trajOk) {
                    result.collisionCount++;
                    continue;
                }

                result.okCount++;
                if (score < bestScore) {
                    bestScore = score;
                    result.v = v;
                    result.w = w;
                    result.ok = true;
                    result.score = score;
                    result.clearance = minClear;
                }
            }
        }

        return result;
    }

private:
    // Pointer to static costmap buffer (set by buildCostmap)
    uint8_t* _costmapPtr = nullptr;

    // ── Helpers ──────────────────────────────────────────────

    static inline float normalizeAngle(float a) {
        while (a > M_PI)  a -= 2.0f * M_PI;
        while (a < -M_PI) a += 2.0f * M_PI;
        return a;
    }

    static float sampleLinear(float minV, float maxV, int idx, int total) {
        if (total <= 1) return maxV;
        float t = (float)idx / (float)(total - 1);
        return minV + (maxV - minV) * t;
    }

    static float sampleAngular(float minW, float maxW, int idx, int total) {
        if (total <= 1) return 0;
        float t = (float)idx / (float)(total - 1);
        return minW + (maxW - minW) * t;
    }

    // ── Pick local goal (carrot on path) ─────────────────────
    void pickLocalGoal(float px, float py,
                       const Waypoint* plan, int planLen,
                       float& outX, float& outY) const
    {
        // Find closest point on path
        int closestIdx = 0;
        float closestDist = 1e9f;
        for (int i = 0; i < planLen; i++) {
            float d = hypotf(plan[i].x - px, plan[i].y - py);
            if (d < closestDist) {
                closestDist = d;
                closestIdx = i;
            }
        }
        // Look ahead from closest
        for (int i = closestIdx; i < planLen; i++) {
            float d = hypotf(plan[i].x - px, plan[i].y - py);
            if (d >= cfg.headingLookahead) {
                outX = plan[i].x;
                outY = plan[i].y;
                return;
            }
        }
        // Fallback: final goal
        outX = plan[planLen - 1].x;
        outY = plan[planLen - 1].y;
    }

    // ── Distance from point to path segment ──────────────────
    static float distanceToPath(float x, float y,
                                const Waypoint* plan, int planLen)
    {
        if (planLen <= 1) {
            if (planLen == 1) return hypotf(plan[0].x - x, plan[0].y - y);
            return 0;
        }
        float minDist = 1e9f;
        for (int i = 0; i < planLen - 1; i++) {
            float dx = plan[i+1].x - plan[i].x;
            float dy = plan[i+1].y - plan[i].y;
            float l2 = dx*dx + dy*dy;
            if (l2 < 1e-6f) {
                minDist = fminf(minDist, hypotf(plan[i].x - x, plan[i].y - y));
                continue;
            }
            float t = ((x - plan[i].x) * dx + (y - plan[i].y) * dy) / l2;
            t = fmaxf(0, fminf(1, t));
            float projX = plan[i].x + t * dx;
            float projY = plan[i].y + t * dy;
            minDist = fminf(minDist, hypotf(projX - x, projY - y));
        }
        return minDist;
    }

    // ── O(1) cost lookup from pre-computed LOCAL costmap ───────
    // Translates global grid coords → local window coords.
    uint8_t getCostmapCost(int gx, int gy) const {
        int lx = gx - _winOriginX;
        int ly = gy - _winOriginY;
        if (lx < 0 || lx >= DWA_LOCAL_SIZE || ly < 0 || ly >= DWA_LOCAL_SIZE) return 254;
        if (!_costmapPtr) return 0; // Costmap not built yet
        return _costmapPtr[ly * DWA_LOCAL_SIZE + lx];
    }

    // ── Check collision at position (circular footprint + costmap) ─────
    bool checkCollisionAt(float x, float y, float theta,
                          const OccupancyGridMapper& mapper) const
    {
        // Check center + 8-connected points at robotRadius (9 total)
        float r = cfg.robotRadius;
        float d = r * 0.707f; // diagonal offset (r / sqrt(2))
        float offsets[][2] = {
            {0, 0},
            { r, 0}, {-r, 0},
            {0,  r}, {0, -r},
            { d,  d}, { d, -d},
            {-d,  d}, {-d, -d}
        };
        float cosT = cosf(theta);
        float sinT = sinf(theta);
        for (auto& off : offsets) {
            float wx = x + off[0] * cosT - off[1] * sinT;
            float wy = y + off[0] * sinT + off[1] * cosT;
            int gx = mapper.world_to_grid_x(wx);
            int gy = mapper.world_to_grid_y(wy);
            // O(1) lookup from pre-computed costmap
            if (getCostmapCost(gx, gy) >= 253) return true;
        }
        return false;
    }

    // ── Get circular clearance at position (O(1) via costmap) ────
    // Uses pre-computed costmap for fast nearest-obstacle estimation
    // instead of brute-force grid scan. Cost 254=lethal(dist=0), 0=free(dist>=inflate_range)
    float getCircularClearance(float x, float y,
                               const OccupancyGridMapper& mapper) const
    {
        int cx = mapper.world_to_grid_x(x);
        int cy = mapper.world_to_grid_y(y);
        uint8_t cost = getCostmapCost(cx, cy);
        
        if (cost >= 254) return 0.0f;       // On top of obstacle
        if (cost >= 253) return 0.01f;      // Inscribed zone
        if (cost == 0)   return cfg.preferredClearance + 0.1f; // Free space
        
        // Inverse exponential: cost = 252 * exp(-COST_SCALING * distM)
        // → distM = -ln(cost/252) / COST_SCALING + inscribed_dist
        float distM = -logf((float)cost / 252.0f) / COST_SCALING;
        distM += (float)INSCRIBED_CELLS * GRID_RESOLUTION;
        return distM;
    }

    // ── Score a single trajectory ────────────────────────────
    bool scoreTrajectory(
        float v, float w,
        float startX, float startY, float startTheta,
        float goalX, float goalY,
        const Waypoint* path, int pathLen,
        const OccupancyGridMapper& mapper,
        float& outScore, float& outMinClearance
    ) {
        float x = startX;
        float y = startY;
        float theta = startTheta;
        int steps = (int)fmaxf(1, cfg.simTime / cfg.simGranularity);
        float minClear = 1e9f;

        for (int i = 0; i < steps; i++) {
            x += v * cosf(theta) * cfg.simGranularity;
            y += v * sinf(theta) * cfg.simGranularity;
            theta = normalizeAngle(theta + w * cfg.simGranularity);

            // Collision check — O(1) via costmap
            if (checkCollisionAt(x, y, theta, mapper)) {
                return false;
            }

            // Clearance check
            float clearance = getCircularClearance(x, y, mapper);
            minClear = fminf(minClear, clearance);
            if (clearance < cfg.stopOnClearance) {
                return false;
            }
        }

        // ── Compute cost ─────────────────────────────────────
        float goalDist = hypotf(goalX - x, goalY - y);
        float pathDist = distanceToPath(x, y, path, pathLen);
        float finalHeading = atan2f(goalY - y, goalX - x);
        float headingErr = fabsf(normalizeAngle(finalHeading - theta));
        float clearancePenalty = fmaxf(0, cfg.preferredClearance - minClear);
        float speedReward = cfg.maxSpeedTrans - v;

        outScore = cfg.goalDistBias    * goalDist
                 + cfg.pathDistBias    * pathDist
                 + cfg.goalHeadingBias * headingErr
                 + cfg.clearanceBias   * clearancePenalty
                 + cfg.speedBias       * speedReward;
        outMinClearance = minClear;
        return true;
    }
};

#endif // DWA_PLANNER_H
