/**
 * AMR 2.0 — SLAM Diagnostics
 * Real-time quality metrics for onboard SLAM pipeline
 * 
 * Metrics:
 *   - matchScore:    Last scan match quality (0..1, higher = better)
 *   - tfNorm:        Accumulated TF correction magnitude (meters)
 *   - odomDriftRate: Estimated drift between odom and map (m/s)
 *   - gridCoverage:  Percentage of grid cells explored (not UNKNOWN)
 *   - frontierCount: Current number of frontier cells
 *   - scanMatchMs:   Time spent in last scan matching (ms)
 */

#ifndef SLAM_DIAGNOSTICS_H
#define SLAM_DIAGNOSTICS_H

#include <cmath>
#include "lidar_mapper.h"

struct SlamDiag {
    float matchScore   = 0.0f;   // 0..1 (1 = perfect match)
    float tfNorm       = 0.0f;   // sqrt(tfDx² + tfDy²)
    float tfAngleDeg   = 0.0f;   // |tfDTheta| in degrees
    float odomDriftRate = 0.0f;  // estimated m/s drift
    int   gridCoverage = 0;      // percentage 0..100
    int   gridOccupied = 0;      // number of occupied cells
    int   gridFree     = 0;      // number of free cells
    int   frontierCount = 0;     // current frontier cells
    float scanMatchMs  = 0.0f;   // last match duration
    int   scanCount    = 0;      // total grid updates performed

    /**
     * Compute grid coverage statistics from the occupancy grid
     */
    void updateGridStats(const OccupancyGridMapper& mapper) {
        int unknown = 0, free = 0, occ = 0;
        int total = GRID_SIZE * GRID_SIZE;

        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                int8_t v = mapper.grid[y][x];
                if (v == 0) unknown++;
                else if (v < 0) free++;
                else occ++;
            }
        }

        gridFree     = free;
        gridOccupied = occ;
        gridCoverage = (int)(100.0f * (float)(free + occ) / (float)total);
        scanCount    = mapper.scanCount;
    }

    /**
     * Update TF norm from current transform values
     */
    void updateTfNorm(float dx, float dy, float dtheta) {
        tfNorm     = sqrtf(dx * dx + dy * dy);
        tfAngleDeg = fabsf(dtheta) * 180.0f / M_PI;
    }

    /**
     * Convert ICP RMS to a 0..1 match score
     * RMS < 0.01m → score ≈ 1.0 (excellent)
     * RMS > 0.1m  → score ≈ 0.0 (poor)
     */
    void updateMatchScore(float icpRms) {
        // Sigmoid-like mapping: score = exp(-rms * 30)
        if (icpRms <= 0.0f) {
            matchScore = 0.0f;
        } else {
            matchScore = expf(-icpRms * 30.0f);
            if (matchScore > 1.0f) matchScore = 1.0f;
        }
    }
};

#endif // SLAM_DIAGNOSTICS_H
