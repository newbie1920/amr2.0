/**
 * AMR 2.0 — DWA Local Planner (ESP32-S3)
 * Ported from dwaPlanner.js (Web Dashboard)
 * 
 * Dynamic Window Approach with:
 *   - Velocity sampling (vSamples × wSamples)
 *   - Trajectory simulation with collision detection
 *   - Circular footprint clearance scoring
 *   - Path alignment + goal heading + speed scoring
 *   - Rotate-in-place for large heading errors
 *   - Predictive braking near obstacles
 * 
 * All memory is statically allocated — zero malloc in real-time loop.
 * Designed to run at 10-20Hz inside controlTask (Core 1).
 */

#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <Arduino.h>
#include <cmath>
#include "config.h"
#include "navigator.h"       // Waypoint, MAX_WAYPOINTS
#include "lidar_mapper.h"    // OccupancyGridMapper, GRID_SIZE, GRID_RESOLUTION

// ============================================================
//   DWA CONFIGURATION
// ============================================================

struct DwaConfig {
    float maxSpeedTrans   = 0.15f;   // m/s (match NAV_MAX_LINEAR_VEL)
    float minSpeedTrans   = 0.0f;
    float maxSpeedRot     = 1.5f;    // rad/s
    float maxAccelTrans   = 0.8f;    // m/s²
    float maxAccelRot     = 2.5f;    // rad/s²
    float simTime         = 2.0f;    // Tăng từ 1.5s lên 2.0s để dự đoán xa hơn
    float simGranularity  = 0.15f;   // seconds per step
    int   vSamples        = 9;       // linear velocity samples (Tăng từ 7 lên 9)
    int   wSamples        = 21;      // angular velocity samples (Tăng từ 15 lên 21 để cua mượt hơn)
    float robotRadius     = 0.08f;   // meters (giảm xuống 8cm để robot tự tin qua khe hẹp)
    float preferredClearance = 0.25f; // meters (Giảm từ 0.3m xuống 0.25m để không né quá lố)
    float stopOnClearance = 0.04f;   // meters (Dừng nếu quá gần 4cm)
    float headingLookahead = 1.5f;   // meters — local goal pick distance
    float pathDistBias    = 2.0f;    // Tăng từ 1.0 lên 2.0 để robot bám sát đường A* hơn
    float goalDistBias    = 12.0f;   // Cost weight: distance to local goal
    float goalHeadingBias = 12.0f;   // Cost weight: heading alignment to goal
    float clearanceBias   = 15.0f;   // Giảm từ 25.0 xuống 15.0 để tránh bị "sợ" tường
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
//   DWA PLANNER CLASS
// ============================================================

class DwaPlanner {
public:
    DwaConfig cfg;

    DwaPlanner() {}

    /**
     * Compute the best (v, w) velocity command using DWA.
     * 
     * @param poseX, poseY, poseTheta  Current robot pose
     * @param curV, curW               Current velocity
     * @param path                     Global plan waypoints
     * @param pathLen                  Number of waypoints
     * @param mapper                   Reference to OccupancyGridMapper
     * @return DwaResult               Best velocity + diagnostics
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

    // ── Check collision at position (circular footprint) ─────
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
            if (mapper.in_bounds(gx, gy)) {
                if (mapper.grid[gy][gx] > 30) return true; // Occupied
            }
        }
        return false;
    }

    // ── Get circular clearance at position ───────────────────
    float getCircularClearance(float x, float y,
                               const OccupancyGridMapper& mapper) const
    {
        int cx = mapper.world_to_grid_x(x);
        int cy = mapper.world_to_grid_y(y);
        int scanR = (int)(cfg.preferredClearance / GRID_RESOLUTION) + 1;
        float best = cfg.preferredClearance + 0.2f;

        for (int dy = -scanR; dy <= scanR; dy++) {
            for (int dx = -scanR; dx <= scanR; dx++) {
                int gx = cx + dx;
                int gy = cy + dy;
                if (!mapper.in_bounds(gx, gy)) continue;
                if (mapper.grid[gy][gx] > 30) { // Occupied cell
                    float cellDist = hypotf(dx, dy) * GRID_RESOLUTION;
                    best = fminf(best, cellDist);
                    // Early exit: if already below stop threshold, no point searching further
                    if (best < cfg.stopOnClearance) return best;
                }
            }
        }
        return best;
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

            // Collision check
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
