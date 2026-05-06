#include "sim_task.h"

#if SIMULATION_MODE

#include <Arduino.h>
#include "sim_engine.h"
#include "lidar_mapper.h"
#include "odometry.h"
#include "csm_matcher.h"

// Externs from main.cpp
extern float robotX, robotY, robotTheta;
extern float targetLeftVel, targetRightVel;
extern OccupancyGridMapper gridMapper;
extern uint16_t lidarDists[360];
extern bool obstacleDetected;
extern unsigned long timeObstacleLastDetected;
extern bool hitlMode;

// TF externs for SLAM correction
extern float tfDx, tfDy, tfDTheta;
extern float mapX, mapY, mapTheta;

// CSM matcher extern
extern CsmMatcher csmMatcher;
extern bool csmInitialized;

// Mutex to protect shared pose state between simTask (Core 0) and controlTask (Core 1)
SemaphoreHandle_t simPoseMutex = nullptr;

static SimEngine* engine = nullptr;

void simTask(void *pvParameters) {
    // Ensure HITL mode is on (redundant safety — also set in setup())
    hitlMode = true;

    engine = new SimEngine();
    engine->start();

    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / PHYSICS_HZ);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.printf("[SimTask] Started at %dHz, %d segments\n",
                  PHYSICS_HZ, (int)engine->getWorld()->getSegments().size());

    // Tracking for CSM scan matching inside sim
    int simScanCount = 0;

    for (;;) {
        // Read target velocities set by controlTask (Navigator/DWA)
        float v_target = (targetRightVel + targetLeftVel) / 2.0f * WHEEL_RADIUS;
        float w_target = (targetRightVel - targetLeftVel) * WHEEL_RADIUS / WHEEL_SEPARATION;

        engine->setVelocity(v_target, w_target);
        engine->step();

        SimTelemetry t = engine->getTelemetry();

        // ── BUG #1 FIX: Inject ODOM (noisy) instead of ground-truth ──────
        // SimEngine produces:
        //   t.pose  = ground-truth (perfect, no drift)
        //   t.odom  = noisy odometry (with Gaussian noise)
        // Previously we injected t.pose → TF was always 0, scan matching never tested.
        // Now inject t.odom → TF accumulates correction via CSM → real SLAM pipeline.
        if (xSemaphoreTake(simPoseMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            robotX = t.odom.x;
            robotY = t.odom.y;
            robotTheta = t.odom.theta;
            xSemaphoreGive(simPoseMutex);
        }

        // Recompute map-frame pose (mapX, mapY, mapTheta)
        // Navigator uses mapX/mapY, not robotX/robotY
        applyTf();

        // ── Inject Lidar scan (every N steps = ~10Hz) ────────────
        if (t.stepCount % LIDAR_EVERY_N_STEPS == 0) {
            // Update gridMapper pose using MAP-FRAME (odom + TF correction)
            // This is critical: grid cells must be painted at the CORRECTED pose
            gridMapper.update_pose(mapX, mapY, mapTheta);

            // Clear lidarDists and obstacle flag for fresh scan
            memset(lidarDists, 0, sizeof(lidarDists));
            obstacleDetected = false;
            
            for (const auto& pt : t.lidar) {
                if (pt.quality) {
                    int deg = (int)std::round(pt.angle) % 360;
                    lidarDists[deg] = (uint16_t)(pt.distance * 1000.0f); // m → mm
                    gridMapper.add_point(pt.angle, pt.distance);
                }
            }

            // Copy scan buffer BEFORE update_grid clears it (for CSM)
            static LidarPoint csmScanBuf[360];
            int csmScanLen = gridMapper.point_count;
            if (csmScanLen > 360) csmScanLen = 360;
            memcpy(csmScanBuf, gridMapper.points, csmScanLen * sizeof(LidarPoint));

            // Flush pending points into the occupancy grid
            gridMapper.update_grid();
            simScanCount++;

            // ── CSM Scan Matching: correct odom drift via TF ──────────
            // Runs every lidar cycle after grid has enough data
            if (csmInitialized && simScanCount > 10 && csmScanLen > 30 &&
                (fabsf(targetLeftVel) > 0.01f || fabsf(targetRightVel) > 0.01f)) {
                
                CsmResult csmResult;
                if (csmMatcher.matchScan(
                        gridMapper, mapX, mapY, mapTheta,
                        csmScanBuf, csmScanLen,
                        csmResult)) {
                    
                    // Apply CSM correction to TF transform
                    const float CSM_WEIGHT = 0.5f;
                    updateTf(csmResult.dx, csmResult.dy, csmResult.dTheta, CSM_WEIGHT);

                    static unsigned long lastCsmLog = 0;
                    if (millis() - lastCsmLog > 3000) {
                        lastCsmLog = millis();
                        Serial.printf("[SIM-CSM] dx=%.3f dy=%.3f dth=%.3f score=%.2f tf=(%.3f,%.3f,%.3f)\n",
                            csmResult.dx, csmResult.dy, csmResult.dTheta,
                            csmResult.score, tfDx, tfDy, tfDTheta);
                    }
                }
            }

            // Set obstacle flag if front zone has close obstacle
            if (t.obs) {
                obstacleDetected = true;
                timeObstacleLastDetected = millis();
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void initSimTask() {
    // Create mutex BEFORE launching task
    simPoseMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        simTask,
        "SimTask",
        16384,   // Stack size (increased for CSM)
        NULL,
        1,       // Priority (low — controlTask has priority 10)
        NULL,
        0        // Core 0 (same as lidarTask it replaces)
    );
}

#endif
