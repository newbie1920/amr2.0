/**
 * FreeRTOS Tasks — Implementation
 * controlTask:     Motor PID + Odometry + Navigator + DWA (Core 1, 50Hz)
 * lidarTask:        LiDAR scan + SLAM pipeline (Core 0)
 * pathfinderTask:   A* path planning on-demand (Core 0)
 * explorationTask:  Frontier-based auto-explore (Core 0)
 */

#include "tasks.h"
#include "config.h"
#include "robot_state.h"
#include "log.h"
#include <esp_task_wdt.h>

// Drivers
#include "motor_driver.h"
#include "encoder_driver.h"
#include "imu_mpu6050.h"
#include "lidar_a1m8.h"

// Perception
#include "odometry.h"
#include "occupancy_grid.h"
#include "icp_matcher.h"

// Navigation
#include "pid_controller.h"
#include "navigator.h"
#include "dwa_planner.h"
#include "pathfinder.h"
#include "pathfinder_types.h"
#include "frontier_explorer.h"
#include "network_comm.h"

// ── PID controllers (allocated once) ─────────────────────────
WheelPID leftPID(KP_VEL, KI_VEL, 0, FF_GAIN_LEFT, 1.0f / CONTROL_FREQ_HZ, 5.0f, (float)MIN_PWM);
WheelPID rightPID(KP_VEL, KI_VEL, 0, FF_GAIN_RIGHT, 1.0f / CONTROL_FREQ_HZ, 5.0f, (float)MIN_PWM);

// ── ICP Scan Matching buffers ────────────────────────────────
static IcpMatcher icpMatcher;
static LidarPoint* icpPrevScan = nullptr;
static LidarPoint* icpCurrentScan = nullptr;  // PSRAM-backed (was static BSS)
static int  icpPrevLen   = 0;
static bool icpFirstScan = true;

// ── Navigation instances ─────────────────────────────────────
Navigator navigator;
DwaPlanner dwaPlanner;
AStarPathfinder pathfinder;
FrontierExplorer explorer;

// ── FreeRTOS Queue for GoTo requests ─────────────────────────
QueueHandle_t pathfinderQueue = NULL;
static unsigned long lastDwaTime = 0;
static constexpr float NAV_STRAIGHT_RATE_HOLD_GAIN = 0.35f;
static constexpr float NAV_STRAIGHT_RATE_HOLD_MAX_W = 0.12f;
static constexpr float NAV_STRAIGHT_RATE_HOLD_W_BAND = 0.12f;
static constexpr float NAV_STRAIGHT_RATE_HOLD_MIN_V = 0.04f;

// ============================================================
//   CONTROL TASK (Core 1, 50Hz)
//   Reads sensors → odometry → PID → motor output
// ============================================================
void controlTask(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_FREQ_HZ);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float deltaT = 1.0f / CONTROL_FREQ_HZ;

    // Subscribe to hardware WDT
    esp_task_wdt_add(NULL);

    for (;;) {
        // ── Odometry (encoder + IMU fusion) ──────────────
        odometry_update(deltaT);

        // ── Safety: timeout brake ────────────────────────
        if (millis() - state.motor.lastCmdTime > CMD_TIMEOUT_MS) {
            state.motor.targetLeftVel = 0;
            state.motor.targetRightVel = 0;
        }

        // ── Brake override ───────────────────────────────
        float targetL = state.motor.targetLeftVel;
        float targetR = state.motor.targetRightVel;
        if (state.motor.brakeEnabled) {
            targetL = 0;
            targetR = 0;
        }

        // ── Cross-coupling sync ──────────────────────────
        float sync = (state.motor.vL_meas - state.motor.vR_meas) - (targetL - targetR);
        if (fabsf(targetL) > 0.01f || fabsf(targetR) > 0.01f) {
            targetL -= 0.5f * sync;
            targetR += 0.5f * sync;
        }

        // ── PID compute ──────────────────────────────────
        float pwmL = leftPID.update(state.motor.vL_meas, targetL);
        float pwmR = rightPID.update(state.motor.vR_meas, targetR);

        // Cross-coupling PWM adjustment
        if (fabsf(targetL) > 0.01f || fabsf(targetR) > 0.01f) {
            pwmL -= 3.0f * sync;
            pwmR += 3.0f * sync;
        }

        pwmL = constrain(pwmL, -255.0f, 255.0f);
        pwmR = constrain(pwmR, -255.0f, 255.0f);
        state.motor.pwmLeft  = pwmL;
        state.motor.pwmRight = pwmR;

        // ── Motor output ─────────────────────────────────
        if (!state.nav.hitlMode) {
            if (state.motor.brakeEnabled) {
                // Active H-bridge brake (IN1=HIGH, IN2=HIGH) — stops immediately
                motor_brake(MOTOR_LEFT);
                motor_brake(MOTOR_RIGHT);
            } else {
                motor_set(MOTOR_LEFT,  pwmL);
                motor_set(MOTOR_RIGHT, pwmR);
            }
        }

        // ── Navigator update (50Hz) ──────────────────────
        if (state.nav.allowOnboardNav && navigator.isNavigating()) {
            PoseSnapshot mapPose = getMapPose();
            navigator.update(mapPose.x, mapPose.y, mapPose.theta);

            // ── DWA obstacle avoidance (10Hz) ────────────
            if (millis() - lastDwaTime > DWA_INTERVAL_MS &&
                navigator.state == NAV_TRACKING &&
                state.nav.streamOccupancyGrid) {

                lastDwaTime = millis();
                dwaPlanner.buildCostmap(gridMapper, mapPose.x, mapPose.y);
                dwaPlanner.clearRobotFootprint(mapPose.x, mapPose.y, gridMapper);

                DwaResult dwa = dwaPlanner.computeVelocity(
                    mapPose.x, mapPose.y, mapPose.theta,
                    navigator.cmdLinear, navigator.cmdAngular,
                    navigator.waypoints + navigator.currentWpIdx,
                    navigator.waypointCount - navigator.currentWpIdx,
                    gridMapper);

                if (dwa.ok) {
                    navigator.cmdLinear  = dwa.v;
                    navigator.cmdAngular = dwa.w;
                }
            }

            // Convert (v, w) → wheel velocities
            float v = navigator.cmdLinear;
            float w = navigator.cmdAngular;
            if (navigator.state == NAV_TRACKING &&
                fabsf(v) > NAV_STRAIGHT_RATE_HOLD_MIN_V &&
                fabsf(w) < NAV_STRAIGHT_RATE_HOLD_W_BAND) {
                const float measuredW = (state.motor.vR_meas - state.motor.vL_meas) *
                                        WHEEL_RADIUS / WHEEL_SEPARATION;
                const float rateHold = constrain((w - measuredW) * NAV_STRAIGHT_RATE_HOLD_GAIN,
                                                 -NAV_STRAIGHT_RATE_HOLD_MAX_W,
                                                 NAV_STRAIGHT_RATE_HOLD_MAX_W);
                w += rateHold;
                navigator.cmdAngular = w;
            }
            state.motor.targetLeftVel  = (v - w * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS;
            state.motor.targetRightVel = (v + w * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS;
            state.motor.lastCmdTime = millis();
        }

        // ── Feed WDT ─────────────────────────────────────
        esp_task_wdt_reset();

        // ── Timing ───────────────────────────────────────
        TickType_t now = xTaskGetTickCount();
        if (now - xLastWakeTime >= xFrequency) {
            vTaskDelay(1);
            xLastWakeTime = xTaskGetTickCount();
        } else {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }
}

// ============================================================
//   LIDAR TASK (Core 0)
//   Reads LiDAR points → grid mapping → ICP scan matching
// ============================================================
void lidarTask(void* pvParameters) {
    state.lidar.running = true;

    // Remove IDLE0 from WDT — lidarTask is high-throughput on Core 0
    // and would starve IDLE0. This is the standard ESP-IDF pattern.
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCPU(0);
    if (idle0 != NULL) esp_task_wdt_delete(idle0);

    // Diagnostic counters
    unsigned long lidarOkCount = 0;
    unsigned long lidarFailCount = 0;
    int consecutiveFails = 0;

    // Allocate ICP scan buffers in PSRAM if available
    if (ESP.getFreePsram() > 4096) {
        icpPrevScan    = (LidarPoint*)ps_malloc(OccupancyGridMapper::MAX_POINTS * sizeof(LidarPoint));
        icpCurrentScan = (LidarPoint*)ps_malloc(OccupancyGridMapper::MAX_POINTS * sizeof(LidarPoint));
    } else {
        icpPrevScan    = (LidarPoint*)malloc(OccupancyGridMapper::MAX_POINTS * sizeof(LidarPoint));
        icpCurrentScan = (LidarPoint*)malloc(OccupancyGridMapper::MAX_POINTS * sizeof(LidarPoint));
    }
    if (icpPrevScan != nullptr && icpCurrentScan != nullptr) {
        LOG_I("SLAM", "ICP buffers allocated (%d bytes each, PSRAM: %s)",
            OccupancyGridMapper::MAX_POINTS * (int)sizeof(LidarPoint),
            psramFound() ? "yes" : "no");
    } else {
        free(icpPrevScan);
        free(icpCurrentScan);
        icpPrevScan = nullptr;
        icpCurrentScan = nullptr;
        LOG_E("SLAM", "ICP scan buffer allocation FAILED; scan matching disabled");
    }
    // Initialize ICP matcher buffers (PSRAM-backed)
    if (icpMatcher.init()) {
        LOG_I("SLAM", "ICP matcher initialized (PSRAM: %s)", psramFound() ? "yes" : "no");
    } else {
        LOG_E("SLAM", "ICP matcher init FAILED — no memory!");
    }

    auto shouldFreezeMapping = []() -> bool {
        const float vRobot = (state.motor.vR_meas + state.motor.vL_meas) * 0.5f * WHEEL_RADIUS;
        const float wRobot = (state.motor.vR_meas - state.motor.vL_meas) * WHEEL_RADIUS / WHEEL_SEPARATION;
        const bool navActive =
            state.nav.allowOnboardNav &&
            (navigator.state == NAV_TRACKING ||
             navigator.state == NAV_FINAL_TURN ||
             navigator.state == NAV_RECOVERY_SPIN ||
             navigator.state == NAV_RECOVERY_BACKUP ||
             navigator.state == NAV_RECOVERY_WAIT);
        const bool recovering =
            navigator.state == NAV_RECOVERY_SPIN ||
            navigator.state == NAV_RECOVERY_BACKUP ||
            navigator.state == NAV_RECOVERY_WAIT;
        const bool reversing = vRobot < -0.02f;
        const bool turningHard = fabsf(wRobot) > 0.45f;
        return navActive || recovering || reversing || turningHard;
    };

    for (;;) {
        if (state.nav.hitlMode) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float angle, distance;
        uint8_t quality;

        if (lidar_read_point(angle, distance, quality)) {
            state.lidar.receiving = true;
            lidarOkCount++;
            consecutiveFails = 0;

            // Yield every 50 reads to keep system responsive
            if (lidarOkCount % 50 == 0) vTaskDelay(1);

            // A1M8 returns quality=0 for valid long-range/low-reflectance points
            // Filter by distance validity instead of quality to maximize point count
            if (distance > 10.0f) {  // >10mm = valid (rejects noise/zero)
                int deg = (int)roundf(angle) % 360;
                // A1M8 is mounted flipped/backwards; keep raw table for 0x04, map in robot frame.
                int mapDeg = (360 - deg + 180) % 360;
                state.lidar.distances[deg] = (uint16_t)distance;

                // Feed occupancy grid
                if (state.nav.streamOccupancyGrid && !shouldFreezeMapping()) {
                    gridMapper.add_point((float)mapDeg, distance / 1000.0f);
                }

                // Close-range obstacle detection (front ±30°)
                if ((mapDeg <= 30 || mapDeg >= 330) && distance > 50 && distance < 80) {
                    state.lidar.obstacleDetected = true;
                    state.lidar.lastObstacleTime = millis();
                }
            }

            // ── Grid update + SLAM pipeline ──────────────
            const bool mappingFrozen = shouldFreezeMapping();
            if (mappingFrozen && gridMapper.point_count > 0) {
                gridMapper.point_count = 0;
            }

            if (state.nav.mode == RobotState::MODE_ONBOARD &&
                state.nav.streamOccupancyGrid &&
                !mappingFrozen &&
                millis() - state.lidar.lastGridUpdateTime > GRID_UPDATE_INTERVAL_MS &&
                gridMapper.point_count > 180) {

                // Copy scan BEFORE update_grid clears buffer
                int icpCurrentLen = min(gridMapper.point_count, OccupancyGridMapper::MAX_POINTS);
                if (icpCurrentScan != nullptr) {
                    memcpy(icpCurrentScan, gridMapper.points, icpCurrentLen * sizeof(LidarPoint));
                }

                // Track odometry delta for ICP init guess
                static float prevOdomX = state.odom.x;
                static float prevOdomY = state.odom.y;
                static float prevOdomTheta = state.odom.theta;

                PoseSnapshot odomSnap = getOdomPose();
                float odomDx = odomSnap.x - prevOdomX;
                float odomDy = odomSnap.y - prevOdomY;
                float odomDTheta = odomSnap.theta - prevOdomTheta;

                // Transform to local frame for ICP init guess
                float cosPrev = cosf(prevOdomTheta);
                float sinPrev = sinf(prevOdomTheta);
                float localDx = odomDx * cosPrev + odomDy * sinPrev;
                float localDy = -odomDx * sinPrev + odomDy * cosPrev;

                // Update grid using MAP-frame pose (critical section — applyTf writes state.map.*)
                portENTER_CRITICAL(&stateMux);
                applyTf();
                float mapX = state.map.x, mapY = state.map.y, mapTh = state.map.theta;
                portEXIT_CRITICAL(&stateMux);
                gridMapper.update_pose(mapX, mapY, mapTh);
                gridMapper.update_grid();
                state.lidar.lastGridUpdateTime = millis();

                // ── ICP scan matching (always runs — no motor guard) ──
                const float odomTravel = sqrtf(odomDx * odomDx + odomDy * odomDy);
                const float odomTurn = fabsf(odomDTheta);

                if (icpPrevScan != nullptr &&
                    icpCurrentScan != nullptr &&
                    !icpFirstScan &&
                    icpPrevLen > 0 &&
                    (odomTravel > 0.015f || odomTurn > 0.015f)) {

                    IcpMatcher::Pose2D initGuess = {localDx, localDy, odomDTheta};
                    IcpMatcher::Pose2D correction;

                    if (icpMatcher.match(
                            icpPrevScan, icpPrevLen,
                            icpCurrentScan, icpCurrentLen,
                            initGuess, correction)) {

                        // Compute RMS FIRST — use as quality gate
                        state.slam.icpRms = icpMatcher.computeRMS(
                            icpPrevScan, icpPrevLen,
                            icpCurrentScan, icpCurrentLen,
                            correction);

                        // Quality gate: only apply correction when ICP converged well
                        // RMS < 0.08m = good match, apply full weight
                        // RMS 0.08-0.15 = mediocre, apply reduced weight
                        // RMS > 0.15 = bad match, skip entirely
                        float qualityWeight = 0.0f;
                        if (state.slam.icpRms > 0 && state.slam.icpRms < 0.08f) {
                            qualityWeight = 0.35f;  // Good match
                        } else if (state.slam.icpRms < 0.15f) {
                            qualityWeight = 0.15f;  // Mediocre — gentle nudge
                        }
                        // else: bad match — skip

                        if (qualityWeight > 0.0f) {
                            // Clamp correction to reject outliers
                            float errX = constrain(correction.x - localDx, -0.06f, 0.06f);
                            float errY = constrain(correction.y - localDy, -0.06f, 0.06f);
                            float errT = constrain(correction.theta - odomDTheta, -0.08f, 0.08f);

                            // Transform to global map frame
                            float prevMapTheta = prevOdomTheta + state.tf.dTheta;
                            float cosMap = cosf(prevMapTheta);
                            float sinMap = sinf(prevMapTheta);
                            float errMapX = errX * cosMap - errY * sinMap;
                            float errMapY = errX * sinMap + errY * cosMap;

                            // DEBUG: Disable SLAM TF correction to diagnose heading spin
                            // updateTf(errMapX, errMapY, errT, qualityWeight);
                        }

                        // Update SLAM diagnostics for browser UI
                        state.slam.matchScore = (state.slam.icpRms > 0) ? 
                            fminf(1.0f, 0.08f / fmaxf(state.slam.icpRms, 0.001f)) : 0.0f;
                        state.slam.tfNorm = sqrtf(state.tf.dx * state.tf.dx + state.tf.dy * state.tf.dy);
                        state.slam.tfAngleDeg = state.tf.dTheta * 180.0f / M_PI;

                        static unsigned long lastIcpLog = 0;
                        if (millis() - lastIcpLog > 2000) {
                            lastIcpLog = millis();
                            LOG_I("ICP", "dx=%.3f dy=%.3f dth=%.3f rms=%.4f qw=%.2f tf=(%.3f,%.3f,%.3f)",
                                correction.x, correction.y, correction.theta,
                                state.slam.icpRms, qualityWeight,
                                state.tf.dx, state.tf.dy, state.tf.dTheta);
                        }
                    }
                }

                // Save scan for next iteration
                if (icpPrevScan != nullptr && icpCurrentScan != nullptr && icpCurrentLen > 0) {
                    memcpy(icpPrevScan, icpCurrentScan, icpCurrentLen * sizeof(LidarPoint));
                    icpPrevLen = icpCurrentLen;
                    icpFirstScan = false;
                }

                // Update prev odom snapshot
                prevOdomX = odomSnap.x;
                prevOdomY = odomSnap.y;
                prevOdomTheta = odomSnap.theta;

                state.slam.scanCount = gridMapper.scanCount;
            }
        } else {
            // LiDAR read failed
            lidarFailCount++;
            consecutiveFails++;
            if (consecutiveFails > 10) state.lidar.receiving = false;

            vTaskDelay(pdMS_TO_TICKS(10));

            // Auto-reset after prolonged failure
            if (consecutiveFails > 50) {
                LOG_W("LIDAR", "Signal lost >500ms — resetting...");
                lidar_reset();
                consecutiveFails = 0;
            }
        }
    }
}

// ============================================================
//   PATHFINDER TASK (Core 0, on-demand)
//   Reads GoToRequest from queue → A* → loads Navigator
// ============================================================
void pathfinderTask(void* pvParameters) {
    // Init A* with SLAM grid dimensions
    pathfinder.init(GRID_SIZE, GRID_SIZE, GRID_RESOLUTION);
    pathfinder.setSlamMap(&gridMapper);
    LOG_I("A*", "Pathfinder initialized (%dx%d, %.2fm/cell)", GRID_SIZE, GRID_SIZE, GRID_RESOLUTION);

    GoToRequest req;
    for (;;) {
        if (xQueueReceive(pathfinderQueue, &req, portMAX_DELAY) == pdTRUE) {
            if (state.nav.mode == RobotState::MODE_PC_BROWSER) {
                LOG_W("A*", "Ignored request in PC_BROWSER mode");
                continue;
            }

            LOG_I("A*", "Computing path: (%.2f,%.2f) → (%.2f,%.2f)",
                  req.startX, req.startY, req.goalX, req.goalY);

            static uint32_t planSeq = 0;
            Waypoint path[MAX_WAYPOINTS];
            PathPlanDebug debug;
            debug.planId = ++planSeq;
            int wpCount = pathfinder.computePath(
                req.startX, req.startY, req.goalX, req.goalY,
                path, MAX_WAYPOINTS, &debug);
            debug.planId = planSeq;

            if (wpCount > 0) {
                navigator.loadPath(path, wpCount, req.finalHeading);
                LOG_I("A*", "Path loaded: %d waypoints", wpCount);
            } else {
                LOG_W("A*", "No path found!");
                if (explorer.isExploring()) {
                    explorer.blacklistCurrentGoal();
                }
            }
            broadcast_nav_path(debug, path, wpCount);
        }
    }
}

// ============================================================
//   EXPLORATION TASK (Core 0, low priority)
//   Frontier-based autonomous exploration
// ============================================================
void explorationTask(void* pvParameters) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));

        if (!state.nav.explorationRequested || !explorer.isExploring()) {
            if (state.nav.explorationRequested && explorer.state == FrontierExplorer::EXPLORE_IDLE) {
                explorer.start();
            }
            continue;
        }

        // Wait for navigator to finish current goal
        if (navigator.isNavigating()) continue;

        if (navigator.state == NAV_ERROR) {
            LOG_W("EXPLORE", "Navigator ERROR — blacklisting goal");
            explorer.blacklistCurrentGoal();
        }

        // Find next frontier goal
        PoseSnapshot pose = getMapPose();
        float gx, gy;
        if (explorer.findNextGoal(gridMapper, pose.x, pose.y, pose.theta, gx, gy)) {
            explorer.state = FrontierExplorer::EXPLORE_NAVIGATING;
            GoToRequest req = {pose.x, pose.y, gx, gy, NAN};
            xQueueSend(pathfinderQueue, &req, pdMS_TO_TICKS(100));
        }
    }
}

// ============================================================
//   TASK CREATION
// ============================================================
void tasks_create() {
    // Create pathfinder queue
    pathfinderQueue = xQueueCreate(4, sizeof(GoToRequest));

    // controlTask on Core 1 (highest priority — motor + nav)
    xTaskCreatePinnedToCore(controlTask, "controlTask", 12288, NULL, 5, NULL, 1);
    LOG_I("TASK", "controlTask created (Core 1, 12KB stack, priority 5)");

    // lidarTask on Core 0 (sensor processing)
    xTaskCreatePinnedToCore(lidarTask, "lidarTask", 8192, NULL, 3, NULL, 0);
    LOG_I("TASK", "lidarTask created (Core 0, 8KB stack, priority 3)");

    // pathfinderTask on Core 0 (on-demand A*)
    xTaskCreatePinnedToCore(pathfinderTask, "pathfinderTask", 16384, NULL, 2, NULL, 0);
    LOG_I("TASK", "pathfinderTask created (Core 0, 16KB stack, priority 2)");

    // explorationTask on Core 0 (low priority frontier scan)
    xTaskCreatePinnedToCore(explorationTask, "explorationTask", 8192, NULL, 1, NULL, 0);
    LOG_I("TASK", "explorationTask created (Core 0, 8KB stack, priority 1)");
}
