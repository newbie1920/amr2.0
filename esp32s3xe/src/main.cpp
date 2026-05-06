// ============================================================
//   AMR 2.0 — ESP32-S3 Firmware
//   WebSocket + PID Motor Control + IMU Fusion
//   Modular Refactored Architecture
// ============================================================

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <RPLidar.h>
#include <Adafruit_NeoPixel.h>
#include <TelnetStream.h>
#include <esp_task_wdt.h>  // Hardware Watchdog Timer

#include "config.h"
#include "navigator.h"
#include "wheel_pid.h"
#include "lidar_mapper.h"
#include "pathfinder.h"
#include "dwa_planner.h"
#include "icp_matcher.h"
#include "frontier_explorer.h"
#include "slam_diagnostics.h"
#include "sim_task.h"

// Included Modules
#include "imu_sensor.h"
#include "odometry.h"
#include "display_oled.h"
#include "network_comm.h"

Adafruit_NeoPixel rgbLed(1, RGB_BUILTIN_PIN, NEO_GRB + NEO_KHZ800);
SemaphoreHandle_t i2cMutex;

// ─── GLOBAL STATE ────────────────────────────────────────────
Navigator navigator;
AStarPathfinder astar;
DwaPlanner dwaPlanner;

// DWA integration: only run DWA every N control ticks (10Hz vs 50Hz control)
static unsigned long lastDwaRunMs = 0;
const unsigned long DWA_INTERVAL_MS = 100; // 10Hz DWA rate
static bool dwaActive = false;
static float dwaV = 0, dwaW = 0;

// ─── PATHFINDER FREERTOS (decoupled from network ISR) ────────
struct GoToRequest {
    float startX, startY;
    float goalX,  goalY;
    float finalHeading;  // NAN = don't care
};
QueueHandle_t      pathfinderQueue = nullptr; // Main-task sends GOTO here
SemaphoreHandle_t  pathMutex       = nullptr; // Guards sharedPath r/w
Waypoint           sharedPath[MAX_WAYPOINTS];
int                sharedPathLen   = 0;

// ── Lidar A1M8 ──────────────────────────────────────────────
HardwareSerial lidarSerial(1);
RPLidar lidar;
OccupancyGridMapper gridMapper;  // LIDAR-based occupancy grid
uint16_t lidarDists[360] = {0}; // Lưu khoảng cách (mm) theo từng độ
bool lidarRunning = false;
bool obstacleDetected = false;
unsigned long timeObstacleLastDetected = 0;
unsigned long lastGridUpdateTime = 0;
static const unsigned long GRID_UPDATE_INTERVAL = 200; // Cập nhật grid mỗi 200ms
bool streamOccupancyGrid = true;
bool allowOnboardNavigation = true;
bool hitlMode = false; // Hardware-in-the-Loop simulation mode

// ── ICP Scan Matching ────────────────────────────────────────
IcpMatcher icpMatcher;
// Prev-scan buffer: allocated in PSRAM during setup() if available, else SRAM
static LidarPoint* icpPrevScan = nullptr;
static int  icpPrevLen   = 0;
static bool icpFirstScan = true;
float icpRmsLast  = 0.0f;   // Dùng để log chất lượng
const char* architectureProfile = "hybrid";

// ── CSM (Correlative Scan Matching) ──────────────────────────
#include "csm_matcher.h"
CsmMatcher csmMatcher;
bool csmInitialized = false;
// CSM activates after ICP has built enough grid data (scanCount > CSM_MIN_SCANS)
static const int CSM_MIN_SCANS = 10;

// ── SLAM Diagnostics ─────────────────────────────────────────
SlamDiag slamDiag;

// ── Frontier Exploration ─────────────────────────────────────
FrontierExplorer frontierExplorer;
bool explorationRequested = false;  // Set true khi nhận cmd "explore"
static unsigned long lastExploreCheckMs = 0;
const unsigned long EXPLORE_CHECK_INTERVAL = 3000;  // Quét frontier mỗi 3 giây

// Network Globals
WebServer server(HTTP_PORT);
WebSocketsServer webSocket(WEBSOCKET_PORT);
WiFiManager wm;

// ============================================================
//   PATHFINDER FREERTOS TASK (Core 0 — non-blocking A*)
//   Receives GoToRequest via queue, runs A*, loads path to Navigator.
//   Runs on Core 0 alongside lidarTask so it never blocks Core 1 motor loop.
// ============================================================
void pathfinderTask(void *pvParameters) {
    GoToRequest req;
    for (;;) {
        // Block forever until a GOTO request arrives in the queue
        if (xQueueReceive(pathfinderQueue, &req, portMAX_DELAY) == pdTRUE) {
            if (!allowOnboardNavigation) continue;

            Serial.printf("[PFDR] Computing path (%.2f,%.2f) -> (%.2f,%.2f)\n",
                          req.startX, req.startY, req.goalX, req.goalY);

            Waypoint tmpPath[MAX_WAYPOINTS];
            int count = astar.computePath(req.startX, req.startY,
                                          req.goalX,  req.goalY,
                                          tmpPath, MAX_WAYPOINTS);

            if (count > 0) {
                // Write to sharedPath under mutex, then load into Navigator
                if (xSemaphoreTake(pathMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    sharedPathLen = count;
                    memcpy(sharedPath, tmpPath, count * sizeof(Waypoint));
                    xSemaphoreGive(pathMutex);
                }
                navigator.loadPath(tmpPath, count, req.finalHeading);
                Serial.printf("[PFDR] Path loaded: %d WPs\n", count);
            } else {
                Serial.println("[PFDR] No path found — goal blocked or out of map.");
            }
        }
    }
}

// ============================================================
//   FRONTIER EXPLORATION TASK (Core 0)
//   Chạy song song, quét frontier mỗi 3s khi explorationRequested.
//   Khi navigator.state == DONE/ERROR → tìm frontier mới.
// ============================================================
static unsigned long exploreNavStartTime = 0;
void explorationTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(5000));  // Chờ hệ thống ổn định 5s

    for (;;) {
        // Chỉ active khi được yêu cầu explore
        if (!explorationRequested || !allowOnboardNavigation) {
            frontierExplorer.state = FrontierExplorer::EXPLORE_IDLE;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Chờ đủ interval
        if (millis() - lastExploreCheckMs < EXPLORE_CHECK_INTERVAL) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        lastExploreCheckMs = millis();

        // State machine
        switch (frontierExplorer.state) {
            case FrontierExplorer::EXPLORE_IDLE:
                frontierExplorer.start();
                break;

            case FrontierExplorer::EXPLORE_SCANNING: {
                float gx, gy;
                applyTf(); // Ensure map pose is current
                if (frontierExplorer.findNextGoal(gridMapper, mapX, mapY, gx, gy)) {
                    GoToRequest req;
                    req.startX = mapX;
                    req.startY = mapY;
                    req.goalX  = gx;
                    req.goalY  = gy;
                    req.finalHeading = NAN;

                    if (pathfinderQueue) {
                        xQueueSend(pathfinderQueue, &req, 0);
                        frontierExplorer.state = FrontierExplorer::EXPLORE_NAVIGATING;
                        exploreNavStartTime = millis();
                        Serial.printf("[EXPLORE] Sent GOTO (%.2f,%.2f) to pathfinder\n", gx, gy);
                    }
                } else {
                    explorationRequested = false;
                    Serial.println("[EXPLORE] === MAP EXPLORATION COMPLETE ===");
                }
                break;
            }

            case FrontierExplorer::EXPLORE_NAVIGATING: {
                if (navigator.state == NAV_DONE) {
                    Serial.println("[EXPLORE] Goal reached — scanning for next frontier");
                    frontierExplorer.state = FrontierExplorer::EXPLORE_SCANNING;
                } else if (navigator.state == NAV_ERROR || 
                          (navigator.state == NAV_IDLE && (millis() - exploreNavStartTime > 3000))) {
                    Serial.println("[EXPLORE] Nav failed or timed out — blacklisting goal");
                    frontierExplorer.blacklistCurrentGoal();
                    frontierExplorer.state = FrontierExplorer::EXPLORE_SCANNING;
                }
                break;
            }

            case FrontierExplorer::EXPLORE_COMPLETE:
            case FrontierExplorer::EXPLORE_FAILED:
                explorationRequested = false;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ============================================================
//   FREERTOS CONTROL TASK (50Hz)
// ============================================================
void controlTask(void *pvParameters) {

  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_FREQ_HZ);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float deltaT = 1.0f / CONTROL_FREQ_HZ;

  // Subscribe controlTask to hardware watchdog (5 second timeout)
  esp_task_wdt_add(NULL);

  for (;;) {
    // ── IMU Read ────────────────────────────────────────
    if (imuAvailable && !hitlMode) {
      gyroZ_raw = mpu6050_readGyroZ();
      if (!gyroCalibrated) {
        mpu6050_calibrate(gyroZ_raw);
        gyroZ_raw = 0;
      } else {
        gyroZ_raw -= gyroZBias;
        // Zero-velocity clamping
        if (fabs(targetLeftVel) < 0.01f && fabs(targetRightVel) < 0.01f && fabs(gyroZ_raw) < 0.01f)
          gyroZ_raw = 0;
        gyroTheta += gyroZ_raw * deltaT;
        gyroTheta = atan2(sin(gyroTheta), cos(gyroTheta));
      }
    }

    // ── Read Encoders ───────────────────────────────────
    if (!hitlMode) {
      noInterrupts();
      long cL = leftTicks;
      long cR = rightTicks;
      interrupts();

      float vL_raw = (float)(cL - lastTicksL) / TICKS_PER_REV * 2.0f * PI / deltaT;
      float vR_raw = (float)(cR - lastTicksR) / TICKS_PER_REV * 2.0f * PI / deltaT;
      vL_meas = 0.7f * vL_meas + 0.3f * vL_raw; // Low-pass filter
      vR_meas = 0.7f * vR_meas + 0.3f * vR_raw;
      lastTicksL = cL;
      lastTicksR = cR;

      // ── Kinematics ──────────────────────────────────────
      float v_robot = (vR_meas + vL_meas) / 2.0f * WHEEL_RADIUS;
      float w_encoder = (vR_meas - vL_meas) * WHEEL_RADIUS / WHEEL_SEPARATION;

      encoderTheta += w_encoder * deltaT;
      encoderTheta = atan2(sin(encoderTheta), cos(encoderTheta));

      // ── Sensor Fusion (Complementary Filter) ────────────
      float w_fused;
      if (imuAvailable && gyroCalibrated) {
        float diff = gyroTheta - encoderTheta;
        while (diff > PI) diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;
        fusedTheta = encoderTheta + COMP_FILTER_ALPHA * diff;
        fusedTheta = atan2(sin(fusedTheta), cos(fusedTheta));
        encoderTheta = fusedTheta;
        w_fused = gyroZ_raw;
        robotTheta = fusedTheta;
      } else {
        fusedTheta = encoderTheta;
        w_fused = w_encoder;
        robotTheta = encoderTheta;
      }

      // Odometry update
      float dist = v_robot * deltaT;
      robotDistance += fabs(dist);
      robotX += dist * cos(robotTheta);
      robotY += dist * sin(robotTheta);
      
      // Recompute map-frame pose after every odom update
      applyTf();
    } else {
      // ── HITL MODE: SimTask handles pose (robotX/Y/Theta) ────
      // We only set measured velocities for PID feedback + DWA
      vL_meas = targetLeftVel;
      vR_meas = targetRightVel;

#if SIMULATION_MODE
      // SimTask is the sole writer of robotX/Y/Theta (with physics + collision).
      // Do NOT compute kinematics here — it would race with simTask.
      // Just sync encoder/gyro/fused to match simTask's injected theta.
      encoderTheta = gyroTheta = fusedTheta = robotTheta;
      applyTf();
#else
      // Legacy HITL mode (Web-based sim): controlTask computes perfect odom
      float v_sim = (vR_meas + vL_meas) / 2.0f * WHEEL_RADIUS;
      float w_sim = (vR_meas - vL_meas) * WHEEL_RADIUS / WHEEL_SEPARATION;

      robotTheta += w_sim * deltaT;
      robotTheta = atan2f(sinf(robotTheta), cosf(robotTheta));

      float dist_sim = v_sim * deltaT;
      robotDistance += fabsf(dist_sim);
      robotX += dist_sim * cosf(robotTheta);
      robotY += dist_sim * sinf(robotTheta);

      encoderTheta = gyroTheta = fusedTheta = robotTheta;
      applyTf();
#endif
    }

    float v_robot = (vR_meas + vL_meas) / 2.0f * WHEEL_RADIUS;
    float w_fused = (vR_meas - vL_meas) * WHEEL_RADIUS / WHEEL_SEPARATION;

    // ── OBSTACLE HANDLING + DWA LOCAL PLANNER ─────────────────
    // Thay vì E-STOP cứng, DWA sẽ tự tìm quỹ đạo né vật cản.
    // E-STOP chỉ kích hoạt khi vật cản RẤT GẦN (< 0.15m) hoặc DWA thất bại.
    bool hardEstop = false;
    if (obstacleDetected && millis() - timeObstacleLastDetected < 500) {
        hardEstop = true;
    } else {
        obstacleDetected = false;
    }

    // ── AUTONOMOUS NAVIGATOR + DWA ──────────────────────────
    if (navigator.isNavigating()) {
      navigator.update(mapX, mapY, mapTheta);

      if (navigator.isRecovering()) {
        // Trong lúc recovery (lùi/quay), KHÔNG áp dụng DWA.
        targetLeftVel = constrain((navigator.cmdLinear - navigator.cmdAngular * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
        targetRightVel = constrain((navigator.cmdLinear + navigator.cmdAngular * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
        
        // Chỉ cấm tiến thẳng khi bị E-STOP
        if (hardEstop && navigator.cmdLinear > 0) {
           targetLeftVel = 0; 
           targetRightVel = 0;
        }
      } else {
        // 1. DWA override: chạy DWA mỗi 100ms khi đang TRACKING
        if (navigator.state == NAV_TRACKING && millis() - lastDwaRunMs >= DWA_INTERVAL_MS) {
          lastDwaRunMs = millis();
          float curV = v_robot;
          float curW = w_fused;
          // BUG #4 FIX: Pre-compute costmap ONCE before DWA sampling (~5ms vs ~180ms)
          dwaPlanner.buildCostmap(gridMapper);
          DwaResult dwaResult = dwaPlanner.computeVelocity(
            mapX, mapY, mapTheta, curV, curW,
            navigator.waypoints + navigator.currentWpIdx,
            navigator.waypointCount - navigator.currentWpIdx,
            gridMapper
          );

          if (dwaResult.ok) {
            dwaActive = true;
            dwaV = dwaResult.v;
            dwaW = dwaResult.w;
          } else {
            // DWA không tìm được quỹ đạo an toàn
            dwaActive = false;
          }
        }

        // 2. Chọn velocity
        float finalV = 0, finalW = 0;
        if (dwaActive && navigator.state == NAV_TRACKING) {
          finalV = dwaV;
          finalW = dwaW;
        } else {
          // Nếu DWA thất bại, dừng robot để kích hoạt progress check (stuck detection) của navigator
          finalV = 0;
          finalW = 0;
          dwaActive = false;
        }

        targetLeftVel = constrain((finalV - finalW * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
        targetRightVel = constrain((finalV + finalW * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);

        if (hardEstop && finalV > 0) {
          targetLeftVel = 0;
          targetRightVel = 0;
        }
      }
    } else {
      // Chế độ thủ công: dừng nếu bị E-STOP và đang định chạy tới
      float manualV = (targetLeftVel + targetRightVel) / 2.0f * WHEEL_RADIUS;
      if (hardEstop && manualV > 0) {
          targetLeftVel = 0;
          targetRightVel = 0;
      }
    }

    // ── Motor PI + Feedforward ──────────────────────────
    if (brakeEnabled) {
      targetLeftVel = 0;
      targetRightVel = 0;
    }
    float targetL = targetLeftVel;
    float targetR = targetRightVel;
    
    // Cross-coupling sync
    float sync = (vL_meas - vR_meas) - (targetL - targetR);
    if (fabs(targetL) > 0.01f || fabs(targetR) > 0.01f) {
        targetL -= 0.5f * sync;
        targetR += 0.5f * sync;
    }

    float pwmLeft = leftPID->update(vL_meas, targetL);
    float pwmRight = rightPID->update(vR_meas, targetR);

    // Cross-coupling pwm additional adjustment
    if (fabs(targetL) > 0.01f || fabs(targetR) > 0.01f) {
        pwmLeft -= 3.0f * sync; 
        pwmRight += 3.0f * sync;
    }

    pwmLeft = constrain(pwmLeft, -255.0f, 255.0f);
    pwmRight = constrain(pwmRight, -255.0f, 255.0f);
    lastPwmLeft = pwmLeft;
    lastPwmRight = pwmRight;

    if (INVERT_LEFT_MOTOR) pwmLeft = -pwmLeft;
    if (INVERT_RIGHT_MOTOR) pwmRight = -pwmRight;

    if (!hitlMode) {
      setMotor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, 0, pwmLeft);
      setMotor(MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, 1, pwmRight);
    }

    // Feed watchdog — proves controlTask is alive
    esp_task_wdt_reset();

    TickType_t now = xTaskGetTickCount();
    if (now - xLastWakeTime >= xFrequency) {
      vTaskDelay(1); // Yield to prevent starvation of lower priority tasks if loop overruns
      xLastWakeTime = xTaskGetTickCount();
    } else {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}

// ============================================================
//   LIDAR FREERTOS TASK
// ============================================================
void lidarTask(void *pvParameters) {
  lidarRunning = true;
  
  // Diagnostic counters
  unsigned long lidarOkCount = 0;
  unsigned long lidarFailCount = 0;
  unsigned long lidarQualityOkCount = 0;
  unsigned long lastDiagTime = 0;
  int nonZeroDistCount = 0;
  static int consecutiveFails = 0;
  
  for (;;) {
    if (hitlMode) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (IS_OK(lidar.waitPoint())) {
      lidarOkCount++;
      consecutiveFails = 0; // Reset fail counter when we successfully read a point
      float distance = lidar.getCurrentPoint().distance; // distance value in mm
      float angle    = lidar.getCurrentPoint().angle;    // angle value in degrees
      uint8_t quality = lidar.getCurrentPoint().quality; // quality of the current measurement

      if (quality > 0) {
        lidarQualityOkCount++;
        int deg = (int)round(angle) % 360;
        lidarDists[deg] = (uint16_t)distance;
        
        if (streamOccupancyGrid) {
          gridMapper.add_point(angle, distance / 1000.0f);
        }

        if ((deg <= 30 || deg >= 330) && distance > 50 && distance < 80) {
            obstacleDetected = true;
            timeObstacleLastDetected = millis();
        }
      }
      
      if (streamOccupancyGrid &&
          millis() - lastGridUpdateTime > GRID_UPDATE_INTERVAL &&
          gridMapper.point_count > 180) {

        // ── SLAM v2: Copy scan buffer BEFORE update_grid clears it ──
        // This fixes the race condition where ICP was comparing against an empty buffer
        static LidarPoint icpCurrentScan[360];
        int icpCurrentLen = gridMapper.point_count;
        memcpy(icpCurrentScan, gridMapper.points, icpCurrentLen * sizeof(LidarPoint));
        
        // Track odometry delta between scans (for ICP init guess)
        static float prevOdomX = robotX, prevOdomY = robotY, prevOdomTheta = robotTheta;
        float odomDx = robotX - prevOdomX;
        float odomDy = robotY - prevOdomY;
        float odomDTheta = robotTheta - prevOdomTheta;
        
        // Transform global displacement to prevOdom's local frame for ICP init guess
        float cosPrev = cosf(prevOdomTheta);
        float sinPrev = sinf(prevOdomTheta);
        float localOdomDx = odomDx * cosPrev + odomDy * sinPrev;
        float localOdomDy = -odomDx * sinPrev + odomDy * cosPrev;

        // Update grid using MAP-FRAME pose (odom + TF correction)
        applyTf();
        gridMapper.update_pose(mapX, mapY, mapTheta);
        gridMapper.update_grid();  // This clears gridMapper.points!
        lastGridUpdateTime = millis();

        // ── ICP Pose Correction → updates TF, NOT odom ──────────────
        if (!hitlMode &&
            icpPrevScan != nullptr &&
            !icpFirstScan &&
            icpPrevLen > 0 &&
            (fabsf(targetLeftVel) > 0.01f || fabsf(targetRightVel) > 0.01f)) {

            // Use odometry delta as initial guess (helps ICP converge faster)
            IcpMatcher::Pose2D initGuess = {localOdomDx, localOdomDy, odomDTheta};
            IcpMatcher::Pose2D correction;

            if (icpMatcher.match(
                    icpPrevScan, icpPrevLen,
                    icpCurrentScan, icpCurrentLen,
                    initGuess, correction)) {

                // Calculate ICP error in local frame (Difference between matched and odom)
                float errLocalX = correction.x - localOdomDx;
                float errLocalY = correction.y - localOdomDy;
                float errTheta  = correction.theta - odomDTheta;

                // Clamp error — tránh jump lớn do nhiễu
                errLocalX = constrain(errLocalX, -0.05f, 0.05f);
                errLocalY = constrain(errLocalY, -0.05f, 0.05f);
                errTheta  = constrain(errTheta,  -0.08f, 0.08f);

                // Transform error to global map frame
                float prevMapTheta = prevOdomTheta + tfDTheta;
                float cosMap = cosf(prevMapTheta);
                float sinMap = sinf(prevMapTheta);
                float errMapX = errLocalX * cosMap - errLocalY * sinMap;
                float errMapY = errLocalX * sinMap + errLocalY * cosMap;

                // SLAM v2: Update TF map→odom transform (NOT raw odometry!)
                const float ICP_WEIGHT = 0.4f;
                updateTf(errMapX, errMapY, errTheta, ICP_WEIGHT);

                // Log RMS chất lượng
                icpRmsLast = icpMatcher.computeRMS(
                    icpPrevScan, icpPrevLen,
                    icpCurrentScan, icpCurrentLen,
                    correction);

                static unsigned long lastIcpLog = 0;
                if (millis() - lastIcpLog > 2000) {
                    lastIcpLog = millis();
                    Serial.printf("[ICP] dx=%.3f dy=%.3f dth=%.3f rms=%.4f tf=(%.3f,%.3f,%.3f)\n",
                        correction.x, correction.y, correction.theta, icpRmsLast,
                        tfDx, tfDy, tfDTheta);
                }
            }
        }

        // Save current scan as prev for next iteration
        if (icpPrevScan != nullptr && icpCurrentLen > 0) {
            memcpy(icpPrevScan, icpCurrentScan, icpCurrentLen * sizeof(LidarPoint));
            icpPrevLen   = icpCurrentLen;
            icpFirstScan = false;
        }
        
        // Update prev odom for next delta calculation
        prevOdomX = robotX;
        prevOdomY = robotY;
        prevOdomTheta = robotTheta;

        // ── CSM: Correlative Scan Matching (activates after grid has data) ──
        // CSM uses the occupancy grid directly (likelihood field) → more robust
        // than ICP point-to-point for structured environments.
        // Strategy: ICP runs always (fast, incremental). CSM runs additionally
        // when grid is mature, providing higher-quality corrections.
        if (csmInitialized && 
            !hitlMode &&
            gridMapper.scanCount > CSM_MIN_SCANS &&
            icpCurrentLen > 30 &&
            (fabsf(targetLeftVel) > 0.01f || fabsf(targetRightVel) > 0.01f)) {
            
            CsmResult csmResult;
            if (csmMatcher.matchScan(
                    gridMapper, mapX, mapY, mapTheta,
                    icpCurrentScan, icpCurrentLen,
                    csmResult)) {
                
                // CSM correction → accumulate into TF
                const float CSM_WEIGHT = 0.3f;  // Slightly lower than ICP to avoid overcorrection
                updateTf(csmResult.dx, csmResult.dy, csmResult.dTheta, CSM_WEIGHT);

                static unsigned long lastCsmLog = 0;
                if (millis() - lastCsmLog > 3000) {
                    lastCsmLog = millis();
                    Serial.printf("[CSM] dx=%.3f dy=%.3f dth=%.3f score=%.2f ms=%.0f\n",
                        csmResult.dx, csmResult.dy, csmResult.dTheta,
                        csmResult.score, csmMatcher.lastMatchMs);
                }
            }
            slamDiag.scanMatchMs = csmMatcher.lastMatchMs;
        }

        // ── SLAM Diagnostics update ──────────────────────────────
        slamDiag.updateMatchScore(icpRmsLast);
        slamDiag.updateTfNorm(tfDx, tfDy, tfDTheta);
        slamDiag.frontierCount = frontierExplorer.frontierCellCount;
        // Grid stats: only compute every 5 scans (128x128 iteration is ~3ms)
        if (gridMapper.scanCount % 5 == 0) {
            slamDiag.updateGridStats(gridMapper);
        }

      }
    } else {
      lidarFailCount++;
      consecutiveFails++;
      
      vTaskDelay(pdMS_TO_TICKS(10)); // Tránh spam vòng lặp
      
      // Nếu lỗi quá 50 lần liên tiếp (hơn 500ms không có data), tiến hành reset Lidar
      if (consecutiveFails > 50) {
        Serial.println("[LIDAR] Mat tin hieu qua lau. Reset Lidar...");
        analogWrite(LIDAR_PWM_PIN, 0); 
        vTaskDelay(pdMS_TO_TICKS(500));
        
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            lidar.startScan(); 
            analogWrite(LIDAR_PWM_PIN, 200); // 80% PWM — A1M8 tối ưu
            vTaskDelay(pdMS_TO_TICKS(3000)); // QUAN TRỌNG: Phải chờ 3s để motor đạt tốc độ ổn định trước khi đọc!
        } else {
            // Không tìm thấy thiết bị, thử reset serial và bật lại motor để thử lại ở chu kỳ sau
            Serial.println("\n[DEBUG] --- BẮT ĐẦU BÀI TEST RAW UART (TRONG VÒNG LẶP) ---");
            Serial.println("[DEBUG] Gửi lệnh GET_INFO (0xA5 0x50) thủ công...");
            uint8_t get_info_cmd[] = {0xA5, 0x50};
            lidarSerial.write(get_info_cmd, 2);
            vTaskDelay(pdMS_TO_TICKS(200)); // Chờ Lidar phản hồi
            
            int bytes_avail = lidarSerial.available();
            Serial.printf("[DEBUG] Số byte Lidar trả về: %d\n", bytes_avail);
            
            if (bytes_avail > 0) {
                Serial.print("[DEBUG] Dữ liệu (HEX): ");
                while(lidarSerial.available()) {
                    Serial.printf("%02X ", lidarSerial.read());
                }
                Serial.println("\n[DEBUG] KẾT LUẬN: Dây RX Tốt! Lỗi do Baudrate hoặc nhiễu.");
            } else {
                Serial.println("[DEBUG] KẾT LUẬN: KHÔNG CÓ TÍN HIỆU ĐIỆN VỀ ESP32!");
                Serial.println("  1. Dây RX/TX đang cắm lỏng hoặc cắm sai.");
                Serial.println("  2. Lidar bị treo mạch logic.");
            }
            Serial.println("------------------------------------------------------\n");

            lidar.begin(lidarSerial);
            analogWrite(LIDAR_PWM_PIN, 200);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        consecutiveFails = 0;
      }
    }
    
    // === DIAGNOSTIC: In ra mỗi 3 giây ===
    if (millis() - lastDiagTime > 3000) {
      lastDiagTime = millis();
      // Đếm số khoảng cách khác 0
      nonZeroDistCount = 0;
      for (int i = 0; i < 360; i++) {
        if (lidarDists[i] > 0) nonZeroDistCount++;
      }
      // Serial.printf("[LIDAR] ok=%lu fail=%lu qualOk=%lu | dists_nonzero=%d/360 | sample[0]=%d [90]=%d [180]=%d [270]=%d\n",
      //   lidarOkCount, lidarFailCount, lidarQualityOkCount,
      //   nonZeroDistCount,
      //   lidarDists[0], lidarDists[90], lidarDists[180], lidarDists[270]);
      lidarOkCount = lidarFailCount = lidarQualityOkCount = 0;
    }
  }
}

// ============================================================
//   SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10); // Wait for USB CDC (max 3s)
  delay(300);
  Serial.println("\n\n[BOOT] Bat dau setup()...");

  init_motors();
  
  // Init Pathfinder to match SLAM grid dimensions
  astar.init(GRID_SIZE, GRID_SIZE, GRID_RESOLUTION);
  astar.setSlamMap(&gridMapper);
  
#if SIMULATION_MODE
  // Force HITL mode ON so controlTask skips encoder/IMU/motor
  hitlMode = true;
  streamOccupancyGrid = true;
  Serial.println("[SIM] SIMULATION_MODE active — hitlMode forced ON");
#endif

  // Center SLAM grid on robot's spawn position
  gridMapper.centerOnPosition(robotX, robotY);
  applyTf(); // Initialize map pose = odom pose (tfDx/Dy/DTheta = 0)

  // Initialize Onboard RGB LED (dim and soft color)
  rgbLed.begin();
  rgbLed.setBrightness(15);
  rgbLed.setPixelColor(0, rgbLed.Color(30, 80, 150));
  rgbLed.show();

  // Initialize Lidar — THỨ TỰ QUAN TRỌNG:
  // 1) Mở serial  2) Bind library  3) Bật motor  4) Chờ motor ổn định  5) startScan
  pinMode(LIDAR_PWM_PIN, OUTPUT);
  lidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(50); // chờ serial ổn định
  lidar.begin(lidarSerial);
  
  // Force stop any ongoing scan from previous boot and flush buffer
  lidar.stop();
  delay(100);
  while(lidarSerial.available()) lidarSerial.read();
  
  // Bật motor TRƯỚC — RPLidar A1M8 cần motor quay ổn định trước khi nhận lệnh scan
  analogWrite(LIDAR_PWM_PIN, 200);
  Serial.println("[LIDAR] Motor ON (PWM=200). Cho motor quay 2s...");
  delay(2000); // CHỜ 2 GIÂY cho motor đạt tốc độ ổn định
  
  // Kiểm tra UART thông với Lidar không
  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 1000))) {
    Serial.printf("[LIDAR] Device OK! Model:%d FW:%d.%d HW:%d\n", 
                  info.model, info.firmware_version >> 8, info.firmware_version & 0xFF, info.hardware_version);
    // Bây giờ mới startScan
    if (IS_OK(lidar.startScan())) {
      Serial.println("[LIDAR] startScan() THANH CONG!");
    } else {
      Serial.println("[LIDAR] startScan() THAT BAI!");
    }
  } else {
    Serial.println("[LIDAR] !!! KHONG TIM THAY THIET BI LIDAR! Kiem tra day noi:");
    Serial.printf("[LIDAR]   RX_PIN=%d (noi vao TX cua Lidar)\n", LIDAR_RX_PIN);
    Serial.printf("[LIDAR]   TX_PIN=%d (noi vao RX cua Lidar)\n", LIDAR_TX_PIN);
    Serial.println("[LIDAR]   Kiem tra: cap 5V, GND, va dau cam UART co chac khong.");
    
    // ==========================================
    // BÀI TEST CHẨN ĐOÁN MẠCH CỨNG (RAW UART) DÀNH CHO OTA
    // ==========================================
    Serial.println("\n[DEBUG] --- BAT DAU BÀI TEST ĐỌC RAW UART TỪ LIDAR ---");
    Serial.println("[DEBUG] Gửi lệnh GET_INFO (0xA5 0x50) thủ công...");
    uint8_t get_info_cmd[] = {0xA5, 0x50};
    lidarSerial.write(get_info_cmd, 2);
    delay(200); // Chờ Lidar phản hồi
    
    int bytes_avail = lidarSerial.available();
    Serial.printf("[DEBUG] So byte Lidar tra ve: %d\n", bytes_avail);
    
    if (bytes_avail > 0) {
        Serial.print("[DEBUG] Du lieu (HEX): ");
        while(lidarSerial.available()) {
            Serial.printf("%02X ", lidarSerial.read());
        }
        Serial.println("\n[DEBUG] KET LUAN: Day RX Tot! Loi nam o Baudrate (115200 vs 256000) hoac nhieu tin hieu.");
    } else {
        Serial.println("[DEBUG] KET LUAN: Khong co bat ky tin hieu dien nao truyen ve ESP32!");
        Serial.println("  1. Day TX cua Lidar bi dut hoac tiep xuc kem.");
        Serial.println("  2. Ban cam sai chan RX tren ESP32.");
        Serial.println("  3. Mach giao tiep logic cua Lidar da hong.");
    }
    Serial.println("------------------------------------------------------\n");
  }

  leftPID = new WheelPID(KP_VEL, KI_VEL, 0.0f, FF_GAIN_LEFT, 1.0f / CONTROL_FREQ_HZ, 5.0f, MIN_PWM);
  rightPID = new WheelPID(KP_VEL, KI_VEL, 0.0f, FF_GAIN_RIGHT, 1.0f / CONTROL_FREQ_HZ, 5.0f, MIN_PWM);

  analogSetPinAttenuation(BATT_PIN, ADC_11db);
  pinMode(BATT_PIN, INPUT);

  init_encoders();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeout(10); // Stream timeout
#if defined(ESP32)
  Wire.setTimeOut(10); // Hardware I2C timeout
#endif

  i2cMutex = xSemaphoreCreateMutex();
  
  init_oled();

  Serial.println("[BOOT] Khoi tao MPU6050...");
  imuAvailable = mpu6050_init();

  Serial.println("[BOOT] Kiem tra INA3221...");
  inaAvailable = ina3221_init();

  init_network();

  // Initialize hardware watchdog: 5 second timeout, auto-reset on hang
  esp_task_wdt_init(5, true);  // 5s timeout, panic on trigger (auto-reset)
  // Subscribe main loop (loop runs on core 1)
  esp_task_wdt_add(NULL);
  Serial.println("[BOOT] Hardware Watchdog Timer: 5s timeout, panic=true");

  // ── ICP Prev-scan buffer: PSRAM nếu có, fallback SRAM ───────
  icpPrevScan = (LidarPoint*)heap_caps_malloc(
      360 * sizeof(LidarPoint),
      psramFound() ? MALLOC_CAP_SPIRAM : MALLOC_CAP_DEFAULT);
  if (icpPrevScan) {
      Serial.printf("[ICP] Prev-scan buffer: %d bytes in %s\n",
          (int)(360 * sizeof(LidarPoint)),
          psramFound() ? "PSRAM" : "SRAM");
  } else {
      Serial.println("[ICP] WARN: heap_caps_malloc failed! ICP disabled.");
  }

  // ── CSM Matcher: likelihood field buffer (GRID_SIZE² = ~16KB) ──
  csmInitialized = csmMatcher.init();
  if (csmInitialized) {
      Serial.printf("[CSM] Likelihood field: %d bytes in %s\n",
          GRID_SIZE * GRID_SIZE,
          psramFound() ? "PSRAM" : "SRAM");
  } else {
      Serial.println("[CSM] WARN: init failed! CSM disabled.");
  }

  // ── FreeRTOS inter-task communication ───────────────────────

  // Queue depth = 2: drop old requests if pathfinder is busy (GOTO from UI)
  pathfinderQueue = xQueueCreate(2, sizeof(GoToRequest));
  pathMutex       = xSemaphoreCreateMutex();
  Serial.println("[BOOT] Pathfinder queue + mutex created.");

  xTaskCreatePinnedToCore(pathfinderTask, "PathfinderTask", 16384, NULL, 5,  NULL, 0);
  xTaskCreatePinnedToCore(controlTask,    "ControlTask",    8192,  NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(explorationTask, "ExploreTask",   16384, NULL, 2,  NULL, 0);

#if SIMULATION_MODE
  // SimTask replaces LidarTask — injects virtual lidar & pose data
  initSimTask();
  Serial.println("[SIM] SimTask launched (replaces LidarTask)");
#else
  xTaskCreatePinnedToCore(lidarTask,      "LidarTask",      16384, NULL, 1,  NULL, 0);
#endif


  Serial.println("================================================");
  Serial.println("  AMR 2.0 FIRMWARE - ESP32-S3 N16R8             ");
  Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("  IMU: %s\n", imuAvailable ? "MPU6050 OK" : "KHONG CO");
  Serial.printf("  INA3221: %s\n", inaAvailable ? "OK" : "KHONG CO");
  Serial.printf("  WebSocket port: %d\n", WEBSOCKET_PORT);
  Serial.println("================================================");
}

// ============================================================
//   MAIN LOOP - Orchestrator
// ============================================================
void loop() {
  // Feed main loop watchdog
  esp_task_wdt_reset();

  update_network();
  broadcast_telemetry();
  update_oled();

  // ── One-time boot banner (prints when monitor is definitely connected) ──
  static bool bootBannerPrinted = false;
  if (!bootBannerPrinted && millis() > 5000) {
    bootBannerPrinted = true;
    Serial.println("\n================================================");
    Serial.println("  AMR 2.0 FIRMWARE - ESP32-S3 N16R8");
    Serial.printf("  IP:        %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  IMU:       %s\n", imuAvailable ? "MPU6050 OK" : "KHONG CO");
    Serial.printf("  INA3221:   %s\n", inaAvailable ? "OK" : "KHONG CO");
    Serial.printf("  Lidar:     %s\n", lidarRunning ? "RUNNING" : "INIT...");
    Serial.printf("  WebSocket: port %d\n", WEBSOCKET_PORT);
    Serial.printf("  Arch:      %s\n", architectureProfile);
    Serial.printf("  Uptime:    %lu ms\n", millis());
    Serial.println("================================================\n");
  }

  // ── Periodic heartbeat (mỗi 10 giây) ──
  static unsigned long lastPrint = 0;
  if(millis() - lastPrint > 10000) {
      lastPrint = millis();
      int battPct = constrain((int)((filteredVBatt - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V) * 100), 0, 100);
      Serial.printf("[LOOP] IP:%s | Batt:%d%% | IMU:%s | WS:%d | Pos:(%.1f,%.1f) h:%.0f\n",
          WiFi.localIP().toString().c_str(),
          battPct,
          (imuAvailable && gyroCalibrated) ? "OK" : (imuAvailable ? "CAL" : "--"),
          webSocket.connectedClients(),
          robotX, robotY, robotTheta * 180.0f / PI);
  }

  if (!navigator.isNavigating() && millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    targetLeftVel = 0;
    targetRightVel = 0;
  }

  if (brakeEnabled) {
    targetLeftVel = 0;
    targetRightVel = 0;
    if (navigator.isNavigating()) {
        navigator.abort();
    }
  }
}
