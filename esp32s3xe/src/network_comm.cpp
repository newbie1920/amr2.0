#include "network_comm.h"
#include "config.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <ArduinoJson.h>

#include "navigator.h"
#include "odometry.h"
#include "imu_sensor.h"
#include "lidar_mapper.h"
#include "pathfinder.h"
#include "frontier_explorer.h"
#include "slam_diagnostics.h"

extern WebServer server;
extern WebSocketsServer webSocket;
extern WiFiManager wm;

extern Navigator navigator;
extern OccupancyGridMapper gridMapper;
extern AStarPathfinder astar;
extern uint16_t lidarDists[360];
extern bool obstacleDetected;
extern unsigned long timeObstacleLastDetected;
extern bool streamOccupancyGrid;
extern bool allowOnboardNavigation;
extern bool hitlMode;
extern const char* architectureProfile;

extern FrontierExplorer frontierExplorer;
extern bool explorationRequested;

// SLAM diagnostics (defined in main.cpp)
extern SlamDiag slamDiag;
extern float icpRmsLast;

// Pathfinder task inter-process communication (defined in main.cpp)
struct GoToRequest {
    float startX, startY;
    float goalX,  goalY;
    float finalHeading;
};
extern QueueHandle_t pathfinderQueue;


unsigned long lastTelemetryTime = 0;

void setArchitectureProfile(const char* profile) {
  if (strcmp(profile, "pc_slam") == 0) {
    architectureProfile = "pc_slam";
    streamOccupancyGrid = false;
    allowOnboardNavigation = false;
    gridMapper.reset();
    navigator.abort();
    Serial.println("[ARCH] Switched to PC_SLAM profile");
    return;
  }

  if (strcmp(profile, "aggressive") == 0) {
    architectureProfile = "aggressive";
    streamOccupancyGrid = true;
    allowOnboardNavigation = true;
    Serial.println("[ARCH] Switched to AGGRESSIVE profile");
    return;
  }

  architectureProfile = "hybrid";
  streamOccupancyGrid = true;
  allowOnboardNavigation = true;
  Serial.println("[ARCH] Switched to HYBRID profile");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  // Raw binary handlers for custom protocols
  if (type == WStype_BIN && length > 0) {
    if (payload[0] == 0x03) {
      // ── MAP_DATA binary frame (Chunked) ───────────────────────────────
      // Layout: [0x03][width:uint16_LE][height:uint16_LE][res:float_LE][offset:uint32_LE][data...]
      // Header = 1 (type) + 2 + 2 + 4 + 4 = 13 bytes minimum
      // ────────────────────────────────────────────────────────────────
      const size_t HEADER_SIZE = 13; // 1 type + 2 w + 2 h + 4 res + 4 offset
      if (length < HEADER_SIZE) {
        Serial.printf("[NET] MAP_DATA quá ngắn: %d bytes\n", length);
        return;
      }

      uint16_t newW, newH;
      float    newRes;
      uint32_t offset;
      memcpy(&newW,  payload + 1, 2);
      memcpy(&newH,  payload + 3, 2);
      memcpy(&newRes, payload + 5, 4);
      memcpy(&offset, payload + 9, 4);

      // Reinit A* if dimensions changed (or first time)
      if (!astar.isInitialized() ||
          newW != (uint16_t)astar.getMapWidth()  ||
          newH != (uint16_t)astar.getMapHeight() ||
          fabsf(newRes - astar.getMapResolution()) > 1e-4f) {
        Serial.printf("[NET] MAP reinit: %dx%d @ %.3fm/cell\n", newW, newH, newRes);
        astar.init(newW, newH, newRes);
      }

      const uint8_t* mapPayload = payload + HEADER_SIZE;
      size_t         payloadLen = length - HEADER_SIZE;
      astar.updateStaticMap(mapPayload, payloadLen, offset);

      // Send map_ack so Web knows ESP32 received the map chunk
      // Only send ack when the full map is received to prevent spam, or send every chunk
      // Web expects one map_ack to know it's done.
      if (offset + payloadLen >= (uint32_t)(newW * newH)) {
        JsonDocument ack;
        ack["type"]  = "map_ack";
        ack["w"]     = newW;
        ack["h"]     = newH;
        ack["bytes"] = (int)(offset + payloadLen);
        static uint8_t ackBuf[64];
        size_t ackLen = serializeMsgPack(ack, ackBuf, sizeof(ackBuf));
        webSocket.sendBIN(num, ackBuf, ackLen);
        Serial.printf("[NET] Static Map complete: %dx%d (%.3f m/cell, %d bytes total)\n",
                      newW, newH, newRes, offset + payloadLen);
      }
      return;
    }
  }

  // Dual-mode: Accept both JSON (TEXT) and MessagePack (BIN)
  JsonDocument doc;
  if (type == WStype_TEXT) {
    deserializeJson(doc, payload, length);
  } else if (type == WStype_BIN) {
    deserializeMsgPack(doc, payload, length);
  } else {
    return;
  }

  if (doc["type"] == "ping") {
    JsonDocument pong;
    pong["type"] = "pong";
    pong["ts"] = doc["ts"];
    static uint8_t outBuf[64];
    size_t len = serializeMsgPack(pong, outBuf, sizeof(outBuf));
    webSocket.sendBIN(num, outBuf, len);
    return;
  }

  if (doc["cmd"] == "reset_odom") {
    // FIX Bug #12: Accept optional coordinates, default to (0, 0, 0)
    float resetX = doc["x"] | 0.0f;
    float resetY = doc["y"] | 0.0f;
    float resetTheta = doc["theta"] | 0.0f;
    robotX = resetX; robotY = resetY; robotTheta = resetTheta; robotDistance = 0;
    leftTicks = rightTicks = lastTicksL = lastTicksR = 0;
    targetLeftVel = targetRightVel = 0;
    gyroTheta = encoderTheta = fusedTheta = robotTheta;
    // Reset TF and recenter grid on new position
    tfDx = tfDy = tfDTheta = 0;
    applyTf();
    gridMapper.centerOnPosition(robotX, robotY);
    leftPID->reset();
    rightPID->reset();
    Serial.printf("[CMD] Odometry reset to (%.2f, %.2f, %.2f).\n", resetX, resetY, resetTheta);
  }

  if (doc["cmd"] == "set_pose") {
    if (!doc["x"].isNull()) robotX = doc["x"];
    if (!doc["y"].isNull()) robotY = doc["y"];
    if (!doc["theta"].isNull()) robotTheta = doc["theta"];
    gyroTheta = encoderTheta = fusedTheta = robotTheta;
  }

  if (doc["cmd"] == "hitl_mode") {
    hitlMode = doc["enable"] | false;
    Serial.printf("[HITL] Mode set to: %d\n", hitlMode);
    if (hitlMode) {
      // Reset mapping for fresh SLAM in virtual environment
      gridMapper.reset();
      streamOccupancyGrid = true;
      Serial.println("[HITL] GridMapper reset, grid streaming ON");
    }
  }

  if (doc["cmd"] == "hitl_sensor") {
    if (hitlMode) {
      if (!doc["x"].isNull()) robotX = doc["x"];
      if (!doc["y"].isNull()) robotY = doc["y"];
      if (!doc["theta"].isNull()) robotTheta = doc["theta"];
      gyroTheta = encoderTheta = fusedTheta = robotTheta;
      
      JsonArray lidarArr = doc["lidar"].as<JsonArray>();
      memset(lidarDists, 0, sizeof(lidarDists));
      obstacleDetected = false;
      
      if (streamOccupancyGrid) {
        gridMapper.update_pose(robotX, robotY, robotTheta);
      }

      for (JsonVariant v : lidarArr) {
        int a = v["a"];
        int d = v["d"];
        if (a >= 0 && a < 360) {
          lidarDists[a] = d;
          // Only trigger obstacle for forward-facing beams (±30°), matching real lidarTask logic
          if ((a <= 30 || a >= 330) && d > 50 && d < 150) {
            obstacleDetected = true;
            timeObstacleLastDetected = millis();
          }
          if (streamOccupancyGrid && d > 0) {
            gridMapper.add_point(a, d / 1000.0f);
          }
        }
      }

      if (streamOccupancyGrid) {
        gridMapper.update_grid();
      }
    }
  }

  if (doc["cmd"] == "navigate") {
    if (!allowOnboardNavigation) {
      Serial.println("[ARCH] Ignored onboard navigate in PC_SLAM profile");
      return;
    }
    JsonArray pathArr = doc["path"].as<JsonArray>();
    int count = pathArr.size();
    if (count > 0 && count <= MAX_WAYPOINTS) {
      // FIX Bug #3: Dùng loadPath() thay vì gán thủ công
      // loadPath() sẽ reset recovery counters, progress check, ref position
      Waypoint tempWps[MAX_WAYPOINTS];
      for (int i = 0; i < count; i++) {
        tempWps[i].x = pathArr[i]["x"];
        tempWps[i].y = pathArr[i]["y"];
        tempWps[i].heading = NAN;
        tempWps[i].useReverse = false;
      }
      float endH = NAN;
      if (!doc["finalHeading"].isNull()) {
        endH = doc["finalHeading"].as<float>() * PI / 180.0f;
      }
      navigator.loadPath(tempWps, count, endH);
      
      JsonDocument ack;
      ack["type"] = "nav_ack";
      ack["wp_count"] = count;
      ack["finalH"] = isnan(endH) ? -1 : (int)(endH * 180.0f / PI);
      static uint8_t outBuf[64];
      size_t ackLen = serializeMsgPack(ack, outBuf, sizeof(outBuf));
      webSocket.sendBIN(num, outBuf, ackLen);
    }
  }

  // ── KIẾN TRÚC PHÂN TÁN (Decentralized Architecture) ──
  
  if (doc["cmd"] == "goto") {
    if (!allowOnboardNavigation) {
      Serial.println("[ARCH] Forcing hybrid mode for GOTO command");
      setArchitectureProfile("hybrid");
    }

    GoToRequest req;
    req.startX      = robotX;
    req.startY      = robotY;
    req.goalX       = doc["x"];
    req.goalY       = doc["y"];
    req.finalHeading = doc["finalHeading"].isNull() ? NAN : doc["finalHeading"].as<float>() * PI / 180.0f;

    Serial.printf("[NET] Queuing GOTO: (%.2f,%.2f) -> (%.2f,%.2f)\n",
                  req.startX, req.startY, req.goalX, req.goalY);

    if (pathfinderQueue) {
      // Overwrite old request in queue (xQueueOverwrite only works for depth-1 queues)
      // For depth-2 we just try to send; if full the oldest is naturally consumed first
      xQueueSend(pathfinderQueue, &req, 0);
    }
  }

  if (doc["cmd"] == "traffic") {
    // Nhận thông tin các xe khác từ Web để update DWA / A*
    astar.clearDynamicObstacles();
    JsonArray robots = doc["robots"].as<JsonArray>();
    for (JsonObject r : robots) {
      float rx = r["x"];
      float ry = r["y"];
      float radius = r["r"] | 0.3f; // Default 30cm collision radius
      astar.setDynamicObstacle(rx, ry, radius);
    }
  }
  
  if (doc["cmd"] == "nav_stop") {
    if (allowOnboardNavigation) {
      navigator.abort();
    }
  }

  if (doc["cmd"] == "map_request_ack") {
    // Web xác nhận sẽ push map ngay — không cần xử lý gì thêm
    Serial.println("[NET] Web acknowledged MAP_REQUEST — map incoming");
  }


  if (doc["cmd"] == "pause") {
    if (allowOnboardNavigation) {
      navigator.pause();
    }
  }

  if (doc["cmd"] == "resume") {
    if (allowOnboardNavigation) {
      navigator.resume();
    }
  }

  if (doc["cmd"] == "set_arch_mode") {
    const char* profile = doc["profile"] | "hybrid";
    setArchitectureProfile(profile);
  }

  // ── Set Robot Pose (for HITL sync) ───────────────────────
  if (doc["cmd"] == "set_pose") {
    float newX = doc["x"] | 0.0f;
    float newY = doc["y"] | 0.0f;
    float newTheta = doc["theta"] | 0.0f;
    robotX = newX;
    robotY = newY;
    robotTheta = newTheta;
    gyroTheta = encoderTheta = fusedTheta = robotTheta;
    robotDistance = 0;
    leftTicks = rightTicks = lastTicksL = lastTicksR = 0;
    // Reset TF and recenter grid on new position
    tfDx = tfDy = tfDTheta = 0;
    applyTf();
    gridMapper.centerOnPosition(robotX, robotY);
    gridMapper.update_pose(mapX, mapY, mapTheta);
    Serial.printf("[NET] set_pose: (%.2f, %.2f, %.1f°)\n", robotX, robotY, robotTheta * 180.0f / M_PI);
  }

  // ── Frontier Exploration Commands ─────────────────────────
  if (doc["cmd"] == "explore") {
    // Force hybrid mode — exploration REQUIRES onboard navigation
    if (!allowOnboardNavigation) {
      setArchitectureProfile("hybrid");
      Serial.println("[NET] explore cmd forced HYBRID profile");
    }
    
    explorationRequested = true;
    
    // Reset map for fresh SLAM
    gridMapper.reset();
    gridMapper.centerOnPosition(robotX, robotY);  // Center grid around current robot pose
    streamOccupancyGrid = true;
    
    // Reset TF transform for fresh SLAM session
    tfDx = tfDy = tfDTheta = 0;
    applyTf();
    
    // Reset PID controllers (odometry position is set via set_pose from Web)
    targetLeftVel = targetRightVel = 0;
    leftPID->reset();
    rightPID->reset();
    navigator.abort(); // Cancel any previous navigation
    frontierExplorer.stop(); // Reset frontier state machine
    
    Serial.printf("[NET] Exploration started! pos=(%.2f,%.2f) grid_origin=(%.2f,%.2f) hitl=%d\n", 
                  robotX, robotY, gridMapper.originX, gridMapper.originY, hitlMode);

    JsonDocument ack;
    ack["type"] = "explore_ack";
    ack["status"] = "started";
    static uint8_t outBuf[64];
    size_t ackLen = serializeMsgPack(ack, outBuf, sizeof(outBuf));
    webSocket.sendBIN(num, outBuf, ackLen);
  }

  if (doc["cmd"] == "explore_stop") {
    explorationRequested = false;
    frontierExplorer.stop();
    navigator.abort();
    Serial.println("[NET] Exploration stopped by Web");
  }

  if (doc["cmd"] == "recal_gyro") {
    gyroCalibrated = false;
    gyroCalSamples = 0;
    gyroCalSum = 0;
    gyroZBias = 0;
  }

  if (doc["cmd"] == "brake") {
    brakeEnabled = doc["val"];
  }

  if (!doc["linear"].isNull() && !navigator.isNavigating()) {
    if (!brakeEnabled) {
      float v = doc["linear"];
      float w = doc["angular"];
      targetLeftVel = constrain((v - w * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
      targetRightVel = constrain((v + w * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
      lastCmdTime = millis();
    }
  }
}

void init_network() {
  Serial.println("[BOOT] Khoi tao WiFi...");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  wm.setConnectTimeout(15);
  wm.setConfigPortalTimeout(120);
  
  Serial.println("[BOOT] Chay WiFiManager autoConnect...");
  if (!wm.autoConnect(WIFI_AP_NAME)) {
    Serial.println("[WIFI] Ket noi that bai hoac Timeout Portal!");
  }
  Serial.printf("[BOOT] WiFi autoConnect xong, vong lap Loop bat dau! IP: %s\n", WiFi.localIP().toString().c_str());
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  ArduinoOTA.setHostname("AMR2_S3");
  ArduinoOTA.onStart([]() { Serial.println("\n[OTA] Start..."); });
  ArduinoOTA.onEnd([]() { Serial.println("\n[OTA] Done!"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n[OTA] Error[%u]\n", error);
  });
  ArduinoOTA.begin();

  TelnetStream.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  server.begin();
}

void update_network() {
  webSocket.loop();
  server.handleClient();
  ArduinoOTA.handle();

  static unsigned long lastWifiCheck = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    Serial.println("[WIFI] Mat ket noi! Dang thu reconnect...");
    WiFi.reconnect();
  }
}

void send_occupancy_grid() {
  // Buffer: 1(type) + 1(w) + 1(h) + 4(res) + 4(rx) + 4(ry) + 4(rh) + 4(origX) + 4(origY) + GRID_SIZE² = ~16411 bytes
  static uint8_t gridBuffer[20000];
  int idx = 0;
  
  // Message type (1 = occupancy grid)
  gridBuffer[idx++] = 0x01;
  
  // Grid dimensions (uint8_t OK — GRID_SIZE=128 fits in 0-255)
  gridBuffer[idx++] = GRID_SIZE;
  gridBuffer[idx++] = GRID_SIZE;
  
  // Resolution
  float gridRes = GRID_RESOLUTION;
  memcpy(&gridBuffer[idx], &gridRes, 4);
  idx += 4;
  
  // Robot pose (map frame)
  float rx = robotX, ry = robotY, rh = robotTheta;
  memcpy(&gridBuffer[idx], &rx, 4);
  idx += 4;
  memcpy(&gridBuffer[idx], &ry, 4);
  idx += 4;
  memcpy(&gridBuffer[idx], &rh, 4);
  idx += 4;
  
  // Grid origin (world coordinates) — new in SLAM v2
  float ox = gridMapper.originX, oy = gridMapper.originY;
  memcpy(&gridBuffer[idx], &ox, 4);
  idx += 4;
  memcpy(&gridBuffer[idx], &oy, 4);
  idx += 4;
  
  // Occupancy grid
  int payloadLen = 0;
  gridMapper.serialize_grid(&gridBuffer[idx], payloadLen);
  idx += payloadLen;
  
  // Broadcast
  webSocket.broadcastBIN(gridBuffer, idx);
}

void broadcast_telemetry() {
  if (millis() - lastTelemetryTime > TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();

    // Read Power Monitor
    read_ina3221();

    long b_sum = 0;
    for (int i = 0; i < 20; i++) b_sum += analogRead(BATT_PIN);
    
    // N?u có INA3221, dùng đi?n áp kênh 1 thay cho ADC ?o
    float v_now = 0;
    if (inaAvailable && ina_busV[INA_CH_BATT] > 1.0f) {
        v_now = ina_busV[INA_CH_BATT];
    } else {
        v_now = (b_sum / 20.0f / 4095.0f) * 3.3f * BATT_SCALE_FACTOR + BATT_OFFSET;
    }

    filteredVBatt = filteredVBatt * 0.9f + v_now * 0.1f;
    if (filteredVBatt < 1.0f) filteredVBatt = v_now;
    int battPct = constrain((int)((filteredVBatt - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V) * 100), 0, 100);

    float v_robot = (vR_meas + vL_meas) / 2.0f * WHEEL_RADIUS;
    float w_fused; 
    if (imuAvailable && gyroCalibrated) w_fused = gyroZ_raw;
    else w_fused = (vR_meas - vL_meas) * WHEEL_RADIUS / WHEEL_SEPARATION;

    JsonDocument telem;
    telem["telem"] = true;
    telem["vx"] = v_robot;
    telem["wz"] = w_fused;
    telem["theta"] = robotTheta;
    telem["h"] = robotTheta * 180.0f / PI;
    telem["d"] = robotDistance;
    telem["x"] = robotX;
    telem["y"] = robotY;
    telem["imu"] = imuAvailable;
    telem["imu_cal"] = gyroCalibrated;
    telem["gyroZ"] = gyroZ_raw;
    telem["fTheta"] = fusedTheta * 180.0f / PI;

    JsonObject enc = telem["enc"].to<JsonObject>();
    enc["l"] = leftTicks;
    enc["r"] = rightTicks;

    telem["vL_t"] = targetLeftVel;
    telem["vR_t"] = targetRightVel;
    telem["vL_r"] = vL_meas;
    telem["vR_r"] = vR_meas;
    telem["pwmL"] = (int)lastPwmLeft;
    telem["pwmR"] = (int)lastPwmRight;
    telem["batt"] = battPct;

    telem["nav"] = navigator.getStateName();
    telem["nav_wp"] = navigator.currentWpIdx;
    telem["nav_total"] = navigator.waypointCount;
    telem["nav_rec"] = navigator.recoveryAttempts;
    telem["nav_recovering"] = navigator.isRecovering();
    telem["arch"] = architectureProfile;
    telem["grid_stream"] = streamOccupancyGrid;
    telem["onboard_nav"] = allowOnboardNavigation;
    telem["hitl"] = hitlMode;
    telem["explore"] = frontierExplorer.getStateName();
    telem["explore_goals"] = frontierExplorer.exploredGoals;
    telem["explore_frontiers"] = frontierExplorer.frontierCellCount;

    // ── SLAM v2 Diagnostics ──────────────────────────────────
    JsonObject slam = telem["slam"].to<JsonObject>();
    slam["score"]    = (int)(slamDiag.matchScore * 100);  // 0..100%
    slam["tfNorm"]   = slamDiag.tfNorm;                   // meters
    slam["tfDeg"]    = slamDiag.tfAngleDeg;               // degrees
    slam["coverage"] = slamDiag.gridCoverage;             // 0..100%
    slam["occ"]      = slamDiag.gridOccupied;             // cell count
    slam["free"]     = slamDiag.gridFree;                 // cell count
    slam["rms"]      = icpRmsLast;                        // raw ICP RMS
    slam["scans"]    = slamDiag.scanCount;                // total grid updates
    // Map-frame pose (for dashboard to display corrected position)
    slam["mX"]  = mapX;
    slam["mY"]  = mapY;
    slam["mTh"] = mapTheta * 180.0f / PI;

    telem["eX"] = navigator.error_x;
    telem["eY"] = navigator.error_y;
    telem["eYaw"] = navigator.error_yaw;

    if (inaAvailable) {
        JsonArray inaV = telem["inaV"].to<JsonArray>();
        JsonArray inaA = telem["inaA"].to<JsonArray>();
        for (int i = 0; i < 3; i++) {
            inaV.add(ina_busV[i]);
            inaA.add(ina_currentA[i]);
        }

        JsonObject pwr = telem["power"].to<JsonObject>();
        pwr["battV"] = ina_busV[INA_CH_BATT];
        pwr["battA"] = ina_currentA[INA_CH_BATT];
        pwr["motorV"] = ina_busV[INA_CH_MOTOR];
        pwr["motorA"] = ina_currentA[INA_CH_MOTOR];
    }

    // PATH export cho Web Dashboard vẽ
    extern Waypoint sharedPath[];
    extern int sharedPathLen;
    extern SemaphoreHandle_t pathMutex;
    if (sharedPathLen > 0 && pathMutex) {
        if (xSemaphoreTake(pathMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            JsonArray pArr = telem["path"].to<JsonArray>();
            for(int i = 0; i < sharedPathLen; i++) {
                JsonObject pt = pArr.add<JsonObject>();
                pt["x"] = sharedPath[i].x;
                pt["y"] = sharedPath[i].y;
            }
            xSemaphoreGive(pathMutex);
        }
    }

    JsonArray ls = telem["lidar"].to<JsonArray>();
    bool hasObstruction = obstacleDetected && millis() - timeObstacleLastDetected < 500;
    telem["obs"] = hasObstruction;
    for (int i = 0; i < 360; i += 3) { // Mỗi 3° → ~120 points max, tiết kiệm buffer
      if (lidarDists[i] > 0 && lidarDists[i] < 3000) {
        JsonObject p = ls.add<JsonObject>();
        p["a"] = i;             
        p["d"] = lidarDists[i]; 
      }
    }

    static uint8_t telemBuf[8192]; // Tăng buffer cho lidar data + path
    telemBuf[0] = 0x02; 
    size_t reqLen = measureMsgPack(telem);
    size_t len = 0;
    if (reqLen <= sizeof(telemBuf) - 1) {
        len = serializeMsgPack(telem, &telemBuf[1], sizeof(telemBuf) - 1);
    } else {
        Serial.printf("[TELEM] WARNING: MsgPack overflow! reqLen=%d, max=%d\n", reqLen, sizeof(telemBuf) - 1);
    }
    if (len > 0) webSocket.broadcastBIN(telemBuf, len + 1);
    
    static unsigned long lastGridSendTime = 0;
    if (streamOccupancyGrid && millis() - lastGridSendTime > 200) {
      lastGridSendTime = millis();
      send_occupancy_grid();
    }
  }
}
