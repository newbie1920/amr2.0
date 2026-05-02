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
extern const char* architectureProfile;

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

  architectureProfile = "hybrid";
  streamOccupancyGrid = true;
  allowOnboardNavigation = true;
  Serial.println("[ARCH] Switched to HYBRID profile");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  // Raw binary handlers for custom protocols
  if (type == WStype_BIN && length > 0) {
    if (payload[0] == 0x03) {
      // MAP_DATA (Raw occupancy grid)
      astar.updateStaticMap(payload + 1, length - 1);
      Serial.printf("[NET] Nhận Static Map: %d bytes\n", length - 1);
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
    if (!allowOnboardNavigation) return;
    
    float targetX = doc["x"];
    float targetY = doc["y"];
    float endH = doc["finalHeading"].isNull() ? NAN : doc["finalHeading"].as<float>() * PI / 180.0f;
    
    Serial.printf("[NET] Received GOTO: %.2f, %.2f\n", targetX, targetY);
    
    Waypoint tempWps[MAX_WAYPOINTS];
    int count = astar.computePath(robotX, robotY, targetX, targetY, tempWps, MAX_WAYPOINTS);
    
    if (count > 0) {
      navigator.loadPath(tempWps, count, endH);
      Serial.println("[NET] A* Path generated and loaded to Navigator!");
    } else {
      Serial.println("[NET] GOTO Failed: No path found or goal blocked.");
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
  static uint8_t gridBuffer[1620];
  int idx = 0;
  
  // Message type (1 = occupancy grid)
  gridBuffer[idx++] = 0x01;
  
  // Grid dimensions
  gridBuffer[idx++] = GRID_SIZE;
  gridBuffer[idx++] = GRID_SIZE;
  
  // Resolution
  float gridRes = GRID_RESOLUTION;
  memcpy(&gridBuffer[idx], &gridRes, 4);
  idx += 4;
  
  // Robot pose
  float rx = robotX, ry = robotY, rh = robotTheta;
  memcpy(&gridBuffer[idx], &rx, 4);
  idx += 4;
  memcpy(&gridBuffer[idx], &ry, 4);
  idx += 4;
  memcpy(&gridBuffer[idx], &rh, 4);
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

    static uint8_t telemBuf[4096]; // Tăng buffer cho lidar data
    telemBuf[0] = 0x02; 
    size_t len = serializeMsgPack(telem, &telemBuf[1], sizeof(telemBuf) - 1);
    if (len == 0 || len > sizeof(telemBuf) - 1) {
      Serial.printf("[TELEM] WARNING: MsgPack overflow! len=%d, max=%d\n", len, sizeof(telemBuf) - 1);
      len = 0; // Bỏ frame này, không gửi data bị cắt
    }
    if (len > 0) webSocket.broadcastBIN(telemBuf, len + 1);
    
    static unsigned long lastGridSendTime = 0;
    if (streamOccupancyGrid && millis() - lastGridSendTime > 500) {
      lastGridSendTime = millis();
      send_occupancy_grid();
    }
  }
}
