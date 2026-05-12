/**
 * AMR 2.0 v2 — Network Communication
 * Ported from v1: all extern globals replaced with RobotState.
 */

#include "network_comm.h"
#include "config.h"
#include "robot_state.h"
#include "log.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <ArduinoJson.h>
#include <WiFiMulti.h>
#include <Preferences.h>

// Navigation modules
#include "navigator.h"
#include "pathfinder.h"
#include "pathfinder_types.h"
#include "frontier_explorer.h"
#include "occupancy_grid.h"
#include "pid_controller.h"
#include "odometry.h"
#include "battery_adc.h"
#include "ina3221_power.h"
#include "tasks.h"

// ── External instances (from tasks.cpp) ─────────────────────
extern Navigator navigator;
extern AStarPathfinder pathfinder;
extern FrontierExplorer explorer;
extern OccupancyGridMapper gridMapper;
extern WheelPID leftPID, rightPID;

// ── Network globals ─────────────────────────────────────────
static WebServer server(HTTP_PORT);
static WebSocketsServer webSocket(WEBSOCKET_PORT);
static WiFiManager wm;
static WiFiMulti wifiMulti;

static unsigned long lastTelemetryTime = 0;

// ── Architecture Profile ────────────────────────────────────
void setArchitectureProfile(const char* profile) {
    if (strcmp(profile, "pc_slam") == 0) {
        state.nav.archProfile = "pc_slam";
        state.nav.streamOccupancyGrid = false;
        state.nav.allowOnboardNav = false;
        gridMapper.reset();
        navigator.abort();
        LOG_I("ARCH", "Switched to PC_SLAM profile");
        return;
    }
    if (strcmp(profile, "aggressive") == 0) {
        state.nav.archProfile = "aggressive";
        state.nav.streamOccupancyGrid = true;
        state.nav.allowOnboardNav = true;
        LOG_I("ARCH", "Switched to AGGRESSIVE profile");
        return;
    }
    state.nav.archProfile = "hybrid";
    state.nav.streamOccupancyGrid = true;
    state.nav.allowOnboardNav = true;
    LOG_I("ARCH", "Switched to HYBRID profile");
}

// ── WebSocket Event Handler ─────────────────────────────────
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    // Binary MAP_DATA frame
    if (type == WStype_BIN && length > 0 && payload[0] == 0x03) {
        const size_t HEADER_SIZE = 13;
        if (length < HEADER_SIZE) return;
        uint16_t newW, newH; float newRes; uint32_t offset;
        memcpy(&newW, payload+1, 2); memcpy(&newH, payload+3, 2);
        memcpy(&newRes, payload+5, 4); memcpy(&offset, payload+9, 4);

        if (!pathfinder.isInitialized() ||
            newW != (uint16_t)pathfinder.getMapWidth() ||
            newH != (uint16_t)pathfinder.getMapHeight() ||
            fabsf(newRes - pathfinder.getMapResolution()) > 1e-4f) {
            pathfinder.init(newW, newH, newRes);
        }
        pathfinder.updateStaticMap(payload + HEADER_SIZE, length - HEADER_SIZE, offset);

        if (offset + (length - HEADER_SIZE) >= (uint32_t)(newW * newH)) {
            JsonDocument ack; ack["type"] = "map_ack"; ack["w"] = newW; ack["h"] = newH;
            static uint8_t ackBuf[64];
            size_t ackLen = serializeMsgPack(ack, ackBuf, sizeof(ackBuf));
            webSocket.sendBIN(num, ackBuf, ackLen);
        }
        return;
    }

    // Parse JSON/MsgPack
    JsonDocument doc;
    if (type == WStype_TEXT) deserializeJson(doc, payload, length);
    else if (type == WStype_BIN) deserializeMsgPack(doc, payload, length);
    else return;

    // ── Ping/Pong ──
    if (doc["type"] == "ping") {
        JsonDocument pong; pong["type"] = "pong"; pong["ts"] = doc["ts"];
        static uint8_t buf[64];
        size_t len = serializeMsgPack(pong, buf, sizeof(buf));
        webSocket.sendBIN(num, buf, len);
        return;
    }

    // ── Reset Odometry ──
    if (doc["cmd"] == "reset_odom") {
        float rx = doc["x"] | 0.0f, ry = doc["y"] | 0.0f, rt = doc["theta"] | 0.0f;
        portENTER_CRITICAL(&stateMux);
        state.odom.x = rx; state.odom.y = ry; state.odom.theta = rt;
        state.odom.distance = 0;
        state.motor.leftTicks = state.motor.rightTicks = 0;
        state.motor.lastTicksL = state.motor.lastTicksR = 0;
        state.motor.targetLeftVel = state.motor.targetRightVel = 0;
        state.odom.gyroTheta = state.odom.encoderTheta = state.odom.fusedTheta = rt;
        state.tf.dx = state.tf.dy = state.tf.dTheta = 0;
        applyTf();
        portEXIT_CRITICAL(&stateMux);
        gridMapper.centerOnPosition(rx, ry);
        leftPID.reset(); rightPID.reset();
        LOG_I("CMD", "Odometry reset to (%.2f, %.2f, %.2f)", rx, ry, rt);
    }

    // ── HITL Mode ──
    if (doc["cmd"] == "hitl_mode") {
        state.nav.hitlMode = doc["enable"] | false;
        if (state.nav.hitlMode) {
            gridMapper.reset();
            state.nav.streamOccupancyGrid = true;
        }
    }

    // ── HITL Sensor Inject ──
    if (doc["cmd"] == "hitl_sensor" && state.nav.hitlMode) {
        portENTER_CRITICAL(&stateMux);
        if (!doc["x"].isNull()) state.odom.x = doc["x"];
        if (!doc["y"].isNull()) state.odom.y = doc["y"];
        if (!doc["theta"].isNull()) state.odom.theta = doc["theta"];
        state.odom.gyroTheta = state.odom.encoderTheta = state.odom.fusedTheta = state.odom.theta;
        portEXIT_CRITICAL(&stateMux);

        if (state.nav.streamOccupancyGrid)
            gridMapper.update_pose(state.map.x, state.map.y, state.map.theta);

        JsonArray lidarArr = doc["lidar"].as<JsonArray>();
        memset(state.lidar.distances, 0, sizeof(state.lidar.distances));
        state.lidar.obstacleDetected = false;
        for (JsonVariant v : lidarArr) {
            int a = v["a"], d = v["d"];
            if (a >= 0 && a < 360) {
                state.lidar.distances[a] = d;
                if ((a <= 30 || a >= 330) && d > 50 && d < 150) {
                    state.lidar.obstacleDetected = true;
                    state.lidar.lastObstacleTime = millis();
                }
                if (state.nav.streamOccupancyGrid && d > 0)
                    gridMapper.add_point(a, d / 1000.0f);
            }
        }
        if (state.nav.streamOccupancyGrid) gridMapper.update_grid();
    }

    // ── Navigate (path from Web) ──
    if (doc["cmd"] == "navigate") {
        if (!state.nav.allowOnboardNav) return;
        JsonArray pathArr = doc["path"].as<JsonArray>();
        int count = pathArr.size();
        if (count > 0 && count <= MAX_WAYPOINTS) {
            Waypoint tempWps[MAX_WAYPOINTS];
            for (int i = 0; i < count; i++) {
                tempWps[i].x = pathArr[i]["x"]; tempWps[i].y = pathArr[i]["y"];
                tempWps[i].heading = NAN; tempWps[i].useReverse = false;
            }
            float endH = doc["finalHeading"].isNull() ? NAN : doc["finalHeading"].as<float>() * PI / 180.0f;
            navigator.loadPath(tempWps, count, endH);
            JsonDocument ack; ack["type"] = "nav_ack"; ack["wp_count"] = count;
            static uint8_t buf[64];
            size_t len = serializeMsgPack(ack, buf, sizeof(buf));
            webSocket.sendBIN(num, buf, len);
        }
    }

    // ── GOTO (A* onboard pathfinding) ──
    if (doc["cmd"] == "goto") {
        if (!state.nav.allowOnboardNav) setArchitectureProfile("hybrid");
        PoseSnapshot pose = getMapPose();
        GoToRequest req;
        req.startX = pose.x; req.startY = pose.y;
        req.goalX = doc["x"]; req.goalY = doc["y"];
        req.finalHeading = doc["finalHeading"].isNull() ? NAN : doc["finalHeading"].as<float>() * PI / 180.0f;
        if (pathfinderQueue) xQueueSend(pathfinderQueue, &req, 0);
    }

    // ── Traffic (multi-robot obstacles) ──
    if (doc["cmd"] == "traffic") {
        pathfinder.clearDynamicObstacles();
        for (JsonObject r : doc["robots"].as<JsonArray>()) {
            pathfinder.setDynamicObstacle(r["x"], r["y"], r["r"] | 0.3f);
        }
    }

    if (doc["cmd"] == "nav_stop") { if (state.nav.allowOnboardNav) navigator.abort(); }
    if (doc["cmd"] == "pause")    { if (state.nav.allowOnboardNav) navigator.pause(); }
    if (doc["cmd"] == "resume")   { if (state.nav.allowOnboardNav) navigator.resume(); }
    if (doc["cmd"] == "set_arch_mode") setArchitectureProfile(doc["profile"] | "hybrid");

    // ── Set Pose ──
    if (doc["cmd"] == "set_pose") {
        portENTER_CRITICAL(&stateMux);
        state.odom.x = doc["x"] | 0.0f; state.odom.y = doc["y"] | 0.0f;
        state.odom.theta = doc["theta"] | 0.0f;
        state.odom.gyroTheta = state.odom.encoderTheta = state.odom.fusedTheta = state.odom.theta;
        state.odom.distance = 0;
        state.motor.leftTicks = state.motor.rightTicks = 0;
        state.tf.dx = state.tf.dy = state.tf.dTheta = 0;
        applyTf();
        portEXIT_CRITICAL(&stateMux);
        gridMapper.centerOnPosition(state.odom.x, state.odom.y);
        gridMapper.update_pose(state.map.x, state.map.y, state.map.theta);
    }

    // ── Explore ──
    if (doc["cmd"] == "explore") {
        if (!state.nav.allowOnboardNav) setArchitectureProfile("hybrid");
        state.nav.explorationRequested = true;
        gridMapper.reset();
        gridMapper.centerOnPosition(state.odom.x, state.odom.y);
        state.nav.streamOccupancyGrid = true;
        portENTER_CRITICAL(&stateMux);
        state.tf.dx = state.tf.dy = state.tf.dTheta = 0; applyTf();
        portEXIT_CRITICAL(&stateMux);
        state.motor.targetLeftVel = state.motor.targetRightVel = 0;
        leftPID.reset(); rightPID.reset();
        navigator.abort(); explorer.stop();
        JsonDocument ack; ack["type"] = "explore_ack"; ack["status"] = "started";
        static uint8_t buf[64];
        size_t len = serializeMsgPack(ack, buf, sizeof(buf));
        webSocket.sendBIN(num, buf, len);
    }

    if (doc["cmd"] == "explore_stop") {
        state.nav.explorationRequested = false;
        explorer.stop(); navigator.abort();
    }

    if (doc["cmd"] == "recal_gyro") {
        state.imu.calibrated = false; state.imu.calSamples = 0;
        state.imu.calSum = 0; state.imu.bias = 0;
    }

    if (doc["cmd"] == "brake") state.motor.brakeEnabled = doc["val"];

    // ── Manual velocity control ──
    if (!doc["linear"].isNull() && !navigator.isNavigating()) {
        if (!state.motor.brakeEnabled) {
            float v = doc["linear"], w = doc["angular"];
            state.motor.targetLeftVel  = constrain((v - w * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
            state.motor.targetRightVel = constrain((v + w * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
            state.motor.lastCmdTime = millis();
        }
    }
}

// ============================================================
//   WIFI INIT
// ============================================================
void init_network() {
    LOG_I("BOOT", "Initializing WiFi...");

    // loopTask is NOT on WDT yet (added at end of this function)

    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_15dBm);

    // Load saved networks
    Preferences prefs;
    prefs.begin("wifi_cfg", false);
    int count = prefs.getInt("count", 0);
    for (int i = 0; i < count; i++) {
        String ssid = prefs.getString(("ssid" + String(i)).c_str(), "");
        String pass = prefs.getString(("pass" + String(i)).c_str(), "");
        if (ssid.length() > 0) wifiMulti.addAP(ssid.c_str(), pass.c_str());
    }
    String nativeSsid = WiFi.SSID(), nativePass = WiFi.psk();
    if (nativeSsid.length() > 0) wifiMulti.addAP(nativeSsid.c_str(), nativePass.c_str());

    // Try WiFiMulti first (15s) — feed WDT each iteration
    bool connected = false;
    unsigned long startT = millis();
    while (millis() - startT < 15000) {
        esp_task_wdt_reset();  // Prevent WDT crash during blocking scan
        if (wifiMulti.run() == WL_CONNECTED) { connected = true; break; }
        delay(500);
    }
    esp_task_wdt_reset();

    // Fallback: WiFiManager portal
    if (!connected) {
        WiFi.disconnect(true, true); delay(500);
        wm.setConnectTimeout(15);
        wm.setConfigPortalTimeout(120);
        if (!wm.autoConnect(WIFI_AP_NAME)) { ESP.restart(); }

        String newSsid = WiFi.SSID(), newPass = WiFi.psk();
        if (newSsid.length() > 0) {
            bool exists = false;
            for (int i = 0; i < count; i++) {
                if (prefs.getString(("ssid" + String(i)).c_str(), "") == newSsid) {
                    exists = true; prefs.putString(("pass" + String(i)).c_str(), newPass); break;
                }
            }
            if (!exists && count < 10) {
                prefs.putString(("ssid" + String(count)).c_str(), newSsid);
                prefs.putString(("pass" + String(count)).c_str(), newPass);
                prefs.putInt("count", count + 1);
            }
            wifiMulti.addAP(newSsid.c_str(), newPass.c_str());
        }
    }
    prefs.end();

    LOG_I("BOOT", "WiFi ready! IP: %s", WiFi.localIP().toString().c_str());
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);

    // OTA
    ArduinoOTA.setHostname("AMR2_S3");
    ArduinoOTA.onStart([]() { LOG_I("OTA", "Start..."); });
    ArduinoOTA.onEnd([]() { LOG_I("OTA", "Done!"); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        Serial.printf("[OTA] %u%%\r", p / (t / 100));
    });
    ArduinoOTA.onError([](ota_error_t e) { LOG_W("OTA", "Error[%u]", e); });
    ArduinoOTA.begin();

    TelnetStream.begin();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    // NOTE: TCP_NODELAY is already set by WebSocketsServer on each client connect.
    // Server-side heartbeat: detect and evict stale (half-open) client connections.
    // After abnormal disconnect (code 1006), the old client slot stays occupied
    // and blocks new connections (max 5 slots). This sends WS ping every 10s,
    // expects pong within 3s, and evicts after 2 consecutive misses.
    webSocket.enableHeartbeat(10000, 3000, 2);
    server.begin();

    // NOTE: Do NOT add loopTask to WDT.
    // loopTask (priority 1) shares I2C bus with controlTask (priority 5).
    // controlTask reads IMU at 50Hz via I2C, causing priority inversion that
    // starves loopTask when it tries oled_update()/ina3221_read().
    // loopTask only does housekeeping (telemetry, OLED, battery) — not safety-critical.
    // controlTask already has its own WDT feed (tasks.cpp:154).
}

void update_network() {
    webSocket.loop();
    server.handleClient();
    ArduinoOTA.handle();

    static unsigned long lastWifiCheck = 0;
    if (WiFi.status() != WL_CONNECTED && millis() - lastWifiCheck > 5000) {
        lastWifiCheck = millis();
        LOG_W("WIFI", "Disconnected! SDK auto-reconnecting...");
    }
}

void flush_network() {
    webSocket.loop();  // Flush TX buffer — call after broadcast_telemetry()
}

// ============================================================
//   OCCUPANCY GRID BROADCAST
// ============================================================
void send_occupancy_grid() {
    static uint8_t gridBuffer[20000];
    int idx = 0;
    gridBuffer[idx++] = 0x01;
    gridBuffer[idx++] = GRID_SIZE; gridBuffer[idx++] = GRID_SIZE;
    float gridRes = GRID_RESOLUTION;
    memcpy(&gridBuffer[idx], &gridRes, 4); idx += 4;

    PoseSnapshot mp = getMapPose();
    memcpy(&gridBuffer[idx], &mp.x, 4); idx += 4;
    memcpy(&gridBuffer[idx], &mp.y, 4); idx += 4;
    memcpy(&gridBuffer[idx], &mp.theta, 4); idx += 4;

    float ox = gridMapper.originX, oy = gridMapper.originY;
    memcpy(&gridBuffer[idx], &ox, 4); idx += 4;
    memcpy(&gridBuffer[idx], &oy, 4); idx += 4;

    int payloadLen = 0;
    gridMapper.serialize_grid(&gridBuffer[idx], payloadLen);

    // Bounds check — prevent buffer overflow
    if (idx + payloadLen > (int)sizeof(gridBuffer)) {
        LOG_W("GRID", "Grid payload too large (%d bytes) — skipping", idx + payloadLen);
        return;
    }
    idx += payloadLen;
    webSocket.broadcastBIN(gridBuffer, idx);
}

// ============================================================
//   TELEMETRY BROADCAST — Split fast/slow for real-time
// ============================================================

// Fast pose: binary 0x05 + 5 floats = 21 bytes, 20Hz
// No JSON, no MsgPack — raw memcpy for <1ms serialize time
static void broadcast_fast_pose() {
    PoseSnapshot op = getOdomPose();
    float vL = state.motor.vL_meas, vR = state.motor.vR_meas;
    float v_robot = (vR + vL) / 2.0f * WHEEL_RADIUS;
    float w_fused = (state.imu.available && state.imu.calibrated)
                    ? state.imu.gyroZ_raw
                    : (vR - vL) * WHEEL_RADIUS / WHEEL_SEPARATION;

    static uint8_t poseBuf[1 + 5 * 4 + 1];  // 0x05 + 5 floats + batt% = 22 bytes
    poseBuf[0] = 0x05;
    float pose[5] = { op.x, op.y, op.theta, v_robot, w_fused };
    memcpy(&poseBuf[1], pose, 20);
    poseBuf[21] = (uint8_t)state.power.percent;  // Battery 0-100
    webSocket.broadcastBIN(poseBuf, 22);
}

// Full telemetry: MsgPack 0x02, 2Hz (500ms) — all diagnostics
static unsigned long lastFullTelemTime = 0;
static void broadcast_full_telemetry() {
    if (millis() - lastFullTelemTime < 500) return;  // 2Hz
    lastFullTelemTime = millis();

    JsonDocument telem;
    telem["telem"] = true;

    PoseSnapshot op = getOdomPose();
    PoseSnapshot mp = getMapPose();

    float vL = state.motor.vL_meas, vR = state.motor.vR_meas;
    float v_robot = (vR + vL) / 2.0f * WHEEL_RADIUS;
    float w_fused = (state.imu.available && state.imu.calibrated)
                    ? state.imu.gyroZ_raw
                    : (vR - vL) * WHEEL_RADIUS / WHEEL_SEPARATION;

    telem["vx"] = v_robot; telem["wz"] = w_fused;
    telem["theta"] = op.theta; telem["h"] = op.theta * 180.0f / PI;
    telem["d"] = state.odom.distance;
    telem["x"] = op.x; telem["y"] = op.y;
    telem["imu"] = state.imu.available; telem["imu_cal"] = state.imu.calibrated;
    telem["gyroZ"] = state.imu.gyroZ_raw;
    telem["fTheta"] = state.odom.fusedTheta * 180.0f / PI;

    JsonObject enc = telem["enc"].to<JsonObject>();
    enc["l"] = state.motor.leftTicks; enc["r"] = state.motor.rightTicks;

    telem["vL_t"] = state.motor.targetLeftVel; telem["vR_t"] = state.motor.targetRightVel;
    telem["vL_r"] = vL; telem["vR_r"] = vR;
    telem["pwmL"] = (int)state.motor.pwmLeft; telem["pwmR"] = (int)state.motor.pwmRight;
    telem["batt"] = state.power.percent;

    telem["nav"] = navigator.getStateName();
    telem["nav_wp"] = navigator.currentWpIdx;
    telem["nav_total"] = navigator.waypointCount;
    telem["nav_rec"] = navigator.recoveryAttempts;
    telem["nav_recovering"] = navigator.isRecovering();
    telem["arch"] = state.nav.archProfile;
    telem["grid_stream"] = state.nav.streamOccupancyGrid;
    telem["onboard_nav"] = state.nav.allowOnboardNav;
    telem["hitl"] = state.nav.hitlMode;
    telem["explore"] = explorer.getStateName();
    telem["explore_goals"] = explorer.exploredGoals;
    telem["explore_frontiers"] = explorer.frontierCellCount;

    // SLAM diagnostics
    JsonObject slam = telem["slam"].to<JsonObject>();
    slam["score"]    = (int)(state.slam.matchScore * 100);
    slam["tfNorm"]   = state.slam.tfNorm;
    slam["tfDeg"]    = state.slam.tfAngleDeg;
    slam["coverage"] = state.slam.gridCoverage;
    slam["occ"]      = state.slam.gridOccupied;
    slam["free"]     = state.slam.gridFree;
    slam["rms"]      = state.slam.icpRms;
    slam["scans"]    = state.slam.scanCount;
    slam["mX"] = mp.x; slam["mY"] = mp.y; slam["mTh"] = mp.theta * 180.0f / PI;

    telem["eX"] = navigator.error_x;
    telem["eY"] = navigator.error_y;
    telem["eYaw"] = navigator.error_yaw;

    // INA3221 power
    if (state.power.inaAvailable) {
        JsonArray inaV = telem["inaV"].to<JsonArray>();
        JsonArray inaA = telem["inaA"].to<JsonArray>();
        for (int i = 0; i < 3; i++) { inaV.add(state.power.busV[i]); inaA.add(state.power.currentA[i]); }
        JsonObject pwr = telem["power"].to<JsonObject>();
        pwr["battV"] = state.power.busV[INA_CH_BATT]; pwr["battA"] = state.power.currentA[INA_CH_BATT];
        pwr["motorV"] = state.power.busV[INA_CH_MOTOR]; pwr["motorA"] = state.power.currentA[INA_CH_MOTOR];
    }

    // LiDAR — obstacle flag only
    bool hasObs = state.lidar.obstacleDetected && millis() - state.lidar.lastObstacleTime < 500;
    telem["obs"] = hasObs;

    // Send as MsgPack binary (0x02 prefix)
    static uint8_t telemBuf[2048];
    telemBuf[0] = 0x02;
    size_t reqLen = measureMsgPack(telem);
    size_t len = 0;
    if (reqLen <= sizeof(telemBuf) - 1) {
        len = serializeMsgPack(telem, &telemBuf[1], sizeof(telemBuf) - 1);
    }
    if (len > 0) webSocket.broadcastBIN(telemBuf, len + 1);
}

void broadcast_telemetry() {
    if (millis() - lastTelemetryTime < TELEMETRY_INTERVAL) return;
    lastTelemetryTime = millis();

    // ── FAST: Binary pose every tick (20Hz, 22 bytes, <1ms) ──
    broadcast_fast_pose();

    // ── SLOW: Full MsgPack telemetry at 2Hz (500ms) ──
    broadcast_full_telemetry();

    // ── LiDAR binary (20Hz, 721 bytes) ──
    static uint8_t lidarBuf[1 + 360 * 2];
    lidarBuf[0] = 0x04;
    uint16_t* lidarData = (uint16_t*)&lidarBuf[1];
    for (int i = 0; i < 360; i++) {
        lidarData[i] = state.lidar.distances[i];
    }
    webSocket.broadcastBIN(lidarBuf, sizeof(lidarBuf));

    // Grid broadcast (2Hz)
    static unsigned long lastGridSendTime = 0;
    if (state.nav.streamOccupancyGrid && millis() - lastGridSendTime > 500) {
        lastGridSendTime = millis();
        send_occupancy_grid();
    }
}


