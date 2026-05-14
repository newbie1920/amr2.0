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
// WiFiMulti and Preferences removed — using WiFiManager only

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
// WiFiMulti removed — WiFiManager handles everything

static unsigned long lastTelemetryTime = 0;

// ── Architecture Profile ────────────────────────────────────
void setArchitectureProfile(const char* profile) {
    if (strcmp(profile, "pc_slam") == 0) {
        state.nav.archProfile = "pc_slam";
        state.nav.streamOccupancyGrid = false;
        state.nav.allowOnboardNav = false;
        state.nav.mode = RobotState::MODE_PC_BROWSER;
        gridMapper.reset();
        navigator.abort();
        LOG_I("ARCH", "Switched to PC_SLAM profile (MODE_PC_BROWSER)");
        return;
    }
    if (strcmp(profile, "aggressive") == 0) {
        state.nav.archProfile = "aggressive";
        state.nav.streamOccupancyGrid = true;
        state.nav.allowOnboardNav = true;
        state.nav.mode = RobotState::MODE_ONBOARD;
        LOG_I("ARCH", "Switched to AGGRESSIVE profile (MODE_ONBOARD)");
        return;
    }
    state.nav.archProfile = "hybrid";
    state.nav.streamOccupancyGrid = true;
    state.nav.allowOnboardNav = true;
    state.nav.mode = RobotState::MODE_ONBOARD;
    LOG_I("ARCH", "Switched to HYBRID profile (MODE_ONBOARD)");
}

// ── Switch NavMode directly ─────────────────────────────────
static void switchNavMode(RobotState::NavMode newMode) {
    if (state.nav.mode == newMode) return;
    state.nav.mode = newMode;

    if (newMode == RobotState::MODE_PC_BROWSER) {
        // Suspend onboard SLAM & pathfinding — ESP32 becomes "spinal cord"
        state.nav.streamOccupancyGrid = false;
        state.nav.allowOnboardNav = false;
        state.nav.archProfile = "pc_slam";
        navigator.abort();
        explorer.stop();
        state.nav.explorationRequested = false;
        // Keep motor control alive — PC sends velocity or waypoints
        LOG_I("MODE", "→ PC_BROWSER: onboard SLAM/Nav suspended, raw streaming active");
    } else {
        // Resume onboard SLAM
        state.nav.streamOccupancyGrid = true;
        state.nav.allowOnboardNav = true;
        state.nav.archProfile = "hybrid";
        gridMapper.reset();
        gridMapper.centerOnPosition(state.odom.x, state.odom.y);
        portENTER_CRITICAL(&stateMux);
        state.tf.dx = state.tf.dy = state.tf.dTheta = 0;
        applyTf();
        portEXIT_CRITICAL(&stateMux);
        leftPID.reset(); rightPID.reset();
        LOG_I("MODE", "→ ONBOARD: SLAM/Nav resumed, grid reset");
    }
}

// ── WebSocket Event Handler ─────────────────────────────────
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    // ── Client connect/disconnect — evict stale slots from same IP ──
    if (type == WStype_CONNECTED) {
        IPAddress newIP = webSocket.remoteIP(num);
        LOG_I("WS", "Client #%d connected from %s", num, newIP.toString().c_str());

        // Evict any OTHER client with the same IP (stale slot from previous session)
        for (uint8_t i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
            if (i != num && webSocket.remoteIP(i) == newIP) {
                LOG_W("WS", "Evicting stale client #%d (same IP: %s)", i, newIP.toString().c_str());
                webSocket.disconnect(i);
            }
        }
        return;
    }
    if (type == WStype_DISCONNECTED) {
        LOG_I("WS", "Client #%d disconnected", num);
        return;
    }

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
        JsonArray pathArr = doc["path"].as<JsonArray>();
        int count = pathArr.size();
        if (count > 0 && count <= MAX_WAYPOINTS) {
            state.nav.allowOnboardNav = true;
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

    // ── Switch Dual-Mode ──
    if (doc["cmd"] == "set_mode") {
        const char* m = doc["mode"] | "onboard";
        if (strcmp(m, "pc_browser") == 0) {
            switchNavMode(RobotState::MODE_PC_BROWSER);
        } else {
            switchNavMode(RobotState::MODE_ONBOARD);
        }
        // ACK
        JsonDocument ack;
        ack["type"] = "mode_ack";
        ack["mode"] = (state.nav.mode == RobotState::MODE_PC_BROWSER) ? "pc_browser" : "onboard";
        static uint8_t buf[64];
        size_t len = serializeMsgPack(ack, buf, sizeof(buf));
        webSocket.sendBIN(num, buf, len);
    }

    // ── External Waypoints (PC_BROWSER mode: PC sends path, ESP32 follows) ──
    if (doc["cmd"] == "ext_waypoints") {
        JsonArray pathArr = doc["path"].as<JsonArray>();
        int count = pathArr.size();
        if (count > 0 && count <= MAX_WAYPOINTS) {
            // Temporarily allow onboard nav to execute the path
            state.nav.allowOnboardNav = true;
            Waypoint tempWps[MAX_WAYPOINTS];
            for (int i = 0; i < count; i++) {
                tempWps[i].x = pathArr[i]["x"];
                tempWps[i].y = pathArr[i]["y"];
                tempWps[i].heading = NAN;
                tempWps[i].useReverse = pathArr[i]["rev"] | false;
            }
            float endH = doc["finalHeading"].isNull() ? NAN : doc["finalHeading"].as<float>() * PI / 180.0f;
            navigator.loadPath(tempWps, count, endH);
            LOG_I("EXT_WP", "PC sent %d waypoints → Navigator loaded", count);

            JsonDocument ack;
            ack["type"] = "ext_wp_ack";
            ack["wp_count"] = count;
            static uint8_t buf[64];
            size_t len = serializeMsgPack(ack, buf, sizeof(buf));
            webSocket.sendBIN(num, buf, len);
        }
    }

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
//   WIFI INIT — Simple: WiFiManager only
// ============================================================
void init_network() {
    LOG_I("BOOT", "Initializing WiFi...");

    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);     // No power saving — max throughput
    WiFi.setTxPower(WIFI_POWER_15dBm);

    // WiFiManager: tries last known network first (~3s), opens AP portal if fails
    wm.setConnectTimeout(8);           // Max 8s to connect to known network
    wm.setConfigPortalTimeout(120);    // AP portal stays open 2min

    unsigned long startT = millis();
    esp_task_wdt_reset();

    if (!wm.autoConnect(WIFI_AP_NAME)) {
        LOG_W("WIFI", "Failed to connect — restarting...");
        ESP.restart();
    }

    LOG_I("WIFI", "Connected in %lums — IP: %s", millis() - startT,
          WiFi.localIP().toString().c_str());
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
    // Heartbeat: 5s ping, 2s pong timeout, 2 misses = evict after 14s max
    webSocket.enableHeartbeat(5000, 2000, 2);
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
//   OCCUPANCY GRID BROADCAST — Windowed RLE for large grids
//   Sends only the local window around the robot (VIEW_SIZE x VIEW_SIZE)
//   with Run-Length Encoding to keep packets < 8KB.
// ============================================================

// Viewport size around robot (cells). 256 cells × 0.05m = 12.8m window.
#define GRID_VIEW_SIZE 256

void send_occupancy_grid() {
    // Allocate on PSRAM if available, else use a smaller static buffer
    static uint8_t* gridBuffer = nullptr;
    static const int GRID_BUF_SIZE = 32000;  // 32KB max packet
    if (!gridBuffer) {
        gridBuffer = (uint8_t*)heap_caps_malloc(GRID_BUF_SIZE, MALLOC_CAP_SPIRAM);
        if (!gridBuffer) gridBuffer = (uint8_t*)malloc(GRID_BUF_SIZE);
        if (!gridBuffer) { LOG_E("GRID", "Buffer alloc failed!"); return; }
    }

    PoseSnapshot mp = getMapPose();

    // Compute windowed view centered on robot
    int robotGX = gridMapper.world_to_grid_x(mp.x);
    int robotGY = gridMapper.world_to_grid_y(mp.y);
    int half = GRID_VIEW_SIZE / 2;
    int viewStartX = constrain(robotGX - half, 0, GRID_SIZE - GRID_VIEW_SIZE);
    int viewStartY = constrain(robotGY - half, 0, GRID_SIZE - GRID_VIEW_SIZE);
    int viewW = min(GRID_VIEW_SIZE, GRID_SIZE - viewStartX);
    int viewH = min(GRID_VIEW_SIZE, GRID_SIZE - viewStartY);

    // ── Build header ──
    int idx = 0;
    gridBuffer[idx++] = 0x01;  // Frame type

    // View dimensions (uint16)
    uint16_t vw = (uint16_t)viewW, vh = (uint16_t)viewH;
    memcpy(&gridBuffer[idx], &vw, 2); idx += 2;
    memcpy(&gridBuffer[idx], &vh, 2); idx += 2;

    // Resolution
    float gridRes = GRID_RESOLUTION;
    memcpy(&gridBuffer[idx], &gridRes, 4); idx += 4;

    // Robot pose (map frame)
    memcpy(&gridBuffer[idx], &mp.x, 4); idx += 4;
    memcpy(&gridBuffer[idx], &mp.y, 4); idx += 4;
    memcpy(&gridBuffer[idx], &mp.theta, 4); idx += 4;

    // Grid origin (world coords of cell [0,0])
    float ox = gridMapper.originX, oy = gridMapper.originY;
    memcpy(&gridBuffer[idx], &ox, 4); idx += 4;
    memcpy(&gridBuffer[idx], &oy, 4); idx += 4;

    // View offset (uint16) — so browser knows where this window starts
    uint16_t vsX = (uint16_t)viewStartX, vsY = (uint16_t)viewStartY;
    memcpy(&gridBuffer[idx], &vsX, 2); idx += 2;
    memcpy(&gridBuffer[idx], &vsY, 2); idx += 2;

    // Full grid size (uint16)
    uint16_t fullSize = (uint16_t)GRID_SIZE;
    memcpy(&gridBuffer[idx], &fullSize, 2); idx += 2;

    // ── RLE encode the view window ──
    // Format: [value (uint8), count (uint8)] pairs. Max run = 255.
    int maxPayload = GRID_BUF_SIZE - idx - 4;  // Safety margin
    int rleStart = idx;

    uint8_t prevVal = gridMapper.get_occupancy(viewStartX, viewStartY);
    uint8_t runLen = 1;

    for (int y = 0; y < viewH; y++) {
        for (int x = 0; x < viewW; x++) {
            if (y == 0 && x == 0) continue;  // Already captured first cell
            uint8_t val = gridMapper.get_occupancy(viewStartX + x, viewStartY + y);
            if (val == prevVal && runLen < 255) {
                runLen++;
            } else {
                // Flush run
                if (idx - rleStart + 2 > maxPayload) goto rle_done;
                gridBuffer[idx++] = prevVal;
                gridBuffer[idx++] = runLen;
                prevVal = val;
                runLen = 1;
            }
        }
    }
    // Flush final run
    if (idx - rleStart + 2 <= maxPayload) {
        gridBuffer[idx++] = prevVal;
        gridBuffer[idx++] = runLen;
    }
rle_done:

    webSocket.broadcastBIN(gridBuffer, idx);
}

// ============================================================
//   TELEMETRY BROADCAST — Split fast/slow for real-time
// ============================================================

// Fast pose: binary 0x05 + 5 floats = 21 bytes, 20Hz
// No JSON, no MsgPack — raw memcpy for <1ms serialize time
// NOTE: Uses MAP-frame pose (odom + TF correction) so browser renders
//       robot in the same coordinate system as the occupancy grid.
static void broadcast_fast_pose() {
    PoseSnapshot mp = getMapPose();  // MAP frame — matches grid coordinate system
    float vL = state.motor.vL_meas, vR = state.motor.vR_meas;
    float v_robot = (vR + vL) / 2.0f * WHEEL_RADIUS;
    float w_fused = (state.imu.available && state.imu.calibrated)
                    ? state.imu.gyroZ_raw
                    : (vR - vL) * WHEEL_RADIUS / WHEEL_SEPARATION;

    static uint8_t poseBuf[1 + 5 * 4 + 1];  // 0x05 + 5 floats + batt% = 22 bytes
    poseBuf[0] = 0x05;
    float pose[5] = { mp.x, mp.y, mp.theta, v_robot, w_fused };
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
    telem["theta"] = mp.theta; telem["h"] = mp.theta * 180.0f / PI;
    telem["d"] = state.odom.distance;
    telem["x"] = mp.x; telem["y"] = mp.y;  // MAP frame — matches grid
    telem["oX"] = op.x; telem["oY"] = op.y;  // Raw ODOM (debug / Topic Inspector)
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

// ── Raw LiDAR scan for PC_BROWSER SLAM ──────────────────────
// Frame 0x06: raw (angle_deg:float, distance_m:float, quality:uint8) per point
// Sent at grid-update rate (~5Hz) with all buffered points
static void broadcast_raw_lidar_scan() {
    // In PC_BROWSER mode, gridMapper still collects points via add_point()
    // but does NOT call update_grid(). We read the buffer and stream it.
    int count = gridMapper.point_count;
    if (count == 0) return;

    // Frame: 0x06 + uint16(count) + N × (float angle + float dist + uint8 quality) = 9 bytes/pt
    const int POINT_SIZE = 9;  // 4+4+1
    int frameSize = 1 + 2 + count * POINT_SIZE;

    // Use PSRAM-backed buffer for large scans
    static uint8_t* rawBuf = nullptr;
    static int rawBufSize = 0;
    if (frameSize > rawBufSize) {
        if (rawBuf) heap_caps_free(rawBuf);
        rawBufSize = frameSize + 256;  // Extra headroom
        rawBuf = (uint8_t*)heap_caps_malloc(rawBufSize, MALLOC_CAP_SPIRAM);
        if (!rawBuf) rawBuf = (uint8_t*)malloc(rawBufSize);
        if (!rawBuf) { rawBufSize = 0; return; }
    }

    int idx = 0;
    rawBuf[idx++] = 0x06;  // Raw scan frame type
    uint16_t cnt = (uint16_t)count;
    memcpy(&rawBuf[idx], &cnt, 2); idx += 2;

    for (int i = 0; i < count; i++) {
        const LidarPoint& pt = gridMapper.points[i];
        memcpy(&rawBuf[idx], &pt.angle, 4);    idx += 4;
        memcpy(&rawBuf[idx], &pt.distance, 4); idx += 4;
        rawBuf[idx++] = pt.quality ? 1 : 0;
    }

    webSocket.broadcastBIN(rawBuf, idx);

    // Clear the point buffer (since onboard SLAM isn't consuming it)
    gridMapper.point_count = 0;
}

void broadcast_telemetry() {
    if (millis() - lastTelemetryTime < TELEMETRY_INTERVAL) return;
    lastTelemetryTime = millis();

    // ── FAST: Binary pose every tick (20Hz, 22 bytes, <1ms) ──
    broadcast_fast_pose();

    // ── SLOW: Full MsgPack telemetry at 2Hz (500ms) ──
    broadcast_full_telemetry();

    // ── LiDAR binary (20Hz, 721 bytes) — distance table ──
    static uint8_t lidarBuf[1 + 360 * 2];
    lidarBuf[0] = 0x04;
    uint16_t* lidarData = (uint16_t*)&lidarBuf[1];
    for (int i = 0; i < 360; i++) {
        lidarData[i] = state.lidar.distances[i];
    }
    webSocket.broadcastBIN(lidarBuf, sizeof(lidarBuf));

    // ── MODE-dependent streaming ──
    if (state.nav.mode == RobotState::MODE_PC_BROWSER) {
        // PC_BROWSER: stream raw lidar scans for PC-side SLAM
        static unsigned long lastRawScanTime = 0;
        if (millis() - lastRawScanTime > GRID_UPDATE_INTERVAL_MS) {
            lastRawScanTime = millis();
            broadcast_raw_lidar_scan();
        }
    } else {
        // ONBOARD: send occupancy grid at 2Hz
        static unsigned long lastGridSendTime = 0;
        if (state.nav.streamOccupancyGrid && millis() - lastGridSendTime > 500) {
            lastGridSendTime = millis();
            send_occupancy_grid();
        }
    }
}

