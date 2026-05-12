// ============================================================
//   AMR 2.0 v2 — ESP32-S3 Firmware
//   Clean Layered Architecture
//   Phase 5: Cleanup + Verification
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <Preferences.h>

#include "config.h"
#include "robot_state.h"
#include "log.h"

// Layer 1: Drivers
#include "motor_driver.h"
#include "encoder_driver.h"
#include "imu_mpu6050.h"
#include "lidar_a1m8.h"
#include "oled_display.h"
#include "ina3221_power.h"
#include "battery_adc.h"

// Layer 2: Perception
#include "odometry.h"
#include "occupancy_grid.h"

// Layer 4: Network
#include "network_comm.h"

// Core: Tasks
#include "tasks.h"

// ── Global Peripherals ──────────────────────────────────────
SemaphoreHandle_t i2cMutex;
Adafruit_NeoPixel rgbLed(1, RGB_BUILTIN_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================
//   SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 1000) {}
    LOG_I("BOOT", "=== AMR 2.0 v2 — Starting setup() ===");

    // ── Layer 0: Core ────────────────────────────────────────
    i2cMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeout(10);
#if defined(ESP32)
    Wire.setTimeOut(10);
#endif

    rgbLed.begin();
    rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 50));  // Blue = booting
    rgbLed.show();

    LOG_I("BOOT", "I2C initialized (SDA=%d, SCL=%d, 400kHz)", SDA_PIN, SCL_PIN);

    // ── Layer 1: Drivers ─────────────────────────────────────
    motor_init();
    encoder_init();

    state.imu.available = imu_init();

    bool oledOk = oled_init();
    LOG_I("BOOT", "OLED: %s", oledOk ? "OK" : "FAIL");

    state.power.inaAvailable = ina3221_init();

    battery_init();
    state.power.filteredVBatt = 12.0f;  // Init at nominal so EMA converges fast
    for (int i = 0; i < 20; i++) battery_update();  // 20 samples to settle
    LOG_I("BOOT", "Battery: %.2fV (%d%%)", state.power.filteredVBatt, state.power.percent);

    // LiDAR init (takes ~3s due to motor spin-up)
    bool lidarOk = lidar_init();
    state.lidar.running = lidarOk;
    LOG_I("BOOT", "LiDAR: %s", lidarOk ? "OK" : "FAIL");

    // ── Layer 2: Perception ──────────────────────────────────
    odometry_init();
    gridMapper.centerOnPosition(state.odom.x, state.odom.y);
    LOG_I("BOOT", "Occupancy grid centered at (%.1f, %.1f)", state.odom.x, state.odom.y);

    // ── Watchdog Timer ───────────────────────────────────────
    // Configure WDT but don't add loopTask yet — init_network blocks >5s
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_task_wdt_config_t wdtCfg = { .timeout_ms = 8000, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_reconfigure(&wdtCfg);
#else
    esp_task_wdt_init(8, true);  // 8s timeout, panic on trigger
#endif
    // NOTE: Do NOT add loopTask to WDT here — init_network() blocks for WiFi

    // ── FreeRTOS Tasks ───────────────────────────────────────
    tasks_create();

    // ── Layer 4: Network (WiFi + WebSocket + OTA) ────────────
    init_network();  // Blocking WiFi connect (~15s max)

    // ── Bump loopTask priority ──────────────────────────────
    // Default Arduino loopTask = priority 1, but lidarTask = 3 on same Core 0.
    // This starves webSocket.loop() → ESP32 can't accept TCP connections.
    // Bump to 4 so WebSocket ALWAYS gets CPU time above LiDAR processing.
    vTaskPrioritySet(NULL, 4);
    LOG_I("BOOT", "loopTask priority raised to 4 (above lidarTask=3)");

    // ── Status LED ───────────────────────────────────────────
    if (state.imu.available && lidarOk) {
        rgbLed.setPixelColor(0, rgbLed.Color(0, 50, 0));  // Green = all good
    } else {
        rgbLed.setPixelColor(0, rgbLed.Color(50, 50, 0));  // Yellow = partial
    }
    rgbLed.show();

    LOG_I("BOOT", "=== Setup complete (Phase 4) ===");
    LOG_I("BOOT", "  RAM free: %d bytes", ESP.getFreeHeap());
    LOG_I("BOOT", "  PSRAM free: %d bytes", ESP.getFreePsram());
}

// ============================================================
//   LOOP — Housekeeping (slow tasks)
//   controlTask/lidarTask run on dedicated FreeRTOS tasks
// ============================================================
void loop() {
    // NOTE: loopTask is NOT on WDT (see network_comm.cpp).
    // Only controlTask is WDT-monitored (safety-critical motor PID).

    // ── 1. WebSocket — service incoming commands FIRST ──
    update_network();

    // ── 2. Telemetry broadcast (10Hz) — send BEFORE any I2C blocking ──
    broadcast_telemetry();

    // ── 3. Flush WebSocket TX buffer immediately after broadcast ──
    flush_network();

    // ── 4. I2C slow operations — AFTER network is serviced ──────
    static unsigned long lastSlowUpdate = 0;
    if (millis() - lastSlowUpdate > 500) {
        lastSlowUpdate = millis();
        battery_update();
        ina3221_read();

        for (int i = 0; i < 3; i++) {
            state.power.busV[i]     = ina_busV[i];
            state.power.currentA[i] = ina_currentA[i];
        }
    }

    oled_update();  // I2C OLED (~30-50ms) — now AFTER telemetry, won't delay data

    // ── Serial status heartbeat (5s) ─────────────────────────
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        LOG_I("LOOP", "Batt:%d%% IMU:%s LiDAR:%s WS:%d RSSI:%d Pos:(%.2f,%.2f) h:%.0f Heap:%d",
              state.power.percent,
              state.imu.available ? (state.imu.calibrated ? "OK" : "CAL") : "FAIL",
              state.lidar.receiving ? "RUN" : (state.lidar.running ? "WAIT" : "OFF"),
              WiFi.status() == WL_CONNECTED ? 1 : 0,
              WiFi.RSSI(),
              state.map.x, state.map.y, state.map.theta * 180.0f / PI,
              ESP.getFreeHeap());
    }

    // ── Serial Commands ───────────────────────────────────────
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "reset_wifi") {
            LOG_I("CMD", "Erasing WiFi credentials...");
            Preferences prefs;
            prefs.begin("wifi_cfg", false);
            prefs.clear();
            prefs.end();
            WiFi.disconnect(true, true);  // Erase native WiFi credentials
            LOG_I("CMD", "WiFi reset! Rebooting into AP portal...");
            delay(500);
            ESP.restart();
        } else if (cmd == "reboot") {
            LOG_I("CMD", "Rebooting...");
            delay(200);
            ESP.restart();
        } else if (cmd == "ip") {
            LOG_I("CMD", "IP: %s", WiFi.localIP().toString().c_str());
        } else if (cmd == "help") {
            Serial.println("=== AMR 2.0 Serial Commands ===");
            Serial.println("  reset_wifi  — Xóa WiFi đã lưu, reboot vào AP portal");
            Serial.println("  reboot      — Khởi động lại");
            Serial.println("  ip          — Hiện IP hiện tại");
            Serial.println("  help        — Hiện danh sách lệnh");
        }
    }

    delay(5);  // Reduced from 10ms → 5ms for faster WebSocket servicing
}
