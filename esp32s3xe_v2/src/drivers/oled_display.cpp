/**
 * OLED Display — SSD1306 128x64 Implementation
 * Shows IP, battery, sensor status, navigation info.
 */

#include "oled_display.h"
#include "config.h"
#include "robot_state.h"
#include "log.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>

extern SemaphoreHandle_t i2cMutex;

#define OLED_INTERVAL 500

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static unsigned long lastOledTime = 0;

bool oled_init() {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    
    bool ok = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    if (ok) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("AMR 2.0 v2 Boot...");
        display.display();
        LOG_I("OLED", "SSD1306 initialized");
    } else {
        LOG_E("OLED", "SSD1306 not found at 0x%02X", SCREEN_ADDRESS);
    }
    xSemaphoreGive(i2cMutex);
    return ok;
}

void oled_update() {
    if (millis() - lastOledTime < OLED_INTERVAL) return;
    lastOledTime = millis();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.setTextSize(1, 2);

    // Row 1: IP & WiFi status
    display.setCursor(0, 0);
    display.printf("AMR %s %s",
        WiFi.localIP().toString().c_str(),
        WiFi.status() == WL_CONNECTED ? "ON" : "OFF");

    // Row 2: Battery, IMU, LiDAR status
    display.setCursor(0, 16);
    display.printf("B:%d%% I:%s L:%s",
        state.power.percent,
        state.imu.available ? "OK" : "--",
        state.lidar.receiving ? "OK" : "--");

    // Row 3: Speed & heading
    display.setCursor(0, 32);
    float v_avg = (state.motor.vR_meas + state.motor.vL_meas) / 2.0f * WHEEL_RADIUS;
    display.printf("Spd:%.2fm/s H:%.0f", v_avg, state.odom.fusedTheta * 180.0f / PI);

    // Row 4: Position
    display.setCursor(0, 48);
    display.printf("Pos X:%.1f Y:%.1f", state.map.x, state.map.y);

    // Send to display (I2C protected)
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        display.display();
        xSemaphoreGive(i2cMutex);
    }
}
