#include "display_oled.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "navigator.h"
#include "imu_sensor.h"
#include "odometry.h"

extern SemaphoreHandle_t i2cMutex;
extern WebSocketsServer webSocket;
extern Navigator navigator;

#define OLED_INTERVAL 500

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned long lastOledTime = 0;

void init_oled() {
  if (i2cMutex != NULL) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("AMR 2.0 Booting...");
        display.display();
      }
      xSemaphoreGive(i2cMutex);
    }
  }
}

void update_oled() {
  if (millis() - lastOledTime > OLED_INTERVAL) {
    lastOledTime = millis();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.setTextSize(1, 2);
    
    display.setCursor(0, 0);
    display.printf("AMR %s %s", WiFi.localIP().toString().c_str(), WiFi.status() == WL_CONNECTED ? "ON" : "OFF");

    display.setCursor(0, 16);
    int b_pct = (int)map(constrain(filteredVBatt, BATT_MIN_V, BATT_MAX_V), BATT_MIN_V, BATT_MAX_V, 0, 100);
    display.printf("Bat:%d%% IMU:%s WS:%d", b_pct, imuAvailable ? "OK" : "--", webSocket.connectedClients());

    display.setCursor(0, 32);
    if (navigator.isNavigating()) {
      String st = navigator.getStateName();
      if (st == "NAV_TURNING") st = "TURN";
      else if (st == "NAV_DRIVING") st = "DRIVE";
      else if (st == "NAV_FINAL_TURN") st = "F_TURN";
      display.printf("NAV:%s WP:%d/%d", st.c_str(), navigator.currentWpIdx + 1, navigator.waypointCount);
    } else {
      float v_avg = (vR_meas + vL_meas) / 2.0f * WHEEL_RADIUS;
      display.printf("Spd:%.2fm/s H:%.0f", v_avg, fusedTheta * 180.0f / PI);
    }

    display.setCursor(0, 48);
    display.printf("Pos X:%.1f Y:%.1f", robotX, robotY);

    if (i2cMutex != NULL) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            display.display();
            xSemaphoreGive(i2cMutex);
        }
    }
  }
}
