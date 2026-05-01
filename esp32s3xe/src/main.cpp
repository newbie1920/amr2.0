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

#include "config.h"
#include "navigator.h"
#include "wheel_pid.h"
#include "lidar_mapper.h"

// Included Modules
#include "imu_sensor.h"
#include "odometry.h"
#include "display_oled.h"
#include "network_comm.h"

Adafruit_NeoPixel rgbLed(1, RGB_BUILTIN_PIN, NEO_GRB + NEO_KHZ800);
SemaphoreHandle_t i2cMutex;

// ─── GLOBAL STATE ────────────────────────────────────────────
Navigator navigator;

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
const char* architectureProfile = "hybrid";

// Network Globals
WebServer server(HTTP_PORT);
WebSocketsServer webSocket(WEBSOCKET_PORT);
WiFiManager wm;

// ============================================================
//   FREERTOS CONTROL TASK (50Hz)
// ============================================================
void controlTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_FREQ_HZ);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float deltaT = 1.0f / CONTROL_FREQ_HZ;

  for (;;) {
    // ── IMU Read ────────────────────────────────────────
    if (imuAvailable) {
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

    // ── AUTONOMOUS NAVIGATOR ──
    if (navigator.isNavigating()) {
      navigator.update(robotX, robotY, robotTheta);
      float navV = navigator.cmdLinear;
      float navW = navigator.cmdAngular;
      targetLeftVel = constrain((navV - navW * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
      targetRightVel = constrain((navV + navW * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
    }

    // ── Motor PI + Feedforward ──────────────────────────
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

    setMotor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, 0, pwmLeft);
    setMotor(MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, 1, pwmRight);

    // Obstacle Avoidance Auto-Pause Logic
    if (obstacleDetected && millis() - timeObstacleLastDetected < 500) {
        if (navigator.isNavigating() && navigator.state == NAV_TRACKING) {
            navigator.pause();
            Serial.println("[LIDAR] E-STOP: Phat hien vat can truoc xe!");
        }
        if (targetL > 0 || targetR > 0) {
            targetLeftVel = 0;
            targetRightVel = 0;
        }
    } else if (obstacleDetected && millis() - timeObstacleLastDetected >= 500) {
        obstacleDetected = false;
        if (navigator.isNavigating() && navigator.state == NAV_PAUSED) {
            navigator.resume();
            Serial.println("[LIDAR] Vat can da di chuyen, RESUME!");
        }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================================
//   LIDAR FREERTOS TASK
// ============================================================
void lidarTask(void *pvParameters) {
  lidarRunning = true;
  for (;;) {
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance; // distance value in mm
      float angle    = lidar.getCurrentPoint().angle;    // angle value in degrees
      uint8_t quality = lidar.getCurrentPoint().quality; // quality of the current measurement

      if (quality > 0) {
        int deg = (int)round(angle) % 360;
        lidarDists[deg] = (uint16_t)distance;
        
        if (streamOccupancyGrid) {
          gridMapper.add_point(angle, distance / 1000.0f);
        }

        if ((deg <= 30 || deg >= 330) && distance > 50 && distance < 450) {
            obstacleDetected = true;
            timeObstacleLastDetected = millis();
        }
      }
      
      if (streamOccupancyGrid &&
          millis() - lastGridUpdateTime > GRID_UPDATE_INTERVAL &&
          gridMapper.point_count > 180) {
        gridMapper.update_pose(robotX, robotY, robotTheta);
        gridMapper.update_grid();
        lastGridUpdateTime = millis();
      }
    } else {
      analogWrite(LIDAR_PWM_PIN, 0); 
      vTaskDelay(pdMS_TO_TICKS(100));
      lidar.begin(lidarSerial);
      analogWrite(LIDAR_PWM_PIN, 255); // Tăng tốc độ quay Lidar lên tối đa
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

  // Initialize Onboard RGB LED (dim and soft color)
  rgbLed.begin();
  rgbLed.setBrightness(15);
  rgbLed.setPixelColor(0, rgbLed.Color(30, 80, 150));
  rgbLed.show();

  // Initialize Lidar
  pinMode(LIDAR_PWM_PIN, OUTPUT);
  lidar.begin(lidarSerial);
  lidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  analogWrite(LIDAR_PWM_PIN, 255); // Tăng tốc độ quay Lidar lên tối đa

  leftPID = new WheelPID(KP_VEL, KI_VEL, 0.0f, FF_GAIN_LEFT, 1.0f / CONTROL_FREQ_HZ, 5.0f, MIN_PWM);
  rightPID = new WheelPID(KP_VEL, KI_VEL, 0.0f, FF_GAIN_RIGHT, 1.0f / CONTROL_FREQ_HZ, 5.0f, MIN_PWM);

  analogSetPinAttenuation(BATT_PIN, ADC_11db);
  pinMode(BATT_PIN, INPUT);

  init_encoders();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeout(20);

  i2cMutex = xSemaphoreCreateMutex();
  
  init_oled();

  Serial.println("[BOOT] Khoi tao MPU6050...");
  imuAvailable = mpu6050_init();

  Serial.println("[BOOT] Kiem tra INA3221...");
  inaAvailable = ina3221_init();

  init_network();

  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 8192, NULL, 1, NULL, 0);

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
