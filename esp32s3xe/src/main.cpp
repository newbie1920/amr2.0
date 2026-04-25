// ============================================================
//   AMR 2.0 — ESP32-S3 Firmware
//   WebSocket + PID Motor Control + IMU Fusion
//   Kế thừa từ amrs3_uart, bỏ LiDAR/OLED/OTA
// ============================================================

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RPLidar.h>
#include "config.h"
#include "navigator.h"
#include "wheel_pid.h"
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel rgbLed(1, RGB_BUILTIN_PIN, NEO_GRB + NEO_KHZ800);

// ─── MPU6050 & OLED ──────────────────────────────────────────
#define MPU6050_ADDR 0x68
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SemaphoreHandle_t i2cMutex;


// ─── GLOBAL STATE ────────────────────────────────────────────
// Encoders
volatile long leftTicks = 0, rightTicks = 0;
volatile int lastEncodedLeft = 0, lastEncodedRight = 0;

// Control targets (set by WebSocket commands)
float targetLeftVel = 0, targetRightVel = 0;
unsigned long lastCmdTime = 0;

// Motor control
WheelPID* leftPID;
WheelPID* rightPID;
float lastPwmLeft = 0, lastPwmRight = 0;

// Velocity measurement (low-pass filtered)
float vL_meas = 0, vR_meas = 0;

// IMU Fusion
float gyroZBias = 0;
bool gyroCalibrated = false;
int gyroCalSamples = 0;
float gyroCalSum = 0;
float gyroZ_raw = 0;
float gyroTheta = 0, encoderTheta = 0, fusedTheta = 0;
bool imuAvailable = false;
bool brakeEnabled = false;

// INA3221 (Power Monitor)
bool inaAvailable = false;
float ina_busV[3] = {0, 0, 0};
float ina_currentA[3] = {0, 0, 0};

// Autonomous Navigator
Navigator navigator;

// Odometry
float robotX = 2.5, robotY = 9.0, robotTheta = -PI/2;
float robotDistance = 0;
float filteredVBatt = 12.0f;

// ── Lidar A1M8 ──────────────────────────────────────────────
HardwareSerial lidarSerial(1);
RPLidar lidar;
uint16_t lidarDists[360] = {0}; // Lưu khoảng cách (mm) theo từng độ
bool lidarRunning = false;
bool obstacleDetected = false;
unsigned long timeObstacleLastDetected = 0;
// ────────────────────────────────────────────────────────────

// Timing
long lastTicksL = 0, lastTicksR = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastOledTime = 0;
#define OLED_INTERVAL 1000

// Network
WebServer server(HTTP_PORT);
WebSocketsServer webSocket(WEBSOCKET_PORT);
WiFiManager wm;

// ============================================================
//   MPU6050 BARE-METAL FUNCTIONS
// ============================================================
void mpu6050_writeReg(uint8_t reg, uint8_t val) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  xSemaphoreGive(i2cMutex);
}

int16_t mpu6050_readReg16(uint8_t reg) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    xSemaphoreGive(i2cMutex);
    return 0;
  }
  uint8_t rcv = Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
  if (rcv < 2) {
    Wire.end();
    delay(2);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    xSemaphoreGive(i2cMutex);
    return 0;
  }
  int16_t res = (Wire.read() << 8) | Wire.read();
  xSemaphoreGive(i2cMutex);
  return res;
}

bool mpu6050_init() {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);

  if (Wire.available() < 1) {
    Serial.println("[IMU] MPU6050 KHONG TIM THAY!");
    xSemaphoreGive(i2cMutex);
    return false;
  }

  uint8_t whoAmI = Wire.read();
  xSemaphoreGive(i2cMutex);
  Serial.printf("[IMU] WHO_AM_I: 0x%02X\n", whoAmI);

  mpu6050_writeReg(0x6B, 0x00); // Wake up
  delay(100);
  mpu6050_writeReg(0x1B, 0x00); // ±250°/s
  mpu6050_writeReg(0x1A, 0x06); // DLPF 5Hz
  mpu6050_writeReg(0x19, 0x04); // 200Hz sample rate

  Serial.println("[IMU] MPU6050 OK (±250°/s, DLPF=5Hz, 200Hz)");
  return true;
}

float mpu6050_readGyroZ() {
  int16_t raw = mpu6050_readReg16(0x47);
  return (raw / 131.0f) * (PI / 180.0f); // rad/s
}

void mpu6050_calibrate(float rawZ) {
  gyroCalSum += rawZ;
  gyroCalSamples++;
  if (gyroCalSamples >= GYRO_CAL_COUNT) {
    gyroZBias = gyroCalSum / (float)gyroCalSamples;
    gyroCalibrated = true;
    Serial.printf("[IMU] Gyro calibrated. Bias: %.6f rad/s\n", gyroZBias);
  }
}

// ============================================================
//   INA3221 BARE-METAL FUNCTIONS
// ============================================================
#define INA3221_ADDR 0x40

int16_t ina3221_readReg(uint8_t reg) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(INA3221_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    xSemaphoreGive(i2cMutex);
    return 0;
  }
  Wire.requestFrom((uint8_t)INA3221_ADDR, (uint8_t)2);
  if (Wire.available() < 2) {
    xSemaphoreGive(i2cMutex);
    return 0;
  }
  int16_t res = (Wire.read() << 8) | Wire.read();
  xSemaphoreGive(i2cMutex);
  return res;
}

void read_ina3221() {
  if (!inaAvailable) return;
  for (int ch = 1; ch <= 3; ch++) {
    int16_t rawBus = ina3221_readReg(2 + (ch - 1) * 2);
    int16_t rawShunt = ina3221_readReg(1 + (ch - 1) * 2);
    // Shift right 3 bits, LSB = 8mV
    float voltage = (rawBus >> 3) * 0.008f;
    if (voltage > 1.0f) {
        voltage -= 0.85f; // Bù trừ sai số của mạch INA3221 (so với đồng hồ đo thực tế)
    }
    ina_busV[ch - 1] = voltage;

    // Shift right 3 bits, LSB = 40uV, Shunt = 0.1Ohm -> I(A) = Vshunt / 0.1 = Vshunt * 10
    // => Current = (raw >> 3) * 0.00004 * 10 = (raw >> 3) * 0.0004
    ina_currentA[ch - 1] = (rawShunt >> 3) * 0.0004f; 
  }

  // Debug INA3221 mỗi 5 giây
  static unsigned long lastInaDebug = 0;
  if (millis() - lastInaDebug > 5000) {
    lastInaDebug = millis();
    Serial.printf("[INA3221] CH1: %.2fV %.3fA | CH2: %.2fV %.3fA | CH3: %.2fV %.3fA\n",
                  ina_busV[0], ina_currentA[0],
                  ina_busV[1], ina_currentA[1],
                  ina_busV[2], ina_currentA[2]);
  }
}

// ============================================================
//   ENCODER ISRs
// ============================================================
void IRAM_ATTR leftISR() {
  int MSB = digitalRead(ENCODER_LEFT_A);
  int LSB = digitalRead(ENCODER_LEFT_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedLeft << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    INVERT_LEFT_ENCODER ? leftTicks-- : leftTicks++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    INVERT_LEFT_ENCODER ? leftTicks++ : leftTicks--;
  lastEncodedLeft = encoded;
}

void IRAM_ATTR rightISR() {
  int MSB = digitalRead(ENCODER_RIGHT_A);
  int LSB = digitalRead(ENCODER_RIGHT_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedRight << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    INVERT_RIGHT_ENCODER ? rightTicks-- : rightTicks++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    INVERT_RIGHT_ENCODER ? rightTicks++ : rightTicks--;
  lastEncodedRight = encoded;
}

// ============================================================
//   MOTOR CONTROL
// ============================================================
void setMotor(int pinIN1, int pinIN2, int pwmCh, float u) {
  int pwr = (int)fabs(u);
  if (pwr > 255) pwr = 255;

  if (u > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else if (u < 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  } else {
    if (brakeEnabled) {
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, HIGH);
      pwr = 255;
    } else {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      pwr = 0;
    }
  }
  ledcWrite(pwmCh, pwr);
}

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
        // Force stop if not navigating but manually driving with positive velocity
        if (targetL > 0 || targetR > 0) {
            targetLeftVel = 0;
            targetRightVel = 0;
        }
    } else if (obstacleDetected && millis() - timeObstacleLastDetected >= 500) {
        // Clear obstacle
        obstacleDetected = false;
        if (navigator.isNavigating() && navigator.state == NAV_PAUSED) {
            navigator.resume();
            Serial.println("[LIDAR] Vat can da di chuyen, RESUME!");
        }
    }

    // Hard real-time sleep
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
        // Lưu khoảng cách vào mảng 360 độ (làm tròn độ)
        int deg = (int)round(angle) % 360;
        lidarDists[deg] = (uint16_t)distance;

        // Xử lý E-STOP: Kiểm tra góc phía trước [-30 đến 30 độ tương ứng 330->359 và 0->30]
        if ((deg <= 30 || deg >= 330) && distance > 50 && distance < 450) {
            obstacleDetected = true;
            timeObstacleLastDetected = millis();
        }
      }
    } else {
      // Mất kết nối hoặc quay quá chậm, restart
      analogWrite(LIDAR_PWM_PIN, 0); 
      vTaskDelay(pdMS_TO_TICKS(100));
      lidar.begin(lidarSerial);
      // Chạy lại động cơ Lidar (PWM ≈ 60% vòng tua 5.5Hz)
      analogWrite(LIDAR_PWM_PIN, 150); 
    }
  }
}

// ============================================================
//   SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  printf("\n\n[BOOT] Bat dau setup()...\n");

  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(MOTOR_LEFT_EN, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MOTOR_RIGHT_EN, 1);
  ledcWrite(0, 0);
  ledcWrite(1, 0);

  // Initialize Onboard RGB LED (dim and soft color)
  rgbLed.begin();
  rgbLed.setBrightness(15); // giảm độ sáng (15/255)
  rgbLed.setPixelColor(0, rgbLed.Color(30, 80, 150)); // màu xanh sương mai dịu (sky blue/cyan)
  rgbLed.show();

  // Initialize Lidar
  pinMode(LIDAR_PWM_PIN, OUTPUT);
  lidar.begin(lidarSerial);
  // Fix RPLidar library bug: it calls begin() natively, overwriting our RX/TX pins.
  // We MUST call lidarSerial.begin again with our custom pins!
  lidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  
  // RPM Lidar (5.5Hz tiêu chuẩn)
  analogWrite(LIDAR_PWM_PIN, 150); 

  leftPID = new WheelPID(KP_VEL, KI_VEL, 0.0f, FF_GAIN_LEFT, 1.0f / CONTROL_FREQ_HZ, 5.0f, MIN_PWM);
  rightPID = new WheelPID(KP_VEL, KI_VEL, 0.0f, FF_GAIN_RIGHT, 1.0f / CONTROL_FREQ_HZ, 5.0f, MIN_PWM);

  analogSetPinAttenuation(BATT_PIN, ADC_11db);
  pinMode(BATT_PIN, INPUT);

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), rightISR, CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeout(20);

  i2cMutex = xSemaphoreCreateMutex();
  
  if(i2cMutex != NULL) {
      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("AMR 2.0 Booting...");
        display.display();
      }
      xSemaphoreGive(i2cMutex);
  }

  printf("[BOOT] Khoi tao MPU6050...\n");
  imuAvailable = mpu6050_init();

  printf("[BOOT] Kiem tra INA3221...\n");
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(INA3221_ADDR);
  if (Wire.endTransmission() == 0) {
      inaAvailable = true;
      Serial.println("[BOOT] INA3221 OK (0x40)");
  } else {
      Serial.println("[BOOT] INA3221 KHONG Thay!");
  }
  xSemaphoreGive(i2cMutex);

  printf("[BOOT] Khoi tao WiFi...\n");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  wm.setConnectTimeout(15);
  wm.setConfigPortalTimeout(120);
  
  printf("[BOOT] Chay WiFiManager autoConnect...\n");
  if (!wm.autoConnect(WIFI_AP_NAME)) {
    printf("[WIFI] Ket noi that bai hoac Timeout Portal!\n");
  }
  printf("[BOOT] WiFi autoConnect xong, vong lap Loop bat dau! IP: %s\n", WiFi.localIP().toString().c_str());
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
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type != WStype_TEXT) return;
    JsonDocument doc;
    deserializeJson(doc, payload);

    if (doc["type"] == "ping") {
      JsonDocument pong;
      pong["type"] = "pong";
      pong["ts"] = doc["ts"];
      static char outBuf[128];
      serializeJson(pong, outBuf, sizeof(outBuf));
      webSocket.sendTXT(num, outBuf);
      return;
    }

    if (doc["cmd"] == "reset_odom") {
      robotX = 2.5; robotY = 9.0; robotTheta = -PI/2; robotDistance = 0;
      leftTicks = rightTicks = lastTicksL = lastTicksR = 0;
      targetLeftVel = targetRightVel = 0;
      gyroTheta = encoderTheta = fusedTheta = robotTheta;
      leftPID->reset();
      rightPID->reset();
      Serial.println("[CMD] Odometry reset.");
    }

    if (doc["cmd"] == "set_pose") {
      if (!doc["x"].isNull()) robotX = doc["x"];
      if (!doc["y"].isNull()) robotY = doc["y"];
      if (!doc["theta"].isNull()) robotTheta = doc["theta"];
      gyroTheta = encoderTheta = fusedTheta = robotTheta;
    }

    if (doc["cmd"] == "navigate") {
      JsonArray pathArr = doc["path"].as<JsonArray>();
      int count = pathArr.size();
      if (count > 0 && count <= MAX_WAYPOINTS) {
        navigator.waypointCount = count;
        navigator.currentWpIdx = 0;
        for (int i = 0; i < count; i++) {
          navigator.waypoints[i].x = pathArr[i]["x"];
          navigator.waypoints[i].y = pathArr[i]["y"];
          navigator.waypoints[i].heading = NAN;
          navigator.waypoints[i].useReverse = false;
        }
        float endH = NAN;
        if (!doc["finalHeading"].isNull()) {
          endH = doc["finalHeading"].as<float>() * PI / 180.0f;
        }
        navigator.finalHeading = endH;
        
        navigator.state = NAV_TRACKING;
        navigator.cmdLinear = 0;
        navigator.cmdAngular = 0;
        navigator.navStartTime = millis();
        navigator.lastWpReachTime = millis();
        
        JsonDocument ack;
        ack["type"] = "nav_ack";
        ack["wp_count"] = count;
        ack["finalH"] = isnan(endH) ? -1 : (int)(endH * 180.0f / PI);
        static char outBuf[128];
        serializeJson(ack, outBuf, sizeof(outBuf));
        webSocket.sendTXT(num, outBuf);
      }
    }
    
    if (doc["cmd"] == "nav_stop") {
      navigator.abort();
    }

    if (doc["cmd"] == "pause") {
      navigator.pause();
    }

    if (doc["cmd"] == "resume") {
      navigator.resume();
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
      float v = doc["linear"];
      float w = doc["angular"];
      targetLeftVel = constrain((v - w * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
      targetRightVel = constrain((v + w * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
      lastCmdTime = millis();
    }
  });

  server.begin();

  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 1, NULL, 0);

  Serial.println("================================================");
  Serial.println("  AMR 2.0 FIRMWARE - ESP32-S3 N16R8             ");
  Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("  IMU: %s\n", imuAvailable ? "MPU6050 OK" : "KHONG CO");
  Serial.printf("  INA3221: %s\n", inaAvailable ? "OK" : "KHONG CO");
  Serial.printf("  WebSocket port: %d\n", WEBSOCKET_PORT);
  Serial.println("================================================");
}


// ============================================================
//   MAIN LOOP - Network, Telemetry & Display Only
// ============================================================
void loop() {
  webSocket.loop();
  server.handleClient();
  
  static unsigned long lastPrint = 0;
  if(millis() - lastPrint > 5000) {
      lastPrint = millis();
      printf("[LOOP] AMR dang chay... IP: %s\n", WiFi.localIP().toString().c_str());
  }
  ArduinoOTA.handle();

  if (!navigator.isNavigating() && millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    targetLeftVel = 0;
    targetRightVel = 0;
  }

  static unsigned long lastWifiCheck = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    Serial.println("[WIFI] Mat ket noi! Dang thu reconnect...");
    WiFi.reconnect();
  }

  if (brakeEnabled) {
    targetLeftVel = 0;
    targetRightVel = 0;
    if (navigator.isNavigating()) {
        navigator.abort();
    }
  }

  // ── Telemetry Broadcast (30Hz) ───────────────────────
  if (millis() - lastTelemetryTime > TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();

    // Read Power Monitor
    read_ina3221();

    long b_sum = 0;
    for (int i = 0; i < 20; i++) b_sum += analogRead(BATT_PIN);
    
    // Nếu có INA3221, dùng điện áp kênh 1 thay cho ADC ảo
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

        // Đóng gói thêm object power chi tiết để Frontend dễ vẽ biểu đồ
        JsonObject pwr = telem["power"].to<JsonObject>();
        pwr["battV"] = ina_busV[INA_CH_BATT];
        pwr["battA"] = ina_currentA[INA_CH_BATT];
        pwr["motorV"] = ina_busV[INA_CH_MOTOR];
        pwr["motorA"] = ina_currentA[INA_CH_MOTOR];
    }

    // Lidar payload culling
    JsonArray ls = telem["lidar"].to<JsonArray>();
    bool hasObstruction = obstacleDetected && millis() - timeObstacleLastDetected < 500;
    telem["obs"] = hasObstruction;
    for (int i = 0; i < 360; i += 2) { // Giảm băng thông lấy độ cách 2 (180 điểm)
      if (lidarDists[i] > 0 && lidarDists[i] < 3000) {
        // Format object gộp: { a: angle, d: distance }
        JsonObject p = ls.add<JsonObject>();
        p["a"] = i;             // Angle 0-359
        p["d"] = lidarDists[i]; // Distance in mm
      }
    }

    static char telemBuf[2048]; // Tăng buffer size để chứa Lidar Data
    size_t len = serializeJson(telem, telemBuf, sizeof(telemBuf));
    webSocket.broadcastTXT(telemBuf, len);
    
    Serial.printf("[AMR2] V:%.1f,%.1f IMU:%s Bat:%d%% X:%.2f Y:%.2f H:%.1f OBS:%s\n",
                  vL_meas, vR_meas,
                  imuAvailable ? "OK" : "--", battPct,
                  robotX, robotY, robotTheta * 180.0f / PI,
                  hasObstruction ? "YES" : "NO");
  }

  // ── OLED Update (1Hz) ──────────────────────────────────
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
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        display.display();
        xSemaphoreGive(i2cMutex);
    }
  }
}
