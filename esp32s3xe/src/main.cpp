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

  // Subscribe controlTask to hardware watchdog (5 second timeout)
  esp_task_wdt_add(NULL);

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

    // ── OBSTACLE CHECK TRƯỚC — ưu tiên cao nhất ──────────
    // FIX Bug #2: Di chuyển obstacle check LÊN TRƯỚC navigator
    // để đảm bảo E-STOP xử lý trước khi navigator tính velocity mới.
    if (obstacleDetected && millis() - timeObstacleLastDetected < 500) {
        if (navigator.isNavigating() && navigator.state == NAV_TRACKING) {
            navigator.pause();
            Serial.println("[LIDAR] E-STOP: Phat hien vat can truoc xe!");
        }
        if (targetLeftVel > 0 || targetRightVel > 0) {
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

    // ── AUTONOMOUS NAVIGATOR (chỉ chạy nếu không bị E-STOP) ──
    if (navigator.isNavigating()) {
      navigator.update(robotX, robotY, robotTheta);
      float navV = navigator.cmdLinear;
      float navW = navigator.cmdAngular;
      targetLeftVel = constrain((navV - navW * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
      targetRightVel = constrain((navV + navW * WHEEL_SEPARATION / 2.0f) / WHEEL_RADIUS, -30.0f, 30.0f);
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

    setMotor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, 0, pwmLeft);
    setMotor(MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, 1, pwmRight);

    // Feed watchdog — proves controlTask is alive
    esp_task_wdt_reset();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
  
  // Init Pathfinder (20m x 20m, 0.1m resolution)
  astar.init(200, 200, 0.1f);

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
  Wire.setTimeout(20);

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
      // Serial.printf("[LOOP] IP:%s | Batt:%d%% | IMU:%s | WS:%d | Pos:(%.1f,%.1f) h:%.0f\n",
      //     WiFi.localIP().toString().c_str(),
      //     battPct,
      //     (imuAvailable && gyroCalibrated) ? "OK" : (imuAvailable ? "CAL" : "--"),
      //     webSocket.connectedClients(),
      //     robotX, robotY, robotTheta * 180.0f / PI);
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
