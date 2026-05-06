/**
 * AMR 2.0 — ESP32-S3 Firmware Configuration
 * Kế thừa pin map từ amrs3_uart
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//   SIMULATION CONFIG
// ============================================================
#define SIMULATION_MODE true

// ============================================================
//   PIN MAP — ESP32-S3 N16R8
//   (Giữ nguyên từ amrs3_uart)
// ============================================================

// Motor Driver (L298N)
#define MOTOR_LEFT_EN 8 // PWM
#define MOTOR_LEFT_IN1 9
#define MOTOR_LEFT_IN2 10
#define MOTOR_RIGHT_EN 11 // PWM
#define MOTOR_RIGHT_IN3 12
#define MOTOR_RIGHT_IN4 13

// Quadrature Encoders
#define ENCODER_LEFT_A 4
#define ENCODER_LEFT_B 5
#define ENCODER_RIGHT_A 6
#define ENCODER_RIGHT_B 7

// I2C — IMU (MPU6050) & OLED
#define SDA_PIN 39
#define SCL_PIN 40

// OLED (SSD1306)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Battery ADC (ADC1 — an toàn khi WiFi bật)
#define BATT_PIN 2

// ============================================================
//   LIDAR A1M8
// ============================================================
#define LIDAR_RX_PIN 16  // Gắn vào TX của Lidar
#define LIDAR_TX_PIN 17  // Gắn vào RX của Lidar
#define LIDAR_PWM_PIN 15 // Gắn vào chân quay Motor Lidar

// ============================================================
//   NEOPIXEL / RGB LED (Onboard ESP32-S3)
// ============================================================
#define RGB_BUILTIN_PIN 48

// ============================================================
//   ROBOT KINEMATICS
// ============================================================

#define WHEEL_RADIUS 0.0264f    // Mét (26.4mm)
#define WHEEL_SEPARATION 0.170f // Mét (170mm — khoảng cách 2 bánh)
#define TICKS_PER_REV 1665      // Encoder ticks per revolution

// ============================================================
//   PID / CONTROL PARAMETERS
// ============================================================

// Feedforward gains (bù motor)
#define FF_GAIN_LEFT 24.0f
#define FF_GAIN_RIGHT 32.0f
#define MIN_PWM 50

// PI velocity controller
#define KP_VEL 2.0f
#define KI_VEL 1.5f

// Safety
#define CMD_TIMEOUT_MS 1000 // Dừng motor nếu mất lệnh (ms)
#define CONTROL_FREQ_HZ 50  // 50Hz control loop (20ms)
#define TELEMETRY_INTERVAL                                                     \
  100 // ~10Hz telemetry broadcast — giảm tải WiFi cho lidar data

// ============================================================
//   IMU FUSION
// ============================================================

#define COMP_FILTER_ALPHA                                                      \
  0.95f                    // Complementary filter (0=encoder only, 1=gyro only)
#define GYRO_CAL_COUNT 500 // Số mẫu calibrate gyro

// ============================================================
//   BATTERY
// ============================================================

#define BATT_SCALE_FACTOR 2.0f // Tỉ lệ phân áp
#define BATT_OFFSET 0.0f
#define BATT_MIN_V 9.9f  // Pin cạn (3S LiPo: 3.3V × 3)
#define BATT_MAX_V 12.6f // Pin đầy (3S LiPo: 4.2V × 3)

// ============================================================
//   WIFI
// ============================================================

#define WIFI_AP_NAME "AMR_S3_AP"
#define WEBSOCKET_PORT 81
#define HTTP_PORT 80

// ============================================================
//   MOTOR INVERSION (Đảo chiều nếu cần)
// ============================================================

#define INVERT_LEFT_ENCODER false
#define INVERT_RIGHT_ENCODER false
#define INVERT_LEFT_MOTOR true
#define INVERT_RIGHT_MOTOR false

// ============================================================
//   INA3221 CHANNEL MAPPING (Đã chốt)
// ============================================================

#define INA_CH_BATT                                                            \
  1 // Kênh 2 (CH2): Đo Pin tổng (áp & dòng) - Thay thế cho CH1
#define INA_CH_MOTOR 2 // Kênh 3 (CH3): Đo đường nguồn 12V vào L298N
// Đã bỏ chức năng đo mạch 5V vì INA3221 chỉ còn 2 kênh sống.

#endif // CONFIG_H
