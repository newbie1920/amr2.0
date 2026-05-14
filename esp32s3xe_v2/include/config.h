/**
 * AMR 2.0 v2 — Hardware Configuration
 * Pin map giữ nguyên từ esp32s3xe v1
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//   BUILD OPTIONS
// ============================================================
#define SIMULATION_MODE   false   // true = SimTask replaces real hardware
#define ENABLE_CSM        true    // Correlative Scan Matching (extra SLAM accuracy)

// ============================================================
//   PIN MAP — ESP32-S3 N16R8
// ============================================================

// Motor Driver (L298N)
#define MOTOR_LEFT_EN   8   // PWM
#define MOTOR_LEFT_IN1  9
#define MOTOR_LEFT_IN2  10
#define MOTOR_RIGHT_EN  11  // PWM
#define MOTOR_RIGHT_IN3 12
#define MOTOR_RIGHT_IN4 13

// Quadrature Encoders
#define ENCODER_LEFT_A  4
#define ENCODER_LEFT_B  5
#define ENCODER_RIGHT_A 6
#define ENCODER_RIGHT_B 7

// I2C — IMU (MPU6050) & OLED & INA3221
#define SDA_PIN 39
#define SCL_PIN 40

// OLED (SSD1306)
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C

// Battery ADC (ADC1 — safe with WiFi)
#define BATT_PIN 2

// LiDAR A1M8 (UART1)
#define LIDAR_RX_PIN  18  // ESP32 RX ← Lidar TX
#define LIDAR_TX_PIN  3   // ESP32 TX → Lidar RX
#define LIDAR_PWM_PIN 15  // Motor PWM

// NeoPixel RGB LED (Onboard)
#define RGB_BUILTIN_PIN 48

// ============================================================
//   ROBOT KINEMATICS
// ============================================================
#define WHEEL_RADIUS      0.0264f   // m (26.4mm)
#define WHEEL_SEPARATION  0.170f    // m (170mm)
#define TICKS_PER_REV     1665      // Encoder CPR

// ============================================================
//   PID / CONTROL
// ============================================================
#define FF_GAIN_LEFT   24.0f
#define FF_GAIN_RIGHT  32.0f
#define MIN_PWM        50
#define KP_VEL         2.0f
#define KI_VEL         1.5f
#define CMD_TIMEOUT_MS    1000
#define CONTROL_FREQ_HZ   50     // 50Hz control loop
#define TELEMETRY_INTERVAL 50    // ~20Hz telemetry (fast real-time)

// ============================================================
//   IMU FUSION
// ============================================================
#define COMP_FILTER_ALPHA 0.95f  // 0=encoder, 1=gyro
#define GYRO_CAL_COUNT    500

// ============================================================
//   BATTERY
// ============================================================
#define BATT_SCALE_FACTOR 4.27f  // Voltage divider ratio (12.6V → 2.95V ADC)
#define BATT_OFFSET       0.15f  // ADC diode drop compensation
#define BATT_MIN_V        9.9f   // 3S LiPo empty
#define BATT_MAX_V        12.6f  // 3S LiPo full

// ============================================================
//   WIFI / WEBSOCKET
// ============================================================
#define WIFI_AP_NAME    "AMR_S3_AP"
#define WEBSOCKET_PORT  81
#define HTTP_PORT       80

// ============================================================
//   MQTT AUTO-DISCOVERY
// ============================================================
#define MQTT_BROKER       "broker.hivemq.com"
#define MQTT_PORT         1883
#define MQTT_TOPIC_PREFIX "amr2/discovery"
#define MQTT_HEARTBEAT_MS 30000
#define MQTT_ROBOT_PREFIX "AMR2_"

// ============================================================
//   MOTOR INVERSION
// ============================================================
#define INVERT_LEFT_ENCODER   false
#define INVERT_RIGHT_ENCODER  false
#define INVERT_LEFT_MOTOR     true
#define INVERT_RIGHT_MOTOR    false

// ============================================================
//   INA3221 CHANNEL MAPPING
// ============================================================
#define INA_CH_BATT   1  // CH2: Battery total
#define INA_CH_MOTOR  2  // CH3: 12V motor supply

// ============================================================
//   SLAM / MAPPING
// ============================================================
#define GRID_SIZE       1024
#define GRID_RESOLUTION 0.05f  // 5cm per cell
#define GRID_UPDATE_INTERVAL_MS 200
#define CSM_MIN_SCANS   10

// ============================================================
//   NAVIGATION
// ============================================================
#define MAX_WAYPOINTS       100
#define DWA_INTERVAL_MS     100   // 10Hz DWA rate
#define NAV_WP_TIMEOUT_MS   30000 // 30s per waypoint

#endif // CONFIG_H
