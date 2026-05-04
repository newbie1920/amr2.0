// ============================================================
//   AMR 2.0 — Hardware Pin Map & Constants (Rust)
//   Ported 1:1 from config.h (C++ firmware)
// ============================================================

// ── Motor Driver (L298N) ────────────────────────────────────
pub const MOTOR_LEFT_EN: u8 = 8;
pub const MOTOR_LEFT_IN1: u8 = 9;
pub const MOTOR_LEFT_IN2: u8 = 10;
pub const MOTOR_RIGHT_EN: u8 = 11;
pub const MOTOR_RIGHT_IN3: u8 = 12;
pub const MOTOR_RIGHT_IN4: u8 = 13;

// ── Quadrature Encoders ─────────────────────────────────────
pub const ENCODER_LEFT_A: u8 = 4;
pub const ENCODER_LEFT_B: u8 = 5;
pub const ENCODER_RIGHT_A: u8 = 6;
pub const ENCODER_RIGHT_B: u8 = 7;

// ── I2C (MPU6050 + OLED) ────────────────────────────────────
pub const SDA_PIN: u8 = 39;
pub const SCL_PIN: u8 = 40;

// ── OLED (SSD1306) ──────────────────────────────────────────
pub const SCREEN_WIDTH: u16 = 128;
pub const SCREEN_HEIGHT: u16 = 64;
pub const SCREEN_ADDRESS: u8 = 0x3C;

// ── Battery ADC ─────────────────────────────────────────────
pub const BATT_PIN: u8 = 2;
pub const BATT_SCALE_FACTOR: f32 = 2.0;
pub const BATT_MIN_V: f32 = 9.9;
pub const BATT_MAX_V: f32 = 12.6;

// ── LiDAR A1M8 ─────────────────────────────────────────────
pub const LIDAR_RX_PIN: u8 = 16;
pub const LIDAR_TX_PIN: u8 = 17;
pub const LIDAR_PWM_PIN: u8 = 15;

// ── NeoPixel ────────────────────────────────────────────────
pub const RGB_BUILTIN_PIN: u8 = 48;

// ── Robot Kinematics ────────────────────────────────────────
pub const WHEEL_RADIUS: f32 = 0.0264;      // meters
pub const WHEEL_SEPARATION: f32 = 0.170;   // meters
pub const TICKS_PER_REV: u32 = 1665;

// ── PID / Control ───────────────────────────────────────────
pub const FF_GAIN_LEFT: f32 = 24.0;
pub const FF_GAIN_RIGHT: f32 = 32.0;
pub const MIN_PWM: f32 = 50.0;
pub const KP_VEL: f32 = 2.0;
pub const KI_VEL: f32 = 1.5;
pub const CMD_TIMEOUT_MS: u64 = 1000;
pub const CONTROL_FREQ_HZ: u32 = 50;
pub const TELEMETRY_INTERVAL_MS: u64 = 100;

// ── IMU Fusion ──────────────────────────────────────────────
pub const COMP_FILTER_ALPHA: f32 = 0.95;
pub const GYRO_CAL_COUNT: u32 = 500;

// ── WiFi ────────────────────────────────────────────────────
pub const WIFI_AP_NAME: &str = "AMR_S3_AP";
pub const WEBSOCKET_PORT: u16 = 81;
pub const HTTP_PORT: u16 = 80;

// ── Motor Inversion ─────────────────────────────────────────
pub const INVERT_LEFT_ENCODER: bool = false;
pub const INVERT_RIGHT_ENCODER: bool = false;
pub const INVERT_LEFT_MOTOR: bool = true;
pub const INVERT_RIGHT_MOTOR: bool = false;

// ── INA3221 ─────────────────────────────────────────────────
pub const INA_CH_BATT: usize = 1;
pub const INA_CH_MOTOR: usize = 2;
