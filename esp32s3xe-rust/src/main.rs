// ============================================================
//   AMR 2.0 — ESP32-S3 Rust Firmware
//   Phase 1: WiFi + WebSocket + Hardware Drivers
// ============================================================

mod config;
mod drivers;
mod comms;

use esp_idf_svc::sys::link_patches;
use esp_idf_svc::log::EspLogger;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::uart::{UartConfig, UartDriver};
use esp_idf_hal::units::Hertz;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use config::*;
use drivers::imu::Mpu6050;
use drivers::oled::Oled;
use comms::wifi::connect_wifi;

/// Shared robot state accessible by all tasks
pub struct RobotState {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub distance: f32,
    pub vl_meas: f32,
    pub vr_meas: f32,
    pub target_vl: f32,
    pub target_vr: f32,
    pub gyro_z: f32,
    pub gyro_calibrated: bool,
    pub imu_available: bool,
    pub brake: bool,
    pub obstacle_detected: bool,
    pub obstacle_time: u64,
    pub batt_pct: u8,
    pub nav_state: &'static str,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            x: 0.0, y: 0.0, theta: 0.0, distance: 0.0,
            vl_meas: 0.0, vr_meas: 0.0,
            target_vl: 0.0, target_vr: 0.0,
            gyro_z: 0.0, gyro_calibrated: false,
            imu_available: false, brake: false,
            obstacle_detected: false, obstacle_time: 0,
            batt_pct: 0, nav_state: "IDLE",
        }
    }
}

fn main() {
    link_patches();
    EspLogger::initialize_default();

    log::info!("========================================");
    log::info!("  AMR 2.0 FIRMWARE — ESP32-S3 (Rust)");
    log::info!("  Phase 1: Boot + WiFi + WebSocket");
    log::info!("========================================");

    // ── Peripherals ──────────────────────────────────────────
    let peripherals = Peripherals::take().expect("Failed to take peripherals");
    let sysloop = EspSystemEventLoop::take().expect("Failed to take event loop");
    let nvs = EspDefaultNvsPartition::take().expect("Failed to take NVS partition");

    // ── Shared State ─────────────────────────────────────────
    let state = Arc::new(Mutex::new(RobotState::default()));

    // ── WiFi ─────────────────────────────────────────────────
    log::info!("[BOOT] Khoi tao WiFi...");
    match connect_wifi(peripherals.modem, &sysloop, &nvs) {
        Ok(()) => log::info!("[BOOT] WiFi OK!"),
        Err(e) => log::error!("[BOOT] WiFi FAILED: {:?}", e),
    }

    // ── I2C Bus (shared: MPU6050 + OLED) ─────────────────────
    let i2c_config = I2cConfig::new().baudrate(Hertz(400_000));
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio39, // SDA
        peripherals.pins.gpio40, // SCL
        &i2c_config,
    ).expect("Failed to init I2C");
    let i2c = Arc::new(Mutex::new(i2c));

    // ── IMU (MPU6050) ────────────────────────────────────────
    {
        let mut i2c_lock = i2c.lock().unwrap();
        match Mpu6050::init(&mut *i2c_lock) {
            Ok(_) => {
                log::info!("[BOOT] MPU6050 OK!");
                state.lock().unwrap().imu_available = true;
            }
            Err(e) => log::warn!("[BOOT] MPU6050 not found: {:?}", e),
        }
    }

    // ── OLED (SSD1306) ───────────────────────────────────────
    {
        let mut i2c_lock = i2c.lock().unwrap();
        match Oled::init(&mut *i2c_lock) {
            Ok(_) => log::info!("[BOOT] OLED SSD1306 OK!"),
            Err(e) => log::warn!("[BOOT] OLED not found: {:?}", e),
        }
    }

    // ── LiDAR UART ───────────────────────────────────────────
    log::info!("[BOOT] Khoi tao LiDAR UART (pins {}/{})", LIDAR_RX_PIN, LIDAR_TX_PIN);
    let uart_config = UartConfig::new().baudrate(Hertz(115200));
    let _lidar_uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio17, // TX -> Lidar RX
        peripherals.pins.gpio16, // RX <- Lidar TX
        Option::<esp_idf_hal::gpio::AnyIOPin>::None,
        Option::<esp_idf_hal::gpio::AnyIOPin>::None,
        &uart_config,
    ).expect("Failed to init LiDAR UART");
    log::info!("[BOOT] LiDAR UART ready");

    // ── Motor PWM Setup ──────────────────────────────────────
    log::info!("[BOOT] Khoi tao Motor PWM...");
    // Motors will be initialized in the control task

    // ── Boot Complete ────────────────────────────────────────
    log::info!("================================================");
    log::info!("  AMR 2.0 — All peripherals initialized!");
    log::info!("  Entering main loop...");
    log::info!("================================================");

    // ── Telemetry Loop (main thread) ─────────────────────────
    // In Phase 1, we just print heartbeat to prove boot stability
    let heartbeat_state = Arc::clone(&state);
    loop {
        {
            let st = heartbeat_state.lock().unwrap();
            log::info!(
                "[HEARTBEAT] pos=({:.2},{:.2}) θ={:.1}° batt={}% nav={} imu={}",
                st.x, st.y,
                st.theta * 180.0 / std::f32::consts::PI,
                st.batt_pct, st.nav_state,
                if st.imu_available { "OK" } else { "--" }
            );
        }
        thread::sleep(Duration::from_secs(5));
    }
}
