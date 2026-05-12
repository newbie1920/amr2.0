// ============================================================
//   AMR 2.0 — ESP32-S3 Rust Firmware
//   Phase 1: WiFi + WebSocket + Hardware Drivers
// ============================================================

mod config;
mod drivers;
mod comms;
mod core;

use esp_idf_svc::sys::link_patches;
use esp_idf_svc::log::EspLogger;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::uart::{UartConfig, UartDriver};
use esp_idf_hal::units::Hertz;
use esp_idf_hal::pcnt::{PcntUnitDriver, config::UnitConfig as PcntUnitConfig};
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, config::TimerConfig as LedcTimerConfig};
use esp_idf_hal::gpio::PinDriver;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;


use drivers::imu::Mpu6050;
use drivers::oled::Oled;
use drivers::encoder::Encoder;
use drivers::motor::Motor;
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
    pub grid: Arc<Mutex<core::mapper::OccupancyGrid>>,
    pub path: Vec<core::pathfinder::Point>,
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
            grid: Arc::new(Mutex::new(core::mapper::OccupancyGrid::new())),
            path: Vec::new(),
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
            Ok(_) => {
                log::info!("[BOOT] OLED SSD1306 OK!");
                let _ = Oled::draw_status(&mut *i2c_lock, "Connecting WiFi...", 0, "IDLE", state.lock().unwrap().imu_available, 0.0, 0.0);
            }
            Err(e) => log::warn!("[BOOT] OLED not found: {:?}", e),
        }
    }

    // ── WiFi ─────────────────────────────────────────────────
    log::info!("[BOOT] Khoi tao WiFi...");
    // If we enter AP mode, we should update the OLED first
    match connect_wifi(peripherals.modem, &sysloop, &nvs, i2c.clone()) {
        Ok(()) => log::info!("[BOOT] WiFi OK!"),
        Err(e) => log::error!("[BOOT] WiFi FAILED: {:?}", e),
    }

    // ── LiDAR Motor Control (PWM) ────────────────────────────
    // Dùng PWM điều tốc motor RPLidar A1M8 ở mức 60% để đạt tốc độ quét chuẩn 5.5Hz (300 vòng/phút)
    let lidar_timer_config = LedcTimerConfig::new().frequency(25000.into()).resolution(esp_idf_hal::ledc::Resolution::Bits8);
    let lidar_timer = Arc::new(LedcTimerDriver::new(peripherals.ledc.timer2, &lidar_timer_config).unwrap());
    let mut lidar_motor_pwm = LedcDriver::new(peripherals.ledc.channel2, lidar_timer, peripherals.pins.gpio15).unwrap();
    let _ = lidar_motor_pwm.set_duty(150); // Khoảng 60% của 255
    log::info!("[BOOT] LiDAR motor PWM ON (GPIO15, 60% duty)");

    // ── LiDAR UART ───────────────────────────────────────────
    let uart_config = UartConfig::new().baudrate(Hertz(115200));
    let lidar_uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio17, // TX -> Lidar RX
        peripherals.pins.gpio16, // RX <- Lidar TX
        Option::<esp_idf_hal::gpio::AnyIOPin>::None,
        Option::<esp_idf_hal::gpio::AnyIOPin>::None,
        &uart_config,
    ).expect("Failed to init LiDAR UART");
    log::info!("[BOOT] LiDAR UART ready");

    // ── LiDAR Task ───────────────────────────────────────────
    let lidar_state = Arc::clone(&state);
    let _lidar_driver = drivers::lidar::RpLidar;
    let mut lidar_uart = lidar_uart; // Take ownership

    thread::spawn(move || {
        log::info!("[LIDAR_TASK] Bat dau...");
        let _ = drivers::lidar::RpLidar::stop(&mut lidar_uart);
        if let Ok(true) = drivers::lidar::RpLidar::get_device_info(&mut lidar_uart) {
            let _ = drivers::lidar::RpLidar::start_scan(&mut lidar_uart);
            
            loop {
                if let Some(point) = drivers::lidar::RpLidar::read_point(&mut lidar_uart) {
                    if point.distance > 0.0 && point.quality > 10 {
                        let st = lidar_state.lock().unwrap();
                        let mut grid = st.grid.lock().unwrap();
                        
                        // Angle in radians (compensating for robot orientation)
                        let angle_rad = point.angle.to_radians() + st.theta;
                        let range_m = point.distance / 1000.0;
                        
                        grid.cast_ray(st.x, st.y, angle_rad, range_m);
                    }
                }
            }
        }
    });

    // ── Motors & Encoders ────────────────────────────────────
    log::info!("[BOOT] Khoi tao Motors & Encoders...");

    let timer_config = LedcTimerConfig::new().frequency(5000.into()).resolution(esp_idf_hal::ledc::Resolution::Bits8);
    let timer0 = Arc::new(LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config).unwrap());
    let timer1 = Arc::new(LedcTimerDriver::new(peripherals.ledc.timer1, &timer_config).unwrap());

    let motor_l_pwm = LedcDriver::new(peripherals.ledc.channel0, timer0.clone(), peripherals.pins.gpio8).unwrap();
    let motor_r_pwm = LedcDriver::new(peripherals.ledc.channel1, timer1.clone(), peripherals.pins.gpio11).unwrap();

    let motor_l_in1 = PinDriver::output(peripherals.pins.gpio9).unwrap();
    let motor_l_in2 = PinDriver::output(peripherals.pins.gpio10).unwrap();
    let motor_r_in3 = PinDriver::output(peripherals.pins.gpio12).unwrap();
    let motor_r_in4 = PinDriver::output(peripherals.pins.gpio13).unwrap();

    let motor_l = Motor::new(motor_l_pwm, motor_l_in1, motor_l_in2, false);
    let motor_r = Motor::new(motor_r_pwm, motor_r_in3, motor_r_in4, false);

    let pcnt_unit0 = PcntUnitDriver::new(&PcntUnitConfig::default()).unwrap();
    let pcnt_unit1 = PcntUnitDriver::new(&PcntUnitConfig::default()).unwrap();

    let encoder_l = Encoder::new(pcnt_unit0, peripherals.pins.gpio4, peripherals.pins.gpio5, false).unwrap();
    let encoder_r = Encoder::new(pcnt_unit1, peripherals.pins.gpio6, peripherals.pins.gpio7, false).unwrap();

    let control_state = Arc::clone(&state);
    thread::spawn(move || {
        core::control::control_task_loop(control_state, encoder_l, encoder_r, motor_l, motor_r);
    });

    // ── Boot Complete ────────────────────────────────────────
    log::info!("================================================");
    log::info!("  AMR 2.0 — All peripherals initialized!");
    log::info!("  Entering main loop...");
    log::info!("================================================");


    // ── Inflation Task ───────────────────────────────────────
    let inflation_state = Arc::clone(&state);
    thread::spawn(move || {
        log::info!("[INFLATION_TASK] Bat dau...");
        loop {
            thread::sleep(Duration::from_millis(1000));
            {
                let st = inflation_state.lock().unwrap();
                let mut grid = st.grid.lock().unwrap();
                grid.inflate_obstacles();
            }
        }
    });

    // ── Battery ADC ─────────────────────────────────────────────
    let mut battery_monitor = drivers::battery::BatteryMonitor::new(
        peripherals.adc1, 
        peripherals.pins.gpio2
    ).ok();

    // ── Telemetry Loop (main thread) ─────────────────────────
    // In Phase 1, we just print heartbeat to prove boot stability
    let heartbeat_state = Arc::clone(&state);
    let mut last_log_time = std::time::Instant::now();
    loop {
        {
            let mut st = heartbeat_state.lock().unwrap();
            
            if let Some(ref mut batt) = battery_monitor {
                if let Ok(pct) = batt.get_percentage() {
                    st.batt_pct = pct;
                }
            }

            if last_log_time.elapsed() >= Duration::from_secs(5) {
                log::info!(
                    "[HEARTBEAT] pos=({:.2},{:.2}) θ={:.1}° batt={}% nav={} imu={}",
                    st.x, st.y,
                    st.theta * 180.0 / std::f32::consts::PI,
                    st.batt_pct, st.nav_state,
                    if st.imu_available { "OK" } else { "--" }
                );
                last_log_time = std::time::Instant::now();
            }

            if let Ok(mut i2c_lock) = i2c.lock() {
                let speed = (st.vr_meas + st.vl_meas) / 2.0 * config::WHEEL_RADIUS;
                let _ = drivers::oled::Oled::draw_status(
                    &mut *i2c_lock,
                    "AP Mode", // We can put actual IP later
                    st.batt_pct,
                    st.nav_state,
                    st.imu_available,
                    speed,
                    st.theta * 180.0 / std::f32::consts::PI
                );
            }
        }
        thread::sleep(Duration::from_millis(500));
    }
}
