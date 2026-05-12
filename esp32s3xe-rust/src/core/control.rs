use std::sync::{Arc, Mutex};
use std::time::Duration;
use crate::RobotState;
use crate::drivers::encoder::Encoder;
use crate::drivers::motor::Motor;
use crate::core::pid::WheelPid;
use crate::config;

pub fn control_task_loop(
    state: Arc<Mutex<RobotState>>,
    encoder_l: Encoder<'static>,
    encoder_r: Encoder<'static>,
    mut motor_l: Motor<'static>,
    mut motor_r: Motor<'static>,
) {
    let mut pid_l = WheelPid::new(2.0, 0.5, 0.1, 255.0);
    let mut pid_r = WheelPid::new(2.0, 0.5, 0.1, 255.0);
    
    let dt = 1.0 / 50.0; // 50 Hz control loop
    let mut last_ticks_l = 0;
    let mut last_ticks_r = 0;

    log::info!("[CONTROL] Started Control Task");

    loop {
        std::thread::sleep(Duration::from_millis(20));

        let mut st = state.lock().unwrap();

        // Read Encoders
        let ticks_l = encoder_l.get_count().unwrap_or(0);
        let ticks_r = encoder_r.get_count().unwrap_or(0);

        let v_l_raw = ((ticks_l - last_ticks_l) as f32 / config::TICKS_PER_REV as f32) * 2.0 * std::f32::consts::PI / dt;
        let v_r_raw = ((ticks_r - last_ticks_r) as f32 / config::TICKS_PER_REV as f32) * 2.0 * std::f32::consts::PI / dt;

        st.vl_meas = 0.7 * st.vl_meas + 0.3 * v_l_raw;
        st.vr_meas = 0.7 * st.vr_meas + 0.3 * v_r_raw;

        last_ticks_l = ticks_l;
        last_ticks_r = ticks_r;

        // Kinematics
        let v_robot = (st.vr_meas + st.vl_meas) / 2.0 * config::WHEEL_RADIUS;
        let w_encoder = (st.vr_meas - st.vl_meas) * config::WHEEL_RADIUS / config::WHEEL_SEPARATION;

        // Simplified Odometry without IMU for now
        let dist = v_robot * dt;
        st.distance += dist.abs();
        st.theta += w_encoder * dt;
        st.theta = st.theta.sin().atan2(st.theta.cos());
        
        st.x += dist * st.theta.cos();
        st.y += dist * st.theta.sin();

        // Target velocities (could be updated by navigation)
        let target_l = st.target_vl;
        let target_r = st.target_vr;

        // Update PID
        let pwm_l = pid_l.update(st.vl_meas, target_l, dt);
        let pwm_r = pid_r.update(st.vr_meas, target_r, dt);

        let brake = st.brake;
        let _ = motor_l.set_speed(pwm_l, brake);
        let _ = motor_r.set_speed(pwm_r, brake);
    }
}
