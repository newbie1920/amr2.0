/**
 * Odometry — Sensor Fusion Implementation
 * Reads encoders + IMU, computes odometry pose, applies TF.
 */

#include "odometry.h"
#include "config.h"
#include "robot_state.h"
#include "log.h"
#include "encoder_driver.h"
#include "imu_mpu6050.h"

void odometry_init() {
    state.odom.x = 5.0f;
    state.odom.y = 5.0f;
    state.odom.theta = 0.0f;
    state.odom.distance = 0.0f;
    state.odom.encoderTheta = 0.0f;
    state.odom.gyroTheta = 0.0f;
    state.odom.fusedTheta = 0.0f;
    LOG_I("ODOM", "Odometry initialized at (5.0, 5.0)");
}

void odometry_update(float deltaT) {
    // ── IMU Read ─────────────────────────────────────────
    float gyroZ_raw = 0.0f;
    if (state.imu.available) {
        gyroZ_raw = imu_read_gyro_z();
        if (!imu_calibrated) {
            imu_calibrate_step(gyroZ_raw);
            gyroZ_raw = 0.0f;
        } else {
            gyroZ_raw -= imu_gyro_bias;
            // Zero-velocity clamping
            if (fabsf(state.motor.targetLeftVel) < 0.01f &&
                fabsf(state.motor.targetRightVel) < 0.01f &&
                fabsf(gyroZ_raw) < 0.01f) {
                gyroZ_raw = 0.0f;
            }
            state.odom.gyroTheta += gyroZ_raw * deltaT;
            state.odom.gyroTheta = atan2f(sinf(state.odom.gyroTheta), cosf(state.odom.gyroTheta));
        }
    }

    // ── Read Encoders ────────────────────────────────────
    noInterrupts();
    long cL = encoderLeftTicks;
    long cR = encoderRightTicks;
    interrupts();

    float vL_raw = (float)(cL - state.motor.lastTicksL) / TICKS_PER_REV * 2.0f * PI / deltaT;
    float vR_raw = (float)(cR - state.motor.lastTicksR) / TICKS_PER_REV * 2.0f * PI / deltaT;

    // Low-pass filter
    state.motor.vL_meas = 0.7f * state.motor.vL_meas + 0.3f * vL_raw;
    state.motor.vR_meas = 0.7f * state.motor.vR_meas + 0.3f * vR_raw;
    state.motor.lastTicksL = cL;
    state.motor.lastTicksR = cR;

    // ── Kinematics ───────────────────────────────────────
    float v_robot = (state.motor.vR_meas + state.motor.vL_meas) / 2.0f * WHEEL_RADIUS;
    float w_encoder = (state.motor.vR_meas - state.motor.vL_meas) * WHEEL_RADIUS / WHEEL_SEPARATION;

    state.odom.encoderTheta += w_encoder * deltaT;
    state.odom.encoderTheta = atan2f(sinf(state.odom.encoderTheta), cosf(state.odom.encoderTheta));

    // ── Sensor Fusion (Complementary Filter) ─────────────
    if (state.imu.available && imu_calibrated) {
        float diff = state.odom.gyroTheta - state.odom.encoderTheta;
        while (diff > PI) diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;
        state.odom.fusedTheta = state.odom.encoderTheta + COMP_FILTER_ALPHA * diff;
        state.odom.fusedTheta = atan2f(sinf(state.odom.fusedTheta), cosf(state.odom.fusedTheta));
        state.odom.encoderTheta = state.odom.fusedTheta;
        state.odom.theta = state.odom.fusedTheta;
    } else {
        state.odom.fusedTheta = state.odom.encoderTheta;
        state.odom.theta = state.odom.encoderTheta;
    }

    // ── Odometry position update ─────────────────────────
    float dist = v_robot * deltaT;
    state.odom.distance += fabsf(dist);
    state.odom.x += dist * cosf(state.odom.theta);
    state.odom.y += dist * sinf(state.odom.theta);

    // Recompute map-frame pose (needs critical section — tf.* written by lidarTask on Core 0)
    portENTER_CRITICAL(&stateMux);
    applyTf();
    portEXIT_CRITICAL(&stateMux);
}
