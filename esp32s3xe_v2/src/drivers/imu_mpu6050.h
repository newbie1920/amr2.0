/**
 * IMU Driver — MPU6050 via raw I2C
 * Gyro-Z only (for heading fusion), auto-calibration on boot.
 */

#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <Arduino.h>

bool  imu_init();           // Returns true if MPU6050 found
float imu_read_gyro_z();    // Returns rad/s (bias-corrected after calibration)
bool  imu_calibrate_step(float rawGyroZ);  // Call N times, returns true when done

// Calibration state (managed internally)
extern float imu_gyro_bias;
extern bool  imu_calibrated;

#endif
