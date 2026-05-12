/**
 * IMU Driver — MPU6050 via raw I2C
 * Gyro-Z reading + auto-calibration + I2C bus recovery.
 */

#include "imu_mpu6050.h"
#include "config.h"
#include "robot_state.h"
#include "log.h"
#include <Wire.h>

extern SemaphoreHandle_t i2cMutex;

#define MPU6050_ADDR 0x68

// Calibration state
float imu_gyro_bias  = 0.0f;
bool  imu_calibrated = false;
static int   calSamples = 0;
static float calSum     = 0.0f;
static int   failCount  = 0;

// ── I2C Helpers ──────────────────────────────────────────────

static void i2c_recover() {
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeout(10);
#if defined(ESP32)
    Wire.setTimeOut(10);
#endif
}

static void writeReg(uint8_t reg, uint8_t val) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
    xSemaphoreGive(i2cMutex);
}

static int16_t readReg16(uint8_t reg, bool* ok = nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        if (ok) *ok = false;
        return 0;
    }
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        i2c_recover();
        xSemaphoreGive(i2cMutex);
        if (ok) *ok = false;
        return 0;
    }
    uint8_t rcv = Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
    if (rcv < 2) {
        i2c_recover();
        xSemaphoreGive(i2cMutex);
        if (ok) *ok = false;
        return 0;
    }
    int16_t res = (Wire.read() << 8) | Wire.read();
    xSemaphoreGive(i2cMutex);
    if (ok) *ok = true;
    return res;
}

// ── Public API ───────────────────────────────────────────────

bool imu_init() {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);  // WHO_AM_I register
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);

    if (Wire.available() < 1) {
        LOG_E("IMU", "MPU6050 not found!");
        xSemaphoreGive(i2cMutex);
        return false;
    }

    uint8_t whoAmI = Wire.read();
    xSemaphoreGive(i2cMutex);
    LOG_I("IMU", "WHO_AM_I: 0x%02X", whoAmI);

    writeReg(0x6B, 0x00);  // Wake up
    delay(100);
    writeReg(0x1B, 0x00);  // ±250°/s gyro range
    writeReg(0x1A, 0x06);  // DLPF = 5Hz bandwidth
    writeReg(0x19, 0x04);  // Sample rate divider → 200Hz

    LOG_I("IMU", "MPU6050 OK (±250°/s, DLPF=5Hz, 200Hz)");
    return true;
}

float imu_read_gyro_z() {
    bool ok = true;
    int16_t raw = readReg16(0x47, &ok);

    if (!ok) {
        failCount++;
        if (failCount > 20) {
            LOG_E("IMU", "Too many I2C failures — disabling MPU6050");
            // Caller should check return and handle
        }
        return 0.0f;
    }
    failCount = 0;
    return (raw / 131.0f) * (PI / 180.0f);  // Convert to rad/s
}

bool imu_calibrate_step(float rawGyroZ) {
    calSum += rawGyroZ;
    calSamples++;
    if (calSamples >= GYRO_CAL_COUNT) {
        imu_gyro_bias = calSum / (float)calSamples;
        imu_calibrated = true;
        // Sync with RobotState
        state.imu.calibrated = true;
        state.imu.bias = imu_gyro_bias;
        LOG_I("IMU", "Gyro calibrated. Bias: %.6f rad/s", imu_gyro_bias);
        return true;
    }
    return false;
}
