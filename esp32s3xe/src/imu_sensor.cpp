#include "imu_sensor.h"
#include "config.h"
#include <Wire.h>

extern SemaphoreHandle_t i2cMutex;

// Global state
float gyroZBias = 0;
bool gyroCalibrated = false;
int gyroCalSamples = 0;
float gyroCalSum = 0;
float gyroZ_raw = 0;
bool imuAvailable = false;

bool inaAvailable = false;
float ina_busV[3] = {0, 0, 0};
float ina_currentA[3] = {0, 0, 0};

#define MPU6050_ADDR 0x68
#define INA3221_ADDR 0x40

// ============================================================
//   MPU6050 BARE-METAL FUNCTIONS
// ============================================================
void mpu6050_writeReg(uint8_t reg, uint8_t val) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  xSemaphoreGive(i2cMutex);
}

int16_t mpu6050_readReg16(uint8_t reg, bool* ok = nullptr) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      if(ok) *ok = false;
      return 0;
  }
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeout(10);
#if defined(ESP32)
    Wire.setTimeOut(10);
#endif
    xSemaphoreGive(i2cMutex);
    if(ok) *ok = false;
    return 0;
  }
  uint8_t rcv = Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
  if (rcv < 2) {
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeout(10);
#if defined(ESP32)
    Wire.setTimeOut(10);
#endif
    xSemaphoreGive(i2cMutex);
    if(ok) *ok = false;
    return 0;
  }
  int16_t res = (Wire.read() << 8) | Wire.read();
  xSemaphoreGive(i2cMutex);
  if(ok) *ok = true;
  return res;
}

bool mpu6050_init() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);

  if (Wire.available() < 1) {
    Serial.println("[IMU] MPU6050 KHONG TIM THAY!");
    xSemaphoreGive(i2cMutex);
    return false;
  }

  uint8_t whoAmI = Wire.read();
  xSemaphoreGive(i2cMutex);
  Serial.printf("[IMU] WHO_AM_I: 0x%02X\n", whoAmI);

  mpu6050_writeReg(0x6B, 0x00); // Wake up
  delay(100);
  mpu6050_writeReg(0x1B, 0x00); // ±250°/s
  mpu6050_writeReg(0x1A, 0x06); // DLPF 5Hz
  mpu6050_writeReg(0x19, 0x04); // 200Hz sample rate

  Serial.println("[IMU] MPU6050 OK (±250°/s, DLPF=5Hz, 200Hz)");
  return true;
}

float mpu6050_readGyroZ() {
  static int imuFailCount = 0;
  bool ok = true;
  int16_t raw = mpu6050_readReg16(0x47, &ok);
  
  if (!ok) {
      imuFailCount++;
      if (imuFailCount > 20) {
          Serial.println("[IMU] Too many I2C failures. Disabling MPU6050.");
          imuAvailable = false;
      }
      return 0.0f;
  } else {
      imuFailCount = 0;
  }
  return (raw / 131.0f) * (PI / 180.0f); // rad/s
}

void mpu6050_calibrate(float rawZ) {
  gyroCalSum += rawZ;
  gyroCalSamples++;
  if (gyroCalSamples >= GYRO_CAL_COUNT) {
    gyroZBias = gyroCalSum / (float)gyroCalSamples;
    gyroCalibrated = true;
    Serial.printf("[IMU] Gyro calibrated. Bias: %.6f rad/s\n", gyroZBias);
  }
}

// ============================================================
//   INA3221 BARE-METAL FUNCTIONS
// ============================================================
int16_t ina3221_readReg(uint8_t reg, bool* ok = nullptr) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      if(ok) *ok = false;
      return 0;
  }
  Wire.beginTransmission(INA3221_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeout(10);
#if defined(ESP32)
    Wire.setTimeOut(10);
#endif
    xSemaphoreGive(i2cMutex);
    if(ok) *ok = false;
    return 0;
  }
  uint8_t rcv = Wire.requestFrom((uint8_t)INA3221_ADDR, (uint8_t)2);
  if (rcv < 2) {
    Wire.end();
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Wire.setTimeout(10);
#if defined(ESP32)
    Wire.setTimeOut(10);
#endif
    xSemaphoreGive(i2cMutex);
    if(ok) *ok = false;
    return 0;
  }
  int16_t res = (Wire.read() << 8) | Wire.read();
  xSemaphoreGive(i2cMutex);
  if(ok) *ok = true;
  return res;
}

bool ina3221_init() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
  Wire.beginTransmission(INA3221_ADDR);
  if (Wire.endTransmission() == 0) {
      Serial.println("[BOOT] INA3221 OK (0x40)");
      xSemaphoreGive(i2cMutex);
      return true;
  } else {
      Serial.println("[BOOT] INA3221 KHONG Thay!");
      xSemaphoreGive(i2cMutex);
      return false;
  }
}

void read_ina3221() {
  if (!inaAvailable) return;
  static int inaFailCount = 0;
  bool success = true;

  for (int ch = 1; ch <= 3; ch++) {
    bool ok1 = true, ok2 = true;
    int16_t rawBus = ina3221_readReg(2 + (ch - 1) * 2, &ok1);
    int16_t rawShunt = ina3221_readReg(1 + (ch - 1) * 2, &ok2);
    
    if (!ok1 || !ok2) {
        success = false;
        continue;
    }

    // Shift right 3 bits, LSB = 8mV
    float voltage = (rawBus >> 3) * 0.008f;
    if (voltage > 1.0f) {
        voltage -= 0.85f; // Bù trừ sai số của mạch INA3221 (so với đồng hồ đo thực tế)
    }
    ina_busV[ch - 1] = voltage;

    // Shift right 3 bits, LSB = 40uV, Shunt = 0.1Ohm -> I(A) = Vshunt / 0.1 = Vshunt * 10
    // => Current = (raw >> 3) * 0.00004 * 10 = (raw >> 3) * 0.0004
    ina_currentA[ch - 1] = (rawShunt >> 3) * 0.0004f; 
  }

  if (success) {
      inaFailCount = 0;
  } else {
      inaFailCount++;
      if (inaFailCount > 20) {
          Serial.println("[INA3221] Too many I2C failures. Disabling INA3221.");
          inaAvailable = false;
      }
  }

  // Debug INA3221 mỗi 5 giây
  static unsigned long lastInaDebug = 0;
  if (millis() - lastInaDebug > 5000) {
    lastInaDebug = millis();
    // Serial.printf("[INA3221] CH1: %.2fV %.3fA | CH2: %.2fV %.3fA | CH3: %.2fV %.3fA\n",
    //               ina_busV[0], ina_currentA[0],
    //               ina_busV[1], ina_currentA[1],
    //               ina_busV[2], ina_currentA[2]);
  }
}
