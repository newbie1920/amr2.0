#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>

extern float gyroZBias;
extern bool gyroCalibrated;
extern int gyroCalSamples;
extern float gyroCalSum;
extern float gyroZ_raw;
extern bool imuAvailable;

extern bool inaAvailable;
extern float ina_busV[3];
extern float ina_currentA[3];

bool mpu6050_init();
float mpu6050_readGyroZ();
void mpu6050_calibrate(float rawZ);

bool ina3221_init();
void read_ina3221();

#endif // IMU_SENSOR_H
