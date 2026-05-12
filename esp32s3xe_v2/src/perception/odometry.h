/**
 * Odometry — Encoder + IMU Sensor Fusion
 * Differential drive kinematics + complementary filter.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

void odometry_init();

// Call at 50Hz from controlTask
void odometry_update(float deltaT);

#endif
