#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "wheel_pid.h"

// Encoders
extern volatile long leftTicks;
extern volatile long rightTicks;
extern volatile int lastEncodedLeft;
extern volatile int lastEncodedRight;

// Timing & State
extern long lastTicksL;
extern long lastTicksR;

// Control targets
extern float targetLeftVel;
extern float targetRightVel;
extern unsigned long lastCmdTime;
extern bool brakeEnabled;

// Motor control
extern WheelPID* leftPID;
extern WheelPID* rightPID;
extern float lastPwmLeft;
extern float lastPwmRight;

// Measurements
extern float vL_meas;
extern float vR_meas;

// Pose (odometry frame — pure encoder + IMU fusion)
extern float robotX;
extern float robotY;
extern float robotTheta;
extern float robotDistance;
extern float encoderTheta;
extern float gyroTheta;
extern float fusedTheta;

// Map-frame pose (odom pose ⊕ TF correction)
// mapPose = odomPose + tfMapOdom
extern float mapX;
extern float mapY;
extern float mapTheta;

// TF map→odom transform (accumulated scan matching corrections)
extern float tfDx;
extern float tfDy;
extern float tfDTheta;

// Battery
extern float filteredVBatt;

// Functions
void init_encoders();
void init_motors();
void setMotor(int pinIN1, int pinIN2, int pwmCh, float u);
void applyTf();  // Compute mapPose = odomPose ⊕ tfMapOdom
void updateTf(float dx, float dy, float dtheta, float weight);

#endif // ODOMETRY_H
