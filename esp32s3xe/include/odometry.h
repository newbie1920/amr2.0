#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "wheel_pid.h"

// ============================================================
//   THREAD-SAFE POSE SNAPSHOT (Multi-core protection)
//   controlTask (Core 1) writes → lidarTask/explorationTask (Core 0) reads
//   Uses portMUX spinlock (fastest sync primitive on ESP32)
// ============================================================

struct PoseSnapshot {
    float x;
    float y;
    float theta;
};

extern portMUX_TYPE poseMux;  // Protects robotX/Y/Theta + mapX/Y/Theta + tfDx/Dy/DTheta

// Thread-safe pose read/write helpers
inline PoseSnapshot getOdomPose() {
    PoseSnapshot p;
    portENTER_CRITICAL(&poseMux);
    extern float robotX, robotY, robotTheta;
    p.x = robotX; p.y = robotY; p.theta = robotTheta;
    portEXIT_CRITICAL(&poseMux);
    return p;
}

inline PoseSnapshot getMapPose() {
    PoseSnapshot p;
    portENTER_CRITICAL(&poseMux);
    extern float mapX, mapY, mapTheta;
    p.x = mapX; p.y = mapY; p.theta = mapTheta;
    portEXIT_CRITICAL(&poseMux);
    return p;
}

inline void getTfCorrection(float& dx, float& dy, float& dtheta) {
    portENTER_CRITICAL(&poseMux);
    extern float tfDx, tfDy, tfDTheta;
    dx = tfDx; dy = tfDy; dtheta = tfDTheta;
    portEXIT_CRITICAL(&poseMux);
}

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
