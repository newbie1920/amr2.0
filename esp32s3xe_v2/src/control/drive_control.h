/**
 * Differential-drive target generation with optional straight heading hold.
 *
 * Wheel PID remains independent: this helper only converts robot-frame
 * velocity commands into left/right wheel targets.
 */
#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <Arduino.h>
#include "config.h"
#include "robot_state.h"

static constexpr float DRIVE_MAX_WHEEL_RAD_S = 30.0f;
static constexpr float DRIVE_MAX_ANGULAR_RATE = 1.2f;
static constexpr float DRIVE_STRAIGHT_MIN_V = 0.04f;
static constexpr float DRIVE_STRAIGHT_W_BAND = 0.12f;
static constexpr float DRIVE_HEADING_HOLD_GAIN = 1.1f;
static constexpr float DRIVE_RATE_HOLD_GAIN = 0.25f;
static constexpr float DRIVE_STRAIGHT_MAX_CORRECTION_W = 0.18f;

struct DriveTargetResult {
    float linear = 0.0f;
    float requestedAngular = 0.0f;
    float angular = 0.0f;
    float targetLeft = 0.0f;
    float targetRight = 0.0f;
    float measuredAngular = 0.0f;
    float headingError = 0.0f;
    float headingCorrection = 0.0f;
    bool headingHoldActive = false;
};

inline float driveNormalizeAngle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

inline float driveMeasuredAngularRate() {
    return (state.motor.vR_meas - state.motor.vL_meas) *
           WHEEL_RADIUS / WHEEL_SEPARATION;
}

inline bool driveCanHoldStraight(float linear, float angular) {
    return fabsf(linear) > DRIVE_STRAIGHT_MIN_V &&
           fabsf(angular) < DRIVE_STRAIGHT_W_BAND;
}

inline DriveTargetResult driveSetVelocityTargets(float linear,
                                                 float angular,
                                                 bool holdHeading,
                                                 float targetTheta,
                                                 float currentTheta) {
    DriveTargetResult result;
    result.linear = linear;
    result.requestedAngular = angular;
    result.angular = angular;
    result.measuredAngular = driveMeasuredAngularRate();

    if (holdHeading && driveCanHoldStraight(linear, angular)) {
        result.headingHoldActive = true;
        result.headingError = driveNormalizeAngle(targetTheta - currentTheta);
        const float headingTerm = result.headingError * DRIVE_HEADING_HOLD_GAIN;
        const float rateTerm = (angular - result.measuredAngular) * DRIVE_RATE_HOLD_GAIN;
        result.headingCorrection = constrain(headingTerm + rateTerm,
                                             -DRIVE_STRAIGHT_MAX_CORRECTION_W,
                                             DRIVE_STRAIGHT_MAX_CORRECTION_W);
        result.angular = constrain(angular + result.headingCorrection,
                                   -DRIVE_MAX_ANGULAR_RATE,
                                   DRIVE_MAX_ANGULAR_RATE);
    }

    result.targetLeft = constrain((linear - result.angular * WHEEL_SEPARATION / 2.0f) /
                                      WHEEL_RADIUS,
                                  -DRIVE_MAX_WHEEL_RAD_S,
                                  DRIVE_MAX_WHEEL_RAD_S);
    result.targetRight = constrain((linear + result.angular * WHEEL_SEPARATION / 2.0f) /
                                       WHEEL_RADIUS,
                                   -DRIVE_MAX_WHEEL_RAD_S,
                                   DRIVE_MAX_WHEEL_RAD_S);

    state.motor.targetLeftVel = result.targetLeft;
    state.motor.targetRightVel = result.targetRight;
    state.motor.cmdLinear = linear;
    state.motor.cmdAngularRequested = result.requestedAngular;
    state.motor.cmdAngularApplied = result.angular;
    state.motor.headingHoldError = result.headingError;
    state.motor.headingHoldCorrection = result.headingCorrection;
    state.motor.headingHoldActive = result.headingHoldActive;
    state.motor.lastCmdTime = millis();
    return result;
}

#endif // DRIVE_CONTROL_H
