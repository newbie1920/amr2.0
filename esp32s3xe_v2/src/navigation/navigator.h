/**
 * AMR 2.0 v2 — Autonomous Navigator
 * Backstepping Trajectory Tracking (Lyapunov stability)
 * Recovery Behaviors: SPIN → BACKUP → WAIT (Nav2-style)
 *
 * Ported from v1 navigator.h — uses RobotState instead of globals.
 */

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "config.h"
#include "trajectory/trajectory_profile.h"

// ── Navigator Config ─────────────────────────────────────────
#define NAV_MAX_LINEAR_VEL    0.40f
#define NAV_APPROACH_VEL      0.05f
#define NAV_SLOWDOWN_DIST     0.25f
#define NAV_GOAL_TOLERANCE    0.08f
#define NAV_TURN_SPEED        1.2f
#define NAV_CMD_LINEAR_STEP    0.015f
#define NAV_CMD_ANGULAR_STEP   0.08f
#define NAV_LATERAL_SLOWDOWN_DIST 0.12f
#define NAV_LATERAL_STOP_DIST     0.35f
#define NAV_LATERAL_ANGULAR_LIMIT 0.75f
#define NAV_ALIGN_TO_PATH_ANGLE   0.70f
#define NAV_ALIGN_TURN_GAIN       1.60f
#define NAV_ALIGN_MIN_TURN_SPEED  0.22f

// Backstepping gains
#define GAIN_KX  2.0f
#define GAIN_KY  5.0f
#define GAIN_KTH 1.5f

#define NAV_DT 0.02f  // 50Hz

// Recovery Config
#define NAV_PROGRESS_CHECK_MS   3000
#define NAV_PROGRESS_DIST       0.05f
#define NAV_RECOVERY_SPIN_MS    3000
#define NAV_RECOVERY_BACKUP_MS  2000
#define NAV_RECOVERY_WAIT_MS    3000
#define NAV_RECOVERY_SPIN_W     1.0f
#define NAV_RECOVERY_BACKUP_V  -0.08f
#define NAV_MAX_RECOVERY_ATTEMPTS 3

// ── Nav States ───────────────────────────────────────────────
enum NavState {
    NAV_IDLE = 0,
    NAV_TRACKING,
    NAV_FINAL_TURN,
    NAV_PAUSED,
    NAV_RECOVERY_SPIN,
    NAV_RECOVERY_BACKUP,
    NAV_RECOVERY_WAIT,
    NAV_DONE,
    NAV_ERROR
};

// ── Waypoint ─────────────────────────────────────────────────
struct Waypoint {
    float x, y, heading;
    bool useReverse;
};

enum TrackingControllerMode : uint8_t {
    TRACK_CTRL_BACKSTEPPING = 0,
    TRACK_CTRL_FL_REGULARIZED = 1
};

struct TrackingConfig {
    TrackingControllerMode mode = TRACK_CTRL_BACKSTEPPING;
    bool straightYawGuard = true;
    float straightYawGuardGain = 0.40f;
    float lateralVelocityFloor = 0.08f;
    float flLateralDecayMin = 0.50f;
    float flSegmentAngularLimit = 0.85f;
    float flKx = GAIN_KX;
    float flKy = GAIN_KY;
    float flKth = GAIN_KTH;
};

// ── Navigator Class ──────────────────────────────────────────
class Navigator {
public:
    NavState state = NAV_IDLE;
    NavState prePauseState = NAV_IDLE;
    Waypoint waypoints[MAX_WAYPOINTS];
    int waypointCount = 0;
    int currentWpIdx  = 0;
    float finalHeading = NAN;

    float cmdLinear  = 0;
    float cmdAngular = 0;

    // Virtual reference robot (carrot)
    float ref_x = 0, ref_y = 0, ref_theta = 0;
    float ref_v = 0, ref_w = 0;
    bool refInitialized = false;
    TrajectoryProfile trajectory;
    TrajectoryConfig trajectoryConfig;
    TrajectoryReference trajRef;
    TrackingConfig trackingConfig;

    // Control errors (for telemetry)
    float error_x = 0, error_y = 0, error_yaw = 0;
    float body_error_x = 0, body_error_y = 0;

    unsigned long navStartTime = 0;
    unsigned long lastWpReachTime = 0;

    // Recovery state
    int recoveryAttempts = 0;
    unsigned long recoveryStartTime = 0;
    float progressCheckX = 0, progressCheckY = 0;
    unsigned long lastProgressCheckTime = 0;

    void loadPath(Waypoint* wps, int count, float endHeading = NAN) {
        if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
        waypointCount = count;
        currentWpIdx = 0;
        finalHeading = endHeading;
        for (int i = 0; i < count; i++) waypoints[i] = wps[i];
        trajectoryConfig.vMax = fminf(trajectoryConfig.vMax, NAV_MAX_LINEAR_VEL);
        trajectory.loadPath(toTrajectoryWaypoints(), count, trajectoryConfig);
        trajRef = trajectory.current();
        state = NAV_IDLE;
        stopCommands();
        ref_v = ref_w = 0;
        resetErrors();
        navStartTime = millis();
        recoveryAttempts = 0;
        refInitialized = false;
        if (count > 0) {
            state = NAV_TRACKING;
            lastWpReachTime = millis();
            lastProgressCheckTime = millis();
            ref_x = ref_y = 0;
        }
    }

    void abort() {
        state = NAV_IDLE;
        waypointCount = currentWpIdx = 0;
        stopCommands();
        ref_v = ref_w = 0;
        recoveryAttempts = 0;
        resetErrors();
        trajectory.reset();
        trajRef = trajectory.current();
    }

    void setTrajectoryConfig(const TrajectoryConfig& cfg) {
        trajectoryConfig = cfg;
        trajectoryConfig.vMax = constrain(trajectoryConfig.vMax, 0.02f, NAV_MAX_LINEAR_VEL);
        trajectoryConfig.aMax = fmaxf(0.05f, trajectoryConfig.aMax);
        trajectoryConfig.jMax = fmaxf(0.10f, trajectoryConfig.jMax);
        trajectory.setConfig(trajectoryConfig);
    }

    void setTrackingConfig(const TrackingConfig& cfg) {
        trackingConfig = cfg;
        trackingConfig.straightYawGuardGain = constrain(trackingConfig.straightYawGuardGain, 0.0f, 2.0f);
        trackingConfig.lateralVelocityFloor = constrain(trackingConfig.lateralVelocityFloor, 0.02f, NAV_MAX_LINEAR_VEL);
        trackingConfig.flLateralDecayMin = constrain(trackingConfig.flLateralDecayMin, 0.0f, 1.0f);
        trackingConfig.flSegmentAngularLimit = constrain(trackingConfig.flSegmentAngularLimit, 0.1f, NAV_TURN_SPEED);
        trackingConfig.flKx = constrain(trackingConfig.flKx, 0.0f, 4.0f);
        trackingConfig.flKy = constrain(trackingConfig.flKy, 0.0f, 8.0f);
        trackingConfig.flKth = constrain(trackingConfig.flKth, 0.0f, 4.0f);
    }

    const char* trackingModeName() const {
        return trackingConfig.mode == TRACK_CTRL_FL_REGULARIZED ? "fl_regularized" : "backstepping";
    }

    void pause() {
        if (state != NAV_IDLE && state != NAV_DONE && state != NAV_ERROR && state != NAV_PAUSED) {
            prePauseState = state;
            state = NAV_PAUSED;
            stopCommands();
        }
    }

    void resume() {
        if (state == NAV_PAUSED) {
            state = prePauseState;
            navStartTime = lastWpReachTime = lastProgressCheckTime = millis();
        }
    }

    void update(float robotX, float robotY, float robotTheta) {
        if (state == NAV_IDLE || state == NAV_DONE || state == NAV_ERROR) {
            stopCommands();
            return;
        }
        if (state == NAV_PAUSED) {
            lastWpReachTime = lastProgressCheckTime = millis();
            stopCommands();
            return;
        }

        // ── Recovery behaviors ───────────────────────────────
        if (state == NAV_RECOVERY_SPIN) {
            if (millis() - recoveryStartTime < NAV_RECOVERY_SPIN_MS) {
                setDirectCommand(0.0f, NAV_RECOVERY_SPIN_W);
            } else {
                state = NAV_RECOVERY_BACKUP;
                recoveryStartTime = millis();
            }
            return;
        }
        if (state == NAV_RECOVERY_BACKUP) {
            if (millis() - recoveryStartTime < NAV_RECOVERY_BACKUP_MS) {
                setDirectCommand(NAV_RECOVERY_BACKUP_V, 0.0f);
            } else {
                state = NAV_RECOVERY_WAIT;
                recoveryStartTime = millis();
                stopCommands();
            }
            return;
        }
        if (state == NAV_RECOVERY_WAIT) {
            stopCommands();
            if (millis() - recoveryStartTime >= NAV_RECOVERY_WAIT_MS) {
                state = NAV_TRACKING;
                lastWpReachTime = lastProgressCheckTime = millis();
                progressCheckX = robotX; progressCheckY = robotY;
                ref_x = robotX; ref_y = robotY;
                trajectory.loadPath(toTrajectoryWaypoints(), waypointCount, trajectoryConfig);
            }
            return;
        }

        // ── Progress check (stuck detection) ─────────────────
        if (state == NAV_TRACKING && millis() - lastProgressCheckTime > NAV_PROGRESS_CHECK_MS) {
            float moved = sqrtf((robotX - progressCheckX) * (robotX - progressCheckX) +
                                (robotY - progressCheckY) * (robotY - progressCheckY));
            if (moved < NAV_PROGRESS_DIST) {
                recoveryAttempts++;
                if (recoveryAttempts > NAV_MAX_RECOVERY_ATTEMPTS) {
                    state = NAV_ERROR;
                    stopCommands();
                    return;
                }
                state = NAV_RECOVERY_SPIN;
                recoveryStartTime = millis();
                return;
            }
            progressCheckX = robotX; progressCheckY = robotY;
            lastProgressCheckTime = millis();
        }

        // ── Waypoint timeout ─────────────────────────────────
        if (millis() - lastWpReachTime > NAV_WP_TIMEOUT_MS) {
            state = NAV_ERROR;
            stopCommands();
            return;
        }

        if (state == NAV_TRACKING) {
            if (!refInitialized) {
                ref_x = robotX; ref_y = robotY;
                progressCheckX = robotX; progressCheckY = robotY;
                refInitialized = true;
            }

            int previousWpIdx = currentWpIdx;
            trajRef = trajectory.sample(NAV_DT);
            currentWpIdx = constrain(trajRef.targetIndex, 0, waypointCount - 1);
            bool segmentChanged = currentWpIdx != previousWpIdx;
            if (segmentChanged) {
                lastWpReachTime = millis();
            }
            Waypoint& wp = waypoints[currentWpIdx];
            bool isFinalWp = (currentWpIdx == waypointCount - 1);
            float physGoalDx = wp.x - robotX;
            float physGoalDy = wp.y - robotY;
            float physDistToWp = sqrtf(physGoalDx * physGoalDx + physGoalDy * physGoalDy);

            if (isFinalWp && trajectory.isFinished() && physDistToWp <= NAV_GOAL_TOLERANCE) {
                state = !isnan(finalHeading) ? NAV_FINAL_TURN : NAV_DONE;
                stopCommands();
                return;
            }

            ref_x = trajRef.x;
            ref_y = trajRef.y;
            ref_theta = trajRef.theta;
            ref_v = fabsf(trajRef.v);
            ref_w = trajRef.w;

            // Elastic tether
            float physDist = sqrtf((ref_x - robotX) * (ref_x - robotX) + (ref_y - robotY) * (ref_y - robotY));
            if (physDist > 0.4f) ref_v = 0.0f;
            else if (physDist > 0.2f) ref_v *= 0.5f;

            // Backstepping control law
            float e_x_global = ref_x - robotX;
            float e_y_global = ref_y - robotY;
            float e_theta = _normalizeAngle(ref_theta - robotTheta);
            float guideDx = e_x_global;
            float guideDy = e_y_global;
            float guideDist = sqrtf(guideDx * guideDx + guideDy * guideDy);
            if (guideDist < NAV_GOAL_TOLERANCE) {
                guideDx = physGoalDx;
                guideDy = physGoalDy;
                guideDist = physDistToWp;
            }

            float ex = e_x_global * cosf(robotTheta) + e_y_global * sinf(robotTheta);
            float ey = -e_x_global * sinf(robotTheta) + e_y_global * cosf(robotTheta);
            error_x = e_x_global;
            error_y = e_y_global;
            body_error_x = ex;
            body_error_y = ey;
            error_yaw = e_theta;

            if (!wp.useReverse && guideDist > NAV_GOAL_TOLERANCE) {
                float lineOfSightHeading = atan2f(guideDy, guideDx);
                float lineOfSightError = _normalizeAngle(lineOfSightHeading - robotTheta);
                if (fabsf(lineOfSightError) > NAV_ALIGN_TO_PATH_ANGLE) {
                    ref_theta = lineOfSightHeading;
                    ref_v = 0.0f;
                    ref_w = 0.0f;
                    error_yaw = lineOfSightError;
                    publishRotateToHeadingCommand(lineOfSightError);
                    return;
                }
            }

            float feedForwardW = (fabsf(ref_w) < 0.001f) ? 0.0f : ref_w;
            if (trackingConfig.mode == TRACK_CTRL_FL_REGULARIZED) {
                float absYaw = fabsf(e_theta);
                if (absYaw > (2.0f * PI / 3.0f)) ref_v *= 0.3f;

                float kxScale = constrain(ref_v / 0.10f, 0.6f, 1.0f);
                float scheduledKx = trackingConfig.flKx * kxScale;
                float lateralDecay = 1.0f;
                if (absYaw > PI / 4.0f) {
                    lateralDecay = fmaxf(trackingConfig.flLateralDecayMin, fabsf(cosf(e_theta)));
                }

                float lateralTerm = trackingConfig.flKy * lateralDecay * ey;
                float headingTerm = trackingConfig.flKth * sinf(e_theta);
                float straightGuard = (fabsf(ref_w) < 0.001f)
                                    ? trackingConfig.straightYawGuardGain * e_theta
                                    : 0.0f;
                float angularCorrection = lateralTerm + headingTerm + straightGuard;
                if (segmentChanged) {
                    angularCorrection = constrain(angularCorrection,
                                                  -trackingConfig.flSegmentAngularLimit,
                                                  trackingConfig.flSegmentAngularLimit);
                }

                float targetLinear = ref_v * cosf(e_theta) + scheduledKx * ex;
                float targetAngular = feedForwardW + angularCorrection;
                publishTrackingCommand(targetLinear, targetAngular, ey, isFinalWp, physDistToWp, wp.useReverse);
            } else {
                if (fabsf(e_theta) > PI / 2) ref_v *= 0.2f;
                float targetLinear = ref_v * cosf(e_theta) + GAIN_KX * ex;
                float targetAngular = feedForwardW + GAIN_KY * ey + GAIN_KTH * sinf(e_theta);
                if (trackingConfig.straightYawGuard && fabsf(ref_w) < 0.001f) {
                    targetAngular += trackingConfig.straightYawGuardGain * e_theta;
                }
                publishTrackingCommand(targetLinear, targetAngular, ey, isFinalWp, physDistToWp, wp.useReverse);
            }

        } else if (state == NAV_FINAL_TURN) {
            float finalErr = _normalizeAngle(finalHeading - robotTheta);
            if (fabsf(finalErr) < 0.052f) {
                state = NAV_DONE;
                stopCommands();
            } else {
                float turnSpd = (fabsf(finalErr) < 0.26f) ? 0.3f : NAV_TURN_SPEED;
                cmdLinear = limitRate(0.0f, lastCmdLinear, NAV_CMD_LINEAR_STEP);
                cmdAngular = limitRate((finalErr > 0) ? turnSpd : -turnSpd,
                                       lastCmdAngular,
                                       NAV_CMD_ANGULAR_STEP);
                lastCmdLinear = cmdLinear;
                lastCmdAngular = cmdAngular;
            }
        }
    }

    void resetErrors() {
        error_x = error_y = error_yaw = 0.0f;
        body_error_x = body_error_y = 0.0f;
    }

    const char* getStateName() const {
        switch (state) {
            case NAV_IDLE:            return "IDLE";
            case NAV_TRACKING:        return "TRACK";
            case NAV_FINAL_TURN:      return "F_TURN";
            case NAV_PAUSED:          return "PAUSED";
            case NAV_RECOVERY_SPIN:   return "REC_SPIN";
            case NAV_RECOVERY_BACKUP: return "REC_BACK";
            case NAV_RECOVERY_WAIT:   return "REC_WAIT";
            case NAV_DONE:            return "DONE";
            case NAV_ERROR:           return "ERROR";
            default:                  return "???";
        }
    }

    bool isNavigating() const {
        return state == NAV_TRACKING || state == NAV_FINAL_TURN ||
               state == NAV_RECOVERY_SPIN || state == NAV_RECOVERY_BACKUP ||
               state == NAV_RECOVERY_WAIT;
    }

    bool isRecovering() const {
        return state == NAV_RECOVERY_SPIN || state == NAV_RECOVERY_BACKUP || state == NAV_RECOVERY_WAIT;
    }

private:
    float lastCmdLinear = 0.0f;
    float lastCmdAngular = 0.0f;

    void setDirectCommand(float linear, float angular) {
        cmdLinear = linear;
        cmdAngular = angular;
        lastCmdLinear = linear;
        lastCmdAngular = angular;
    }

    void stopCommands() {
        setDirectCommand(0.0f, 0.0f);
    }

    float limitRate(float target, float previous, float maxStep) {
        return constrain(target, previous - maxStep, previous + maxStep);
    }

    void publishTrackingCommand(float targetLinear,
                                float targetAngular,
                                float lateralError,
                                bool isFinalWp,
                                float distToWp,
                                bool allowReverse = false) {
        targetLinear = constrain(targetLinear, -NAV_MAX_LINEAR_VEL, NAV_MAX_LINEAR_VEL);
        if (!allowReverse && targetLinear < 0.0f) {
            targetLinear = 0.0f;
        }
        if (isFinalWp && distToWp < NAV_SLOWDOWN_DIST) {
            targetLinear = constrain(targetLinear, -NAV_APPROACH_VEL, NAV_APPROACH_VEL);
        }

        float absLateral = fabsf(lateralError);
        if (absLateral > NAV_LATERAL_SLOWDOWN_DIST) {
            float slowSpan = NAV_LATERAL_STOP_DIST - NAV_LATERAL_SLOWDOWN_DIST;
            float scale = 1.0f - ((absLateral - NAV_LATERAL_SLOWDOWN_DIST) / fmaxf(slowSpan, 0.001f));
            scale = constrain(scale, 0.20f, 1.0f);
            targetLinear *= scale;
            targetAngular = constrain(targetAngular,
                                      -NAV_LATERAL_ANGULAR_LIMIT,
                                      NAV_LATERAL_ANGULAR_LIMIT);
        }

        targetAngular = constrain(targetAngular, -NAV_TURN_SPEED, NAV_TURN_SPEED);
        cmdLinear = limitRate(targetLinear, lastCmdLinear, NAV_CMD_LINEAR_STEP);
        cmdAngular = limitRate(targetAngular, lastCmdAngular, NAV_CMD_ANGULAR_STEP);
        lastCmdLinear = cmdLinear;
        lastCmdAngular = cmdAngular;
    }

    void publishRotateToHeadingCommand(float headingError) {
        float absErr = fabsf(headingError);
        float targetAngular = constrain(headingError * NAV_ALIGN_TURN_GAIN,
                                        -NAV_TURN_SPEED,
                                        NAV_TURN_SPEED);
        if (absErr > 0.08f && fabsf(targetAngular) < NAV_ALIGN_MIN_TURN_SPEED) {
            targetAngular = (headingError > 0.0f)
                          ? NAV_ALIGN_MIN_TURN_SPEED
                          : -NAV_ALIGN_MIN_TURN_SPEED;
        }

        cmdLinear = limitRate(0.0f, lastCmdLinear, NAV_CMD_LINEAR_STEP);
        cmdAngular = limitRate(targetAngular, lastCmdAngular, NAV_CMD_ANGULAR_STEP);
        lastCmdLinear = cmdLinear;
        lastCmdAngular = cmdAngular;
    }

    TrajectoryWaypoint* toTrajectoryWaypoints() {
        static TrajectoryWaypoint trajWps[MAX_WAYPOINTS];
        for (int i = 0; i < waypointCount; i++) {
            trajWps[i].x = waypoints[i].x;
            trajWps[i].y = waypoints[i].y;
            trajWps[i].heading = waypoints[i].heading;
            trajWps[i].useReverse = waypoints[i].useReverse;
        }
        return trajWps;
    }

    float _normalizeAngle(float a) {
        while (a > PI)  a -= 2.0f * PI;
        while (a < -PI) a += 2.0f * PI;
        return a;
    }
};

#endif // NAVIGATOR_H
