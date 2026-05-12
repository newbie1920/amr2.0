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

// ── Navigator Config ─────────────────────────────────────────
#define NAV_MAX_LINEAR_VEL    0.40f
#define NAV_APPROACH_VEL      0.05f
#define NAV_SLOWDOWN_DIST     0.25f
#define NAV_TURN_SPEED        1.2f

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
    bool refInitialized = false;

    // Control errors (for telemetry)
    float error_x = 0, error_y = 0, error_yaw = 0;

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
        state = NAV_IDLE;
        cmdLinear = cmdAngular = 0;
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
        cmdLinear = cmdAngular = 0;
        recoveryAttempts = 0;
    }

    void pause() {
        if (state != NAV_IDLE && state != NAV_DONE && state != NAV_ERROR && state != NAV_PAUSED) {
            prePauseState = state;
            state = NAV_PAUSED;
            cmdLinear = cmdAngular = 0;
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
            cmdLinear = cmdAngular = 0;
            return;
        }
        if (state == NAV_PAUSED) {
            lastWpReachTime = lastProgressCheckTime = millis();
            cmdLinear = cmdAngular = 0;
            return;
        }

        // ── Recovery behaviors ───────────────────────────────
        if (state == NAV_RECOVERY_SPIN) {
            if (millis() - recoveryStartTime < NAV_RECOVERY_SPIN_MS) {
                cmdLinear = 0; cmdAngular = NAV_RECOVERY_SPIN_W;
            } else {
                state = NAV_RECOVERY_BACKUP;
                recoveryStartTime = millis();
            }
            return;
        }
        if (state == NAV_RECOVERY_BACKUP) {
            if (millis() - recoveryStartTime < NAV_RECOVERY_BACKUP_MS) {
                cmdLinear = NAV_RECOVERY_BACKUP_V; cmdAngular = 0;
            } else {
                state = NAV_RECOVERY_WAIT;
                recoveryStartTime = millis();
                cmdLinear = cmdAngular = 0;
            }
            return;
        }
        if (state == NAV_RECOVERY_WAIT) {
            cmdLinear = cmdAngular = 0;
            if (millis() - recoveryStartTime >= NAV_RECOVERY_WAIT_MS) {
                state = NAV_TRACKING;
                lastWpReachTime = lastProgressCheckTime = millis();
                progressCheckX = robotX; progressCheckY = robotY;
                ref_x = robotX; ref_y = robotY;
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
                    cmdLinear = cmdAngular = 0;
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
            cmdLinear = cmdAngular = 0;
            return;
        }

        if (state == NAV_TRACKING) {
            if (!refInitialized) {
                ref_x = robotX; ref_y = robotY;
                progressCheckX = robotX; progressCheckY = robotY;
                refInitialized = true;
            }

            Waypoint& wp = waypoints[currentWpIdx];
            bool isFinalWp = (currentWpIdx == waypointCount - 1);

            float dx = wp.x - ref_x;
            float dy = wp.y - ref_y;
            float distToWp = sqrtf(dx * dx + dy * dy);

            float virtual_speed = NAV_MAX_LINEAR_VEL;
            if (isFinalWp && distToWp < NAV_SLOWDOWN_DIST) {
                virtual_speed = NAV_APPROACH_VEL + (NAV_MAX_LINEAR_VEL - NAV_APPROACH_VEL) * (distToWp / NAV_SLOWDOWN_DIST);
            }

            // Elastic tether
            float physDist = sqrtf((ref_x - robotX) * (ref_x - robotX) + (ref_y - robotY) * (ref_y - robotY));
            if (physDist > 0.4f) virtual_speed = 0.0f;
            else if (physDist > 0.2f) virtual_speed *= 0.5f;

            float step = virtual_speed * NAV_DT;

            // Move virtual robot
            if (distToWp <= step && distToWp > 0.001f) {
                ref_x = wp.x; ref_y = wp.y;
                lastWpReachTime = millis();
                if (isFinalWp) {
                    state = !isnan(finalHeading) ? NAV_FINAL_TURN : NAV_DONE;
                    cmdLinear = cmdAngular = 0;
                    return;
                }
                currentWpIdx++;
            } else if (distToWp > 0.001f) {
                ref_x += (dx / distToWp) * step;
                ref_y += (dy / distToWp) * step;
            }

            ref_theta = atan2f(wp.y - ref_y, wp.x - ref_x);

            // Backstepping control law
            float e_x_global = ref_x - robotX;
            float e_y_global = ref_y - robotY;
            float e_theta = _normalizeAngle(ref_theta - robotTheta);

            float ex = e_x_global * cosf(robotTheta) + e_y_global * sinf(robotTheta);
            float ey = -e_x_global * sinf(robotTheta) + e_y_global * cosf(robotTheta);
            error_x = ex; error_y = ey; error_yaw = e_theta;

            float ref_v = virtual_speed;
            if (fabsf(e_theta) > PI / 2) ref_v *= 0.2f;

            cmdLinear = ref_v * cosf(e_theta) + GAIN_KX * ex;
            float effective_v = fmax(ref_v, 0.05f);
            cmdAngular = effective_v * (GAIN_KY * ey + GAIN_KTH * sinf(e_theta));

            cmdLinear = constrain(cmdLinear, -NAV_MAX_LINEAR_VEL, NAV_MAX_LINEAR_VEL);
            cmdAngular = constrain(cmdAngular, -NAV_TURN_SPEED, NAV_TURN_SPEED);

        } else if (state == NAV_FINAL_TURN) {
            float finalErr = _normalizeAngle(finalHeading - robotTheta);
            if (fabsf(finalErr) < 0.052f) {
                state = NAV_DONE;
                cmdLinear = cmdAngular = 0;
            } else {
                cmdLinear = 0;
                float turnSpd = (fabsf(finalErr) < 0.26f) ? 0.3f : NAV_TURN_SPEED;
                cmdAngular = (finalErr > 0) ? turnSpd : -turnSpd;
            }
        }
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
    float _normalizeAngle(float a) {
        while (a > PI)  a -= 2.0f * PI;
        while (a < -PI) a += 2.0f * PI;
        return a;
    }
};

#endif // NAVIGATOR_H
