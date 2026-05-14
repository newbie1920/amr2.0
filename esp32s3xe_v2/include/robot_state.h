/**
 * AMR 2.0 v2 — Centralized Robot State
 * Replaces 70+ extern globals with a single thread-safe struct.
 * 
 * Write rules:
 *   - controlTask owns odom/motor writes (Core 1)
 *   - lidarTask owns lidar/slam writes (Core 0)
 *   - networkTask reads everything, writes targets/flags only
 * 
 * Thread safety: use portENTER_CRITICAL(&stateMux) for cross-core access
 */

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Arduino.h>

// ── Pose Snapshot (for thread-safe reads) ──────────────────
struct PoseSnapshot {
    float x, y, theta;
};

// ── Robot State ────────────────────────────────────────────
struct RobotState {
    // ── Odometry frame (encoder + IMU fusion) ──
    struct {
        float x       = 5.0f;  // Start at (5,5) center of grid
        float y       = 5.0f;
        float theta   = 0.0f;
        float distance = 0.0f;
        float encoderTheta = 0.0f;
        float gyroTheta    = 0.0f;
        float fusedTheta   = 0.0f;
    } odom;

    // ── Map frame (odom ⊕ TF correction from SLAM) ──
    struct {
        float x     = 5.0f;
        float y     = 5.0f;
        float theta = 0.0f;
    } map;

    // ── TF map→odom transform ──
    struct {
        float dx     = 0.0f;
        float dy     = 0.0f;
        float dTheta = 0.0f;
    } tf;

    // ── Motor / Encoder ──
    struct {
        volatile long leftTicks  = 0;
        volatile long rightTicks = 0;
        long lastTicksL = 0;
        long lastTicksR = 0;
        float vL_meas = 0.0f;  // Measured velocities (rad/s)
        float vR_meas = 0.0f;
        float targetLeftVel  = 0.0f;  // Target velocities
        float targetRightVel = 0.0f;
        float pwmLeft  = 0.0f;
        float pwmRight = 0.0f;
        unsigned long lastCmdTime = 0;
        bool brakeEnabled = false;
    } motor;

    // ── IMU ──
    struct {
        float gyroZ_raw = 0.0f;
        float bias      = 0.0f;
        bool available  = false;
        bool calibrated = false;
        int calSamples  = 0;
        float calSum    = 0.0f;
    } imu;

    // ── LiDAR ──
    struct {
        uint16_t distances[360] = {0};
        bool running   = false;
        bool receiving = false;
        bool obstacleDetected = false;
        unsigned long lastObstacleTime = 0;
        unsigned long lastGridUpdateTime = 0;
    } lidar;

    // ── Battery & Power ──
    struct {
        float filteredVBatt = 0.0f;
        int percent = 0;
        // INA3221 readings
        float busV[3]    = {0};
        float currentA[3] = {0};
        bool inaAvailable = false;
    } power;

    // ── Navigation ──
    enum NavMode { MODE_ONBOARD, MODE_PC_BROWSER };
    
    struct {
        bool hitlMode = false;
        bool explorationRequested = false;
        bool streamOccupancyGrid = true;
        bool allowOnboardNav = true;
        const char* archProfile = "hybrid";
        NavMode mode = MODE_ONBOARD;
    } nav;

    // ── ICP/SLAM diagnostics ──
    struct {
        float icpRms = 0.0f;
        float matchScore = 0.0f;
        float tfNorm = 0.0f;
        float tfAngleDeg = 0.0f;
        float gridCoverage = 0.0f;
        int gridOccupied = 0;
        int gridFree = 0;
        int scanCount = 0;
        float scanMatchMs = 0.0f;
        int frontierCount = 0;
    } slam;
};

// ── Global state instance ──
extern RobotState state;
extern portMUX_TYPE stateMux;

// ── Thread-safe pose helpers ──────────────────────────────

inline PoseSnapshot getOdomPose() {
    PoseSnapshot p;
    portENTER_CRITICAL(&stateMux);
    p.x = state.odom.x;
    p.y = state.odom.y;
    p.theta = state.odom.theta;
    portEXIT_CRITICAL(&stateMux);
    return p;
}

inline PoseSnapshot getMapPose() {
    PoseSnapshot p;
    portENTER_CRITICAL(&stateMux);
    p.x = state.map.x;
    p.y = state.map.y;
    p.theta = state.map.theta;
    portEXIT_CRITICAL(&stateMux);
    return p;
}

// Apply TF: map = odom ⊕ tf
inline void applyTf() {
    // Keep translation in the fixed map frame. Rotating absolute odom around
    // the world origin makes the displayed map jump when dTheta changes.
    state.map.x     = state.odom.x + state.tf.dx;
    state.map.y     = state.odom.y + state.tf.dy;
    state.map.theta = state.odom.theta + state.tf.dTheta;
    // Normalize angle
    state.map.theta = atan2f(sinf(state.map.theta), cosf(state.map.theta));
}

// Weighted TF update (from ICP/CSM corrections)
inline void updateTf(float dx, float dy, float dtheta, float weight) {
    const float maxStepXY = 0.03f;      // meters per scan update
    const float maxStepTheta = 0.035f;  // about 2 deg per scan update
    const float stepX = constrain(dx * weight, -maxStepXY, maxStepXY);
    const float stepY = constrain(dy * weight, -maxStepXY, maxStepXY);
    const float stepTheta = constrain(dtheta * weight, -maxStepTheta, maxStepTheta);

    portENTER_CRITICAL(&stateMux);
    state.tf.dx     += stepX;
    state.tf.dy     += stepY;
    state.tf.dTheta += stepTheta;
    state.tf.dTheta = atan2f(sinf(state.tf.dTheta), cosf(state.tf.dTheta));
    applyTf();
    portEXIT_CRITICAL(&stateMux);
}

#endif // ROBOT_STATE_H
