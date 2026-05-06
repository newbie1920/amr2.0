#include "sim_engine.h"
#include <cmath>
#include <algorithm>
#include "esp_log.h"

static const char* TAG = "SimEngine";

SimEngine::SimEngine(SimMapType mapType) : running(false), stepCount(0), simTime(0.0f) {
    // Load selected map
    if (mapType == SIM_MAP_SLAM_TEST) {
        world.loadSlamTestMap();
        ESP_LOGI(TAG, "Using SLAM Test Map");
    } else {
        // Warehouse map is the default from constructor
        ESP_LOGI(TAG, "Using Warehouse Map");
    }
    reset();
}

void SimEngine::start() {
    running = true;
    ESP_LOGI(TAG, "SimEngine started");
}

void SimEngine::stop() {
    running = false;
    targetVel = {0, 0};
    vel = {0, 0};
    ESP_LOGI(TAG, "SimEngine stopped");
}

void SimEngine::reset(float spawnX, float spawnY, float spawnTheta) {
    pose = {spawnX, spawnY, spawnTheta};
    odom = {0, 0, 0};
    vel = {0, 0};
    targetVel = {0, 0};
    stepCount = 0;
    simTime = 0.0f;
    
    lastLidarScan = lidar.scan(pose.x, pose.y, pose.theta, world.getSegments());
    ESP_LOGI(TAG, "Reset to (%.2f, %.2f, %.2f)", pose.x, pose.y, pose.theta);
}

void SimEngine::reset() {
    SpawnPoint spawn = world.getDefaultSpawn();
    reset(spawn.x, spawn.y, spawn.theta);
}

void SimEngine::setVelocity(float linear, float angular) {
    targetVel.v = std::max(-MAX_LINEAR_VEL, std::min(MAX_LINEAR_VEL, linear));
    targetVel.w = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, angular));
}

float SimEngine::rampTo(float current, float target, float maxStep) {
    float diff = target - current;
    if (std::abs(diff) <= maxStep) return target;
    return current + ((diff > 0) ? maxStep : -maxStep);
}

void SimEngine::step() {
    if (!running) return;

    float dt = PHYSICS_DT;
    stepCount++;
    simTime += dt;

    // 1. Acceleration
    vel.v = rampTo(vel.v, targetVel.v, LINEAR_ACCEL * dt);
    vel.w = rampTo(vel.w, targetVel.w, ANGULAR_ACCEL * dt);

    // 2. Kinematics
    float newTheta = pose.theta + vel.w * dt;
    float newX = pose.x + vel.v * std::cos(pose.theta) * dt;
    float newY = pose.y + vel.v * std::sin(pose.theta) * dt;

    // Normalize theta
    while (newTheta > M_PI) newTheta -= 2 * M_PI;
    while (newTheta < -M_PI) newTheta += 2 * M_PI;

    // 3. Collision
    bool isCollided = world.checkCollision(newX, newY, ROBOT_RADIUS, newTheta);
    if (!isCollided) {
        pose.x = newX;
        pose.y = newY;
        pose.theta = newTheta;
    } else {
        // Collision -> stop linear, allow angular
        vel.v = 0;
        pose.theta = newTheta;
    }

    // 4. Odometry
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    float nV = ODOM_NOISE_LINEAR * std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * M_PI * u2);
    
    u1 = (float)rand() / RAND_MAX;
    u2 = (float)rand() / RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    float nW = ODOM_NOISE_ANGULAR * std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * M_PI * u2);

    odom.theta += (vel.w + nW) * dt;
    odom.x += (vel.v + nV) * std::cos(odom.theta) * dt;
    odom.y += (vel.v + nV) * std::sin(odom.theta) * dt;

    while (odom.theta > M_PI) odom.theta -= 2 * M_PI;
    while (odom.theta < -M_PI) odom.theta += 2 * M_PI;

    // 5. Lidar Scan
    if (stepCount % LIDAR_EVERY_N_STEPS == 0) {
        lastLidarScan = lidar.scan(pose.x, pose.y, pose.theta, world.getSegments());
    }
}

void SimEngine::buildTelemetry(SimTelemetry& t) {
    t.pose = pose;
    t.odom = odom;
    t.vel = vel;
    t.lidar = lastLidarScan;
    t.simTime = simTime;
    t.stepCount = stepCount;
    t.battery = 85;

    t.obs = false;
    for (const auto& pt : lastLidarScan) {
        if (pt.quality && (pt.angle >= 340.0f || pt.angle <= 20.0f)) {
            if (pt.distance < 0.25f) { // 25cm
                t.obs = true;
                break;
            }
        }
    }
}

SimTelemetry SimEngine::getTelemetry() {
    SimTelemetry t;
    buildTelemetry(t);
    return t;
}
