#pragma once

#include "sim_world.h"
#include "sim_lidar.h"

// Physics config
#define PHYSICS_HZ 50
#define PHYSICS_DT (1.0f / PHYSICS_HZ)
#define LIDAR_EVERY_N_STEPS 5

#define MAX_LINEAR_VEL 1.0f
#define MAX_ANGULAR_VEL 3.0f
#define LINEAR_ACCEL 2.0f
#define ANGULAR_ACCEL 5.0f

#define ODOM_NOISE_LINEAR 0.002f
#define ODOM_NOISE_ANGULAR 0.005f

enum SimMapType {
    SIM_MAP_WAREHOUSE,   // 4 shelves + chargers (warehouse.js default)
    SIM_MAP_SLAM_TEST    // 3-room test map with corridors & pillars
};

struct Pose {
    float x;
    float y;
    float theta;
};

struct Velocity {
    float v;
    float w;
};

struct SimTelemetry {
    Pose pose;
    Pose odom;
    Velocity vel;
    std::vector<LidarPoint> lidar;
    bool obs;
    int battery;
    float simTime;
    unsigned long stepCount;
};

class SimEngine {
private:
    SimWorld world;
    SimLidar lidar;

    Pose pose;
    Pose odom;
    Velocity vel;
    Velocity targetVel;

    bool running;
    unsigned long stepCount;
    float simTime;

    std::vector<LidarPoint> lastLidarScan;

    float rampTo(float current, float target, float maxStep);
    void buildTelemetry(SimTelemetry& telemetry);

public:
    SimEngine(SimMapType mapType = SIM_MAP_SLAM_TEST);

    void start();
    void stop();
    void reset(float spawnX, float spawnY, float spawnTheta);
    void reset();

    void setVelocity(float linear, float angular);
    
    // Call this at PHYSICS_HZ (50Hz)
    void step();

    SimWorld* getWorld() { return &world; }
    SimLidar* getLidar() { return &lidar; }

    SimTelemetry getTelemetry();
};
