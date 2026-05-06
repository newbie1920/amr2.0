#pragma once

#include <vector>
#include "sim_world.h"
#include "lidar_mapper.h"

struct SimLidarConfig {
    int numRays = 360;
    float maxRange = 8.0f;     // meters
    float minRange = 0.02f;    // meters
    float noiseStdDev = 0.005f;// meters (5mm)
    float failRate = 0.02f;    // 2%
};

class SimLidar {
private:
    SimLidarConfig config;
    float angularResRad;

    float raySegmentIntersect(float px, float py, float dx, float dy, float x1, float y1, float x2, float y2) const;
    float gaussianNoise(float stdDev) const;

public:
    SimLidar(const SimLidarConfig& cfg = SimLidarConfig());

    // Returns an array of LidarPoints suitable for the OccupancyGridMapper
    std::vector<LidarPoint> scan(float robotX, float robotY, float robotTheta, const std::vector<Segment>& segments) const;
};
