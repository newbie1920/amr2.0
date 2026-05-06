#include "sim_lidar.h"
#include <cmath>
#include <random>
#include <chrono>

SimLidar::SimLidar(const SimLidarConfig& cfg) : config(cfg) {
    angularResRad = (2.0f * M_PI) / config.numRays;
}

float SimLidar::raySegmentIntersect(float px, float py, float dx, float dy, float x1, float y1, float x2, float y2) const {
    float ex = x2 - x1;
    float ey = y2 - y1;

    float denom = dx * ey - dy * ex;
    if (std::abs(denom) < 1e-6f) return -1.0f; // Parallel

    float tx = ((x1 - px) * ey - (y1 - py) * ex) / denom;
    float sx = ((x1 - px) * dy - (y1 - py) * dx) / denom;

    if (tx >= 0.0f && sx >= 0.0f && sx <= 1.0f) {
        return tx;
    }

    return -1.0f;
}

float SimLidar::gaussianNoise(float stdDev) const {
    // Simple Box-Muller transform
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    return stdDev * std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * M_PI * u2);
}

std::vector<LidarPoint> SimLidar::scan(float robotX, float robotY, float robotTheta, const std::vector<Segment>& segments) const {
    std::vector<LidarPoint> points;
    points.reserve(config.numRays);

    for (int i = 0; i < config.numRays; i++) {
        float localAngleDeg = i * (360.0f / config.numRays);
        float localAngleRad = i * angularResRad;
        float worldAngleRad = robotTheta + localAngleRad;

        float dx = std::cos(worldAngleRad);
        float dy = std::sin(worldAngleRad);

        float minDist = config.maxRange;

        for (const auto& seg : segments) {
            float dist = raySegmentIntersect(robotX, robotY, dx, dy, seg.x1, seg.y1, seg.x2, seg.y2);
            if (dist >= 0.0f && dist >= config.minRange && dist < minDist) {
                minDist = dist;
            }
        }

        // Simulate failures
        float randVal = (float)rand() / RAND_MAX;
        if (randVal < config.failRate) continue;

        // Add noise
        if (minDist < config.maxRange) {
            minDist += gaussianNoise(config.noiseStdDev);
            minDist = std::max(config.minRange, minDist);
        }

        LidarPoint pt;
        pt.angle = std::round(localAngleDeg); // LD19 output is in degrees
        if (pt.angle >= 360.0f) pt.angle -= 360.0f;
        pt.distance = minDist; 
        pt.quality = (minDist > 0 && minDist < config.maxRange);

        points.push_back(pt);
    }

    return points;
}
