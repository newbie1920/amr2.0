#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
#include <vector>
#include "navigator.h"

#define PATHFINDER_MAX_NODES 4000
#define PATHFINDER_COST_LETHAL 254
#define PATHFINDER_COST_UNKNOWN 255

struct PathNode {
    int x, y;
    float g_cost;
    float f_cost;
    int parent_idx;
    bool closed;
    bool opened;
};

class AStarPathfinder {
public:
    AStarPathfinder();
    ~AStarPathfinder();

    // Initialize with a global map dimension
    void init(int width, int height, float resolution);

    // Update static map data (received from Web)
    void updateStaticMap(const uint8_t* mapData, int length);

    // Compute path
    // Returns number of waypoints generated (0 if failed)
    int computePath(float startX, float startY, float goalX, float goalY, Waypoint* outPath, int maxWaypoints);

    // For traffic injection (dynamic obstacles from other robots)
    void setDynamicObstacle(float cx, float cy, float radius_m);
    void clearDynamicObstacles();

private:
    int mapWidth;
    int mapHeight;
    float mapResolution;
    
    // Map buffers (Using pointer to allocate in PSRAM if available)
    uint8_t* staticMap;
    
    struct DynObs {
        float x;
        float y;
        float radius;
    };
    std::vector<DynObs> dynamicObstacles;

    // Node memory pool to avoid dynamic allocation during search
    PathNode* nodePool;

    int gridToIndex(int gx, int gy) const;
    uint8_t getCombinedCost(int gx, int gy) const;
    float heuristic(int x1, int y1, int x2, int y2) const;
};

#endif // PATHFINDER_H
