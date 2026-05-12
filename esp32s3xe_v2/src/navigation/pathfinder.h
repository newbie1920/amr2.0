#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
#include <vector>
#include "navigator.h"
#include "occupancy_grid.h"

#define PATHFINDER_MAX_NODES       4000
#define PATHFINDER_COST_LETHAL     254
#define PATHFINDER_COST_UNKNOWN    255

struct PathNode {
    int16_t x, y;
    float g_cost, f_cost;
    int16_t parent_idx;
    bool closed, opened;
};

class AStarPathfinder {
public:
    AStarPathfinder();
    ~AStarPathfinder();
    void init(int width, int height, float resolution);
    void updateStaticMap(const uint8_t* mapData, int length, uint32_t offset = 0);
    int computePath(float startX, float startY, float goalX, float goalY,
                    Waypoint* outPath, int maxWaypoints);
    void setDynamicObstacle(float cx, float cy, float radius_m);
    void clearDynamicObstacles();
    void setSlamMap(const OccupancyGridMapper* mapper);

    int   getMapWidth()      const { return mapWidth; }
    int   getMapHeight()     const { return mapHeight; }
    float getMapResolution() const { return mapResolution; }
    bool  isInitialized()    const { return staticMap != nullptr || slamMap != nullptr; }

private:
    int mapWidth, mapHeight;
    float mapResolution;
    uint8_t* staticMap;
    const OccupancyGridMapper* slamMap = nullptr;
    PathNode* nodePool;
    int32_t* nodeIdx;
    struct DynObs { float x, y, radius; };
    std::vector<DynObs> dynamicObstacles;

    int gridToIndex(int gx, int gy) const;
    uint8_t getCombinedCost(int gx, int gy) const;
    float heuristic(int x1, int y1, int x2, int y2) const;
};

#endif
