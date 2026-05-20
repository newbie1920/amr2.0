#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
#include <vector>
#include "navigator.h"
#include "occupancy_grid.h"

#define PATHFINDER_MAX_NODES       50000
#define PATHFINDER_COST_LETHAL     254
#define PATHFINDER_COST_UNKNOWN    255
#define PATHFINDER_DEBUG_MAX_RAW   256

struct PathNode {
    int16_t x, y;
    float g_cost, f_cost;
    int32_t parent_idx;
    bool closed, opened;
};

enum PlannerMode : uint8_t {
    PLANNER_ASTAR = 0,
    PLANNER_THETA_LOS = 1
};

struct PathPlanDebug {
    uint32_t planId = 0;
    bool ok = false;
    const char* reason = "idle";
    const char* planner = "astar";
    bool debugPath = false;
    float goalX = 0.0f;
    float goalY = 0.0f;
    int rawTotal = 0;
    int rawStride = 1;
    int rawCount = 0;
    float rawX[PATHFINDER_DEBUG_MAX_RAW];
    float rawY[PATHFINDER_DEBUG_MAX_RAW];
};

class AStarPathfinder {
public:
    AStarPathfinder();
    ~AStarPathfinder();
    void init(int width, int height, float resolution);
    void updateStaticMap(const uint8_t* mapData, int length, uint32_t offset = 0);
    int computePath(float startX, float startY, float goalX, float goalY,
                    Waypoint* outPath, int maxWaypoints,
                    PathPlanDebug* debug = nullptr);
    void setDynamicObstacle(float cx, float cy, float radius_m);
    void clearDynamicObstacles();
    void setSlamMap(const OccupancyGridMapper* mapper);
    void setPlannerMode(PlannerMode mode) { plannerMode = mode; }
    PlannerMode getPlannerMode() const { return plannerMode; }
    const char* plannerModeName() const { return plannerMode == PLANNER_THETA_LOS ? "theta_los" : "astar"; }
    void setDebugPathEnabled(bool enabled) { debugPathEnabled = enabled; }
    bool debugPathIsEnabled() const { return debugPathEnabled; }

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
    PlannerMode plannerMode = PLANNER_ASTAR;
    bool debugPathEnabled = false;

    int gridToIndex(int gx, int gy) const;
    uint8_t getCombinedCost(int gx, int gy) const;
    float heuristic(int x1, int y1, int x2, int y2) const;
    bool lineOfSightClear(int x0, int y0, int x1, int y1, uint8_t safetyThreshold) const;
    void gridToWorld(int gx, int gy, float& wx, float& wy) const;
    void fillDebugRaw(PathPlanDebug* debug, const int* pathX, const int* pathY, int rawLen) const;
};

#endif
