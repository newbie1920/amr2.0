#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
#include <vector>
#include "navigator.h"
#include "lidar_mapper.h"

// ── Limits ────────────────────────────────────────────────────
#define PATHFINDER_MAX_NODES       4000   // Max open/closed nodes per search
#define PATHFINDER_COST_LETHAL     254
#define PATHFINDER_COST_UNKNOWN    255

// ── Node used in the pool ─────────────────────────────────────
struct PathNode {
    int16_t x, y;      // Grid coordinates (shrink to 16-bit → save RAM)
    float   g_cost;
    float   f_cost;
    int16_t parent_idx; // Index into nodePool (-1 = no parent); 16-bit → max 32767 nodes
    bool    closed;
    bool    opened;
};

class AStarPathfinder {
public:
    AStarPathfinder();
    ~AStarPathfinder();

    /**
     * Initialize with global map dimensions.
     * Allocates staticMap, nodePool, and nodeIdx lookup table — all in PSRAM if available.
     */
    void init(int width, int height, float resolution);

    /**
     * Load occupancy data from Web dashboard.
     * @param mapData  Raw byte array (0 = free, 100 = obstacle, 255 = unknown)
     * @param length   Number of bytes
     * @param offset   Offset into the static map array for chunked updates
     */
    void updateStaticMap(const uint8_t* mapData, int length, uint32_t offset = 0);

    /**
     * Run A* from (startX, startY) to (goalX, goalY) in world coordinates (meters).
     * Uses priority_queue min-heap + O(1) nodeIdx lookup + Douglas-Peucker smoothing.
     * @return Number of waypoints written to outPath (0 on failure).
     */
    int computePath(float startX, float startY,
                    float goalX,  float goalY,
                    Waypoint* outPath, int maxWaypoints);

    /** Register a dynamic obstacle (another robot) for A* cost inflation. */
    void setDynamicObstacle(float cx, float cy, float radius_m);
    void clearDynamicObstacles();

    /** Update SLAM map reference for real-time planning */
    void setSlamMap(const OccupancyGridMapper* mapper);

    // Map metadata accessors (used by network_comm to verify incoming map header)
    int   getMapWidth()      const { return mapWidth; }
    int   getMapHeight()     const { return mapHeight; }
    float getMapResolution() const { return mapResolution; }
    bool  isInitialized()    const { return staticMap != nullptr || slamMap != nullptr; }

private:
    int   mapWidth;
    int   mapHeight;
    float mapResolution;

    uint8_t*  staticMap; // Flat occupancy array [gy*width + gx]
    const OccupancyGridMapper* slamMap = nullptr; // Pointer to real-time SLAM map
    PathNode* nodePool;  // Fixed-size node memory pool
    int32_t*  nodeIdx;   // nodeIdx[gy*width + gx] = pool index, or -1 if unvisited

    struct DynObs { float x, y, radius; };
    std::vector<DynObs> dynamicObstacles;

    int     gridToIndex(int gx, int gy) const;
    uint8_t getCombinedCost(int gx, int gy) const;
    float   heuristic(int x1, int y1, int x2, int y2) const;
};

#endif // PATHFINDER_H
