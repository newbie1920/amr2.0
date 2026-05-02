#include "pathfinder.h"
#include <esp_heap_caps.h>
#include <cmath>
#include <algorithm>

AStarPathfinder::AStarPathfinder() : mapWidth(0), mapHeight(0), mapResolution(0.1f), staticMap(nullptr), nodePool(nullptr) {
}

AStarPathfinder::~AStarPathfinder() {
    if (staticMap) heap_caps_free(staticMap);
    if (nodePool) heap_caps_free(nodePool);
}

void AStarPathfinder::init(int width, int height, float resolution) {
    if (staticMap) heap_caps_free(staticMap);
    if (nodePool) heap_caps_free(nodePool);
    
    mapWidth = width;
    mapHeight = height;
    mapResolution = resolution;
    
    int mapSize = width * height;
    
    // Allocate static map in PSRAM if possible, otherwise SRAM
    staticMap = (uint8_t*)heap_caps_malloc(mapSize, MALLOC_CAP_SPIRAM);
    if (!staticMap) {
        staticMap = (uint8_t*)malloc(mapSize);
        Serial.println("[A*] Warning: PSRAM alloc failed for static map. Using SRAM.");
    } else {
        Serial.println("[A*] Allocated Static Map in PSRAM.");
    }
    
    if (staticMap) {
        memset(staticMap, 0, mapSize);
    }

    // Allocate node pool
    nodePool = (PathNode*)heap_caps_malloc(sizeof(PathNode) * PATHFINDER_MAX_NODES, MALLOC_CAP_SPIRAM);
    if (!nodePool) {
        nodePool = (PathNode*)malloc(sizeof(PathNode) * PATHFINDER_MAX_NODES);
    }
}

void AStarPathfinder::updateStaticMap(const uint8_t* mapData, int length) {
    int maxLen = mapWidth * mapHeight;
    int copyLen = (length < maxLen) ? length : maxLen;
    if (staticMap) {
        memcpy(staticMap, mapData, copyLen);
        Serial.printf("[A*] Static map updated (%d bytes)\n", copyLen);
    }
}

void AStarPathfinder::setDynamicObstacle(float cx, float cy, float radius_m) {
    dynamicObstacles.push_back({cx, cy, radius_m});
}

void AStarPathfinder::clearDynamicObstacles() {
    dynamicObstacles.clear();
}

int AStarPathfinder::gridToIndex(int gx, int gy) const {
    if (gx < 0 || gx >= mapWidth || gy < 0 || gy >= mapHeight) return -1;
    return gy * mapWidth + gx;
}

uint8_t AStarPathfinder::getCombinedCost(int gx, int gy) const {
    int idx = gridToIndex(gx, gy);
    if (idx < 0) return PATHFINDER_COST_LETHAL; // Out of bounds
    
    uint8_t cost = staticMap ? staticMap[idx] : 0;
    
    // Inflate cost based on dynamic obstacles
    float worldX = gx * mapResolution;
    float worldY = gy * mapResolution;
    
    for (const auto& obs : dynamicObstacles) {
        float dx = worldX - obs.x;
        float dy = worldY - obs.y;
        float dist = sqrtf(dx*dx + dy*dy);
        if (dist <= obs.radius) {
            return PATHFINDER_COST_LETHAL;
        } else if (dist <= obs.radius * 2.0f) {
            // Inflation zone
            uint8_t dyn_cost = (uint8_t)(253.0f * (1.0f - (dist - obs.radius) / obs.radius));
            if (dyn_cost > cost) cost = dyn_cost;
        }
    }
    return cost;
}

float AStarPathfinder::heuristic(int x1, int y1, int x2, int y2) const {
    // Octile distance
    int dx = std::abs(x1 - x2);
    int dy = std::abs(y1 - y2);
    return 1.0f * (dx + dy) + (1.414f - 2.0f) * std::min(dx, dy);
}

int AStarPathfinder::computePath(float startX, float startY, float goalX, float goalY, Waypoint* outPath, int maxWaypoints) {
    if (!staticMap || !nodePool) return 0;
    
    int startGX = startX / mapResolution;
    int startGY = startY / mapResolution;
    int goalGX = goalX / mapResolution;
    int goalGY = goalY / mapResolution;
    
    if (getCombinedCost(goalGX, goalGY) >= PATHFINDER_COST_LETHAL) {
        Serial.println("[A*] Goal is in lethal zone!");
        return 0;
    }

    // Reset node pool (simple array implementation for speed)
    memset(nodePool, 0, sizeof(PathNode) * PATHFINDER_MAX_NODES);
    
    std::vector<int> openList;
    openList.reserve(100);
    
    int nodeCount = 0;
    int startIdx = nodeCount++;
    nodePool[startIdx].x = startGX;
    nodePool[startIdx].y = startGY;
    nodePool[startIdx].g_cost = 0;
    nodePool[startIdx].f_cost = heuristic(startGX, startGY, goalGX, goalGY);
    nodePool[startIdx].parent_idx = -1;
    nodePool[startIdx].opened = true;
    
    openList.push_back(startIdx);
    
    int bestNodeIdx = -1;
    
    // 8-way movements: (dx, dy, cost)
    const float SQRT2 = 1.414f;
    const int dirs[8][2] = {
        {1, 0}, {0, 1}, {-1, 0}, {0, -1},
        {1, 1}, {-1, 1}, {-1, -1}, {1, -1}
    };
    
    while (!openList.empty()) {
        if (nodeCount >= PATHFINDER_MAX_NODES) {
            Serial.println("[A*] Max nodes reached.");
            break;
        }
        
        // Find lowest f_cost
        int currentOpenIdx = 0;
        for (int i = 1; i < openList.size(); i++) {
            if (nodePool[openList[i]].f_cost < nodePool[openList[currentOpenIdx]].f_cost) {
                currentOpenIdx = i;
            }
        }
        
        int currentIdx = openList[currentOpenIdx];
        openList.erase(openList.begin() + currentOpenIdx);
        
        PathNode& current = nodePool[currentIdx];
        current.closed = true;
        
        if (current.x == goalGX && current.y == goalGY) {
            bestNodeIdx = currentIdx;
            break; // Found path
        }
        
        for (int i = 0; i < 8; i++) {
            int nx = current.x + dirs[i][0];
            int ny = current.y + dirs[i][1];
            float moveCost = (i < 4) ? 1.0f : SQRT2;
            
            uint8_t cellCost = getCombinedCost(nx, ny);
            if (cellCost >= 253) continue; // Lethal or near-lethal obstacle
            
            float penalty = (cellCost / 254.0f) * 5.0f; // Add penalty for close obstacles
            float new_g = current.g_cost + moveCost + penalty;
            
            // Find if neighbor exists in nodePool
            int neighborIdx = -1;
            for (int j = 0; j < nodeCount; j++) {
                if (nodePool[j].x == nx && nodePool[j].y == ny) {
                    neighborIdx = j;
                    break;
                }
            }
            
            if (neighborIdx != -1) {
                if (nodePool[neighborIdx].closed) continue;
                if (new_g < nodePool[neighborIdx].g_cost) {
                    nodePool[neighborIdx].g_cost = new_g;
                    nodePool[neighborIdx].f_cost = new_g + heuristic(nx, ny, goalGX, goalGY);
                    nodePool[neighborIdx].parent_idx = currentIdx;
                }
            } else {
                if (nodeCount < PATHFINDER_MAX_NODES) {
                    neighborIdx = nodeCount++;
                    nodePool[neighborIdx].x = nx;
                    nodePool[neighborIdx].y = ny;
                    nodePool[neighborIdx].g_cost = new_g;
                    nodePool[neighborIdx].f_cost = new_g + heuristic(nx, ny, goalGX, goalGY);
                    nodePool[neighborIdx].parent_idx = currentIdx;
                    nodePool[neighborIdx].opened = true;
                    openList.push_back(neighborIdx);
                }
            }
        }
    }
    
    if (bestNodeIdx == -1) {
        Serial.println("[A*] No path found.");
        return 0;
    }
    
    // Reconstruct path
    std::vector<int> pathIndices;
    int curr = bestNodeIdx;
    while (curr != -1) {
        pathIndices.push_back(curr);
        curr = nodePool[curr].parent_idx;
    }
    
    // Path is from goal to start, reverse it and apply to output
    int wpCount = 0;
    for (int i = pathIndices.size() - 1; i >= 0 && wpCount < maxWaypoints; i--) {
        // Simple smoothing/downsampling could be added here
        outPath[wpCount].x = nodePool[pathIndices[i]].x * mapResolution;
        outPath[wpCount].y = nodePool[pathIndices[i]].y * mapResolution;
        outPath[wpCount].useReverse = false; // Default
        
        // Calculate heading towards next point
        if (i > 0) {
            float dx = nodePool[pathIndices[i-1]].x - nodePool[pathIndices[i]].x;
            float dy = nodePool[pathIndices[i-1]].y - nodePool[pathIndices[i]].y;
            outPath[wpCount].heading = atan2f(dy, dx);
        } else if (wpCount > 0) {
            outPath[wpCount].heading = outPath[wpCount-1].heading;
        } else {
            outPath[wpCount].heading = 0;
        }
        
        wpCount++;
    }
    
    Serial.printf("[A*] Path found! Nodes: %d, WPs: %d\n", nodeCount, wpCount);
    return wpCount;
}
