#include "pathfinder.h"
#include <esp_heap_caps.h>
#include <cmath>
#include <algorithm>
#include <queue>
#include <functional>

AStarPathfinder::AStarPathfinder()
    : mapWidth(0), mapHeight(0), mapResolution(0.1f),
      staticMap(nullptr), nodePool(nullptr), nodeIdx(nullptr) {}

AStarPathfinder::~AStarPathfinder() {
    if (staticMap) heap_caps_free(staticMap);
    if (nodePool)  heap_caps_free(nodePool);
    if (nodeIdx)   heap_caps_free(nodeIdx);
}

void AStarPathfinder::init(int width, int height, float resolution) {
    if (staticMap) heap_caps_free(staticMap);
    if (nodePool)  heap_caps_free(nodePool);
    if (nodeIdx)   heap_caps_free(nodeIdx);

    mapWidth     = width;
    mapHeight    = height;
    mapResolution = resolution;

    int mapSize = width * height;

    // ── Static map in PSRAM ─────────────────────────────────
    staticMap = (uint8_t*)heap_caps_malloc(mapSize, MALLOC_CAP_SPIRAM);
    if (!staticMap) {
        staticMap = (uint8_t*)malloc(mapSize);
        Serial.println("[A*] Warning: PSRAM alloc failed for static map. Using SRAM.");
    } else {
        Serial.println("[A*] Allocated Static Map in PSRAM.");
    }
    if (staticMap) memset(staticMap, 0, mapSize);

    // ── Node pool in PSRAM ───────────────────────────────────
    nodePool = (PathNode*)heap_caps_malloc(sizeof(PathNode) * PATHFINDER_MAX_NODES, MALLOC_CAP_SPIRAM);
    if (!nodePool) nodePool = (PathNode*)malloc(sizeof(PathNode) * PATHFINDER_MAX_NODES);

    // ── O(1) lookup table: nodeIdx[gy*width+gx] = pool index (-1 = unused) ──
    // Stored as int32 in PSRAM.  mapSize * 4 bytes ≈ 160 KB for 200×200.
    nodeIdx = (int32_t*)heap_caps_malloc(mapSize * sizeof(int32_t), MALLOC_CAP_SPIRAM);
    if (!nodeIdx) nodeIdx = (int32_t*)malloc(mapSize * sizeof(int32_t));
    if (nodeIdx) memset(nodeIdx, 0xFF, mapSize * sizeof(int32_t)); // 0xFFFFFFFF → -1
}

void AStarPathfinder::updateStaticMap(const uint8_t* mapData, int length, uint32_t offset) {
    int maxLen  = mapWidth * mapHeight;
    if (offset >= (uint32_t)maxLen) return;
    int copyLen = (length + offset <= (uint32_t)maxLen) ? length : (maxLen - offset);
    if (staticMap && copyLen > 0) {
        memcpy(staticMap + offset, mapData, copyLen);
        Serial.printf("[A*] Static map chunk updated (offset: %u, %d bytes)\n", offset, copyLen);
    }
}

void AStarPathfinder::setDynamicObstacle(float cx, float cy, float radius_m) {
    dynamicObstacles.push_back({cx, cy, radius_m});
}

void AStarPathfinder::clearDynamicObstacles() {
    dynamicObstacles.clear();
}

void AStarPathfinder::setSlamMap(const OccupancyGridMapper* mapper) {
    this->slamMap = mapper;
}

int AStarPathfinder::gridToIndex(int gx, int gy) const {
    if (gx < 0 || gx >= mapWidth || gy < 0 || gy >= mapHeight) return -1;
    return gy * mapWidth + gx;
}

uint8_t AStarPathfinder::getCombinedCost(int gx, int gy) const {
    int idx = gridToIndex(gx, gy);
    if (idx < 0) return PATHFINDER_COST_LETHAL;

    uint8_t cost = 0;
    uint8_t slam_cost = 0;
    uint8_t static_cost = staticMap ? staticMap[idx] : 0;

    // Use SLAM map if available, with dynamic inflation
    if (slamMap) {
        int max_logodds = -100;
        int center_logodds = -100;
        int check_radius = 3; // 3 cells = 0.3m inflation radius (sync with DWA INFLATE_CELLS=4)
        
        if (gx >= 0 && gx < GRID_SIZE && gy >= 0 && gy < GRID_SIZE) {
            center_logodds = slamMap->grid_cell_const(gx, gy);
        }

        for (int dy = -check_radius; dy <= check_radius; dy++) {
            for (int dx = -check_radius; dx <= check_radius; dx++) {
                int nx = gx + dx;
                int ny = gy + dy;
                // SLAM grid is GRID_SIZE x GRID_SIZE
                if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
                    if (slamMap->grid_cell_const(nx, ny) > max_logodds) {
                        max_logodds = slamMap->grid_cell_const(nx, ny);
                    }
                }
            }
        }
        
        if (center_logodds >= 10) {
            slam_cost = PATHFINDER_COST_LETHAL; // Actual wall is Lethal
        } else if (max_logodds >= 10) {
            slam_cost = 180; // Inflation zone cost (Giảm từ 253 xuống 180 để A* dám đi qua khe hẹp nếu cần)
        } else if (max_logodds == 0) {
            slam_cost = 50; // Unknown space
        } else {
            slam_cost = 0;  // Free space
        }
    }

    // Combine costs (take the maximum obstacle value)
    cost = std::max(slam_cost, static_cost);

    // Dynamic obstacles: convert world→grid using origin offset when SLAM map present
    float worldX, worldY;
    if (slamMap) {
        worldX = slamMap->grid_to_world_x(gx);
        worldY = slamMap->grid_to_world_y(gy);
    } else {
        worldX = gx * mapResolution;
        worldY = gy * mapResolution;
    }

    for (const auto& obs : dynamicObstacles) {
        float dx   = worldX - obs.x;
        float dy   = worldY - obs.y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist <= obs.radius) {
            return PATHFINDER_COST_LETHAL;
        } else if (dist <= obs.radius * 2.0f) {
            uint8_t dyn_cost = (uint8_t)(253.0f * (1.0f - (dist - obs.radius) / obs.radius));
            if (dyn_cost > cost) cost = dyn_cost;
        }
    }
    return cost;
}

float AStarPathfinder::heuristic(int x1, int y1, int x2, int y2) const {
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    return 1.0f * (dx + dy) + (1.414f - 2.0f) * std::min(dx, dy);
}

void AStarPathfinder::gridToWorld(int gx, int gy, float& wx, float& wy) const {
    if (slamMap) {
        wx = slamMap->grid_to_world_x(gx);
        wy = slamMap->grid_to_world_y(gy);
    } else {
        wx = gx * mapResolution;
        wy = gy * mapResolution;
    }
}

bool AStarPathfinder::lineOfSightClear(int x0, int y0, int x1, int y1, uint8_t safetyThreshold) const {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;

    for (;;) {
        if (getCombinedCost(x, y) >= safetyThreshold) return false;
        if (x == x1 && y == y1) return true;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
        if (x < 0 || x >= mapWidth || y < 0 || y >= mapHeight) return false;
    }
}

void AStarPathfinder::fillDebugRaw(PathPlanDebug* debug, const int* pathX, const int* pathY, int rawLen) const {
    if (!debug || !debugPathEnabled || rawLen <= 0) return;

    debug->rawTotal = rawLen;
    int sampleCount = std::min(rawLen, PATHFINDER_DEBUG_MAX_RAW);
    debug->rawCount = sampleCount;
    debug->rawStride = (rawLen <= PATHFINDER_DEBUG_MAX_RAW)
        ? 1
        : (rawLen + PATHFINDER_DEBUG_MAX_RAW - 1) / PATHFINDER_DEBUG_MAX_RAW;

    for (int i = 0; i < sampleCount; i++) {
        int src = (sampleCount <= 1)
            ? 0
            : (int)roundf((float)i * (float)(rawLen - 1) / (float)(sampleCount - 1));
        src = constrain(src, 0, rawLen - 1);
        gridToWorld(pathX[src], pathY[src], debug->rawX[i], debug->rawY[i]);
    }
}

// ── Douglas-Peucker path simplification ──────────────────────
// Operates on the raw node-pool indices list to avoid extra allocation.
static float perpendicularDist(int px, int py, int ax, int ay, int bx, int by) {
    float abx = (float)(bx - ax), aby = (float)(by - ay);
    float apx = (float)(px - ax), apy = (float)(py - ay);
    float ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-6f) return sqrtf(apx * apx + apy * apy);
    float t = (apx * abx + apy * aby) / ab2;
    t = (t < 0.0f) ? 0.0f : (t > 1.0f ? 1.0f : t);
    float dx = apx - t * abx;
    float dy = apy - t * aby;
    return sqrtf(dx * dx + dy * dy);
}

// Simple iterative Douglas-Peucker; marks points to keep in `keep[]`.
static void dpSimplify(const int* xs, const int* ys, int start, int end,
                        float eps, bool* keep) {
    if (end <= start + 1) return;

    float maxDist  = 0.0f;
    int   maxIndex = start;
    for (int i = start + 1; i < end; i++) {
        float d = perpendicularDist(xs[i], ys[i], xs[start], ys[start], xs[end], ys[end]);
        if (d > maxDist) { maxDist = d; maxIndex = i; }
    }

    if (maxDist > eps) {
        dpSimplify(xs, ys, start, maxIndex, eps, keep);
        keep[maxIndex] = true;
        dpSimplify(xs, ys, maxIndex, end, eps, keep);
    }
}

int AStarPathfinder::computePath(float startX, float startY,
                                  float goalX,  float goalY,
                                  Waypoint* outPath, int maxWaypoints,
                                  PathPlanDebug* debug) {
    if (debug) {
        *debug = PathPlanDebug{};
        debug->planner = plannerModeName();
        debug->debugPath = debugPathEnabled;
        debug->goalX = goalX;
        debug->goalY = goalY;
    }

    if (!isInitialized() || !nodePool || !nodeIdx) {
        if (debug) debug->reason = "not_initialized";
        return 0;
    }

    // Convert world → grid coordinates
    // When using SLAM map, use its origin-aware helpers for correct conversion
    int startGX, startGY, goalGX, goalGY;
    if (slamMap) {
        startGX = slamMap->world_to_grid_x(startX);
        startGY = slamMap->world_to_grid_y(startY);
        goalGX  = slamMap->world_to_grid_x(goalX);
        goalGY  = slamMap->world_to_grid_y(goalY);
    } else {
        startGX = (int)(startX / mapResolution);
        startGY = (int)(startY / mapResolution);
        goalGX  = (int)(goalX  / mapResolution);
        goalGY  = (int)(goalY  / mapResolution);
    }

    // Clamp to map bounds
    startGX = std::max(0, std::min(mapWidth  - 1, startGX));
    startGY = std::max(0, std::min(mapHeight - 1, startGY));
    goalGX  = std::max(0, std::min(mapWidth  - 1, goalGX));
    goalGY  = std::max(0, std::min(mapHeight - 1, goalGY));

    if (getCombinedCost(goalGX, goalGY) >= PATHFINDER_COST_LETHAL) {
        Serial.println("[A*] Goal is in lethal zone!");
        if (debug) debug->reason = "goal_lethal";
        return 0;
    }

    // ── Reset node pool & lookup table ──────────────────────
    int mapSize = mapWidth * mapHeight;
    memset(nodePool, 0, sizeof(PathNode) * PATHFINDER_MAX_NODES);
    memset(nodeIdx, 0xFF, mapSize * sizeof(int32_t)); // -1 means unvisited

    // ── Priority queue: min-heap by f_cost ──────────────────
    // pair<f_cost * 1000 (int), pool_index>
    using PQEntry = std::pair<int, int>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> openHeap;

    int nodeCount = 0;

    // Push start node
    {
        int si = nodeCount++;
        nodePool[si].x          = startGX;
        nodePool[si].y          = startGY;
        nodePool[si].g_cost     = 0.0f;
        nodePool[si].f_cost     = heuristic(startGX, startGY, goalGX, goalGY);
        nodePool[si].parent_idx = -1;
        nodePool[si].opened     = true;
        nodeIdx[gridToIndex(startGX, startGY)] = si;
        openHeap.push({(int)(nodePool[si].f_cost * 1000.0f), si});
    }

    int bestNodeIdx = -1;

    const float SQRT2 = 1.414f;
    const int dirs[8][2] = {
        { 1,  0}, { 0,  1}, {-1,  0}, { 0, -1},
        { 1,  1}, {-1,  1}, {-1, -1}, { 1, -1}
    };

    while (!openHeap.empty()) {
        if (nodeCount >= PATHFINDER_MAX_NODES) {
            Serial.println("[A*] Max nodes reached.");
            break;
        }

        auto [_f, currentIdx] = openHeap.top();
        openHeap.pop();

        PathNode& current = nodePool[currentIdx];
        if (current.closed) continue; // Stale entry
        current.closed = true;

        if (current.x == goalGX && current.y == goalGY) {
            bestNodeIdx = currentIdx;
            break;
        }

        for (int i = 0; i < 8; i++) {
            int nx = current.x + dirs[i][0];
            int ny = current.y + dirs[i][1];

            if (nx < 0 || nx >= mapWidth || ny < 0 || ny >= mapHeight) continue;

            // Diagonal: skip if either cardinal neighbor is lethal (corner-cutting)
            if (i >= 4) {
                if (getCombinedCost(current.x + dirs[i][0], current.y) >= PATHFINDER_COST_LETHAL) continue;
                if (getCombinedCost(current.x, current.y + dirs[i][1]) >= PATHFINDER_COST_LETHAL) continue;
            }

            uint8_t cellCost = getCombinedCost(nx, ny);
            if (cellCost >= 253) continue;

            float moveCost = (i < 4) ? 1.0f : SQRT2;
            float penalty  = (cellCost / 254.0f) * 5.0f;
            float new_g    = current.g_cost + moveCost + penalty;

            // O(1) neighbor lookup
            int mapI       = gridToIndex(nx, ny);
            int neighborIdx = nodeIdx[mapI];

            if (neighborIdx != -1) {
                // Node exists
                PathNode& nb = nodePool[neighborIdx];
                if (nb.closed) continue;
                if (new_g < nb.g_cost) {
                    nb.g_cost     = new_g;
                    nb.f_cost     = new_g + heuristic(nx, ny, goalGX, goalGY);
                    nb.parent_idx = currentIdx;
                    // Re-insert (lazy deletion: old entry ignored via closed flag)
                    openHeap.push({(int)(nb.f_cost * 1000.0f), neighborIdx});
                }
            } else if (nodeCount < PATHFINDER_MAX_NODES) {
                int ni               = nodeCount++;
                nodePool[ni].x       = nx;
                nodePool[ni].y       = ny;
                nodePool[ni].g_cost  = new_g;
                nodePool[ni].f_cost  = new_g + heuristic(nx, ny, goalGX, goalGY);
                nodePool[ni].parent_idx = currentIdx;
                nodePool[ni].opened  = true;
                nodeIdx[mapI]        = ni;
                openHeap.push({(int)(nodePool[ni].f_cost * 1000.0f), ni});
            }
        }
    }

    if (bestNodeIdx == -1) {
        Serial.println("[A*] No path found.");
        if (debug) debug->reason = "no_path";
        return 0;
    }

    // ── Reconstruct raw path ─────────────────────────────────
    // PSRAM-backed path buffers (50000 × int = 200KB each — too large for DRAM)
    static int* pathX = nullptr;
    static int* pathY = nullptr;
    if (!pathX) {
        pathX = (int*)heap_caps_malloc(PATHFINDER_MAX_NODES * sizeof(int), MALLOC_CAP_SPIRAM);
        pathY = (int*)heap_caps_malloc(PATHFINDER_MAX_NODES * sizeof(int), MALLOC_CAP_SPIRAM);
        if (!pathX || !pathY) {
            Serial.println("[A*] PSRAM alloc failed for path!");
            if (debug) debug->reason = "alloc_path_failed";
            return 0;
        }
    }
    int rawLen = 0;

    {
        // Collect indices from goal → start
        static int* tmp = nullptr;
        if (!tmp) {
            tmp = (int*)heap_caps_malloc(PATHFINDER_MAX_NODES * sizeof(int), MALLOC_CAP_SPIRAM);
            if (!tmp) {
                Serial.println("[A*] PSRAM alloc failed for tmp!");
                if (debug) debug->reason = "alloc_tmp_failed";
                return 0;
            }
        }
        int tmpLen = 0;
        int curr   = bestNodeIdx;
        while (curr != -1 && tmpLen < PATHFINDER_MAX_NODES) {
            tmp[tmpLen++] = curr;
            curr = nodePool[curr].parent_idx;
        }
        // Reverse to get start → goal
        for (int i = tmpLen - 1; i >= 0; i--) {
            pathX[rawLen] = nodePool[tmp[i]].x;
            pathY[rawLen] = nodePool[tmp[i]].y;
            rawLen++;
        }
    }

    fillDebugRaw(debug, pathX, pathY, rawLen);
    if (rawLen <= 0) {
        if (debug) debug->reason = "empty_raw_path";
        return 0;
    }

    // ── Douglas-Peucker or LOS smoothing ─────────────────────
    const float DP_EPS = 1.5f;
    static bool* keep = nullptr;
    if (!keep) {
        keep = (bool*)heap_caps_malloc(PATHFINDER_MAX_NODES * sizeof(bool), MALLOC_CAP_SPIRAM);
        if (!keep) {
            Serial.println("[A*] PSRAM alloc failed for keep!");
            if (debug) debug->reason = "alloc_keep_failed";
            return 0;
        }
    }
    memset(keep, 0, rawLen * sizeof(bool));
    keep[0]        = true;
    keep[rawLen-1] = true;
    if (plannerMode == PLANNER_THETA_LOS && rawLen > 2) {
        int anchor = 0;
        while (anchor < rawLen - 1) {
            int farthest = anchor + 1;
            for (int j = rawLen - 1; j > anchor + 1; j--) {
                if (lineOfSightClear(pathX[anchor], pathY[anchor], pathX[j], pathY[j], 180)) {
                    farthest = j;
                    break;
                }
            }
            keep[farthest] = true;
            anchor = farthest;
        }
    } else {
        dpSimplify(pathX, pathY, 0, rawLen - 1, DP_EPS, keep);
    }

    // ── Build Waypoint output ────────────────────────────────
    int wpCount = 0;
    int prevX = pathX[0], prevY = pathY[0];

    for (int i = 0; i < rawLen && wpCount < maxWaypoints; i++) {
        if (!keep[i]) continue;
        int cx = pathX[i], cy = pathY[i];
        float dx = (float)(cx - prevX);
        float dy = (float)(cy - prevY);
        float hdg = (dx != 0.0f || dy != 0.0f) ? atan2f(dy, dx) : 0.0f;

        gridToWorld(cx, cy, outPath[wpCount].x, outPath[wpCount].y);
        outPath[wpCount].heading    = hdg;
        outPath[wpCount].useReverse = false;

        prevX = cx; prevY = cy;
        wpCount++;
    }

    Serial.printf("[A*] Path found! Raw: %d nodes → Smoothed: %d WPs (used %d pool nodes)\n",
                  rawLen, wpCount, nodeCount);
    if (wpCount > 0) {
        outPath[wpCount - 1].x = goalX;
        outPath[wpCount - 1].y = goalY;
    }
    if (debug) {
        debug->ok = wpCount > 0;
        debug->reason = wpCount > 0 ? "ok" : "empty_waypoints";
    }
    return wpCount;
}
