/**
 * AMR 2.0 — Frontier Explorer (ESP32-S3)
 *
 * Tự động khám phá bản đồ bằng cách:
 *   1. Quét Occupancy Grid tìm "frontier cells" (ô FREE giáp UNKNOWN)
 *   2. Gom các frontier cells thành clusters (nhóm liên kề)
 *   3. Chọn cluster tốt nhất (lớn nhất + gần nhất)
 *   4. Gửi centroid của cluster đó vào pathfinderQueue như GoToRequest
 *   5. Khi navigator báo DONE → quay lại bước 1
 *   6. Khi không còn frontier nào → báo MAP COMPLETE
 *
 * SLAM Upgrade v2:
 *   - Uses mapper's origin-aware coordinate helpers
 *   - Centroid correctly computed in world frame
 *   - snapToSafe uses mapper coordinate system
 *
 * Thiết kế:
 *   - Header-only, zero malloc, static arrays
 *   - Chạy trên Core 0, gọi từ explorationTask (FreeRTOS)
 *   - Sử dụng trực tiếp gridMapper.grid[][] (128x128 log-odds)
 *   - BFS flood-fill để cluster frontiers
 */

#ifndef FRONTIER_EXPLORER_H
#define FRONTIER_EXPLORER_H

#include <Arduino.h>
#include <cmath>
#include <cstring>
#include "lidar_mapper.h"   // OccupancyGridMapper, GRID_SIZE, LOGODDS_*

// ============================================================
//   CONFIG
// ============================================================

#define FRONTIER_MAX_CELLS    512    // Max frontier cells phát hiện được
#define FRONTIER_MAX_CLUSTERS 20     // Max clusters
#define FRONTIER_MIN_CLUSTER_SIZE 3  // Bỏ qua clusters < 3 cells (nhiễu)
#define FRONTIER_SAFE_DIST_CELLS 3   // Đích phải cách obstacle ít nhất 3 cells (0.3m)
#define FRONTIER_MIN_GOAL_DIST 0.3f  // Không đặt goal quá gần robot (m)
#define FRONTIER_BLACKLIST_MAX 10    // Max số goal bị blacklist (unreachable)

// ============================================================
//   DATA TYPES
// ============================================================

struct FrontierCell {
    int16_t gx, gy;  // Grid coordinates (0..GRID_SIZE-1) — int16_t to handle grid > 127
};

struct FrontierCluster {
    float centroidX;    // World X (meters)
    float centroidY;    // World Y (meters)
    int   size;         // Số cells trong cluster
    float distToRobot;  // Khoảng cách tới robot (meters)
};

// ============================================================
//   FRONTIER EXPLORER CLASS
// ============================================================

class FrontierExplorer {
public:
    // ── State ─────────────────────────────────────────────────
    enum ExploreState {
        EXPLORE_IDLE,       // Chưa bắt đầu
        EXPLORE_SCANNING,   // Đang quét frontier
        EXPLORE_NAVIGATING, // Đang đi tới frontier goal
        EXPLORE_COMPLETE,   // Không còn frontier → bản đồ hoàn chỉnh
        EXPLORE_FAILED      // Không thể tới bất kỳ frontier nào
    };

    ExploreState state = EXPLORE_IDLE;
    float goalX = 0, goalY = 0;  // Current exploration goal
    int   clusterCount = 0;
    int   frontierCellCount = 0;
    int   exploredGoals = 0;     // Tổng số goals đã explore

    // Blacklist: goals mà A* không tìm được path
    float blacklistX[FRONTIER_BLACKLIST_MAX];
    float blacklistY[FRONTIER_BLACKLIST_MAX];
    int   blacklistCount = 0;

    // ── Public API ────────────────────────────────────────────

    void start() {
        state = EXPLORE_SCANNING;
        blacklistCount = 0;
        exploredGoals = 0;
        Serial.println("[EXPLORE] Started autonomous exploration");
    }

    void stop() {
        state = EXPLORE_IDLE;
        Serial.println("[EXPLORE] Stopped");
    }

    bool isExploring() const {
        return state == EXPLORE_SCANNING || state == EXPLORE_NAVIGATING;
    }

    const char* getStateName() const {
        switch (state) {
            case EXPLORE_IDLE:       return "IDLE";
            case EXPLORE_SCANNING:   return "SCAN";
            case EXPLORE_NAVIGATING: return "NAV";
            case EXPLORE_COMPLETE:   return "DONE";
            case EXPLORE_FAILED:     return "FAIL";
            default:                 return "???";
        }
    }

    /**
     * Quét grid, tìm frontier, chọn goal tốt nhất.
     *
     * @param mapper  Occupancy grid mapper (để đọc grid[][] + coordinate helpers)
     * @param robotX  Vị trí robot hiện tại (meters, world frame)
     * @param robotY  Vị trí robot hiện tại (meters, world frame)
     * @param outGoalX [OUT] X của goal tốt nhất (meters, world frame)
     * @param outGoalY [OUT] Y của goal tốt nhất (meters, world frame)
     * @return true nếu tìm được goal, false nếu không còn frontier
     */
    bool findNextGoal(
        const OccupancyGridMapper& mapper,
        float robotX, float robotY,
        float& outGoalX, float& outGoalY
    ) {
        // 1. Tìm tất cả frontier cells
        frontierCellCount = 0;
        detectFrontiers(mapper);

        if (frontierCellCount == 0) {
            state = EXPLORE_COMPLETE;
            Serial.println("[EXPLORE] No frontiers left — MAP COMPLETE!");
            return false;
        }

        // 2. Cluster frontiers bằng BFS flood-fill
        clusterCount = 0;
        clusterFrontiers(mapper, robotX, robotY);

        if (clusterCount == 0) {
            state = EXPLORE_COMPLETE;
            return false;
        }

        // 3. Chọn cluster tốt nhất (score = size / distance)
        int bestIdx = selectBestCluster(robotX, robotY);
        if (bestIdx < 0) {
            state = EXPLORE_FAILED;
            Serial.println("[EXPLORE] All clusters blacklisted or too close — FAILED");
            return false;
        }

        outGoalX = _clusters[bestIdx].centroidX;
        outGoalY = _clusters[bestIdx].centroidY;

        // Snap goal vào vùng an toàn (cách obstacle ít nhất 3 cells)
        snapToSafe(mapper, outGoalX, outGoalY);

        goalX = outGoalX;
        goalY = outGoalY;
        exploredGoals++;

        Serial.printf("[EXPLORE] Goal #%d: (%.2f, %.2f) cluster_size=%d dist=%.2fm\n",
            exploredGoals, goalX, goalY,
            _clusters[bestIdx].size, _clusters[bestIdx].distToRobot);

        return true;
    }

    /**
     * Đánh dấu goal hiện tại là unreachable (A* không tìm được path)
     */
    void blacklistCurrentGoal() {
        if (blacklistCount < FRONTIER_BLACKLIST_MAX) {
            blacklistX[blacklistCount] = goalX;
            blacklistY[blacklistCount] = goalY;
            blacklistCount++;
            Serial.printf("[EXPLORE] Blacklisted goal (%.2f, %.2f) — total %d\n",
                goalX, goalY, blacklistCount);
        }
    }

private:
    // ── Internal buffers ──────────────────────────────────────
    FrontierCell   _cells[FRONTIER_MAX_CELLS];
    FrontierCluster _clusters[FRONTIER_MAX_CLUSTERS];
    bool           _visited[GRID_SIZE][GRID_SIZE];  // BFS visited map

    // ── Step 1: Detect frontier cells ─────────────────────────
    // Frontier = cell FREE (logodds < -5) với ≥1 neighbor UNKNOWN (logodds == 0)
    void detectFrontiers(const OccupancyGridMapper& mapper) {
        frontierCellCount = 0;

        for (int y = 1; y < GRID_SIZE - 1; y++) {
            for (int x = 1; x < GRID_SIZE - 1; x++) {
                // Cell này phải FREE
                if (mapper.grid[y][x] >= -5) continue;

                // Kiểm tra 4-connected neighbors: có UNKNOWN không?
                bool hasUnknown = false;
                if (mapper.grid[y-1][x] == 0) hasUnknown = true;
                if (mapper.grid[y+1][x] == 0) hasUnknown = true;
                if (mapper.grid[y][x-1] == 0) hasUnknown = true;
                if (mapper.grid[y][x+1] == 0) hasUnknown = true;

                if (hasUnknown && frontierCellCount < FRONTIER_MAX_CELLS) {
                    _cells[frontierCellCount].gx = (int16_t)x;
                    _cells[frontierCellCount].gy = (int16_t)y;
                    frontierCellCount++;
                }
            }
        }
    }

    // ── Step 2: Cluster frontiers (BFS flood-fill) ────────────
    // Now receives mapper reference for proper world coordinate conversion
    void clusterFrontiers(const OccupancyGridMapper& mapper, float robotX, float robotY) {
        clusterCount = 0;

        // Mark tất cả frontier cells trong visited map
        memset(_visited, 0, sizeof(_visited));

        // Tạo lookup nhanh: cell nào là frontier
        static bool isFrontier[GRID_SIZE][GRID_SIZE];
        memset(isFrontier, 0, sizeof(isFrontier));
        for (int i = 0; i < frontierCellCount; i++) {
            isFrontier[_cells[i].gy][_cells[i].gx] = true;
        }

        // BFS cluster mỗi frontier cell chưa visited
        for (int i = 0; i < frontierCellCount; i++) {
            int sx = _cells[i].gx;
            int sy = _cells[i].gy;
            if (_visited[sy][sx]) continue;
            if (clusterCount >= FRONTIER_MAX_CLUSTERS) break;

            // BFS flood-fill từ cell này
            static FrontierCell queue[FRONTIER_MAX_CELLS];
            int qHead = 0, qTail = 0;
            queue[qTail++] = {(int16_t)sx, (int16_t)sy};
            _visited[sy][sx] = true;

            float sumGx = 0, sumGy = 0;
            int count = 0;

            while (qHead < qTail) {
                FrontierCell c = queue[qHead++];
                sumGx += c.gx;
                sumGy += c.gy;
                count++;

                // Expand 4-connected
                const int dx[] = {0, 0, -1, 1};
                const int dy[] = {-1, 1, 0, 0};
                for (int d = 0; d < 4; d++) {
                    int nx = c.gx + dx[d];
                    int ny = c.gy + dy[d];
                    if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) continue;
                    if (_visited[ny][nx]) continue;
                    if (!isFrontier[ny][nx]) continue;
                    _visited[ny][nx] = true;
                    if (qTail < FRONTIER_MAX_CELLS) {
                        queue[qTail++] = {(int16_t)nx, (int16_t)ny};
                    }
                }
            }

            if (count < FRONTIER_MIN_CLUSTER_SIZE) continue;

            // Centroid: average grid cell → convert to world using mapper's origin
            float avgGx = sumGx / count;
            float avgGy = sumGy / count;
            float cx = mapper.grid_to_world_x((int)avgGx);
            float cy = mapper.grid_to_world_y((int)avgGy);
            float dist = sqrtf((cx - robotX) * (cx - robotX) + (cy - robotY) * (cy - robotY));

            _clusters[clusterCount].centroidX = cx;
            _clusters[clusterCount].centroidY = cy;
            _clusters[clusterCount].size = count;
            _clusters[clusterCount].distToRobot = dist;
            clusterCount++;
        }
    }

    // ── Step 3: Select best cluster ───────────────────────────
    // Score = size × (1 / distance) — ưu tiên clusters lớn + gần
    int selectBestCluster(float robotX, float robotY) {
        float bestScore = -1.0f;
        int   bestIdx   = -1;

        for (int i = 0; i < clusterCount; i++) {
            // Skip clusters quá gần
            if (_clusters[i].distToRobot < FRONTIER_MIN_GOAL_DIST) continue;

            // Skip blacklisted goals
            if (isBlacklisted(_clusters[i].centroidX, _clusters[i].centroidY)) continue;

            // Score: size weighted by proximity
            float score = (float)_clusters[i].size / fmaxf(0.5f, _clusters[i].distToRobot);
            if (score > bestScore) {
                bestScore = score;
                bestIdx = i;
            }
        }

        return bestIdx;
    }

    // ── Check blacklist ───────────────────────────────────────
    bool isBlacklisted(float x, float y) const {
        for (int i = 0; i < blacklistCount; i++) {
            float d = sqrtf((x - blacklistX[i]) * (x - blacklistX[i]) +
                           (y - blacklistY[i]) * (y - blacklistY[i]));
            if (d < 0.3f) return true;  // Cùng vùng 30cm → coi như blacklisted
        }
        return false;
    }

    // ── Snap goal away from obstacles (uses mapper coordinate helpers) ──
    void snapToSafe(const OccupancyGridMapper& mapper, float& wx, float& wy) {
        int gx = mapper.world_to_grid_x(wx);
        int gy = mapper.world_to_grid_y(wy);

        // Nếu goal cell đã safe → return nguyên
        if (gx >= FRONTIER_SAFE_DIST_CELLS && gx < GRID_SIZE - FRONTIER_SAFE_DIST_CELLS &&
            gy >= FRONTIER_SAFE_DIST_CELLS && gy < GRID_SIZE - FRONTIER_SAFE_DIST_CELLS) {
            // Kiểm tra xung quanh: có obstacle quá gần không?
            bool tooClose = false;
            for (int dy = -FRONTIER_SAFE_DIST_CELLS; dy <= FRONTIER_SAFE_DIST_CELLS && !tooClose; dy++) {
                for (int dx = -FRONTIER_SAFE_DIST_CELLS; dx <= FRONTIER_SAFE_DIST_CELLS && !tooClose; dx++) {
                    if (mapper.grid[gy + dy][gx + dx] > 30) tooClose = true;
                }
            }
            if (!tooClose) return;  // Safe
        }

        // Tìm cell FREE gần nhất trong bán kính 10 cells mà an toàn
        float bestDist = 1e9f;
        int bestGx = gx, bestGy = gy;
        for (int sy = -10; sy <= 10; sy++) {
            for (int sx = -10; sx <= 10; sx++) {
                int cx = gx + sx;
                int cy = gy + sy;
                if (cx < 2 || cx >= GRID_SIZE - 2 || cy < 2 || cy >= GRID_SIZE - 2) continue;
                if (mapper.grid[cy][cx] >= -5) continue;  // Phải FREE

                // Kiểm tra safe radius
                bool safe = true;
                for (int dy = -2; dy <= 2 && safe; dy++) {
                    for (int dx = -2; dx <= 2 && safe; dx++) {
                        if (mapper.grid[cy + dy][cx + dx] > 30) safe = false;
                    }
                }
                if (!safe) continue;

                float d = sqrtf((float)(sx * sx + sy * sy));
                if (d < bestDist) {
                    bestDist = d;
                    bestGx = cx;
                    bestGy = cy;
                }
            }
        }

        // Convert back to world coordinates using mapper helpers
        wx = mapper.grid_to_world_x(bestGx);
        wy = mapper.grid_to_world_y(bestGy);
    }
};

#endif // FRONTIER_EXPLORER_H
