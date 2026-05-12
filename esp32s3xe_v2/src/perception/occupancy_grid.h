/**
 * AMR 2.0 — Occupancy Grid Mapper (v2)
 * Real-time 2D grid-based mapping from RPLIDAR A1M8
 * Bresenham's raycast + log-odds update
 *
 * Grid: 128x128 @ 0.1m = 12.8m x 12.8m coverage
 * RAM: 16KB (fits in SRAM)
 */

#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <Arduino.h>
#include <stdint.h>
#include <cstring>
#include <cmath>
#include "config.h"  // GRID_SIZE, GRID_RESOLUTION from central config

// ── Derived Grid Config ──────────────────────────────────────
#define GRID_WIDTH_M (GRID_SIZE * GRID_RESOLUTION)
#define GRID_HEIGHT_M (GRID_SIZE * GRID_RESOLUTION)

// ── Log-odds values ──────────────────────────────────────────
#define LOGODDS_OCC 30
#define LOGODDS_FREE -15
#define LOGODDS_UNKNOWN 0
#define LOGODDS_MAX_OCC 80
#define LOGODDS_MIN_FREE -50
#define LOGODDS_MAX LOGODDS_MAX_OCC
#define LOGODDS_MIN LOGODDS_MIN_FREE

// ── LiDAR range ──────────────────────────────────────────────
#define LIDAR_MAX_RANGE 6.0f
#define LIDAR_RANGE_TRUNCATION_FACTOR 0.95f

// ── LiDAR scan point ─────────────────────────────────────────
struct LidarPoint {
    float angle;    // degrees [0,360)
    float distance; // meters
    bool quality;
};

// ── Occupancy Grid Mapper ────────────────────────────────────
class OccupancyGridMapper {
public:
    int8_t grid[GRID_SIZE][GRID_SIZE];
    float originX, originY;
    float robot_x, robot_y, robot_heading;

    static const int MAX_POINTS = 360;
    LidarPoint points[MAX_POINTS];
    int point_count;
    int scanCount;

    OccupancyGridMapper()
        : originX(-6.4f), originY(-6.4f),
          robot_x(0), robot_y(0), robot_heading(0),
          point_count(0), scanCount(0) {
        memset(grid, LOGODDS_UNKNOWN, sizeof(grid));
    }

    void reset() {
        memset(grid, LOGODDS_UNKNOWN, sizeof(grid));
        point_count = 0;
        scanCount = 0;
    }

    void centerOnPosition(float worldX, float worldY) {
        originX = worldX - GRID_WIDTH_M / 2.0f;
        originY = worldY - GRID_HEIGHT_M / 2.0f;
    }

    void update_pose(float x, float y, float heading) {
        robot_x = x; robot_y = y; robot_heading = heading;
    }

    void add_point(float angle_deg, float distance_m) {
        if (point_count < MAX_POINTS) {
            points[point_count].angle = angle_deg;
            points[point_count].distance = distance_m;
            points[point_count].quality = (distance_m > 0 && distance_m < LIDAR_MAX_RANGE);
            point_count++;
        }
    }

    void update_grid() {
        for (int i = 0; i < point_count; i++) {
            const LidarPoint& pt = points[i];
            if (!pt.quality) continue;

            float angle_rad = (pt.angle * M_PI / 180.0f) + robot_heading;
            float end_x = robot_x + pt.distance * cosf(angle_rad);
            float end_y = robot_y + pt.distance * sinf(angle_rad);

            raycast_free(robot_x, robot_y, end_x, end_y);

            bool isMaxRange = pt.distance >= (LIDAR_MAX_RANGE * LIDAR_RANGE_TRUNCATION_FACTOR);
            if (!isMaxRange) {
                int gx = world_to_grid_x(end_x);
                int gy = world_to_grid_y(end_y);
                if (in_bounds(gx, gy)) {
                    grid[gy][gx] = constrain_logodds(grid[gy][gx] + LOGODDS_OCC);
                }
            }
        }
        point_count = 0;
        scanCount++;
    }

    uint8_t get_occupancy(int gx, int gy) const {
        if (!in_bounds(gx, gy)) return 50;
        int8_t logodds = grid[gy][gx];
        float prob = 50.0f + (logodds * 0.625f);
        return (uint8_t)constrain(prob, 0.0f, 100.0f);
    }

    // ── Coordinate helpers ───────────────────────────────────
    int world_to_grid_x(float wx) const { return (int)floorf((wx - originX) / GRID_RESOLUTION); }
    int world_to_grid_y(float wy) const { return (int)floorf((wy - originY) / GRID_RESOLUTION); }
    float grid_to_world_x(int gx) const { return originX + (gx + 0.5f) * GRID_RESOLUTION; }
    float grid_to_world_y(int gy) const { return originY + (gy + 0.5f) * GRID_RESOLUTION; }
    bool in_bounds(int gx, int gy) const { return gx >= 0 && gx < GRID_SIZE && gy >= 0 && gy < GRID_SIZE; }

    // ── Serialize for WebSocket ──────────────────────────────
    void serialize_grid(uint8_t* buffer, int& len) {
        len = 0;
        for (int y = 0; y < GRID_SIZE; y++)
            for (int x = 0; x < GRID_SIZE; x++)
                buffer[len++] = get_occupancy(x, y);
    }

private:
    int8_t constrain_logodds(int val) const {
        if (val > LOGODDS_MAX_OCC) return LOGODDS_MAX_OCC;
        if (val < LOGODDS_MIN_FREE) return LOGODDS_MIN_FREE;
        return (int8_t)val;
    }

    void raycast_free(float x0, float y0, float x1, float y1) {
        int gx0 = world_to_grid_x(x0), gy0 = world_to_grid_y(y0);
        int gx1 = world_to_grid_x(x1), gy1 = world_to_grid_y(y1);

        int dx = abs(gx1 - gx0), dy = abs(gy1 - gy0);
        int sx = (gx0 < gx1) ? 1 : -1;
        int sy = (gy0 < gy1) ? 1 : -1;
        int err = dx - dy;
        int x = gx0, y = gy0;

        while (true) {
            if (x == gx1 && y == gy1) break;
            if (in_bounds(x, y)) {
                grid[y][x] = constrain_logodds(grid[y][x] + LOGODDS_FREE);
            }
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }
    }
};

// ── Global instance (defined in robot_state.cpp) ─────────────
extern OccupancyGridMapper gridMapper;

#endif // OCCUPANCY_GRID_H
