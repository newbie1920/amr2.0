/**
 * AMR 2.0 — LIDAR Occupancy Grid Mapper
 * Real-time 2D grid-based mapping from RPLIDAR A1M8
 * Sử dụng Bresenham's line algorithm để cập nhật grid cells
 */

#ifndef LIDAR_MAPPER_H
#define LIDAR_MAPPER_H

#include <Arduino.h>
#include <stdint.h>
#include <cstring>
#include <cmath>

// ============================================================
//   CONFIG
// ============================================================

/** Grid size: 8m x 8m = 80x80 cells @ 0.1m resolution
 *  FIX Bug #9: Tăng từ 40x40@0.25m → 80x80@0.1m cho resolution 
 *  phù hợp hơn (robot wheelbase = 0.17m, cell = 0.1m).
 *  RAM usage: 80*80*1 = 6.4KB — chấp nhận được cho ESP32-S3 (320KB)
 */
#define GRID_SIZE 80                    // 80x80 cells
#define GRID_RESOLUTION 0.1f            // mét/cell (khớp với web-side grid)
#define GRID_WIDTH_M (GRID_SIZE * GRID_RESOLUTION)  // 8m
#define GRID_HEIGHT_M (GRID_SIZE * GRID_RESOLUTION) // 8m

/** Occupancy probabilities (log-odds style) */
#define LOGODDS_OCC 50      // Ghi nhận cell bị chiếm (+ 50)
#define LOGODDS_FREE -10    // Ghi nhận cell trống (- 10)
#define LOGODDS_UNKNOWN 0   // Chưa biết
#define LOGODDS_MAX 100     // Giới hạn trên (cắt)
#define LOGODDS_MIN -100    // Giới hạn dưới

/** Max LIDAR range (mét) */
#define LIDAR_MAX_RANGE 6.0f

// ============================================================
//   LIDAR SCAN POINT
// ============================================================

struct LidarPoint {
  float angle;    // Góc quét (độ) [0, 360)
  float distance; // Khoảng cách (mét)
  bool quality;   // Điểm hợp lệ
};

// ============================================================
//   OCCUPANCY GRID MAPPER
// ============================================================

class OccupancyGridMapper {
public:
  // Grid cells (log-odds format, int8_t)
  int8_t grid[GRID_SIZE][GRID_SIZE];
  
  // Robot pose (for raycast)
  float robot_x, robot_y, robot_heading;
  
  // Scan buffer
  static const int MAX_POINTS = 360;
  LidarPoint points[MAX_POINTS];
  int point_count;

  OccupancyGridMapper() 
    : robot_x(5.0f), robot_y(5.0f), robot_heading(0.0f), point_count(0) {
    memset(grid, LOGODDS_UNKNOWN, sizeof(grid));
  }

  /**
   * Reset grid về trống toàn bộ
   */
  void reset() {
    memset(grid, LOGODDS_UNKNOWN, sizeof(grid));
    point_count = 0;
  }

  /**
   * Cập nhật robot pose (từ odometry)
   */
  void update_pose(float x, float y, float heading) {
    robot_x = x;
    robot_y = y;
    robot_heading = heading;
  }

  /**
   * Thêm LIDAR scan point
   */
  void add_point(float angle_deg, float distance_m) {
    if (point_count < MAX_POINTS) {
      points[point_count].angle = angle_deg;
      points[point_count].distance = distance_m;
      points[point_count].quality = (distance_m > 0 && distance_m < LIDAR_MAX_RANGE);
      point_count++;
    }
  }

  /**
   * Tiến hành raycasting + cập nhật grid
   * (Thường gọi sau khi có đủ points từ một vòng quét)
   */
  void update_grid() {
    // Raycast từ robot position qua từng point
    for (int i = 0; i < point_count; i++) {
      const LidarPoint& pt = points[i];
      
      if (!pt.quality) continue;
      
      // Convert polar → Cartesian (world frame)
      // FIX Bug #4: pt.angle is in degrees, robot_heading is in radians
      // Convert angle to radians first, THEN add heading
      float angle_rad = (pt.angle * M_PI / 180.0f) + robot_heading;
      float end_x = robot_x + pt.distance * cosf(angle_rad);
      float end_y = robot_y + pt.distance * sinf(angle_rad);
      
      // Raycast: cells từ robot→end point được mark FREE
      raycast_free(robot_x, robot_y, end_x, end_y);
      
      // End point được mark OCCUPIED
      int gx = world_to_grid_x(end_x);
      int gy = world_to_grid_y(end_y);
      if (in_bounds(gx, gy)) {
        grid[gy][gx] = constrain_logodds(grid[gy][gx] + LOGODDS_OCC);
      }
    }
    
    point_count = 0; // Clear buffer
  }

  /**
   * Lấy occupancy value (0-100) tại cell grid
   * 0 = trống, 50 = unknown, 100 = occupied
   */
  uint8_t get_occupancy(int gx, int gy) const {
    if (!in_bounds(gx, gy)) return 50; // unknown
    
    int8_t logodds = grid[gy][gx];
    // Convert log-odds to probability [0,100]
    // logodds=0 → 50%, logodds=100 → ~100%, logodds=-100 → ~0%
    float prob = 50.0f + (logodds * 0.5f);
    return (uint8_t)constrain(prob, 0.0f, 100.0f);
  }

  /**
   * Serialize grid cho WebSocket transmission
   * Output: 40x40 = 1600 bytes (uint8_t occupancy values)
   */
  void serialize_grid(uint8_t* buffer, int& len) {
    len = 0;
    for (int y = 0; y < GRID_SIZE; y++) {
      for (int x = 0; x < GRID_SIZE; x++) {
        buffer[len++] = get_occupancy(x, y);
      }
    }
  }

  /**
   * Serialize scan points (current buffer)
   * Output: array of {angle, distance} pairs
   */
  void serialize_scan(uint8_t* buffer, int& len) {
    len = 0;
    // Format: [count_u16][angle_u16][dist_u16][angle_u16][dist_u16]...
    uint16_t count = point_count;
    memcpy(buffer, &count, 2);
    len = 2;
    
    for (int i = 0; i < point_count; i++) {
      uint16_t angle_int = (uint16_t)(points[i].angle * 100); // angle*100
      uint16_t dist_int = (uint16_t)(points[i].distance * 1000); // dist*1000
      memcpy(buffer + len, &angle_int, 2);
      len += 2;
      memcpy(buffer + len, &dist_int, 2);
      len += 2;
    }
  }

private:
  // Helpers
  
  int world_to_grid_x(float x) const {
    return (int)((x / GRID_WIDTH_M) * GRID_SIZE);
  }
  
  int world_to_grid_y(float y) const {
    return (int)((y / GRID_HEIGHT_M) * GRID_SIZE);
  }
  
  bool in_bounds(int gx, int gy) const {
    return gx >= 0 && gx < GRID_SIZE && gy >= 0 && gy < GRID_SIZE;
  }
  
  int8_t constrain_logodds(int val) const {
    if (val > LOGODDS_MAX) return LOGODDS_MAX;
    if (val < LOGODDS_MIN) return LOGODDS_MIN;
    return (int8_t)val;
  }

  /**
   * Bresenham's line algorithm — raycast từ (x0,y0) → (x1,y1)
   * Tất cả cells dọc đường được mark FREE
   */
  void raycast_free(float x0, float y0, float x1, float y1) {
    int gx0 = world_to_grid_x(x0);
    int gy0 = world_to_grid_y(y0);
    int gx1 = world_to_grid_x(x1);
    int gy1 = world_to_grid_y(y1);
    
    int dx = abs(gx1 - gx0);
    int dy = abs(gy1 - gy0);
    int sx = (gx0 < gx1) ? 1 : -1;
    int sy = (gy0 < gy1) ? 1 : -1;
    int err = dx - dy;
    
    int x = gx0, y = gy0;
    
    while (true) {
      if (in_bounds(x, y)) {
        // Chỉ cập nhật nếu chưa occupied (không ghi đè)
        if (grid[y][x] < LOGODDS_OCC / 2) {
          grid[y][x] = constrain_logodds(grid[y][x] + LOGODDS_FREE);
        }
      }
      
      if (x == gx1 && y == gy1) break;
      
      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }
  }
};

#endif // LIDAR_MAPPER_H
