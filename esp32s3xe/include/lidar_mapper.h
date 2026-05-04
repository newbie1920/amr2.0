/**
 * AMR 2.0 — LIDAR Occupancy Grid Mapper
 * Real-time 2D grid-based mapping from RPLIDAR A1M8
 * Sử dụng Bresenham's line algorithm để cập nhật grid cells
 *
 * SLAM Upgrade v2:
 *   - Grid 128x128 @ 0.1m = 12.8m x 12.8m coverage
 *   - Origin-based coordinate system (grid centered at configurable origin)
 *   - Proper log-odds update (raycast luôn cập nhật free, không skip occupied)
 *   - Max range truncation (tránh ghost walls ở rìa scan)
 *   - Public coordinate helpers cho DWA/Frontier/Pathfinder dùng chung
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

/** Grid size: 12.8m x 12.8m = 128x128 cells @ 0.1m resolution
 *  RAM usage: 128*128*1 = 16KB — fits in SRAM (ESP32-S3 has 320KB)
 *  Coverage đủ cho warehouse nhỏ/phòng test ~12m
 */
#define GRID_SIZE 128                   // 128x128 cells
#define GRID_RESOLUTION 0.1f            // mét/cell (khớp với web-side grid)
#define GRID_WIDTH_M (GRID_SIZE * GRID_RESOLUTION)  // 12.8m
#define GRID_HEIGHT_M (GRID_SIZE * GRID_RESOLUTION) // 12.8m

/** Occupancy probabilities (log-odds style)
 *  Tuned for balanced map: occupied cells cần nhiều evidence để build up,
 *  free cells dễ clear hơn → tránh phantom walls.
 */
#define LOGODDS_OCC 30       // Ghi nhận cell bị chiếm (giảm từ 50 → 30 cho smoother build-up)
#define LOGODDS_FREE -15     // Ghi nhận cell trống (tăng từ -10 → -15 cho faster clearing)
#define LOGODDS_UNKNOWN 0    // Chưa biết
#define LOGODDS_MAX_OCC 80   // Giới hạn trên (giảm từ 100 → 80, tránh saturate)
#define LOGODDS_MIN_FREE -50 // Giới hạn dưới (tăng từ -100 → -50, enough for free)
// Legacy compatibility defines
#define LOGODDS_MAX LOGODDS_MAX_OCC
#define LOGODDS_MIN LOGODDS_MIN_FREE

/** Max LIDAR range (mét) */
#define LIDAR_MAX_RANGE 6.0f
/** Truncation factor: points >= MAX_RANGE * 0.95 treated as max-range (no occupied mark) */
#define LIDAR_RANGE_TRUNCATION_FACTOR 0.95f

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
  
  // Grid origin in world coordinates (meters)
  // Grid cell [0][0] corresponds to world point (originX, originY)
  float originX, originY;
  
  // Robot pose (for raycast) — in world/map frame
  float robot_x, robot_y, robot_heading;
  
  // Scan buffer
  static const int MAX_POINTS = 360;
  LidarPoint points[MAX_POINTS];
  int point_count;
  
  // Stats
  int scanCount;  // Total number of grid updates performed

  OccupancyGridMapper() 
    : originX(-6.4f), originY(-6.4f),   // Grid centered at world (0,0): origin = -GRID_WIDTH_M/2
      robot_x(0.0f), robot_y(0.0f), robot_heading(0.0f), 
      point_count(0), scanCount(0) {
    memset(grid, LOGODDS_UNKNOWN, sizeof(grid));
  }

  /**
   * Reset grid về trống toàn bộ
   */
  void reset() {
    memset(grid, LOGODDS_UNKNOWN, sizeof(grid));
    point_count = 0;
    scanCount = 0;
  }

  /**
   * Set grid origin — call before first scan if robot spawns far from (0,0)
   * Centers the grid around the given world position.
   */
  void centerOnPosition(float worldX, float worldY) {
    originX = worldX - GRID_WIDTH_M / 2.0f;
    originY = worldY - GRID_HEIGHT_M / 2.0f;
  }

  /**
   * Cập nhật robot pose (từ odometry/map frame)
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
   *
   * SLAM Upgrade v2:
   *   - Raycast LUÔN cập nhật free cells (không skip occupied cells trên ray)
   *   - Max range truncation: points near LIDAR_MAX_RANGE chỉ mark free, không mark occupied
   */
  void update_grid() {
    for (int i = 0; i < point_count; i++) {
      const LidarPoint& pt = points[i];
      
      if (!pt.quality) continue;
      
      // Convert polar → Cartesian (world frame)
      float angle_rad = (pt.angle * M_PI / 180.0f) + robot_heading;
      float end_x = robot_x + pt.distance * cosf(angle_rad);
      float end_y = robot_y + pt.distance * sinf(angle_rad);
      
      // Raycast: cells từ robot→end point được mark FREE
      raycast_free(robot_x, robot_y, end_x, end_y);
      
      // End point được mark OCCUPIED — nhưng chỉ khi không phải max-range hit
      // Max-range hits thường là noise hoặc open space, không phải wall
      bool isMaxRange = pt.distance >= (LIDAR_MAX_RANGE * LIDAR_RANGE_TRUNCATION_FACTOR);
      if (!isMaxRange) {
        int gx = world_to_grid_x(end_x);
        int gy = world_to_grid_y(end_y);
        if (in_bounds(gx, gy)) {
          grid[gy][gx] = constrain_logodds(grid[gy][gx] + LOGODDS_OCC);
        }
      }
    }
    
    point_count = 0; // Clear buffer
    scanCount++;
  }

  /**
   * Lấy occupancy value (0-100) tại cell grid
   * 0 = trống, 50 = unknown, 100 = occupied
   */
  uint8_t get_occupancy(int gx, int gy) const {
    if (!in_bounds(gx, gy)) return 50; // unknown
    
    int8_t logodds = grid[gy][gx];
    // Convert log-odds to probability [0,100]
    float prob = 50.0f + (logodds * 0.625f);  // Scale: -80→0, 0→50, +80→100
    return (uint8_t)constrain(prob, 0.0f, 100.0f);
  }

  // ============================================================
  //   PUBLIC COORDINATE HELPERS
  //   (DWA, Frontier Explorer, Pathfinder dùng chung)
  // ============================================================

  /** World → Grid X (chuẩn, dùng origin offset) */
  int world_to_grid_x(float x) const {
    return (int)floorf((x - originX) / GRID_RESOLUTION);
  }
  
  /** World → Grid Y (chuẩn, dùng origin offset) */
  int world_to_grid_y(float y) const {
    return (int)floorf((y - originY) / GRID_RESOLUTION);
  }
  
  /** Grid → World X (center of cell) */
  float grid_to_world_x(int gx) const {
    return originX + (gx + 0.5f) * GRID_RESOLUTION;
  }
  
  /** Grid → World Y (center of cell) */
  float grid_to_world_y(int gy) const {
    return originY + (gy + 0.5f) * GRID_RESOLUTION;
  }
  
  /** Bounds check */
  bool in_bounds(int gx, int gy) const {
    return gx >= 0 && gx < GRID_SIZE && gy >= 0 && gy < GRID_SIZE;
  }

  /**
   * Serialize grid cho WebSocket transmission
   * Output: GRID_SIZE × GRID_SIZE bytes (uint8_t occupancy values)
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
  int8_t constrain_logodds(int val) const {
    if (val > LOGODDS_MAX_OCC) return LOGODDS_MAX_OCC;
    if (val < LOGODDS_MIN_FREE) return LOGODDS_MIN_FREE;
    return (int8_t)val;
  }

  /**
   * Bresenham's line algorithm — raycast từ (x0,y0) → (x1,y1)
   * Tất cả cells dọc đường được mark FREE
   *
   * SLAM Upgrade v2: LUÔN cập nhật log-odds free cho mọi cell trên ray,
   * kể cả cell đã occupied. Điều này đúng theo lý thuyết Bayesian:
   * nếu ta nhìn thấy xuyên qua 1 cell → evidence cho "free".
   * Cell occupied chỉ survive nếu nó cũng nhận đủ occupied evidence.
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
      // Không cập nhật cell cuối cùng (đó là hit cell, sẽ được mark occupied riêng)
      if (x == gx1 && y == gy1) break;
      
      if (in_bounds(x, y)) {
        grid[y][x] = constrain_logodds(grid[y][x] + LOGODDS_FREE);
      }
      
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
