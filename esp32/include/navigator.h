/**
 * AMR 2.0 — Autonomous Navigator
 * Xe tự bám lộ trình, xoay góc, dé lùi thông minh
 * App chỉ gửi danh sách waypoint → ESP32 tự làm hết
 */

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "config.h"

// ============================================================
//   NAVIGATOR CONFIG
// ============================================================

// Khoảng cách coi là "đã tới" waypoint (mét)
#define WP_REACH_DIST        0.08f

// Khoảng cách waypoint cuối (đích) — chính xác hơn
#define WP_FINAL_REACH_DIST  0.04f

// Sai số góc chấp nhận khi xoay tại chỗ (radian ~ 3 độ)
#define HEADING_TOLERANCE     0.052f

// Tốc độ tối đa khi di chuyển thẳng (m/s)
#define NAV_MAX_LINEAR_VEL    0.15f

// Tốc độ khi gần đích — chậm lại cho chính xác (m/s)
#define NAV_APPROACH_VEL      0.06f

// Khoảng cách bắt đầu giảm tốc (mét)
#define NAV_SLOWDOWN_DIST     0.25f

// Tốc độ xoay tại chỗ (rad/s)
#define NAV_TURN_SPEED        1.2f

// Tốc độ xoay chậm khi gần đúng hướng (rad/s)
#define NAV_TURN_SLOW_SPEED   0.5f

// Ngưỡng góc bắt đầu xoay chậm (radian ~ 15 độ)
#define NAV_TURN_SLOW_ZONE    0.26f

// Ngưỡng góc để quyết định dé lùi thay vì quay đầu (radian ~ 120 độ)
// Nếu waypoint nằm phía sau > 120 độ, dé lùi sẽ ngắn hơn quay đầu
#define NAV_REVERSE_THRESHOLD 2.09f

// Hệ số PID bám đường (Pure Pursuit Curvature → Angular Vel)
#define NAV_KP_HEADING        3.5f

// Lookahead distance cho Pure Pursuit (mét)
#define NAV_LOOKAHEAD_DIST    0.20f

// Số waypoint tối đa mà ESP32 lưu được
#define MAX_WAYPOINTS         64

// Timeout cho mỗi waypoint (ms) — nếu kẹt 1 chỗ quá lâu thì báo lỗi
#define NAV_WP_TIMEOUT_MS     15000

// ============================================================
//   NAVIGATOR STATES
// ============================================================

enum NavState {
  NAV_IDLE = 0,       // Đứng yên, chờ lệnh
  NAV_TURNING,        // Xoay tại chỗ để hướng về waypoint tiếp theo
  NAV_DRIVING,        // Chạy thẳng bám đường Pure Pursuit
  NAV_REVERSING,      // Dé lùi (waypoint ở phía sau)
  NAV_FINAL_TURN,     // Xoay về heading cuối cùng tại đích
  NAV_DONE,           // Hoàn thành lộ trình
  NAV_ERROR           // Lỗi (timeout, vượt quá sai số)
};

// ============================================================
//   WAYPOINT STRUCTURE
// ============================================================

struct Waypoint {
  float x;            // Tọa độ X (mét)
  float y;            // Tọa độ Y (mét)
  float heading;      // Heading mong muốn khi tới (radian, NAN = ko quan tâm)
  bool  useReverse;   // true = cho phép dé lùi tới điểm này
};

// ============================================================
//   NAVIGATOR CLASS
// ============================================================

class Navigator {
public:
  // === Trạng thái ===
  NavState state = NAV_IDLE;
  
  // === Waypoint buffer ===
  Waypoint waypoints[MAX_WAYPOINTS];
  int      waypointCount = 0;
  int      currentWpIdx  = 0;
  
  // === Heading cuối cùng (góc robot cần xoay khi tới đích) ===
  float finalHeading = NAN;
  
  // === Output: Linear + Angular velocity cho PID motor ===
  float cmdLinear  = 0;
  float cmdAngular = 0;
  
  // === Thống kê ===
  unsigned long navStartTime = 0;
  unsigned long lastWpReachTime = 0;

  /**
   * Nạp lộ trình mới từ App
   * @param wps     Mảng waypoint
   * @param count   Số waypoint
   * @param endHeading  Góc xoay cuối cùng tại đích (radian), NAN nếu không cần
   */
  void loadPath(Waypoint* wps, int count, float endHeading = NAN) {
    if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
    
    waypointCount = count;
    currentWpIdx = 0;
    finalHeading = endHeading;
    
    for (int i = 0; i < count; i++) {
      waypoints[i] = wps[i];
    }
    
    // Quyết định chiến lược cho từng waypoint
    for (int i = 0; i < count; i++) {
      waypoints[i].useReverse = false; // Mặc định tiến
    }
    
    state = NAV_IDLE;
    cmdLinear = 0;
    cmdAngular = 0;
    navStartTime = millis();
    
    if (count > 0) {
      state = NAV_TURNING;
      lastWpReachTime = millis();
      Serial.printf("[NAV] Path loaded: %d waypoints, finalH=%.1f°\n", 
                    count, isnan(endHeading) ? -999.0f : endHeading * 180.0f / PI);
    }
  }
  
  /**
   * Hủy lộ trình, dừng xe ngay
   */
  void abort() {
    state = NAV_IDLE;
    waypointCount = 0;
    currentWpIdx = 0;
    cmdLinear = 0;
    cmdAngular = 0;
    Serial.println("[NAV] ABORTED");
  }
  
  /**
   * Hàm cập nhật chính — gọi mỗi chu kỳ control loop (50Hz)
   * @param robotX     Tọa độ hiện tại X (mét)
   * @param robotY     Tọa độ hiện tại Y (mét)  
   * @param robotTheta Hướng hiện tại (radian)
   */
  void update(float robotX, float robotY, float robotTheta) {
    if (state == NAV_IDLE || state == NAV_DONE || state == NAV_ERROR) {
      cmdLinear = 0;
      cmdAngular = 0;
      return;
    }
    
    // === TIMEOUT CHECK: xe bị kẹt quá lâu tại 1 waypoint? ===
    if (millis() - lastWpReachTime > NAV_WP_TIMEOUT_MS) {
      Serial.printf("[NAV] ⚠️ TIMEOUT at WP%d! Aborting.\n", currentWpIdx);
      state = NAV_ERROR;
      cmdLinear = 0;
      cmdAngular = 0;
      return;
    }
    
    // Lấy waypoint hiện tại
    Waypoint& wp = waypoints[currentWpIdx];
    bool isFinalWp = (currentWpIdx == waypointCount - 1);
    
    // Tính vector tới waypoint
    float dx = wp.x - robotX;
    float dy = wp.y - robotY;
    float dist = sqrtf(dx * dx + dy * dy);
    float angleToWp = atan2f(dy, dx);
    
    // Sai số góc: góc cần xoay để hướng về waypoint
    float headingError = _normalizeAngle(angleToWp - robotTheta);
    
    // Ngưỡng "đã tới" tùy theo waypoint giữa đường hay đích cuối
    float reachDist = isFinalWp ? WP_FINAL_REACH_DIST : WP_REACH_DIST;
    
    switch (state) {
      
      // ─── XOAY TẠI CHỖ ────────────────────────────
      case NAV_TURNING: {
        // Kiểm tra: nếu waypoint ở phía sau > threshold, dé lùi thay vì quay đầu 180°
        if (fabsf(headingError) > NAV_REVERSE_THRESHOLD && dist > 0.15f) {
          // Dé lùi! Tính heading ngược lại
          state = NAV_REVERSING;
          Serial.printf("[NAV] WP%d: Reverse mode (err=%.0f°, dist=%.2fm)\n", 
                        currentWpIdx, headingError * 180.0f / PI, dist);
          break;
        }
        
        if (fabsf(headingError) < HEADING_TOLERANCE) {
          // Đã xoay đúng hướng → chuyển sang chạy thẳng
          state = NAV_DRIVING;
          cmdAngular = 0;
          Serial.printf("[NAV] WP%d: Heading locked → DRIVE (dist=%.2fm)\n", 
                        currentWpIdx, dist);
        } else {
          // Xoay tại chỗ
          cmdLinear = 0;
          if (fabsf(headingError) < NAV_TURN_SLOW_ZONE) {
            cmdAngular = (headingError > 0) ? NAV_TURN_SLOW_SPEED : -NAV_TURN_SLOW_SPEED;
          } else {
            cmdAngular = (headingError > 0) ? NAV_TURN_SPEED : -NAV_TURN_SPEED;
          }
        }
        break;
      }
      
      // ─── CHẠY THẲNG BÁM ĐƯỜNG ────────────────────
      case NAV_DRIVING: {
        // Kiểm tra đã tới chưa
        if (dist < reachDist) {
          _onWaypointReached(isFinalWp, robotTheta);
          break;
        }
        
        // Nếu lệch quá nhiều (> 90°), dừng lại xoay
        if (fabsf(headingError) > 1.57f) {
          state = NAV_TURNING;
          cmdLinear = 0;
          cmdAngular = 0;
          break;
        }
        
        // Pure Pursuit: Tính tốc độ thẳng và xoay
        // Giảm tốc khi gần đích
        float speed = NAV_MAX_LINEAR_VEL;
        if (isFinalWp && dist < NAV_SLOWDOWN_DIST) {
          speed = NAV_APPROACH_VEL + (NAV_MAX_LINEAR_VEL - NAV_APPROACH_VEL) * (dist / NAV_SLOWDOWN_DIST);
        }
        
        cmdLinear = speed;
        cmdAngular = NAV_KP_HEADING * headingError;
        
        // Giới hạn angular để không quay vòng vòng
        cmdAngular = constrain(cmdAngular, -NAV_TURN_SPEED, NAV_TURN_SPEED);
        break;
      }
      
      // ─── DÉ LÙI ──────────────────────────────────
      case NAV_REVERSING: {
        // Khi dé lùi, ta muốn ĐUÔI xe hướng về waypoint
        // → heading mong muốn = angleToWp + PI
        float reverseHeading = _normalizeAngle(angleToWp + PI);
        float reverseError = _normalizeAngle(reverseHeading - robotTheta);
        
        // Bước 1: Xoay đuôi về hướng waypoint  
        if (fabsf(reverseError) > HEADING_TOLERANCE) {
          cmdLinear = 0;
          if (fabsf(reverseError) < NAV_TURN_SLOW_ZONE) {
            cmdAngular = (reverseError > 0) ? NAV_TURN_SLOW_SPEED : -NAV_TURN_SLOW_SPEED;
          } else {
            cmdAngular = (reverseError > 0) ? NAV_TURN_SPEED : -NAV_TURN_SPEED;
          }
          break;
        }
        
        // Bước 2: Đã xoay xong → lùi!
        if (dist < reachDist) {
          _onWaypointReached(isFinalWp, robotTheta);
          break;
        }
        
        // Lùi với tốc độ chậm hơn tiến
        float reverseSpeed = NAV_APPROACH_VEL;
        if (isFinalWp && dist < NAV_SLOWDOWN_DIST) {
          reverseSpeed = NAV_APPROACH_VEL * 0.6f;
        }
        
        cmdLinear = -reverseSpeed; // ÂM = lùi
        cmdAngular = -NAV_KP_HEADING * reverseError * 0.5f; // Bù lái nhẹ khi lùi
        cmdAngular = constrain(cmdAngular, -NAV_TURN_SLOW_SPEED, NAV_TURN_SLOW_SPEED);
        break;
      }
      
      // ─── XOAY VỀ HEADING CUỐI CÙNG ───────────────
      case NAV_FINAL_TURN: {
        if (isnan(finalHeading)) {
          state = NAV_DONE;
          cmdLinear = 0;
          cmdAngular = 0;
          break;
        }
        
        float finalErr = _normalizeAngle(finalHeading - robotTheta);
        
        if (fabsf(finalErr) < HEADING_TOLERANCE) {
          state = NAV_DONE;
          cmdLinear = 0;
          cmdAngular = 0;
          unsigned long totalMs = millis() - navStartTime;
          Serial.printf("[NAV] ✅ MISSION COMPLETE! (%.1fs)\n", totalMs / 1000.0f);
        } else {
          cmdLinear = 0;
          if (fabsf(finalErr) < NAV_TURN_SLOW_ZONE) {
            cmdAngular = (finalErr > 0) ? NAV_TURN_SLOW_SPEED : -NAV_TURN_SLOW_SPEED;
          } else {
            cmdAngular = (finalErr > 0) ? NAV_TURN_SPEED : -NAV_TURN_SPEED;
          }
        }
        break;
      }
      
      default:
        cmdLinear = 0;
        cmdAngular = 0;
        break;
    }
  }
  
  /**
   * Lấy tên trạng thái (dùng cho Telemetry + OLED)
   */
  const char* getStateName() {
    switch (state) {
      case NAV_IDLE:       return "IDLE";
      case NAV_TURNING:    return "TURN";
      case NAV_DRIVING:    return "DRIVE";
      case NAV_REVERSING:  return "REV";
      case NAV_FINAL_TURN: return "F_TURN";
      case NAV_DONE:       return "DONE";
      case NAV_ERROR:      return "ERROR";
      default:             return "???";
    }
  }
  
  bool isNavigating() {
    return state == NAV_TURNING || state == NAV_DRIVING || 
           state == NAV_REVERSING || state == NAV_FINAL_TURN;
  }

private:
  /**
   * Chuẩn hóa góc về [-PI, PI]
   */
  float _normalizeAngle(float a) {
    while (a > PI)  a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
  }
  
  /**
   * Xử lý khi tới waypoint
   */
  void _onWaypointReached(bool isFinalWp, float robotTheta) {
    Serial.printf("[NAV] ✓ WP%d reached!\n", currentWpIdx);
    lastWpReachTime = millis();
    
    if (isFinalWp) {
      // Đã tới đích cuối
      if (!isnan(finalHeading)) {
        state = NAV_FINAL_TURN;
        Serial.printf("[NAV] At destination → Final turn to %.1f°\n", 
                      finalHeading * 180.0f / PI);
      } else {
        state = NAV_DONE;
        unsigned long totalMs = millis() - navStartTime;
        Serial.printf("[NAV] ✅ MISSION COMPLETE! (%.1fs)\n", totalMs / 1000.0f);
      }
    } else {
      // Còn waypoint tiếp → nhảy sang
      currentWpIdx++;
      state = NAV_TURNING;
    }
    
    cmdLinear = 0;
    cmdAngular = 0;
  }
};

#endif // NAVIGATOR_H
