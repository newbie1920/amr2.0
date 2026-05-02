/**
 * AMR 2.0 — Autonomous Navigator
 * Backstepping Trajectory Tracking implementation based on Lyapunov stability
 * 
 * Recovery Behaviors (Nav2-style):
 *   - RECOVERY_SPIN: Xoay ~180° để quét lại môi trường
 *   - RECOVERY_BACKUP: Lùi ~30cm để thoát khỏi ngõ cụt
 *   - RECOVERY_WAIT: Chờ 3 giây cho vật cản di chuyển
 */

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "config.h"

// ============================================================
//   NAVIGATOR CONFIG
// ============================================================
// Tốc độ tối đa
#define NAV_MAX_LINEAR_VEL    0.15f
#define NAV_APPROACH_VEL      0.05f
#define NAV_SLOWDOWN_DIST     0.25f
#define NAV_TURN_SPEED        1.2f

// Hệ số Backstepping (Có thể tinh chỉnh thực tế)
#define GAIN_KX  2.0f
#define GAIN_KY  5.0f
#define GAIN_KTH 1.5f

// Chu kỳ thời gian
#define NAV_DT 0.02f // 50Hz

#define MAX_WAYPOINTS         64
#define NAV_WP_TIMEOUT_MS     15000

// ── Recovery Config (Nav2-inspired) ──
#define NAV_PROGRESS_CHECK_MS   10000   // 10s không tiến triển → trigger recovery
#define NAV_PROGRESS_DIST       0.05f   // Tối thiểu 5cm tiến triển
#define NAV_RECOVERY_SPIN_MS    3000    // Xoay 3 giây (~180°)
#define NAV_RECOVERY_BACKUP_MS  2000    // Lùi 2 giây (~30cm)
#define NAV_RECOVERY_WAIT_MS    3000    // Chờ 3 giây
#define NAV_RECOVERY_SPIN_W     1.0f    // Tốc độ xoay (rad/s)
#define NAV_RECOVERY_BACKUP_V  -0.08f   // Tốc độ lùi (m/s)
#define NAV_MAX_RECOVERY_ATTEMPTS 3     // Tối đa 3 lần recovery

// ============================================================
//   NAVIGATOR STATES
// ============================================================
enum NavState {
  NAV_IDLE = 0,
  NAV_TRACKING,         // Áp dụng Backstepping bám Virtual Target
  NAV_FINAL_TURN,       // Xoay về góc chỉ định tại điểm đến
  NAV_PAUSED,           // Dừng khẩn cấp / Nhường đường
  NAV_RECOVERY_SPIN,    // Recovery: xoay tại chỗ
  NAV_RECOVERY_BACKUP,  // Recovery: lùi lại
  NAV_RECOVERY_WAIT,    // Recovery: chờ vật cản đi
  NAV_DONE,
  NAV_ERROR
};

// ============================================================
//   WAYPOINT STRUCTURE
// ============================================================
struct Waypoint {
  float x;
  float y;
  float heading;
  bool  useReverse;
};

// ============================================================
//   NAVIGATOR CLASS
// ============================================================
class Navigator {
public:
  NavState state = NAV_IDLE;
  NavState prePauseState = NAV_IDLE;
  Waypoint waypoints[MAX_WAYPOINTS];
  int      waypointCount = 0;
  int      currentWpIdx  = 0;
  float    finalHeading = NAN;
  
  float cmdLinear  = 0;
  float cmdAngular = 0;
  
  // Trạng thái của Virtual Reference Robot (Carrot)
  float ref_x = 0;
  float ref_y = 0;
  float ref_theta = 0;
  bool refInitialized = false; // FIX Bug #5: Dùng flag thay vì so sánh position

  // Sai số điều khiển (để vẽ biểu đồ trên Web)
  float error_x = 0;
  float error_y = 0;
  float error_yaw = 0;

  unsigned long navStartTime = 0;
  unsigned long lastWpReachTime = 0;

  // ── Recovery state ──
  int  recoveryAttempts = 0;
  unsigned long recoveryStartTime = 0;
  float progressCheckX = 0;
  float progressCheckY = 0;
  unsigned long lastProgressCheckTime = 0;

  void loadPath(Waypoint* wps, int count, float endHeading = NAN) {
    if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
    waypointCount = count;
    currentWpIdx = 0;
    finalHeading = endHeading;
    
    for (int i = 0; i < count; i++) {
        waypoints[i] = wps[i];
    }
    
    state = NAV_IDLE;
    cmdLinear = 0;
    cmdAngular = 0;
    navStartTime = millis();
    recoveryAttempts = 0;
    refInitialized = false; // Reset flag mỗi lần load path mới
    
    if (count > 0) {
      state = NAV_TRACKING;
      lastWpReachTime = millis();
      lastProgressCheckTime = millis();
      // Khởi tạo ở tọa độ (0,0) tạm thời, sẽ override ở update tick đầu tiên
      ref_x = 0;
      ref_y = 0; 
    }
  }
  
  void abort() {
    state = NAV_IDLE;
    waypointCount = 0;
    currentWpIdx = 0;
    cmdLinear = 0;
    cmdAngular = 0;
    recoveryAttempts = 0;
    Serial.println("[NAV] ABORTED");
  }

  void pause() {
    if (state != NAV_IDLE && state != NAV_DONE && state != NAV_ERROR && state != NAV_PAUSED) {
      prePauseState = state;
      state = NAV_PAUSED;
      cmdLinear = 0;
      cmdAngular = 0;
    }
  }

  void resume() {
    if (state == NAV_PAUSED) {
      state = prePauseState;
      navStartTime = millis(); 
      lastWpReachTime = millis();
      lastProgressCheckTime = millis();
    }
  }
  
  void update(float robotX, float robotY, float robotTheta) {
    if (state == NAV_IDLE || state == NAV_DONE || state == NAV_ERROR) {
      cmdLinear = 0;
      cmdAngular = 0;
      return;
    }

    if (state == NAV_PAUSED) {
      lastWpReachTime = millis(); // Prevent timeout while paused
      lastProgressCheckTime = millis();
      cmdLinear = 0;
      cmdAngular = 0;
      return;
    }

    // ── RECOVERY BEHAVIORS ──────────────────────────────────
    if (state == NAV_RECOVERY_SPIN) {
      unsigned long elapsed = millis() - recoveryStartTime;
      if (elapsed < NAV_RECOVERY_SPIN_MS) {
        cmdLinear = 0;
        cmdAngular = NAV_RECOVERY_SPIN_W;
      } else {
        // Spin done → try backup
        Serial.println("[NAV] Recovery SPIN done → BACKUP");
        state = NAV_RECOVERY_BACKUP;
        recoveryStartTime = millis();
      }
      return;
    }

    if (state == NAV_RECOVERY_BACKUP) {
      unsigned long elapsed = millis() - recoveryStartTime;
      if (elapsed < NAV_RECOVERY_BACKUP_MS) {
        cmdLinear = NAV_RECOVERY_BACKUP_V;
        cmdAngular = 0;
      } else {
        // Backup done → wait
        Serial.println("[NAV] Recovery BACKUP done → WAIT");
        state = NAV_RECOVERY_WAIT;
        recoveryStartTime = millis();
        cmdLinear = 0;
        cmdAngular = 0;
      }
      return;
    }

    if (state == NAV_RECOVERY_WAIT) {
      cmdLinear = 0;
      cmdAngular = 0;
      unsigned long elapsed = millis() - recoveryStartTime;
      if (elapsed >= NAV_RECOVERY_WAIT_MS) {
        // Wait done → resume tracking
        Serial.printf("[NAV] Recovery WAIT done (attempt %d/%d) → RESUME TRACKING\n", 
                      recoveryAttempts, NAV_MAX_RECOVERY_ATTEMPTS);
        state = NAV_TRACKING;
        lastWpReachTime = millis();
        lastProgressCheckTime = millis();
        progressCheckX = robotX;
        progressCheckY = robotY;
        // Re-sync virtual robot to current position
        ref_x = robotX;
        ref_y = robotY;
      }
      return;
    }

    // ── PROGRESS CHECK (stuck detection) ──────────────────
    if (state == NAV_TRACKING && millis() - lastProgressCheckTime > NAV_PROGRESS_CHECK_MS) {
      float moved = sqrtf((robotX - progressCheckX) * (robotX - progressCheckX) + 
                          (robotY - progressCheckY) * (robotY - progressCheckY));
      if (moved < NAV_PROGRESS_DIST) {
        // Robot stuck! Trigger recovery
        recoveryAttempts++;
        if (recoveryAttempts > NAV_MAX_RECOVERY_ATTEMPTS) {
          Serial.printf("[NAV] Max recovery attempts (%d) exceeded → ERROR\n", NAV_MAX_RECOVERY_ATTEMPTS);
          state = NAV_ERROR;
          cmdLinear = 0;
          cmdAngular = 0;
          return;
        }
        Serial.printf("[NAV] Stuck! (moved %.2fcm in %dms) → RECOVERY_SPIN (attempt %d)\n", 
                      moved * 100, NAV_PROGRESS_CHECK_MS, recoveryAttempts);
        state = NAV_RECOVERY_SPIN;
        recoveryStartTime = millis();
        return;
      }
      // Not stuck → reset progress check
      progressCheckX = robotX;
      progressCheckY = robotY;
      lastProgressCheckTime = millis();
    }
    
    // ── Waypoint timeout safety ──
    if (millis() - lastWpReachTime > NAV_WP_TIMEOUT_MS) {
      Serial.printf("[NAV] TIMEOUT at WP%d!\n", currentWpIdx);
      state = NAV_ERROR;
      cmdLinear = 0;
      cmdAngular = 0;
      return;
    }
    
    if (state == NAV_TRACKING) {
      // 1. Gắn Virtual Robot vào vị trí hiện tại của xe trong tick đầu tiên
      // FIX Bug #5: Dùng boolean flag thay vì so sánh ref_x == 0
      // (vì robot có thể spawn tại origin 0,0)
      if (!refInitialized) {
          ref_x = robotX;
          ref_y = robotY;
          progressCheckX = robotX;
          progressCheckY = robotY;
          refInitialized = true;
      }

      Waypoint& wp = waypoints[currentWpIdx];
      bool isFinalWp = (currentWpIdx == waypointCount - 1);
      
      float dx = wp.x - ref_x;
      float dy = wp.y - ref_y;
      float distToWp = sqrtf(dx * dx + dy * dy);
      
      float virtual_speed = NAV_MAX_LINEAR_VEL;
      
      // 2. Chỉnh tốc độ Virtual Robot
      if (isFinalWp && distToWp < NAV_SLOWDOWN_DIST) {
        virtual_speed = NAV_APPROACH_VEL + (NAV_MAX_LINEAR_VEL - NAV_APPROACH_VEL) * (distToWp / NAV_SLOWDOWN_DIST);
      }

      // Đàn hồi: Nếu xe thực tế bị tụt lùi > 0.3m, Virtual Robot phải dừng lại chờ
      float physDist = sqrtf((ref_x - robotX)*(ref_x - robotX) + (ref_y - robotY)*(ref_y - robotY));
      if (physDist > 0.4f) {
           virtual_speed = 0.0f; // Bị tụt quá xa, ngừng đi tiếp
      } else if (physDist > 0.2f) {
           virtual_speed *= 0.5f; // Đi chậm lại
      }

      float step = virtual_speed * NAV_DT; 
      
      // 3. Tiến Virtual Robot (Moving the Carrot)
      if (distToWp <= step && distToWp > 0.001f) {
        ref_x = wp.x;
        ref_y = wp.y;
        lastWpReachTime = millis();
        
        Serial.printf("[NAV] Passed WP%d\n", currentWpIdx);
        
        if (isFinalWp) {
          if (!isnan(finalHeading)) {
            state = NAV_FINAL_TURN;
          } else {
            state = NAV_DONE;
            Serial.println("[NAV] MISSION COMPLETE!");
          }
          cmdLinear = 0;
          cmdAngular = 0;
          return;
        } else {
          currentWpIdx++; // Nhắm waypoint tiếp theo
        }
      } else if (distToWp > 0.001f) {
        ref_x += (dx / distToWp) * step;
        ref_y += (dy / distToWp) * step;
      }
      
      ref_theta = atan2f(wp.y - ref_y, wp.x - ref_x);
      
      // 4. LUẬT BACKSTEPPING LYAPUNOV (High-Level Control)
      float e_x_global = ref_x - robotX;
      float e_y_global = ref_y - robotY;
      float e_theta = _normalizeAngle(ref_theta - robotTheta);
      
      // Transform qua Local Frame
      float ex = e_x_global * cosf(robotTheta) + e_y_global * sinf(robotTheta);
      float ey = -e_x_global * sinf(robotTheta) + e_y_global * cosf(robotTheta);
      
      this->error_x = ex;
      this->error_y = ey;
      this->error_yaw = e_theta;
      
      float ref_v = virtual_speed;
      float ref_w = 0.0f; 

      // Mẹo: Tăng hệ số bám khi góc lệch lớn (Gập khúc) để robot rẽ mạnh hơn trước khi chạy
      if (fabsf(e_theta) > PI/2) {
         ref_v *= 0.2f; 
      }

      cmdLinear = ref_v * cosf(e_theta) + GAIN_KX * ex;
      
      // Tránh Singularity: Đảm bảo bộ điều khiển vẫn có thể xuất momen xoay kể cả khi Virtual Robot đụng tới đích (ref_v = 0)
      float effective_v = fmax(ref_v, 0.05f); 
      cmdAngular = ref_w + effective_v * (GAIN_KY * ey + GAIN_KTH * sinf(e_theta));
      
      cmdLinear = constrain(cmdLinear, -NAV_MAX_LINEAR_VEL, NAV_MAX_LINEAR_VEL);
      cmdAngular = constrain(cmdAngular, -NAV_TURN_SPEED, NAV_TURN_SPEED);
      
    } else if (state == NAV_FINAL_TURN) {
      float finalErr = _normalizeAngle(finalHeading - robotTheta);
      if (fabsf(finalErr) < 0.052f) { // ~3 độ
        state = NAV_DONE;
        cmdLinear = 0;
        cmdAngular = 0;
        Serial.println("[NAV] FINAL TURN COMPLETE!");
      } else {
        cmdLinear = 0;
        float turnSpd = (fabsf(finalErr) < 0.26f) ? 0.3f : NAV_TURN_SPEED;
        cmdAngular = (finalErr > 0) ? turnSpd : -turnSpd;
      }
    }
  }
  
  const char* getStateName() {
    switch (state) {
      case NAV_IDLE:            return "IDLE";
      case NAV_TRACKING:        return "TRACK";
      case NAV_FINAL_TURN:      return "F_TURN";
      case NAV_PAUSED:          return "PAUSED";
      case NAV_RECOVERY_SPIN:   return "REC_SPIN";
      case NAV_RECOVERY_BACKUP: return "REC_BACK";
      case NAV_RECOVERY_WAIT:   return "REC_WAIT";
      case NAV_DONE:            return "DONE";
      case NAV_ERROR:           return "ERROR";
      default:                  return "???";
    }
  }
  
  bool isNavigating() {
    return state == NAV_TRACKING || state == NAV_FINAL_TURN || 
           state == NAV_RECOVERY_SPIN || state == NAV_RECOVERY_BACKUP || 
           state == NAV_RECOVERY_WAIT;
  }

  bool isRecovering() {
    return state == NAV_RECOVERY_SPIN || state == NAV_RECOVERY_BACKUP || state == NAV_RECOVERY_WAIT;
  }

private:
  float _normalizeAngle(float a) {
    while (a > PI)  a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
  }
};

#endif // NAVIGATOR_H
