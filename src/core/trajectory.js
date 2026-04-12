/**
 * AMR 2.0 — Pure Pursuit Trajectory Tracking
 * Thuật toán bám quỹ đạo cho robot differential-drive
 * 
 * Flow:
 *   1. App nhận odometry (x, y, θ) từ ESP32 qua WebSocket
 *   2. Pure Pursuit tính cmd_vel (linear, angular)
 *   3. App gửi cmd_vel xuống ESP32
 *   4. ESP32 PID điều khiển motor
 */

import { normalizeAngle, distance } from './warehouse.js';

// ============================================================
//   CONFIGURATION
// ============================================================

/** Pure Pursuit parameters */
const DEFAULT_CONFIG = {
  lookaheadDistance: 0.30,    // Khoảng nhìn trước (mét) — nhỏ = chính xác, lớn = mượt
  maxLinearVel: 0.35,         // Tốc độ thẳng tối đa (m/s)
  minLinearVel: 0.08,         // Tốc độ thẳng tối thiểu (m/s)
  maxAngularVel: 2.5,         // Tốc độ xoay tối đa (rad/s)
  goalTolerance: 0.05,        // Sai số vị trí chấp nhận (mét) — 5cm
  headingTolerance: 0.035,    // Sai số góc chấp nhận (rad) — ~2°
  slowdownDistance: 0.3,      // Bắt đầu giảm tốc khi cách đích (mét)
  rotationKp: 3.0,            // Hệ số P cho pivot turn
  rotationKd: 0.3,            // Hệ số D cho pivot turn
  updateInterval: 50,         // Chu kỳ cập nhật (ms) — 20Hz
};

// ============================================================
//   TRAJECTORY CONTROLLER CLASS
// ============================================================

/**
 * TrajectoryController — Bộ điều khiển quỹ đạo Pure Pursuit
 * 
 * Sử dụng:
 *   const tc = new TrajectoryController();
 *   tc.setPath(waypoints);
 *   // Trong loop 20Hz:
 *   const cmd = tc.update(robotX, robotY, robotTheta);
 *   // cmd = { linear: 0.3, angular: 0.5, done: false }
 */
export class TrajectoryController {
  constructor(config = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.path = [];              // Mảng waypoints [{x, y}]
    this.targetHeading = null;   // Góc đích cuối (rad)
    this.currentWaypointIdx = 0; // Waypoint hiện tại đang nhắm tới
    this.state = 'idle';         // idle | tracking | rotating | done
    this.prevHeadingError = 0;   // Cho D controller trong pivot turn
    this.onStateChange = null;   // Callback khi state thay đổi
  }

  /**
   * Đặt đường đi mới
   * @param {Array<{x, y}>} path - Mảng waypoints
   * @param {number|null} finalHeading - Góc đích cuối cùng (rad), null = giữ nguyên
   */
  setPath(path, finalHeading = null) {
    this.path = [...path];
    this.targetHeading = finalHeading;
    this.currentWaypointIdx = 0;
    this.prevHeadingError = 0;

    if (path.length === 0) {
      this._setState('idle');
    } else {
      this._setState('tracking');
    }
  }

  /**
   * Dừng ngay lập tức
   */
  stop() {
    this.path = [];
    this._setState('idle');
    return { linear: 0, angular: 0, done: true };
  }

  /**
   * Cập nhật — gọi ở 20Hz (50ms)
   * @param {number} robotX - Vị trí X robot (mét)
   * @param {number} robotY - Vị trí Y robot (mét)
   * @param {number} robotTheta - Hướng robot (rad)
   * @returns {{linear: number, angular: number, done: boolean, state: string, waypointIdx: number, progress: number}}
   */
  update(robotX, robotY, robotTheta) {
    if (this.state === 'idle' || this.state === 'done') {
      return { linear: 0, angular: 0, done: true, state: this.state, waypointIdx: 0, progress: 1 };
    }

    if (this.state === 'tracking') {
      return this._trackPath(robotX, robotY, robotTheta);
    }

    if (this.state === 'rotating') {
      return this._pivotTurn(robotX, robotY, robotTheta);
    }

    return { linear: 0, angular: 0, done: true, state: 'idle', waypointIdx: 0, progress: 1 };
  }

  /**
   * Tiến trình hoàn thành (0→1)
   */
  getProgress() {
    if (this.path.length === 0) return 1;
    return this.currentWaypointIdx / this.path.length;
  }

  // ============================================================
  //   PURE PURSUIT TRACKING
  // ============================================================

  _trackPath(robotX, robotY, robotTheta) {
    const cfg = this.config;
    const path = this.path;

    // Tìm waypoint gần nhất editable
    this._advanceWaypoint(robotX, robotY);

    // Kiểm tra đã đến đích chưa
    const lastWp = path[path.length - 1];
    const distToGoal = distance(robotX, robotY, lastWp.x, lastWp.y);

    if (distToGoal < cfg.goalTolerance) {
      // Đã đến vị trí đích
      if (this.targetHeading !== null) {
        // Cần xoay góc cuối cùng
        this._setState('rotating');
        return this._pivotTurn(robotX, robotY, robotTheta);
      } else {
        this._setState('done');
        return { linear: 0, angular: 0, done: true, state: 'done', waypointIdx: this.currentWaypointIdx, progress: 1 };
      }
    }

    // Tìm lookahead point
    const lookahead = this._findLookaheadPoint(robotX, robotY);
    if (!lookahead) {
      // Không tìm được lookahead → đi thẳng đến đích
      return this._goToPoint(robotX, robotY, robotTheta, lastWp.x, lastWp.y, distToGoal);
    }

    return this._goToPoint(robotX, robotY, robotTheta, lookahead.x, lookahead.y, distToGoal);
  }

  /**
   * Tính cmd_vel để đi đến một điểm
   */
  _goToPoint(robotX, robotY, robotTheta, targetX, targetY, distToGoal) {
    const cfg = this.config;

    // Góc từ robot đến target
    const angleToTarget = Math.atan2(targetY - robotY, targetX - robotX);
    const headingError = normalizeAngle(angleToTarget - robotTheta);

    // Pure Pursuit curvature
    const ld = distance(robotX, robotY, targetX, targetY);
    const curvature = (2.0 * Math.sin(headingError)) / Math.max(ld, 0.01);

    // Tốc độ thẳng — giảm khi gần đích và khi góc lệch lớn
    let linearVel = cfg.maxLinearVel;

    // Giảm tốc gần đích
    if (distToGoal < cfg.slowdownDistance) {
      linearVel *= distToGoal / cfg.slowdownDistance;
    }

    // Giảm tốc khi góc lệch lớn (tránh robot lượn vòng lớn)
    const absHeading = Math.abs(headingError);
    if (absHeading > 0.5) {
      linearVel *= Math.max(0.1, 1.0 - absHeading / Math.PI);
    }

    // Nếu góc lệch quá lớn (>90°), xoay tại chỗ trước
    if (absHeading > Math.PI / 2) {
      const angular = Math.sign(headingError) * Math.min(cfg.maxAngularVel, cfg.rotationKp * absHeading);
      return {
        linear: 0,
        angular,
        done: false,
        state: 'tracking',
        waypointIdx: this.currentWaypointIdx,
        progress: this.getProgress(),
      };
    }

    linearVel = Math.max(cfg.minLinearVel, linearVel);

    // Tốc độ xoay
    let angularVel = linearVel * curvature;
    angularVel = Math.max(-cfg.maxAngularVel, Math.min(cfg.maxAngularVel, angularVel));

    return {
      linear: linearVel,
      angular: angularVel,
      done: false,
      state: 'tracking',
      waypointIdx: this.currentWaypointIdx,
      progress: this.getProgress(),
    };
  }

  // ============================================================
  //   PIVOT TURN — Xoay tại chỗ (QUAN TRỌNG cho gắp hàng)
  // ============================================================

  _pivotTurn(robotX, robotY, robotTheta) {
    const cfg = this.config;
    const headingError = normalizeAngle(this.targetHeading - robotTheta);

    if (Math.abs(headingError) < cfg.headingTolerance) {
      this._setState('done');
      this.prevHeadingError = 0;
      return { linear: 0, angular: 0, done: true, state: 'done', waypointIdx: this.currentWaypointIdx, progress: 1 };
    }

    // PD controller cho xoay
    const dError = headingError - this.prevHeadingError;
    this.prevHeadingError = headingError;

    let angular = cfg.rotationKp * headingError + cfg.rotationKd * dError;
    angular = Math.max(-cfg.maxAngularVel, Math.min(cfg.maxAngularVel, angular));

    return {
      linear: 0,
      angular,
      done: false,
      state: 'rotating',
      waypointIdx: this.currentWaypointIdx,
      progress: this.getProgress(),
    };
  }

  // ============================================================
  //   LOOKAHEAD POINT FINDER
  // ============================================================

  /**
   * Tìm điểm lookahead trên path
   * Giao điểm giữa vòng tròn lookahead và các đoạn thẳng path
   */
  _findLookaheadPoint(robotX, robotY) {
    const ld = this.config.lookaheadDistance;

    for (let i = this.currentWaypointIdx; i < this.path.length - 1; i++) {
      const p1 = this.path[i];
      const p2 = this.path[i + 1];

      // Tìm giao điểm vòng tròn (robotX, robotY, ld) với đoạn p1-p2
      const intersections = circleLineIntersection(robotX, robotY, ld, p1.x, p1.y, p2.x, p2.y);

      if (intersections.length > 0) {
        // Chọn giao điểm xa nhất theo hướng path (gần p2 hơn)
        let best = intersections[0];
        let bestDist = distance(best.x, best.y, p2.x, p2.y);
        for (const pt of intersections) {
          const d = distance(pt.x, pt.y, p2.x, p2.y);
          if (d < bestDist) {
            bestDist = d;
            best = pt;
          }
        }
        return best;
      }
    }

    // Fallback: trả về waypoint tiếp theo
    if (this.currentWaypointIdx < this.path.length) {
      return this.path[Math.min(this.currentWaypointIdx + 1, this.path.length - 1)];
    }

    return null;
  }

  /**
   * Tiến waypoint index khi robot vượt qua
   */
  _advanceWaypoint(robotX, robotY) {
    while (this.currentWaypointIdx < this.path.length - 1) {
      const wp = this.path[this.currentWaypointIdx];
      if (distance(robotX, robotY, wp.x, wp.y) < this.config.lookaheadDistance) {
        this.currentWaypointIdx++;
      } else {
        break;
      }
    }
  }

  _setState(newState) {
    if (this.state !== newState) {
      const oldState = this.state;
      this.state = newState;
      if (this.onStateChange) {
        this.onStateChange(newState, oldState);
      }
    }
  }
}

// ============================================================
//   GEOMETRY UTIL
// ============================================================

/**
 * Tìm giao điểm vòng tròn (cx, cy, r) với đoạn thẳng (x1,y1)-(x2,y2)
 * @returns {Array<{x, y}>}
 */
function circleLineIntersection(cx, cy, r, x1, y1, x2, y2) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const fx = x1 - cx;
  const fy = y1 - cy;

  const a = dx * dx + dy * dy;
  const b = 2 * (fx * dx + fy * dy);
  const c = fx * fx + fy * fy - r * r;

  let discriminant = b * b - 4 * a * c;
  if (discriminant < 0) return [];

  discriminant = Math.sqrt(discriminant);
  const results = [];

  const t1 = (-b - discriminant) / (2 * a);
  const t2 = (-b + discriminant) / (2 * a);

  if (t1 >= 0 && t1 <= 1) {
    results.push({ x: x1 + t1 * dx, y: y1 + t1 * dy });
  }
  if (t2 >= 0 && t2 <= 1 && Math.abs(t2 - t1) > 0.001) {
    results.push({ x: x1 + t2 * dx, y: y1 + t2 * dy });
  }

  return results;
}

export default TrajectoryController;
