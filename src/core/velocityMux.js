/**
 * AMR 2.0 — Velocity Priority Mux (Twist Mux)
 * 
 * Tham khảo: articubot_one twist_mux.yaml + ROS2 twist_mux package
 * 
 * Mục đích:
 *   Khi nhiều nguồn cùng gửi lệnh velocity (manual joystick, exploration auto,
 *   nav2-style path following), cần hệ thống ưu tiên để tránh xung đột.
 * 
 * Priority (cao hơn = ưu tiên hơn):
 *   EMERGENCY (100) — E-stop, obstacle avoidance
 *   MANUAL    (50)  — User joystick/keyboard control
 *   TRACKER   (20)  — Object tracking
 *   NAVIGATION(10)  — Autonomous exploration/path following
 * 
 * Logic:
 *   - Mỗi nguồn gọi velocityMux.send(source, linear, angular)
 *   - Mux chọn nguồn có priority cao nhất ĐANG ACTIVE
 *   - Nguồn bị timeout sau 500ms nếu không gửi lệnh mới → chuyển xuống nguồn thấp hơn
 */

// ============================================================
//   PRIORITY LEVELS (tham khảo articubot_one twist_mux.yaml)
// ============================================================

export const VEL_SOURCE = {
  NAVIGATION: 'navigation',   // priority 10
  TRACKER: 'tracker',         // priority 20
  MANUAL: 'manual',           // priority 50
  EMERGENCY: 'emergency',     // priority 100
};

const PRIORITY_MAP = {
  [VEL_SOURCE.NAVIGATION]: 10,
  [VEL_SOURCE.TRACKER]: 20,
  [VEL_SOURCE.MANUAL]: 50,
  [VEL_SOURCE.EMERGENCY]: 100,
};

const DEFAULT_TIMEOUT_MS = 3000; // PHẢI > exploration tick 2000ms, nếu không robot sẽ dừng giữa chừng

// ============================================================
//   VELOCITY MUX CLASS
// ============================================================

export class VelocityMux {
  constructor() {
    /** @type {Map<string, {linear: number, angular: number, timestamp: number, priority: number}>} */
    this.sources = new Map();
    
    /** Current active source name */
    this.activeSource = null;
    
    /** Callback khi velocity thay đổi */
    this.onVelocityChanged = null;
    
    /** Last sent velocity */
    this.lastLinear = 0;
    this.lastAngular = 0;
    
    /** Timeout cho mỗi source (ms) */
    this.timeoutMs = DEFAULT_TIMEOUT_MS;
  }

  /**
   * Gửi velocity command từ một source
   * @param {string} source - Tên nguồn (VEL_SOURCE.*)
   * @param {number} linear - Tốc độ tuyến tính (m/s)
   * @param {number} angular - Tốc độ góc (rad/s)
   */
  send(source, linear, angular) {
    const priority = PRIORITY_MAP[source];
    if (priority === undefined) {
      console.warn(`[VelMux] Unknown source: ${source}`);
      return;
    }

    this.sources.set(source, {
      linear,
      angular,
      timestamp: Date.now(),
      priority,
    });

    this._resolve();
  }

  /**
   * Hủy lệnh từ một source (ví dụ: user thả tay joystick)
   */
  release(source) {
    this.sources.delete(source);
    this._resolve();
  }

  /**
   * Resolve: chọn source có priority cao nhất chưa timeout
   */
  _resolve(forceEmit = false) {
    const now = Date.now();
    let bestSource = null;
    let bestPriority = -1;
    let bestData = null;

    for (const [name, data] of this.sources) {
      // Kiểm tra timeout
      if (now - data.timestamp > this.timeoutMs) {
        this.sources.delete(name);
        continue;
      }

      if (data.priority > bestPriority) {
        bestPriority = data.priority;
        bestSource = name;
        bestData = data;
      }
    }

    // Output velocity
    const linear = bestData?.linear ?? 0;
    const angular = bestData?.angular ?? 0;

    const changed = linear !== this.lastLinear || angular !== this.lastAngular || bestSource !== this.activeSource;

    // Gửi nếu thay đổi hoặc được yêu cầu gửi lại (để feed ESP32 watchdog khi đang có lệnh chạy)
    if (changed || (forceEmit && bestSource !== null)) {
      this.lastLinear = linear;
      this.lastAngular = angular;

      if (bestSource !== this.activeSource) {
        console.log(`[VelMux] Source: ${this.activeSource || 'none'} → ${bestSource || 'stop'} (pri=${bestPriority})`);
        this.activeSource = bestSource;
      }

      if (this.onVelocityChanged) {
        this.onVelocityChanged(linear, angular, bestSource);
      }
    }
  }

  /**
   * Gọi periodically (khoảng 100ms) để cleanup timeout sources
   */
  tick() {
    this._resolve(true);
  }

  /**
   * Get thông tin debug
   */
  getInfo() {
    const sources = {};
    for (const [name, data] of this.sources) {
      sources[name] = {
        linear: data.linear,
        angular: data.angular,
        priority: data.priority,
        age: Date.now() - data.timestamp,
      };
    }
    return {
      activeSource: this.activeSource,
      linear: this.lastLinear,
      angular: this.lastAngular,
      sources,
    };
  }

  /**
   * Emergency stop — clear tất cả và gửi stop
   */
  emergencyStop() {
    this.sources.clear();
    this.activeSource = null;
    this.lastLinear = 0;
    this.lastAngular = 0;
    if (this.onVelocityChanged) {
      this.onVelocityChanged(0, 0, 'emergency_stop');
    }
  }

  /**
   * Kiểm tra source nào đang active
   */
  isSourceActive(source) {
    return this.activeSource === source;
  }

  /**
   * Kiểm tra manual đang override auto không
   */
  isManualOverride() {
    return this.activeSource === VEL_SOURCE.MANUAL || this.activeSource === VEL_SOURCE.EMERGENCY;
  }
}

export default VelocityMux;
