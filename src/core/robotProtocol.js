/**
 * AMR 2.0 — WebSocket Robot Protocol
 * Giao thức truyền thông giữa App ↔ ESP32-S3
 */

// ============================================================
//   ROBOT CONNECTION CLASS
// ============================================================

/**
 * RobotConnection — Kết nối WebSocket đến 1 robot ESP32-S3
 * 
 * Sử dụng:
 *   const robot = new RobotConnection('192.168.1.100', 81, 'Robot 1');
 *   robot.onTelemetry = (data) => { ... };
 *   robot.connect();
 *   robot.sendVelocity(0.3, 0.5);
 */
export class RobotConnection {
  constructor(ip, port = 81, name = 'Robot') {
    this.ip = ip;
    this.port = port;
    this.name = name;
    this.ws = null;
    this.connected = false;
    this.reconnecting = false;
    this.reconnectTimer = null;
    this.pingTimer = null;
    this.lastPong = 0;
    this.latency = 0;

    // Telemetry data (cập nhật liên tục từ robot)
    this.telemetry = {
      x: 0, y: 0,           // Vị trí (mét)
      heading: 0,            // Hướng (độ)
      headingRad: 0,         // Hướng (rad)
      distance: 0,           // Quãng đường (mét)
      linearVel: 0,          // Vận tốc thẳng (m/s)
      angularVel: 0,         // Vận tốc xoay (rad/s)
      battery: 100,          // Pin (%)
      imuAvailable: false,   // IMU có sẵn
      imuCalibrated: false,  // IMU đã calibrate
      encoderLeft: 0,        // Encoder trái (ticks)
      encoderRight: 0,       // Encoder phải (ticks)
      targetVelL: 0,         // Target velocity trái
      targetVelR: 0,         // Target velocity phải
      measuredVelL: 0,       // Measured velocity trái
      measuredVelR: 0,       // Measured velocity phải
      pwmLeft: 0,            // PWM output trái
      pwmRight: 0,           // PWM output phải
      nav: 'IDLE',           // Navigator state (IDLE/TURN/DRIVE/REV/F_TURN/DONE/ERROR)
      navWp: 0,              // Current waypoint index
      navTotal: 0,           // Total waypoints
      // INA3221 Power Monitor
      battV: 0,              // Điện áp pin (V)
      battA: 0,              // Dòng pin (A)
      motorV: 0,             // Điện áp motor (V)
      motorA: 0,             // Dòng motor (A)
      obs: false,            // Obstacle detected
    };

    // Callbacks
    this.onTelemetry = null;       // (telemetry) => {}
    this.onConnect = null;          // () => {}
    this.onDisconnect = null;       // () => {}
    this.onError = null;            // (error) => {}
    this.onNavAck = null;           // (data) => {} — ESP32 xác nhận đã nhận path
  }

  // ============================================================
  //   CONNECTION MANAGEMENT
  // ============================================================

  connect() {
    if (this.ws) {
      this.ws.close();
    }

    const url = `ws://${this.ip}:${this.port}/`;
    console.log(`[Robot ${this.name}] Kết nối ${url}...`);

    try {
      this.ws = new WebSocket(url);

      this.ws.onopen = () => {
        console.log(`[Robot ${this.name}] ✅ Đã kết nối!`);
        this.connected = true;
        this.reconnecting = false;
        this.lastPong = Date.now();

        // Bắt đầu ping/pong
        this._startPing();

        if (this.onConnect) this.onConnect();
      };

      this.ws.onclose = (e) => {
        console.log(`[Robot ${this.name}] ❌ Mất kết nối (code: ${e.code})`);
        this.connected = false;
        this._stopPing();

        if (this.onDisconnect) this.onDisconnect();

        // Auto reconnect
        if (!this.reconnecting) {
          this._scheduleReconnect();
        }
      };

      this.ws.onerror = (err) => {
        console.error(`[Robot ${this.name}] Lỗi WebSocket:`, err);
        if (this.onError) this.onError(err);
      };

      this.ws.onmessage = (event) => {
        this._handleMessage(event.data);
      };
    } catch (err) {
      console.error(`[Robot ${this.name}] Không thể tạo kết nối:`, err);
      this._scheduleReconnect();
    }
  }

  disconnect() {
    this.reconnecting = false;
    clearTimeout(this.reconnectTimer);
    this._stopPing();

    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.connected = false;
  }

  // ============================================================
  //   SEND COMMANDS
  // ============================================================

  /**
   * Gửi lệnh vận tốc (linear + angular)
   * @param {number} linear  - Tốc độ thẳng (m/s)
   * @param {number} angular - Tốc độ xoay (rad/s)
   */
  sendVelocity(linear, angular) {
    this._send({ linear, angular });
  }

  /**
   * Dừng robot
   */
  sendStop() {
    this._send({ linear: 0, angular: 0 });
  }

  /**
   * Reset odometry
   */
  resetOdometry() {
    this._send({ cmd: 'reset_odom' });
  }

  /**
   * Truyền tọa độ khởi tạo (set_pose)
   */
  setPose(x, y, theta) {
    this._send({ cmd: 'set_pose', x, y, theta });
  }

  /**
   * Gửi lộ trình cho xe tự lái — ESP32 tự bám đường
   * @param {Array<{x: number, y: number}>} path - Danh sách waypoint (mét)
   * @param {number|null} finalHeading - Góc cuối tại đích (độ), null nếu không cần
   */
  navigate(path, finalHeading = null) {
    const msg = { cmd: 'navigate', path };
    if (finalHeading !== null && finalHeading !== undefined) {
      msg.finalHeading = finalHeading;
    }
    this._send(msg);
    console.log(`[Navigate] Sent ${path.length} waypoints, finalH=${finalHeading}°`);
  }

  /**
   * Dừng tự lái ngay lập tức
   */
  navStop() {
    this._send({ cmd: 'nav_stop' });
  }

  /**
   * Tạm dừng tự lái (Nhường đường)
   */
  pause() {
    this._send({ cmd: 'pause' });
  }

  /**
   * Tiếp tục tự lái
   */
  resume() {
    this._send({ cmd: 'resume' });
  }

  /**
   * Gửi cấu hình robot
   */
  sendConfig(config) {
    this._send({ type: 'config', ...config });
  }

  /**
   * Recalibrate gyro IMU
   */
  recalibrateGyro() {
    this._send({ cmd: 'recal_gyro' });
  }

  /**
   * Toggle brake mode
   */
  setBrake(enabled) {
    this._send({ cmd: 'brake', val: enabled });
  }

  // ============================================================
  //   INTERNAL
  // ============================================================

  _send(data) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return false;
    }
    try {
      this.ws.send(JSON.stringify(data));
      return true;
    } catch (err) {
      console.error(`[Robot ${this.name}] Lỗi gửi:`, err);
      return false;
    }
  }

  _handleMessage(raw) {
    try {
      const data = JSON.parse(raw);

      // PONG response
      if (data.type === 'pong') {
        this.latency = Date.now() - data.ts;
        this.lastPong = Date.now();
        return;
      }

      // NAV_ACK — ESP32 đã nhận path thành công
      if (data.type === 'nav_ack') {
        console.log(`[Robot ${this.name}] NAV_ACK: ${data.wp_count} waypoints, finalH=${data.finalH}°`);
        if (this.onNavAck) this.onNavAck(data);
        return;
      }

      // Telemetry
      if (data.telem) {
        this.telemetry = {
          x: data.x ?? this.telemetry.x,
          y: data.y ?? this.telemetry.y,
          heading: data.h ?? this.telemetry.heading,
          headingRad: (data.h ?? 0) * Math.PI / 180,
          distance: data.d ?? this.telemetry.distance,
          linearVel: data.vx ?? this.telemetry.linearVel,
          angularVel: data.wz ?? this.telemetry.angularVel,
          battery: data.batt ?? this.telemetry.battery,
          imuAvailable: data.imu ?? this.telemetry.imuAvailable,
          imuCalibrated: data.imu_cal ?? this.telemetry.imuCalibrated,
          encoderLeft: data.enc?.l ?? this.telemetry.encoderLeft,
          encoderRight: data.enc?.r ?? this.telemetry.encoderRight,
          targetVelL: data.vL_t ?? this.telemetry.targetVelL,
          targetVelR: data.vR_t ?? this.telemetry.targetVelR,
          measuredVelL: data.vL_r ?? this.telemetry.measuredVelL,
          measuredVelR: data.vR_r ?? this.telemetry.measuredVelR,
          pwmLeft: data.pwmL ?? this.telemetry.pwmLeft,
          pwmRight: data.pwmR ?? this.telemetry.pwmRight,
          nav: data.nav ?? this.telemetry.nav,
          navWp: data.nav_wp ?? this.telemetry.navWp,
          navTotal: data.nav_total ?? this.telemetry.navTotal,
          // INA3221 Power
          battV: data.power?.battV ?? this.telemetry.battV,
          battA: data.power?.battA ?? this.telemetry.battA,
          motorV: data.power?.motorV ?? this.telemetry.motorV,
          motorA: data.power?.motorA ?? this.telemetry.motorA,
          obs: data.obs ?? this.telemetry.obs,
        };

        if (this.onTelemetry) this.onTelemetry(this.telemetry);
      }
    } catch (err) {
      // Bỏ qua message không parse được
    }
  }

  _startPing() {
    this._stopPing();
    this.pingTimer = setInterval(() => {
      this._send({ type: 'ping', ts: Date.now() });

      // Kiểm tra pong timeout (2.5 giây)
      if (Date.now() - this.lastPong > 2500 && this.connected) {
        console.warn(`[Robot ${this.name}] Pong timeout — mất kết nối?`);
        this.disconnect();
        this._scheduleReconnect();
      }
    }, 1000);
  }

  _stopPing() {
    if (this.pingTimer) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }
  }

  _scheduleReconnect() {
    this.reconnecting = true;
    console.log(`[Robot ${this.name}] Reconnect sau 1 giây...`);
    this.reconnectTimer = setTimeout(() => {
      if (this.reconnecting) {
        this.connect();
      }
    }, 1000);
  }
}

export default RobotConnection;
