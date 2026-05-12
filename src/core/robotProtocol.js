/**
 * AMR 2.0 — WebSocket Robot Protocol
 * Giao thức truyền thông giữa App ↔ ESP32-S3
 * 
 * Phase: MessagePack Binary Protocol
 * - Outbound (Browser → ESP32): encode bằng MessagePack binary
 * - Inbound (ESP32 → Browser): auto-detect JSON text / MsgPack binary / Grid binary
 * - Backward-compatible: text frames = JSON, binary frames = MsgPack hoặc Grid
 */

import { encode, decode } from '@msgpack/msgpack';
import { SimEngine } from './sim/simEngine.js';

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
    this.missedPongs = 0;  // Count consecutive missed pongs

    // --- HITL MODE ---
    this.hitlEnabled = false;
    this.hitlEngine = new SimEngine();
    this.hitlTimer = null;

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
      architecture: 'hybrid',
      gridStreamEnabled: true,
      onboardNavEnabled: true,
      slam: { score: 0, tfNorm: 0, tfDeg: 0, coverage: 0 },
      path: [],              // ESP32 generated onboard A* path
    };

    // Callbacks
    this.onTelemetry = null;       // (telemetry) => {}
    this.onConnect = null;          // () => {}
    this.onDisconnect = null;       // () => {}
    this.onError = null;            // (error) => {}
    this.onNavAck = null;           // (data) => {} — ESP32 xác nhận đã nhận path
    this.onMapAck = null;           // (data) => {} — ESP32 xác nhận đã nhận static map
    this.onLidarGrid = null;       // (grid) => {} — Occupancy grid from LIDAR
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
      // Nhận binary frames dạng ArrayBuffer (không phải Blob)
      this.ws.binaryType = 'arraybuffer';

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
        if (event.data instanceof ArrayBuffer) {
          this._dispatchBinaryMessage(event.data);
        } else {
          // Text frame — JSON (legacy hoặc fallback)
          this._handleJsonMessage(event.data);
        }
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
  //   SEND COMMANDS (MessagePack Binary)
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
   * Bật/tắt chế độ HITL (Hardware-in-the-loop)
   */
  setHitlMode(enabled) {
    this.hitlEnabled = enabled;
    if (this.connected) {
      this._send({ cmd: 'hitl_mode', enable: enabled });
    }
    
    if (enabled) {
      // Load SLAM test map (corridors, boxes, pillars)
      this.hitlEngine.world.loadSlamTestMap();
      
      // Start sim engine at the test map's spawn point
      const spawn = this.hitlEngine.world.defaultSpawn;
      this.hitlEngine.start();
      this.hitlEngine.reset(spawn.x, spawn.y, spawn.theta);
      
      // Sync ESP32 pose to spawn position
      if (this.connected) {
        this._send({ cmd: 'set_pose', x: spawn.x, y: spawn.y, theta: spawn.theta });
      }
      
      console.log(`[HITL] SLAM Test Map loaded. Spawn: (${spawn.x}, ${spawn.y})`);
      
      this.hitlTimer = setInterval(() => {
        if (!this.connected) return;
        
        const simTelem = this.hitlEngine.telemetry;
        
        // Send virtual sensors to ESP32
        this._send({
          cmd: 'hitl_sensor',
          x: simTelem.x,
          y: simTelem.y,
          theta: simTelem.headingRad,
          lidar: simTelem.lidar || []
        });
      }, 100); // 10Hz
    } else {
      this.hitlEngine.stop();
      if (this.hitlTimer) {
        clearInterval(this.hitlTimer);
        this.hitlTimer = null;
      }
    }
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
   * Phân tán: Gửi tọa độ đích để ESP32 tự tìm đường (A*)
   */
  goto(x, y, finalHeading = null) {
    const msg = { cmd: 'goto', x, y };
    if (finalHeading !== null && finalHeading !== undefined) {
      msg.finalHeading = finalHeading;
    }
    this._send(msg);
    console.log(`[GOTO] Sent target (${x}, ${y}), finalH=${finalHeading}°`);
  }

  /**
   * Phân tán: Gửi bản đồ tĩnh dạng Binary kèm header 9 bytes
   * Layout: [0x03][width:uint16_LE][height:uint16_LE][res:float32_LE][cells:int8[]]
   * @param {Object} gridOrData - OccupancyGrid instance (phải có .width, .height, .resolution, .data hoặc .logOdds)
   * @returns {boolean} true nếu gửi thành công
   */
  sendMapData(gridOrData) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false;

    // Hỗ trợ cả grid instance (có .serialize()) và raw data object
    let width, height, resolution, cells;

    if (typeof gridOrData.serialize === 'function') {
      // OccupancyGrid instance — extract dimensions + binary cells
      width = gridOrData.width;
      height = gridOrData.height;
      resolution = gridOrData.resolution;
      // Convert logOdds → binary cells: >0 = occupied(100), <0 = free(0), ~0 = unknown(50)
      const lo = gridOrData.logOdds || gridOrData.data;
      cells = new Int8Array(width * height);
      for (let i = 0; i < width * height; i++) {
        if (lo[i] > 0.4) cells[i] = 100;       // Occupied
        else if (lo[i] < -0.4) cells[i] = 0;    // Free
        else cells[i] = 50;                       // Unknown
      }
    } else if (gridOrData.width && gridOrData.height && gridOrData.cells) {
      // Pre-built data object with { width, height, resolution, cells: Int8Array }
      width = gridOrData.width;
      height = gridOrData.height;
      resolution = gridOrData.resolution;
      cells = gridOrData.cells instanceof Int8Array ? gridOrData.cells : new Int8Array(gridOrData.cells);
    } else {
      console.error('[MAP] sendMapData: invalid grid format');
      return false;
    }

    // Chunking the map to prevent ESP32 WebSocket max payload limit (Error 1009)
    const CHUNK_SIZE = 8192;
    for (let offset = 0; offset < cells.length; offset += CHUNK_SIZE) {
      const chunkLength = Math.min(CHUNK_SIZE, cells.length - offset);
      // Build binary frame: [type:1][w:2][h:2][res:4][offset:4][cells:chunkLength]
      const HEADER_SIZE = 13; // 1 + 2 + 2 + 4 + 4
      const totalSize = HEADER_SIZE + chunkLength;
      const buffer = new ArrayBuffer(totalSize);
      const view = new DataView(buffer);

      view.setUint8(0, 0x03);                      // Type: MAP_DATA
      view.setUint16(1, width, true);               // Width (Little Endian)
      view.setUint16(3, height, true);              // Height (Little Endian)
      view.setFloat32(5, resolution, true);         // Resolution (Little Endian)
      view.setUint32(9, offset, true);              // Offset (Little Endian)

      // Copy cell data after header
      const byteView = new Uint8Array(buffer);
      byteView.set(new Uint8Array(cells.buffer, offset, chunkLength), HEADER_SIZE);

      this.ws.send(buffer);
    }
    
    console.log(`[MAP] Sent static map chunks: ${width}x${height} @${resolution}m, total ${cells.length} bytes`);
    return true;
  }

  /**
   * Yêu cầu ESP32 push lại bản đồ (nếu có) — hiện chưa dùng nhưng reserved
   */
  requestMap() {
    this._send({ cmd: 'map_request' });
    console.log('[MAP] Requesting map from ESP32...');
  }


  /**
   * Phân tán: Gửi tọa độ các xe khác để né
   */
  sendTraffic(robotsArray) {
    this._send({ cmd: 'traffic', robots: robotsArray });
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

  setArchitectureProfile(profile = 'hybrid') {
    this._send({ cmd: 'set_arch_mode', profile });
  }

  // ============================================================
  //   INTERNAL — SEND
  // ============================================================

  /**
   * Gửi data qua WebSocket.
   * Ưu tiên MessagePack binary, fallback sang JSON text nếu encode lỗi.
   */
  _send(data) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return false;
    }
    try {
      const packed = encode(data);
      this.ws.send(packed);
      return true;
    } catch (err) {
      // Fallback: JSON text nếu MsgPack encode lỗi
      try {
        this.ws.send(JSON.stringify(data));
        return true;
      } catch (e2) {
        console.error(`[Robot ${this.name}] Lỗi gửi:`, e2);
        return false;
      }
    }
  }

  // ============================================================
  //   INTERNAL — RECEIVE
  // ============================================================

  /**
   * Dispatch binary message dựa trên byte đầu tiên:
   * - 0x01 = Occupancy Grid (custom binary format)
   * - 0x02 = MessagePack telemetry (có type marker từ ESP32 mới)
   * - Khác = MessagePack thuần (không có type marker)
   */
  _dispatchBinaryMessage(buffer) {
    if (buffer.byteLength === 0) return;

    const firstByte = new Uint8Array(buffer)[0];

    if (firstByte === 0x01) {
      // Occupancy Grid — custom binary protocol (legacy)
      this._handleGridMessage(buffer);
    } else if (firstByte === 0x02) {
      // MessagePack telemetry với type marker — skip byte đầu
      this._handleMsgPackMessage(buffer.slice(1));
    } else {
      // MessagePack thuần (không có marker) hoặc legacy MsgPack
      this._handleMsgPackMessage(buffer);
    }
  }

  /**
   * Decode MessagePack binary → JS object → xử lý
   */
  _handleMsgPackMessage(buffer) {
    try {
      const data = decode(new Uint8Array(buffer));

      // PONG response
      if (data.type === 'pong') {
        this.latency = Date.now() - data.ts;
        this.lastPong = Date.now();
        return;
      }

      // NAV_ACK
      if (data.type === 'nav_ack') {
        console.log(`[Robot ${this.name}] NAV_ACK: ${data.wp_count} waypoints, finalH=${data.finalH}°`);
        if (this.onNavAck) this.onNavAck(data);
        return;
      }

      // MAP_ACK — ESP32 confirmed static map received
      if (data.type === 'map_ack') {
        console.log(`[Robot ${this.name}] MAP_ACK: ${data.w}x${data.h}, ${data.bytes} bytes`);
        if (this.onMapAck) this.onMapAck(data);
        return;
      }

      // Telemetry
      if (data.x !== undefined || data.batt !== undefined) {
        this._processTelemetryData(data);
      }
    } catch (err) {
      console.error(`[Robot ${this.name}] MsgPack decode error:`, err);
    }
  }

  /**
   * Handle JSON text messages (legacy / fallback)
   */
  _handleJsonMessage(raw) {
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

      // MAP_ACK — ESP32 confirmed static map received
      if (data.type === 'map_ack') {
        console.log(`[Robot ${this.name}] MAP_ACK: ${data.w}x${data.h}, ${data.bytes} bytes`);
        if (this.onMapAck) this.onMapAck(data);
        return;
      }

      // Telemetry
      if (data.x !== undefined || data.batt !== undefined) {
        this._processTelemetryData(data);
      }
    } catch (err) {
      // Bỏ qua message không parse được
    }
  }

  /**
   * Occupancy Grid — custom binary message (byte[0] = 0x01)
   * Giữ nguyên logic cũ.
   */
  _handleGridMessage(buffer) {
    try {
      import('../core/lidarMapper.js').then(module => {
        const OccupancyGrid = module.default;
        const grid = OccupancyGrid.fromBinary(buffer);
        
        if (this.onLidarGrid) {
          this.onLidarGrid(grid);
        }
      }).catch(err => {
        console.error(`[Robot ${this.name}] Error importing lidarMapper:`, err);
      });
    } catch (err) {
      console.error(`[Robot ${this.name}] Error parsing binary message:`, err);
    }
  }

  /**
   * Xử lý telemetry data (shared giữa JSON và MsgPack paths)
   * DRY: Cả 2 luồng decode đều gọi hàm này.
   */
  _processTelemetryData(data) {
    this.lastPong = Date.now(); // Telemetry is a valid heartbeat
    this.missedPongs = 0;       // Reset miss counter — ESP32 is alive
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
      architecture: data.arch ?? this.telemetry.architecture,
      gridStreamEnabled: data.grid_stream ?? this.telemetry.gridStreamEnabled,
      onboardNavEnabled: data.onboard_nav ?? this.telemetry.onboardNavEnabled,
      // INA3221 Power
      battV: data.power?.battV ?? this.telemetry.battV,
      battA: data.power?.battA ?? this.telemetry.battA,
      motorV: data.power?.motorV ?? this.telemetry.motorV,
      motorA: data.power?.motorA ?? this.telemetry.motorA,
      obs: data.obs ?? this.telemetry.obs,
      // Lidar scan data (raw pass-through for mapping)
      lidar: data.lidar || [],
      // HITL mode
      hitl: data.hitl ?? this.telemetry.hitl,
      slam: data.slam ?? this.telemetry.slam,
      // SLAM map-frame pose (from ESP32 ICP correction)
      // These are the corrected positions used by firmware for grid mapping
      slamMapX: data.slam?.mX ?? this.telemetry.slamMapX,
      slamMapY: data.slam?.mY ?? this.telemetry.slamMapY,
      slamMapTheta: data.slam?.mTh != null ? (data.slam.mTh * Math.PI / 180) : this.telemetry.slamMapTheta,
      // Exploration (Onboard SLAM)
      explore: data.explore ?? this.telemetry.explore,
      explore_goals: data.explore_goals ?? this.telemetry.explore_goals,
      explore_frontiers: data.explore_frontiers ?? this.telemetry.explore_frontiers,
      path: data.path ?? this.telemetry.path,
    };

    if (this.hitlEnabled) {
      // Convert wheel angular velocities (rad/s) from ESP32 to robot velocities (m/s, rad/s)
      const WHEEL_RADIUS = 0.0264;      // Must match ESP32 config.h
      const WHEEL_SEPARATION = 0.170;   // Must match ESP32 config.h
      
      const vL_ms = (data.vL_r || 0) * WHEEL_RADIUS;
      const vR_ms = (data.vR_r || 0) * WHEEL_RADIUS;
      
      const v = (vL_ms + vR_ms) / 2.0;
      const w = (vR_ms - vL_ms) / WHEEL_SEPARATION;
      
      this.hitlEngine.setVelocity(v, w);
      
      // Override lidar with virtual lidar for UI
      if (this.hitlEngine.telemetry && this.hitlEngine.telemetry.lidar) {
         this.telemetry.lidar = this.hitlEngine.telemetry.lidar;
         
         // Fix Costmap Flickering: 
         // Force UI telemetry pose to perfectly match the hitlEngine pose used to generate the lidar rays.
         // Otherwise, the slight integration drift between ESP32 odom and UI odom causes lidar rays to erase static walls.
         const simPose = this.hitlEngine.getPose();
         this.telemetry.x = simPose.x;
         this.telemetry.y = simPose.y;
         this.telemetry.headingRad = simPose.theta;
         this.telemetry.heading = simPose.theta * 180 / Math.PI;
      }
    }

    if (this.onTelemetry) this.onTelemetry(this.telemetry);
  }

  // ============================================================
  //   PING / PONG / RECONNECT
  // ============================================================

  _startPing() {
    this._stopPing();
    this.missedPongs = 0;
    this._connectTime = Date.now(); // Grace period: ESP32 busy with MQTT+I2C after boot
    this.pingTimer = setInterval(() => {
      this._send({ type: 'ping', ts: Date.now() });

      // Grace period: Skip pong checks for first 25s after connection.
      // ESP32 main loop blocks ~2s during MQTT broker connect + I2C OLED/INA reads,
      // starving webSocket.loop() and preventing pong responses.
      const sinceConnect = Date.now() - this._connectTime;
      if (sinceConnect < 25000) return;

      // Kiểm tra pong timeout (20 giây — ESP32 cần serialize
      // telemetry 8KB + lidar JSON + grid 20KB + ICP + PID + MQTT keepalive)
      if (Date.now() - this.lastPong > 20000 && this.connected) {
        this.missedPongs++;
        console.warn(`[Robot ${this.name}] Pong timeout #${this.missedPongs}`);
        
        // Cần 3 lần liên tiếp mất pong mới thực sự disconnect
        // (tăng tolerance cho WiFi hiccup + ESP32 CPU-heavy khi lái)
        if (this.missedPongs >= 3) {
          console.error(`[Robot ${this.name}] ${this.missedPongs} consecutive pong misses → disconnecting`);
          this.disconnect();
          this._scheduleReconnect();
        }
      } else {
        this.missedPongs = 0;  // Reset counter khi nhận được pong
      }
    }, 5000);  // Ping mỗi 5 giây (giảm tải cho ESP32)
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
