/**
 * AMR 2.0 — RobotAdapter
 * Tầng trừu tượng thống nhất giao tiếp giữa Real Robot (ESP32) và Sim Robot (GazeboTDTU).
 * 
 * robotStore chỉ cần gọi adapter.sendVelocity(), adapter.navigate()...
 * mà KHÔNG cần biết robot là thật hay mô phỏng.
 * 
 * Pattern: Adapter / Strategy — unified interface, pluggable backends.
 */

import { RobotConnection } from './robotProtocol.js';
import { SimEngine } from './sim/simEngine.js';

// ============================================================
//   BASE ADAPTER (Interface contract)
// ============================================================

class BaseRobotAdapter {
  constructor(type) {
    /** @type {'real'|'sim'} */
    this.type = type;
    this.connected = false;

    // Callbacks — set by store
    this.onTelemetry = null;
    this.onConnect = null;
    this.onDisconnect = null;
    this.onError = null;
    this.onNavAck = null;
    this.onLidarGrid = null;
  }

  // — Connection —
  connect() { throw new Error('Not implemented'); }
  disconnect() { throw new Error('Not implemented'); }

  // — Control —
  sendVelocity(linear, angular) { throw new Error('Not implemented'); }
  sendStop() { this.sendVelocity(0, 0); }
  navigate(path, finalHeading = null) { throw new Error('Not implemented'); }
  navStop() { throw new Error('Not implemented'); }
  pause() { throw new Error('Not implemented'); }
  resume() { throw new Error('Not implemented'); }

  // — Config —
  resetOdometry() { throw new Error('Not implemented'); }
  setPose(x, y, theta) { throw new Error('Not implemented'); }
  recalibrateGyro() { throw new Error('Not implemented'); }
  setBrake(enabled) { throw new Error('Not implemented'); }
  setArchitectureProfile(profile) { throw new Error('Not implemented'); }
  sendConfig(config) { /* optional */ }

  // — Cleanup —
  destroy() { this.disconnect(); }
}

// ============================================================
//   REAL ROBOT ADAPTER (WebSocket → ESP32-S3)
// ============================================================

export class RealRobotAdapter extends BaseRobotAdapter {
  /**
   * @param {string} ip   - IP address of ESP32
   * @param {number} port - WebSocket port (default 81)
   * @param {string} name - Human-readable robot name
   */
  constructor(ip, port = 81, name = 'Robot') {
    super('real');
    this.ip = ip;
    this.port = port;
    this.name = name;

    // Internal RobotConnection instance
    this._conn = new RobotConnection(ip, port, name);

    // Wire callbacks from RobotConnection → Adapter callbacks
    this._conn.onTelemetry = (telem) => {
      if (this.onTelemetry) this.onTelemetry(telem);
    };
    this._conn.onConnect = () => {
      this.connected = true;
      if (this.onConnect) this.onConnect();
    };
    this._conn.onDisconnect = () => {
      this.connected = false;
      if (this.onDisconnect) this.onDisconnect();
    };
    this._conn.onError = (err) => {
      if (this.onError) this.onError(err);
    };
    this._conn.onNavAck = (data) => {
      if (this.onNavAck) this.onNavAck(data);
    };
    this._conn.onLidarGrid = (grid) => {
      if (this.onLidarGrid) this.onLidarGrid(grid);
    };
  }

  connect() { this._conn.connect(); }
  disconnect() { this._conn.disconnect(); this.connected = false; }

  sendVelocity(linear, angular) { this._conn.sendVelocity(linear, angular); }
  sendStop() { this._conn.sendStop(); }
  navigate(path, finalHeading = null) { this._conn.navigate(path, finalHeading); }
  goto(x, y, finalHeading = null) { this._conn.goto(x, y, finalHeading); }
  navStop() { this._conn.navStop(); }
  pause() { this._conn.pause(); }
  resume() { this._conn.resume(); }

  resetOdometry() { this._conn.resetOdometry(); }
  setPose(x, y, theta) { this._conn.setPose(x, y, theta); }
  recalibrateGyro() { this._conn.recalibrateGyro(); }
  setBrake(enabled) { this._conn.setBrake(enabled); }
  setArchitectureProfile(profile) { this._conn.setArchitectureProfile(profile); }
  sendConfig(config) { this._conn.sendConfig(config); }
  sendMapData(grid) { return this._conn.sendMapData(grid); }

  destroy() {
    this._conn.disconnect();
    this.connected = false;
  }
}

// ============================================================
//   SIM ROBOT ADAPTER (in-browser SimEngine)
// ============================================================

export class SimRobotAdapter extends BaseRobotAdapter {
  /**
   * @param {string} name       - SimBot name
   * @param {number} spawnX     - Spawn X position
   * @param {number} spawnY     - Spawn Y position
   * @param {number} spawnTheta - Spawn heading (rad)
   */
  constructor(name = 'SimBot', spawnX = 3.5, spawnY = 2.0, spawnTheta = Math.PI / 2) {
    super('sim');
    this.name = name;
    this.ip = 'sim://gazebotdtu';
    this.port = 0;
    this.connected = false;

    // SimEngine instance
    this.engine = new SimEngine();
    this.engine.reset(spawnX, spawnY, spawnTheta);

    // Telemetry polling timer
    this._telemTimer = null;
  }

  connect() {
    this.connected = true;
    this.engine.start();

    // Poll telemetry at 10Hz
    this._telemTimer = setInterval(() => {
      if (!this.engine.running) return;
      const telem = this.engine.telemetry;
      if (telem && this.onTelemetry) {
        this.onTelemetry(telem);
      }
    }, 100);

    if (this.onConnect) this.onConnect();
  }

  disconnect() {
    this.connected = false;
    this.engine.stop();
    if (this._telemTimer) {
      clearInterval(this._telemTimer);
      this._telemTimer = null;
    }
    if (this.onDisconnect) this.onDisconnect();
  }

  sendVelocity(linear, angular) { this.engine.setVelocity(linear, angular); }
  sendStop() { this.engine.setVelocity(0, 0); }
  navigate(path, finalHeading = null) { /* App navigation handles this */ }
  goto(x, y, finalHeading = null) { /* Sim mode uses app navigation */ }
  navStop() { this.engine.setVelocity(0, 0); }
  pause() { this.engine.setVelocity(0, 0); }
  resume() { /* no-op */ }

  resetOdometry() { this.engine.reset(); }
  setPose(x, y, theta) { this.engine.reset(x, y, theta); }
  recalibrateGyro() { /* no-op for sim */ }
  setBrake(enabled) { if (enabled) this.engine.setVelocity(0, 0); }
  setArchitectureProfile(profile) { /* no-op */ }
  sendMapData(grid) { /* no-op */ return true; }

  /** Get SimEngine info for UI */
  getSimInfo() { return this.engine.getSimInfo(); }

  /** Get world segments for 3D visualization */
  getWorldSegments() { return this.engine.getWorldSegments(); }

  /** Add obstacle to sim world */
  addObstacle(obsId, cx, cy, w, h) { this.engine.addObstacle(obsId, cx, cy, w, h); }

  /** Remove obstacle */
  removeObstacle(obsId) { this.engine.removeObstacle(obsId); }

  /** Set sim speed multiplier */
  setSpeed(factor) { this.engine.setSpeed(factor); }

  /** Toggle pause/resume sim physics */
  togglePause() {
    if (this.engine.running) {
      this.engine.stop();
    } else {
      this.engine.start();
    }
  }

  destroy() {
    this.disconnect();
  }
}

// ============================================================
//   FACTORY — Create adapter by type
// ============================================================

/**
 * Create a robot adapter.
 * @param {'real'|'sim'} type
 * @param {object} options
 * @returns {RealRobotAdapter|SimRobotAdapter}
 */
export function createRobotAdapter(type, options = {}) {
  if (type === 'sim') {
    return new SimRobotAdapter(
      options.name || 'SimBot',
      options.spawnX ?? 3.5,
      options.spawnY ?? 2.0,
      options.spawnTheta ?? Math.PI / 2,
    );
  }
  return new RealRobotAdapter(
    options.ip || '192.168.1.1',
    options.port ?? 81,
    options.name || 'Robot',
  );
}
