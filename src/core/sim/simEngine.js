/**
 * GazeboTDTU — Simulation Engine (Physics Loop)
 * 
 * Game-loop chạy physics simulation:
 *   1. Nhận cmd_vel (v, w) → cập nhật vị trí robot (Differential Drive Kinematics)
 *   2. Chạy Lidar raycasting → tạo dữ liệu cảm biến giả lập
 *   3. Trả về telemetry giống hệt ESP32 format
 * 
 * Loop rate: 50Hz (20ms/step) — chạy trong Web Worker
 * Lidar rate: 10Hz (mỗi 5 step) — giống ESP32 thật
 * 
 * Kinematics: Differential Drive
 *   x'  = x + v * cos(θ) * dt
 *   y'  = y + v * sin(θ) * dt
 *   θ'  = θ + w * dt
 */

import { SimWorld } from './simWorld.js';
import { SimLidar } from './simLidar.js';
import { ROBOT_RADIUS } from '../warehouse.js';
import { normalizeAngle } from '../mathUtils.js';

// ============================================================
//   CONFIG
// ============================================================

const PHYSICS_HZ = 50;                // Physics update rate
const PHYSICS_DT = 1.0 / PHYSICS_HZ;  // 20ms
const LIDAR_EVERY_N_STEPS = 5;        // Lidar mỗi 5 steps = 10Hz

// Robot physical constraints
const MAX_LINEAR_VEL = 0.3;    // m/s
const MAX_ANGULAR_VEL = 2.0;   // rad/s
const LINEAR_ACCEL = 0.5;      // m/s² (acceleration limit)
const ANGULAR_ACCEL = 3.0;     // rad/s²

// Odometry noise (simulating encoder drift)
const ODOM_NOISE_LINEAR = 0.002;   // m per step
const ODOM_NOISE_ANGULAR = 0.005;  // rad per step

// ============================================================
//   SIM ENGINE CLASS
// ============================================================

export class SimEngine {
  constructor() {
    // World
    this.world = new SimWorld();
    this.lidar = new SimLidar({ numRays: 360, maxRange: 8000, noiseStdDev: 8 });

    // Robot state (ground truth)
    this.pose = { x: 5.0, y: 1.5, theta: Math.PI / 2 };

    // Robot odometry (with drift — what the robot THINKS its position is)
    this.odom = { x: 0, y: 0, theta: 0 };

    // Velocity state (actual, after acceleration limits)
    this.vel = { v: 0, w: 0 };

    // Target velocity (from cmd_vel)
    this.targetVel = { v: 0, w: 0 };

    // Simulation state
    this.running = false;
    this.stepCount = 0;
    this.simTime = 0;         // Seconds
    this.realTimeStart = 0;
    this.speedFactor = 1.0;   // 1x, 2x, 5x speed

    // Last Lidar scan result
    this.lastLidarScan = [];

    // Telemetry (output — giống ESP32 format)
    this.telemetry = this._buildTelemetry();

    // Event callbacks
    this.onTelemetry = null;   // (telemetry) => void
    this.onCollision = null;   // () => void

    // Timer handle
    this._intervalId = null;
  }

  // ──────────────────────────────────────────────────────────
  //   LIFECYCLE
  // ──────────────────────────────────────────────────────────

  /**
   * Start physics simulation loop
   */
  start() {
    if (this.running) return;
    this.running = true;
    this.realTimeStart = Date.now();
    this.stepCount = 0;
    this.simTime = 0;

    const intervalMs = (PHYSICS_DT * 1000) / this.speedFactor;
    this._intervalId = setInterval(() => this._step(), intervalMs);

    console.log(`[SimEngine] 🚀 Started at ${PHYSICS_HZ * this.speedFactor}Hz (${this.speedFactor}x speed)`);
  }

  /**
   * Stop simulation
   */
  stop() {
    this.running = false;
    if (this._intervalId) {
      clearInterval(this._intervalId);
      this._intervalId = null;
    }
    this.targetVel = { v: 0, w: 0 };
    this.vel = { v: 0, w: 0 };
    console.log('[SimEngine] ⏹ Stopped');
  }

  /**
   * Reset robot to spawn position
   */
  reset(spawnX, spawnY, spawnTheta) {
    const spawn = this.world.defaultSpawn;
    this.pose = {
      x: spawnX ?? spawn.x,
      y: spawnY ?? spawn.y,
      theta: spawnTheta ?? spawn.theta,
    };
    this.odom = { x: 0, y: 0, theta: 0 };
    this.vel = { v: 0, w: 0 };
    this.targetVel = { v: 0, w: 0 };
    this.stepCount = 0;
    this.simTime = 0;
    this.lastLidarScan = [];

    // Chạy Lidar scan ngay lập tức sau khi reset
    this.lastLidarScan = this.lidar.scan(
      this.pose.x, this.pose.y, this.pose.theta,
      this.world.segments
    );
    this.telemetry = this._buildTelemetry();

    console.log(`[SimEngine] 🔄 Reset to (${this.pose.x.toFixed(2)}, ${this.pose.y.toFixed(2)}, ${(this.pose.theta * 180 / Math.PI).toFixed(1)}°)`);
  }

  /**
   * Set simulation speed multiplier
   */
  setSpeed(factor) {
    this.speedFactor = Math.max(0.1, Math.min(10, factor));
    if (this.running) {
      // Restart with new interval
      clearInterval(this._intervalId);
      const intervalMs = (PHYSICS_DT * 1000) / this.speedFactor;
      this._intervalId = setInterval(() => this._step(), intervalMs);
    }
  }

  // ──────────────────────────────────────────────────────────
  //   CMD_VEL INPUT
  // ──────────────────────────────────────────────────────────

  /**
   * Set target velocity (from navigation stack or manual control)
   * @param {number} linear - m/s (forward positive)
   * @param {number} angular - rad/s (CCW positive)
   */
  setVelocity(linear, angular) {
    this.targetVel = {
      v: Math.max(-MAX_LINEAR_VEL, Math.min(MAX_LINEAR_VEL, linear)),
      w: Math.max(-MAX_ANGULAR_VEL, Math.min(MAX_ANGULAR_VEL, angular)),
    };
  }

  // ──────────────────────────────────────────────────────────
  //   PHYSICS STEP
  // ──────────────────────────────────────────────────────────

  _step() {
    const dt = PHYSICS_DT;
    this.stepCount++;
    this.simTime += dt;

    // 1. Acceleration limiting (trơn, không giật)
    this.vel.v = this._rampTo(this.vel.v, this.targetVel.v, LINEAR_ACCEL * dt);
    this.vel.w = this._rampTo(this.vel.w, this.targetVel.w, ANGULAR_ACCEL * dt);

    // 2. Differential Drive Kinematics
    const newTheta = this.pose.theta + this.vel.w * dt;
    const newX = this.pose.x + this.vel.v * Math.cos(this.pose.theta) * dt;
    const newY = this.pose.y + this.vel.v * Math.sin(this.pose.theta) * dt;

    // 3. Collision check
    const isCollided = this.world.checkCollision(newX, newY, ROBOT_RADIUS);
    if (!isCollided) {
      if (this.vel.v !== 0 && Math.random() < 0.1) {
        console.log(`[SimEngine] MOVING: v=${this.vel.v.toFixed(3)}, newX=${newX.toFixed(3)}, newY=${newY.toFixed(3)}`);
      }
      this.pose.x = newX;
      this.pose.y = newY;
      this.pose.theta = normalizeAngle(newTheta);
    } else {
      // Va chạm → dừng lại nhưng vẫn cho xoay
      if (this.vel.v !== 0 || this.vel.w !== 0) {
        console.warn(`[SimEngine] Collision at x=${newX.toFixed(3)}, y=${newY.toFixed(3)}! Stopping v.`);
      }
      this.vel.v = 0;
      this.pose.theta = normalizeAngle(newTheta);
      if (this.onCollision) this.onCollision();
    }

    // 4. Update odometry (with noise, simulating encoder drift)
    const odomNoiseV = (Math.random() - 0.5) * 2 * ODOM_NOISE_LINEAR;
    const odomNoiseW = (Math.random() - 0.5) * 2 * ODOM_NOISE_ANGULAR;
    this.odom.theta += (this.vel.w + odomNoiseW) * dt;
    this.odom.x += (this.vel.v + odomNoiseV) * Math.cos(this.odom.theta) * dt;
    this.odom.y += (this.vel.v + odomNoiseV) * Math.sin(this.odom.theta) * dt;

    // 5. Lidar scan (mỗi 5 steps = 10Hz)
    if (this.stepCount % LIDAR_EVERY_N_STEPS === 0) {
      this.lastLidarScan = this.lidar.scan(
        this.pose.x, this.pose.y, this.pose.theta,
        this.world.segments
      );

      // Build & broadcast telemetry
      this.telemetry = this._buildTelemetry();
      if (this.onTelemetry) {
        this.onTelemetry(this.telemetry);
      }
    }
  }

  /**
   * Ramp giá trị hiện tại về target với tốc độ tối đa maxStep
   */
  _rampTo(current, target, maxStep) {
    const diff = target - current;
    if (Math.abs(diff) <= maxStep) return target;
    return current + Math.sign(diff) * maxStep;
  }



  // ──────────────────────────────────────────────────────────
  //   TELEMETRY OUTPUT (giống ESP32 format)
  // ──────────────────────────────────────────────────────────

  _buildTelemetry() {
    const headingDeg = this.pose.theta * 180 / Math.PI;

    // Detect obstacle phía trước (giống firmware)
    let frontObstacle = false;
    for (const pt of this.lastLidarScan) {
      if (pt.a >= 340 || pt.a <= 20) { // ±20° phía trước
        if (pt.d < 250) { // < 25cm
          frontObstacle = true;
          break;
        }
      }
    }

    return {
      // Ground truth pose (dùng cho visualization)
      x: this.pose.x,
      y: this.pose.y,
      heading: headingDeg,
      headingRad: this.pose.theta,

      // Odometry (robot THINKS it is here — có drift)
      odomX: this.odom.x,
      odomY: this.odom.y,
      odomTheta: this.odom.theta,

      // Velocities
      linearVel: this.vel.v,
      angularVel: this.vel.w,

      // Lidar data (giống ESP32 format)
      lidar: this.lastLidarScan,

      // Status
      battery: 85, // Simulated battery
      obs: frontObstacle,

      // Sim metadata
      _sim: true,
      _simTime: this.simTime,
      _stepCount: this.stepCount,
      _realTimeFactor: this._computeRTF(),
    };
  }

  _computeRTF() {
    if (this.realTimeStart === 0 || this.simTime === 0) return 0;
    const realElapsed = (Date.now() - this.realTimeStart) / 1000;
    return this.simTime / realElapsed;
  }

  // ──────────────────────────────────────────────────────────
  //   WORLD API (pass-through)
  // ──────────────────────────────────────────────────────────

  addObstacle(id, cx, cy, w, h) {
    this.world.addBoxObstacle(id, cx, cy, w, h);
  }

  removeObstacle(id) {
    this.world.removeObstacle(id);
  }

  getWorldSegments() {
    return this.world.segments;
  }

  getPose() {
    return { ...this.pose };
  }

  getOdom() {
    return { ...this.odom };
  }

  getVelocity() {
    return { ...this.vel };
  }

  getSimInfo() {
    return {
      running: this.running,
      simTime: this.simTime,
      stepCount: this.stepCount,
      speedFactor: this.speedFactor,
      rtf: this._computeRTF(),
      pose: { ...this.pose },
      segments: this.world.segments.length,
      obstacles: this.world.dynamicObstacles.size,
    };
  }
}

export default SimEngine;
