/**
 * GazeboTDTU — Simulation Web Worker
 * 
 * Chạy SimEngine trong background thread để không block UI.
 * Expose API qua Comlink cho Main Thread.
 * 
 * Main Thread gọi:
 *   simWorkerApi.start()
 *   simWorkerApi.setVelocity(v, w)
 *   simWorkerApi.getTelemetry()
 */

import * as Comlink from 'comlink';
import { SimEngine } from './simEngine.js';

class SimWorkerAPI {
  constructor() {
    this.engine = new SimEngine();
    this._telemetryCallback = null;
  }

  // ── Lifecycle ──────────────────────────────────────────

  start() {
    this.engine.start();
  }

  stop() {
    this.engine.stop();
  }

  reset(spawnX, spawnY, spawnTheta) {
    this.engine.reset(spawnX, spawnY, spawnTheta);
    return this.engine.telemetry;
  }

  setSpeed(factor) {
    this.engine.setSpeed(factor);
  }

  // ── Control ────────────────────────────────────────────

  setVelocity(linear, angular) {
    this.engine.setVelocity(linear, angular);
  }

  // ── Data Access ────────────────────────────────────────

  getTelemetry() {
    return this.engine.telemetry;
  }

  getPose() {
    return this.engine.getPose();
  }

  getSimInfo() {
    return this.engine.getSimInfo();
  }

  getWorldSegments() {
    return this.engine.getWorldSegments();
  }

  // ── World Edit ─────────────────────────────────────────

  addObstacle(id, cx, cy, w, h) {
    this.engine.addObstacle(id, cx, cy, w, h);
  }

  removeObstacle(id) {
    this.engine.removeObstacle(id);
  }

  // ── Telemetry Polling ──────────────────────────────────
  // Web Worker cannot use callbacks across thread boundary easily.
  // Instead, Main Thread polls getTelemetry() at 10Hz.

  /**
   * Start simulation and begin auto-polling telemetry.
   * Returns the first telemetry immediately.
   */
  startAndGetInitialTelemetry() {
    this.engine.reset();
    this.engine.start();
    return this.engine.telemetry;
  }
}

Comlink.expose(new SimWorkerAPI());
