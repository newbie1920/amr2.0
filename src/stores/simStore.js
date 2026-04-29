/**
 * AMR 2.0 — Sim Store (Zustand)
 * Quản lý GazeboTDTU Simulation: engines, obstacles, speed, pause.
 * Tách từ robotStore.js để giữ single-responsibility.
 */

import { create } from 'zustand';

const useSimStore = create((set, get) => ({
  // State
  simMode: false,
  simEngines: {},          // { robotId: SimEngine reference }
  simInfo: {},             // { robotId: { running, simTime, rtf, ... } }
  simWorldSegments: [],    // Cached world segments for 3D visualization

  // ============================================================
  //   ACTIONS
  // ============================================================

  /**
   * Register a SimEngine (called from robotStore.addRobot when type='sim')
   */
  registerSimEngine: (id, engine) => {
    set((s) => ({
      simMode: true,
      simEngines: { ...s.simEngines, [id]: engine },
      simWorldSegments: engine.getWorldSegments(),
    }));
  },

  /**
   * Unregister and cleanup a SimEngine
   */
  unregisterSimEngine: (id) => {
    const engine = get().simEngines[id];
    if (engine) engine.stop();

    set((s) => {
      const { [id]: _e, ...restEngines } = s.simEngines;
      const { [id]: _i, ...restInfo } = s.simInfo;
      return {
        simEngines: restEngines,
        simInfo: restInfo,
        simMode: Object.keys(restEngines).length > 0,
      };
    });
  },

  /**
   * Update sim info for a robot (called periodically from telemetry polling)
   */
  updateSimInfo: (id, info) => {
    set((s) => ({
      simInfo: { ...s.simInfo, [id]: info },
    }));
  },

  /**
   * Thêm vật cản vào thế giới mô phỏng
   */
  addSimObstacle: (obsId, cx, cy, w, h) => {
    const engines = get().simEngines;
    for (const engine of Object.values(engines)) {
      engine.addObstacle(obsId, cx, cy, w, h);
    }
    const first = Object.values(engines)[0];
    if (first) set({ simWorldSegments: first.getWorldSegments() });
  },

  /**
   * Xóa vật cản khỏi thế giới mô phỏng
   */
  removeSimObstacle: (obsId) => {
    const engines = get().simEngines;
    for (const engine of Object.values(engines)) {
      engine.removeObstacle(obsId);
    }
    const first = Object.values(engines)[0];
    if (first) set({ simWorldSegments: first.getWorldSegments() });
  },

  /**
   * Set tốc độ mô phỏng (1x, 2x, 5x)
   */
  setSimSpeed: (factor) => {
    for (const engine of Object.values(get().simEngines)) {
      engine.setSpeed(factor);
    }
  },

  /**
   * Pause/Resume tất cả sim engines
   */
  toggleSimPause: () => {
    const engines = get().simEngines;
    for (const engine of Object.values(engines)) {
      if (engine.running) {
        engine.stop();
      } else {
        engine.start();
      }
    }
    const infos = {};
    for (const [id, eng] of Object.entries(engines)) {
      infos[id] = eng.getSimInfo();
    }
    set({ simInfo: infos });
  },

  /**
   * Reset robot mô phỏng về vị trí spawn
   */
  resetSimRobot: (id, spawnX, spawnY, spawnTheta) => {
    const engine = get().simEngines[id];
    if (engine) {
      engine.reset(spawnX, spawnY, spawnTheta);
    }
  },
}));

export default useSimStore;
