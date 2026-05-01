/**
 * AMR 2.0 — Robot Store (Zustand)
 * Quản lý state cho danh sách robots + kết nối WebSocket + SIM mode (GazeboTDTU)
 */

import { create } from 'zustand';
import { RobotConnection } from '../core/robotProtocol.js';
import { OccupancyGrid } from '../core/lidarMapper.js';
import { ScanMatcher } from '../core/scanMatcher.js';
import { startExploration, stopExploration, getExplorationInfo } from '../core/exploration.js';
import { VelocityMux, VEL_SOURCE } from '../core/velocityMux.js';
import { navWorkerApi } from '../core/navWorkerSetup.js';
import { SimEngine } from '../core/sim/simEngine.js';
import { DWA_DEFAULTS } from '../core/dwaPlanner.js';
import useMapStore from './mapStore.js';
import useDWAStore from './dwaStore.js';
import useSimStore from './simStore.js';
import { registerStore, getNavStoreState } from './storeRegistry.js';

function saveRobotsToStorage(robots) {
  const data = Object.values(robots).map((r) => ({
    id: r.id,
    name: r.name,
    ip: r.ip,
    port: r.port,
  }));
  localStorage.setItem('amr_robots', JSON.stringify(data));
}

// ============================================================
//   STORAGE HELPERS (localStorage)
// ============================================================

const MAP_STORAGE_KEY = 'amr_saved_maps';
const DWA_CONFIG_KEY = 'amr_dwa_config';
const DWA_PRESETS_KEY = 'amr_dwa_presets';

function loadDWAConfigFromStorage() {
  try {
    const raw = localStorage.getItem(DWA_CONFIG_KEY);
    return raw ? { ...DWA_DEFAULTS, ...JSON.parse(raw) } : { ...DWA_DEFAULTS };
  } catch (e) {
    return { ...DWA_DEFAULTS };
  }
}

function loadCustomPresetsFromStorage() {
  try {
    const raw = localStorage.getItem(DWA_PRESETS_KEY);
    return raw ? JSON.parse(raw) : {};
  } catch (e) {
    return {};
  }
}

function loadSavedMapsFromStorage() {
  try {
    const raw = localStorage.getItem(MAP_STORAGE_KEY);
    return raw ? JSON.parse(raw) : [];
  } catch (e) {
    console.error('[MapStorage] Error loading maps:', e);
    return [];
  }
}

function saveMapsToStorage(maps) {
  try {
    localStorage.setItem(MAP_STORAGE_KEY, JSON.stringify(maps));
  } catch (e) {
    console.error('[MapStorage] Error saving maps:', e);
  }
}

function setRobotArchitectureProfile(robot, profile) {
  if (robot?.connection?.connected) {
    robot.connection.setArchitectureProfile(profile);
  }
}

// NOTE: useNavStore is accessed via storeRegistry to break circular dependency.
// navStore registers itself after creation, and robotStore accesses it at runtime.

const useRobotStore = create((set, get) => ({
  // State
  robots: {},           // { robotId: { id, name, ip, port, connection, telemetry, status } }
  lidarScans: {},      // { robotId: [{a, d}] }
  appNavigationSessions: {}, // { robotId: app-side navigation session }
  navComputationBusy: {}, // { robotId: boolean } — local planner worker lock
  velocityMuxes: {},   // { robotId: VelocityMux } — Twist Mux (articubot-style)
  selectedRobotId: null,

  // ── GazeboTDTU SIM MODE ──────────────────────────────────
  simMode: false,          // Global sim mode toggle
  simEngines: {},          // { robotId: SimEngine }
  simInfo: {},             // { robotId: { running, simTime, rtf, ... } }
  simWorldSegments: [],    // Cached world segments for 3D visualization

  // ── DWA TUNING (Phase 3) ──────────────────────────────────
  dwaConfig: loadDWAConfigFromStorage(),      // Live DWA parameters
  dwaActivePreset: 'balanced',                // Current preset name
  dwaCustomPresets: loadCustomPresetsFromStorage(), // User-saved presets

  // ============================================================
  //   ACTIONS
  // ============================================================

  loadStoredRobots: () => {
    const saved = JSON.parse(localStorage.getItem('amr_robots') || '[]');
    saved.forEach((r) => {
      if (!get().robots[r.id]) {
        get().addRobot(r.name, r.ip, r.port, r.id);
      }
    });
  },

  // ── Navigation actions migrated to navStore.js ──
  // startAppNavigation, stopAppNavigation, processNavigationTick
  // Lazy getter to break circular dependency (navStore imports robotStore)

  /**
   * Thêm robot mới
   */
  addRobot: (name, ip, port = 81, forcedId = null) => {
    const id = forcedId || `robot_${Date.now()}`;
    const connection = new RobotConnection(ip, port, name);
    // Gắn id robot để các callback có thể truy cập store
    connection.robotId = id;

    // === VELOCITY MUX (articubot twist_mux style) ===
    const mux = new VelocityMux();
    mux.onVelocityChanged = (linear, angular, source) => {
      if (connection.connected) {
        connection.sendVelocity(linear, angular);
      }
    };
    // Cleanup timeout sources mỗi 200ms
    const muxTimer = setInterval(() => mux.tick(), 200);
    // Lưu mux instance
    get().velocityMuxes[id] = mux;

    let lastUpdate = 0;
    // Lắng nghe telemetry
    connection.onTelemetry = (telem) => {
      const now = Date.now();
      
      // Transient state update (For Canvas/3D without rendering UI)
      const currentState = get();
      if (!currentState.transientRobots) currentState.transientRobots = {};
      currentState.transientRobots[id] = telem;

      // === LIDAR MAPPING: Delegate to mapStore ===
      try {
        useMapStore.getState().processLidarTick(id, telem, now, () => get());
      } catch(e) {
        // mapStore not ready
      }



      // Delegate nav tick to navStore
      let appNavOverlay = null;
      const navState = getNavStoreState();
      if (navState.appNavigationSessions?.[id]?.active) {
        appNavOverlay = navState.processNavigationTick(id, telem, now);
      }
      const effectiveTelem = appNavOverlay ? { ...telem, ...appNavOverlay } : telem;
      if (appNavOverlay) {
        telem.nav = effectiveTelem.nav;
        telem.nav_wp = effectiveTelem.nav_wp;
        telem.nav_total = effectiveTelem.nav_total;
      }
      
      // Throttle UI update to ~1Hz (1000ms) to rescue React re-render cycle
      if (now - lastUpdate > 1000) {
        lastUpdate = now;
        set((state) => {
          if (!state.robots[id]) return state;
          return {
            robots: {
              ...state.robots,
              [id]: {
                ...state.robots[id],
                telemetry: { ...effectiveTelem },
              },
            },
          };
        });
      }

      // === DYNAMIC REPLANNING: Detect obstacle stuck & auto-replan ===
      if (effectiveTelem.nav === 'PAUSED' && effectiveTelem.obs) {
        // Robot bị kẹt vật cản — track thời gian
        if (!currentState._obstaclePausedSince) {
          currentState._obstaclePausedSince = {};
        }
        if (!currentState._obstaclePausedSince[id]) {
          currentState._obstaclePausedSince[id] = now;
        }
        const stuckDuration = now - currentState._obstaclePausedSince[id];
        
        // Sau 3 giây kẹt → trigger auto-replan
        if (stuckDuration > 3000 && currentState.replanStatus[id] !== 'replanning') {
          console.log(`[Replan] Robot ${id} kẹt vật cản ${(stuckDuration/1000).toFixed(1)}s → Đang tìm đường mới...`);
          set((s) => ({ replanStatus: { ...s.replanStatus, [id]: 'replanning' } }));
          
          import('./taskStore.js').then(taskModule => {
            const taskStore = taskModule.default.getState();
            const activeTask = taskStore.tasks.find(t => t.status === 'in_progress' && t.assignedRobotId === id);
            
            if (activeTask && activeTask.goalX != null && activeTask.goalY != null) {
              const gridData = get().grid.serialize();
              navWorkerApi.findPath(gridData, effectiveTelem.x, effectiveTelem.y, activeTask.goalX, activeTask.goalY)
                .then(result => {
                  if (result.success && result.path.length > 1) {
                    console.log(`[Replan] Tìm được đường mới qua Grid: ${result.path.length} waypoints`);
                    const robot = get().robots[id];
                    if (robot && robot.connection.connected) {
                      robot.connection.navigate(result.path, activeTask.finalHeading || null);
                      set((s) => ({ replanStatus: { ...s.replanStatus, [id]: 'sent' } }));
                    }
                  } else {
                    console.warn('[Replan] Không tìm được đường mới qua NavWorker!');
                    set((s) => ({ replanStatus: { ...s.replanStatus, [id]: 'idle' } }));
                  }
                }).catch(e => {
                  console.error('[Replan] NavWorker lỗi:', e);
                  set((s) => ({ replanStatus: { ...s.replanStatus, [id]: 'idle' } }));
                });
            } else {
              set((s) => ({ replanStatus: { ...s.replanStatus, [id]: 'idle' } }));
            }
          });
        }
      } else {
        // Reset obstacle tracking khi không còn PAUSED+obs
        if (currentState._obstaclePausedSince && currentState._obstaclePausedSince[id]) {
          delete currentState._obstaclePausedSince[id];
          if (currentState.replanStatus[id] !== 'idle') {
            set((s) => ({ replanStatus: { ...s.replanStatus, [id]: 'idle' } }));
          }
        }
      }

      // Sync Navigation Status with Task System (replaces older setInterval in UI)
      if (effectiveTelem.nav === 'ERROR' || effectiveTelem.nav === 'DONE' || effectiveTelem.nav === 'PAUSED') {
         // Dynamically import to avoid circular dependency issues at boot
         import('./taskStore.js').then(module => {
            const useTaskStore = module.default;
            const state = useTaskStore.getState();
            const activeTasks = state.tasks.filter(t => t.status === 'in_progress' && t.assignedRobotId === id && !t.dbUpdated);
            
            if (activeTasks.length > 0) {
              const taskId = activeTasks[0].id;
              if (effectiveTelem.nav === 'PAUSED') {
                 state.pauseTask(taskId, 'Người dùng tạm dừng hoặc Kẹt vật cản');
              } else {
                 state.processTaskCompletion(taskId, telem.nav, 'Robot bị lỗi điều hướng (Kẹt hoặc Quá thời gian)');
              }
            }
         });
      }
    };

    // Handle LIDAR occupancy grid updates
    connection.onLidarGrid = (grid) => {
      try {
        useMapStore.getState().updateOccupancyGrid(id, grid);
      } catch(e) {}
    };

    connection.onConnect = () => {
      set((state) => {
        if (!state.robots[id]) return state;
        return {
          robots: {
            ...state.robots,
            [id]: { ...state.robots[id], status: 'connected' },
          },
        }
      });
      // Tự động load map nếu có
      useMapStore.getState().autoLoadLatestMap(id);
    };

    connection.onDisconnect = () => {
      set((state) => {
        if (!state.robots[id]) return state;
        return {
          robots: {
            ...state.robots,
            [id]: { ...state.robots[id], status: 'disconnected' },
          },
        }
      });
    };

    set((state) => {
      const newRobots = {
        ...state.robots,
        [id]: {
          id,
          name,
          ip,
          port,
          connection,
          status: 'disconnected',
          telemetry: {
            x: 0, y: 0, heading: 0, headingRad: 0,
            distance: 0, linearVel: 0, battery: 100,
            imuAvailable: false, imuCalibrated: false,
          },
          currentTask: null,
          taskStatus: 'idle', // idle | working | charging
        },
      };
      saveRobotsToStorage(newRobots);
      return { robots: newRobots };
    });

    return id;
  },

  /**
   * Xóa robot
   */
  removeRobot: (id) => {
    const robot = get().robots[id];
    if (robot) {
      try { getNavStoreState().stopAppNavigation(id, 'IDLE', false); } catch(e) {}
      robot.connection.disconnect();
      set((state) => {
        const { [id]: removed, ...rest } = state.robots;
        saveRobotsToStorage(rest);
        return { robots: rest, selectedRobotId: state.selectedRobotId === id ? null : state.selectedRobotId };
      });
    }
  },

  /**
   * Kết nối robot
   */
  connectRobot: (id) => {
    const robot = get().robots[id];
    if (robot) {
      robot.connection.connect();
      set((state) => ({
        robots: {
          ...state.robots,
          [id]: { ...state.robots[id], status: 'connecting' },
        },
      }));
    }
  },

  /**
   * Ngắt kết nối robot
   */
  disconnectRobot: (id) => {
    const robot = get().robots[id];
    if (robot) {
      try { getNavStoreState().stopAppNavigation(id, 'IDLE', false); } catch(e) {}
      robot.connection.disconnect();
      set((state) => ({
        robots: {
          ...state.robots,
          [id]: { ...state.robots[id], status: 'disconnected' },
        },
      }));
    }
  },

  /**
   * Chọn robot
   */
  selectRobot: (id) => set({ selectedRobotId: id }),

  /**
   * Gửi lệnh điều khiển tay
   */
  sendManualControl: (id, linear, angular) => {
    const mux = get().velocityMuxes[id];
    if (mux) {
      // Route qua VelocityMux — manual có priority 50 > navigation 10
      if (linear === 0 && angular === 0) {
        mux.release(VEL_SOURCE.MANUAL);
      } else {
        mux.send(VEL_SOURCE.MANUAL, linear, angular);
      }
    } else {
      // Fallback: gửi trực tiếp nếu chưa có mux
      const robot = get().robots[id];
      if (robot && robot.connection.connected) {
        robot.connection.sendVelocity(linear, angular);
      }
    }
  },

  // Cập nhật dữ liệu LIDAR cho robot
  updateLidar: (id, scan) => {
    set((state) => {
      const newScans = { ...state.lidarScans, [id]: scan };
      // Cập nhật telemetry.lidar để UI có thể dùng
      const robot = state.robots[id];
      if (robot) {
        robot.telemetry = { ...robot.telemetry, lidar: scan };
      }
      return { lidarScans: newScans, robots: { ...state.robots } };
    });
  },

  /**
   * Dừng robot
   */
  stopRobot: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.sendStop();
    }
  },

  /**
   * Reset odometry
   */
  resetOdometry: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.resetOdometry();
    }
  },

  /**
   * Set specific pose
   */
  setPose: (id, x, y, theta) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.setPose(x, y, theta);
    }
  },

  // ── Nav control delegated to navStore ──
  navigateRobot: (id, path, fh) => { try { return getNavStoreState().navigateRobot(id, path, fh); } catch(e) { return false; } },
  navStopRobot: (id) => { try { getNavStoreState().navStopRobot(id); } catch(e) {} },
  pauseRobot: (id) => { try { getNavStoreState().pauseNav(id); } catch(e) {} },
  resumeRobot: (id) => { try { getNavStoreState().resumeNav(id); } catch(e) {} },


  /**
   * Recalibrate con quay hồi chuyển (robot phải đứng yên!)
   */
  recalibrateGyro: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.recalibrateGyro();
    }
  },

  /**
   * Bật/tắt chế độ khóa phanh khẩn cấp
   */
  setBrake: (id, enabled) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.setBrake(enabled);
    }
  },

  // ============================================================
  //   LIDAR MAPPING ACTIONS (Delegated to mapStore)
  // ============================================================
  // Note: These actions are now managed entirely by mapStore.js
  // We keep stubs for backwards compatibility if needed, 
  // but components should use useMapStore directly.

  /** Import map từ JSON file vào danh sách */
  importMapFromJSON: (jsonData, name = null) => {
    try {
      // Validate
      if (!jsonData || (jsonData.version !== 1 && jsonData.version !== 2)) {
        throw new Error('Invalid map format');
      }
      
      const mapEntry = {
        id: `map_${Date.now()}`,
        name: name || `Imported ${new Date().toLocaleDateString('vi-VN')}`,
        robotId: 'imported',
        createdAt: jsonData.timestamp || Date.now(),
        scanCount: jsonData.scanCount || 0,
        width: jsonData.width,
        height: jsonData.height,
        resolution: jsonData.resolution,
        data: jsonData,
      };
      const maps = [...get().savedMaps, mapEntry];
      set({ savedMaps: maps });
      saveMapsToStorage(maps);
      console.log(`[MapManager] Imported map: ${mapEntry.name}`);
      return true;
    } catch (err) {
      console.error('[MapManager] Import error:', err);
      return false;
    }
  },

  // Getters
  getRobot: (id) => get().robots[id],
  getConnectedRobots: () => Object.values(get().robots).filter(r => r.status === 'connected'),
  getRobotList: () => Object.values(get().robots),

  // ============================================================
  //   GAZEBO TDTU — SIM MODE ACTIONS
  // ============================================================

  /**
   * Thêm robot mô phỏng (không cần IP/WebSocket)
   * Robot sẽ chạy hoàn toàn bằng SimEngine trong browser
   */
  addSimRobot: (name = 'SimBot', spawnX = 5.0, spawnY = 1.5, spawnTheta = Math.PI / 2) => {
    const id = `sim_${Date.now()}`;
    const engine = new SimEngine();
    engine.reset(spawnX, spawnY, spawnTheta);

    // Velocity Mux (giống robot thật)
    const mux = new VelocityMux();
    mux.onVelocityChanged = (linear, angular, source) => {
      // Route velocity vào SimEngine thay vì WebSocket
      if (linear !== 0 || angular !== 0) {
        console.log(`[SimMux] v=${linear.toFixed(3)} w=${angular.toFixed(3)} src=${source} running=${engine.running}`);
      }
      engine.setVelocity(linear, angular);
    };
    const muxTimer = setInterval(() => mux.tick(), 200);
    get().velocityMuxes[id] = mux;

    // Polling telemetry từ SimEngine mỗi 100ms (10Hz)
    let lastUIUpdate = 0;
    const telemTimer = setInterval(() => {
      if (!engine.running) return;
      const telem = engine.telemetry;
      if (!telem) return;
      
      const now = Date.now();

      // Transient update (cho Canvas/3D)
      const currentState = get();
      if (!currentState.transientRobots) currentState.transientRobots = {};
      currentState.transientRobots[id] = telem;

      // === LIDAR MAPPING: Delegate to mapStore ===
      try {
        useMapStore.getState().processLidarTick(id, telem, now, () => get());
      } catch(e) {}

      // Delegate nav tick to navStore
      let appNavOverlay = null;
      const navState = getNavStoreState();
      if (navState.appNavigationSessions?.[id]?.active) {
        appNavOverlay = navState.processNavigationTick(id, telem, now);
      }

      let effectiveTelem = appNavOverlay ? { ...telem, ...appNavOverlay } : telem;
      if (appNavOverlay) {
        telem.nav = effectiveTelem.nav;
        telem.nav_wp = effectiveTelem.nav_wp;
        telem.nav_total = effectiveTelem.nav_total;
      } else if (telem.nav && telem.nav !== 'IDLE' && !navState.appNavigationSessions?.[id]?.active) {
        // Session đã kết thúc nhưng telem vẫn giữ status cũ → reset về IDLE
        telem.nav = 'IDLE';
        telem.nav_wp = 0;
        telem.nav_total = 0;
        effectiveTelem.nav = 'IDLE';
      }

      // Sync Navigation Status with Task System (SimBot — mirror of real robot logic)
      if (effectiveTelem.nav === 'ERROR' || effectiveTelem.nav === 'DONE' || effectiveTelem.nav === 'PAUSED') {
        import('./taskStore.js').then(module => {
          const useTaskStore = module.default;
          const taskState = useTaskStore.getState();
          const activeTasks = taskState.tasks.filter(t => t.status === 'in_progress' && t.assignedRobotId === id && !t.dbUpdated);
          
          if (activeTasks.length > 0) {
            const taskId = activeTasks[0].id;
            if (effectiveTelem.nav === 'PAUSED') {
              taskState.pauseTask(taskId, 'Robot tạm dừng hoặc kẹt vật cản');
            } else {
              taskState.processTaskCompletion(taskId, effectiveTelem.nav, 'Robot hoàn thành hoặc lỗi điều hướng');
            }
          }
        });
      }

      // Throttle UI update to ~2Hz
      if (now - lastUIUpdate > 500) {
        lastUIUpdate = now;
        const info = engine.getSimInfo();
        set((s) => {
          if (!s.robots[id]) return s;
          return {
            robots: {
              ...s.robots,
              [id]: { ...s.robots[id], telemetry: { ...effectiveTelem } },
            },
            simInfo: {
              ...s.simInfo,
              [id]: info,
            },
          };
        });
        // Sync to simStore for SimControlPanel
        useSimStore.getState().updateSimInfo(id, info);
      }
    }, 100);

    // Robot entry (giả lập — không có connection thật)
    const simRobot = {
      id,
      name,
      ip: 'sim://gazebotdtu',
      port: 0,
      connection: {
        connected: true,
        sendVelocity: (v, w) => engine.setVelocity(v, w),
        sendStop: () => engine.setVelocity(0, 0),
        navStop: () => engine.setVelocity(0, 0),
        pause: () => engine.setVelocity(0, 0),
        resume: () => {},
        disconnect: () => engine.stop(),
        resetOdometry: () => engine.reset(),
        setPose: (x, y, theta) => engine.reset(x, y, theta),
        setArchitectureProfile: () => {},
        recalibrateGyro: () => {},
        setBrake: () => {},
        navigate: () => {},
      },
      status: 'connected',
      telemetry: engine.telemetry,
      currentTask: null,
      taskStatus: 'idle',
      _sim: true,
      _simEngine: engine,
      _timers: [muxTimer, telemTimer],
    };

    set((s) => ({
      robots: { ...s.robots, [id]: simRobot },
      simMode: true,
      simEngines: { ...s.simEngines, [id]: engine },
      simWorldSegments: engine.getWorldSegments(),
      selectedRobotId: id,
    }));

    // Start physics simulation
    engine.start();

    // Tạo bản đồ kho hàng tự động từ world segments (không cần LiDAR quét)
    const segments = engine.getWorldSegments();
    const mapLoaded = useMapStore.getState().generateSimMap(id, segments);
    if (!mapLoaded) {
      // Fallback: thử load map đã lưu nếu không tạo được từ segments
      useMapStore.getState().autoLoadLatestMap(id);
    }

    // CRITICAL: Register engine in simStore so SimControlPanel can control Play/Pause.
    // Without this, toggleSimPause operates on an empty simStore.simEngines → robot stays PAUSED.
    useSimStore.getState().registerSimEngine(id, engine);

    // AUTO-CREATE MAP: Initialize OccupancyGrid immediately so navigation
    // always has a grid available for pathfinding and DWA obstacle avoidance.
    // Without this, navigateToGoal falls back to straight-line waypoints.
    // NOTE: We don't call startMapping() because that also starts auto-exploration
    // which would conflict with user's manual navigation. We only create the grid
    // and set mappingActive so processLidarTick can populate it.
    try {
      const mapState = useMapStore.getState();
      const grid = new OccupancyGrid(200, 200, 0.1, spawnX, spawnY);
      mapState.updateOccupancyGrid(id, grid);
      // Set mappingActive and mapperInstances directly via setState
      useMapStore.setState((s) => ({
        mappingActive: { ...s.mappingActive, [id]: true },
        mapperInstances: { ...s.mapperInstances, [id]: grid },
        occupancyGrid: { ...s.occupancyGrid, [id]: grid },
        mapToOdom: { ...s.mapToOdom, [id]: { dx: 0, dy: 0, dTheta: 0 } },
      }));
      console.log(`[GazeboTDTU] 🗺️ Auto-created map for SimBot "${name}" — passive mapping enabled`);
    } catch (e) {
      console.warn('[GazeboTDTU] Failed to auto-create map:', e);
    }

    console.log(`[GazeboTDTU] 🤖 SimBot "${name}" spawned at (${spawnX}, ${spawnY}) — Physics running at 50Hz`);
    return id;
  },

  /**
   * Xóa robot mô phỏng
   */
  removeSimRobot: (id) => {
    const engine = get().simEngines[id];
    const robot = get().robots[id];
    if (engine) engine.stop();
    if (robot?._timers) robot._timers.forEach(t => clearInterval(t));

    // Unregister from simStore
    useSimStore.getState().unregisterSimEngine(id);

    set((s) => {
      const { [id]: _e, ...restEngines } = s.simEngines;
      const { [id]: _r, ...restRobots } = s.robots;
      const { [id]: _i, ...restInfo } = s.simInfo;
      const hasSim = Object.keys(restEngines).length > 0;
      return {
        robots: restRobots,
        simEngines: restEngines,
        simInfo: restInfo,
        simMode: hasSim,
        selectedRobotId: s.selectedRobotId === id ? null : s.selectedRobotId,
      };
    });
    console.log(`[GazeboTDTU] 🗑️ SimBot ${id} removed`);
  },

  /**
   * Thêm vật cản vào thế giới mô phỏng
   */
  addSimObstacle: (obsId, cx, cy, w, h) => {
    const engines = get().simEngines;
    for (const engine of Object.values(engines)) {
      engine.addObstacle(obsId, cx, cy, w, h);
    }
    // Refresh cached segments
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
    // Update simInfo
    const first = Object.values(engines)[0];
    if (first) {
      const infos = {};
      for (const [id, eng] of Object.entries(engines)) {
        infos[id] = eng.getSimInfo();
      }
      set({ simInfo: infos });
    }
  },

  /**
   * Reset robot mô phỏng về vị trí spawn
   */
  resetSimRobot: (id, spawnX, spawnY, spawnTheta) => {
    const engine = get().simEngines[id];
    if (engine) {
      engine.reset(spawnX, spawnY, spawnTheta);
      set((s) => ({
        robots: {
          ...s.robots,
          [id]: { ...s.robots[id], telemetry: engine.telemetry },
        },
      }));
    }
  },

  // ── DWA actions delegated to dwaStore.js ──
  setDWAConfig: (p) => { try { useDWAStore.getState().setDWAConfig(p); } catch(e) {} },
  resetDWAConfig: () => { try { useDWAStore.getState().resetDWAConfig(); } catch(e) {} },
  loadDWAPreset: (n) => { try { useDWAStore.getState().loadDWAPreset(n); } catch(e) {} },
  saveDWAPreset: (n) => { try { useDWAStore.getState().saveDWAPreset(n); } catch(e) {} },
  deleteDWAPreset: (n) => { try { useDWAStore.getState().deleteDWAPreset(n); } catch(e) {} },
}));

// Register in store registry for cross-store access (navStore accesses this via registry)
registerStore('robotStore', useRobotStore);

export default useRobotStore;

// ============================================================
//   RE-EXPORTS — Domain Stores (Phase 3 migration bridge)
//   Components can import these directly for cleaner code:
//     import useDWAStore from '../stores/dwaStore';
//     import useMapStore from '../stores/mapStore';
//     import useSimStore from '../stores/simStore';
//     import useNavStore from '../stores/navStore';
// ============================================================
export { default as useDWAStore } from './dwaStore.js';
export { default as useMapStore } from './mapStore.js';
export { default as useSimStore } from './simStore.js';
export { default as useNavStore } from './navStore.js';
