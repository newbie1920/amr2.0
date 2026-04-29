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
import { DWA_DEFAULTS, DWA_PRESETS } from '../core/dwaPlanner.js';
import { normalizeAngle } from '../core/mathUtils.js';

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
//   MAP STORAGE HELPERS (localStorage)
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

const APP_NAV = {
  progressDistance: 0.06,
  progressTimeoutMs: 3500,
  goalTolerance: 0.18,
  finalHeadingTolerance: 0.12,
  rotateGain: 1.6,
  rotateMaxW: 0.9,
  recoverySpinMs: 1400,
  recoveryBackupMs: 1100,
  backupSpeed: -0.08,
  spinSpeed: 0.75,
  maxRecoveryAttempts: 4,
  waypointReachTolerance: 0.22,
};



function createAppNavigationSession(path, finalHeading = null) {
  const goal = path[path.length - 1];
  return {
    active: true,
    paused: false,
    status: 'TRACK',
    recoveryMode: null,
    recoveryUntil: 0,
    recoveryAttempts: 0,
    path,
    currentWaypointIndex: 0,
    finalHeadingRad: finalHeading == null ? null : (finalHeading * Math.PI) / 180.0,
    goal,
    lastProgressTime: Date.now(),
    lastProgressPose: null,
    replanRequested: false,
  };
}

function buildAppNavOverlay(session) {
  if (!session) return null;
  return {
    nav: session.status || 'IDLE',
    nav_wp: session.currentWaypointIndex || 0,
    nav_total: session.path?.length || 0,
  };
}

const useRobotStore = create((set, get) => ({
  // State
  robots: {},           // { robotId: { id, name, ip, port, connection, telemetry, status } }
  lidarScans: {},      // { robotId: [{a, d}] }
  occupancyGrid: {},   // { robotId: OccupancyGrid instance }
  mappingActive: {},   // { robotId: boolean }
  localizationActive: {}, // { robotId: boolean } - AMCL mode
  mapperInstances: {}, // { robotId: OccupancyGrid } — live mapping instances
  savedMaps: loadSavedMapsFromStorage(), // Persistent map list
  replanStatus: {},    // { robotId: 'idle' | 'replanning' | 'sent' }
  explorationInfo: {}, // { robotId: { phase, noFrontierCount, ... } }
  isMatching: {},      // { robotId: boolean } — Worker SLAM processing lock
  appNavigationSessions: {}, // { robotId: app-side navigation session }
  navComputationBusy: {}, // { robotId: boolean } — local planner worker lock
  velocityMuxes: {},   // { robotId: VelocityMux } — Twist Mux (articubot-style)
  // REP 105 TF Frames: map→odom transform (tích lũy correction từ scan matching)
  // mapPose = odomPose + mapToOdom  (odom smooth, map jumpy nhưng globally correct)
  mapToOdom: {},       // { robotId: { dx: 0, dy: 0, dTheta: 0 } }
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

  startAppNavigation: (id, path, finalHeading = null) => {
    const robot = get().robots[id];
    if (!robot?.connection?.connected || !path || path.length === 0) return false;

    setRobotArchitectureProfile(robot, 'pc_slam');
    const session = createAppNavigationSession(path, finalHeading);
    set((state) => ({
      appNavigationSessions: {
        ...state.appNavigationSessions,
        [id]: session,
      },
      navComputationBusy: {
        ...state.navComputationBusy,
        [id]: false,
      },
    }));
    return true;
  },

  stopAppNavigation: (id, reason = 'IDLE', restoreArchitecture = true) => {
    const state = get();
    const robot = state.robots[id];
    const mux = state.velocityMuxes[id];
    if (mux) {
      mux.release(VEL_SOURCE.NAVIGATION);
    }

    if (restoreArchitecture && !state.mappingActive[id] && !state.localizationActive?.[id]) {
      setRobotArchitectureProfile(robot, 'hybrid');
    }

    set((s) => ({
      appNavigationSessions: {
        ...s.appNavigationSessions,
        [id]: null,
      },
      navComputationBusy: {
        ...s.navComputationBusy,
        [id]: false,
      },
    }));

    return {
      nav: reason,
      nav_wp: 0,
      nav_total: 0,
    };
  },

  processNavigationTick: (id, telem, now = Date.now()) => {
    const state = get();
    const session = state.appNavigationSessions[id];
    if (!session?.active) return null;

    const mux = state.velocityMuxes[id];
    const grid = state.mapperInstances[id] || state.occupancyGrid[id];
    const pose = {
      x: telem.x ?? 0,
      y: telem.y ?? 0,
      theta: ((telem.heading ?? 0) * Math.PI) / 180.0,
    };
    const vel = {
      v: telem.linearVel ?? 0,
      w: telem.angularVel ?? 0,
    };

    const nextSession = { ...session };
    if (!nextSession.lastProgressPose) {
      nextSession.lastProgressPose = { x: pose.x, y: pose.y };
    }

    while (nextSession.currentWaypointIndex < nextSession.path.length - 1) {
      const wp = nextSession.path[nextSession.currentWaypointIndex];
      const d = Math.hypot(wp.x - pose.x, wp.y - pose.y);
      if (d <= APP_NAV.waypointReachTolerance) {
        nextSession.currentWaypointIndex += 1;
      } else {
        break;
      }
    }

    const moved = Math.hypot(
      pose.x - nextSession.lastProgressPose.x,
      pose.y - nextSession.lastProgressPose.y,
    );
    if (moved >= APP_NAV.progressDistance) {
      nextSession.lastProgressTime = now;
      nextSession.lastProgressPose = { x: pose.x, y: pose.y };
    }

    const goal = nextSession.goal;
    const distToGoal = Math.hypot(goal.x - pose.x, goal.y - pose.y);

    if (nextSession.paused) {
      if (mux) mux.release(VEL_SOURCE.NAVIGATION);
      nextSession.status = 'PAUSED';
      set((s) => ({
        appNavigationSessions: {
          ...s.appNavigationSessions,
          [id]: nextSession,
        },
      }));
      return buildAppNavOverlay(nextSession);
    }

    if (distToGoal <= APP_NAV.goalTolerance) {
      if (nextSession.finalHeadingRad != null) {
        const err = normalizeAngle(nextSession.finalHeadingRad - pose.theta);
        if (Math.abs(err) > APP_NAV.finalHeadingTolerance) {
          const cmdW = Math.max(-APP_NAV.rotateMaxW, Math.min(APP_NAV.rotateMaxW, err * APP_NAV.rotateGain));
          if (mux) mux.send(VEL_SOURCE.NAVIGATION, 0, cmdW);
          nextSession.status = 'F_TURN';
          set((s) => ({
            appNavigationSessions: {
              ...s.appNavigationSessions,
              [id]: nextSession,
            },
          }));
          return buildAppNavOverlay(nextSession);
        }
      }

      get().stopAppNavigation(id, 'DONE', true);
      return { nav: 'DONE', nav_wp: nextSession.path.length, nav_total: nextSession.path.length };
    }

    if (nextSession.recoveryMode) {
      if (now < nextSession.recoveryUntil) {
        if (mux) {
          if (nextSession.recoveryMode === 'RECOVERY_SPIN') {
            mux.send(VEL_SOURCE.NAVIGATION, 0, APP_NAV.spinSpeed);
          } else if (nextSession.recoveryMode === 'RECOVERY_BACKUP') {
            mux.send(VEL_SOURCE.NAVIGATION, APP_NAV.backupSpeed, 0);
          }
        }
        nextSession.status = nextSession.recoveryMode;
        set((s) => ({
          appNavigationSessions: {
            ...s.appNavigationSessions,
            [id]: nextSession,
          },
        }));
        return buildAppNavOverlay(nextSession);
      }

      if (nextSession.recoveryMode === 'REPLAN') {
        if (!grid || !navWorkerApi || state.navComputationBusy[id]) {
          nextSession.status = 'RECOVERY_REPLAN';
          set((s) => ({
            appNavigationSessions: {
              ...s.appNavigationSessions,
              [id]: nextSession,
            },
          }));
          return buildAppNavOverlay(nextSession);
        }

        set((s) => ({
          navComputationBusy: {
            ...s.navComputationBusy,
            [id]: true,
          },
        }));

        navWorkerApi.findPath(grid.serialize(), pose.x, pose.y, goal.x, goal.y, false, true)
          .then((result) => {
            const fresh = get().appNavigationSessions[id];
            if (!fresh?.active) return;

            if (result.success && result.path.length > 1) {
              const updated = {
                ...fresh,
                path: result.path,
                goal: result.path[result.path.length - 1],
                currentWaypointIndex: 0,
                recoveryMode: null,
                status: 'TRACK',
                lastProgressTime: Date.now(),
                lastProgressPose: { x: pose.x, y: pose.y },
              };
              set((s) => ({
                appNavigationSessions: {
                  ...s.appNavigationSessions,
                  [id]: updated,
                },
                navComputationBusy: {
                  ...s.navComputationBusy,
                  [id]: false,
                },
              }));
            } else {
              const retries = fresh.recoveryAttempts + 1;
              if (retries >= APP_NAV.maxRecoveryAttempts) {
                get().stopAppNavigation(id, 'ERROR', true);
                set((s) => ({
                  navComputationBusy: {
                    ...s.navComputationBusy,
                    [id]: false,
                  },
                }));
              } else {
                const updated = {
                  ...fresh,
                  recoveryAttempts: retries,
                  recoveryMode: 'RECOVERY_SPIN',
                  recoveryUntil: Date.now() + APP_NAV.recoverySpinMs,
                  status: 'RECOVERY_SPIN',
                };
                set((s) => ({
                  appNavigationSessions: {
                    ...s.appNavigationSessions,
                    [id]: updated,
                  },
                  navComputationBusy: {
                    ...s.navComputationBusy,
                    [id]: false,
                  },
                }));
              }
            }
          })
          .catch((err) => {
            console.error('[AppNav] Replan error:', err);
            get().stopAppNavigation(id, 'ERROR', true);
            set((s) => ({
              navComputationBusy: {
                ...s.navComputationBusy,
                [id]: false,
              },
            }));
          });

        nextSession.status = 'RECOVERY_REPLAN';
        set((s) => ({
          appNavigationSessions: {
            ...s.appNavigationSessions,
            [id]: nextSession,
          },
        }));
        return buildAppNavOverlay(nextSession);
      }

      nextSession.recoveryMode = 'REPLAN';
      nextSession.status = 'RECOVERY_REPLAN';
      
      // Nav2-style: Clear costmap around robot before replanning
      // (removes phantom obstacles that may be blocking the path)
      if (grid && typeof grid.clearCostmapAround === 'function') {
        grid.clearCostmapAround(pose.x, pose.y, 2.0);
        console.log(`[Recovery] Cleared costmap around robot (2m radius) before replan`);
      }
      
      set((s) => ({
        appNavigationSessions: {
          ...s.appNavigationSessions,
          [id]: nextSession,
        },
      }));
      return buildAppNavOverlay(nextSession);
    }

    if (now - nextSession.lastProgressTime > APP_NAV.progressTimeoutMs) {
      nextSession.recoveryAttempts += 1;
      if (nextSession.recoveryAttempts >= APP_NAV.maxRecoveryAttempts) {
        get().stopAppNavigation(id, 'ERROR', true);
        return { nav: 'ERROR', nav_wp: nextSession.currentWaypointIndex, nav_total: nextSession.path.length };
      }

      nextSession.recoveryMode = nextSession.recoveryAttempts % 2 === 1 ? 'RECOVERY_SPIN' : 'RECOVERY_BACKUP';
      nextSession.recoveryUntil = now + (nextSession.recoveryMode === 'RECOVERY_SPIN' ? APP_NAV.recoverySpinMs : APP_NAV.recoveryBackupMs);
      nextSession.status = nextSession.recoveryMode;
      set((s) => ({
        appNavigationSessions: {
          ...s.appNavigationSessions,
          [id]: nextSession,
        },
      }));
      return buildAppNavOverlay(nextSession);
    }

    if (!grid || !navWorkerApi) {
      const target = nextSession.path[Math.min(nextSession.currentWaypointIndex + 1, nextSession.path.length - 1)];
      const headingErr = normalizeAngle(Math.atan2(target.y - pose.y, target.x - pose.x) - pose.theta);
      const cmdV = Math.abs(headingErr) > 0.7 ? 0.0 : 0.12;
      const cmdW = Math.max(-APP_NAV.rotateMaxW, Math.min(APP_NAV.rotateMaxW, headingErr * 1.4));
      if (mux) mux.send(VEL_SOURCE.NAVIGATION, cmdV, cmdW);
      nextSession.status = 'TRACK';
      set((s) => ({
        appNavigationSessions: {
          ...s.appNavigationSessions,
          [id]: nextSession,
        },
      }));
      return buildAppNavOverlay(nextSession);
    }

    if (!state.navComputationBusy[id]) {
      set((s) => ({
        navComputationBusy: {
          ...s.navComputationBusy,
          [id]: true,
        },
      }));

      navWorkerApi.computeVelocity(pose, vel, nextSession.path, grid.serialize(), get().dwaConfig)
        .then((cmd) => {
          const fresh = get().appNavigationSessions[id];
          const freshMux = get().velocityMuxes[id];
          if (!fresh?.active || fresh.paused) {
            set((s) => ({
              navComputationBusy: {
                ...s.navComputationBusy,
                [id]: false,
              },
            }));
            return;
          }

          if (!cmd?.ok || (cmd.v === 0 && cmd.w === 0)) {
            const updated = {
              ...fresh,
              recoveryAttempts: fresh.recoveryAttempts + 1,
              recoveryMode: 'REPLAN',
              recoveryUntil: Date.now(),
              status: 'RECOVERY_REPLAN',
            };
            set((s) => ({
              appNavigationSessions: {
                ...s.appNavigationSessions,
                [id]: updated,
              },
              navComputationBusy: {
                ...s.navComputationBusy,
                [id]: false,
              },
            }));
            if (freshMux) freshMux.release(VEL_SOURCE.NAVIGATION);
            return;
          }

          if (freshMux) {
            freshMux.send(VEL_SOURCE.NAVIGATION, cmd.v, cmd.w);
          }

          set((s) => ({
            appNavigationSessions: {
              ...s.appNavigationSessions,
              [id]: {
                ...fresh,
                status: 'TRACK',
              },
            },
            navComputationBusy: {
              ...s.navComputationBusy,
              [id]: false,
            },
          }));
        })
        .catch((err) => {
          console.error('[AppNav] computeVelocity error:', err);
          get().stopAppNavigation(id, 'ERROR', true);
          set((s) => ({
            navComputationBusy: {
              ...s.navComputationBusy,
              [id]: false,
            },
          }));
        });
    }

    nextSession.status = 'TRACK';
    set((s) => ({
      appNavigationSessions: {
        ...s.appNavigationSessions,
        [id]: nextSession,
      },
    }));
    return buildAppNavOverlay(nextSession);
  },

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

      // === LIDAR MAPPING: Cập nhật Occupancy Grid khi đang quét ===
      const isMapping = currentState.mappingActive[id];
      const isLocalizing = currentState.localizationActive?.[id];

      if ((isMapping || isLocalizing) && currentState.mapperInstances[id]) {
        const grid = currentState.mapperInstances[id];
        const lidarPts = telem.lidar || [];
        if (lidarPts.length > 0) {
          const headingDeg = telem.heading ?? 0;

          // ── REP 105 TF Frames ──────────────────────────────
          //   map (global, jumpy) → odom (local, smooth) → base_link
          //
          //   odomPose = encoder/IMU pose (smooth, drifts over time)
          //   mapToOdom = accumulated SLAM correction (jumpy but globally correct)
          //   mapPose = odomPose + mapToOdom → used for grid update
          // ────────────────────────────────────────────────────

          // 1) odom frame: raw encoder/IMU pose (smooth)
          const odomX = telem.x ?? 0;
          const odomY = telem.y ?? 0;
          const odomTheta = headingDeg * Math.PI / 180;

          // 2) Apply accumulated map→odom transform
          const tf = currentState.mapToOdom[id] || { dx: 0, dy: 0, dTheta: 0 };
          let mapX = odomX + tf.dx;
          let mapY = odomY + tf.dy;
          let mapTheta = odomTheta + tf.dTheta;

          // 3) Scan Matching: tìm correction mới (nếu đủ data)
          if (navWorkerApi && (grid.scanCount >= 3 || isLocalizing) && !currentState.isMatching[id]) {
            // Lock worker to prevent overlapping calls
            set((s) => ({ isMatching: { ...s.isMatching, [id]: true } }));

            // Call Web Worker (Async)
            navWorkerApi.matchScan(id, grid.serialize(), mapX, mapY, mapTheta, lidarPts).then(matched => {
              const freshState = get();
              
              set((s) => ({
                robots: {
                  ...s.robots,
                  [id]: {
                    ...s.robots[id],
                    telemetry: {
                      ...s.robots[id].telemetry,
                      matchScore: matched.score
                    }
                  }
                }
              }));

              if (matched.corrected) {
                // Update map→odom transform (TÍCH LŨY correction)
                const c = matched.correction;
                const freshTf = freshState.mapToOdom[id] || { dx: 0, dy: 0, dTheta: 0 };
                const newTf = {
                  dx: freshTf.dx + c.dx,
                  dy: freshTf.dy + c.dy,
                  dTheta: freshTf.dTheta + c.dTheta,
                };

                set((s) => ({
                  mapToOdom: { ...s.mapToOdom, [id]: newTf },
                  isMatching: { ...s.isMatching, [id]: false }
                }));

                // Log per 10 matches (We can just log all successful ones or track matchCount in worker)
              } else {
                set((s) => ({ isMatching: { ...s.isMatching, [id]: false } }));
              }
            }).catch(e => {
              console.error("[SLAM] Worker Error:", e);
              set((s) => ({ isMatching: { ...s.isMatching, [id]: false } }));
            });
          }

          // 4) Update grid with MAP FRAME pose (globally correct) - CHỈ KHI MAPPING
          if (isMapping) {
            grid.updateFromScan(mapX, mapY, mapTheta, lidarPts);
          }

          // Debug log cho scan đầu tiên
          if (isMapping && grid.scanCount === 1) {
            console.log(`[Mapping] Scan #1: ${lidarPts.length} pts, odom=(${odomX.toFixed(2)}, ${odomY.toFixed(2)}), map=(${mapX.toFixed(2)}, ${mapY.toFixed(2)}), heading=${headingDeg.toFixed(1)}°`);
          }
          // Throttle occupancyGrid state update to 2Hz
          if (!currentState._lastGridUpdate || now - currentState._lastGridUpdate > 500) {
            currentState._lastGridUpdate = now;
            set((state) => ({
              occupancyGrid: {
                ...state.occupancyGrid,
                [id]: grid,
              },
            }));
          }
        }
      }

      const appNavOverlay = currentState.appNavigationSessions?.[id]?.active
        ? get().processNavigationTick(id, telem, now)
        : null;
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
      set((state) => {
        if (!state.robots[id]) return state;
        if (state.mappingActive?.[id] || state.localizationActive?.[id]) {
          return state;
        }
        return {
          occupancyGrid: {
            ...state.occupancyGrid,
            [id]: grid,
          },
        };
      });
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
      get().stopAppNavigation(id, 'IDLE', false);
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
      get().stopAppNavigation(id, 'IDLE', false);
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

  /**
   * Gửi lộ trình tự lái cho robot
   * @param {string} id - Robot ID
   * @param {Array<{x,y}>} path - Danh sách waypoint
   * @param {number|null} finalHeading - Góc cuối tại đích (độ)
   */
  navigateRobot: (id, path, finalHeading = null) => {
    const robot = get().robots[id];
    const state = get();
    if (robot && robot.connection.connected) {
      const hasGrid = !!(state.mapperInstances[id] || state.occupancyGrid[id]);
      const preferAppNav = robot.telemetry?.architecture === 'pc_slam' || hasGrid;
      if (preferAppNav) {
        return get().startAppNavigation(id, path, finalHeading);
      }
      robot.connection.navigate(path, finalHeading);
      return true;
    }
    return false;
  },

  /**
   * Dừng tự lái
   */
  navStopRobot: (id) => {
    const session = get().appNavigationSessions[id];
    if (session?.active) {
      get().stopAppNavigation(id, 'IDLE', true);
    }
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.navStop();
    }
  },

  pauseRobot: (id) => {
    const session = get().appNavigationSessions[id];
    if (session?.active) {
      set((state) => ({
        appNavigationSessions: {
          ...state.appNavigationSessions,
          [id]: { ...session, paused: true, status: 'PAUSED' },
        },
      }));
    }
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.pause();
    }
  },

  resumeRobot: (id) => {
    const session = get().appNavigationSessions[id];
    if (session?.active) {
      set((state) => ({
        appNavigationSessions: {
          ...state.appNavigationSessions,
          [id]: {
            ...session,
            paused: false,
            status: 'TRACK',
            lastProgressTime: Date.now(),
            recoveryMode: null,
          },
        },
      }));
    }
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.resume();
    }
  },

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
  //   LIDAR MAPPING ACTIONS
  // ============================================================

  /**
   * Bắt đầu quét Occupancy Grid cho robot
   * Tạo OccupancyGrid mới 80x80 (20x20m), centered tại vị trí robot hiện tại
   */
  startMapping: (id) => {
    const robot = get().robots[id];
    const telem = robot?.telemetry || {};
    const rx = telem.x ?? 0;
    const ry = telem.y ?? 0;
    setRobotArchitectureProfile(robot, 'pc_slam');
    
    // Grid centered tại robot → dynamic origin
    // Resolution 0.1m (articubot: 0.05m, cân bằng accuracy/performance cho browser)
    // 200x200 * 0.1m = 20x20m coverage (đủ cho kho xưởng)
    const grid = new OccupancyGrid(200, 200, 0.1, rx, ry);
    const matcher = new ScanMatcher(); // SLAM scan matching
    set((state) => ({
      mappingActive: { ...state.mappingActive, [id]: true },
      mapperInstances: { ...state.mapperInstances, [id]: grid },
      occupancyGrid: { ...state.occupancyGrid, [id]: grid },
      scanMatchers: { ...state.scanMatchers, [id]: matcher },
      mapToOdom: { ...state.mapToOdom, [id]: { dx: 0, dy: 0, dTheta: 0 } }, // Reset TF
    }));
    console.log(`[Mapping] Bắt đầu quét map cho robot ${id} tại (${rx.toFixed(2)}, ${ry.toFixed(2)}), grid 200x200 @0.1m + ScanMatcher ON`);
    
    // Bắt đầu tự khám phá
    startExploration(id, () => get());
    
    // Update exploration info mỗi giây cho UI
    const infoTimer = setInterval(() => {
      if (!get().mappingActive[id]) { clearInterval(infoTimer); return; }
      set((s) => ({ explorationInfo: { ...s.explorationInfo, [id]: getExplorationInfo() } }));
    }, 1000);
  },

  /**
   * Dừng quét + TỰ ĐỘNG lưu map vào danh sách
   */
  stopMapping: (id) => {
    // Dừng exploration trước
    stopExploration(id, () => get());
    
    const grid = get().mapperInstances[id];
    const robot = get().robots[id];
    setRobotArchitectureProfile(robot, 'hybrid');
    
    // Auto-save vào savedMaps nếu có data
    if (grid && grid.scanCount > 0) {
      const mapEntry = {
        id: `map_${Date.now()}`,
        name: `Map ${new Date().toLocaleDateString('vi-VN')} ${new Date().toLocaleTimeString('vi-VN', { hour: '2-digit', minute: '2-digit' })}`,
        robotId: id,
        createdAt: Date.now(),
        scanCount: grid.scanCount,
        width: grid.width,
        height: grid.height,
        resolution: grid.resolution,
        data: grid.exportJSON(),
      };
      const maps = [...get().savedMaps, mapEntry];
      set({ mappingActive: { ...get().mappingActive, [id]: false }, savedMaps: maps });
      saveMapsToStorage(maps);
      console.log(`[Mapping] Dừng quét + Tự động lưu "${mapEntry.name}" (${grid.scanCount} scans)`);
    } else {
      set((state) => ({
        mappingActive: { ...state.mappingActive, [id]: false },
      }));
      console.log(`[Mapping] Dừng quét (không có dữ liệu để lưu)`);
    }
  },

  /**
   * Chế độ AMCL: Chỉ định vị trên bản đồ đã có, không cập nhật grid
   */
  startLocalization: (id) => {
    const robot = get().robots[id];
    setRobotArchitectureProfile(robot, 'pc_slam');
    const matcher = new ScanMatcher();
    set((state) => ({
      localizationActive: { ...state.localizationActive, [id]: true },
      mappingActive: { ...state.mappingActive, [id]: false }, // Tắt mapping nếu đang bật
      scanMatchers: { ...state.scanMatchers, [id]: matcher },
      mapToOdom: { ...state.mapToOdom, [id]: { dx: 0, dy: 0, dTheta: 0 } }, // Reset TF
    }));
    console.log(`[Localization] Bắt đầu AMCL mode cho robot ${id}`);
  },

  stopLocalization: (id) => {
    const robot = get().robots[id];
    setRobotArchitectureProfile(robot, 'hybrid');
    set((state) => ({
      localizationActive: { ...state.localizationActive, [id]: false },
    }));
    console.log(`[Localization] Dừng AMCL mode cho robot ${id}`);
  },

  /**
   * Lưu map ra JSON + download file
   */
  saveMap: (id) => {
    const grid = get().mapperInstances[id];
    if (!grid) {
      console.warn('[Mapping] Không có map để lưu');
      return null;
    }

    const json = grid.exportJSON();
    const blob = new Blob([JSON.stringify(json, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `amr_map_${id}_${Date.now()}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    console.log(`[Mapping] Đã lưu map: ${grid.scanCount} scans, ${grid.width}x${grid.height}`);
    return json;
  },

  /**
   * Load map từ JSON (vào occupancyGrid đang active)
   */
  loadMap: (id, jsonData) => {
    try {
      const grid = OccupancyGrid.importJSON(jsonData);
      set((state) => ({
        mapperInstances: { ...state.mapperInstances, [id]: grid },
        occupancyGrid: { ...state.occupancyGrid, [id]: grid },
        mappingActive: { ...state.mappingActive, [id]: false },
      }));
      console.log(`[Mapping] Đã load map: ${grid.scanCount} scans`);
      return true;
    } catch (err) {
      console.error('[Mapping] Lỗi load map:', err);
      return false;
    }
  },

  // ============================================================
  //   MAP MANAGEMENT ACTIONS
  // ============================================================

  /** Lấy danh sách map đã lưu */
  getSavedMaps: () => get().savedMaps,

  /** Xóa map theo ID */
  deleteSavedMap: (mapId) => {
    const maps = get().savedMaps.filter(m => m.id !== mapId);
    set({ savedMaps: maps });
    saveMapsToStorage(maps);
    console.log(`[MapManager] Đã xóa map ${mapId}`);
  },

  /** Đổi tên map */
  renameSavedMap: (mapId, newName) => {
    const maps = get().savedMaps.map(m =>
      m.id === mapId ? { ...m, name: newName } : m
    );
    set({ savedMaps: maps });
    saveMapsToStorage(maps);
  },

  /** Load map từ danh sách đã lưu vào robot */
  loadSavedMap: (mapId, robotId) => {
    const mapEntry = get().savedMaps.find(m => m.id === mapId);
    if (!mapEntry) return false;
    return get().loadMap(robotId, mapEntry.data);
  },

  /** Export 1 map đã lưu ra file */
  exportSavedMap: (mapId) => {
    const mapEntry = get().savedMaps.find(m => m.id === mapId);
    if (!mapEntry) return;
    const blob = new Blob([JSON.stringify(mapEntry.data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${mapEntry.name.replace(/\s+/g, '_')}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  },

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

      // Transient update (cho Canvas/3D)
      const currentState = get();
      if (!currentState.transientRobots) currentState.transientRobots = {};
      currentState.transientRobots[id] = telem;

      // === LIDAR MAPPING: Giống robot thật ===
      const isMapping = currentState.mappingActive[id];
      const isLocalizing = currentState.localizationActive?.[id];
      const now = Date.now();

      if ((isMapping || isLocalizing) && currentState.mapperInstances[id]) {
        const grid = currentState.mapperInstances[id];
        const lidarPts = telem.lidar || [];
        if (lidarPts.length > 0) {
          const odomX = telem.x ?? 0;
          const odomY = telem.y ?? 0;
          const odomTheta = telem.headingRad ?? 0;

          const tf = currentState.mapToOdom[id] || { dx: 0, dy: 0, dTheta: 0 };
          let mapX = odomX + tf.dx;
          let mapY = odomY + tf.dy;
          let mapTheta = odomTheta + tf.dTheta;

          // Scan Matching
          if (navWorkerApi && (grid.scanCount >= 3 || isLocalizing) && !currentState.isMatching[id]) {
            set((s) => ({ isMatching: { ...s.isMatching, [id]: true } }));
            navWorkerApi.matchScan(id, grid.serialize(), mapX, mapY, mapTheta, lidarPts).then(matched => {
              const freshState = get();
              set((s) => ({
                robots: {
                  ...s.robots,
                  [id]: {
                    ...s.robots[id],
                    telemetry: {
                      ...s.robots[id]?.telemetry,
                      matchScore: matched.score
                    }
                  }
                }
              }));
              if (matched.corrected) {
                const c = matched.correction;
                const freshTf = freshState.mapToOdom[id] || { dx: 0, dy: 0, dTheta: 0 };
                set((s) => ({
                  mapToOdom: { ...s.mapToOdom, [id]: {
                    dx: freshTf.dx + c.dx,
                    dy: freshTf.dy + c.dy,
                    dTheta: freshTf.dTheta + c.dTheta,
                  }},
                  isMatching: { ...s.isMatching, [id]: false }
                }));
              } else {
                set((s) => ({ isMatching: { ...s.isMatching, [id]: false } }));
              }
            }).catch(e => {
              set((s) => ({ isMatching: { ...s.isMatching, [id]: false } }));
            });
          }

          // Update grid (chỉ khi mapping)
          if (isMapping) {
            grid.updateFromScan(mapX, mapY, mapTheta, lidarPts);
          }

          // Throttle occupancyGrid update to 2Hz
          if (!currentState._lastGridUpdate || now - currentState._lastGridUpdate > 500) {
            currentState._lastGridUpdate = now;
            set((s) => ({ occupancyGrid: { ...s.occupancyGrid, [id]: grid } }));
          }
        }
      }

      // App Navigation tick
      const appNavOverlay = currentState.appNavigationSessions?.[id]?.active
        ? get().processNavigationTick(id, telem, now)
        : null;

      // Throttle UI update to ~2Hz
      if (now - lastUIUpdate > 500) {
        lastUIUpdate = now;
        const effectiveTelem = appNavOverlay ? { ...telem, ...appNavOverlay } : telem;
        set((s) => {
          if (!s.robots[id]) return s;
          return {
            robots: {
              ...s.robots,
              [id]: { ...s.robots[id], telemetry: { ...effectiveTelem } },
            },
            simInfo: {
              ...s.simInfo,
              [id]: engine.getSimInfo(),
            },
          };
        });
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

  // ============================================================
  //   DWA TUNING ACTIONS (Phase 3)
  // ============================================================

  /**
   * Cập nhật một hoặc nhiều thông số DWA
   * @param {object} partial - { maxSpeedTrans: 0.5, clearanceBias: 15, ... }
   */
  setDWAConfig: (partial) => {
    set((s) => {
      const merged = { ...s.dwaConfig, ...partial };
      localStorage.setItem(DWA_CONFIG_KEY, JSON.stringify(merged));
      return { dwaConfig: merged, dwaActivePreset: 'custom' };
    });
  },

  /**
   * Reset DWA config về giá trị mặc định
   */
  resetDWAConfig: () => {
    localStorage.removeItem(DWA_CONFIG_KEY);
    set({ dwaConfig: { ...DWA_DEFAULTS }, dwaActivePreset: 'balanced' });
  },

  /**
   * Tải một preset DWA (cautious | balanced | aggressive | custom)
   */
  loadDWAPreset: (name) => {
    const state = get();
    const preset = DWA_PRESETS[name] || state.dwaCustomPresets[name];
    if (!preset) {
      console.warn(`[DWA] Unknown preset: ${name}`);
      return;
    }
    const config = { ...DWA_DEFAULTS, ...preset };
    localStorage.setItem(DWA_CONFIG_KEY, JSON.stringify(config));
    set({ dwaConfig: config, dwaActivePreset: name });
  },

  /**
   * Lưu config hiện tại thành custom preset
   */
  saveDWAPreset: (name) => {
    const state = get();
    const updated = { ...state.dwaCustomPresets, [name]: { ...state.dwaConfig } };
    localStorage.setItem(DWA_PRESETS_KEY, JSON.stringify(updated));
    set({ dwaCustomPresets: updated, dwaActivePreset: name });
  },

  /**
   * Xoá custom preset
   */
  deleteDWAPreset: (name) => {
    const state = get();
    const updated = { ...state.dwaCustomPresets };
    delete updated[name];
    localStorage.setItem(DWA_PRESETS_KEY, JSON.stringify(updated));
    set({ dwaCustomPresets: updated });
  },
}));

export default useRobotStore;
