/**
 * AMR 2.0 — Nav Store (Zustand)
 * Quản lý Navigation Sessions, Recovery Behaviors, Path Following.
 * Tách từ robotStore.js để giữ single-responsibility.
 *
 * Cross-store access:
 *   - robotStore: robots, velocityMuxes, adapter
 *   - mapStore: mapperInstances, occupancyGrid
 *   - dwaStore: dwaConfig
 *
 * Pattern: Nav2-style — Global Planner (A*) + Local Planner (DWA) + Recovery
 */

import { create } from 'zustand';
import { VEL_SOURCE } from '../core/velocityMux.js';
import { normalizeAngle } from '../core/mathUtils.js';
import { navWorkerApi } from '../core/navWorkerSetup.js';
import { registerStore, getRobotStoreState, getMapStoreState, getDWAStoreState } from './storeRegistry.js';

// NOTE: useRobotStore is NOT imported at top-level to avoid circular dependency.
// robotStore imports navStore, so navStore cannot import robotStore at evaluation time.
// Instead, we access robotStore via the store registry at runtime.
import useMapStore from './mapStore.js';
import useDWAStore from './dwaStore.js';

const getRobotStore = () => getRobotStoreState();
const getMapStore = () => useMapStore.getState();
const getDWAStore = () => useDWAStore.getState();

// ============================================================
//   NAV CONFIG (Nav2-style parameters)
// ============================================================

const APP_NAV = {
  progressDistance: 0.03,      // Reduced: sim moves slow (was 0.06)
  progressTimeoutMs: 8000,     // Increased: sim RTF is low (was 3500)
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

// ============================================================
//   HELPERS
// ============================================================

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

function setRobotArchitectureProfile(robot, profile) {
  // Support both adapter and legacy connection
  if (robot?.adapter?.connected) {
    robot.adapter.setArchitectureProfile(profile);
  } else if (robot?.connection?.connected) {
    robot.connection.setArchitectureProfile(profile);
  }
}

// Lazy getters replaced by top-level imports

// ============================================================
//   STORE
// ============================================================

const useNavStore = create((set, get) => ({
  // State
  appNavigationSessions: {}, // { robotId: session }
  navComputationBusy: {},    // { robotId: boolean } — local planner worker lock
  replanStatus: {},          // { robotId: 'idle' | 'replanning' | 'sent' }
  dwaTrajectories: {},       // { robotId: [{x,y},...] } — DWA chosen trajectory for viz
  robotTrails: {},           // { robotId: [{x,y},...] } — breadcrumb trail history

  // ============================================================
  //   ACTIONS
  // ============================================================

  /**
   * Start app-side navigation session (Nav2-style path following)
   */
  startAppNavigation: (id, path, finalHeading = null) => {
    const robotState = getRobotStore();
    const robot = robotState.robots[id];
    if (!robot || path?.length === 0) {
      console.warn('[AppNav] startAppNavigation REJECTED: no robot or empty path', { id, hasRobot: !!robot, pathLen: path?.length });
      return false;
    }

    // Check connection via adapter or legacy
    const isConnected = robot.adapter?.connected ?? robot.connection?.connected;
    if (!isConnected) {
      console.warn('[AppNav] startAppNavigation REJECTED: not connected', { id, adapter: robot.adapter?.connected, connection: robot.connection?.connected });
      return false;
    }

    setRobotArchitectureProfile(robot, 'pc_slam');
    const session = createAppNavigationSession(path, finalHeading);
    console.log('[AppNav] ✅ startAppNavigation SUCCESS', { id, pathLen: path.length, goal: session.goal, active: session.active });
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

  /**
   * Stop app-side navigation
   */
  stopAppNavigation: (id, reason = 'IDLE', restoreArchitecture = true) => {
    console.log(`[AppNav] ⏹ stopAppNavigation: ${reason}`);
    const robotState = getRobotStore();
    const robot = robotState.robots[id];
    const mux = robotState.velocityMuxes[id];
    if (mux) {
      mux.release(VEL_SOURCE.NAVIGATION);
    }

    const mapState = getMapStore();
    if (restoreArchitecture && !mapState.mappingActive[id] && !mapState.localizationActive?.[id]) {
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

  /**
   * Process one navigation tick (called from telemetry callback)
   */
  processNavigationTick: (id, telem, now = Date.now()) => {
    const state = get();
    const session = state.appNavigationSessions[id];
    if (!session?.active) return null;

    const robotState = getRobotStore();
    const mapState = getMapStore();
    const mux = robotState.velocityMuxes[id];
    const grid = mapState.mapperInstances[id] || mapState.occupancyGrid[id];
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
      nextSession.lastProgressTime = now;
    }

    // Cập nhật currentPose (dùng cho debug vẽ footprint)
    nextSession.currentPose = pose;

    // --- Xử lý STUCK & RECOVERY ---
    const moved = Math.hypot(
      pose.x - nextSession.lastProgressPose.x,
      pose.y - nextSession.lastProgressPose.y,
    );
    
    // DEBUG LOG HERE
    if (Math.random() < 0.1) {
      console.log(`[AppNav] Pose: x=${pose.x.toFixed(3)}, y=${pose.y.toFixed(3)}, moved=${moved.toFixed(3)}`);
    }

    if (moved >= APP_NAV.progressDistance) {
      nextSession.lastProgressTime = now;
      nextSession.lastProgressPose = { x: pose.x, y: pose.y };
      // Only reset recovery counter on PROVEN movement progress
      if (nextSession.recoveryAttempts > 0) {
        nextSession.recoveryAttempts = Math.max(0, nextSession.recoveryAttempts - 1);
      }
    }

    // Advance waypoint index
    while (nextSession.currentWaypointIndex < nextSession.path.length - 1) {
      const wp = nextSession.path[nextSession.currentWaypointIndex];
      const d = Math.hypot(wp.x - pose.x, wp.y - pose.y);
      if (d <= APP_NAV.waypointReachTolerance) {
        nextSession.currentWaypointIndex += 1;
      } else {
        break;
      }
    }

    const goal = nextSession.goal;
    const distToGoal = Math.hypot(goal.x - pose.x, goal.y - pose.y);

    // ── PAUSED ──
    if (nextSession.paused) {
      if (mux) mux.release(VEL_SOURCE.NAVIGATION);
      nextSession.status = 'PAUSED';
      set((s) => ({
        appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
      }));
      return buildAppNavOverlay(nextSession);
    }

    // ── GOAL REACHED ──
    if (distToGoal <= APP_NAV.goalTolerance) {
      if (nextSession.finalHeadingRad != null) {
        const err = normalizeAngle(nextSession.finalHeadingRad - pose.theta);
        if (Math.abs(err) > APP_NAV.finalHeadingTolerance) {
          const cmdW = Math.max(-APP_NAV.rotateMaxW, Math.min(APP_NAV.rotateMaxW, err * APP_NAV.rotateGain));
          if (mux) mux.send(VEL_SOURCE.NAVIGATION, 0, cmdW);
          nextSession.status = 'F_TURN';
          set((s) => ({
            appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
          }));
          return buildAppNavOverlay(nextSession);
        }
      }

      get().stopAppNavigation(id, 'DONE', true);
      return { nav: 'DONE', nav_wp: nextSession.path.length, nav_total: nextSession.path.length };
    }

    // ── RECOVERY MODE ──
    if (nextSession.recoveryMode) {
      console.warn(`[AppNav] RECOVERY entry: mode=${nextSession.recoveryMode}, until=${nextSession.recoveryUntil - now}ms, attempts=${nextSession.recoveryAttempts}`);
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
          appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
        }));
        return buildAppNavOverlay(nextSession);
      }

      // Recovery REPLAN
      if (nextSession.recoveryMode === 'REPLAN') {
        if (!grid || !navWorkerApi || state.navComputationBusy[id]) {
          nextSession.status = 'RECOVERY_REPLAN';
          set((s) => ({
            appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
          }));
          return buildAppNavOverlay(nextSession);
        }

        set((s) => ({
          navComputationBusy: { ...s.navComputationBusy, [id]: true },
        }));

        navWorkerApi.findPath(grid.serialize(), pose.x, pose.y, goal.x, goal.y, true, true)
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
                appNavigationSessions: { ...s.appNavigationSessions, [id]: updated },
                navComputationBusy: { ...s.navComputationBusy, [id]: false },
              }));
            } else {
              const retries = fresh.recoveryAttempts + 1;
              if (retries >= APP_NAV.maxRecoveryAttempts) {
                get().stopAppNavigation(id, 'ERROR', true);
                set((s) => ({
                  navComputationBusy: { ...s.navComputationBusy, [id]: false },
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
                  appNavigationSessions: { ...s.appNavigationSessions, [id]: updated },
                  navComputationBusy: { ...s.navComputationBusy, [id]: false },
                }));
              }
            }
          })
          .catch((err) => {
            console.error('[AppNav] Replan error:', err);
            get().stopAppNavigation(id, 'ERROR', true);
            set((s) => ({
              navComputationBusy: { ...s.navComputationBusy, [id]: false },
            }));
          });

        nextSession.status = 'RECOVERY_REPLAN';
        set((s) => ({
          appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
        }));
        return buildAppNavOverlay(nextSession);
      }

      // Transition to REPLAN after spin/backup
      nextSession.recoveryMode = 'REPLAN';
      nextSession.status = 'RECOVERY_REPLAN';

      // Nav2-style: Clear costmap around robot before replanning
      if (grid && typeof grid.clearCostmapAround === 'function') {
        grid.clearCostmapAround(pose.x, pose.y, 2.0);
        console.log(`[Recovery] Cleared costmap around robot (2m radius) before replan`);
      }

      set((s) => ({
        appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
      }));
      return buildAppNavOverlay(nextSession);
    }

    // ── STUCK DETECTION ──
    if (now - nextSession.lastProgressTime > APP_NAV.progressTimeoutMs) {
      console.warn(`[AppNav] STUCK detected: no progress for ${((now - nextSession.lastProgressTime)/1000).toFixed(1)}s, attempts=${nextSession.recoveryAttempts}`);
      nextSession.recoveryAttempts += 1;
      if (nextSession.recoveryAttempts >= APP_NAV.maxRecoveryAttempts) {
        get().stopAppNavigation(id, 'ERROR', true);
        return { nav: 'ERROR', nav_wp: nextSession.currentWaypointIndex, nav_total: nextSession.path.length };
      }

      nextSession.recoveryMode = nextSession.recoveryAttempts % 2 === 1 ? 'RECOVERY_SPIN' : 'RECOVERY_BACKUP';
      nextSession.recoveryUntil = now + (nextSession.recoveryMode === 'RECOVERY_SPIN' ? APP_NAV.recoverySpinMs : APP_NAV.recoveryBackupMs);
      nextSession.status = nextSession.recoveryMode;
      set((s) => ({
        appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
      }));
      return buildAppNavOverlay(nextSession);
    }

    // ── DWA LOCAL PLANNER ──
    if (!grid || !navWorkerApi) {
      // Fallback: simple proportional controller
      const target = nextSession.path[Math.min(nextSession.currentWaypointIndex + 1, nextSession.path.length - 1)];
      const headingErr = normalizeAngle(Math.atan2(target.y - pose.y, target.x - pose.x) - pose.theta);
      const cmdV = Math.abs(headingErr) > 0.7 ? 0.0 : 0.12;
      const cmdW = Math.max(-APP_NAV.rotateMaxW, Math.min(APP_NAV.rotateMaxW, headingErr * 1.4));
      if (mux) mux.send(VEL_SOURCE.NAVIGATION, cmdV, cmdW);
      nextSession.status = 'TRACK';
      set((s) => ({
        appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
      }));
      return buildAppNavOverlay(nextSession);
    }

    // DWA via Web Worker
    if (!state.navComputationBusy[id]) {
      set((s) => ({
        navComputationBusy: { ...s.navComputationBusy, [id]: true },
      }));

      const dwaConfig = getDWAStore().dwaConfig;

      // CRITICAL: Only pass REMAINING path (from current waypoint onward).
      // Passing the full path caused pickLocalGoal to select already-passed
      // waypoints BEHIND the robot → robot turns around → circles forever.
      const remainingPath = nextSession.path.slice(nextSession.currentWaypointIndex);

      navWorkerApi.computeVelocity(pose, vel, remainingPath, grid.serialize(), dwaConfig, telem.lidar)
        .then((cmd) => {
          const fresh = get().appNavigationSessions[id];
          const freshRobotState = getRobotStore();
          const freshMux = freshRobotState.velocityMuxes[id];
          if (!fresh?.active || fresh.paused) {
            set((s) => ({
              navComputationBusy: { ...s.navComputationBusy, [id]: false },
            }));
            return;
          }

          if (!cmd?.ok) {
            console.warn(`[AppNav] DWA failed: ${cmd?.reason}`, cmd?.diag || {});
            const updated = {
              ...fresh,
              recoveryAttempts: fresh.recoveryAttempts + 1,
              recoveryMode: 'REPLAN',
              recoveryUntil: Date.now(),
              status: 'RECOVERY_REPLAN',
            };
            set((s) => ({
              appNavigationSessions: { ...s.appNavigationSessions, [id]: updated },
              navComputationBusy: { ...s.navComputationBusy, [id]: false },
            }));
            if (freshMux) freshMux.release(VEL_SOURCE.NAVIGATION);
            return;
          }

          if (freshMux) {
            console.log(`[AppNav] DWA output: v=${cmd.v.toFixed(3)}, w=${cmd.w.toFixed(3)}, diag: ${JSON.stringify(cmd.diag)}`);
            freshMux.send(VEL_SOURCE.NAVIGATION, cmd.v, cmd.w);
          }

          // Store DWA trajectory for visualization
          const traj = cmd.trajectory || [];

          // NOTE: Do NOT reset recoveryAttempts here!
          // Only reset when robot has actually moved (proven progress)
          // in the stuck detection block above. Resetting here caused
          // infinite TRACK↔RECOVERY oscillation.
          set((s) => ({
            appNavigationSessions: {
              ...s.appNavigationSessions,
              [id]: { ...fresh, status: 'TRACK', recoveryMode: null },
            },
            navComputationBusy: { ...s.navComputationBusy, [id]: false },
            dwaTrajectories: { ...s.dwaTrajectories, [id]: traj },
          }));
        })
        .catch((err) => {
          console.error('[AppNav] computeVelocity error:', err);
          get().stopAppNavigation(id, 'ERROR', true);
          set((s) => ({
            navComputationBusy: { ...s.navComputationBusy, [id]: false },
          }));
        });
    }

    nextSession.status = 'TRACK';
    set((s) => ({
      appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
    }));
    return buildAppNavOverlay(nextSession);
  },

  // ============================================================
  //   HIGH-LEVEL NAVIGATION API
  // ============================================================

  /**
   * Navigate robot to a goal position using A* + DWA.
   * Unified entry point — works for both real and sim robots.
   */
  navigateRobot: (id, path, finalHeading = null) => {
    const robotState = getRobotStore();
    const robot = robotState.robots[id];
    const mapState = getMapStore();

    const isConnected = robot?.adapter?.connected ?? robot?.connection?.connected;
    if (!isConnected || !path || path.length === 0) return false;

    const hasGrid = !!(mapState.mapperInstances[id] || mapState.occupancyGrid[id]);
    const preferAppNav = robot.telemetry?.architecture === 'pc_slam' || hasGrid;
    if (preferAppNav) {
      return get().startAppNavigation(id, path, finalHeading);
    }
    // Fallback: send path directly to ESP32
    if (robot.adapter) {
      robot.adapter.navigate(path, finalHeading);
    } else if (robot.connection) {
      robot.connection.navigate(path, finalHeading);
    }
    return true;
  },

  /**
   * Navigate to a goal by computing path first (Point-to-Go)
   * Called from RVizPanel click handler.
   */
  navigateToGoal: async (robotId, goalX, goalY, finalHeading = null) => {
    const robotState = getRobotStore();
    const mapState = getMapStore();
    const robot = robotState.robots[robotId];
    if (!robot) return { success: false, error: 'No robot' };

    const telem = robot.telemetry || {};
    const startX = telem.x ?? 0;
    const startY = telem.y ?? 0;

    const grid = mapState.mapperInstances[robotId] || mapState.occupancyGrid[robotId];

    if (!grid) {
      // No map — send direct waypoint
      console.warn('[NavStore] No map available — sending direct waypoint');
      const path = [{ x: startX, y: startY }, { x: goalX, y: goalY }];
      get().startAppNavigation(robotId, path, finalHeading);
      return { success: true, path };
    }

    if (!navWorkerApi) {
      return { success: false, error: 'NavWorker not available' };
    }

    try {
      // allowUnknown=true: Robot can navigate through unexplored space (Nav2 default)
      // useCostmap=true: A* respects inflation costs to stay away from walls
      const result = await navWorkerApi.findPath(grid.serialize(), startX, startY, goalX, goalY, true, true);
      if (result.success && result.path.length > 1) {
        console.log(`[NavStore] ✅ Path found: ${result.path.length} waypoints. Path:`, result.path.map(p => `(${p.x.toFixed(3)}, ${p.y.toFixed(3)})`).join(' -> '));
        get().startAppNavigation(robotId, result.path, finalHeading);
        return { success: true, path: result.path };
      } else {
        console.warn('[NavStore] ❌ No path found to goal');
        return { success: false, error: 'No path found' };
      }
    } catch (err) {
      console.error('[NavStore] Path error:', err);
      return { success: false, error: err.message };
    }
  },

  /**
   * Stop navigation
   */
  navStopRobot: (id) => {
    const session = get().appNavigationSessions[id];
    if (session?.active) {
      get().stopAppNavigation(id, 'IDLE', true);
    }
    const robotState = getRobotStore();
    const robot = robotState.robots[id];
    if (robot?.adapter?.connected) {
      robot.adapter.navStop();
    } else if (robot?.connection?.connected) {
      robot.connection.navStop();
    }
  },

  /**
   * Pause navigation
   */
  pauseNav: (id) => {
    const session = get().appNavigationSessions[id];
    if (session?.active) {
      set((state) => ({
        appNavigationSessions: {
          ...state.appNavigationSessions,
          [id]: { ...session, paused: true, status: 'PAUSED' },
        },
      }));
    }
    const robotState = getRobotStore();
    const robot = robotState.robots[id];
    if (robot?.adapter?.connected) {
      robot.adapter.pause();
    } else if (robot?.connection?.connected) {
      robot.connection.pause();
    }
  },

  /**
   * Resume navigation
   */
  resumeNav: (id) => {
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
    const robotState = getRobotStore();
    const robot = robotState.robots[id];
    if (robot?.adapter?.connected) {
      robot.adapter.resume();
    } else if (robot?.connection?.connected) {
      robot.connection.resume();
    }
  },
}));

// Register in store registry for cross-store access
registerStore('navStore', useNavStore);

export default useNavStore;
