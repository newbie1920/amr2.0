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
import { injectTrafficIntoGridData } from '../core/trafficManager.js';
import { registerStore, getRobotStoreState, getMapStoreState, getDWAStoreState } from './storeRegistry.js';
import { resolveMapFramePose } from '../core/poseFrames.js';
import { buildDirectPath, isDirectPathClear } from '../core/navigationPathPolicy.js';

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
  progressDistance: 0.03,
  progressTimeoutMs: 6000,
  goalTolerance: 0.18,
  realGoalTolerance: 0.10,
  finalHeadingTolerance: 0.12,
  rotateGain: 1.6,
  rotateMaxW: 0.9,
  recoverySpinMs: 1400,
  recoveryBackupMs: 1100,
  backupSpeed: -0.08,
  spinSpeed: 0.75,
  maxRecoveryAttempts: 999,         // NEVER give up — cycle through recovery strategies
  waypointReachTolerance: 0.22,
  // Pure Pursuit parameters
  lookaheadDist: 0.5,           // Look-ahead distance (meters)
  maxLinearVel: 0.8,            // Max forward speed
  realMaxLinearVel: 0.42,       // Real robots need a calmer final approach than SimBot
  minLinearVel: 0.05,           // Min forward speed
  maxAngularVel: 2.0,           // Max turning speed
  pursuitGain: 2.0,             // Angular gain for Pure Pursuit
  curveSlowdown: 0.4,           // Speed reduction factor when turning hard
  pathDeviationTolerance: 0.55,  // If robot drifts farther from global path, replan from current pose
  pathDeviationGraceMs: 900,
  pathReplanCooldownMs: 1500,
  obstacleDwaThreshold: 8000,   // Only engage DWA if stuck for > 8s (ms)
};

const OBSTACLE_AVOIDANCE_KEY = 'amr_obstacle_avoidance_enabled';

function loadObstacleAvoidanceEnabled() {
  try {
    const raw = localStorage.getItem(OBSTACLE_AVOIDANCE_KEY);
    return raw == null ? true : raw !== 'false';
  } catch {
    return true;
  }
}

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

function pointToSegmentDistance(px, py, ax, ay, bx, by) {
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  if (lenSq <= 1e-9) {
    return { distance: Math.hypot(px - ax, py - ay), t: 0 };
  }
  const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / lenSq));
  const cx = ax + t * dx;
  const cy = ay + t * dy;
  return { distance: Math.hypot(px - cx, py - cy), t };
}

function measurePathDeviation(pose, path, startIndex = 0) {
  if (!path || path.length === 0) {
    return { distance: Infinity, segmentIndex: 0, t: 0 };
  }
  if (path.length === 1) {
    return { distance: Math.hypot(pose.x - path[0].x, pose.y - path[0].y), segmentIndex: 0, t: 0 };
  }

  const firstSegment = Math.max(0, Math.min(startIndex, path.length - 2));
  let best = { distance: Infinity, segmentIndex: firstSegment, t: 0 };
  for (let i = firstSegment; i < path.length - 1; i++) {
    const a = path[i];
    const b = path[i + 1];
    const hit = pointToSegmentDistance(pose.x, pose.y, a.x, a.y, b.x, b.y);
    if (hit.distance < best.distance) {
      best = { distance: hit.distance, segmentIndex: i, t: hit.t };
    }
  }
  return best;
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
//   RECOVERY LOGIC
// ============================================================

function escalateRecovery(id, session, now, get, set) {
  const robotState = getRobotStore();
  const mapState = getMapStore();
  const mux = robotState.velocityMuxes[id];
  const grid = mapState.mapperInstances[id] || mapState.occupancyGrid[id];

  const attempts = session.recoveryAttempts;
  let nextMode = 'REPLAN';
  let delay = 0;

  // Cycle through recovery strategies FOREVER — NEVER give up!
  // Pattern: backup → replan → spin → backup → replan (wider) → repeat
  const cycle = attempts % 5;
  
  if (cycle === 0) {
    // Stage 1: Backup to get out of tight spot
    nextMode = 'RECOVERY_BACKUP';
    delay = APP_NAV.recoveryBackupMs;
  } else if (cycle === 1) {
    // Stage 2: Clear costmap and replan
    nextMode = 'REPLAN';
    if (grid && typeof grid.clearCostmapAround === 'function') {
      grid.clearCostmapAround(session.lastProgressPose?.x ?? 0, session.lastProgressPose?.y ?? 0, 0.8);
    }
  } else if (cycle === 2) {
    // Stage 3: Spin to scan surroundings
    nextMode = 'RECOVERY_SPIN';
    delay = APP_NAV.recoverySpinMs;
  } else if (cycle === 3) {
    // Stage 4: Backup longer to get to open area
    nextMode = 'RECOVERY_BACKUP';
    delay = APP_NAV.recoveryBackupMs * 1.5;
  } else {
    // Stage 5: Clear wider costmap and replan
    nextMode = 'REPLAN';
    if (grid && typeof grid.clearCostmapAround === 'function') {
      const clearRadius = Math.min(1.5, 1.0 + Math.floor(attempts / 5) * 0.2); // Max 1.5m
      grid.clearCostmapAround(session.lastProgressPose?.x ?? 0, session.lastProgressPose?.y ?? 0, clearRadius);
    }
  }

  console.log(`[Recovery] Attempt ${attempts + 1}: mode=${nextMode}, cycle=${cycle}`);

  const updated = {
    ...session,
    recoveryAttempts: attempts + 1,
    recoveryMode: nextMode,
    recoveryUntil: now + delay,
    status: nextMode === 'REPLAN' ? 'RECOVERY_REPLAN' : nextMode,
    lastProgressTime: now, // Reset progress timer to prevent immediate re-trigger
  };

  if (nextMode !== 'REPLAN' && nextMode !== 'RECOVERY_REPLAN' && mux) {
    if (nextMode === 'RECOVERY_SPIN') {
      mux.send(VEL_SOURCE.NAVIGATION, 0, APP_NAV.spinSpeed);
    } else if (nextMode === 'RECOVERY_BACKUP') {
      mux.send(VEL_SOURCE.NAVIGATION, APP_NAV.backupSpeed, 0);
    }
  }

  set((s) => ({
    appNavigationSessions: { ...s.appNavigationSessions, [id]: updated },
  }));
  // BUG #9 FIX: Return raw session, let caller buildAppNavOverlay
  return updated;
}

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
  navModes: {},              // { robotId: 'onboard' | 'pc' } — explicit nav mode per robot
  obstacleAvoidanceEnabled: loadObstacleAvoidanceEnabled(),

  // ============================================================
  //   ACTIONS
  // ============================================================

  setObstacleAvoidanceEnabled: (enabled) => {
    const next = !!enabled;
    try {
      localStorage.setItem(OBSTACLE_AVOIDANCE_KEY, String(next));
    } catch {
      // localStorage can be unavailable in tests or privacy-restricted contexts.
    }
    set({ obstacleAvoidanceEnabled: next });
  },

  /**
   * Set navigation mode for a robot: 'onboard' (ESP32 finds path) or 'pc' (browser A* + DWA)
   * SimBots are always 'pc' — this is enforced in navigateToGoal.
   */
  setNavMode: (robotId, mode) => {
    if (mode !== 'onboard' && mode !== 'pc') {
      console.warn(`[NavStore] Invalid navMode: ${mode}`);
      return;
    }
    console.log(`[NavStore] 🧠 NavMode set: ${robotId} → ${mode}`);
    
    // Also notify the robot to update its dual-mode state
    const robotState = getRobotStore();
    const robot = robotState.robots[robotId];
    if (robot) {
      const modeString = mode === 'pc' ? 'pc_browser' : 'onboard';
      if (robot.adapter?.connected) {
        robot.adapter.setDualMode(modeString);
      } else if (robot.connection?.connected) {
        robot.connection.setDualMode(modeString);
      }
    }

    set((s) => ({
      navModes: { ...s.navModes, [robotId]: mode },
    }));
  },

  /**
   * Get nav mode for a robot. Defaults: SimBot='pc', Real='onboard'
   */
  getNavMode: (robotId) => {
    const explicit = get().navModes[robotId];
    if (explicit) return explicit;
    const robot = getRobotStore().robots[robotId];
    if (robot?._sim) return 'pc';

    // Real robots default to firmware authority. Browser navigation is explicit debug mode only.
    return 'onboard';
  },

  /**
   * Start app-side navigation session (Nav2-style path following)
   * @param {string} id - Robot ID
   * @param {Array} path - Path to follow
   * @param {number|null} finalHeading - Optional final heading
   * @param {boolean} offloaded - If true, delegates execution to ESP32 and only syncs UI state
   */
  startAppNavigation: (id, path, finalHeading = null, offloaded = false) => {
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
    session.offloaded = offloaded; // Track offloaded state
    console.log(`[AppNav] ✅ startAppNavigation SUCCESS (offloaded: ${offloaded})`, { id, pathLen: path.length, goal: session.goal, active: session.active });
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
    const obstacleAvoidanceEnabled = state.obstacleAvoidanceEnabled !== false;

    const robotState = getRobotStore();
    const mapState = getMapStore();
    const robot = robotState.robots[id];
    const mux = robotState.velocityMuxes[id];
    const grid = mapState.mapperInstances[id] || mapState.occupancyGrid[id];
    const pose = resolveMapFramePose({ ...robot, telemetry: telem }, mapState, id);
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
    
    // BUG #8 FIX: Debug log removed (was causing 5 log/s performance leak)

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
    const goalTolerance = robot?._sim ? APP_NAV.goalTolerance : APP_NAV.realGoalTolerance;

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
    if (distToGoal <= goalTolerance) {
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

      if (mux) mux.send(VEL_SOURCE.NAVIGATION, 0, 0);
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

        const navSessions = get().appNavigationSessions;
        const trafficGridData = injectTrafficIntoGridData(grid.serialize(), id, robotState.robots, navSessions);

        navWorkerApi.findPath(trafficGridData, pose.x, pose.y, goal.x, goal.y, true, true)
          .then((result) => {
            const fresh = get().appNavigationSessions[id];
            if (!fresh?.active) return;

            if (result.success && result.path.length > 1) {
              // BUG #3 FIX: Use CURRENT pose from fresh session, not stale closure `pose`
              const currentPose = fresh.currentPose || { x: pose.x, y: pose.y };
              
              // Find the nearest waypoint AHEAD of robot on the new path.
              // Instead of going back to WP 0, merge into path at closest forward point.
              let bestIdx = 0;
              let bestDist = Infinity;
              for (let i = 0; i < result.path.length; i++) {
                const d = Math.hypot(result.path[i].x - currentPose.x, result.path[i].y - currentPose.y);
                if (d < bestDist) {
                  bestDist = d;
                  bestIdx = i;
                }
              }
              // Skip past the closest point (we're already there), advance to next
              const startIdx = Math.min(bestIdx + 1, result.path.length - 1);

              const updated = {
                ...fresh,
                path: result.path,
                goal: result.path[result.path.length - 1],
                currentWaypointIndex: startIdx,
                recoveryMode: null,
                status: 'TRACK',
                _offPathSince: null,
                _lastReplanTime: Date.now(),
                lastProgressTime: Date.now(),
                lastProgressPose: { x: currentPose.x, y: currentPose.y },
              };
              set((s) => ({
                appNavigationSessions: { ...s.appNavigationSessions, [id]: updated },
                navComputationBusy: { ...s.navComputationBusy, [id]: false },
              }));
            } else {
              // Path finding failed (e.g. fully blocked). Escalate recovery!
              escalateRecovery(id, fresh, Date.now(), get, set);
              set((s) => ({
                navComputationBusy: { ...s.navComputationBusy, [id]: false },
              }));
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

      // Nav2-style: Clear costmap only if we're deeply stuck (multiple attempts) to remove ghost obstacles.
      // Reduced radius from 2.0m to 0.8m to prevent erasing real walls which causes an infinite replan loop.
      if (grid && typeof grid.clearCostmapAround === 'function' && nextSession.recoveryAttempts > 1) {
        grid.clearCostmapAround(pose.x, pose.y, 0.8);
        console.log(`[Recovery] Cleared costmap around robot (0.8m radius) before replan`);
      }

      set((s) => ({
        appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
      }));
      return buildAppNavOverlay(nextSession);
    }

      // ── STUCK DETECTION ──
      // If we haven't made progress in progressTimeoutMs, we are stuck.
      if (now - nextSession.lastProgressTime > APP_NAV.progressTimeoutMs) {
        console.warn(`[AppNav] STUCK detected: no progress for ${((now - nextSession.lastProgressTime)/1000).toFixed(1)}s, attempts=${nextSession.recoveryAttempts}`);
        // BUG #9 FIX: escalateRecovery returns raw session, we wrap with buildAppNavOverlay
        const escalatedSession = escalateRecovery(id, nextSession, now, get, set);
        if (escalatedSession) return buildAppNavOverlay(escalatedSession);
      }

      // ── PURE PURSUIT PATH FOLLOWER (main-thread, zero-latency) ──
      // The pre-computed path avoids known obstacles,
      // but we ALSO check live costmap to catch dynamic obstacles and drift.
      
      if (nextSession.offloaded) {
          // Offloaded mode: ESP32 runs DWA. We just sync the UI state.
          if (telem.nav === 'DONE') {
              get().stopAppNavigation(id, 'DONE', true);
              return { nav: 'DONE', nav_wp: nextSession.path.length, nav_total: nextSession.path.length };
          }
          nextSession.currentWaypointIndex = telem.navWp || 0;
          nextSession.status = telem.nav || 'TRACK';
          
          set((s) => ({
            appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
          }));
          return buildAppNavOverlay(nextSession);
      }
      
      let remainingPath = nextSession.path.slice(nextSession.currentWaypointIndex);
      if (remainingPath.length === 0) {
        if (mux) mux.release(VEL_SOURCE.NAVIGATION);
        nextSession.status = 'TRACK';
        set((s) => ({
          appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
        }));
        return buildAppNavOverlay(nextSession);
      }

      // ── LIVE OBSTACLE CHECK (costmap-based forward scan) ──
      // Only scan when robot is actually moving forward — prevents false triggers while rotating.
      // Only brake for INSCRIBED/LETHAL zones (cost >= 200), NOT inflation gradient.
      // The pathfinder already keeps path in safe corridors; this is a LAST RESORT safety net.
      let forwardBlocked = false;
      const currentSpeed = Math.abs(vel.v);
      const replanCooldownMs = 800; // Don't re-trigger replan within 800ms
      const timeSinceLastReplan = now - (nextSession._lastReplanTime || nextSession._lastForwardBlockTime || 0);

      const deviation = measurePathDeviation(
        pose,
        nextSession.path,
        Math.max(0, nextSession.currentWaypointIndex - 1),
      );
      if (deviation.distance <= APP_NAV.pathDeviationTolerance) {
        nextSession._offPathSince = null;
        const rejoinIdx = Math.min(nextSession.path.length - 1, deviation.segmentIndex + 1);
        if (rejoinIdx > nextSession.currentWaypointIndex) {
          nextSession.currentWaypointIndex = rejoinIdx;
          remainingPath = nextSession.path.slice(nextSession.currentWaypointIndex);
        }
      } else if (distToGoal > APP_NAV.lookaheadDist) {
        nextSession._offPathSince = nextSession._offPathSince || now;
        const offPathFor = now - nextSession._offPathSince;
        if (offPathFor >= APP_NAV.pathDeviationGraceMs && timeSinceLastReplan >= APP_NAV.pathReplanCooldownMs) {
          if (mux) mux.send(VEL_SOURCE.NAVIGATION, 0, 0);
          console.warn(`[AppNav] Off global path by ${deviation.distance.toFixed(2)}m for ${offPathFor}ms; replanning from current pose`);
          nextSession.recoveryMode = 'REPLAN';
          nextSession.status = 'RECOVERY_REPLAN';
          nextSession.lastProgressTime = now;
          nextSession._lastReplanTime = now;
          set((s) => ({
            appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
          }));
          return buildAppNavOverlay(nextSession);
        }
      }
      
      if (obstacleAvoidanceEnabled && grid && grid.costmap && currentSpeed > 0.02 && timeSinceLastReplan > replanCooldownMs) {
        const scanSteps = 6;
        // Dynamic braking distance based on current speed
        const scanDist = Math.max(0.20, Math.min(0.40, currentSpeed * 1.2 + 0.10));
        const halfWidth = 0.14;     // Slightly narrower than robot to avoid edge false positives

        for (let step = 1; step <= scanSteps && !forwardBlocked; step++) {
          const d = (step / scanSteps) * scanDist;
          // Only scan forward direction (pose.theta) — NOT target heading
          for (const lateral of [0, -halfWidth, halfWidth]) {
            const checkX = pose.x + d * Math.cos(pose.theta) + lateral * Math.cos(pose.theta + Math.PI/2);
            const checkY = pose.y + d * Math.sin(pose.theta) + lateral * Math.sin(pose.theta + Math.PI/2);
            const g = grid.worldToGrid(checkX, checkY);
            if (grid.inBounds(g.gx, g.gy)) {
              const cost = grid.costmap[g.gy * grid.width + g.gx];
              if (cost >= 200) {  // Only brake for INSCRIBED/LETHAL — real collision imminent
                forwardBlocked = true;
                break;
              }
            }
          }
        }
      }

      if (forwardBlocked) {
        // Emergency brake — stop immediately
        if (mux) mux.send(VEL_SOURCE.NAVIGATION, 0, 0);
        console.warn('[AppNav] 🚨 FORWARD BLOCKED — inscribed/lethal zone! Braking + replan');
        
        nextSession.recoveryMode = 'REPLAN';
        nextSession.status = 'RECOVERY_REPLAN';
        nextSession.lastProgressTime = now;
        nextSession._lastReplanTime = now; // Cooldown timestamp
        set((s) => ({
          appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
        }));
        return buildAppNavOverlay(nextSession);
      }

      // Find look-ahead target (carrot point)
      let targetWp = remainingPath[0];
      let targetIdx = 0;
      for (let i = 0; i < remainingPath.length; i++) {
        const d = Math.hypot(remainingPath[i].x - pose.x, remainingPath[i].y - pose.y);
        if (d >= APP_NAV.lookaheadDist) {
          targetWp = remainingPath[i];
          targetIdx = i;
          break;
        }
        targetWp = remainingPath[i];
        targetIdx = i;
      }
      // If close to all remaining WPs, target the last one
      if (targetIdx === remainingPath.length - 1) {
        targetWp = remainingPath[remainingPath.length - 1];
      }

      // ── LOOKAHEAD PATH COLLISION CHECK ──
      // Scan the upcoming path (up to 2.5 meters ahead) to see if SLAM has newly discovered an obstacle.
      // We sample along the path segments to catch obstacles intersecting long straight lines.
      if (obstacleAvoidanceEnabled && grid && grid.costmap && timeSinceLastReplan > replanCooldownMs) {
        let pathDistChecked = 0;
        let lastX = pose.x;
        let lastY = pose.y;
        let pathBlocked = false;
        
        for (let i = 0; i < remainingPath.length; i++) {
          const wp = remainingPath[i];
          const segDist = Math.hypot(wp.x - lastX, wp.y - lastY);
          
          // Sample along the segment every 0.1m
          const steps = Math.max(1, Math.ceil(segDist / 0.1));
          for (let s = 1; s <= steps; s++) {
            const t = s / steps;
            const cx = lastX + (wp.x - lastX) * t;
            const cy = lastY + (wp.y - lastY) * t;
            
            const cg = grid.worldToGrid(cx, cy);
            if (grid.inBounds(cg.gx, cg.gy)) {
              // Only brake for INSCRIBED/LETHAL (cost >= 200).
              const cost = grid.costmap[cg.gy * grid.width + cg.gx];
              if (cost >= 200) { 
                pathBlocked = true;
                break;
              }
            }
          }
          
          pathDistChecked += segDist;
          lastX = wp.x;
          lastY = wp.y;
          
          if (pathBlocked || pathDistChecked > 2.5) break;
        }
        
        if (pathBlocked) {
          console.warn(`[AppNav] ⚠️ Path blocked ahead (dist=${pathDistChecked.toFixed(2)}m), triggering early replan`);
          nextSession.recoveryMode = 'REPLAN';
          nextSession.status = 'RECOVERY_REPLAN';
          nextSession.lastProgressTime = now;
          nextSession._lastReplanTime = now;
          set((s) => ({
            appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
          }));
          return buildAppNavOverlay(nextSession);
        }
      }

      if (obstacleAvoidanceEnabled && grid && grid.costmap && !nextSession.offloaded) {
        if (!state.navComputationBusy[id]) {
          set((s) => ({
            navComputationBusy: { ...s.navComputationBusy, [id]: true },
          }));

          const navSessions = get().appNavigationSessions;
          const trafficGridData = injectTrafficIntoGridData(grid.serialize(), id, robotState.robots, navSessions);
          const config = getDWAStore().dwaConfig;

          navWorkerApi.computeVelocity(pose, vel, remainingPath, trafficGridData, config, telem.lidar)
            .then((result) => {
              const fresh = get().appNavigationSessions[id];
              if (!fresh?.active) return; // Session ended or paused

              if (result && result.ok && result.v !== undefined) {
                if (mux) {
                  mux.send(VEL_SOURCE.NAVIGATION, result.v, result.w);
                }
                set((s) => ({
                  dwaTrajectories: { ...s.dwaTrajectories, [id]: result.trajectory || [] }
                }));
              } else {
                console.warn('[AppNav] MPPI local planner failed or blocked, reason:', result?.reason);
                // If MPPI fails completely (e.g., all trajectories collided), trigger recovery replan
                if (result?.reason === 'all_trajectories_collided') {
                  const updated = {
                    ...fresh,
                    recoveryMode: 'REPLAN',
                    status: 'RECOVERY_REPLAN',
                    lastProgressTime: Date.now(),
                    _lastReplanTime: Date.now(),
                  };
                  set((s) => ({
                    appNavigationSessions: { ...s.appNavigationSessions, [id]: updated },
                  }));
                }
              }

              set((s) => ({
                navComputationBusy: { ...s.navComputationBusy, [id]: false },
              }));
            })
            .catch((err) => {
              console.error('[AppNav] Local planner computeVelocity error:', err);
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
      }

      // Compute heading error to target
      const targetHeading = Math.atan2(targetWp.y - pose.y, targetWp.x - pose.x);
      const headingErr = normalizeAngle(targetHeading - pose.theta);
      const absErr = Math.abs(headingErr);

      // Pure Pursuit: proportional angular velocity
      let cmdW = headingErr * APP_NAV.pursuitGain;
      cmdW = Math.max(-APP_NAV.maxAngularVel, Math.min(APP_NAV.maxAngularVel, cmdW));

      // Speed control: slow down on sharp turns, speed up on straights
      let cmdV = robot?._sim ? APP_NAV.maxLinearVel : APP_NAV.realMaxLinearVel;
      if (absErr > 0.3) {
        // Turning: reduce speed proportionally
        const turnFactor = Math.max(APP_NAV.curveSlowdown, 1.0 - absErr / 1.57);
        cmdV *= turnFactor;
      }
      if (absErr > 1.2) {
        // Very sharp turn: stop and rotate in-place
        cmdV = 0;
      }
      cmdV = Math.max(APP_NAV.minLinearVel, cmdV);
      if (absErr > 1.2) cmdV = 0; // Override min for in-place rotation

      // Distance to goal — slow approach
      if (distToGoal < 0.55) {
        const approachGain = robot?._sim ? 0.5 : 0.35;
        cmdV = Math.min(cmdV, distToGoal * approachGain);
      }
      if (!robot?._sim && distToGoal < 0.25) {
        cmdV = Math.min(cmdV, 0.08);
      }

      if (mux) mux.send(VEL_SOURCE.NAVIGATION, cmdV, cmdW);

      // Store a simple trajectory preview for visualization
      const previewLen = 5;
      const previewTraj = [];
      for (let i = 0; i < previewLen; i++) {
        const t = (i + 1) * 0.2;
        previewTraj.push({
          x: pose.x + cmdV * Math.cos(pose.theta + cmdW * t * 0.5) * t,
          y: pose.y + cmdV * Math.sin(pose.theta + cmdW * t * 0.5) * t,
        });
      }

      nextSession.status = 'TRACK';
      set((s) => ({
        appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
        dwaTrajectories: { ...s.dwaTrajectories, [id]: previewTraj },
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
    const navMode = get().getNavMode(id);
    
    // Force PC nav for SimBots or explicit 'pc' mode
    if (navMode === 'pc' || robot._sim) {
      return get().startAppNavigation(id, path, finalHeading);
    }
    // Onboard: send path directly to ESP32
    if (robot.adapter) {
      robot.adapter.navigate(path, finalHeading);
    } else if (robot.connection) {
      robot.connection.navigate(path, finalHeading);
    }
    return true;
  },

  navigateToGoal: async (robotId, goalX, goalY, finalHeading = null) => {
    const robotState = getRobotStore();
    const mapState = getMapStore();
    const robot = robotState.robots[robotId];
    if (!robot) return { success: false, error: 'No robot' };

    const isConnected = robot.adapter?.connected ?? robot.connection?.connected;
    if (!isConnected) return { success: false, error: 'Not connected' };

    // ── SIM ROBOTS: Vẫn dùng app-side A* (không có ESP32 thật) ──
    if (robot._sim) {
      const telem = robot.telemetry || {};
      const startX = telem.x ?? 0;
      const startY = telem.y ?? 0;
      const grid = mapState.mapperInstances[robotId] || mapState.occupancyGrid[robotId];

      if (!grid) {
        // No map — send direct waypoint
        const path = [{ x: startX, y: startY }, { x: goalX, y: goalY }];
        get().startAppNavigation(robotId, path, finalHeading);
        return { success: true, path };
      }

      if (!navWorkerApi) {
        return { success: false, error: 'NavWorker not available' };
      }

      try {
        const navSessions = get().appNavigationSessions;
        const trafficGridData = injectTrafficIntoGridData(grid.serialize(), robotId, robotState.robots, navSessions);
        const result = await navWorkerApi.findPath(trafficGridData, startX, startY, goalX, goalY, true, true);
        if (result.success && result.path.length > 1) {
          console.log(`[NavStore] ✅ SimBot path: ${result.path.length} waypoints`);
          get().startAppNavigation(robotId, result.path, finalHeading);
          return { success: true, path: result.path };
        } else {
          return { success: false, error: 'No path found (sim)' };
        }
      } catch (err) {
        console.error('[NavStore] Sim path error:', err);
        return { success: false, error: err.message };
      }
    }

    // ── REAL ROBOTS: Check explicit navMode ──
    const navMode = get().getNavMode(robotId);
    
    if (navMode === 'pc') {
      // PC Navigation: explicit browser debug/simulation path only.
      console.log(`[NavStore] Debug PC Nav mode: browser A* + app-side path follower`);
      const grid = mapState.mapperInstances[robotId] || mapState.occupancyGrid[robotId];
      const enterPcBrowserMode = () => {
        if (robot.adapter?.connected) {
          robot.adapter.setDualMode('pc_browser');
        } else if (robot.connection?.connected) {
          robot.connection.setDualMode('pc_browser');
        }
      };
      enterPcBrowserMode();
      const startPose = resolveMapFramePose(robot, mapState, robotId);
      const startX = startPose.x;
      const startY = startPose.y;

      // Helper: send direct path (fallback when A* fails or no grid)
      const sendDirectPath = (reason) => {
        console.log(`[NavStore] Debug PC Nav fallback (${reason}): direct path to goal`);
        const path = buildDirectPath(startPose, { x: goalX, y: goalY });
        get().startAppNavigation(robotId, path, finalHeading, false);
        return { success: true, path };
      };

      if (get().obstacleAvoidanceEnabled === false) {
        return sendDirectPath('obstacle avoidance disabled');
      }

      if (!grid) {
        return sendDirectPath('no grid available');
      }

      if (!navWorkerApi) {
        return sendDirectPath('NavWorker not available');
      }

      if (isDirectPathClear(grid, startPose, { x: goalX, y: goalY })) {
        return sendDirectPath('clear line-of-sight');
      }

      try {
        // Check if grid has serialize method (ESP32 RLE grid = OccupancyGrid instance)
        if (typeof grid.serialize !== 'function') {
          return sendDirectPath('grid has no serialize method');
        }

        const navSessions = get().appNavigationSessions;
        const trafficGridData = injectTrafficIntoGridData(grid.serialize(), robotId, robotState.robots, navSessions);
        const result = await navWorkerApi.findPath(trafficGridData, startX, startY, goalX, goalY, true, true);
        if (result.success && result.path.length > 1) {
          console.log(`[NavStore] Debug PC path: ${result.path.length} waypoints. Browser path follower will send cmd_vel.`);
          get().startAppNavigation(robotId, result.path, finalHeading, false);

          return { success: true, path: result.path };
        } else {
          // A* failed to find path — fallback to direct
          return sendDirectPath(`A* found no path`);
        }
      } catch (err) {
        console.error('[NavStore] PC nav path error:', err);
        return sendDirectPath(`A* error: ${err.message}`);
      }
    }

    // Onboard Navigation: Send GOTO command, ESP32 handles A* + DWA internally
    console.log(`[NavStore] 🌐 Onboard nav: ESP32 tự dò đường: ${goalX.toFixed(2)}, ${goalY.toFixed(2)}`);
    
    if (robot.adapter) {
        robot.adapter.goto(goalX, goalY, finalHeading);
    } else if (robot.connection) {
        robot.connection.goto(goalX, goalY, finalHeading);
    }

    return { success: true, path: [] }; // Path do ESP32 quản lý
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
