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
  minLinearVel: 0.05,           // Min forward speed
  maxAngularVel: 2.0,           // Max turning speed
  pursuitGain: 2.0,             // Angular gain for Pure Pursuit
  curveSlowdown: 0.4,           // Speed reduction factor when turning hard
  obstacleDwaThreshold: 8000,   // Only engage DWA if stuck for > 8s (ms)
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
      grid.clearCostmapAround(session.lastProgressPose?.x ?? 0, session.lastProgressPose?.y ?? 0, 2.0);
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
      const clearRadius = 3.0 + Math.floor(attempts / 5); // Expand radius each full cycle
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
      const remainingPath = nextSession.path.slice(nextSession.currentWaypointIndex);
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
      const timeSinceLastReplan = now - (nextSession._lastForwardBlockTime || 0);
      
      if (grid && grid.costmap && currentSpeed > 0.02 && timeSinceLastReplan > replanCooldownMs) {
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
        nextSession._lastForwardBlockTime = now; // Cooldown timestamp
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

      // ── NEXT WAYPOINT COLLISION CHECK ──
      // Before driving toward a waypoint, verify the waypoint itself is safe.
      // Only replan for INSCRIBED/LETHAL zones — NOT normal inflation gradient.
      if (grid && grid.costmap) {
        const wpG = grid.worldToGrid(targetWp.x, targetWp.y);
        if (grid.inBounds(wpG.gx, wpG.gy)) {
          const wpCost = grid.costmap[wpG.gy * grid.width + wpG.gx];
          if (wpCost >= 200) {
            // Waypoint is in inscribed/lethal zone — try skipping ahead first
            const nextSafeIdx = nextSession.currentWaypointIndex + targetIdx + 1;
            if (nextSafeIdx < nextSession.path.length - 1) {
              // Skip this dangerous waypoint
              nextSession.currentWaypointIndex = nextSafeIdx;
              console.warn(`[AppNav] ⚠️ Skipping waypoint (cost=${wpCost}), advancing to WP #${nextSafeIdx}`);
            } else {
              // Near end of path, must replan
              console.warn(`[AppNav] ⚠️ Target waypoint cost=${wpCost}, triggering replan`);
              nextSession.recoveryMode = 'REPLAN';
              nextSession.status = 'RECOVERY_REPLAN';
              nextSession.lastProgressTime = now;
              set((s) => ({
                appNavigationSessions: { ...s.appNavigationSessions, [id]: nextSession },
              }));
              return buildAppNavOverlay(nextSession);
            }
          }
        }
      }

      // Compute heading error to target
      const targetHeading = Math.atan2(targetWp.y - pose.y, targetWp.x - pose.x);
      const headingErr = normalizeAngle(targetHeading - pose.theta);
      const absErr = Math.abs(headingErr);

      // Pure Pursuit: proportional angular velocity
      let cmdW = headingErr * APP_NAV.pursuitGain;
      cmdW = Math.max(-APP_NAV.maxAngularVel, Math.min(APP_NAV.maxAngularVel, cmdW));

      // Speed control: slow down on sharp turns, speed up on straights
      let cmdV = APP_NAV.maxLinearVel;
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
      if (distToGoal < 0.4) {
        cmdV = Math.min(cmdV, distToGoal * 0.5);
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

  navigateToGoal: async (robotId, goalX, goalY, finalHeading = null) => {
    const robotState = getRobotStore();
    const robot = robotState.robots[robotId];
    if (!robot) return { success: false, error: 'No robot' };

    const isConnected = robot.adapter?.connected ?? robot.connection?.connected;
    if (!isConnected) return { success: false, error: 'Not connected' };

    // Phân tán: Giao việc trực tiếp cho ESP32 tự tìm đường
    console.log(`[NavStore] 🌐 Giao nhiệm vụ cho ESP32 tự dò đường: ${goalX}, ${goalY}`);
    
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
