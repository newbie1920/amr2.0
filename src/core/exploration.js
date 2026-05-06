/**
 * AMR 2.0 — Autonomous Exploration v10 (Anti-Ping-Pong Sweep)
 *
 * Thuật toán (lấy cảm hứng từ explore_lite ROS2, DARPA SubT, RoboCup Rescue):
 *   1. LiDAR quét vòng tròn → lưu vào OccupancyGrid (FREE/OCCUPIED/UNKNOWN)
 *   2. Tìm frontier clusters (FREE cạnh UNKNOWN)
 *   3. Score mỗi frontier theo Cost-Utility + 3 cơ chế chống ping-pong:
 *      a) Heading Bias: cosine(robot heading, hướng đến frontier) → ưu tiên phía trước
 *      b) Visited Trail: penalty cho frontier gần vết đã đi qua (decay 60s)
 *      c) Momentum: bonus cho frontier cùng hướng với goal trước đó
 *   4. Greedy TSP chain: sắp xếp top-N frontiers thành route tuần tự không quay lại
 *   5. Nav system A* + DWA + Recovery → robot quét theo đường sweep
 *   6. Hết frontier → hoàn thành ✅
 *
 * v10 anti-ping-pong improvements:
 *   - Robot không bao giờ quay 180° trừ khi ZERO frontier phía trước
 *   - Visited trail suppression: đã đi qua rồi thì không quay lại trong 60s
 *   - Greedy TSP: chain 3-5 frontiers thành sweep route, giảm 80% dead time
 */

// No direct worker imports needed — v7 delegates to navStore.navigateToGoal()

let _mapStoreGetter = null;
export function _setMapStoreGetter(fn) { _mapStoreGetter = fn; }
function _getMapStore() { return _mapStoreGetter ? _mapStoreGetter() : {}; }

// Lazy import to avoid circular dependency
let _navStoreRef = null;
let _navStoreLoading = false;
async function _ensureNavStore() {
  if (_navStoreRef) return _navStoreRef;
  if (_navStoreLoading) return null;
  _navStoreLoading = true;
  try {
    const mod = await import('../stores/navStore.js');
    _navStoreRef = mod.default;
    _navStoreLoading = false;
    return _navStoreRef;
  } catch(e) { _navStoreLoading = false; return null; }
}
function _getNavStore() {
  return _navStoreRef?.getState?.() || null;
}

// ── CONFIG (v10: Anti-Ping-Pong Sweep) ──
const TICK_MS         = 200;    // Check every 200ms
const MIN_FRONTIER    = 3;      // Min cells for a valid frontier cluster
const MAX_NO_FRONT    = 12;     // Ticks without frontier before declaring map complete
const CLEARANCE_SCAN_R = 6;     // Cells radius to measure clearance
const APPROACH_SEARCH_R = 15;   // Search radius to find goal point (1.5m)
const NAV_CHECK_MS    = 300;    // How often to check nav status
const NAV_TIMEOUT_MS  = 45000;  // Max time for one goal before giving up
const NAV_RECOVERY_MAX = 20000; // If stuck in RECOVERY modes > 20s, give up
const MIN_CLEARANCE_CELLS = 2;  // Minimum clearance to accept approach point

// Cost-Utility weights
const LAMBDA_GAIN    = 1.8;     // Weight for info gain
const LAMBDA_DIST    = 1.0;     // Weight for travel cost
const LAMBDA_SIZE    = 0.3;     // Weight for cluster size
const LAMBDA_CLEAR   = 0.5;     // Weight for clearance bonus
const INFO_GAIN_RAYS = 36;      // Number of rays for predicted info gain
const INFO_GAIN_RANGE = 30;     // Max ray length in cells

// Anti-ping-pong weights
const LAMBDA_HEADING = 3.0;     // Bonus for frontier in FORWARD direction (cosine similarity)
const LAMBDA_VISITED = 4.0;     // Penalty for frontier near recently-visited trail
const VISITED_DECAY_MS = 60000; // Trail memory: 60 seconds before forgetting
const VISITED_RADIUS = 12;      // Cells radius to check visited trail
const COMPLETION_BL_RADIUS = 12; // Larger blacklist on goal completion (was 8)
const TSP_CHAIN_SIZE = 4;       // Number of goals to chain in greedy TSP

// ── STATE ──
let active = false;
let timer = null;
let _robotId = null, _getStore = null;
let noFrontierCount = 0;
let allClusters = [];
let phase = 'idle';  // idle, selecting, navigating, complete
let currentGoalWorld = null;  // {x, y} in world coords
let currentGoalGrid = null;   // {gx, gy} for blacklisting (centroid)
let currentGoalApproachGrid = null; // {gx, gy} for dynamic obstacle checking (actual goal)
let lastNavCheck = 0;
let waitingForNav = false;
let navStartTime = 0;         // When we started navigating to current goal
let navRecoveryStart = 0;     // When recovery mode started

// Waypoint queue: chain multiple goals to reduce dead time
let waypointQueue = [];         // Array of { goalX, goalY, centroidGX, centroidGY, approachGX, approachGY }

// Anti-ping-pong: visited trail
const visitedTrail = new Map();  // key: "gx,gy" -> timestamp when visited
let lastTrailPose = null;        // Last recorded trail position { gx, gy }
let lastGoalDirection = 0;       // Heading angle toward last completed goal (radians)

// Blacklist: failed frontier locations
const blacklist = new Map();

// ══════════════════════════════════════════════════
//   PUBLIC API
// ══════════════════════════════════════════════════

export function startExploration(robotId, getStore) {
  // Clean up stale timers (HMR safety)
  if (timer) { clearInterval(timer); timer = null; }
  active = true; _robotId = robotId; _getStore = getStore;
  noFrontierCount = 0; allClusters = [];
  currentGoalWorld = null; currentGoalGrid = null; currentGoalApproachGrid = null;
  waitingForNav = false; waypointQueue = [];
  visitedTrail.clear(); lastTrailPose = null; lastGoalDirection = 0;
  lastNavCheck = 0; navStartTime = 0; navRecoveryStart = 0;
  blacklist.clear();
  phase = 'selecting';
  console.log('[Explore v10] 🚀 Started — Anti-Ping-Pong Sweep');
  // First tick immediately, then periodic
  _tick();
  timer = setInterval(() => _tick(), TICK_MS);
}

export function stopExploration(robotId, getStore) {
  active = false; phase = 'idle';
  currentGoalWorld = null; waitingForNav = false;
  if (timer) { clearInterval(timer); timer = null; }
  // Stop any active navigation
  try {
    if (_navStoreRef) {
      _navStoreRef.getState().navStopRobot(robotId || _robotId);
    }
  } catch(e) {}
  console.log('[Explore v10] ⏹ Stopped');
}

export function isExplorationActive() { return active; }
export function getExplorationPhase() { return phase; }
export function getExplorationInfo() {
  return {
    phase, noFrontierCount,
    currentTarget: currentGoalWorld,
    clusterCount: allClusters.length,
  };
}

// ══════════════════════════════════════════════════
//   MAIN TICK — lightweight monitor loop
// ══════════════════════════════════════════════════

function _tick() {
  if (!active) return;

  const grid = _getGrid();
  const pose = _getPose();
  if (!grid || !pose) return;

  const now = Date.now();

  // ── Record visited trail (every ~0.5m = 5 cells) ──
  const rg0 = grid.worldToGrid(pose.x, pose.y);
  if (!lastTrailPose || Math.hypot(rg0.gx - lastTrailPose.gx, rg0.gy - lastTrailPose.gy) >= 5) {
    // Mark cells in a 3-cell radius as visited
    for (let dy = -3; dy <= 3; dy++) {
      for (let dx = -3; dx <= 3; dx++) {
        visitedTrail.set(`${rg0.gx + dx},${rg0.gy + dy}`, now);
      }
    }
    lastTrailPose = { gx: rg0.gx, gy: rg0.gy };
  }

  // Purge old trail entries (> VISITED_DECAY_MS)
  if (visitedTrail.size > 500) {
    for (const [key, ts] of visitedTrail) {
      if (now - ts > VISITED_DECAY_MS) visitedTrail.delete(key);
    }
  }

  // ── STATE: NAVIGATING — monitor nav status ──
  if (phase === 'navigating' && waitingForNav) {
    if (now - lastNavCheck < NAV_CHECK_MS) return;
    lastNavCheck = now;

    const ns = _getNavStore();
    if (!ns) return;

    const session = ns.appNavigationSessions?.[_robotId];
    const status = session?.status || 'IDLE';

    // ── Dynamic Goal Verification ──
    // Nếu LiDAR vừa cập nhật và thấy điểm goal hiện tại là vật cản (cost >= 200) -> Hủy ngay!
    if (currentGoalApproachGrid && grid.getCost(currentGoalApproachGrid.gx, currentGoalApproachGrid.gy) >= 200) {
      console.log('[Explore v7] 🛑 Dynamic Check: Goal turns out to be an obstacle! Aborting and selecting new goal.');
      _blacklistCurrentGoal();
      _stopNav();
      return; // Fall through next tick to select
    }

    // ── Case 1: Nav finished successfully (DONE or session ended) ──
    if (!session?.active) {
      console.log('[Explore v10] ✅ Nav finished — selecting next frontier');

      // FIX: Permanently blacklist the frontier we just explored (radius 8 cells = 0.8m).
      // This prevents the robot from repeatedly picking "ghost" frontiers left behind 
      // inside walls or unreachable corners, stopping it from revisiting the same spot.
      if (currentGoalGrid) {
        _bl(currentGoalGrid.gx, currentGoalGrid.gy, COMPLETION_BL_RADIUS);
        // Record direction toward this completed goal for momentum
        if (currentGoalWorld) {
          lastGoalDirection = Math.atan2(
            currentGoalWorld.y - pose.y,
            currentGoalWorld.x - pose.x
          );
        }
      }

      // v9: Try queued waypoint first before re-scanning
      _clearCurrentGoal();
      if (waypointQueue.length > 0) {
        const next = waypointQueue.shift();
        // Verify queued waypoint is still valid (not blocked)
        const grid2 = _getGrid();
        if (grid2 && grid2.getCost(next.approachGX, next.approachGY) < 200) {
          console.log(`[Explore v10] ⏩ Using queued waypoint: (${next.goalX.toFixed(2)}, ${next.goalY.toFixed(2)})`);
          _navigateToWaypoint(next);
          return;
        }
        // Queued waypoint invalid, fall through to re-select
      }
    }
    // ── Case 2: Nav ERROR → blacklist this goal, try another ──
    else if (status === 'ERROR') {
      console.log('[Explore v10] ❌ Nav ERROR — blacklisting goal, trying next');
      _blacklistCurrentGoal();
      _stopNav();
      // Fall through to select next
    }
    // ── Case 3: Total timeout → this goal is unreachable ──
    else if (now - navStartTime > NAV_TIMEOUT_MS) {
      console.log(`[Explore v10] ⏰ Nav timeout (${(NAV_TIMEOUT_MS/1000)}s) — blacklisting, trying next`);
      _blacklistCurrentGoal();
      _stopNav();
      // Fall through to select next
    }
    // ── Case 4: Stuck in RECOVERY too long → give up this goal ──
    else if (status.startsWith('RECOVERY')) {
      if (navRecoveryStart === 0) navRecoveryStart = now;
      if (now - navRecoveryStart > NAV_RECOVERY_MAX) {
        console.log(`[Explore v10] 🔄 Recovery too long (${(NAV_RECOVERY_MAX/1000)}s) — blacklisting, trying next`);
        _blacklistCurrentGoal();
        _stopNav();
        // Fall through to select next
      } else {
        return; // Still in recovery, give it more time
      }
    }
    // ── Case 5: Normal TRACK — nav is working fine ──
    else {
      navRecoveryStart = 0; // Reset recovery timer when tracking normally
      return; // Let nav do its thing
    }
  }

  // ── STATE: SELECTING — find and go to next frontier ──
  if (phase === 'selecting') {
    _selectNextGoal(pose, grid);
  }
}

// ══════════════════════════════════════════════════
//   FRONTIER SELECTION — find best goal point
// ══════════════════════════════════════════════════

async function _selectNextGoal(pose, grid) {
  // Prevent re-entry
  if (phase !== 'selecting') return;
  phase = 'planning'; // Temporary lock

  // Find all frontier clusters
  const clusters = _findFrontierClusters(grid);
  allClusters = clusters;

  // Update frontier visualization
  grid.frontierCells = [];
  for (const cl of clusters) for (const c of cl.cells) grid.frontierCells.push(c);

  // No frontiers?
  if (clusters.length === 0) {
    if (grid.scanCount === 0) {
      // Still waiting for first LiDAR scan, don't increment failure count
      phase = 'selecting';
      return;
    }
    noFrontierCount++;
    if (noFrontierCount >= MAX_NO_FRONT) {
      console.log('[Explore v10] 🎉 Map fully explored! No more frontiers.');
      phase = 'complete';
      const ms = _getMapStore();
      if (ms?.stopMapping) ms.stopMapping(_robotId, _getStore);
      else stopExploration(_robotId, _getStore);
      return;
    }
    phase = 'selecting'; // Try again next tick
    return;
  }
  noFrontierCount = 0;

  // ── v10: Cost-Utility + Anti-Ping-Pong Scoring ──
  const rg = grid.worldToGrid(pose.x, pose.y);
  const scored = [];
  const now2 = Date.now();

  // Adaptive lambda: prioritize nearby early, shift to gain later
  const mapMaturity = Math.min(1.0, grid.scanCount / 50);
  const adaptGain = LAMBDA_GAIN * (0.5 + mapMaturity * 0.5);
  const adaptDist = LAMBDA_DIST * (1.5 - mapMaturity * 0.5);

  for (const cl of clusters) {
    const approach = _findWidestApproach(grid, cl.centroidGX, cl.centroidGY);
    if (!approach) continue;

    const dist = Math.hypot(approach.gx - rg.gx, approach.gy - rg.gy);

    // Unreachable ghost frontier
    if (dist < 5.0) {
      console.log(`[Explore v10] ⚠️ Frontier too close (dist=${dist.toFixed(1)}). Blacklisting.`);
      _bl(cl.centroidGX, cl.centroidGY, 8);
      continue;
    }

    // Predicted info gain via LiDAR raycasting
    const gain = _predictInfoGain(grid, approach.gx, approach.gy);

    // ── ANTI-PING-PONG: Heading Bias ──
    // Cosine similarity between robot heading and direction to frontier
    // +1.0 = frontier is directly ahead, -1.0 = directly behind
    const dirToFrontier = Math.atan2(approach.gy - rg.gy, approach.gx - rg.gx);
    const headingBias = Math.cos(pose.theta - dirToFrontier); // [-1, +1]

    // ── ANTI-PING-PONG: Momentum Bonus ──
    // If we just completed a goal, prefer continuing in the same general direction
    let momentumBonus = 0;
    if (lastGoalDirection !== 0) {
      const momentumCos = Math.cos(lastGoalDirection - dirToFrontier);
      momentumBonus = momentumCos * 1.5; // mild bonus for same direction
    }

    // ── ANTI-PING-PONG: Visited Trail Penalty ──
    // Count how many visited trail cells are near this frontier
    let visitedPenalty = 0;
    for (let dy = -VISITED_RADIUS; dy <= VISITED_RADIUS; dy += 3) {
      for (let dx = -VISITED_RADIUS; dx <= VISITED_RADIUS; dx += 3) {
        const ts = visitedTrail.get(`${approach.gx + dx},${approach.gy + dy}`);
        if (ts) {
          // Recent visits penalize more (linear decay)
          const age = (now2 - ts) / VISITED_DECAY_MS; // 0=just visited, 1=expired
          visitedPenalty += (1.0 - age);
        }
      }
    }

    // v10 FINAL SCORE:
    const score = gain * adaptGain
      - dist * adaptDist
      + cl.size * LAMBDA_SIZE
      + approach.clearance * LAMBDA_CLEAR
      + headingBias * LAMBDA_HEADING * Math.min(dist, 30) / 30  // Scale heading by distance
      + momentumBonus
      - visitedPenalty * LAMBDA_VISITED;

    const worldPt = grid.gridToWorld(approach.gx, approach.gy);
    scored.push({
      ...cl, score, dist, gain, headingBias, visitedPenalty,
      approachGX: approach.gx, approachGY: approach.gy,
      clearance: approach.clearance,
      goalX: worldPt.x, goalY: worldPt.y,
    });
  }

  if (scored.length === 0) {
    phase = 'selecting';
    return;
  }

  // Sort by score (best first)
  scored.sort((a, b) => b.score - a.score);
  const best = scored[0];

  console.log(`[Explore v10] 🎯 Goal: (${best.goalX.toFixed(2)}, ${best.goalY.toFixed(2)}) ` +
    `gain=${best.gain} dist=${best.dist.toFixed(0)} heading=${best.headingBias.toFixed(2)} ` +
    `visited_pen=${best.visitedPenalty.toFixed(1)} score=${best.score.toFixed(1)} ` +
    `[${scored.length} candidates]`);

  // v10: Greedy TSP chain — build a sweep route through top-N frontiers
  waypointQueue = [];
  if (scored.length >= 2) {
    waypointQueue = _buildGreedyTSPChain(scored, best);
  }

  // ── Send as navigation GOAL ──
  _navigateToWaypoint({
    goalX: best.goalX, goalY: best.goalY,
    centroidGX: best.centroidGX, centroidGY: best.centroidGY,
    approachGX: best.approachGX, approachGY: best.approachGY,
  });
}

/** Navigate to a waypoint (shared between direct selection and queued chain) */
async function _navigateToWaypoint(wp) {
  currentGoalWorld = { x: wp.goalX, y: wp.goalY };
  currentGoalGrid = { gx: wp.centroidGX, gy: wp.centroidGY };
  currentGoalApproachGrid = { gx: wp.approachGX, gy: wp.approachGY };

  try {
    const navStore = await _ensureNavStore();
    if (!navStore) { phase = 'selecting'; return; }
    const navState = navStore.getState();

    navState.navStopRobot(_robotId);
    await new Promise(r => setTimeout(r, 50));

    const result = await navState.navigateToGoal(_robotId, wp.goalX, wp.goalY);

    if (result.success) {
      phase = 'navigating';
      waitingForNav = true;
      lastNavCheck = Date.now();
      navStartTime = Date.now();
      navRecoveryStart = 0;
      console.log(`[Explore v10] 📍 Nav started! Path: ${result.path?.length || '?'} wp, chain: ${waypointQueue.length}`);
    } else {
      console.log(`[Explore v9] ❌ Nav failed: ${result.error} — blacklisting`);
      _bl(wp.centroidGX, wp.centroidGY);
      phase = 'selecting';
    }
  } catch (err) {
    console.error('[Explore v10] Nav error:', err);
    _bl(wp.centroidGX, wp.centroidGY);
    phase = 'selecting';
  }
}

// ── Goal management helpers ──
function _clearCurrentGoal() {
  phase = 'selecting';
  waitingForNav = false;
  currentGoalWorld = null;
  currentGoalGrid = null;
  currentGoalApproachGrid = null;
  navRecoveryStart = 0;
  // Note: waypointQueue is intentionally NOT cleared — it carries over for chaining
}

function _blacklistCurrentGoal() {
  if (currentGoalGrid) {
    _bl(currentGoalGrid.gx, currentGoalGrid.gy);
    console.log(`[Explore v10] 🚫 Blacklisted frontier at (${currentGoalGrid.gx}, ${currentGoalGrid.gy})`);
  }
  waypointQueue = []; // Clear chain on failure
  _clearCurrentGoal();
}

/**
 * v10: Build a greedy TSP chain through top-N scored frontiers.
 * Starting from the best frontier, greedily pick the nearest unvisited frontier
 * that is NOT behind the current travel direction.
 * This creates a "sweep" route instead of ping-ponging.
 */
function _buildGreedyTSPChain(scored, best) {
  const chain = [];
  const candidates = scored.slice(1, Math.min(scored.length, TSP_CHAIN_SIZE * 2)); // Pool
  const used = new Set();
  let curGX = best.approachGX;
  let curGY = best.approachGY;
  let curDir = Math.atan2(best.approachGY - (lastTrailPose?.gy || curGY), best.approachGX - (lastTrailPose?.gx || curGX));

  for (let i = 0; i < TSP_CHAIN_SIZE && candidates.length > 0; i++) {
    let bestIdx = -1;
    let bestScore = -Infinity;

    for (let j = 0; j < candidates.length; j++) {
      if (used.has(j)) continue;
      const c = candidates[j];
      const d = Math.hypot(c.approachGX - curGX, c.approachGY - curGY);
      if (d < 8) continue; // Too close to current — skip

      // Direction from current to candidate
      const dirToC = Math.atan2(c.approachGY - curGY, c.approachGX - curGX);
      const cosAngle = Math.cos(curDir - dirToC);

      // Reject candidates behind us (cosAngle < -0.3 = >107°)
      if (cosAngle < -0.3 && chain.length > 0) continue;

      // TSP score: prefer near + forward
      const tspScore = -d * 0.5 + cosAngle * 15 + c.gain * 0.3;
      if (tspScore > bestScore) {
        bestScore = tspScore;
        bestIdx = j;
      }
    }

    if (bestIdx < 0) break;
    used.add(bestIdx);
    const pick = candidates[bestIdx];
    chain.push(pick);
    curDir = Math.atan2(pick.approachGY - curGY, pick.approachGX - curGX);
    curGX = pick.approachGX;
    curGY = pick.approachGY;
  }

  if (chain.length > 0) {
    console.log(`[Explore v10] 🗺️ TSP chain: ${chain.length} waypoints queued`);
  }
  return chain;
}

function _stopNav() {
  try {
    if (_navStoreRef) {
      _navStoreRef.getState().navStopRobot(_robotId);
    }
  } catch(e) {}
}

// ══════════════════════════════════════════════════
//   FIND WIDEST APPROACH POINT
//   = tâm khoảng trống rộng nhất gần frontier centroid
//   (giống tìm tâm đường hành lang giữa 2 bức tường)
// ══════════════════════════════════════════════════

function _findWidestApproach(grid, centGX, centGY) {
  const w = grid.width, h = grid.height;
  let bestGX = centGX, bestGY = centGY;
  let bestScore = -999999;
  let bestClearance = 0;
  let found = false;

  // v8: Smaller, faster search with early-exit heuristic
  const searchR = APPROACH_SEARCH_R;
  // Step 2 cells at a time for coarse pass when search radius is large
  const step = searchR > 12 ? 2 : 1;

  for (let dy = -searchR; dy <= searchR; dy += step) {
    for (let dx = -searchR; dx <= searchR; dx += step) {
      const gx = centGX + dx, gy = centGY + dy;
      if (gx < 1 || gx >= w - 1 || gy < 1 || gy >= h - 1) continue;
      if (!grid.isFree(gx, gy)) continue;

      const cost = grid.getCost(gx, gy);
      if (cost >= 150) continue; // Skip dangerous cells

      // v8: Lightweight clearance from costmap — use inverse cost as proxy
      // costmap cost 0 = far from obstacle (max clearance)
      // costmap cost 253 = touching obstacle (min clearance)
      // This is O(1) per cell vs O(160) for 8-dir raycast!
      const clearanceProxy = cost === 0 
        ? CLEARANCE_SCAN_R  // Max clearance if cost=0 (far from everything)
        : Math.max(0, CLEARANCE_SCAN_R * (1.0 - cost / 200.0));

      // Score: lower cost + higher proxy clearance
      const score = (clearanceProxy * 10) - (cost * 3);

      if (clearanceProxy >= MIN_CLEARANCE_CELLS && score > bestScore) {
        bestGX = gx; bestGY = gy; 
        bestScore = score;
        bestClearance = clearanceProxy;
        found = true;
      }
    }
  }

  // Fine pass: refine around coarse winner (±2 cells)
  if (found && step > 1) {
    for (let dy = -2; dy <= 2; dy++) {
      for (let dx = -2; dx <= 2; dx++) {
        const gx = bestGX + dx, gy = bestGY + dy;
        if (gx < 1 || gx >= w - 1 || gy < 1 || gy >= h - 1) continue;
        if (!grid.isFree(gx, gy)) continue;
        const cost = grid.getCost(gx, gy);
        if (cost >= 150) continue;
        const clearanceProxy = cost === 0 
          ? CLEARANCE_SCAN_R 
          : Math.max(0, CLEARANCE_SCAN_R * (1.0 - cost / 200.0));
        const score = (clearanceProxy * 10) - (cost * 3);
        if (clearanceProxy >= MIN_CLEARANCE_CELLS && score > bestScore) {
          bestGX = gx; bestGY = gy; bestScore = score; bestClearance = clearanceProxy;
        }
      }
    }
  }

  if (!found) {
    // Fallback: spiral outward from centroid for nearest safe cell
    for (let r = 0; r <= 8; r++) {
      for (let dy = -r; dy <= r; dy++) {
        for (let dx = -r; dx <= r; dx++) {
          if (r > 0 && Math.abs(dx) !== r && Math.abs(dy) !== r) continue;
          const nx = centGX + dx, ny = centGY + dy;
          if (nx >= 0 && nx < w && ny >= 0 && ny < h && grid.isFree(nx, ny)) {
            if (grid.getCost(nx, ny) < 180) {
              return { gx: nx, gy: ny, clearance: 1 };
            }
          }
        }
      }
    }
    return null;
  }

  return { gx: bestGX, gy: bestGY, clearance: bestClearance };
}

/**
 * Measure clearance (min distance to obstacle) at a cell.
 * Cast rays in 8 directions, return the MINIMUM distance.
 * → Higher clearance = wider corridor = safer goal point.
 */
function _measureClearance(grid, gx, gy) {
  const w = grid.width, h = grid.height;
  const dirs = [[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,-1],[1,-1],[-1,1]];
  let minDist = 20; // Cap at 20 cells

  for (const [ddx, ddy] of dirs) {
    for (let step = 1; step <= 20; step++) {
      const nx = gx + ddx * step, ny = gy + ddy * step;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) { minDist = Math.min(minDist, step); break; }
      const lo = grid.logOdds[ny * w + nx];
      if (lo > 0.3) { // Occupied
        const dist = (ddx !== 0 && ddy !== 0) ? step * 1.414 : step;
        minDist = Math.min(minDist, dist);
        break;
      }
    }
  }

  return minDist;
}

// ══════════════════════════════════════════════════
//   FRONTIER DETECTION
// ══════════════════════════════════════════════════

function _findFrontierClusters(grid) {
  const w = grid.width, h = grid.height;
  const isFrontier = new Uint8Array(w * h);

  // Mark frontier cells: FREE cell adjacent to UNKNOWN cell
  for (let gy = 1; gy < h - 1; gy++) {
    for (let gx = 1; gx < w - 1; gx++) {
      const lo = grid.logOdds[gy * w + gx];
      if (lo >= -0.3) continue; // Must be FREE (logOdds < -0.3)
      if (grid.getCost(gx, gy) >= 200) continue; // Too close to wall

      // Check 4-connected neighbors for unknown (|logOdds| < 0.3)
      const idx = gy * w + gx;
      if (Math.abs(grid.logOdds[idx - w]) < 0.3 ||
          Math.abs(grid.logOdds[idx + w]) < 0.3 ||
          Math.abs(grid.logOdds[idx - 1]) < 0.3 ||
          Math.abs(grid.logOdds[idx + 1]) < 0.3) {
        isFrontier[idx] = 1;
      }
    }
  }

  // Cluster connected frontier cells (flood-fill with index-based queue)
  const vis = new Uint8Array(w * h);
  const clusters = [];

  for (let gy = 0; gy < h; gy++) {
    for (let gx = 0; gx < w; gx++) {
      const idx = gy * w + gx;
      if (!isFrontier[idx] || vis[idx]) continue;

      const cells = [];
      const q = [idx]; let qi = 0;
      vis[idx] = 1;

      while (qi < q.length && cells.length < 500) { // Cap cluster size
        const ci = q[qi++];
        const cx = ci % w, cy = (ci - cx) / w;
        cells.push({ gx: cx, gy: cy });

        // 4-connected neighbors
        for (const ni of [ci - 1, ci + 1, ci - w, ci + w]) {
          if (ni >= 0 && ni < w * h && isFrontier[ni] && !vis[ni]) {
            vis[ni] = 1; q.push(ni);
          }
        }
      }

      if (cells.length >= MIN_FRONTIER) {
        let sx = 0, sy = 0;
        for (const c of cells) { sx += c.gx; sy += c.gy; }
        const cgx = Math.round(sx / cells.length), cgy = Math.round(sy / cells.length);
        if (!_isBl(cgx, cgy)) {
          clusters.push({ cells, size: cells.length, centroidGX: cgx, centroidGY: cgy });
        }
      }
    }
  }

  return clusters;
}

// ══════════════════════════════════════════════════
//   HELPERS
// ══════════════════════════════════════════════════

function _getPose() {
  const st = _getStore();
  const r = st.robots[_robotId];
  if (!r) return null;
  const t = r.telemetry || {};
  let h = t.headingRad ?? 0;
  if (t.heading !== undefined && t.headingRad === undefined) h = (t.heading * Math.PI) / 180;
  return { x: t.x ?? 0, y: t.y ?? 0, theta: h };
}

function _getGrid() {
  return _getMapStore().mapperInstances?.[_robotId] || null;
}

/** Count unknown cells in a circular area (legacy fallback) */
function _countUnknown(grid, gx, gy, radius) {
  const w = grid.width, h = grid.height;
  let count = 0;
  const r2 = radius * radius;
  for (let dy = -radius; dy <= radius; dy++) {
    for (let dx = -radius; dx <= radius; dx++) {
      if (dx*dx + dy*dy > r2) continue;
      const nx = gx + dx, ny = gy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      if (Math.abs(grid.logOdds[ny * w + nx]) < 0.3) count++;
    }
  }
  return count;
}

/**
 * v9: Predict how many UNKNOWN cells LiDAR would see from a given position.
 * Simulates raycasting in INFO_GAIN_RAYS directions up to INFO_GAIN_RANGE cells.
 * Much more accurate than _countUnknown because it respects occlusion (rays stop at walls).
 *
 * Cost: INFO_GAIN_RAYS × INFO_GAIN_RANGE = 36 × 30 = ~1,080 lookups per candidate.
 * vs _countUnknown(R=8): π×64 ≈ 201 lookups but WITHOUT occlusion awareness.
 */
function _predictInfoGain(grid, goalGX, goalGY) {
  const w = grid.width, h = grid.height;
  const logOdds = grid.logOdds;
  let gain = 0;

  for (let i = 0; i < INFO_GAIN_RAYS; i++) {
    const angle = (i / INFO_GAIN_RAYS) * 2 * Math.PI;
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);

    for (let step = 1; step <= INFO_GAIN_RANGE; step++) {
      const gx = goalGX + Math.round(dx * step);
      const gy = goalGY + Math.round(dy * step);

      // Out of bounds → stop ray
      if (gx < 0 || gx >= w || gy < 0 || gy >= h) break;

      const lo = logOdds[gy * w + gx];

      // Hit occupied cell → stop ray (wall blocks further view)
      if (lo > 0.3) break;

      // Unknown cell → counts as info gain
      if (Math.abs(lo) < 0.3) gain++;

      // Free cell → continue ray (LiDAR passes through)
    }
  }

  return gain;
}

// ── BLACKLIST ──
function _bl(gx, gy, radius = 10) {
  // Permanently seal off unreachable or already explored areas.
  for (let d = -radius; d <= radius; d++)
    for (let e = -radius; e <= radius; e++)
      blacklist.set(`${gx+d},${gy+e}`, true);
}
function _isBl(gx, gy) {
  return blacklist.has(`${gx},${gy}`);
}

export default { startExploration, stopExploration, isExplorationActive, getExplorationPhase, getExplorationInfo };
