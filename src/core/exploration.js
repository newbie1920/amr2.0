/**
 * AMR 2.0 — Autonomous Exploration v7 (Goal-Based Frontier Vacuum)
 *
 * Thuật toán:
 *   1. LiDAR quét vòng tròn → lưu vào OccupancyGrid (FREE/OCCUPIED/UNKNOWN)
 *   2. Tìm "frontier" = ô FREE nằm cạnh ô UNKNOWN (viền vòng tròn chưa khám phá)
 *   3. Với mỗi frontier, đo "clearance" = khoảng rộng nhất giữa 2 vật cản
 *      → Chọn tâm khoảng trống rộng nhất (giống tìm tâm đường hành lang)
 *   4. Gửi điểm đó làm GOAL cho hệ thống nav có sẵn (navigateToGoal)
 *   5. Nav system tự A* + Pure Pursuit + Recovery → robot đi đến đó
 *   6. Khi đến nơi (nav DONE) → quay lại bước 2, chọn frontier mới
 *   7. Hết frontier → hoàn thành ✅
 *
 * Ưu điểm:
 *   - Tái sử dụng toàn bộ hệ thống nav đã verify (A*, Pure Pursuit, Recovery)
 *   - Exploration chỉ là "brain" chọn goal, không tự lái
 *   - Chọn goal vào chỗ RỘNG RÃI nhất → an toàn, ít kẹt
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

// ── CONFIG ──
const TICK_MS         = 500;    // Check every 500ms (lightweight — just monitors nav status)
const MIN_FRONTIER    = 3;      // Min cells for a valid frontier cluster
const MAX_NO_FRONT    = 8;      // Ticks without frontier before declaring map complete
const CLEARANCE_SCAN_R = 6;     // Cells radius to measure clearance
const APPROACH_SEARCH_R = 20;   // Search radius to find widest goal point (2m)
const NAV_CHECK_MS    = 800;    // How often to check nav status
const NAV_TIMEOUT_MS  = 60000;  // Max time for one goal before giving up
const NAV_RECOVERY_MAX = 30000; // If stuck in RECOVERY modes > 30s, give up this goal

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
  waitingForNav = false;
  lastNavCheck = 0; navStartTime = 0; navRecoveryStart = 0;
  blacklist.clear();
  phase = 'selecting';
  console.log('[Explore v7] 🚀 Started — Goal-based frontier vacuum');
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
  console.log('[Explore v7] ⏹ Stopped');
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
      console.log('[Explore v7] ✅ Nav finished — selecting next frontier');

      // FIX: Permanently blacklist the frontier we just explored (radius 8 cells = 0.8m).
      // This prevents the robot from repeatedly picking "ghost" frontiers left behind 
      // inside walls or unreachable corners, stopping it from revisiting the same spot.
      if (currentGoalGrid) {
        _bl(currentGoalGrid.gx, currentGoalGrid.gy, 8);
      }

      _clearCurrentGoal();
      // Fall through to select next
    }
    // ── Case 2: Nav ERROR → blacklist this goal, try another ──
    else if (status === 'ERROR') {
      console.log('[Explore v7] ❌ Nav ERROR — blacklisting goal, trying next');
      _blacklistCurrentGoal();
      _stopNav();
      // Fall through to select next
    }
    // ── Case 3: Total timeout → this goal is unreachable ──
    else if (now - navStartTime > NAV_TIMEOUT_MS) {
      console.log(`[Explore v7] ⏰ Nav timeout (${(NAV_TIMEOUT_MS/1000)}s) — blacklisting, trying next`);
      _blacklistCurrentGoal();
      _stopNav();
      // Fall through to select next
    }
    // ── Case 4: Stuck in RECOVERY too long → give up this goal ──
    else if (status.startsWith('RECOVERY')) {
      if (navRecoveryStart === 0) navRecoveryStart = now;
      if (now - navRecoveryStart > NAV_RECOVERY_MAX) {
        console.log(`[Explore v7] 🔄 Recovery too long (${(NAV_RECOVERY_MAX/1000)}s) — blacklisting, trying next`);
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
      console.log('[Explore v7] 🎉 Map fully explored! No more frontiers.');
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

  // ── Score each cluster: prefer WIDE + CLOSE + HIGH INFO GAIN ──
  const rg = grid.worldToGrid(pose.x, pose.y);
  const scored = [];

  for (const cl of clusters) {
    // Find the WIDEST approach point near the frontier centroid
    const approach = _findWidestApproach(grid, cl.centroidGX, cl.centroidGY);
    if (!approach) continue;

    // Distance from robot (Euclidean in grid cells)
    const dist = Math.hypot(approach.gx - rg.gx, approach.gy - rg.gy);

    // If we are already extremely close to this frontier approach point (e.g. < 5 cells = 0.5m),
    // and it's STILL a frontier, it means our LiDAR cannot clear the unknown cells 
    // (they are inside a wall or physically unreachable).
    // Blacklist it so we don't get stuck in an infinite loop visiting the same spot.
    if (dist < 5.0) {
      console.log(`[Explore v7] ⚠️ Frontier too close (dist=${dist.toFixed(1)}). Blacklisting unreachable frontier.`);
      _bl(cl.centroidGX, cl.centroidGY, 8);
      continue;
    }

    // Info gain: how many unknown cells nearby
    const gain = _countUnknown(grid, approach.gx, approach.gy, 10);

    // Score: high clearance + high gain + low distance
    const score = approach.clearance * 5.0 + gain * 0.3 - dist * 0.8;

    const worldPt = grid.gridToWorld(approach.gx, approach.gy);
    scored.push({
      ...cl, score, dist, gain,
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

  console.log(`[Explore v7] 🎯 Goal: (${best.goalX.toFixed(2)}, ${best.goalY.toFixed(2)}) ` +
    `clearance=${best.clearance.toFixed(1)} gain=${best.gain} dist=${best.dist.toFixed(0)} ` +
    `score=${best.score.toFixed(1)}`);

  // ── Send as navigation GOAL ──
  currentGoalWorld = { x: best.goalX, y: best.goalY };
  currentGoalGrid = { gx: best.centroidGX, gy: best.centroidGY };
  currentGoalApproachGrid = { gx: best.approachGX, gy: best.approachGY };

  try {
    // Use the existing goal navigation system
    const navStore = await _ensureNavStore();
    if (!navStore) { phase = 'selecting'; return; }
    const navState = navStore.getState();

    // Stop any existing navigation first
    navState.navStopRobot(_robotId);

    // Small delay to let nav system clean up
    await new Promise(r => setTimeout(r, 50));

    const result = await navState.navigateToGoal(_robotId, best.goalX, best.goalY);

    if (result.success) {
      phase = 'navigating';
      waitingForNav = true;
      lastNavCheck = Date.now();
      navStartTime = Date.now();
      navRecoveryStart = 0;
      console.log(`[Explore v7] 📍 Nav started! Path: ${result.path?.length || '?'} waypoints`);
    } else {
      console.log(`[Explore v7] ❌ Nav failed: ${result.error} — blacklisting`);
      _bl(best.centroidGX, best.centroidGY);
      phase = 'selecting'; // Try another frontier next tick
    }
  } catch (err) {
    console.error('[Explore v7] Nav error:', err);
    _bl(best.centroidGX, best.centroidGY);
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
}

function _blacklistCurrentGoal() {
  if (currentGoalGrid) {
    _bl(currentGoalGrid.gx, currentGoalGrid.gy);
    console.log(`[Explore v7] 🚫 Blacklisted frontier at (${currentGoalGrid.gx}, ${currentGoalGrid.gy})`);
  }
  _clearCurrentGoal();
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

  // Search in a radius around the centroid
  const searchR = APPROACH_SEARCH_R;
  for (let dy = -searchR; dy <= searchR; dy++) {
    for (let dx = -searchR; dx <= searchR; dx++) {
      const gx = centGX + dx, gy = centGY + dy;
      if (gx < 1 || gx >= w - 1 || gy < 1 || gy >= h - 1) continue;
      if (!grid.isFree(gx, gy)) continue;

      const cost = grid.getCost(gx, gy);
      if (cost >= 150) continue; // Skip extremely dangerous cells

      // Measure clearance: distance to nearest obstacle in all 8 directions
      const clearance = _measureClearance(grid, gx, gy);

      // Score: lower cost (blue area = 0) is MUCH better. Higher clearance is better.
      const score = (clearance * 10) - (cost * 5);

      // Must be at least 2 cells clearance (robot width)
      if (clearance >= 2 && score > bestScore) {
        bestGX = gx; bestGY = gy; 
        bestScore = score;
        bestClearance = clearance;
        found = true;
      }
    }
  }

  if (!found) {
    // Fallback: Tìm một điểm FREE quanh centroid nhưng BẮT BUỘC phải an toàn (cost < 180)
    for (let r = 0; r <= 8; r++) { // r=0 chính là centroid
      for (let dy = -r; dy <= r; dy++) {
        for (let dx = -r; dx <= r; dx++) {
          // Chỉ check vòng ngoài cùng của bán kính r (để ưu tiên gần trước)
          if (r > 0 && Math.abs(dx) !== r && Math.abs(dy) !== r) continue;
          
          const nx = centGX + dx, ny = centGY + dy;
          if (nx >= 0 && nx < w && ny >= 0 && ny < h && grid.isFree(nx, ny)) {
            if (grid.getCost(nx, ny) < 180) { // Không được chui vào vùng đỏ/sát tường
              return { gx: nx, gy: ny, clearance: 1 };
            }
          }
        }
      }
    }
    return null; // Bỏ qua cụm frontier này nếu không tìm được điểm an toàn
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

/** Count unknown cells in a circular area */
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
