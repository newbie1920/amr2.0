/**
 * AMR 2.0 — Autonomous Exploration v2 (State Machine)
 * 
 * Tham khảo: ROS2 explore_lite + Yamauchi (1997) Frontier Exploration
 * 
 * State Machine:
 *   INIT_SPIN → FIND_FRONTIER → NAVIGATE → ARRIVED_SCAN
 *   → FIND_FRONTIER → ... → RECOVERY → COMPLETE
 * 
 * Cải tiến v2:
 *   - Proper flood-fill frontier clustering (O(n) thay vì O(n²))
 *   - Frontier scoring: information_gain + distance + size
 *   - Recovery behaviors: spin, backup, spiral
 *   - Mini-goal navigation (gửi waypoint ngắn, update liên tục)
 *   - Inflation costmap integration
 */

import { VEL_SOURCE } from './velocityMux.js';
import { navWorkerApi } from './navWorkerSetup.js';

// MapStore reference — set by mapStore.js to avoid circular import
let _mapStoreGetter = null;
export function _setMapStoreGetter(fn) { _mapStoreGetter = fn; }
function _getMapStore() {
  return _mapStoreGetter ? _mapStoreGetter() : {};
}

let workerBusy = false;

/**
 * Helper: gửi velocity qua VelocityMux (nếu có) hoặc trực tiếp
 * Exploration dùng priority NAVIGATION (10) — thấp hơn manual (50)
 */
function _sendVel(robotId, getStore, linear, angular) {
  const state = getStore();
  const mux = state.velocityMuxes?.[robotId];
  if (mux) {
    if (linear === 0 && angular === 0) {
      mux.release(VEL_SOURCE.NAVIGATION);
    } else {
      mux.send(VEL_SOURCE.NAVIGATION, linear, angular);
    }
  } else {
    // Fallback direct
    const robot = state.robots[robotId];
    if (robot?.connection?.connected) {
      robot.connection.sendVelocity(linear, angular);
    }
  }
}

// ============================================================
//   CONFIG
// ============================================================

const TICK_INTERVAL_MS = 500; // Phải < CMD_TIMEOUT_MS (1000ms) của ESP32 để keep-alive
const MIN_FRONTIER_SIZE = 3;
const SPIN_SPEED = 0.8;
const INIT_SPIN_MS = 5000;
const SCAN_SPIN_MS = 3000;
const FORWARD_SPEED = 0.15;
const MAX_NO_FRONTIER = 3;
const STUCK_TIMEOUT_MS = 6000;
const MINI_GOAL_DIST = 0.75; // Chỉ gửi waypoint 0.75m trước mặt

// ============================================================
//   EXPLORATION STATES
// ============================================================

const Phase = {
  IDLE: 'idle',
  INIT_SPIN: 'init_spin',
  FIND_FRONTIER: 'find_frontier',
  NAVIGATE: 'navigate',
  ARRIVED_SCAN: 'arrived_scan',
  RECOVERY_SPIN: 'recovery_spin',
  RECOVERY_BACKUP: 'recovery_backup',
  COMPLETE: 'complete',
};

// ============================================================
//   MODULE STATE
// ============================================================

let timer = null;
let active = false;
let phase = Phase.IDLE;
let phaseStart = 0;
let noFrontierCount = 0;
let lastRobotPos = { x: 0, y: 0 };
let stuckCheckTime = 0;
let recoveryCount = 0;
let currentTarget = null;
let allFrontierClusters = [];

// ============================================================
//   PUBLIC API
// ============================================================

export function startExploration(robotId, getStore) {
  if (active) return;
  active = true;
  noFrontierCount = 0;
  recoveryCount = 0;
  currentTarget = null;
  allFrontierClusters = [];

  _setPhase(Phase.INIT_SPIN);
  console.log('[Explore] 🚀 Bắt đầu tự khám phá!');

  // Xoay tại chỗ 360° đầu tiên
  _sendVel(robotId, getStore, 0, SPIN_SPEED);

  timer = setInterval(() => _tick(robotId, getStore), TICK_INTERVAL_MS);
}

export function stopExploration(robotId, getStore) {
  active = false;
  _setPhase(Phase.IDLE);
  currentTarget = null;

  if (timer) { clearInterval(timer); timer = null; }

  _sendVel(robotId, getStore, 0, 0);
  const state = getStore();
  const robot = state.robots[robotId];
  if (robot?.connection?.connected) {
    robot.connection.navStop();
  }
  console.log('[Explore] ⏹ Dừng khám phá');
}

export function isExplorationActive() { return active; }
export function getExplorationPhase() { return phase; }
export function getExplorationInfo() {
  return {
    phase,
    noFrontierCount,
    recoveryCount,
    currentTarget,
    clusterCount: allFrontierClusters.length,
  };
}

// ============================================================
//   STATE MACHINE TICK
// ============================================================

function _setPhase(p) {
  phase = p;
  phaseStart = Date.now();
}

function _tick(robotId, getStore) {
  if (!active) return;
  // workerBusy chỉ block các phase cần worker (FIND_FRONTIER, NAVIGATE)

  const robotState = getStore();
  const robot = robotState.robots[robotId];
  // Grid nằm ở mapStore, KHÔNG phải robotStore!
  const mapState = _getMapStore();
  const grid = mapState.mapperInstances?.[robotId];

  if (!robot?.connection?.connected) return;
  // Grid có thể chưa có dữ liệu ở INIT_SPIN, cho phép chạy spin mà không cần grid

  const telem = robot.telemetry || {};
  const rx = telem.x ?? 0;
  const ry = telem.y ?? 0;
  const elapsed = Date.now() - phaseStart;

  switch (phase) {
    // ── INIT SPIN: Xoay 360° tại chỗ lần đầu ──
    case Phase.INIT_SPIN:
      // Re-send velocity mỗi tick để chống ESP32 CMD_TIMEOUT (1s)
      _sendVel(robotId, getStore, 0, SPIN_SPEED);
      if (elapsed >= INIT_SPIN_MS) {
        _sendVel(robotId, getStore, 0, 0);
        console.log('[Explore] Init spin xong, tìm frontier...');
        _setPhase(Phase.FIND_FRONTIER);
      }
      break;

    // ── FIND FRONTIER ──
    case Phase.FIND_FRONTIER: {
      if (workerBusy || !grid) break; // Cần grid và worker rảnh
      // Inflate obstacles trước khi tìm đường (dynamic radius based on resolution)
      const inflCells = Math.ceil(0.55 / grid.resolution); // articubot: 0.55m
      grid.inflateObstacles(inflCells);

      // Tìm frontiers
      const clusters = _findFrontierClusters(grid);
      allFrontierClusters = clusters;

      // Cập nhật frontier cells cho visualization
      grid.frontierCells = [];
      for (const cl of clusters) {
        for (const c of cl.cells) grid.frontierCells.push(c);
      }

      if (clusters.length === 0) {
        noFrontierCount++;
        console.log(`[Explore] Không frontier (${noFrontierCount}/${MAX_NO_FRONTIER})`);
        if (noFrontierCount >= MAX_NO_FRONTIER) {
          if (recoveryCount < 2) {
            _setPhase(Phase.RECOVERY_SPIN);
            _sendVel(robotId, getStore, 0, SPIN_SPEED);
            recoveryCount++;
          } else {
            console.log('[Explore] ✅ Map quét xong!');
            _setPhase(Phase.COMPLETE);
            stopExploration(robotId, getStore);
          }
        }
        break;
      }

      noFrontierCount = 0;

      // Chọn frontier tốt nhất
      const best = _scoreFrontiers(clusters, grid, rx, ry);
      if (!best) break;

      // Tìm đường A*
      const goalWorld = grid.gridToWorld(best.approachGX, best.approachGY);
      workerBusy = true;
      navWorkerApi.findPath(grid.serialize(), rx, ry, goalWorld.x, goalWorld.y).then(result => {
        workerBusy = false;
        if (result.success && result.path.length > 1) {
          currentTarget = { gx: best.approachGX, gy: best.approachGY, ...goalWorld, path: result.path };
          console.log(`[Explore] 🎯 Frontier(${best.size} cells) → path ${result.path.length} WPs`);
          
          lastRobotPos = { x: rx, y: ry };
          stuckCheckTime = Date.now();
          _setPhase(Phase.NAVIGATE);
        } else {
          console.log('[Explore] Không tìm được đường, thử frontier khác...');
          noFrontierCount++;
        }
      }).catch(e => {
        workerBusy = false;
        console.error('[Explore] Worker Error:', e);
      });
      break;
    }

    // ── NAVIGATE: Đang di chuyển đến frontier (bằng DWA) ──
    case Phase.NAVIGATE: {
      if (!grid) break;
      // Kiểm tra stuck
      const moved = Math.hypot(rx - lastRobotPos.x, ry - lastRobotPos.y);
      if (moved > 0.05) {
        lastRobotPos = { x: rx, y: ry };
        stuckCheckTime = Date.now();
      } else if (Date.now() - stuckCheckTime > STUCK_TIMEOUT_MS) {
        console.log('[Explore] ⚠️ Robot stuck! Recovery...');
        _sendVel(robotId, getStore, 0, 0);
        _setPhase(Phase.RECOVERY_BACKUP);
        break;
      }

      // ── DWA LOCAL PLANNER ──
      if (workerBusy) break; // Chờ worker rảnh
      if (currentTarget && currentTarget.path && grid) {
        const pose = { x: rx, y: ry, theta: telem.heading * Math.PI / 180.0 };
        const vel = { v: telem.linearVel || 0, w: telem.angularVel || 0 };
        
        workerBusy = true;
        const dwaConfig = getStore().dwaConfig || null;
        navWorkerApi.computeVelocity(pose, vel, currentTarget.path, grid.serialize(), dwaConfig).then(cmd => {
          workerBusy = false;
          if (cmd.v === 0 && cmd.w === 0) {
            // Bị kẹt vật cản không qua được
            console.log(`[Explore] 🛑 DWA không tìm được quỹ đạo an toàn, re-planning...`);
            _sendVel(robotId, getStore, 0, 0);
            _setPhase(Phase.FIND_FRONTIER);
          } else {
            _sendVel(robotId, getStore, cmd.v, cmd.w);
          }
        }).catch(e => {
          workerBusy = false;
          console.error('[Explore] DWA Worker Error:', e);
        });

        // Kiểm tra xem đã đến đích chưa
        const distToGoal = Math.hypot(currentTarget.x - rx, currentTarget.y - ry);
        if (distToGoal < 0.2) {
          console.log('[Explore] Đã đến frontier bằng DWA!');
          _setPhase(Phase.ARRIVED_SCAN);
          _sendVel(robotId, getStore, 0, SPIN_SPEED * 0.7);
        }
      } else {
        _setPhase(Phase.FIND_FRONTIER);
      }
      break;
    }

    // ── ARRIVED SCAN: Quét thêm tại vị trí mới ──
    case Phase.ARRIVED_SCAN:
      // Re-send spin velocity mỗi tick để chống timeout
      _sendVel(robotId, getStore, 0, SPIN_SPEED * 0.7);
      if (elapsed >= SCAN_SPIN_MS) {
        _sendVel(robotId, getStore, 0, 0);
        _setPhase(Phase.FIND_FRONTIER);
      }
      break;

    // ── RECOVERY: Xoay tìm vùng mới ──
    case Phase.RECOVERY_SPIN:
      // Re-send spin velocity mỗi tick
      _sendVel(robotId, getStore, 0, SPIN_SPEED);
      if (elapsed >= INIT_SPIN_MS) {
        _sendVel(robotId, getStore, 0, 0);
        noFrontierCount = 0;
        _setPhase(Phase.FIND_FRONTIER);
      }
      break;

    // ── RECOVERY: Lùi lại ──
    case Phase.RECOVERY_BACKUP:
      // Re-send backup velocity mỗi tick
      if (elapsed < 1500) {
        _sendVel(robotId, getStore, -0.10, 0);
      } else {
        _sendVel(robotId, getStore, 0, 0);
        _setPhase(Phase.FIND_FRONTIER);
      }
      break;

    case Phase.COMPLETE:
      stopExploration(robotId, getStore);
      break;
  }
}

// ============================================================
//   FRONTIER DETECTION — Proper Flood-Fill Clustering
// ============================================================

function _findFrontierClusters(grid) {
  const w = grid.width, h = grid.height;

  // Bước 1: Tạo boolean frontier map
  const isFrontier = new Uint8Array(w * h);
  for (let gy = 1; gy < h - 1; gy++) {
    for (let gx = 1; gx < w - 1; gx++) {
      const lo = grid.logOdds[gy * w + gx];
      if (lo >= -0.3) continue; // Phải là FREE

      // Kiểm tra neighbor UNKNOWN
      let hasUnknown = false;
      for (let dy = -1; dy <= 1 && !hasUnknown; dy++) {
        for (let dx = -1; dx <= 1 && !hasUnknown; dx++) {
          if (dx === 0 && dy === 0) continue;
          const nlo = grid.logOdds[(gy + dy) * w + (gx + dx)];
          if (Math.abs(nlo) < 0.3) hasUnknown = true;
        }
      }
      if (hasUnknown) isFrontier[gy * w + gx] = 1;
    }
  }

  // Bước 2: Flood-fill clustering (O(n))
  const visited = new Uint8Array(w * h);
  const clusters = [];

  for (let gy = 0; gy < h; gy++) {
    for (let gx = 0; gx < w; gx++) {
      const idx = gy * w + gx;
      if (!isFrontier[idx] || visited[idx]) continue;

      // BFS flood-fill
      const cells = [];
      const queue = [idx];
      visited[idx] = 1;

      while (queue.length > 0) {
        const ci = queue.shift();
        const cx = ci % w;
        const cy = Math.floor(ci / w);
        cells.push({ gx: cx, gy: cy });

        // 8-connected neighbors
        for (let dy = -1; dy <= 1; dy++) {
          for (let dx = -1; dx <= 1; dx++) {
            if (dx === 0 && dy === 0) continue;
            const nx = cx + dx, ny = cy + dy;
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            const ni = ny * w + nx;
            if (isFrontier[ni] && !visited[ni]) {
              visited[ni] = 1;
              queue.push(ni);
            }
          }
        }
      }

      if (cells.length >= MIN_FRONTIER_SIZE) {
        // Centroid
        let sx = 0, sy = 0;
        for (const c of cells) { sx += c.gx; sy += c.gy; }
        clusters.push({
          cells,
          size: cells.length,
          centroidGX: Math.round(sx / cells.length),
          centroidGY: Math.round(sy / cells.length),
        });
      }
    }
  }

  return clusters;
}

// ============================================================
//   FRONTIER SCORING
// ============================================================

function _scoreFrontiers(clusters, grid, robotX, robotY) {
  const rg = grid.worldToGrid(robotX, robotY);
  let best = null;
  let bestScore = -Infinity;

  for (const cl of clusters) {
    // Tìm approach point: cell FREE trong cluster gần robot nhất
    let approachCell = cl.cells[0];
    let minDist = Infinity;
    for (const c of cl.cells) {
      const d = Math.hypot(c.gx - rg.gx, c.gy - rg.gy);
      if (d < minDist) { minDist = d; approachCell = c; }
    }

    // Tìm cell FREE gần approach (không đứng trên frontier)
    let approachGX = approachCell.gx;
    let approachGY = approachCell.gy;
    for (let dy = -2; dy <= 2; dy++) {
      for (let dx = -2; dx <= 2; dx++) {
        const nx = approachCell.gx + dx, ny = approachCell.gy + dy;
        if (grid.isFree(nx, ny) && grid.getCost(nx, ny) < 100) {
          approachGX = nx;
          approachGY = ny;
          dy = 3; // break outer
          break;
        }
      }
    }

    // Score = information_gain - travel_cost
    const travelCost = minDist * grid.resolution;
    const infoGain = cl.size;
    const score = 0.6 * infoGain - 0.4 * (travelCost * 10);

    if (score > bestScore) {
      bestScore = score;
      best = { ...cl, approachGX, approachGY, score };
    }
  }

  return best;
}

// ============================================================
//   MINI-GOAL EXTRACTION
// ============================================================

function _extractMiniGoal(path, maxDist) {
  if (path.length <= 2) return path;

  let totalDist = 0;
  const mini = [path[0]];

  for (let i = 1; i < path.length; i++) {
    const d = Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    totalDist += d;
    mini.push(path[i]);
    if (totalDist >= maxDist) break;
  }

  return mini;
}

export default { startExploration, stopExploration, isExplorationActive, getExplorationPhase, getExplorationInfo };
