/**
 * AMR 2.0 — Theta* Any-Angle Pathfinder on LiDAR Occupancy Grid
 * 
 * UPGRADE: Thay thế A* 8-hướng cũ bằng Theta* (any-angle pathfinding).
 * 
 * Ưu điểm so với A* cũ:
 *   - Đường đi tự do góc bất kỳ, không bị giới hạn 8 hướng grid
 *   - Ưu tiên hành lang rộng (exponential costmap penalty)
 *   - Line-of-Sight shortcutting loại bỏ waypoint thừa
 *   - Chaikin smoothing 6 iterations cho đường cong mượt mà
 * 
 * Tham khảo: Theta* (Nash et al. 2007) + ROS2 nav2_smac_planner
 */

import { ROBOT_HALF_WIDTH, ROBOT_HALF_LENGTH } from './warehouse.js';

// ============================================================
//   DATA STRUCTURES
// ============================================================

class MinHeap {
  constructor() { this.data = []; }
  push(item) { this.data.push(item); this._up(this.data.length - 1); }
  pop() {
    const top = this.data[0];
    const last = this.data.pop();
    if (this.data.length > 0) { this.data[0] = last; this._down(0); }
    return top;
  }
  get size() { return this.data.length; }
  _up(i) {
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.data[i].f < this.data[p].f) {
        [this.data[i], this.data[p]] = [this.data[p], this.data[i]];
        i = p;
      } else break;
    }
  }
  _down(i) {
    const n = this.data.length;
    while (true) {
      let s = i;
      const l = 2 * i + 1, r = 2 * i + 2;
      if (l < n && this.data[l].f < this.data[s].f) s = l;
      if (r < n && this.data[r].f < this.data[s].f) s = r;
      if (s !== i) { [this.data[i], this.data[s]] = [this.data[s], this.data[i]]; i = s; }
      else break;
    }
  }
}

// 8-connected neighbors
const DIRS = [
  { dx: 1, dy: 0, cost: 1.0 },
  { dx: -1, dy: 0, cost: 1.0 },
  { dx: 0, dy: 1, cost: 1.0 },
  { dx: 0, dy: -1, cost: 1.0 },
  { dx: 1, dy: 1, cost: 1.414 },
  { dx: -1, dy: 1, cost: 1.414 },
  { dx: 1, dy: -1, cost: 1.414 },
  { dx: -1, dy: -1, cost: 1.414 },
];

// ============================================================
//   HEURISTIC — Octile distance (admissible for 8-connected)
// ============================================================

function heuristic(x1, y1, x2, y2) {
  const dx = Math.abs(x2 - x1), dy = Math.abs(y2 - y1);
  return Math.max(dx, dy) + 0.414 * Math.min(dx, dy);
}

// ============================================================
//   LINE-OF-SIGHT CHECK (Bresenham)
// ============================================================

/**
 * Check if there is a clear line-of-sight between two grid cells.
 * Uses Bresenham's line algorithm to trace the line.
 * Returns true if the line is free of obstacles (considering costmap).
 * 
 * @param {number} x0 - Start grid X
 * @param {number} y0 - Start grid Y
 * @param {number} x1 - End grid X
 * @param {number} y1 - End grid Y
 * @param {object} grid - OccupancyGrid instance
 * @param {boolean} useCostmap - Whether to use costmap for checking
 * @param {number} safetyThreshold - Maximum cost allowed (default: 100, stay in blue zone)
 * @returns {boolean} true if line-of-sight is clear
 */
function lineOfSight(x0, y0, x1, y1, grid, useCostmap = true, safetyThreshold = 100) {
  let dx = Math.abs(x1 - x0);
  let dy = Math.abs(y1 - y0);
  const sx = x0 < x1 ? 1 : -1;
  const sy = y0 < y1 ? 1 : -1;
  let err = dx - dy;
  let x = x0, y = y0;

  // Robot footprint radius in cells — check a small buffer around the line
  const robotCells = Math.ceil(Math.max(ROBOT_HALF_WIDTH, ROBOT_HALF_LENGTH) / grid.resolution);

  while (true) {
    // Check a FULL SQUARE area around the line point (catches diagonal obstacles)
    for (let oy = -robotCells; oy <= robotCells; oy++) {
      for (let ox = -robotCells; ox <= robotCells; ox++) {
        const cx = x + ox;
        const cy = y + oy;
        if (!grid.inBounds(cx, cy)) continue;

        if (useCostmap && grid.costmap) {
          const cost = grid.costmap[cy * grid.width + cx];
          if (cost >= safetyThreshold) return false;
        } else {
          if (grid.logOdds[cy * grid.width + cx] > 0.3) return false;
        }
      }
    }

    if (x === x1 && y === y1) break;
    const e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 < dx) { err += dx; y += sy; }
  }
  return true;
}

// ============================================================
//   THETA* PATHFINDING (Any-Angle)
// ============================================================

/**
 * Theta* pathfinding on OccupancyGrid.
 * 
 * Theta* extends A* by allowing any-angle parent connections via 
 * line-of-sight checks. This produces much smoother and shorter paths
 * than grid-restricted A*, especially in open environments.
 * 
 * Additional features:
 *   - Exponential costmap penalty: strongly repels paths from obstacles
 *   - Corridor-width scoring: prefers wider passages
 *   - Line-of-sight shortcutting: eliminates unnecessary waypoints
 * 
 * @param {OccupancyGrid} grid - Grid instance
 * @param {number} startX - World X (meters)
 * @param {number} startY - World Y (meters)
 * @param {number} goalX - World X (meters)
 * @param {number} goalY - World Y (meters)
 * @param {object} options
 * @returns {{ path: Array<{x,y}>, success: boolean, gridPath: Array<{gx,gy}> }}
 */
export function findPathOnGrid(grid, startX, startY, goalX, goalY, options = {}) {
  const {
    allowUnknown = true,
    maxIterations = 15000,
    useCostmap = true,
  } = options;

  const sg = grid.worldToGrid(startX, startY);
  const eg = grid.worldToGrid(goalX, goalY);

  // Clamp to grid bounds
  sg.gx = Math.max(0, Math.min(grid.width - 1, sg.gx));
  sg.gy = Math.max(0, Math.min(grid.height - 1, sg.gy));
  eg.gx = Math.max(0, Math.min(grid.width - 1, eg.gx));
  eg.gy = Math.max(0, Math.min(grid.height - 1, eg.gy));

  // If goal is blocked → find nearest free cell
  if (grid.isOccupied(eg.gx, eg.gy)) {
    const free = _findNearestPassable(grid, eg.gx, eg.gy, allowUnknown);
    if (free) { eg.gx = free.gx; eg.gy = free.gy; }
    else return { path: [], success: false, gridPath: [] };
  }

  const w = grid.width;
  const h = grid.height;
  const openSet = new MinHeap();
  const gScore = new Float32Array(w * h);
  gScore.fill(Infinity);
  const cameFrom = new Int32Array(w * h);
  cameFrom.fill(-1);
  const closed = new Uint8Array(w * h);

  const startIdx = sg.gy * w + sg.gx;
  gScore[startIdx] = 0;
  openSet.push({ gx: sg.gx, gy: sg.gy, f: heuristic(sg.gx, sg.gy, eg.gx, eg.gy) });

  let iterations = 0;

  while (openSet.size > 0 && iterations < maxIterations) {
    iterations++;
    const curr = openSet.pop();
    const cIdx = curr.gy * w + curr.gx;

    if (closed[cIdx]) continue;
    closed[cIdx] = 1;

    // Goal reached (±1 cell tolerance)
    if (Math.abs(curr.gx - eg.gx) <= 1 && Math.abs(curr.gy - eg.gy) <= 1) {
      eg.gx = curr.gx;
      eg.gy = curr.gy;

      // ── RECONSTRUCT PATH ──
      const gridPath = [];
      let idx = curr.gy * w + curr.gx;
      while (idx >= 0) {
        const gy = Math.floor(idx / w);
        const gx = idx % w;
        gridPath.unshift({ gx, gy });
        idx = cameFrom[idx];
      }

      // Convert to world coordinates
      let worldPath = gridPath.map(p => grid.gridToWorld(p.gx, p.gy));

      // ── POST-PROCESSING PIPELINE ──
      // 1) Line-of-Sight shortcutting (remove unnecessary waypoints)
      worldPath = _losShortcut(worldPath, grid, useCostmap);

      // 2) RDP simplification (keep key turning points, aggressive epsilon)
      const simplified = _rdpSimplify(worldPath, 0.08);

      // 3) Chaikin B-Spline smoothing (3 iterations — smooth but controlled WP count)
      const smoothed = _smoothPath(simplified, grid);

      // 4) Cap waypoints to prevent DWA overload (max 60 WP)
      const capped = _limitWaypoints(smoothed, 60);

      // Ensure start is exact; only set goal if it's safe
      if (capped.length > 0) {
        capped[0] = { x: startX, y: startY };
        // BUG #1 FIX: Only overwrite goal if the original goal cell is safe.
        // If Theta* adjusted the goal (obstacle), keep the adjusted endpoint.
        const goalG = grid.worldToGrid(goalX, goalY);
        let goalSafe = true;
        if (grid.costmap && grid.inBounds(goalG.gx, goalG.gy)) {
          if (grid.costmap[goalG.gy * grid.width + goalG.gx] >= 100) goalSafe = false;
        }
        if (grid.isOccupied && grid.isOccupied(goalG.gx, goalG.gy)) goalSafe = false;
        if (goalSafe) {
          capped[capped.length - 1] = { x: goalX, y: goalY };
        }
      }

      return { path: capped, success: true, gridPath };
    }

    // ── EXPAND NEIGHBORS (Theta* style) ──
    for (const dir of DIRS) {
      const nx = curr.gx + dir.dx;
      const ny = curr.gy + dir.dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

      const nIdx = ny * w + nx;
      if (closed[nIdx]) continue;

      // Passability check
      const lo = grid.logOdds[nIdx];
      if (lo > 0.3) continue; // Occupied
      if (!allowUnknown && Math.abs(lo) <= 0.3) continue; // Unknown

      // Diagonal corner-cutting safety
      if (dir.dx !== 0 && dir.dy !== 0) {
        const lo1 = grid.getLogOdds(curr.gx + dir.dx, curr.gy);
        const lo2 = grid.getLogOdds(curr.gx, curr.gy + dir.dy);
        if (lo1 > 0.3 || lo2 > 0.3) continue;
      }

      // ── COSTMAP PENALTY (Exponential — strongly repels from obstacles) ──
      // Threshold 100: BLOCK any cell approaching the pink zone on the costmap.
      // This ensures the path stays firmly in the blue (safe) corridor.
      let costPenalty = 0;
      if (useCostmap && grid.costmap) {
        const cm = grid.costmap[nIdx];
        if (cm >= 100) continue;          // Approaching pink zone — BLOCKED
        if (cm > 0) {
          // Very steep exponential penalty: pushes path to center of corridors.
          // cm=20 → penalty ~1.7,  cm=40 → penalty ~6.4,  cm=60 → penalty ~19.1,  cm=99 → penalty ~140
          const normalized = cm / 100;
          costPenalty = Math.exp(normalized * 5.0) - 1.0;  // Range: 0 to ~147
        }
      }
      // Unknown cells penalty
      if (Math.abs(lo) <= 0.3) {
        costPenalty += 3.0;
      }

      // ── THETA* LINE-OF-SIGHT: Try connecting to grandparent ──
      // Instead of only connecting curr → neighbor, try parent(curr) → neighbor.
      // If line-of-sight is clear, skip curr entirely → any-angle path!
      const parentIdx = cameFrom[cIdx];
      let bestG = Infinity;
      let bestParentIdx = -1;

      if (parentIdx >= 0) {
        const parentGX = parentIdx % w;
        const parentGY = Math.floor(parentIdx / w);

        // Check line-of-sight from parent to neighbor
        if (lineOfSight(parentGX, parentGY, nx, ny, grid, useCostmap, 100)) {
          // Direct path from grandparent → neighbor
          const directDist = Math.hypot(nx - parentGX, ny - parentGY);
          const directG = gScore[parentIdx] + directDist + costPenalty * directDist / Math.max(1, Math.hypot(dir.dx, dir.dy));
          if (directG < bestG) {
            bestG = directG;
            bestParentIdx = parentIdx;
          }
        }
      }

      // Standard A* path: curr → neighbor
      const standardG = gScore[cIdx] + dir.cost + costPenalty;
      if (standardG < bestG) {
        bestG = standardG;
        bestParentIdx = cIdx;
      }

      if (bestG < gScore[nIdx]) {
        gScore[nIdx] = bestG;
        cameFrom[nIdx] = bestParentIdx;
        openSet.push({ gx: nx, gy: ny, f: bestG + heuristic(nx, ny, eg.gx, eg.gy) });
      }
    }
  }

  return { path: [], success: false, gridPath: [] };
}

// ============================================================
//   LINE-OF-SIGHT PATH SHORTCUTTING (Post-processing)
// ============================================================

/**
 * Remove unnecessary waypoints by checking line-of-sight between
 * non-adjacent points. If we can go directly from A to C, remove B.
 * Uses greedy forward scanning for efficiency.
 */
function _losShortcut(path, grid, useCostmap) {
  if (path.length <= 2) return path;
  
  const result = [path[0]];
  let current = 0;

  while (current < path.length - 1) {
    // Try to skip as far ahead as possible
    let farthest = current + 1;
    
    for (let ahead = path.length - 1; ahead > current + 1; ahead--) {
      const fromG = grid.worldToGrid(path[current].x, path[current].y);
      const toG = grid.worldToGrid(path[ahead].x, path[ahead].y);
      
      if (lineOfSight(fromG.gx, fromG.gy, toG.gx, toG.gy, grid, useCostmap, 100)) {
        farthest = ahead;
        break;
      }
    }

    current = farthest;
    result.push(path[current]);
  }

  return result;
}

// ============================================================
//   HELPERS
// ============================================================

function _findNearestPassable(grid, gx, gy, allowUnknown) {
  const queue = [{ gx, gy }];
  const visited = new Set();
  visited.add(`${gx},${gy}`);
  let maxIter = 500;

  while (queue.length > 0 && maxIter-- > 0) {
    const c = queue.shift();
    const lo = grid.getLogOdds(c.gx, c.gy);
    if (lo < -0.3 || (allowUnknown && Math.abs(lo) <= 0.3)) {
      // Also check costmap — don't return inscribed cells
      if (grid.costmap) {
        const cost = grid.costmap[c.gy * grid.width + c.gx];
        if (cost >= 100) {
          // This cell is too close to an obstacle, keep searching
        } else {
          return c;
        }
      } else {
        return c;
      }
    }
    for (const { dx, dy } of [{ dx: 1, dy: 0 }, { dx: -1, dy: 0 }, { dx: 0, dy: 1 }, { dx: 0, dy: -1 }]) {
      const nx = c.gx + dx, ny = c.gy + dy;
      const key = `${nx},${ny}`;
      if (!visited.has(key) && grid.inBounds(nx, ny)) {
        visited.add(key);
        queue.push({ gx: nx, gy: ny });
      }
    }
  }
  return null;
}

// ── RDP SIMPLIFICATION ──

function _rdpSimplify(points, epsilon) {
  if (points.length <= 2) return [...points];
  let maxDist = 0, maxIdx = 0;
  const s = points[0], e = points[points.length - 1];
  for (let i = 1; i < points.length - 1; i++) {
    const d = _ptLineDist(points[i], s, e);
    if (d > maxDist) { maxDist = d; maxIdx = i; }
  }
  if (maxDist > epsilon) {
    const left = _rdpSimplify(points.slice(0, maxIdx + 1), epsilon);
    const right = _rdpSimplify(points.slice(maxIdx), epsilon);
    return left.slice(0, -1).concat(right);
  }
  return [s, e];
}

function _ptLineDist(p, a, b) {
  const dx = b.x - a.x, dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  if (len2 === 0) return Math.hypot(p.x - a.x, p.y - a.y);
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

// ── PATH SMOOTHING (Chaikin B-Spline — Collision-Aware) ──

function _smoothPath(path, grid) {
  if (path.length <= 2) return path;

  let smoothed = [...path];
  const iterations = 3; // 3 iterations = smooth curves without too many waypoints

  for (let iter = 0; iter < iterations; iter++) {
    const newPath = [];
    newPath.push(smoothed[0]); // Keep start point

    for (let i = 0; i < smoothed.length - 1; i++) {
      const p0 = smoothed[i];
      const p1 = smoothed[i + 1];

      // Chaikin corner-cutting points (75/25 and 25/75)
      const q = { x: 0.75 * p0.x + 0.25 * p1.x, y: 0.75 * p0.y + 0.25 * p1.y };
      const r = { x: 0.25 * p0.x + 0.75 * p1.x, y: 0.25 * p0.y + 0.75 * p1.y };

      // Check if the new points clip into obstacles
      let qBlocked = false;
      let rBlocked = false;

      if (grid.costmap) {
        const qg = grid.worldToGrid(q.x, q.y);
        if (grid.inBounds(qg.gx, qg.gy)) {
          const cost = grid.costmap[qg.gy * grid.width + qg.gx];
          if (cost >= 100) qBlocked = true; // Stay in blue: reject any cell approaching pink zone
        }
        const rg = grid.worldToGrid(r.x, r.y);
        if (grid.inBounds(rg.gx, rg.gy)) {
          const cost = grid.costmap[rg.gy * grid.width + rg.gx];
          if (cost >= 100) rBlocked = true; // Stay in blue zone
        }
      } else {
        const qg = grid.worldToGrid(q.x, q.y);
        if (grid.inBounds(qg.gx, qg.gy) && grid.getLogOdds(qg.gx, qg.gy) > 0.3) qBlocked = true;
        const rg = grid.worldToGrid(r.x, r.y);
        if (grid.inBounds(rg.gx, rg.gy) && grid.getLogOdds(rg.gx, rg.gy) > 0.3) rBlocked = true;
      }

      // Also check the midpoint between q and r (catches edge cases)
      if (!qBlocked && !rBlocked) {
        const mid = { x: (q.x + r.x) / 2, y: (q.y + r.y) / 2 };
        const mg = grid.worldToGrid(mid.x, mid.y);
        if (grid.inBounds(mg.gx, mg.gy)) {
          if (grid.costmap) {
            const midCost = grid.costmap[mg.gy * grid.width + mg.gx];
            if (midCost >= 100) {
              qBlocked = true;
              rBlocked = true;
            }
          } else {
            if (grid.getLogOdds(mg.gx, mg.gy) > 0.3) {
              qBlocked = true;
              rBlocked = true;
            }
          }
        }
      }

      // If corner cutting clips obstacle → keep original sharp corner
      if (qBlocked || rBlocked) {
        newPath.push(p1);
      } else {
        newPath.push(q);
        newPath.push(r);
      }
    }
    
    newPath.push(smoothed[smoothed.length - 1]); // Keep goal point
    smoothed = newPath;
  }

  return smoothed;
}

/**
 * Downsample path to a maximum number of waypoints.
 * BUG #7 FIX: Uses curvature-aware sampling — high-curvature (turning) points
 * are preserved while straight segments are aggressively simplified.
 */
function _limitWaypoints(path, maxWaypoints) {
  if (path.length <= maxWaypoints) return path;
  
  // Calculate curvature score at each point
  const scores = new Float32Array(path.length);
  scores[0] = Infinity;  // Always keep start
  scores[path.length - 1] = Infinity;  // Always keep end
  
  for (let i = 1; i < path.length - 1; i++) {
    const prev = path[i - 1], curr = path[i], next = path[i + 1];
    const dx1 = curr.x - prev.x, dy1 = curr.y - prev.y;
    const dx2 = next.x - curr.x, dy2 = next.y - curr.y;
    const len1 = Math.hypot(dx1, dy1), len2 = Math.hypot(dx2, dy2);
    if (len1 < 1e-6 || len2 < 1e-6) { scores[i] = 0; continue; }
    // Angle change (curvature proxy)
    const dot = (dx1 * dx2 + dy1 * dy2) / (len1 * len2);
    scores[i] = 1.0 - Math.max(-1, Math.min(1, dot)); // 0 = straight, 2 = U-turn
  }
  
  // Build index-score pairs and sort by importance (descending)
  const indexed = [];
  for (let i = 0; i < path.length; i++) indexed.push({ i, s: scores[i] });
  indexed.sort((a, b) => b.s - a.s);
  
  // Take top maxWaypoints by importance, then sort by index to preserve order
  const selected = indexed.slice(0, maxWaypoints);
  selected.sort((a, b) => a.i - b.i);
  
  return selected.map(s => path[s.i]);
}

export default findPathOnGrid;
