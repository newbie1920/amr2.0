/**
 * AMR 2.0 — A* Pathfinder on LiDAR Occupancy Grid
 * 
 * Tách riêng khỏi warehouse pathfinder (pathfinder.js).
 * Chạy trên OccupancyGrid logOdds + inflation costmap.
 * 
 * Tham khảo: ROS2 nav2_navfn_planner (Dijkstra/A*)
 */

// ============================================================
//   A* ON OCCUPANCY GRID
// ============================================================

/**
 * MinHeap for A*
 */
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

function heuristic(x1, y1, x2, y2) {
  const dx = Math.abs(x2 - x1), dy = Math.abs(y2 - y1);
  return Math.max(dx, dy) + 0.414 * Math.min(dx, dy);
}

/**
 * A* pathfinding trên OccupancyGrid
 * 
 * @param {OccupancyGrid} grid - Grid instance
 * @param {number} startX - World X (meters)
 * @param {number} startY - World Y (meters)
 * @param {number} goalX - World X (meters)
 * @param {number} goalY - World Y (meters)
 * @param {object} options
 * @param {boolean} options.allowUnknown - Cho đi qua vùng unknown (default: true)
 * @param {number} options.maxIterations - Giới hạn iterations (default: 10000)
 * @param {boolean} options.useCostmap - Sử dụng inflation costmap (default: true)
 * @returns {{ path: Array<{x,y}>, success: boolean, gridPath: Array<{gx,gy}> }}
 */
export function findPathOnGrid(grid, startX, startY, goalX, goalY, options = {}) {
  const {
    allowUnknown = true,
    maxIterations = 10000,
    useCostmap = true,
  } = options;

  const sg = grid.worldToGrid(startX, startY);
  const eg = grid.worldToGrid(goalX, goalY);

  // Clamp
  sg.gx = Math.max(0, Math.min(grid.width - 1, sg.gx));
  sg.gy = Math.max(0, Math.min(grid.height - 1, sg.gy));
  eg.gx = Math.max(0, Math.min(grid.width - 1, eg.gx));
  eg.gy = Math.max(0, Math.min(grid.height - 1, eg.gy));

  // Nếu goal blocked → tìm free cell gần nhất
  if (grid.isOccupied(eg.gx, eg.gy)) {
    const free = _findNearestPassable(grid, eg.gx, eg.gy, allowUnknown);
    if (free) { eg.gx = free.gx; eg.gy = free.gy; }
    else return { path: [], success: false, gridPath: [] };
  }

  const w = grid.width;
  const openSet = new MinHeap();
  const gScore = new Float32Array(w * grid.height);
  gScore.fill(Infinity);
  const cameFrom = new Int32Array(w * grid.height);
  cameFrom.fill(-1);
  const closed = new Uint8Array(w * grid.height);

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

    // Đã đến đích (±1 cell)
    if (Math.abs(curr.gx - eg.gx) <= 1 && Math.abs(curr.gy - eg.gy) <= 1) {
      eg.gx = curr.gx;
      eg.gy = curr.gy;

      // Reconstruct
      const gridPath = [];
      let idx = curr.gy * w + curr.gx;
      while (idx >= 0) {
        const gy = Math.floor(idx / w);
        const gx = idx % w;
        gridPath.unshift({ gx, gy });
        idx = cameFrom[idx];
      }

      // 2) Chuyển đổi sang tọa độ World (mét)
      let worldPath = gridPath.map(p => grid.gridToWorld(p.gx, p.gy));

      // 3) Gradient Descent Smoothing (Làm mượt đường đi)
      worldPath = _smoothPath(worldPath, grid);

      // 4) Đơn giản hóa đường (bỏ các điểm nằm trên đường thẳng)
      const simplified = _rdpSimplify(worldPath, 0.05);

      // 5) Giới hạn số lượng waypoint
      const limited = _limitWaypoints(simplified, 20);

      // Đảm bảo điểm đầu/cuối chính xác
      if (limited.length > 0) {
        limited[0] = { x: startX, y: startY };
        limited[limited.length - 1] = { x: goalX, y: goalY };
      }

      return { path: limited, success: true, gridPath };
    }

    for (const dir of DIRS) {
      const nx = curr.gx + dir.dx;
      const ny = curr.gy + dir.dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= grid.height) continue;

      const nIdx = ny * w + nx;
      if (closed[nIdx]) continue;

      // Check passability
      const lo = grid.logOdds[nIdx];
      if (lo > 0.3) continue; // Occupied
      if (!allowUnknown && Math.abs(lo) <= 0.3) continue; // Unknown

      // Diagonal corner-cutting check
      if (dir.dx !== 0 && dir.dy !== 0) {
        const lo1 = grid.getLogOdds(curr.gx + dir.dx, curr.gy);
        const lo2 = grid.getLogOdds(curr.gx, curr.gy + dir.dy);
        if (lo1 > 0.3 || lo2 > 0.3) continue;
      }

      // Cost: base + costmap penalty
      let moveCost = dir.cost;
      if (useCostmap) {
        const cm = grid.costmap[nIdx];
        moveCost += cm * 0.02; // Soft penalty near obstacles
      }
      // Unknown cells have extra penalty
      if (Math.abs(lo) <= 0.3) {
        moveCost += 2.0;
      }

      const tentG = gScore[cIdx] + moveCost;
      if (tentG < gScore[nIdx]) {
        gScore[nIdx] = tentG;
        cameFrom[nIdx] = cIdx;
        openSet.push({ gx: nx, gy: ny, f: tentG + heuristic(nx, ny, eg.gx, eg.gy) });
      }
    }
  }

  return { path: [], success: false, gridPath: [] };
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
      return c;
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

function _limitWaypoints(path, maxWP) {
  if (path.length <= maxWP) return path;
  const step = Math.ceil(path.length / (maxWP - 1));
  const result = [path[0]];
  for (let i = step; i < path.length - 1; i += step) {
    result.push(path[i]);
  }
  result.push(path[path.length - 1]);
  return result;
}

// ── PATH SMOOTHING ──────────────────────────────────────────
function _smoothPath(path, grid, weight_data = 0.5, weight_smooth = 0.3, tolerance = 0.001) {
  if (path.length <= 2) return path;

  const newPath = path.map(p => ({ x: p.x, y: p.y }));
  let change = tolerance;
  let maxIter = 100;

  while (change >= tolerance && maxIter-- > 0) {
    change = 0.0;
    for (let i = 1; i < path.length - 1; i++) {
      const auxX = newPath[i].x;
      const auxY = newPath[i].y;

      newPath[i].x += weight_data * (path[i].x - newPath[i].x) +
                      weight_smooth * (newPath[i - 1].x + newPath[i + 1].x - 2.0 * newPath[i].x);
      newPath[i].y += weight_data * (path[i].y - newPath[i].y) +
                      weight_smooth * (newPath[i - 1].y + newPath[i + 1].y - 2.0 * newPath[i].y);

      // Check collision on the grid
      const gPos = grid.worldToGrid(newPath[i].x, newPath[i].y);
      if (grid.getLogOdds(gPos.gx, gPos.gy) > 0.3 || (grid.costmap && grid.getCost && grid.getCost(gPos.gx, gPos.gy) >= 200)) {
        // Revert if smoothing pushes path into obstacle
        newPath[i].x = auxX;
        newPath[i].y = auxY;
      } else {
        change += Math.abs(auxX - newPath[i].x) + Math.abs(auxY - newPath[i].y);
      }
    }
  }
  return newPath;
}

export default findPathOnGrid;
