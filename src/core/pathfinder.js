/**
 * AMR 2.0 — A* Pathfinding Algorithm
 * Thuật toán tìm đường A* trên lưới occupancy grid
 * với Ramer-Douglas-Peucker path smoothing
 */

import {
  GRID_COLS,
  GRID_ROWS,
  GRID_CELL_SIZE,
  createOccupancyGrid,
  meterToGrid,
  gridToMeter,
} from './warehouse.js';

// ============================================================
//   A* PATHFINDING
// ============================================================

/**
 * MinHeap (Priority Queue) cho A* — tối ưu hiệu năng
 */
class MinHeap {
  constructor() {
    this.data = [];
  }

  push(item) {
    this.data.push(item);
    this._bubbleUp(this.data.length - 1);
  }

  pop() {
    const top = this.data[0];
    const last = this.data.pop();
    if (this.data.length > 0) {
      this.data[0] = last;
      this._sinkDown(0);
    }
    return top;
  }

  get size() {
    return this.data.length;
  }

  _bubbleUp(i) {
    while (i > 0) {
      const parent = Math.floor((i - 1) / 2);
      if (this.data[i].f < this.data[parent].f) {
        [this.data[i], this.data[parent]] = [this.data[parent], this.data[i]];
        i = parent;
      } else break;
    }
  }

  _sinkDown(i) {
    const n = this.data.length;
    while (true) {
      let smallest = i;
      const left = 2 * i + 1;
      const right = 2 * i + 2;
      if (left < n && this.data[left].f < this.data[smallest].f) smallest = left;
      if (right < n && this.data[right].f < this.data[smallest].f) smallest = right;
      if (smallest !== i) {
        [this.data[i], this.data[smallest]] = [this.data[smallest], this.data[i]];
        i = smallest;
      } else break;
    }
  }
}

/**
 * 8 hướng di chuyển (bao gồm chéo)
 * dx, dy, cost (chéo = √2)
 */
const DIRECTIONS = [
  { dx: 0, dy: 1, cost: 1.0 },     // Up
  { dx: 0, dy: -1, cost: 1.0 },    // Down
  { dx: 1, dy: 0, cost: 1.0 },     // Right
  { dx: -1, dy: 0, cost: 1.0 },    // Left
  { dx: 1, dy: 1, cost: 1.414 },   // Up-Right
  { dx: -1, dy: 1, cost: 1.414 },  // Up-Left
  { dx: 1, dy: -1, cost: 1.414 },  // Down-Right
  { dx: -1, dy: -1, cost: 1.414 }, // Down-Left
];

/**
 * Heuristic: Octile distance (tốt hơn Euclidean cho 8 hướng)
 */
function heuristic(col1, row1, col2, row2) {
  const dx = Math.abs(col2 - col1);
  const dy = Math.abs(row2 - row1);
  return Math.max(dx, dy) + (1.414 - 1) * Math.min(dx, dy);
}

/**
 * A* Pathfinding
 * @param {number} startX - Vị trí bắt đầu (mét)
 * @param {number} startY
 * @param {number} goalX  - Vị trí đích (mét)
 * @param {number} goalY
 * @param {number[][]} [grid] - Occupancy grid (tự tạo nếu không truyền)
 * @returns {{path: {x: number, y: number}[], success: boolean, gridPath: {col: number, row: number}[]}}
 */
export function findPath(startX, startY, goalX, goalY, grid = null) {
  if (!grid) grid = createOccupancyGrid();

  const start = meterToGrid(startX, startY);
  const goal = meterToGrid(goalX, goalY);

  // Kiểm tra vị trí hợp lệ
  if (
    start.col < 0 || start.col >= GRID_COLS ||
    start.row < 0 || start.row >= GRID_ROWS ||
    goal.col < 0 || goal.col >= GRID_COLS ||
    goal.row < 0 || goal.row >= GRID_ROWS
  ) {
    console.error('[Pathfinder] Vị trí ngoài biên kho!');
    return { path: [], success: false, gridPath: [] };
  }

  // Nếu đích là obstacle, tìm ô trống gần nhất
  if (grid[goal.row][goal.col] === 1) {
    const nearest = findNearestFreeCell(grid, goal.col, goal.row);
    if (nearest) {
      goal.col = nearest.col;
      goal.row = nearest.row;
    } else {
      console.error('[Pathfinder] Đích bị chặn, không tìm được ô trống!');
      return { path: [], success: false, gridPath: [] };
    }
  }

  // A* algorithm
  const openSet = new MinHeap();
  const gScore = new Map();
  const cameFrom = new Map();
  const closedSet = new Set();

  const startKey = `${start.col},${start.row}`;
  const goalKey = `${goal.col},${goal.row}`;

  gScore.set(startKey, 0);
  openSet.push({
    col: start.col,
    row: start.row,
    f: heuristic(start.col, start.row, goal.col, goal.row),
  });

  while (openSet.size > 0) {
    const current = openSet.pop();
    const currentKey = `${current.col},${current.row}`;

    if (currentKey === goalKey) {
      // Reconstruct path
      const gridPath = [];
      let key = goalKey;
      while (key) {
        const [c, r] = key.split(',').map(Number);
        gridPath.unshift({ col: c, row: r });
        key = cameFrom.get(key);
      }

      // Convert grid path → meter path
      const meterPath = gridPath.map(p => gridToMeter(p.col, p.row));

      // Smooth path
      const smoothed = smoothPath(meterPath);

      // Đảm bảo điểm đầu và cuối chính xác
      if (smoothed.length > 0) {
        smoothed[0] = { x: startX, y: startY };
        smoothed[smoothed.length - 1] = { x: goalX, y: goalY };
      }

      return { path: smoothed, success: true, gridPath };
    }

    closedSet.add(currentKey);

    for (const dir of DIRECTIONS) {
      const nc = current.col + dir.dx;
      const nr = current.row + dir.dy;
      const neighborKey = `${nc},${nr}`;

      if (nc < 0 || nc >= GRID_COLS || nr < 0 || nr >= GRID_ROWS) continue;
      if (grid[nr][nc] === 1) continue;
      if (closedSet.has(neighborKey)) continue;

      // Kiểm tra corner-cutting cho đường chéo
      if (dir.dx !== 0 && dir.dy !== 0) {
        if (grid[current.row][current.col + dir.dx] === 1 ||
            grid[current.row + dir.dy][current.col] === 1) {
          continue;
        }
      }

      const tentativeG = (gScore.get(currentKey) || 0) + dir.cost;

      if (tentativeG < (gScore.get(neighborKey) ?? Infinity)) {
        cameFrom.set(neighborKey, currentKey);
        gScore.set(neighborKey, tentativeG);
        const f = tentativeG + heuristic(nc, nr, goal.col, goal.row);
        openSet.push({ col: nc, row: nr, f });
      }
    }
  }

  console.error('[Pathfinder] Không tìm được đường đi!');
  return { path: [], success: false, gridPath: [] };
}

// ============================================================
//   PATH SMOOTHING — Ramer-Douglas-Peucker
// ============================================================

/**
 * Ramer-Douglas-Peucker algorithm
 * Giảm số điểm trên path mà vẫn giữ hình dạng
 * @param {Array<{x, y}>} points
 * @param {number} epsilon - Sai số cho phép (mét)
 * @returns {Array<{x, y}>}
 */
function rdpSimplify(points, epsilon = 0.1) {
  if (points.length <= 2) return points;

  // Tìm điểm xa nhất so với đoạn thẳng start-end
  let maxDist = 0;
  let maxIdx = 0;

  const start = points[0];
  const end = points[points.length - 1];

  for (let i = 1; i < points.length - 1; i++) {
    const d = pointToLineDistance(points[i], start, end);
    if (d > maxDist) {
      maxDist = d;
      maxIdx = i;
    }
  }

  if (maxDist > epsilon) {
    const left = rdpSimplify(points.slice(0, maxIdx + 1), epsilon);
    const right = rdpSimplify(points.slice(maxIdx), epsilon);
    return left.slice(0, -1).concat(right);
  } else {
    return [start, end];
  }
}

/**
 * Khoảng cách từ điểm đến đoạn thẳng
 */
function pointToLineDistance(point, lineStart, lineEnd) {
  const dx = lineEnd.x - lineStart.x;
  const dy = lineEnd.y - lineStart.y;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return Math.hypot(point.x - lineStart.x, point.y - lineStart.y);

  const t = Math.max(0, Math.min(1,
    ((point.x - lineStart.x) * dx + (point.y - lineStart.y) * dy) / lenSq
  ));

  const projX = lineStart.x + t * dx;
  const projY = lineStart.y + t * dy;
  return Math.hypot(point.x - projX, point.y - projY);
}

/**
 * Chaikin's Corner Cutting Algorithm
 * Giúp mài tròn các góc vuông thành đường cong mượt mà, không lấn dải phân cách
 * @param {Array<{x, y}>} points
 * @param {number} iterations - Số lần cắt góc (Độ mượt)
 * @returns {Array<{x, y}>}
 */
function chaikinSmooth(points, iterations = 3) {
  if (points.length <= 2) return points;
  let currentPath = points;
  
  for (let iter = 0; iter < iterations; iter++) {
    let newPath = [];
    newPath.push(currentPath[0]); // Giữ nguyên điểm xuất phát
    
    for (let i = 0; i < currentPath.length - 1; i++) {
        const p0 = currentPath[i];
        const p1 = currentPath[i + 1];
        
        // Cắt khúc ở tỷ lệ 20% - 80% để gọt êm hơn (Chaikin biến thể ủ cong dần)
        newPath.push({
          x: 0.8 * p0.x + 0.2 * p1.x,
          y: 0.8 * p0.y + 0.2 * p1.y
        });
        
        newPath.push({
          x: 0.2 * p0.x + 0.8 * p1.x,
          y: 0.2 * p0.y + 0.8 * p1.y
        });
    }
    newPath.push(currentPath[currentPath.length - 1]); // Giữ nguyên điểm cuối
    currentPath = newPath;
  }
  
  // Lọc bớt các điểm trùng lặp cực gần nhau (dưới 1cm) để tối ưu hóa bộ nhớ ESP32
  let optimizedPath = [currentPath[0]];
  for (let i = 1; i < currentPath.length; i++) {
     const lastPt = optimizedPath[optimizedPath.length - 1];
     const pt = currentPath[i];
     const d = Math.hypot(pt.x - lastPt.x, pt.y - lastPt.y);
     if (d > 0.01) { 
        optimizedPath.push(pt);
     }
  }
  
  return optimizedPath;
}

/**
 * Smooth path: RDP + Chaikin (corner cutting) spline
 */
function smoothPath(path) {
  if (path.length <= 2) return path;

  // Bước 1: RDP simplify (Lọc điểm nhiễu A*, tạo các đoạn hẻm khuyều 90 độ)
  const simplified = rdpSimplify(path, 0.15);

  // Bước 2: Curve Spline nội suy uốn cong dải điểm (Làm tròn góc gắt)
  const splined = chaikinSmooth(simplified, 3);

  // Lưu ý: ESP32 chỉ chứa được MAX_WAYPOINTS (Vd: 64 điểm).
  // Vì vậy nếu đường splined quá lớn, ta sẽ trích xuất (downsample).
  if (splined.length > 60) {
     const downsampled = [];
     const step = Math.ceil(splined.length / 60);
     for (let i = 0; i < splined.length; i += step) {
         downsampled.push(splined[i]);
     }
     if (downsampled[downsampled.length - 1] !== splined[splined.length - 1]) {
         downsampled.push(splined[splined.length - 1]);
     }
     return downsampled;
  }

  return splined;
}

// ============================================================
//   HELPER
// ============================================================

/**
 * Tìm ô trống gần nhất (BFS)
 */
function findNearestFreeCell(grid, col, row) {
  const visited = new Set();
  const queue = [{ col, row }];

  while (queue.length > 0) {
    const { col: c, row: r } = queue.shift();
    const key = `${c},${r}`;
    if (visited.has(key)) continue;
    visited.add(key);

    if (c < 0 || c >= GRID_COLS || r < 0 || r >= GRID_ROWS) continue;

    if (grid[r][c] === 0) return { col: c, row: r };

    queue.push({ col: c + 1, row: r });
    queue.push({ col: c - 1, row: r });
    queue.push({ col: c, row: r + 1 });
    queue.push({ col: c, row: r - 1 });
  }
  return null;
}

/**
 * Kiểm tra đường thẳng từ A→B có va chạm obstacle không
 * Dùng cho path validation
 */
export function isLineOfSightClear(x1, y1, x2, y2, grid = null) {
  if (!grid) grid = createOccupancyGrid();

  const start = meterToGrid(x1, y1);
  const end = meterToGrid(x2, y2);

  let x0 = start.col;
  let y0 = start.row;
  const x1_idx = end.col;
  const y1_idx = end.row;

  const dx = Math.abs(x1_idx - x0);
  const dy = Math.abs(y1_idx - y0);
  const sx = x0 < x1_idx ? 1 : -1;
  const sy = y0 < y1_idx ? 1 : -1;
  let err = dx - dy;

  while (true) {
    if (x0 < 0 || x0 >= GRID_COLS || y0 < 0 || y0 >= GRID_ROWS) return false;
    if (grid[y0][x0] === 1) return false;

    if (x0 === x1_idx && y0 === y1_idx) break;
    const e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
  return true;
}
