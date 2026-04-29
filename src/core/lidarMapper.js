/**
 * AMR 2.0 — LIDAR-based Occupancy Grid Mapper (v2)
 * 
 * Thuật toán: Log-Odds Occupancy Grid Mapping + Bresenham Ray Tracing
 * Tham khảo: Probabilistic Robotics (Thrun) + ROS2 nav2_costmap_2d
 * 
 * Cải tiến v2:
 *   - Dynamic Origin: Grid tự center theo vị trí ban đầu robot
 *   - Auto-Expand: Tự mở rộng khi robot tiến gần biên
 *   - Inflation Layer: Phình vật cản ra để robot giữ khoảng cách an toàn
 *   - Improved Render: Alpha cao hơn, color scheme rõ ràng hơn
 */

// ============================================================
//   LOG-ODDS CONSTANTS
// ============================================================

const L_PRIOR = 0;
const L_OCC = 0.85;
const L_FREE = -0.40;
const L_MIN = -5.0;
const L_MAX = 5.0;
const MAX_LIDAR_RANGE_M = 3.0;
const MIN_LIDAR_RANGE_M = 0.12;  // Lọc noise thân robot (articubot: 0.3m)

/** Số cells cách biên khi bắt đầu auto-expand */
const EXPAND_MARGIN = 8;
/** Số cells mở rộng thêm mỗi lần */
const EXPAND_AMOUNT = 30;

/** Articubot Nav2 inflation params */
const COST_SCALING_FACTOR = 3.0;  // articubot: 3.0 — controls exponential decay
const INSCRIBED_RADIUS = 0.22;    // articubot robot_radius: 0.22m
const INFLATION_RADIUS_M = 0.55;  // articubot: 0.55m

// ============================================================
//   OCCUPANCY GRID DATA STRUCTURE
// ============================================================

export class OccupancyGrid {
  /**
   * @param {number} width - Grid width in cells
   * @param {number} height - Grid height in cells
   * @param {number} resolution - Cell size in meters (e.g., 0.25m)
   * @param {number} initRobotX - Robot X khi bắt đầu (world meters)
   * @param {number} initRobotY - Robot Y khi bắt đầu (world meters)
   */
  constructor(width = 100, height = 100, resolution = 0.1, initRobotX = 0, initRobotY = 0) {
    this.width = width;
    this.height = height;
    this.resolution = resolution;

    // === DYNAMIC ORIGIN ===
    // Grid origin = world position tương ứng cell (0,0)
    // Robot sẽ nằm ở giữa grid khi khởi tạo
    this.originX = initRobotX - (width * resolution) / 2;
    this.originY = initRobotY - (height * resolution) / 2;

    this.worldWidth = width * resolution;
    this.worldHeight = height * resolution;

    // Log-odds grid
    this.logOdds = new Float32Array(width * height);
    this.logOdds.fill(L_PRIOR);

    // Occupancy cache (0-255)
    this.data = new Uint8Array(width * height);
    this.data.fill(128);

    // Inflation costmap (0=free, 255=lethal)
    this.costmap = new Uint8Array(this.width * this.height);
    this.costmap.fill(0); // 0: free, 254: lethal, 255: unknown

    this.forbiddenZones = []; // Array of { x, y, w, h } in world coords

    // Robot pose
    this.robotX = initRobotX;
    this.robotY = initRobotY;
    this.robotHeading = 0;

    // Statistics
    this.scanCount = 0;
    this.lastUpdate = Date.now();
    this._dirty = false;

    // Frontier cache (updated by exploration)
    this.frontierCells = [];
  }

  setForbiddenZones(zones) {
    this.forbiddenZones = zones;
  }

  // ============================================================
  //   COORDINATE CONVERSION (with dynamic origin)
  // ============================================================

  worldToGrid(wx, wy) {
    return {
      gx: Math.floor((wx - this.originX) / this.resolution),
      gy: Math.floor((wy - this.originY) / this.resolution),
    };
  }

  gridToWorld(gx, gy) {
    return {
      x: this.originX + (gx + 0.5) * this.resolution,
      y: this.originY + (gy + 0.5) * this.resolution,
    };
  }

  inBounds(gx, gy) {
    return gx >= 0 && gx < this.width && gy >= 0 && gy < this.height;
  }

  // ============================================================
  //   CORE MAPPING ALGORITHM
  // ============================================================

  updateFromScan(robotX, robotY, robotHeading, lidarPoints) {
    if (!lidarPoints || lidarPoints.length === 0) return;

    this.robotX = robotX;
    this.robotY = robotY;
    this.robotHeading = robotHeading;

    // Auto-expand nếu robot gần biên grid
    const rg = this.worldToGrid(robotX, robotY);
    this._maybeExpandGrid(rg.gx, rg.gy);

    const robotGX = rg.gx;
    const robotGY = rg.gy;

    if (!this.inBounds(robotGX, robotGY)) return;

    for (let i = 0; i < lidarPoints.length; i++) {
      const pt = lidarPoints[i];
      const distM = pt.d / 1000.0;

      if (distM < MIN_LIDAR_RANGE_M || distM > MAX_LIDAR_RANGE_M) continue;

      const lidarRad = (pt.a * Math.PI) / 180.0;
      // World angle: same convention as simLidar (theta + localAngle)
      const worldAngle = robotHeading + lidarRad;

      const endX = robotX + Math.cos(worldAngle) * distM;
      const endY = robotY + Math.sin(worldAngle) * distM;

      const eg = this.worldToGrid(endX, endY);

      // Clamp endpoint vào grid bounds
      const endGX = Math.max(0, Math.min(this.width - 1, eg.gx));
      const endGY = Math.max(0, Math.min(this.height - 1, eg.gy));

      this._bresenhamUpdate(robotGX, robotGY, endGX, endGY);
    }

    this.scanCount++;
    this.lastUpdate = Date.now();
    this._dirty = true;
    this._syncDataFromLogOdds();

    // Auto-inflate costmap mỗi 5 scans (Nav2 costmap_2d style)
    if (this.scanCount % 5 === 0) {
      // Inflation radius in cells = INFLATION_RADIUS_M / resolution
      const inflCells = Math.ceil(INFLATION_RADIUS_M / this.resolution);
      this.inflateObstacles(inflCells);
    }
  }

  // ============================================================
  //   AUTO-EXPAND GRID
  // ============================================================

  _maybeExpandGrid(robotGX, robotGY) {
    let expandLeft = 0, expandRight = 0, expandDown = 0, expandUp = 0;

    if (robotGX < EXPAND_MARGIN) expandLeft = EXPAND_AMOUNT;
    if (robotGX > this.width - EXPAND_MARGIN) expandRight = EXPAND_AMOUNT;
    if (robotGY < EXPAND_MARGIN) expandDown = EXPAND_AMOUNT;
    if (robotGY > this.height - EXPAND_MARGIN) expandUp = EXPAND_AMOUNT;

    if (expandLeft === 0 && expandRight === 0 && expandDown === 0 && expandUp === 0) return;

    const newW = this.width + expandLeft + expandRight;
    const newH = this.height + expandDown + expandUp;

    // Giới hạn max size (200×200 = 50×50m @ 0.25m)
    if (newW > 300 || newH > 300) return;  // Max 300×300 = 30×30m @0.1m

    const newLogOdds = new Float32Array(newW * newH);
    newLogOdds.fill(L_PRIOR);

    // Copy old grid vào vị trí mới (offset bởi expandLeft, expandDown)
    for (let gy = 0; gy < this.height; gy++) {
      for (let gx = 0; gx < this.width; gx++) {
        const oldIdx = gy * this.width + gx;
        const newIdx = (gy + expandDown) * newW + (gx + expandLeft);
        newLogOdds[newIdx] = this.logOdds[oldIdx];
      }
    }

    // Update origin (shift left/down in world coords)
    this.originX -= expandLeft * this.resolution;
    this.originY -= expandDown * this.resolution;

    this.width = newW;
    this.height = newH;
    this.worldWidth = newW * this.resolution;
    this.worldHeight = newH * this.resolution;
    this.logOdds = newLogOdds;
    this.data = new Uint8Array(newW * newH);
    this.data.fill(128);
    this.costmap = new Uint8Array(newW * newH);
    this.costmap.fill(0);

    this._syncDataFromLogOdds();
    console.log(`[Grid] Auto-expanded to ${newW}×${newH} (${this.worldWidth.toFixed(0)}×${this.worldHeight.toFixed(0)}m)`);
  }

  // ============================================================
  //   BRESENHAM RAY TRACING + LOG-ODDS
  // ============================================================

  _bresenhamUpdate(x0, y0, x1, y1) {
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;
    let x = x0, y = y0;

    while (true) {
      if (x === x1 && y === y1) {
        this._updateLogOdds(x, y, L_OCC);
        break;
      }
      this._updateLogOdds(x, y, L_FREE);

      const e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 < dx) { err += dx; y += sy; }
    }
  }

  _updateLogOdds(gx, gy, delta) {
    if (!this.inBounds(gx, gy)) return;
    const idx = gy * this.width + gx;
    this.logOdds[idx] = Math.max(L_MIN, Math.min(L_MAX, this.logOdds[idx] + delta));
  }

  _syncDataFromLogOdds() {
    for (let i = 0; i < this.logOdds.length; i++) {
      const p = 1.0 / (1.0 + Math.exp(-this.logOdds[i]));
      this.data[i] = Math.round(p * 255);
    }
  }

  // ============================================================
  //   INFLATION LAYER (tham khảo nav2_costmap_2d)
  // ============================================================

  /**
   * Phình vật cản ra inflationRadius cells
   * Tham khảo: nav2_costmap_2d::InflationLayer
   *
   * Cost model (articubot style):
   *   - Lethal (254): obstacle cell
   *   - Inscribed (253): within robot_radius (robot center touches obstacle)
   *   - Inflation (1-252): exponential decay = 252 * exp(-cost_scaling_factor * (dist - inscribed_radius))
   *   - Free (0): beyond inflation_radius
   */
  inflateObstacles(inflationRadius = 6) {
    this.costmap.fill(0);
    const w = this.width, h = this.height;
    const res = this.resolution;

    // Nav2-style radii in cells
    const inscribedCells = Math.ceil(INSCRIBED_RADIUS / res);

    // BFS distance grid (in cells)
    const distGrid = new Float32Array(w * h);
    distGrid.fill(Infinity);

    // BFS queue: [index, distance_cells]
    const queue = [];

    // Seed: all occupied cells (Lidar Obstacle Layer)
    for (let i = 0; i < this.logOdds.length; i++) {
      if (this.logOdds[i] > 0.5) {
        this.costmap[i] = 254; // Lethal
        distGrid[i] = 0;
        queue.push(i);
      }
    }

    // Apply Forbidden Zones Layer
    if (this.forbiddenZones && this.forbiddenZones.length > 0) {
      this.forbiddenZones.forEach(zone => {
        const gx1 = Math.max(0, Math.floor((zone.x - this.originX) / res));
        const gy1 = Math.max(0, Math.floor((zone.y - this.originY) / res));
        const gx2 = Math.min(w - 1, Math.floor((zone.x + zone.w - this.originX) / res));
        const gy2 = Math.min(h - 1, Math.floor((zone.y + zone.h - this.originY) / res));
        
        for (let gy = gy1; gy <= gy2; gy++) {
          for (let gx = gx1; gx <= gx2; gx++) {
            const idx = gy * w + gx;
            if (distGrid[idx] > 0) {
              this.costmap[idx] = 254; // Lethal Forbidden
              distGrid[idx] = 0;
              queue.push(idx);
            }
          }
        }
      });
    }

    // BFS expand with proper level tracking
    let qi = 0;
    while (qi < queue.length) {
      const idx = queue[qi++];
      const gx = idx % w;
      const gy = Math.floor(idx / w);
      const parentDist = distGrid[idx];

      // 8-connected neighbors
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (dx === 0 && dy === 0) continue;
          const nx = gx + dx, ny = gy + dy;
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
          const nIdx = ny * w + nx;

          // True distance (diagonal = √2)
          const stepDist = (dx !== 0 && dy !== 0) ? 1.414 : 1.0;
          const newDist = parentDist + stepDist;

          if (newDist >= distGrid[nIdx] || newDist > inflationRadius) continue;
          distGrid[nIdx] = newDist;

          // Nav2-style exponential cost decay
          const distMeters = newDist * res;
          let cost;
          if (newDist <= inscribedCells) {
            cost = 253; // Inscribed — robot center would collide
          } else {
            // Exponential decay: cost = 252 * exp(-factor * (dist - inscribed))
            const decayDist = distMeters - INSCRIBED_RADIUS;
            cost = Math.round(252 * Math.exp(-COST_SCALING_FACTOR * decayDist));
          }

          if (cost > 0 && cost > this.costmap[nIdx]) {
            this.costmap[nIdx] = cost;
            queue.push(nIdx);
          }
        }
      }
    }
  }

  /**
   * Clear costmap — Nav2 recovery action
   * Xóa toàn bộ inflation layer, giữ nguyên obstacle data (logOdds)
   * Dùng khi robot bị stuck do phantom obstacles
   */
  clearCostmap() {
    this.costmap.fill(0);
    // Re-inflate ngay để có data mới
    const inflCells = Math.ceil(INFLATION_RADIUS_M / this.resolution);
    this.inflateObstacles(inflCells);
    this._dirty = true;
    console.log('[Grid] Costmap cleared and re-inflated');
  }

  /**
   * Clear a region of the costmap around the robot (partial clear)
   * Useful for clearing phantom obstacles near robot without resetting entire map
   */
  clearCostmapAround(centerX, centerY, radiusM = 2.0) {
    const g = this.worldToGrid(centerX, centerY);
    const radiusCells = Math.ceil(radiusM / this.resolution);
    
    for (let dy = -radiusCells; dy <= radiusCells; dy++) {
      for (let dx = -radiusCells; dx <= radiusCells; dx++) {
        const gx = g.gx + dx, gy = g.gy + dy;
        if (!this.inBounds(gx, gy)) continue;
        if (dx * dx + dy * dy > radiusCells * radiusCells) continue;
        const idx = gy * this.width + gx;
        // Only clear if the logOdds aren't strongly occupied
        // (keep walls, clear phantom readings)
        if (this.logOdds[idx] < 2.0) {
          this.logOdds[idx] = Math.max(L_MIN, this.logOdds[idx] * 0.3);
        }
      }
    }
    this._syncDataFromLogOdds();
    const inflCells = Math.ceil(INFLATION_RADIUS_M / this.resolution);
    this.inflateObstacles(inflCells);
    this._dirty = true;
  }

  // ============================================================
  //   DATA ACCESS
  // ============================================================

  getCell(gx, gy) {
    if (!this.inBounds(gx, gy)) return 128;
    return this.data[gy * this.width + gx];
  }

  getLogOdds(gx, gy) {
    if (!this.inBounds(gx, gy)) return L_PRIOR;
    return this.logOdds[gy * this.width + gx];
  }

  getCost(gx, gy) {
    if (!this.inBounds(gx, gy)) return 255;
    return this.costmap[gy * this.width + gx];
  }

  isFree(gx, gy) {
    return this.inBounds(gx, gy) && this.logOdds[gy * this.width + gx] < -0.3;
  }

  isOccupied(gx, gy) {
    return this.inBounds(gx, gy) && this.logOdds[gy * this.width + gx] > 0.3;
  }

  isUnknown(gx, gy) {
    if (!this.inBounds(gx, gy)) return true;
    return Math.abs(this.logOdds[gy * this.width + gx]) <= 0.3;
  }

  setCell(gx, gy, value) {
    if (this.inBounds(gx, gy)) {
      this.data[gy * this.width + gx] = value;
      const p = Math.max(0.001, Math.min(0.999, value / 255.0));
      this.logOdds[gy * this.width + gx] = Math.log(p / (1 - p));
    }
  }

  getFreeCell(gx, gy) {
    return this.getCell(gx, gy) < 100;
  }

  getObstacles() {
    const obstacles = [];
    for (let gy = 0; gy < this.height; gy++) {
      for (let gx = 0; gx < this.width; gx++) {
        if (this.getCell(gx, gy) > 150) {
          const { x, y } = this.gridToWorld(gx, gy);
          obstacles.push({ x, y, occupancy: this.getCell(gx, gy) });
        }
      }
    }
    return obstacles;
  }

  isDirty() { return this._dirty; }
  clearDirty() { this._dirty = false; }

  clear() {
    this.logOdds.fill(L_PRIOR);
    this.data.fill(128);
    this.costmap.fill(0);
    this.scanCount = 0;
    this._dirty = true;
  }

  // ============================================================
  //   VISUALIZATION — Render to ImageData
  // ============================================================

  renderToImageData(showCostmap = true) {
    const imgData = new ImageData(this.width, this.height);
    const pixels = imgData.data;

    for (let gy = 0; gy < this.height; gy++) {
      for (let gx = 0; gx < this.width; gx++) {
        const idx = gy * this.width + gx;
        const lo = this.logOdds[idx];
        const cm = this.costmap[idx]; // 0=free, 1-199=inflation, 200=inscribed, 254=lethal
        const pxIdx = ((this.height - 1 - gy) * this.width + gx) * 4;

        // ── Nav2 RViz Costmap Color Scheme ──────────────
        //   Lethal (254)     → Magenta   #FF00FF
        //   Inscribed (200+) → Dark Red  #CC0033
        //   Inflation (1-199)→ Purple→Cyan gradient
        //   Free (0)         → Light gray / White
        //   Unknown          → Medium gray
        // ─────────────────────────────────────────────────

        if (lo > 0.5) {
          // ── OCCUPIED (Lethal obstacle) — Magenta like RViz ──
          pixels[pxIdx]     = 255;  // R
          pixels[pxIdx + 1] = 0;    // G
          pixels[pxIdx + 2] = 255;  // B — Magenta
          pixels[pxIdx + 3] = 240;

        } else if (showCostmap && cm >= 200) {
          // ── INSCRIBED zone — Dark Red/Magenta ──
          pixels[pxIdx]     = 200;
          pixels[pxIdx + 1] = 0;
          pixels[pxIdx + 2] = 80;
          pixels[pxIdx + 3] = 220;

        } else if (showCostmap && cm > 0) {
          // ── INFLATION gradient — Purple → Blue → Cyan ──
          // cm: 199→1 maps to intense purple → fading cyan
          const t = cm / 199; // 1.0=near obstacle, 0.0=far
          
          if (t > 0.6) {
            // Near obstacle: Purple/Red (hot)
            const s = (t - 0.6) / 0.4; // 0→1
            pixels[pxIdx]     = Math.round(180 * s + 60);   // R: 60→240
            pixels[pxIdx + 1] = Math.round(20);              // G: low
            pixels[pxIdx + 2] = Math.round(200 - 80 * s);   // B: 200→120
            pixels[pxIdx + 3] = Math.round(160 + 60 * s);
          } else if (t > 0.2) {
            // Mid zone: Blue/Purple
            const s = (t - 0.2) / 0.4; // 0→1
            pixels[pxIdx]     = Math.round(60 * s);          // R: 0→60
            pixels[pxIdx + 1] = Math.round(80 * (1 - s));    // G: 80→0
            pixels[pxIdx + 2] = Math.round(200 + 40 * s);    // B: 200→240
            pixels[pxIdx + 3] = Math.round(120 + 40 * s);
          } else {
            // Far from obstacle: Cyan (safe zone)
            const s = t / 0.2; // 0→1
            pixels[pxIdx]     = 0;
            pixels[pxIdx + 1] = Math.round(200 - 120 * s);   // G: 200→80
            pixels[pxIdx + 2] = Math.round(230 - 30 * s);    // B: 230→200
            pixels[pxIdx + 3] = Math.round(80 + 40 * s);
          }

        } else if (lo < -0.5) {
          // ── FREE — Light gray/white (like RViz) ──
          const intensity = Math.min(1.0, (-lo - 0.5) / 4.5);
          pixels[pxIdx]     = 200 + Math.round(40 * intensity);
          pixels[pxIdx + 1] = 200 + Math.round(40 * intensity);
          pixels[pxIdx + 2] = 210 + Math.round(30 * intensity);
          pixels[pxIdx + 3] = Math.round(160 + 95 * intensity);

        } else {
          // ── UNKNOWN — Medium gray ──
          pixels[pxIdx]     = 128;
          pixels[pxIdx + 1] = 128;
          pixels[pxIdx + 2] = 128;
          pixels[pxIdx + 3] = 100;
        }
      }
    }

    // ── Frontier cells — Bright Cyan dots (exploration targets) ──
    for (const f of this.frontierCells) {
      const pxIdx = ((this.height - 1 - f.gy) * this.width + f.gx) * 4;
      if (pxIdx >= 0 && pxIdx < pixels.length - 3) {
        pixels[pxIdx]     = 0;
        pixels[pxIdx + 1] = 255;
        pixels[pxIdx + 2] = 255;
        pixels[pxIdx + 3] = 240;
      }
    }

    return imgData;
  }

  // ============================================================
  //   SERIALIZATION
  // ============================================================

  exportJSON() {
    return {
      version: 2,
      timestamp: Date.now(),
      width: this.width,
      height: this.height,
      resolution: this.resolution,
      originX: this.originX,
      originY: this.originY,
      scanCount: this.scanCount,
      robotX: this.robotX,
      robotY: this.robotY,
      robotHeading: this.robotHeading,
      logOdds: _float32ToBase64(this.logOdds),
    };
  }

  static importJSON(json) {
    if (!json || (json.version !== 1 && json.version !== 2)) {
      throw new Error('Invalid map format or version');
    }

    const grid = new OccupancyGrid(json.width, json.height, json.resolution,
      json.robotX || 0, json.robotY || 0);

    // Restore origin for v2, calculate for v1
    if (json.version === 2) {
      grid.originX = json.originX ?? 0;
      grid.originY = json.originY ?? 0;
    }

    grid.scanCount = json.scanCount || 0;
    grid.robotX = json.robotX || 0;
    grid.robotY = json.robotY || 0;
    grid.robotHeading = json.robotHeading || 0;

    const decoded = _base64ToFloat32(json.logOdds);
    if (decoded.length === grid.logOdds.length) {
      grid.logOdds.set(decoded);
    }

    grid._syncDataFromLogOdds();
    grid.lastUpdate = json.timestamp || Date.now();
    grid._dirty = true;
    return grid;
  }

  static fromBinary(buffer) {
    const view = new DataView(buffer);
    const messageType = view.getUint8(0);
    if (messageType !== 0x01) {
      throw new Error(`Unsupported occupancy grid message type: ${messageType}`);
    }

    const width = view.getUint8(1);
    const height = view.getUint8(2);
    const resolution = view.getFloat32(3, true);
    const robotX = view.getFloat32(7, true);
    const robotY = view.getFloat32(11, true);
    const robotHeading = view.getFloat32(15, true);
    const expectedBytes = 19 + width * height;

    if (buffer.byteLength < expectedBytes) {
      throw new Error(`Occupancy grid buffer too short: ${buffer.byteLength} < ${expectedBytes}`);
    }

    const grid = new OccupancyGrid(width, height, resolution, robotX, robotY);
    grid.robotX = robotX;
    grid.robotY = robotY;
    grid.robotHeading = robotHeading;

    let offset = 19;
    for (let gy = 0; gy < height; gy++) {
      for (let gx = 0; gx < width; gx++) {
        const occupancy100 = view.getUint8(offset++);
        const probability = Math.max(0.001, Math.min(0.999, occupancy100 / 100.0));
        const logOdds = Math.log(probability / (1 - probability));
        const idx = gy * width + gx;
        grid.logOdds[idx] = Math.max(L_MIN, Math.min(L_MAX, logOdds));
      }
    }

    grid._syncDataFromLogOdds();
    grid._dirty = true;
    grid.lastUpdate = Date.now();
    return grid;
  }

  // ============================================================
  //   SERIALIZATION FOR WEB WORKER
  // ============================================================

  serialize() {
    return {
      width: this.width,
      height: this.height,
      resolution: this.resolution,
      originX: this.originX,
      originY: this.originY,
      scanCount: this.scanCount,
      logOdds: this.logOdds,
      costmap: this.costmap,
    };
  }
}

// ============================================================
//   STANDALONE BRESENHAM (for external use)
// ============================================================

export function bresenhamRay(grid, x0, y0, x1, y1) {
  const dx = Math.abs(x1 - x0);
  const dy = Math.abs(y1 - y0);
  const sx = x0 < x1 ? 1 : -1;
  const sy = y0 < y1 ? 1 : -1;
  let err = dx - dy;
  let x = x0, y = y0;
  const cells = [];

  while (true) {
    cells.push({ gx: x, gy: y });
    if (x === x1 && y === y1) break;
    const e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 < dx) { err += dx; y += sy; }
  }
  return cells;
}

// ============================================================
//   INTERNAL HELPERS
// ============================================================

function _float32ToBase64(float32Array) {
  const bytes = new Uint8Array(float32Array.buffer);
  let binary = '';
  for (let i = 0; i < bytes.length; i++) {
    binary += String.fromCharCode(bytes[i]);
  }
  return btoa(binary);
}

function _base64ToFloat32(base64) {
  const binary = atob(base64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) {
    bytes[i] = binary.charCodeAt(i);
  }
  return new Float32Array(bytes.buffer);
}

export class LidarScan {
  constructor() {
    this.points = [];
    this.timestamp = Date.now();
  }

  static fromBinary(buffer) {
    const view = new DataView(buffer);
    const scan = new LidarScan();
    const count = view.getUint16(0, true);
    let offset = 2;
    for (let i = 0; i < count; i++) {
      const angle = view.getUint16(offset, true) / 100.0;
      offset += 2;
      const distance = view.getUint16(offset, true) / 1000.0;
      offset += 2;
      if (distance > 0 && distance < 10) {
        scan.points.push({ angle, distance });
      }
    }
    scan.timestamp = Date.now();
    return scan;
  }
}

export default OccupancyGrid;
