import * as Comlink from 'comlink';
import { ScanMatcher } from './scanMatcher.js';
import findPathOnGrid from './lidarPathfinder.js';
import MPPIPlanner from './mppiPlanner.js';
import { ROBOT_RADIUS } from './warehouse.js';

// Dummy Grid object to reconstruct methods on the worker side
class WorkerGrid {
  constructor(data) {
    Object.assign(this, data);
  }
  
  inBounds(gx, gy) {
    return gx >= 0 && gx < this.width && gy >= 0 && gy < this.height;
  }
  
  getLogOdds(gx, gy) {
    if (!this.inBounds(gx, gy)) return 0;
    return this.logOdds[gy * this.width + gx];
  }
  
  getCost(gx, gy) {
    if (!this.inBounds(gx, gy) || !this.costmap) return 0;
    return this.costmap[gy * this.width + gx];
  }

  isFree(gx, gy) {
    return this.getLogOdds(gx, gy) < -0.3;
  }

  isOccupied(gx, gy) {
    return this.getLogOdds(gx, gy) > 0.3;
  }

  worldToGrid(wx, wy) {
    const gx = Math.floor((wx - this.originX) / this.resolution);
    const gy = Math.floor((wy - this.originY) / this.resolution);
    return { gx, gy };
  }

  gridToWorld(gx, gy) {
    const wx = this.originX + gx * this.resolution + this.resolution / 2.0;
    const wy = this.originY + gy * this.resolution + this.resolution / 2.0;
    return { x: wx, y: wy };
  }
}

class NavWorkerAPI {
  constructor() {
    this.scanMatchers = new Map(); // robotId -> ScanMatcher
  }

  // SLAM: Phù hợp Scan Lidar để tìm tọa độ
  matchScan(robotId, gridData, mapX, mapY, mapTheta, lidarPts) {
    if (!this.scanMatchers.has(robotId)) {
      this.scanMatchers.set(robotId, new ScanMatcher());
    }
    const matcher = this.scanMatchers.get(robotId);
    const grid = new WorkerGrid(gridData);
    return matcher.matchScan(grid, mapX, mapY, mapTheta, lidarPts);
  }

  /**
   * processSlamTick — Entire SLAM pipeline in Worker thread
   * 
   * Replaces the inline SLAM logic that was running on main thread.
   * Main thread sends raw data → Worker does all computation → returns results.
   * 
   * @param {string}  robotId    - Robot identifier
   * @param {object}  gridData   - Serialized OccupancyGrid
   * @param {object}  odomPose   - { x, y, theta } from encoder/IMU (odom frame)
   * @param {object}  tfState    - Current map→odom transform { dx, dy, dTheta }
   * @param {Array}   lidarPts   - Raw lidar points [{a, d}, ...]
   * @param {boolean} isMapping  - Whether to update the grid (vs localization-only)
   * @param {boolean} doMatch    - Whether to run scan matching this tick
   * @returns {{ newTf, matchScore, corrected, gridCells }}
   */
  processSlamTick(robotId, gridData, odomPose, tfState, lidarPts, isMapping, doMatch) {
    const grid = new WorkerGrid(gridData);
    const tf = tfState || { dx: 0, dy: 0, dTheta: 0 };

    // Apply current TF to get map-frame pose
    const mapX = odomPose.x + tf.dx;
    const mapY = odomPose.y + tf.dy;
    const mapTheta = odomPose.theta + tf.dTheta;

    let newTf = { ...tf };
    let matchScore = 0;
    let corrected = false;

    // 1) Scan Matching (if requested)
    if (doMatch && lidarPts.length > 0) {
      if (!this.scanMatchers.has(robotId)) {
        this.scanMatchers.set(robotId, new ScanMatcher());
      }
      const matcher = this.scanMatchers.get(robotId);
      const result = matcher.matchScan(grid, mapX, mapY, mapTheta, lidarPts);
      matchScore = result.score || 0;
      if (result.corrected && result.correction) {
        newTf = {
          dx: tf.dx + result.correction.dx,
          dy: tf.dy + result.correction.dy,
          dTheta: tf.dTheta + result.correction.dTheta,
        };
        corrected = true;
      }
    }

    // 2) Update grid cells (if mapping mode)
    //    We compute the corrected map pose for grid update
    let gridCells = null;
    if (isMapping && lidarPts.length > 0) {
      const correctedX = odomPose.x + newTf.dx;
      const correctedY = odomPose.y + newTf.dy;
      const correctedTheta = odomPose.theta + newTf.dTheta;

      // Update grid using the WorkerGrid's update method
      // Since WorkerGrid is a lightweight proxy, we compute the cells to update
      // and return them so the main thread can apply them to the real grid
      gridCells = this._computeGridUpdates(grid, correctedX, correctedY, correctedTheta, lidarPts);
    }

    return { newTf, matchScore, corrected, gridCells };
  }

  /**
   * Compute which grid cells to update from a scan (ray-casting)
   * Returns sparse update list so main thread can apply to real OccupancyGrid
   */
  _computeGridUpdates(grid, robotX, robotY, robotTheta, lidarPts) {
    const updates = []; // { gx, gy, hit: boolean }
    const { resolution, originX, originY, width, height } = grid;

    const rgx = Math.floor((robotX - originX) / resolution);
    const rgy = Math.floor((robotY - originY) / resolution);

    for (const pt of lidarPts) {
      const angle = robotTheta + (pt.a * Math.PI) / 180;
      const dist = pt.d;
      if (dist <= 0.05 || dist > 12) continue;

      // Hit cell
      const hitX = robotX + Math.cos(angle) * dist;
      const hitY = robotY + Math.sin(angle) * dist;
      const hgx = Math.floor((hitX - originX) / resolution);
      const hgy = Math.floor((hitY - originY) / resolution);

      // Bresenham ray: mark free cells along the ray
      const steps = Math.max(Math.abs(hgx - rgx), Math.abs(hgy - rgy));
      if (steps === 0) continue;

      for (let i = 0; i < steps; i++) {
        const t = i / steps;
        const cx = Math.floor(rgx + (hgx - rgx) * t);
        const cy = Math.floor(rgy + (hgy - rgy) * t);
        if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
          updates.push({ gx: cx, gy: cy, hit: false });
        }
      }

      // Mark hit cell as occupied
      if (hgx >= 0 && hgx < width && hgy >= 0 && hgy < height) {
        updates.push({ gx: hgx, gy: hgy, hit: true });
      }
    }

    return updates;
  }

  // PATHFINDING: Tìm đường A* trên grid
  findPath(gridData, startX, startY, goalX, goalY, allowUnknown = false, useCostmap = true) {
    const grid = new WorkerGrid(gridData);
    return findPathOnGrid(grid, startX, startY, goalX, goalY, {
      allowUnknown,
      useCostmap,
    });
  }

  // DWA LOCAL PLANNER: Tính toán v, w để lách vật cản
  computeVelocity(pose, vel, globalPlan, gridData, dwaConfig = null, lidarPts = null) {
    const t0 = performance.now();
    const grid = new WorkerGrid(gridData);

    // INJECT LOCAL COSTMAP from live LiDAR data
    if (lidarPts && lidarPts.length > 0 && grid.logOdds) {
      if (!grid.costmap) {
        grid.costmap = new Uint8Array(grid.width * grid.height);
      }

      // ── RADIUS CONSTANTS — Must match warehouse.js ROBOT_RADIUS ──
      const INSCRIBED_R = ROBOT_RADIUS;   // 0.15m — matches physics collision
      const INFLATION_R = 0.40;           // Was 0.55 — too wide for corridors
      const COST_SCALING = 2.5;           // Was 3.0 — smoother gradient
      const inflCells = Math.ceil(INFLATION_R / grid.resolution);
      const inscribedCells = Math.ceil(INSCRIBED_R / grid.resolution);

      for (const pt of lidarPts) {
        const distM = pt.d / 1000.0;
        if (distM < 0.12 || distM > 3.5) continue;

        const angle = pose.theta + (pt.a * Math.PI) / 180;
        const hitX = pose.x + Math.cos(angle) * distM;
        const hitY = pose.y + Math.sin(angle) * distM;
        const { gx, gy } = grid.worldToGrid(hitX, hitY);

        if (!grid.inBounds(gx, gy)) continue;

        const centerIdx = gy * grid.width + gx;
        grid.logOdds[centerIdx] = Math.max(grid.logOdds[centerIdx], 5.0);
        grid.costmap[centerIdx] = 254;

        for (let dy = -inflCells; dy <= inflCells; dy++) {
          for (let dx = -inflCells; dx <= inflCells; dx++) {
            if (dx === 0 && dy === 0) continue;
            const nx = gx + dx;
            const ny = gy + dy;
            if (!grid.inBounds(nx, ny)) continue;

            const cellDist = Math.hypot(dx, dy);
            if (cellDist > inflCells) continue;

            const idx = ny * grid.width + nx;
            let cost;

            if (cellDist <= inscribedCells) {
              cost = 253;
            } else {
              const distMeters = cellDist * grid.resolution;
              const decayDist = distMeters - INSCRIBED_R;
              cost = Math.round(252 * Math.exp(-COST_SCALING * decayDist));
            }

            if (cost > 0 && cost > grid.costmap[idx]) {
              grid.costmap[idx] = cost;
            }

            if (cellDist <= inscribedCells && grid.logOdds[idx] < 2.0) {
              grid.logOdds[idx] = 2.0;
            }
          }
        }
      }

      // ── SELF-CLEAR: Remove costmap artifacts at robot's current position ──
      // Without this, nearby LiDAR reflections write lethal costs INTO the
      // robot's own footprint → DWA sees "phantom obstacle" → all trajectories rejected.
      const robotG = grid.worldToGrid(pose.x, pose.y);
      const clearR = Math.ceil((ROBOT_RADIUS + 0.05) / grid.resolution); // Slightly larger than physical radius
      for (let dy = -clearR; dy <= clearR; dy++) {
        for (let dx = -clearR; dx <= clearR; dx++) {
          const nx = robotG.gx + dx;
          const ny = robotG.gy + dy;
          if (!grid.inBounds(nx, ny)) continue;
          const cellDist = Math.hypot(dx, dy) * grid.resolution;
          if (cellDist <= ROBOT_RADIUS + 0.05) {
            const idx = ny * grid.width + nx;
            grid.costmap[idx] = 0;   // Clear costmap
            // Don't clear logOdds — that's the persistent map
          }
        }
      }
    }

    if (!this.mppiPlanner) {
      this.mppiPlanner = new MPPIPlanner(dwaConfig || {});
    }
    // Update config if needed
    this.mppiPlanner.cfg = { ...this.mppiPlanner.cfg, ...(dwaConfig || {}) };
    
    const result = this.mppiPlanner.computeVelocityCmd(pose, globalPlan, grid);
    const t1 = performance.now();
    console.log(`[Worker] MPPI computeVelocity took ${(t1 - t0).toFixed(1)}ms`);
    return result;
  }
}

Comlink.expose(new NavWorkerAPI());
