import * as Comlink from 'comlink';
import { ScanMatcher } from './scanMatcher.js';
import findPathOnGrid from './lidarPathfinder.js';
import { computeVelocityCmd } from './dwaPlanner.js';

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
  computeVelocity(pose, vel, globalPlan, gridData, dwaConfig = null) {
    const grid = new WorkerGrid(gridData);
    return computeVelocityCmd(pose, vel, globalPlan, grid, dwaConfig);
  }
}

Comlink.expose(new NavWorkerAPI());
