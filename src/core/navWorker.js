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
