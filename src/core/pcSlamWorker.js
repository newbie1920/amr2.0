import * as Comlink from 'comlink';
import { OccupancyGrid } from './lidarMapper.js';

let _grids = {};

const slamWorker = {
  /**
   * Khởi tạo grid mới
   * @param {string} id 
   * @param {number} width 
   * @param {number} height 
   * @param {number} resolution 
   * @param {number} startX 
   * @param {number} startY 
   */
  init(id, width, height, resolution, startX, startY) {
    _grids[id] = new OccupancyGrid(width, height, resolution, startX, startY);
    return true;
  },

  /**
   * Tải grid có sẵn
   * @param {string} id 
   * @param {Object} serializedGrid 
   */
  loadGrid(id, serializedGrid) {
    const grid = new OccupancyGrid(serializedGrid.width, serializedGrid.height, serializedGrid.resolution, serializedGrid.originX, serializedGrid.originY);
    grid.scanCount = serializedGrid.scanCount;
    grid.logOdds.set(serializedGrid.logOdds);
    grid.costmap.set(serializedGrid.costmap);
    _grids[id] = grid;
    return true;
  },

  /**
   * Xử lý raw lidar scan (từ frame 0x06) và cập nhật bản đồ
   * Trả về các phần thay đổi để UI vẽ lại
   * 
   * @param {string} id 
   * @param {number} robotX 
   * @param {number} robotY 
   * @param {number} robotHeading 
   * @param {Array<{a: number, d: number}>} lidarPoints 
   * @returns {Object} Serialized grid data if dirty, else null
   */
  processScan(id, robotX, robotY, robotHeading, lidarPoints) {
    let grid = _grids[id];
    if (!grid) {
      console.warn(`[PcSlamWorker] Grid ${id} not initialized. Auto-initializing.`);
      grid = new OccupancyGrid(1024, 1024, 0.05, robotX, robotY);
      _grids[id] = grid;
    }

    grid.updateFromScan(robotX, robotY, robotHeading, lidarPoints);

    if (grid.isDirty()) {
      grid.clearDirty();
      return grid.serialize();
    }
    return null;
  },

  /**
   * Serialize grid hiện tại
   */
  getGrid(id) {
    const grid = _grids[id];
    if (!grid) return null;
    return grid.serialize();
  }
};

Comlink.expose(slamWorker);
