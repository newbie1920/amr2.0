/**
 * AMR 2.0 — Map Store (Zustand)
 * Quản lý toàn bộ Map lifecycle: Mapping, Localization, Map CRUD.
 * Tách từ robotStore.js để giữ single-responsibility.
 * 
 * Cross-store: Truy cập robotStore qua useRobotStore.getState() (Option A).
 */

import { create } from 'zustand';
import { OccupancyGrid } from '../core/lidarMapper.js';
import { ScanMatcher } from '../core/scanMatcher.js';
import { startExploration, stopExploration, getExplorationInfo, _setMapStoreGetter } from '../core/exploration.js';
import { navWorkerApi } from '../core/navWorkerSetup.js';

const MAP_STORAGE_KEY = 'amr_saved_maps';

function loadSavedMapsFromStorage() {
  try {
    const raw = localStorage.getItem(MAP_STORAGE_KEY);
    return raw ? JSON.parse(raw) : [];
  } catch (e) {
    console.error('[MapStorage] Error loading maps:', e);
    return [];
  }
}

function saveMapsToStorage(maps) {
  try {
    localStorage.setItem(MAP_STORAGE_KEY, JSON.stringify(maps));
  } catch (e) {
    console.error('[MapStorage] Error saving maps:', e);
  }
}

const useMapStore = create((set, get) => ({
  // State
  occupancyGrid: {},       // { robotId: OccupancyGrid instance }
  mappingActive: {},       // { robotId: boolean }
  localizationActive: {},  // { robotId: boolean } - AMCL mode
  mapperInstances: {},     // { robotId: OccupancyGrid } — live mapping instances
  savedMaps: loadSavedMapsFromStorage(),
  explorationInfo: {},     // { robotId: { phase, noFrontierCount, ... } }
  isMatching: {},          // { robotId: boolean } — Worker SLAM processing lock
  mapToOdom: {},           // { robotId: { dx, dy, dTheta } } — REP 105 TF Frames

  // ============================================================
  //   MAPPING ACTIONS
  // ============================================================

  /**
   * Bắt đầu quét Occupancy Grid cho robot
   */
  startMapping: (id, getRobotStore) => {
    const robotStore = getRobotStore();
    const robot = robotStore.robots[id];
    const telem = robot?.telemetry || {};
    const rx = telem.x ?? 0;
    const ry = telem.y ?? 0;

    // Set architecture profile
    const adapter = robot?.adapter;
    if (adapter) adapter.setArchitectureProfile('pc_slam');

    const grid = new OccupancyGrid(200, 200, 0.1, rx, ry);
    set((state) => ({
      mappingActive: { ...state.mappingActive, [id]: true },
      mapperInstances: { ...state.mapperInstances, [id]: grid },
      occupancyGrid: { ...state.occupancyGrid, [id]: grid },
      mapToOdom: { ...state.mapToOdom, [id]: { dx: 0, dy: 0, dTheta: 0 } },
    }));
    console.log(`[Mapping] Bắt đầu quét map cho robot ${id} tại (${rx.toFixed(2)}, ${ry.toFixed(2)}), grid 200x200 @0.1m`);

    // Bắt đầu tự khám phá
    startExploration(id, getRobotStore);

    // Update exploration info mỗi giây cho UI
    const infoTimer = setInterval(() => {
      if (!get().mappingActive[id]) { clearInterval(infoTimer); return; }
      set((s) => ({ explorationInfo: { ...s.explorationInfo, [id]: getExplorationInfo() } }));
    }, 1000);
  },

  /**
   * Dừng quét + TỰ ĐỘNG lưu map vào danh sách
   */
  stopMapping: (id, getRobotStore) => {
    stopExploration(id, getRobotStore);

    const grid = get().mapperInstances[id];
    const robotStore = getRobotStore();
    const robot = robotStore.robots[id];
    const adapter = robot?.adapter;
    if (adapter && !get().localizationActive[id]) {
      adapter.setArchitectureProfile('hybrid');
    }

    // Auto-save vào savedMaps nếu có data
    if (grid && grid.scanCount > 0) {
      const mapEntry = {
        id: `map_${Date.now()}`,
        name: `Map ${new Date().toLocaleDateString('vi-VN')} ${new Date().toLocaleTimeString('vi-VN', { hour: '2-digit', minute: '2-digit' })}`,
        robotId: id,
        createdAt: Date.now(),
        scanCount: grid.scanCount,
        width: grid.width,
        height: grid.height,
        resolution: grid.resolution,
        data: grid.exportJSON(),
      };
      const maps = [...get().savedMaps, mapEntry];
      set({ mappingActive: { ...get().mappingActive, [id]: false }, savedMaps: maps });
      saveMapsToStorage(maps);
      console.log(`[Mapping] Dừng quét + Tự động lưu "${mapEntry.name}" (${grid.scanCount} scans)`);
    } else {
      set((state) => ({
        mappingActive: { ...state.mappingActive, [id]: false },
      }));
      console.log(`[Mapping] Dừng quét (không có dữ liệu để lưu)`);
    }
  },

  // ============================================================
  //   LOCALIZATION (AMCL)
  // ============================================================

  startLocalization: (id, getRobotStore) => {
    const robotStore = getRobotStore();
    const robot = robotStore.robots[id];
    const adapter = robot?.adapter;
    if (adapter) adapter.setArchitectureProfile('pc_slam');

    set((state) => ({
      localizationActive: { ...state.localizationActive, [id]: true },
      mappingActive: { ...state.mappingActive, [id]: false },
      mapToOdom: { ...state.mapToOdom, [id]: { dx: 0, dy: 0, dTheta: 0 } },
    }));
    console.log(`[Localization] Bắt đầu AMCL mode cho robot ${id}`);
  },

  stopLocalization: (id, getRobotStore) => {
    const robotStore = getRobotStore();
    const robot = robotStore.robots[id];
    const adapter = robot?.adapter;
    if (adapter) adapter.setArchitectureProfile('hybrid');

    set((state) => ({
      localizationActive: { ...state.localizationActive, [id]: false },
    }));
    console.log(`[Localization] Dừng AMCL mode cho robot ${id}`);
  },

  // ============================================================
  //   SLAM TICK (Called from telemetry callback)
  // ============================================================

  /**
   * Cập nhật Lidar SLAM (gọi từ telemetry callback)
   */
  processLidarTick: (id, telem, now, getRobotStore) => {
    const state = get();
    const isMapping = state.mappingActive[id];
    const isLocalizing = state.localizationActive?.[id];

    if ((isMapping || isLocalizing) && state.mapperInstances[id]) {
      const grid = state.mapperInstances[id];
      const lidarPts = telem.lidar || [];
      if (lidarPts.length > 0) {
        const headingDeg = telem.heading ?? 0;
        const odomX = telem.x ?? 0;
        const odomY = telem.y ?? 0;
        const odomTheta = headingDeg * Math.PI / 180;

        const tf = state.mapToOdom[id] || { dx: 0, dy: 0, dTheta: 0 };
        let mapX = odomX + tf.dx;
        let mapY = odomY + tf.dy;
        let mapTheta = odomTheta + tf.dTheta;
        if (navWorkerApi && (grid.scanCount >= 3 || isLocalizing) && !state.isMatching[id]) {
          set((s) => ({ isMatching: { ...s.isMatching, [id]: true } }));

          navWorkerApi.matchScan(id, grid.serialize(), mapX, mapY, mapTheta, lidarPts).then(matched => {
            const freshState = get();
            
            if (matched.corrected) {
              const c = matched.correction;
              const freshTf = freshState.mapToOdom[id] || { dx: 0, dy: 0, dTheta: 0 };
              const newTf = {
                dx: freshTf.dx + c.dx,
                dy: freshTf.dy + c.dy,
                dTheta: freshTf.dTheta + c.dTheta,
              };

              set((s) => ({
                mapToOdom: { ...s.mapToOdom, [id]: newTf },
                isMatching: { ...s.isMatching, [id]: false }
              }));
            } else {
              set((s) => ({ isMatching: { ...s.isMatching, [id]: false } }));
            }
          }).catch(e => {
            console.error("[SLAM] Worker Error:", e);
            set((s) => ({ isMatching: { ...s.isMatching, [id]: false } }));
          });
        }

        if (isMapping) {
          grid.updateFromScan(mapX, mapY, mapTheta, lidarPts);
        }

        if (!state._lastGridUpdate || now - state._lastGridUpdate > 500) {
          set((s) => ({
            _lastGridUpdate: now,
            occupancyGrid: { ...s.occupancyGrid, [id]: grid },
          }));
        }
      }
    }
  },

  setSlamBusy: (id, busy) => {
    set((s) => ({ isMatching: { ...s.isMatching, [id]: busy } }));
  },

  updateOccupancyGrid: (id, grid) => {
    set((s) => ({ occupancyGrid: { ...s.occupancyGrid, [id]: grid } }));
  },

  // ============================================================
  //   MAP CRUD
  // ============================================================

  /** Lưu map ra JSON + download file */
  saveMap: (id) => {
    const grid = get().mapperInstances[id];
    if (!grid) {
      console.warn('[Mapping] Không có map để lưu');
      return null;
    }

    const json = grid.exportJSON();
    const blob = new Blob([JSON.stringify(json, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `amr_map_${id}_${Date.now()}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    console.log(`[Mapping] Đã lưu map: ${grid.scanCount} scans, ${grid.width}x${grid.height}`);
    return json;
  },

  /** Load map từ JSON */
  loadMap: (id, jsonData) => {
    try {
      const grid = OccupancyGrid.importJSON(jsonData);
      set((state) => ({
        mapperInstances: { ...state.mapperInstances, [id]: grid },
        occupancyGrid: { ...state.occupancyGrid, [id]: grid },
        mappingActive: { ...state.mappingActive, [id]: false },
      }));
      console.log(`[Mapping] Đã load map: ${grid.scanCount} scans`);
      return true;
    } catch (err) {
      console.error('[Mapping] Lỗi load map:', err);
      return false;
    }
  },

  getSavedMaps: () => get().savedMaps,

  deleteSavedMap: (mapId) => {
    const maps = get().savedMaps.filter(m => m.id !== mapId);
    set({ savedMaps: maps });
    saveMapsToStorage(maps);
    console.log(`[MapManager] Đã xóa map ${mapId}`);
  },

  renameSavedMap: (mapId, newName) => {
    const maps = get().savedMaps.map(m =>
      m.id === mapId ? { ...m, name: newName } : m
    );
    set({ savedMaps: maps });
    saveMapsToStorage(maps);
  },

  loadSavedMap: (mapId, robotId) => {
    const mapEntry = get().savedMaps.find(m => m.id === mapId);
    if (!mapEntry) return false;
    return get().loadMap(robotId, mapEntry.data);
  },

  /** Tự động load map mới nhất đã lưu cho robot */
  autoLoadLatestMap: (robotId) => {
    const maps = get().savedMaps;
    if (!maps || maps.length === 0) return false;
    
    // Tìm map mới nhất (sort theo createdAt giảm dần)
    const sortedMaps = [...maps].sort((a, b) => b.createdAt - a.createdAt);
    const latestMap = sortedMaps[0];
    
    console.log(`[MapManager] Tự động load map mới nhất: "${latestMap.name}" cho robot ${robotId}`);
    return get().loadMap(robotId, latestMap.data);
  },

  /**
   * Tạo bản đồ tĩnh từ world segments (SIM mode).
   * Không cần LiDAR quét — bản đồ kho hàng đã biết trước.
   * @param {string} robotId 
   * @param {Array<{x1,y1,x2,y2}>} segments - Wall segments từ SimEngine
   */
  generateSimMap: (robotId, segments) => {
    if (!segments || segments.length === 0) {
      console.warn('[MapManager] generateSimMap: no segments');
      return false;
    }
    
    const grid = OccupancyGrid.createFromSegments(segments, 0.1, 1.0);
    if (!grid) return false;

    set((state) => ({
      mapperInstances: { ...state.mapperInstances, [robotId]: grid },
      occupancyGrid: { ...state.occupancyGrid, [robotId]: grid },
      mappingActive: { ...state.mappingActive, [robotId]: false },
    }));

    console.log(`[MapManager] ✅ Bản đồ kho hàng SIM đã sẵn sàng cho robot ${robotId}`);
    return true;
  },

  exportSavedMap: (mapId) => {
    const mapEntry = get().savedMaps.find(m => m.id === mapId);
    if (!mapEntry) return;
    const blob = new Blob([JSON.stringify(mapEntry.data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${mapEntry.name.replace(/\s+/g, '_')}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  },

  importMapFromJSON: (jsonData, name = null) => {
    try {
      if (!jsonData || (jsonData.version !== 1 && jsonData.version !== 2)) {
        throw new Error('Invalid map format');
      }

      const mapEntry = {
        id: `map_${Date.now()}`,
        name: name || `Imported ${new Date().toLocaleDateString('vi-VN')}`,
        robotId: 'imported',
        createdAt: jsonData.timestamp || Date.now(),
        scanCount: jsonData.scanCount || 0,
        width: jsonData.width,
        height: jsonData.height,
        resolution: jsonData.resolution,
        data: jsonData,
      };
      const maps = [...get().savedMaps, mapEntry];
      set({ savedMaps: maps });
      saveMapsToStorage(maps);
      console.log(`[MapManager] Imported map: ${mapEntry.name}`);
      return true;
    } catch (err) {
      console.error('[MapManager] Import error:', err);
      return false;
    }
  },
}));

// Register mapStore getter with exploration module
_setMapStoreGetter(() => useMapStore.getState());

export default useMapStore;
