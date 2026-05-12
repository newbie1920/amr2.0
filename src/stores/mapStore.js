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
import { simNavWorkerApi } from '../core/navWorkerSetup.js';
import { getRobotStoreState } from './storeRegistry.js';

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

    const conn = robot?.adapter || robot?.connection;  // Fallback to connection for real robots
    const isHitl = robot?.connection?.hitlEnabled || telem.hitl;

    // Set architecture profile — MUST reach ESP32 before explore command
    if (conn) {
      if (isHitl || telem.onboardNavEnabled) {
        // HITL / Onboard mode requires hybrid architecture for onboard SLAM
        if (typeof conn.setArchitectureProfile === 'function') {
          conn.setArchitectureProfile('hybrid');
        }
      } else {
        // PC SLAM overrides architecture
        if (typeof conn.setArchitectureProfile === 'function') {
          conn.setArchitectureProfile('pc_slam');
        }
      }
    }

    const grid = new OccupancyGrid(200, 200, 0.1, rx, ry);
    set((state) => ({
      mappingActive: { ...state.mappingActive, [id]: true },
      mapperInstances: { ...state.mapperInstances, [id]: grid },
      occupancyGrid: { ...state.occupancyGrid, [id]: grid },
      mapToOdom: { ...state.mapToOdom, [id]: { dx: 0, dy: 0, dTheta: 0 } },
    }));
    console.log(`[Mapping] Bắt đầu quét map cho robot ${id} tại (${rx.toFixed(2)}, ${ry.toFixed(2)}), grid 200x200 @0.1m`);

    if (isHitl || telem.onboardNavEnabled) {
      // ESP32 Onboard SLAM: Send 'explore' command
      if (robot?.connection?.connected) {
        // 1. Reset hitlEngine to safe spawn position FIRST
        const spawnX = 3.5, spawnY = 2.0, spawnTheta = Math.PI / 2;
        if (isHitl && robot.connection.hitlEngine) {
          robot.connection.hitlEngine.reset(spawnX, spawnY, spawnTheta);
          console.log(`[Mapping] HITL engine reset to (${spawnX}, ${spawnY})`);
        }

        // 2. Sync ESP32 pose to match hitlEngine spawn (so odometry + grid mapper align)
        robot.connection._send({ cmd: 'set_pose', x: spawnX, y: spawnY, theta: spawnTheta });

        // 3. Small delay to let architecture profile and pose propagate, then send explore
        setTimeout(() => {
          robot.connection._send({ cmd: "explore" });
          console.log(`[Mapping] Đã gửi lệnh 'explore' cho ESP32 (Onboard SLAM)`);
        }, 200);
      }
    } else {
      // Web-based PC SLAM Exploration
      startExploration(id, getRobotStore);

      // Update exploration info mỗi giây cho UI
      const infoTimer = setInterval(() => {
        if (!get().mappingActive[id]) { clearInterval(infoTimer); return; }
        set((s) => ({ explorationInfo: { ...s.explorationInfo, [id]: getExplorationInfo() } }));
      }, 1000);
    }
  },

  /**
   * Dừng quét + TỰ ĐỘNG lưu map vào danh sách
   */
  stopMapping: (id, getRobotStore) => {
    const robotStore = getRobotStore();
    const robot = robotStore.robots[id];
    const isHitl = robot?.connection?.hitlEnabled || robot?.telemetry?.hitl;

    if (isHitl || robot?.telemetry?.onboardNavEnabled) {
      if (robot?.connection?.connected) {
        robot.connection._send({ cmd: "explore_stop" });
        robot.connection.navStop(); // Force physical halt on the hardware
        console.log(`[Mapping] Đã gửi lệnh 'explore_stop' và 'navStop' cho ESP32`);
      }
    } else {
      stopExploration(id, getRobotStore);
    }

    const grid = get().mapperInstances[id];
    const adapter = robot?.adapter;
    if (adapter && !get().localizationActive[id] && !isHitl) {
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
      const safeSavedMaps = Array.isArray(get().savedMaps) ? get().savedMaps : [];
      const maps = [...safeSavedMaps, mapEntry];
      set({ mappingActive: { ...get().mappingActive, [id]: false }, savedMaps: maps });
      saveMapsToStorage(maps);
      console.log(`[Mapping] Dừng quét + Tự động lưu "${mapEntry.name}" (${grid.scanCount} scans)`);

      // Tự động push map xuống ESP32 để xe tự chạy được
      setTimeout(() => get().pushMapToRobot(id, getRobotStore), 500);
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

    // Detect HITL mode — sim engine provides ground-truth pose,
    // so scan matching (ICP/CSM) and TF correction must be DISABLED
    // to prevent a "ghost" duplicate map from appearing at an offset.
    const robotStore = getRobotStore();
    const robot = robotStore.robots?.[id];
    const isHitl = robot?.connection?.hitlEnabled || telem.hitl;

    const isSim = robot?._sim;

    // ── AUTO-CREATE passive grid for REAL robots on first lidar data ──
    // This ensures the map starts building immediately when robot connects
    const lidarPtsCheck = telem.lidar || [];
    if (!state.mapperInstances[id] && !isSim && lidarPtsCheck.length > 0) {
      const initX = telem.x ?? 0;
      const initY = telem.y ?? 0;
      const grid = new OccupancyGrid(200, 200, 0.1, initX, initY);
      set((s) => ({
        mapperInstances: { ...s.mapperInstances, [id]: grid },
        occupancyGrid: { ...s.occupancyGrid, [id]: grid },
        mapToOdom: { ...s.mapToOdom, [id]: { dx: 0, dy: 0, dTheta: 0 } },
      }));
      console.log(`[Mapping] 🗺️ Auto-created passive grid for robot ${id} at (${initX.toFixed(2)}, ${initY.toFixed(2)})`);
      return; // Grid will be available on next tick
    }

    // Passive mapping: always update grid when scan data arrives.
    // isMapping flag only controls exploration auto-drive behavior.
    if (state.mapperInstances[id]) {
      const grid = state.mapperInstances[id];
      const lidarPts = telem.lidar || [];
      if (lidarPts.length > 0) {
        const headingDeg = telem.heading ?? 0;
        const odomX = telem.x ?? 0;
        const odomY = telem.y ?? 0;
        const odomTheta = headingDeg * Math.PI / 180;

        // ── REAL ROBOT: ESP32 sends pre-built grid via binary WS (onLidarGrid).
        //    Browser MUST NOT also run updateFromScan — two maps fighting = flicker + rotation.
        //    Only SIM/HITL robots need browser-side scan integration.
        const isRealRobot = !(isHitl || isSim);
        if (isRealRobot) {
          // Real robot: ESP32 grid is single source of truth.
          // Just update robot pose on the existing grid for visualization.
          grid.robotX = odomX;
          grid.robotY = odomY;
          grid.robotHeading = odomTheta;
          return; // Skip browser scan integration entirely
        }

        let mapX, mapY, mapTheta;
        if (isHitl || isSim) {
          // HITL & SIM: telem pose IS ground-truth.
          // No TF correction — applying it would create a duplicate map at an offset.
          mapX = odomX;
          mapY = odomY;
          mapTheta = odomTheta;
        } else if (telem.slamMapX != null && telem.slamMapY != null && telem.slamMapTheta != null) {
          // REAL robot with active SLAM: use firmware's ICP-corrected map pose.
          mapX = telem.slamMapX;
          mapY = telem.slamMapY;
          mapTheta = telem.slamMapTheta;
        } else {
          // REAL robot without SLAM data: fallback to odom + browser TF
          const tf = state.mapToOdom[id] || { dx: 0, dy: 0, dTheta: 0 };
          mapX = odomX + tf.dx;
          mapY = odomY + tf.dy;
          mapTheta = odomTheta + tf.dTheta;
        }

        // Scan matching: SKIP in HITL/SIM mode (sim pose is already ground truth)
        if (!(isHitl || isSim) && simNavWorkerApi && (grid.scanCount >= 3 || isLocalizing) && !state.isMatching[id]) {
          set((s) => ({ isMatching: { ...s.isMatching, [id]: true } }));

          simNavWorkerApi.matchScan(id, grid.serialize(), mapX, mapY, mapTheta, lidarPts).then(matched => {
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

        // SIM/HITL: integrate scans into grid (browser-side mapping)
        grid.updateFromScan(mapX, mapY, mapTheta, lidarPts);

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

      // Đẩy map xuống ESP32
      setTimeout(() => get().pushMapToRobot(id, getRobotStoreState), 500);
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

  // ============================================================
  //   MAP PUSH — Đẩy bản đồ tĩnh xuống ESP32 (1 lần)
  // ============================================================

  /**
   * Push bản đồ tĩnh xuống 1 robot ESP32 cụ thể
   * @param {string} robotId - ID robot cần push
   * @param {Function} getRobotStore - getter robotStore
   * @returns {boolean} true nếu push thành công
   */
  pushMapToRobot: (robotId, getRobotStore) => {
    const state = get();
    const grid = state.mapperInstances[robotId] || state.occupancyGrid[robotId];
    if (!grid) {
      console.warn(`[MapPush] No map available for robot ${robotId}`);
      return false;
    }

    const robotStore = getRobotStore();
    const robot = robotStore.robots[robotId];
    const conn = robot?.adapter || robot?.connection;
    if (!conn?.connected) {
      console.warn(`[MapPush] Robot ${robotId} not connected`);
      return false;
    }

    // Prefer sendMapData which builds proper header
    if (typeof conn.sendMapData === 'function') {
      const ok = conn.sendMapData(grid);
      if (ok) {
        console.log(`[MapPush] ✅ Map pushed to robot ${robotId}: ${grid.width}x${grid.height}`);
      }
      return ok;
    }

    console.warn(`[MapPush] Robot ${robotId} connection has no sendMapData method`);
    return false;
  },

  /**
   * Push bản đồ xuống TẤT CẢ robot đang kết nối
   * @param {Function} getRobotStore - getter robotStore
   */
  pushMapToAllRobots: (getRobotStore) => {
    const robotStore = getRobotStore();
    const robots = robotStore.robots;
    let pushed = 0;
    for (const [id, robot] of Object.entries(robots)) {
      const isConn = robot?.adapter?.connected ?? robot?.connection?.connected;
      if (isConn) {
        const ok = get().pushMapToRobot(id, getRobotStore);
        if (ok) pushed++;
      }
    }
    console.log(`[MapPush] Pushed map to ${pushed} robot(s)`);
    return pushed;
  },
}));


// Register mapStore getter with exploration module
_setMapStoreGetter(() => useMapStore.getState());

export default useMapStore;
