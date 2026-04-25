/**
 * AMR 2.0 — Robot Store (Zustand)
 * Quản lý state cho danh sách robots + kết nối WebSocket
 */

import { create } from 'zustand';
import { RobotConnection } from '../core/robotProtocol.js';

function saveRobotsToStorage(robots) {
  const data = Object.values(robots).map((r) => ({
    id: r.id,
    name: r.name,
    ip: r.ip,
    port: r.port,
  }));
  localStorage.setItem('amr_robots', JSON.stringify(data));
}

const useRobotStore = create((set, get) => ({
  // State
  robots: {},           // { robotId: { id, name, ip, port, connection, telemetry, status } }
  lidarScans: {},      // { robotId: [{a, d}] }
  occupancyGrid: {},   // { robotId: { width, height, resolution, data: Uint8Array } }
  selectedRobotId: null,

  // ============================================================
  //   ACTIONS
  // ============================================================

  loadStoredRobots: () => {
    const saved = JSON.parse(localStorage.getItem('amr_robots') || '[]');
    saved.forEach((r) => {
      if (!get().robots[r.id]) {
        get().addRobot(r.name, r.ip, r.port, r.id);
      }
    });
  },

  /**
   * Thêm robot mới
   */
  addRobot: (name, ip, port = 81, forcedId = null) => {
    const id = forcedId || `robot_${Date.now()}`;
    const connection = new RobotConnection(ip, port, name);
    // Gắn id robot để các callback có thể truy cập store
    connection.robotId = id;

    let lastUpdate = 0;
    // Lắng nghe telemetry
    connection.onTelemetry = (telem) => {
      const now = Date.now();
      
      // Transient state update (For Canvas/3D without rendering UI)
      const currentState = get();
      if (!currentState.transientRobots) currentState.transientRobots = {};
      currentState.transientRobots[id] = telem;
      
      // Throttle UI update to ~1Hz (1000ms) to rescue React re-render cycle
      if (now - lastUpdate > 1000) {
        lastUpdate = now;
        set((state) => {
          if (!state.robots[id]) return state;
          return {
            robots: {
              ...state.robots,
              [id]: {
                ...state.robots[id],
                telemetry: { ...telem },
              },
            },
          };
        });
      }

      // Sync Navigation Status with Task System (replaces older setInterval in UI)
      if (telem.nav === 'ERROR' || telem.nav === 'DONE') {
         // Dynamically import to avoid circular dependency issues at boot
         import('./taskStore.js').then(module => {
            const useTaskStore = module.default;
            const state = useTaskStore.getState();
            const activeTasks = state.tasks.filter(t => t.status === 'in_progress' && t.assignedRobotId === id && !t.dbUpdated);
            if (activeTasks.length > 0) {
              state.processTaskCompletion(activeTasks[0].id, telem.nav, 'Robot navigation error - timeout or stuck');
            }
         });
      }
    };

    connection.onConnect = () => {
      set((state) => {
        if (!state.robots[id]) return state;
        return {
          robots: {
            ...state.robots,
            [id]: { ...state.robots[id], status: 'connected' },
          },
        }
      });
    };

    connection.onDisconnect = () => {
      set((state) => {
        if (!state.robots[id]) return state;
        return {
          robots: {
            ...state.robots,
            [id]: { ...state.robots[id], status: 'disconnected' },
          },
        }
      });
    };

    set((state) => {
      const newRobots = {
        ...state.robots,
        [id]: {
          id,
          name,
          ip,
          port,
          connection,
          status: 'disconnected',
          telemetry: {
            x: 0, y: 0, heading: 0, headingRad: 0,
            distance: 0, linearVel: 0, battery: 100,
            imuAvailable: false, imuCalibrated: false,
          },
          currentTask: null,
          taskStatus: 'idle', // idle | working | charging
        },
      };
      saveRobotsToStorage(newRobots);
      return { robots: newRobots };
    });

    return id;
  },

  /**
   * Xóa robot
   */
  removeRobot: (id) => {
    const robot = get().robots[id];
    if (robot) {
      robot.connection.disconnect();
      set((state) => {
        const { [id]: removed, ...rest } = state.robots;
        saveRobotsToStorage(rest);
        return { robots: rest, selectedRobotId: state.selectedRobotId === id ? null : state.selectedRobotId };
      });
    }
  },

  /**
   * Kết nối robot
   */
  connectRobot: (id) => {
    const robot = get().robots[id];
    if (robot) {
      robot.connection.connect();
      set((state) => ({
        robots: {
          ...state.robots,
          [id]: { ...state.robots[id], status: 'connecting' },
        },
      }));
    }
  },

  /**
   * Ngắt kết nối robot
   */
  disconnectRobot: (id) => {
    const robot = get().robots[id];
    if (robot) {
      robot.connection.disconnect();
      set((state) => ({
        robots: {
          ...state.robots,
          [id]: { ...state.robots[id], status: 'disconnected' },
        },
      }));
    }
  },

  /**
   * Chọn robot
   */
  selectRobot: (id) => set({ selectedRobotId: id }),

  /**
   * Gửi lệnh điều khiển tay
   */
  sendManualControl: (id, linear, angular) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.sendVelocity(linear, angular);
    }
  },

  // Cập nhật dữ liệu LIDAR cho robot
  updateLidar: (id, scan) => {
    set((state) => {
      const newScans = { ...state.lidarScans, [id]: scan };
      // Cập nhật telemetry.lidar để UI có thể dùng
      const robot = state.robots[id];
      if (robot) {
        robot.telemetry = { ...robot.telemetry, lidar: scan };
      }
      return { lidarScans: newScans, robots: { ...state.robots } };
    });
  },

  /**
   * Dừng robot
   */
  stopRobot: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.sendStop();
    }
  },

  /**
   * Reset odometry
   */
  resetOdometry: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.resetOdometry();
    }
  },

  /**
   * Set specific pose
   */
  setPose: (id, x, y, theta) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.setPose(x, y, theta);
    }
  },

  /**
   * Gửi lộ trình tự lái cho robot
   * @param {string} id - Robot ID
   * @param {Array<{x,y}>} path - Danh sách waypoint
   * @param {number|null} finalHeading - Góc cuối tại đích (độ)
   */
  navigateRobot: (id, path, finalHeading = null) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.navigate(path, finalHeading);
    }
  },

  /**
   * Dừng tự lái
   */
  navStopRobot: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.navStop();
    }
  },

  pauseRobot: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.pause();
    }
  },

  resumeRobot: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.resume();
    }
  },

  /**
   * Recalibrate con quay hồi chuyển (robot phải đứng yên!)
   */
  recalibrateGyro: (id) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.recalibrateGyro();
    }
  },

  /**
   * Bật/tắt chế độ khóa phanh khẩn cấp
   */
  setBrake: (id, enabled) => {
    const robot = get().robots[id];
    if (robot && robot.connection.connected) {
      robot.connection.setBrake(enabled);
    }
  },

  // Getters
  getRobot: (id) => get().robots[id],
  getConnectedRobots: () => Object.values(get().robots).filter(r => r.status === 'connected'),
  getRobotList: () => Object.values(get().robots),
}));

export default useRobotStore;
