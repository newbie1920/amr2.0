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

    // Lắng nghe telemetry
    connection.onTelemetry = (telem) => {
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

  // Getters
  getRobot: (id) => get().robots[id],
  getConnectedRobots: () => Object.values(get().robots).filter(r => r.status === 'connected'),
  getRobotList: () => Object.values(get().robots),
}));

export default useRobotStore;
