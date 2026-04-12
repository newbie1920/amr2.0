/**
 * AMR 2.0 — Fleet Logic
 * Quản lý đội xe: chọn xe tối ưu, phân công nhiệm vụ, tránh va chạm
 */

import { distance, findNearestCharger, GATES, degToRad } from './warehouse.js';
import { findPath } from './pathfinder.js';
import { TrajectoryController } from './trajectory.js';

// ============================================================
//   CONSTANTS
// ============================================================

/** Ngưỡng pin để tự đi sạc */
export const LOW_BATTERY_THRESHOLD = 20;

/** Ngưỡng pin tối thiểu để nhận nhiệm vụ */
export const MIN_BATTERY_FOR_TASK = 25;

/** Khoảng cách tối thiểu giữa 2 robot (tránh va chạm) */
export const COLLISION_DISTANCE = 0.4; // mét

// ============================================================
//   FLEET MANAGER
// ============================================================

/**
 * FleetManager — Quản lý đội xe AMR
 */
export class FleetManager {
  constructor() {
    this.robots = new Map();           // robotId → RobotState
    this.tasks = [];                   // Danh sách nhiệm vụ
    this.taskIdCounter = 1;
    this.trajectoryControllers = new Map(); // robotId → TrajectoryController
    this.controlLoopTimer = null;
  }

  // ============================================================
  //   ROBOT MANAGEMENT
  // ============================================================

  /**
   * Thêm robot vào fleet
   * @param {string} id 
   * @param {RobotConnection} connection 
   */
  addRobot(id, connection) {
    const state = {
      id,
      connection,
      status: 'idle',       // idle | working | charging | error | offline
      currentTaskId: null,
      position: { x: 0, y: 0 },
      heading: 0,
      battery: 100,
    };

    this.robots.set(id, state);
    this.trajectoryControllers.set(id, new TrajectoryController());

    // Lắng nghe telemetry
    connection.onTelemetry = (telem) => {
      state.position = { x: telem.x, y: telem.y };
      state.heading = telem.headingRad;
      state.battery = telem.battery;

      // Tự động đi sạc khi pin thấp
      if (telem.battery < LOW_BATTERY_THRESHOLD && state.status !== 'charging') {
        this._autoCharge(id);
      }
    };

    connection.onDisconnect = () => {
      state.status = 'offline';
    };

    connection.onConnect = () => {
      if (state.status === 'offline') {
        state.status = 'idle';
      }
    };
  }

  /**
   * Xóa robot khỏi fleet
   */
  removeRobot(id) {
    const state = this.robots.get(id);
    if (state) {
      state.connection.disconnect();
      this.robots.delete(id);
      this.trajectoryControllers.delete(id);
    }
  }

  // ============================================================
  //   TASK MANAGEMENT
  // ============================================================

  /**
   * Tạo nhiệm vụ nhập hàng
   * @param {string} slotId - ID ô kệ đích (VD: 's1_l2_1')
   * @param {object} orderInfo - Thông tin đơn hàng
   * @returns {object} task
   */
  createImportTask(slotId, orderInfo = {}) {
    const task = {
      id: this.taskIdCounter++,
      type: 'import',
      slotId,
      orderInfo,
      status: 'pending',      // pending | assigned | in_progress | completed | failed
      assignedRobotId: null,
      steps: [],
      currentStepIdx: 0,
      createdAt: Date.now(),
      completedAt: null,
    };

    this.tasks.push(task);
    return task;
  }

  /**
   * Tạo nhiệm vụ xuất hàng
   */
  createExportTask(slotId, orderInfo = {}) {
    const task = {
      id: this.taskIdCounter++,
      type: 'export',
      slotId,
      orderInfo,
      status: 'pending',
      assignedRobotId: null,
      steps: [],
      currentStepIdx: 0,
      createdAt: Date.now(),
      completedAt: null,
    };

    this.tasks.push(task);
    return task;
  }

  /**
   * Tự động chọn xe và giao nhiệm vụ
   * @param {number} taskId 
   * @returns {boolean} success
   */
  assignTask(taskId) {
    const task = this.tasks.find(t => t.id === taskId);
    if (!task || task.status !== 'pending') return false;

    // Tìm xe tối ưu
    const gate = task.type === 'import' ? GATES.import : GATES.export;
    const bestRobot = this._findBestRobot(gate.x, gate.y);

    if (!bestRobot) {
      console.warn('[Fleet] Không tìm được xe phù hợp!');
      return false;
    }

    // Tạo chuỗi steps cho nhiệm vụ
    task.assignedRobotId = bestRobot.id;
    task.status = 'assigned';
    task.steps = this._generateTaskSteps(task, bestRobot);
    task.currentStepIdx = 0;

    bestRobot.status = 'working';
    bestRobot.currentTaskId = taskId;

    console.log(`[Fleet] Giao nhiệm vụ #${taskId} cho ${bestRobot.id}`);
    return true;
  }

  /**
   * Bắt đầu thực hiện nhiệm vụ (chạy control loop)
   */
  startTask(taskId) {
    const task = this.tasks.find(t => t.id === taskId);
    if (!task || task.status !== 'assigned') return false;

    task.status = 'in_progress';
    this._executeTaskStep(task);
    return true;
  }

  // ============================================================
  //   CONTROL LOOP — Gửi cmd_vel xuống robot
  // ============================================================

  /**
   * Bắt đầu control loop (20Hz)
   * Gọi 1 lần khi app khởi động
   */
  startControlLoop() {
    if (this.controlLoopTimer) return;

    this.controlLoopTimer = setInterval(() => {
      this._controlLoopTick();
    }, 50); // 20Hz
  }

  stopControlLoop() {
    if (this.controlLoopTimer) {
      clearInterval(this.controlLoopTimer);
      this.controlLoopTimer = null;
    }
  }

  _controlLoopTick() {
    // Cho mỗi robot đang có trajectory active
    for (const [robotId, tc] of this.trajectoryControllers) {
      if (tc.state === 'idle' || tc.state === 'done') continue;

      const robot = this.robots.get(robotId);
      if (!robot || !robot.connection.connected) continue;

      // Update trajectory controller với odometry hiện tại
      const cmd = tc.update(robot.position.x, robot.position.y, robot.heading);

      // Gửi cmd_vel xuống ESP32
      robot.connection.sendVelocity(cmd.linear, cmd.angular);

      // Kiểm tra hoàn thành step
      if (cmd.done) {
        this._onStepCompleted(robotId);
      }
    }

    // Kiểm tra va chạm giữa các robot
    this._checkCollisions();
  }

  // ============================================================
  //   INTERNAL LOGIC
  // ============================================================

  /**
   * Tìm robot tối ưu:
   * 1. Đang rảnh (idle)
   * 2. Đủ pin (>25%)
   * 3. Gần cổng nhất
   */
  _findBestRobot(gateX, gateY) {
    let best = null;
    let bestDist = Infinity;

    for (const [, robot] of this.robots) {
      if (robot.status !== 'idle') continue;
      if (!robot.connection.connected) continue;
      if (robot.battery < MIN_BATTERY_FOR_TASK) continue;

      const d = distance(robot.position.x, robot.position.y, gateX, gateY);
      if (d < bestDist) {
        bestDist = d;
        best = robot;
      }
    }

    return best;
  }

  /**
   * Tạo chuỗi steps cho nhiệm vụ
   */
  _generateTaskSteps(task, robot) {
    const steps = [];

    if (task.type === 'import') {
      // Bước 1: Đi đến cổng nhập
      steps.push({
        action: 'move',
        target: { x: GATES.import.x, y: GATES.import.y },
        heading: degToRad(GATES.import.heading),
        description: 'Đi đến cổng nhập hàng',
      });

      // Bước 2: Bốc hàng (chờ)
      steps.push({
        action: 'wait',
        duration: 3000,
        description: 'Bốc hàng tại cổng nhập',
      });

      // Bước 3: Đi đến kệ (approach point)
      // Tìm approach point của slot đích
      const slotInfo = this._findSlot(task.slotId);
      if (slotInfo) {
        steps.push({
          action: 'move',
          target: slotInfo.approach,
          heading: degToRad(slotInfo.heading),
          description: `Đi đến ${slotInfo.name}`,
        });

        // Bước 4: Đặt hàng (chờ)
        steps.push({
          action: 'wait',
          duration: 3000,
          description: `Đặt hàng lên ${slotInfo.name}`,
        });
      }
    } else if (task.type === 'export') {
      // Bước 1: Đi đến kệ
      const slotInfo = this._findSlot(task.slotId);
      if (slotInfo) {
        steps.push({
          action: 'move',
          target: slotInfo.approach,
          heading: degToRad(slotInfo.heading),
          description: `Đi đến ${slotInfo.name}`,
        });

        // Bước 2: Bốc hàng
        steps.push({
          action: 'wait',
          duration: 3000,
          description: `Bốc hàng từ ${slotInfo.name}`,
        });
      }

      // Bước 3: Đi đến cổng xuất
      steps.push({
        action: 'move',
        target: { x: GATES.export.x, y: GATES.export.y },
        heading: degToRad(GATES.export.heading),
        description: 'Đi đến cổng xuất hàng',
      });

      // Bước 4: Đặt hàng
      steps.push({
        action: 'wait',
        duration: 3000,
        description: 'Đặt hàng tại cổng xuất',
      });
    }

    return steps;
  }

  /**
   * Thực hiện step hiện tại của task
   */
  _executeTaskStep(task) {
    if (task.currentStepIdx >= task.steps.length) {
      // Nhiệm vụ hoàn thành
      task.status = 'completed';
      task.completedAt = Date.now();
      const robot = this.robots.get(task.assignedRobotId);
      if (robot) {
        robot.status = 'idle';
        robot.currentTaskId = null;
      }
      console.log(`[Fleet] ✅ Nhiệm vụ #${task.id} hoàn thành!`);
      return;
    }

    const step = task.steps[task.currentStepIdx];
    const robot = this.robots.get(task.assignedRobotId);
    if (!robot) return;

    console.log(`[Fleet] Bước ${task.currentStepIdx + 1}/${task.steps.length}: ${step.description}`);

    if (step.action === 'move') {
      // Tìm đường và giao cho trajectory controller
      const result = findPath(
        robot.position.x, robot.position.y,
        step.target.x, step.target.y
      );

      if (result.success) {
        const tc = this.trajectoryControllers.get(task.assignedRobotId);
        tc.setPath(result.path, step.heading);
      } else {
        console.error(`[Fleet] Không tìm được đường cho bước ${task.currentStepIdx + 1}!`);
        task.status = 'failed';
      }
    } else if (step.action === 'wait') {
      // Chờ + mô phỏng bốc/đặt hàng
      setTimeout(() => {
        this._onStepCompleted(task.assignedRobotId);
      }, step.duration);
    }
  }

  /**
   * Khi hoàn thành 1 step → chuyển sang step tiếp theo
   */
  _onStepCompleted(robotId) {
    const robot = this.robots.get(robotId);
    if (!robot || !robot.currentTaskId) return;

    const task = this.tasks.find(t => t.id === robot.currentTaskId);
    if (!task || task.status !== 'in_progress') return;

    task.currentStepIdx++;
    this._executeTaskStep(task);
  }

  /**
   * Tự động đi sạc
   */
  _autoCharge(robotId) {
    const robot = this.robots.get(robotId);
    if (!robot || robot.status === 'charging') return;

    const charger = findNearestCharger(robot.position.x, robot.position.y);
    if (!charger) return;

    console.log(`[Fleet] ⚡ ${robotId} pin yếu (${robot.battery}%), đi sạc tại ${charger.name}`);

    robot.status = 'charging';

    const result = findPath(
      robot.position.x, robot.position.y,
      charger.x, charger.y
    );

    if (result.success) {
      const tc = this.trajectoryControllers.get(robotId);
      tc.setPath(result.path, degToRad(charger.heading));
    }
  }

  /**
   * Kiểm tra va chạm giữa các robot
   */
  _checkCollisions() {
    const activeRobots = [];
    for (const [id, robot] of this.robots) {
      if (robot.connection.connected && robot.status !== 'offline') {
        activeRobots.push({ id, ...robot.position });
      }
    }

    for (let i = 0; i < activeRobots.length; i++) {
      for (let j = i + 1; j < activeRobots.length; j++) {
        const d = distance(
          activeRobots[i].x, activeRobots[i].y,
          activeRobots[j].x, activeRobots[j].y
        );

        if (d < COLLISION_DISTANCE) {
          // Robot có priority thấp hơn (index cao hơn) dừng lại
          const robotToStop = this.robots.get(activeRobots[j].id);
          if (robotToStop) {
            robotToStop.connection.sendStop();
            console.warn(`[Fleet] ⚠️ Va chạm! Dừng ${activeRobots[j].id}`);
          }
        }
      }
    }
  }

  /**
   * Tìm thông tin slot kệ
   */
  _findSlot(slotId) {
    for (const shelf of SHELVES) {
      for (const level of shelf.levels) {
        for (const slot of level.slots) {
          if (slot.id === slotId) {
            return {
              approach: slot.approach,
              heading: slot.heading,
              name: slot.name,
            };
          }
        }
      }
    }
    return null;
  }
}

export default FleetManager;
