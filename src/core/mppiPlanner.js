/**
 * AMR 2.0 — MPPI (Model Predictive Path Integral) Local Planner
 * 
 * MPPI là thuật toán điều khiển dự đoán mô hình phi tuyến tính sử dụng
 * lấy mẫu Monte Carlo. So với DWA, MPPI sinh ra hàng trăm quỹ đạo
 * ngẫu nhiên thay vì quét cửa sổ tĩnh, giúp nó tránh local minima cực tốt
 * và tạo ra đường đi mượt mà hơn rất nhiều trong môi trường chật hẹp.
 */

import { normalizeAngle } from './mathUtils.js';
import { ROBOT_HALF_WIDTH, ROBOT_HALF_LENGTH } from './warehouse.js';

export const MPPI_DEFAULTS = {
  samples: 400,             // Số lượng quỹ đạo sinh ra (K)
  timeSteps: 15,            // Số bước thời gian (T)
  dt: 0.1,                  // Delta T mỗi bước (giây) -> horizon = 1.5s
  temperature: 0.1,         // Hệ số nhiệt độ (lambda) -> ảnh hưởng mức độ mượt
  
  // Giới hạn vật lý
  maxSpeedTrans: 0.5,
  minSpeedTrans: -0.1,      // Cho phép lùi nhẹ
  maxSpeedRot: 1.5,
  
  // Noise StdDev cho Monte Carlo
  noiseV: 0.2,              // StdDev nhiễu vận tốc tuyến tính
  noiseW: 0.4,              // StdDev nhiễu vận tốc góc
  
  // Trọng số tính điểm (Cost Function)
  costMapWeight: 200.0,     // Trọng số phạt nếu đi vào vùng inflation
  pathDistWeight: 10.0,     // Trọng số bám theo đường Global Plan
  goalDistWeight: 5.0,      // Trọng số tiến về đích
  headingWeight: 3.0,       // Trọng số hướng về đích
  collisionCost: 1e6,       // Cost vô cùng nếu va chạm
};

class MPPIPlanner {
  constructor(config = {}) {
    this.cfg = { ...MPPI_DEFAULTS, ...config };
    
    // Khởi tạo control sequence danh định (Nominal Control)
    // Mỗi phần tử là [v, w]
    this.nominalControl = Array.from({ length: this.cfg.timeSteps }, () => [0, 0]);
  }

  /**
   * Tính toán lệnh điều khiển (v, w) tối ưu
   */
  computeVelocityCmd(pose, globalPlan, grid) {
    if (!globalPlan || globalPlan.length === 0) {
      return { v: 0, w: 0, ok: false, reason: 'no_plan' };
    }

    const { samples, timeSteps, dt, temperature, noiseV, noiseW, collisionCost } = this.cfg;
    const localGoal = this._pickLocalGoal(globalPlan, pose, 1.5);
    
    const trajectories = [];
    const costs = new Float32Array(samples);
    let minCost = Infinity;

    // 1. Sinh ngẫu nhiên K quỹ đạo (Rollouts)
    for (let k = 0; k < samples; k++) {
      let x = pose.x;
      let y = pose.y;
      let theta = pose.theta;
      let trajectoryCost = 0;
      
      const traj = [];
      const noises = [];

      for (let t = 0; t < timeSteps; t++) {
        // Sinh nhiễu Gaussian
        const nv = this._gaussianRandom() * noiseV;
        const nw = this._gaussianRandom() * noiseW;
        noises.push([nv, nw]);

        // Áp dụng nhiễu vào lệnh điều khiển danh định
        let v = this.nominalControl[t][0] + nv;
        let w = this.nominalControl[t][1] + nw;

        // Clamp theo giới hạn vật lý
        v = Math.max(this.cfg.minSpeedTrans, Math.min(this.cfg.maxSpeedTrans, v));
        w = Math.max(-this.cfg.maxSpeedRot, Math.min(this.cfg.maxSpeedRot, w));

        // Kinematics forward model
        x += v * Math.cos(theta) * dt;
        y += v * Math.sin(theta) * dt;
        theta = normalizeAngle(theta + w * dt);

        traj.push({ x, y });

        // --- Đánh giá Cost (Cost Function) ---
        let stepCost = 0;

        // a) Costmap Penalty & Collision
        const cellCost = this._getCostmapCost(x, y, grid);
        if (cellCost >= 253) {
          stepCost += collisionCost; // Va chạm hoặc Inscribed
        } else if (cellCost > 0) {
          // Inflation cost
          stepCost += (cellCost / 253.0) * this.cfg.costMapWeight;
        }

        // b) Path tracking cost
        const distToPath = this._distanceToPath(x, y, globalPlan);
        stepCost += distToPath * this.cfg.pathDistWeight;

        // Ghi nhận cost
        trajectoryCost += stepCost;
      }

      // Terminal cost (Cost tại điểm cuối của quỹ đạo)
      const finalPt = traj[timeSteps - 1];
      const distToGoal = Math.hypot(localGoal.x - finalPt.x, localGoal.y - finalPt.y);
      const finalHeading = Math.atan2(localGoal.y - finalPt.y, localGoal.x - finalPt.x);
      const headingErr = Math.abs(normalizeAngle(finalHeading - theta));
      
      trajectoryCost += distToGoal * this.cfg.goalDistWeight;
      trajectoryCost += headingErr * this.cfg.headingWeight;

      costs[k] = trajectoryCost;
      trajectories.push({ traj, noises, cost: trajectoryCost });

      if (trajectoryCost < minCost) {
        minCost = trajectoryCost;
      }
    }

    // 2. Tính trọng số phân phối chuẩn (Path Integral Weights)
    let totalWeight = 0;
    const weights = new Float32Array(samples);
    for (let k = 0; k < samples; k++) {
      // Exponential weight, trừ đi minCost để tránh bị underflow
      weights[k] = Math.exp(-(costs[k] - minCost) / temperature);
      totalWeight += weights[k];
    }

    // 3. Cập nhật Nominal Control Sequence
    let optimalV = 0;
    let optimalW = 0;
    
    if (totalWeight > 1e-10) { // Đảm bảo không chia cho 0
      for (let t = 0; t < timeSteps; t++) {
        let sumNv = 0;
        let sumNw = 0;

        for (let k = 0; k < samples; k++) {
          const w_k = weights[k] / totalWeight;
          sumNv += w_k * trajectories[k].noises[t][0];
          sumNw += w_k * trajectories[k].noises[t][1];
        }

        // Cập nhật lệnh danh định với nhiễu đã được weighting
        this.nominalControl[t][0] += sumNv;
        this.nominalControl[t][1] += sumNw;

        // Clamp lại
        this.nominalControl[t][0] = Math.max(this.cfg.minSpeedTrans, Math.min(this.cfg.maxSpeedTrans, this.nominalControl[t][0]));
        this.nominalControl[t][1] = Math.max(-this.cfg.maxSpeedRot, Math.min(this.cfg.maxSpeedRot, this.nominalControl[t][1]));
      }

      // Lấy lệnh đầu tiên làm lệnh tối ưu hiện tại
      optimalV = this.nominalControl[0][0];
      optimalW = this.nominalControl[0][1];
      
      // Shift left control sequence (chuẩn bị cho bước sau)
      const lastCmd = [...this.nominalControl[timeSteps - 1]];
      this.nominalControl.shift();
      this.nominalControl.push(lastCmd); // Recycle lệnh cuối
    } else {
      // Toàn bộ quỹ đạo bị va chạm -> Tự động dừng
      return { v: 0, w: 0, ok: false, reason: 'all_trajectories_collided' };
    }

    // Diagnostic info
    // Lấy quỹ đạo tốt nhất để vẽ lên màn hình
    const bestTraj = trajectories.find(t => t.cost === minCost)?.traj || [];

    return {
      v: optimalV,
      w: optimalW,
      ok: true,
      trajectory: bestTraj,
      diag: {
        minCost: minCost.toFixed(1),
        optimalV: optimalV.toFixed(2),
        optimalW: optimalW.toFixed(2)
      }
    };
  }

  // --- Helpers ---

  _gaussianRandom() {
    let u = 0, v = 0;
    while(u === 0) u = Math.random();
    while(v === 0) v = Math.random();
    return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
  }

  _getCostmapCost(x, y, grid) {
    if (!grid || !grid.costmap) return 0;
    
    // Kiểm tra tâm robot
    const g = grid.worldToGrid(x, y);
    if (!grid.inBounds(g.gx, g.gy)) return 254; // Ngoài map -> Lethal

    const centerCost = grid.costmap[g.gy * grid.width + g.gx];
    if (centerCost >= 253) return centerCost; // Đã quá sát tường

    // Kiểm tra thêm 4 góc footprint (giảm tính toán so với check toàn bộ)
    const hw = ROBOT_HALF_WIDTH;
    const hl = ROBOT_HALF_LENGTH;
    const cosT = Math.cos(0); // Coi như hình tròn bán kính lớn nhất để tối ưu
    const sinT = Math.sin(0);
    
    // Bán kính bao tiếp (circumscribed radius)
    const circumRadius = Math.hypot(hw, hl);
    
    const corners = [
      { x: x + circumRadius, y: y },
      { x: x - circumRadius, y: y },
      { x: x, y: y + circumRadius },
      { x: x, y: y - circumRadius },
    ];

    let maxCost = centerCost;
    for (const c of corners) {
      const cg = grid.worldToGrid(c.x, c.y);
      if (grid.inBounds(cg.gx, cg.gy)) {
        maxCost = Math.max(maxCost, grid.costmap[cg.gy * grid.width + cg.gx]);
      } else {
        return 254;
      }
    }
    return maxCost;
  }

  _pickLocalGoal(globalPlan, pose, lookahead) {
    if (globalPlan.length === 0) return pose;
    if (globalPlan.length === 1) return globalPlan[0];

    let closestIdx = 0;
    let closestDist = Infinity;
    for (let i = 0; i < globalPlan.length; i++) {
      const d = Math.hypot(globalPlan[i].x - pose.x, globalPlan[i].y - pose.y);
      if (d < closestDist) {
        closestDist = d;
        closestIdx = i;
      }
    }

    for (let i = closestIdx; i < globalPlan.length; i++) {
      const d = Math.hypot(globalPlan[i].x - pose.x, globalPlan[i].y - pose.y);
      if (d >= lookahead) return globalPlan[i];
    }

    return globalPlan[globalPlan.length - 1];
  }

  _distanceToPath(x, y, globalPlan) {
    if (globalPlan.length === 0) return 0;
    if (globalPlan.length === 1) return Math.hypot(globalPlan[0].x - x, globalPlan[0].y - y);

    let minDist = Infinity;
    for (let i = 0; i < globalPlan.length - 1; i++) {
      const p1 = globalPlan[i];
      const p2 = globalPlan[i + 1];

      const l2 = Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2);
      if (l2 === 0) {
        minDist = Math.min(minDist, Math.hypot(p1.x - x, p1.y - y));
        continue;
      }

      let t = ((x - p1.x) * (p2.x - p1.x) + (y - p1.y) * (p2.y - p1.y)) / l2;
      t = Math.max(0, Math.min(1, t));

      const projX = p1.x + t * (p2.x - p1.x);
      const projY = p1.y + t * (p2.y - p1.y);

      minDist = Math.min(minDist, Math.hypot(projX - x, projY - y));
    }
    return minDist;
  }
}

export default MPPIPlanner;
