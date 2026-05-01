/**
 * AMR 2.0 - Nav2-style local planner
 *
 * A lightweight browser-side planner inspired by DWA / regulated local planners:
 * - dynamic window sampling
 * - footprint-aware collision checks
 * - obstacle clearance scoring
 * - path alignment scoring
 * - goal heading regulation
 *
 * Phase 3: All parameters are now injectable via `config` argument,
 * enabling live tuning from the DWATuningPanel UI.
 */

import { normalizeAngle } from './mathUtils.js';
import { ROBOT_RADIUS } from './warehouse.js';

export const DWA_DEFAULTS = {
  maxSpeedTrans: 0.40,              // Faster top speed (was 0.35)
  minSpeedTrans: 0.0,
  maxSpeedRot: 1.5,                 // Faster turning (was 1.2)
  maxAccelTrans: 0.8,               // Snappier acceleration (was 0.7)
  maxAccelRot: 2.5,                 // Snappier rotation accel (was 2.2)
  simTime: 2.0,                     // Slightly shorter sim time for better responsiveness (was 2.5)
  simGranularity: 0.10,
  vSamples: 11,                     // More velocity samples
  wSamples: 31,                     // More rotational samples
  robotRadius: ROBOT_RADIUS,
  preferredClearance: 0.35,         // Increased slightly to stay safer from walls
  stopOnClearance: 0.06,            // Increased from 0.04 to prevent getting stuck against walls
  headingLookahead: 1.5,            // Look further ahead (1.5m) to start turning EARLIER
  pathDistBias: 1.0,                // Drastically reduced (was 8.0). Allows Pure Pursuit-like corner cutting.
  goalDistBias: 12.0,               // Pull robot along path
  goalHeadingBias: 12.0,            // Align to goal heading more aggressively
  clearanceBias: 25.0,              // Keep high to avoid obstacles while corner cutting
  speedBias: 3.0,                   // Favor higher speeds when safe
  rotateInPlaceAngle: Math.PI / 2,  // Only stop to rotate if error > 90deg (allows smooth arcs instead of stopping)
};

export const DWA_PRESETS = {
  cautious: {
    maxSpeedTrans: 0.18,
    minSpeedTrans: 0.0,
    maxSpeedRot: 0.8,
    maxAccelTrans: 0.4,
    maxAccelRot: 1.5,
    simTime: 3.0,
    simGranularity: 0.08,
    vSamples: 11,
    wSamples: 29,
    robotRadius: ROBOT_RADIUS + 0.03,  // Slightly larger for cautious mode
    preferredClearance: 0.40,
    stopOnClearance: 0.06,
    headingLookahead: 0.5,
    pathDistBias: 18.0,
    goalDistBias: 8.0,
    goalHeadingBias: 5.0,
    clearanceBias: 25.0,
    speedBias: 1.0,
    rotateInPlaceAngle: Math.PI / 6,
  },
  balanced: { ...DWA_DEFAULTS },
  aggressive: {
    maxSpeedTrans: 0.55,
    minSpeedTrans: 0.0,
    maxSpeedRot: 1.8,
    maxAccelTrans: 1.2,
    maxAccelRot: 3.5,
    simTime: 1.5,
    simGranularity: 0.12,
    vSamples: 7,
    wSamples: 19,
    robotRadius: ROBOT_RADIUS,         // Match physics exactly
    preferredClearance: 0.22,
    stopOnClearance: 0.03,
    headingLookahead: 1.0,
    pathDistBias: 12.0,
    goalDistBias: 16.0,
    goalHeadingBias: 8.0,
    clearanceBias: 10.0,
    speedBias: 5.0,
    rotateInPlaceAngle: Math.PI / 3,
  },
};

// Ensure balanced preset is a proper copy
DWA_PRESETS.balanced = { ...DWA_DEFAULTS };

export function computeVelocityCmd(pose, vel, globalPlan, grid, config = null) {
  const cfg = config ? { ...DWA_DEFAULTS, ...config } : DWA_DEFAULTS;

  if (!globalPlan || globalPlan.length === 0) {
    return { v: 0, w: 0, ok: false, reason: 'no_plan' };
  }

  const localGoal = pickLocalGoal(globalPlan, pose, cfg.headingLookahead);
  const goalHeading = Math.atan2(localGoal.y - pose.y, localGoal.x - pose.x);
  const headingError = normalizeAngle(goalHeading - pose.theta);

  // ── ROTATE-IN-PLACE: Nav2-style ──
  // When heading error is large, FORCE pure rotation (v=0).
  // Without this, DWA picks arc trajectories (v>0, w>0) that circle forever.
  if (Math.abs(headingError) > cfg.rotateInPlaceAngle) {
    const safeTurn = checkInPlaceRotation(pose, grid, cfg);
    if (!safeTurn) {
      return { v: 0, w: 0, ok: false, reason: 'blocked_rotation' };
    }
    // Pure proportional rotation toward goal heading
    const gain = 1.8;
    const cmdW = Math.sign(headingError) * Math.min(
      cfg.maxSpeedRot,
      Math.abs(headingError) * gain
    );
    return {
      v: 0,
      w: cmdW,
      ok: true,
      clearance: cfg.preferredClearance,
      score: 0,
      trajectory: [{ x: pose.x, y: pose.y }],
      diag: { rotateInPlace: true, headingError: headingError.toFixed(3), cmdW: cmdW.toFixed(3) }
    };
  }

  // ── DYNAMIC WINDOW ──
  // controlPeriodDt: time between DWA commands (~100-300ms actual).
  // Using simGranularity (0.1s) was too restrictive — once the robot had angular
  // velocity from recovery/heading alignment, the narrow window trapped it in circles.
  // 0.4s allows the robot to explore a much wider velocity range per cycle.
  const controlPeriodDt = 0.4;
  const vMin = Math.max(cfg.minSpeedTrans, vel.v - cfg.maxAccelTrans * controlPeriodDt);
  const vMax = Math.min(cfg.maxSpeedTrans, vel.v + cfg.maxAccelTrans * controlPeriodDt);
  const wMin = Math.max(-cfg.maxSpeedRot, vel.w - cfg.maxAccelRot * controlPeriodDt);
  const wMax = Math.min(cfg.maxSpeedRot, vel.w + cfg.maxAccelRot * controlPeriodDt);

  let best = null;
  let collisionCount = 0;
  let clearanceCount = 0;
  let okCount = 0;

  let candidates = [];

  for (let iv = 0; iv < cfg.vSamples; iv++) {
    const v = sampleLinear(vMin, vMax, iv, cfg.vSamples);

    for (let iw = 0; iw < cfg.wSamples; iw++) {
      const w = sampleAngular(wMin, wMax, iw, cfg.wSamples);
      const candidate = scoreTrajectory(v, w, pose, localGoal, globalPlan, grid, cfg);
      if (!candidate.ok) {
        if (candidate.reason === 'clearance') clearanceCount++;
        else collisionCount++;
        continue;
      }
      okCount++;
      candidates.push(candidate);

      if (!best || candidate.score < best.score) {
        best = candidate;
      }
    }
  }

  candidates.sort((a, b) => a.score - b.score);
  const diagObj = { 
    collision: collisionCount, 
    clearance: clearanceCount, 
    ok: okCount, 
    vRange: [vMin, vMax], 
    wRange: [wMin, wMax],
    localGoal: { x: localGoal.x.toFixed(3), y: localGoal.y.toFixed(3) },
    topScores: candidates.slice(0, 5).map(c => ({
      v: c.v.toFixed(3), 
      w: c.w.toFixed(3), 
      score: c.score.toFixed(3),
      hErr: c.headingError.toFixed(3),
      pDist: c.pathDist.toFixed(3),
      gDist: c.goalDist.toFixed(3)
    }))
  };

  if (!best) {
    return { v: 0, w: 0, ok: false, reason: 'no_safe_trajectory', trajectory: [], diag: diagObj };
  }

  return {
    v: best.v,
    w: best.w,
    ok: true,
    clearance: best.minClearance,
    score: best.score,
    trajectory: best.trajectory || [],
    diag: diagObj
  };
}

function scoreTrajectory(v, w, pose, localGoal, globalPlan, grid, cfg) {
  let x = pose.x;
  let y = pose.y;
  let theta = pose.theta;
  const steps = Math.max(1, Math.floor(cfg.simTime / cfg.simGranularity));
  let minClearance = Infinity;
  const trajectory = [{ x, y }];

  for (let i = 0; i < steps; i++) {
    x += v * Math.cos(theta) * cfg.simGranularity;
    y += v * Math.sin(theta) * cfg.simGranularity;
    theta = normalizeAngle(theta + w * cfg.simGranularity);

    // Record trajectory (every 3 steps for lightweight output)
    if (i % 3 === 0) trajectory.push({ x, y });

    const collision = footprintCollision(x, y, grid, cfg);
    if (collision) {
      return { ok: false, reason: 'collision' };
    }

    const clearance = obstacleClearance(x, y, grid, cfg);
    minClearance = Math.min(minClearance, clearance);

    if (clearance < cfg.stopOnClearance) {
      return { ok: false, reason: 'clearance' };
    }
  }
  trajectory.push({ x, y }); // Final point

  const goalDist = Math.hypot(localGoal.x - x, localGoal.y - y);
  const pathDist = distanceToPath(x, y, globalPlan);
  const finalHeading = Math.atan2(localGoal.y - y, localGoal.x - x);
  const headingError = Math.abs(normalizeAngle(finalHeading - theta));
  const clearancePenalty = Math.max(0, cfg.preferredClearance - minClearance);
  const speedReward = cfg.maxSpeedTrans - v;

  const score =
    cfg.goalDistBias * goalDist +
    cfg.pathDistBias * pathDist +
    cfg.goalHeadingBias * headingError +
    cfg.clearanceBias * clearancePenalty +
    cfg.speedBias * speedReward;

  return { ok: true, score, v, w, minClearance, trajectory, headingError, pathDist, goalDist };
}

function pickLocalGoal(globalPlan, pose, lookahead) {
  if (globalPlan.length === 0) return pose;
  if (globalPlan.length === 1) return globalPlan[0];

  // Step 1: Find the closest point on the path (the "carrot" anchor)
  let closestIdx = 0;
  let closestDist = Infinity;
  for (let i = 0; i < globalPlan.length; i++) {
    const d = Math.hypot(globalPlan[i].x - pose.x, globalPlan[i].y - pose.y);
    if (d < closestDist) {
      closestDist = d;
      closestIdx = i;
    }
  }

  // Step 2: Look AHEAD from the closest point (never backward)
  for (let i = closestIdx; i < globalPlan.length; i++) {
    const d = Math.hypot(globalPlan[i].x - pose.x, globalPlan[i].y - pose.y);
    if (d >= lookahead) {
      return globalPlan[i];
    }
  }

  // Fallback: final goal
  return globalPlan[globalPlan.length - 1];
}

function distanceToPath(x, y, globalPlan) {
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

function footprintCollision(x, y, grid, cfg) {
  const center = grid.worldToGrid(x, y);
  if (!grid.inBounds(center.gx, center.gy)) return false;

  // If costmap is available (it should be), it is ALREADY inflated by INSCRIBED_RADIUS.
  // Therefore, we only need to check if the robot's CENTER is inside a lethal/inscribed cell.
  if (grid.costmap) {
    const cost = grid.getCost(center.gx, center.gy);
    if (cost >= 253) return true;
    return false;
  }

  // Fallback if no costmap (raw logOdds grid): manually check footprint
  const radiusCells = Math.max(1, Math.ceil(cfg.robotRadius / grid.resolution));
  for (let dy = -radiusCells; dy <= radiusCells; dy++) {
    for (let dx = -radiusCells; dx <= radiusCells; dx++) {
      const dist = Math.hypot(dx, dy) * grid.resolution;
      if (dist > cfg.robotRadius) continue;

      const gx = center.gx + dx;
      const gy = center.gy + dy;
      if (!grid.inBounds(gx, gy)) continue;

      if (grid.getLogOdds(gx, gy) > 0.3) {
        return true;
      }
    }
  }

  return false;
}

function obstacleClearance(x, y, grid, cfg) {
  const center = grid.worldToGrid(x, y);
  const maxCells = Math.max(3, Math.ceil(cfg.preferredClearance / grid.resolution));
  let best = Infinity;

  for (let dy = -maxCells; dy <= maxCells; dy++) {
    for (let dx = -maxCells; dx <= maxCells; dx++) {
      const gx = center.gx + dx;
      const gy = center.gy + dy;
      if (!grid.inBounds(gx, gy)) continue;
      const cost = grid.getCost ? grid.getCost(gx, gy) : 0;
      const cellDist = Math.hypot(dx, dy) * grid.resolution;

      if (cost >= 253) {
        // Hard obstacle or inscribed zone
        best = Math.min(best, cellDist);
      } else if (cost > 50) {
        // Inflation zone — create virtual clearance penalty
        const inflFactor = cost / 253;
        const effectiveDist = cellDist + (1.0 - inflFactor) * cfg.preferredClearance;
        best = Math.min(best, effectiveDist);
      }
    }
  }

  return best === Infinity ? cfg.preferredClearance + 0.2 : best;
}

function checkInPlaceRotation(pose, grid, cfg) {
  return !footprintCollision(pose.x, pose.y, grid, cfg);
}

function sampleLinear(min, max, index, total) {
  if (total <= 1) return max;
  const t = index / (total - 1);
  return min + (max - min) * t;
}

function sampleAngular(min, max, index, total) {
  if (total <= 1) return 0;
  const t = index / (total - 1);
  return min + (max - min) * t;
}


