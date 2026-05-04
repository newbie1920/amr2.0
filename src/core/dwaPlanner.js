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
import { ROBOT_RADIUS, ROBOT_HALF_WIDTH, ROBOT_HALF_LENGTH } from './warehouse.js';

export const DWA_DEFAULTS = {
  maxSpeedTrans: 0.40,
  minSpeedTrans: 0.0,
  maxSpeedRot: 1.5,
  maxAccelTrans: 0.8,
  maxAccelRot: 2.5,
  simTime: 1.8,                     // Shorter sim = faster computation + quicker reactions
  simGranularity: 0.15,             // Coarser steps = fewer collision checks (was 0.10)
  vSamples: 9,                      // Fewer samples = faster (was 11)
  wSamples: 21,                     // Fewer samples = faster (was 31)
  robotRadius: ROBOT_RADIUS,
  preferredClearance: 0.45,         // INCREASED: keep robot further from walls (was 0.35)
  stopOnClearance: 0.08,            // INCREASED: stop earlier near obstacles (was 0.06)
  headingLookahead: 2.0,            // Look further ahead (2.0m) for smoother turns
  pathDistBias: 1.0,
  goalDistBias: 12.0,
  goalHeadingBias: 12.0,
  clearanceBias: 30.0,              // INCREASED: strongly prefer clear corridors (was 25)
  speedBias: 4.0,                   // INCREASED: favor speed more to reduce stop-and-go (was 3)
  rotateInPlaceAngle: Math.PI / 2,
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

  // ── CURVE SPEED CONTROLLER + PREDICTIVE TURN BRAKING ──
  let dynamicVMax = cfg.maxSpeedTrans;
  const absHeadingErr = Math.abs(headingError);
  if (absHeadingErr > 0.26) {
    const brakeFactor = Math.max(0.4, 1.0 - (absHeadingErr / 1.57)); 
    dynamicVMax = cfg.maxSpeedTrans * brakeFactor;
  }

  // Lightweight predictive braking: single look-ahead at 0.8s
  if (grid && vel.v > 0.05) {
    const lt = 0.8;
    const futureX = pose.x + vel.v * Math.cos(pose.theta) * lt;
    const futureY = pose.y + vel.v * Math.sin(pose.theta) * lt;
    const futureTheta = normalizeAngle(pose.theta + vel.w * lt);
    const futureClearance = footprintClearance(futureX, futureY, futureTheta, grid, cfg);
    if (futureClearance < cfg.preferredClearance * 0.6) {
      const urgency = 1.0 - (futureClearance / (cfg.preferredClearance * 0.6));
      dynamicVMax = Math.min(dynamicVMax, cfg.maxSpeedTrans * Math.max(0.2, 1.0 - urgency * 0.6));
    }
  }

  // ── DYNAMIC WINDOW ──
  // controlPeriodDt: time between DWA commands (~100-300ms actual).
  const controlPeriodDt = 0.4;
  const vMin = Math.max(cfg.minSpeedTrans, vel.v - cfg.maxAccelTrans * controlPeriodDt);
  const vMax = Math.min(dynamicVMax, vel.v + cfg.maxAccelTrans * controlPeriodDt);
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

    // ★ RECTANGULAR FOOTPRINT collision — checks all corners + edges at current heading
    if (rectangularFootprintCollision(x, y, theta, grid, cfg)) {
      return { ok: false, reason: 'collision' };
    }

    // ★ FOOTPRINT-AWARE clearance — measures from nearest corner, not center
    const clearance = footprintClearance(x, y, theta, grid, cfg);
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

// ============================================================
//   RECTANGULAR FOOTPRINT GEOMETRY
// ============================================================

/**
 * Get sample points along the robot's rectangular footprint perimeter.
 * Returns 4 corners + edge sample points (every ~5cm along each edge).
 * All points are rotated by `theta` and offset to world position (x, y).
 *
 * @param {number} x - World X of robot center
 * @param {number} y - World Y of robot center
 * @param {number} theta - Robot heading (radians)
 * @returns {Array<{x: number, y: number}>}
 */
/**
 * Lightweight footprint: 4 corners + 4 edge midpoints = 8 points total.
 * ~3x faster than the 24-point version while still covering the full rectangle.
 */
function getFootprintPoints(x, y, theta) {
  const hw = ROBOT_HALF_WIDTH;
  const hl = ROBOT_HALF_LENGTH;
  const cosT = Math.cos(theta);
  const sinT = Math.sin(theta);

  // 8 key points: 4 corners + 4 edge midpoints
  const locals = [
    // Corners
    { lx: -hw, ly:  hl },  // front-left
    { lx:  hw, ly:  hl },  // front-right
    { lx:  hw, ly: -hl },  // rear-right
    { lx: -hw, ly: -hl },  // rear-left
    // Edge midpoints
    { lx:  0,  ly:  hl },  // front-center
    { lx:  0,  ly: -hl },  // rear-center
    { lx: -hw, ly:  0  },  // left-center
    { lx:  hw, ly:  0  },  // right-center
  ];

  const points = new Array(locals.length);
  for (let i = 0; i < locals.length; i++) {
    const { lx, ly } = locals[i];
    points[i] = {
      x: x + lx * cosT - ly * sinT,
      y: y + lx * sinT + ly * cosT,
    };
  }
  return points;
}

/**
 * Rectangular footprint collision check.
 * Tests all corners + edge sample points against the occupancy grid.
 *
 * @param {number} x - World X
 * @param {number} y - World Y
 * @param {number} theta - Robot heading
 * @param {object} grid - OccupancyGrid instance
 * @param {object} cfg - DWA config
 * @returns {boolean} true if any part of the footprint collides
 */
function rectangularFootprintCollision(x, y, theta, grid, cfg) {
  const points = getFootprintPoints(x, y, theta);

  for (const pt of points) {
    const g = grid.worldToGrid(pt.x, pt.y);
    if (!grid.inBounds(g.gx, g.gy)) continue;

    if (grid.costmap) {
      const cost = grid.getCost(g.gx, g.gy);
      if (cost >= 253) return true; // Only 253 (Inscribed) or 254 (Lethal) is a physical collision
    } else {
      // Fallback: raw logOdds
      if (grid.getLogOdds(g.gx, g.gy) > 0.3) return true;
    }
  }

  // Also check center
  const center = grid.worldToGrid(x, y);
  if (grid.inBounds(center.gx, center.gy)) {
    if (grid.costmap) {
      if (grid.getCost(center.gx, center.gy) >= 253) return true;
    } else {
      if (grid.getLogOdds(center.gx, center.gy) > 0.3) return true;
    }
  }

  return false;
}

/**
 * Footprint-aware obstacle clearance.
 * Measures minimum distance from any footprint point (corners + edges) to nearest obstacle.
 *
 * @param {number} x - World X
 * @param {number} y - World Y
 * @param {number} theta - Robot heading
 * @param {object} grid - OccupancyGrid instance
 * @param {object} cfg - DWA config
 * @returns {number} Minimum clearance in meters
 */
function footprintClearance(x, y, theta, grid, cfg) {
  const points = getFootprintPoints(x, y, theta);
  const scanRadius = Math.max(3, Math.ceil(cfg.preferredClearance / grid.resolution));
  let globalBest = Infinity;

  // For performance, only check corners (4 points) for clearance scan
  // Corners are at indices 0, edgeSamples, 2*edgeSamples, 3*edgeSamples
  const hw = ROBOT_HALF_WIDTH;
  const hl = ROBOT_HALF_LENGTH;
  const cosT = Math.cos(theta);
  const sinT = Math.sin(theta);
  const corners = [
    { x: x + (-hw) * cosT - ( hl) * sinT, y: y + (-hw) * sinT + ( hl) * cosT },
    { x: x + ( hw) * cosT - ( hl) * sinT, y: y + ( hw) * sinT + ( hl) * cosT },
    { x: x + ( hw) * cosT - (-hl) * sinT, y: y + ( hw) * sinT + (-hl) * cosT },
    { x: x + (-hw) * cosT - (-hl) * sinT, y: y + (-hw) * sinT + (-hl) * cosT },
  ];

  for (const corner of corners) {
    const cg = grid.worldToGrid(corner.x, corner.y);
    let cornerBest = Infinity;

    for (let dy = -scanRadius; dy <= scanRadius; dy++) {
      for (let dx = -scanRadius; dx <= scanRadius; dx++) {
        const gx = cg.gx + dx;
        const gy = cg.gy + dy;
        if (!grid.inBounds(gx, gy)) continue;

        const cost = grid.getCost ? grid.getCost(gx, gy) : 0;
        const cellDist = Math.hypot(dx, dy) * grid.resolution;

        if (cost >= 253) {
          cornerBest = Math.min(cornerBest, cellDist);
        } else if (cost > 50) {
          const inflFactor = cost / 253;
          const effectiveDist = cellDist + (1.0 - inflFactor) * cfg.preferredClearance;
          cornerBest = Math.min(cornerBest, effectiveDist);
        }
      }
    }

    globalBest = Math.min(globalBest, cornerBest);
  }

  return globalBest === Infinity ? cfg.preferredClearance + 0.2 : globalBest;
}

/**
 * Check if the robot can safely rotate in-place.
 * Tests footprint at several rotation angles to ensure no corner clips an obstacle.
 */
function checkInPlaceRotation(pose, grid, cfg) {
  // Check current orientation
  if (rectangularFootprintCollision(pose.x, pose.y, pose.theta, grid, cfg)) {
    return false;
  }
  // Check at ±30°, ±60°, ±90° to ensure sweep is clear
  const sweepAngles = [Math.PI/6, -Math.PI/6, Math.PI/3, -Math.PI/3, Math.PI/2, -Math.PI/2];
  for (const da of sweepAngles) {
    const testTheta = normalizeAngle(pose.theta + da);
    if (rectangularFootprintCollision(pose.x, pose.y, testTheta, grid, cfg)) {
      return false;
    }
  }
  return true;
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


