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

export const DWA_DEFAULTS = {
  maxSpeedTrans: 0.35,
  minSpeedTrans: 0.0,
  maxSpeedRot: 1.2,
  maxAccelTrans: 0.7,
  maxAccelRot: 2.2,
  simTime: 1.6,
  simGranularity: 0.08,
  vSamples: 7,
  wSamples: 17,
  robotRadius: 0.19,
  preferredClearance: 0.32,
  stopOnClearance: 0.16,
  headingLookahead: 0.7,
  pathDistBias: 14.0,
  goalDistBias: 11.0,
  goalHeadingBias: 6.0,
  clearanceBias: 12.0,
  speedBias: 2.0,
  rotateInPlaceAngle: Math.PI / 4,
};

export const DWA_PRESETS = {
  cautious: {
    maxSpeedTrans: 0.18,
    minSpeedTrans: 0.0,
    maxSpeedRot: 0.8,
    maxAccelTrans: 0.4,
    maxAccelRot: 1.5,
    simTime: 2.2,
    simGranularity: 0.06,
    vSamples: 9,
    wSamples: 21,
    robotRadius: 0.22,
    preferredClearance: 0.5,
    stopOnClearance: 0.25,
    headingLookahead: 0.5,
    pathDistBias: 16.0,
    goalDistBias: 8.0,
    goalHeadingBias: 5.0,
    clearanceBias: 20.0,
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
    simTime: 1.0,
    simGranularity: 0.1,
    vSamples: 5,
    wSamples: 13,
    robotRadius: 0.17,
    preferredClearance: 0.2,
    stopOnClearance: 0.1,
    headingLookahead: 1.0,
    pathDistBias: 10.0,
    goalDistBias: 16.0,
    goalHeadingBias: 8.0,
    clearanceBias: 6.0,
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

  if (Math.abs(headingError) > cfg.rotateInPlaceAngle) {
    const safeTurn = checkInPlaceRotation(pose, grid, cfg);
    if (!safeTurn) {
      return { v: 0, w: 0, ok: false, reason: 'blocked_rotation' };
    }
  }

  const dt = cfg.simGranularity;
  const vMin = Math.max(cfg.minSpeedTrans, vel.v - cfg.maxAccelTrans * dt);
  const vMax = Math.min(cfg.maxSpeedTrans, vel.v + cfg.maxAccelTrans * dt);
  const wMin = Math.max(-cfg.maxSpeedRot, vel.w - cfg.maxAccelRot * dt);
  const wMax = Math.min(cfg.maxSpeedRot, vel.w + cfg.maxAccelRot * dt);

  let best = null;

  for (let iv = 0; iv < cfg.vSamples; iv++) {
    const v = sampleLinear(vMin, vMax, iv, cfg.vSamples);

    for (let iw = 0; iw < cfg.wSamples; iw++) {
      const w = sampleAngular(wMin, wMax, iw, cfg.wSamples);
      const candidate = scoreTrajectory(v, w, pose, localGoal, globalPlan, grid, cfg);
      if (!candidate.ok) continue;

      if (!best || candidate.score < best.score) {
        best = candidate;
      }
    }
  }

  if (!best) {
    return { v: 0, w: 0, ok: false, reason: 'no_safe_trajectory' };
  }

  return {
    v: best.v,
    w: best.w,
    ok: true,
    clearance: best.minClearance,
    score: best.score,
  };
}

function scoreTrajectory(v, w, pose, localGoal, globalPlan, grid, cfg) {
  let x = pose.x;
  let y = pose.y;
  let theta = pose.theta;
  const steps = Math.max(1, Math.floor(cfg.simTime / cfg.simGranularity));
  let minClearance = Infinity;

  for (let i = 0; i < steps; i++) {
    x += v * Math.cos(theta) * cfg.simGranularity;
    y += v * Math.sin(theta) * cfg.simGranularity;
    theta = normalizeAngle(theta + w * cfg.simGranularity);

    const collision = footprintCollision(x, y, grid, cfg);
    if (collision) {
      return { ok: false };
    }

    const clearance = obstacleClearance(x, y, grid, cfg);
    minClearance = Math.min(minClearance, clearance);

    if (clearance < cfg.stopOnClearance) {
      return { ok: false };
    }
  }

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

  return { ok: true, score, v, w, minClearance };
}

function pickLocalGoal(globalPlan, pose, lookahead) {
  let candidate = globalPlan[globalPlan.length - 1];

  for (let i = 0; i < globalPlan.length; i++) {
    const pt = globalPlan[i];
    const d = Math.hypot(pt.x - pose.x, pt.y - pose.y);
    if (d >= lookahead) {
      candidate = pt;
      break;
    }
  }

  return candidate;
}

function distanceToPath(x, y, globalPlan) {
  let minDist = Infinity;
  for (let i = 0; i < globalPlan.length; i++) {
    const p = globalPlan[i];
    minDist = Math.min(minDist, Math.hypot(p.x - x, p.y - y));
  }
  return minDist;
}

function footprintCollision(x, y, grid, cfg) {
  const radiusCells = Math.max(1, Math.ceil(cfg.robotRadius / grid.resolution));
  const center = grid.worldToGrid(x, y);

  for (let dy = -radiusCells; dy <= radiusCells; dy++) {
    for (let dx = -radiusCells; dx <= radiusCells; dx++) {
      const dist = Math.hypot(dx, dy) * grid.resolution;
      if (dist > cfg.robotRadius) continue;

      const gx = center.gx + dx;
      const gy = center.gy + dy;
      if (!grid.inBounds(gx, gy)) return true;

      const lo = grid.getLogOdds(gx, gy);
      const cost = grid.getCost ? grid.getCost(gx, gy) : 0;
      if (lo > 0.3 || cost >= 200) {
        return true;
      }
    }
  }

  return false;
}

function obstacleClearance(x, y, grid, cfg) {
  const center = grid.worldToGrid(x, y);
  const maxCells = Math.max(2, Math.ceil(cfg.preferredClearance / grid.resolution));
  let best = Infinity;

  for (let dy = -maxCells; dy <= maxCells; dy++) {
    for (let dx = -maxCells; dx <= maxCells; dx++) {
      const gx = center.gx + dx;
      const gy = center.gy + dy;
      if (!grid.inBounds(gx, gy)) continue;
      const lo = grid.getLogOdds(gx, gy);
      const cost = grid.getCost ? grid.getCost(gx, gy) : 0;
      if (lo > 0.3 || cost >= 200) {
        best = Math.min(best, Math.hypot(dx, dy) * grid.resolution);
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

function normalizeAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}
