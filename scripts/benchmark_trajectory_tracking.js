import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const NAV = {
  dt: 0.02,
  maxLinear: 0.4,
  approachVel: 0.05,
  slowdownDist: 0.25,
  goalTolerance: 0.08,
  turnSpeed: 1.2,
  kx: 2.0,
  ky: 5.0,
  kth: 1.5,
};

const DEFAULT_TRAJECTORY = {
  enabled: true,
  mode: 'legacy_ramp',
  vMax: 0.3,
  aMax: 0.3,
  jMax: 1.0,
};

const ADAPTIVE_SCURVE_TRAJECTORY = {
  ...DEFAULT_TRAJECTORY,
  mode: 'adaptive_scurve',
};

const TRAJECTORY_BENCHMARK_MODES = [
  { mode: 'firmware_current', controller: 'firmware_current', trajectory: DEFAULT_TRAJECTORY },
  { mode: 'firmware_current_adaptive_scurve', controller: 'firmware_current', trajectory: ADAPTIVE_SCURVE_TRAJECTORY },
  { mode: 'fl_regularized', controller: 'fl_regularized', trajectory: DEFAULT_TRAJECTORY },
];

const SCENARIOS = [
  {
    name: 'short_straight_omega_zero',
    description: 'Short straight path where ref_w is zero for most samples.',
    path: [{ x: 0, y: 0 }, { x: 0.35, y: 0 }],
    initialPose: { x: 0, y: -0.04, theta: 0.04 },
    maxTime: 12,
  },
  {
    name: 'l_turn_multi_waypoint',
    description: 'Two-segment L path that exposes heading discontinuity and lateral tracking.',
    path: [{ x: 0, y: 0 }, { x: 0.9, y: 0 }, { x: 0.9, y: 0.65 }],
    initialPose: { x: 0, y: -0.05, theta: 0.02 },
    maxTime: 18,
  },
  {
    name: 'very_short_type_i_segment',
    description: 'Very short move below 0.5 m for degenerate Type I/III profile behavior.',
    path: [{ x: 0, y: 0 }, { x: 0.2, y: 0.02 }],
    initialPose: { x: 0, y: 0, theta: 0 },
    maxTime: 10,
  },
  {
    name: 'docking_final_heading',
    description: 'Straight approach with a final heading constraint similar to docking.',
    path: [{ x: 0, y: 0 }, { x: 0.75, y: 0 }],
    finalHeading: Math.PI / 2,
    initialPose: { x: 0, y: -0.04, theta: 0 },
    maxTime: 16,
  },
  {
    name: 'u_turn_180deg',
    description: '180-degree U-turn stress test: straight, reverse heading, straight back.',
    path: [{ x: 0, y: 0 }, { x: 0.6, y: 0 }, { x: 0.6, y: 0.5 }, { x: 0, y: 0.5 }],
    initialPose: { x: 0, y: -0.03, theta: 0.01 },
    maxTime: 24,
  },
  {
    name: 's_curve_smooth',
    description: 'S-curve path with gentle heading changes to test smooth tracking.',
    path: [{ x: 0, y: 0 }, { x: 0.4, y: 0.15 }, { x: 0.8, y: -0.1 }, { x: 1.2, y: 0.05 }],
    initialPose: { x: 0, y: 0, theta: 0.02 },
    maxTime: 20,
  },
];

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function normalizeAngle(angle) {
  let out = angle;
  while (out > Math.PI) out -= 2 * Math.PI;
  while (out < -Math.PI) out += 2 * Math.PI;
  return out;
}

function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

class FirmwareLikeTrajectory {
  constructor(points, config = DEFAULT_TRAJECTORY) {
    this.points = points.map((point) => ({ ...point }));
    this.config = {
      enabled: config.enabled !== false,
      mode: config.mode === 'adaptive_scurve' ? 'adaptive_scurve' : 'legacy_ramp',
      vMax: Math.max(0.02, config.vMax),
      aMax: Math.max(0.05, config.aMax),
      jMax: Math.max(0.1, config.jMax),
    };
    this.targetIndex = points.length > 1 ? 1 : 0;
    this.speed = 0;
    this.accel = 0;
    this.distanceOnSegment = 0;
    this.segmentTime = 0;
    this.segmentPlan = null;
    this.lastPlan = null;
    this.done = points.length === 0;
    this.ref = this.initialRef();
  }

  initialRef() {
    if (this.points.length === 0) {
      return {
        x: 0, y: 0, theta: 0, v: 0, w: 0,
        targetIndex: 0, progress: 1, profileType: 'idle',
        profileMode: this.config.mode, segmentTime: 0, tTotal: 0, done: true,
      };
    }
    const from = this.points[0];
    const to = this.points[this.targetIndex] ?? from;
    return {
      x: from.x,
      y: from.y,
      theta: Math.atan2(to.y - from.y, to.x - from.x),
      v: 0,
      w: 0,
      targetIndex: this.targetIndex,
      progress: 0,
      profileType: this.initialProfileType(),
      profileMode: this.config.mode,
      segmentTime: 0,
      tTotal: 0,
      done: false,
    };
  }

  sample(dt) {
    if (this.done || this.points.length <= 1 || this.targetIndex >= this.points.length) {
      this.done = true;
      this.ref = {
        ...this.ref,
        v: 0,
        w: 0,
        progress: 1,
        profileType: 'done',
        profileMode: this.config.mode,
        segmentTime: 0,
        tTotal: 0,
        done: true,
      };
      return this.ref;
    }

    const from = this.points[this.targetIndex - 1];
    const to = this.points[this.targetIndex];
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const segLen = Math.hypot(dx, dy);

    if (segLen < 0.001) {
      this.advance(to);
      return this.sample(dt);
    }

    if (this.config.enabled && this.config.mode === 'adaptive_scurve') {
      return this.sampleAdaptiveScurve(from, to, dx, dy, segLen, dt);
    }
    return this.sampleLegacyRamp(from, to, dx, dy, segLen, dt);
  }

  sampleLegacyRamp(from, to, dx, dy, segLen, dt) {
    const remaining = Math.max(0, segLen - this.distanceOnSegment);
    let profileType = this.classifyLegacyProfile(segLen);

    if (this.config.enabled) {
      const brakingSpeed = Math.sqrt(2 * this.config.aMax * remaining);
      const desiredSpeed = Math.min(this.config.vMax, brakingSpeed);
      const desiredAccel = desiredSpeed > this.speed ? this.config.aMax : -this.config.aMax;
      const maxAccelStep = this.config.jMax * dt;

      if (this.accel < desiredAccel) this.accel = Math.min(this.accel + maxAccelStep, desiredAccel);
      else if (this.accel > desiredAccel) this.accel = Math.max(this.accel - maxAccelStep, desiredAccel);

      this.speed = clamp(this.speed + this.accel * dt, 0, this.config.vMax);
    } else {
      this.accel = 0;
      this.speed = Math.min(this.config.vMax, remaining / Math.max(dt, 0.001));
      profileType = 'direct';
    }

    const step = this.speed * dt;
    if (step >= remaining) {
      this.distanceOnSegment = segLen;
      this.ref = {
        x: to.x,
        y: to.y,
        theta: Math.atan2(dy, dx),
        v: 0,
        w: 0,
        targetIndex: this.targetIndex,
        progress: 1,
        profileType,
        profileMode: this.config.mode,
        segmentTime: 0,
        tTotal: 0,
        done: false,
      };
      this.advance(to);
      return this.ref;
    }

    this.distanceOnSegment += step;
    const t = this.distanceOnSegment / segLen;
    const prevTheta = this.ref.theta;
    const theta = Math.atan2(dy, dx);
    this.ref = {
      x: from.x + dx * t,
      y: from.y + dy * t,
      theta,
      v: this.speed * (to.useReverse ? -1 : 1),
      w: normalizeAngle(theta - prevTheta) / Math.max(dt, 0.001),
      targetIndex: this.targetIndex,
      progress: clamp(t, 0, 1),
      profileType,
      profileMode: this.config.mode,
      segmentTime: 0,
      tTotal: 0,
      done: false,
    };
    return this.ref;
  }

  initialProfileType() {
    if (!this.config.enabled) return 'direct';
    return this.config.mode === 'adaptive_scurve' ? 'type_i' : 'legacy_ramp';
  }

  classifyLegacyProfile(segLen) {
    if (!this.config.enabled) return 'direct';
    const accelDistance = (this.config.vMax * this.config.vMax) / Math.max(this.config.aMax, 0.001);
    const jerkDistance = (this.config.aMax ** 3) / Math.max(this.config.jMax ** 2, 0.001);
    if (segLen >= accelDistance) return 'type_iv';
    if (segLen >= jerkDistance) return 'type_iii';
    return 'type_i';
  }

  buildAdaptivePlan(segLen) {
    const j = this.config.jMax;
    const a = this.config.aMax;
    const v = this.config.vMax;
    const tJMax = a / j;
    const dTypeIBoundary = 2 * j * tJMax ** 3;
    const plan = {
      distance: segLen,
      targetIndex: this.targetIndex,
      tJ: 0,
      tA: 0,
      tV: 0,
      tTotal: 0,
      vPeak: 0,
      aPeak: 0,
      profileType: 'type_i',
    };

    if (segLen <= dTypeIBoundary) {
      plan.tJ = Math.cbrt(segLen / (2 * j));
      plan.vPeak = j * plan.tJ ** 2;
      plan.aPeak = j * plan.tJ;
      plan.profileType = 'type_i';
    } else {
      plan.tJ = tJMax;
      const root = Math.sqrt(Math.max(0, tJMax ** 2 + (4 * segLen) / a));
      const tAForDistance = Math.max(0, (-3 * tJMax + root) * 0.5);
      const peakVNoCruise = a * (tAForDistance + tJMax);

      if (peakVNoCruise <= v) {
        plan.tA = tAForDistance;
        plan.vPeak = peakVNoCruise;
        plan.aPeak = a;
        plan.profileType = 'type_iii';
      } else {
        plan.tA = Math.max(0, v / a - tJMax);
        const accelHalfDistance = a * tJMax ** 2 + 1.5 * a * tJMax * plan.tA + 0.5 * a * plan.tA ** 2;
        const noCruiseDistance = 2 * accelHalfDistance;

        if (noCruiseDistance > segLen) {
          plan.tJ = Math.cbrt(segLen / (2 * j));
          plan.tA = 0;
          plan.vPeak = j * plan.tJ ** 2;
          plan.aPeak = j * plan.tJ;
          plan.profileType = 'type_i';
        } else {
          plan.tV = (segLen - noCruiseDistance) / v;
          plan.vPeak = v;
          plan.aPeak = a;
          plan.profileType = 'type_iv';
        }
      }
    }

    plan.tTotal = Math.max(0.001, 4 * plan.tJ + 2 * plan.tA + plan.tV);
    return plan;
  }

  sampleMotion(plan, time) {
    const durations = [plan.tJ, plan.tA, plan.tJ, plan.tV, plan.tJ, plan.tA, plan.tJ];
    const jerks = [this.config.jMax, 0, -this.config.jMax, 0, -this.config.jMax, 0, this.config.jMax];
    let remaining = clamp(time, 0, plan.tTotal);
    const sample = { s: 0, v: 0, a: 0 };

    for (let i = 0; i < durations.length && remaining > 0; i += 1) {
      const tau = Math.min(remaining, durations[i]);
      const jerk = jerks[i];
      sample.s += sample.v * tau + 0.5 * sample.a * tau ** 2 + (jerk * tau ** 3) / 6;
      sample.v += sample.a * tau + 0.5 * jerk * tau ** 2;
      sample.a += jerk * tau;
      remaining -= tau;
    }

    sample.s = clamp(sample.s, 0, plan.distance);
    sample.v = clamp(sample.v, 0, this.config.vMax);
    return sample;
  }

  sampleAdaptiveScurve(from, to, dx, dy, segLen, dt) {
    if (!this.segmentPlan || this.segmentPlan.targetIndex !== this.targetIndex) {
      this.segmentPlan = this.buildAdaptivePlan(segLen);
      this.lastPlan = this.segmentPlan;
      this.segmentTime = 0;
      this.distanceOnSegment = 0;
    }

    this.segmentTime = Math.min(this.segmentTime + dt, this.segmentPlan.tTotal);
    const motion = this.sampleMotion(this.segmentPlan, this.segmentTime);
    this.distanceOnSegment = motion.s;

    if (this.segmentTime >= this.segmentPlan.tTotal || this.distanceOnSegment >= segLen - 0.0005) {
      this.distanceOnSegment = segLen;
      this.ref = {
        x: to.x,
        y: to.y,
        theta: Math.atan2(dy, dx),
        v: 0,
        w: 0,
        targetIndex: this.targetIndex,
        progress: 1,
        profileType: this.segmentPlan.profileType,
        profileMode: this.config.mode,
        segmentTime: this.segmentPlan.tTotal,
        tTotal: this.segmentPlan.tTotal,
        done: false,
      };
      this.advance(to);
      return this.ref;
    }

    const t = clamp(this.distanceOnSegment / Math.max(segLen, 0.001), 0, 1);
    const prevTheta = this.ref.theta;
    const theta = Math.atan2(dy, dx);
    this.ref = {
      x: from.x + dx * t,
      y: from.y + dy * t,
      theta,
      v: motion.v * (to.useReverse ? -1 : 1),
      w: normalizeAngle(theta - prevTheta) / Math.max(dt, 0.001),
      targetIndex: this.targetIndex,
      progress: t,
      profileType: this.segmentPlan.profileType,
      profileMode: this.config.mode,
      segmentTime: this.segmentTime,
      tTotal: this.segmentPlan.tTotal,
      done: false,
    };
    return this.ref;
  }

  advance(arrived) {
    this.speed = 0;
    this.accel = 0;
    this.distanceOnSegment = 0;
    this.segmentTime = 0;
    this.segmentPlan = null;
    this.targetIndex += 1;
    if (this.targetIndex >= this.points.length) {
      this.done = true;
      this.ref = {
        ...this.ref,
        x: arrived.x,
        y: arrived.y,
        v: 0,
        w: 0,
        progress: 1,
        profileType: 'done',
        profileMode: this.config.mode,
        done: true,
      };
    }
  }
}

function controllerCurrent(pose, ref, finalPoint, isFinalSegment) {
  const errors = bodyErrors(pose, ref);
  let refV = Math.abs(ref.v);
  if (Math.abs(errors.eTheta) > Math.PI / 2) refV *= 0.2;

  let cmdLinear = refV * Math.cos(errors.eTheta) + NAV.kx * errors.bodyEx;
  let cmdAngular = (Math.abs(ref.w) < 0.001 ? 0 : ref.w) + NAV.ky * errors.bodyEy + NAV.kth * Math.sin(errors.eTheta);

  cmdLinear = clamp(cmdLinear, -NAV.maxLinear, NAV.maxLinear);
  if (isFinalSegment && distance(pose, finalPoint) < NAV.slowdownDist) {
    cmdLinear = clamp(cmdLinear, -NAV.approachVel, NAV.approachVel);
  }
  cmdAngular = clamp(cmdAngular, -NAV.turnSpeed, NAV.turnSpeed);
  return { ...errors, cmdLinear, cmdAngular, refV, refW: ref.w };
}

function controllerFlRegularized(pose, ref, finalPoint, isFinalSegment, segmentChanged = false) {
  const errors = bodyErrors(pose, ref);
  let refV = Math.abs(ref.v);

  const absETheta = Math.abs(errors.eTheta);

  // Heading error slowdown: only at very large errors (> 2π/3 = 120°)
  if (absETheta > 2 * Math.PI / 3) refV *= 0.3;

  // Linear command: feed-forward + proportional longitudinal error
  const kxScaled = 2.0 * Math.max(0.6, Math.min(1.0, refV / 0.10));
  let cmdLinear = refV * Math.cos(errors.eTheta) + kxScaled * errors.bodyEx;

  // Angular command: FL-inspired structure with proven proportional lateral
  const feedForwardW = Math.abs(ref.w) < 0.001 ? 0 : ref.w;

  // Lateral: direct proportional with decay once yaw error is large enough
  // to make body-frame cross-track feedback fight the corner turn.
  const lateralDecay = absETheta <= Math.PI / 4 ? 1.0 : Math.max(0.5, Math.abs(Math.cos(errors.eTheta)));
  const lateralTerm = NAV.ky * lateralDecay * errors.bodyEy;

  // Heading: FL-style sin(eTheta) for smooth convergence + straight-line guard
  const headingTerm = NAV.kth * Math.sin(errors.eTheta);
  const straightGuard = Math.abs(ref.w) < 0.001 ? 0.4 * errors.eTheta : 0;

  let angularCorrection = lateralTerm + headingTerm + straightGuard;
  if (segmentChanged) {
    angularCorrection = clamp(angularCorrection, -0.85, 0.85);
  }
  let cmdAngular = feedForwardW + angularCorrection;

  // Apply command limits
  cmdLinear = clamp(cmdLinear, -NAV.maxLinear, NAV.maxLinear);
  if (isFinalSegment && distance(pose, finalPoint) < NAV.slowdownDist) {
    cmdLinear = clamp(cmdLinear, -NAV.approachVel, NAV.approachVel);
  }
  cmdAngular = clamp(cmdAngular, -NAV.turnSpeed, NAV.turnSpeed);
  return { ...errors, cmdLinear, cmdAngular, refV, refW: ref.w };
}

function bodyErrors(pose, ref) {
  const dx = ref.x - pose.x;
  const dy = ref.y - pose.y;
  return {
    eX: dx,
    eY: dy,
    eTheta: normalizeAngle(ref.theta - pose.theta),
    bodyEx: dx * Math.cos(pose.theta) + dy * Math.sin(pose.theta),
    bodyEy: -dx * Math.sin(pose.theta) + dy * Math.cos(pose.theta),
  };
}

function applyRobotDynamics(pose, command, state, dt) {
  const tauV = 0.18;
  const tauW = 0.14;
  state.v += (command.cmdLinear - state.v) * clamp(dt / tauV, 0, 1);
  state.w += (command.cmdAngular - state.w) * clamp(dt / tauW, 0, 1);

  pose.x += state.v * Math.cos(pose.theta) * dt;
  pose.y += state.v * Math.sin(pose.theta) * dt;
  pose.theta = normalizeAngle(pose.theta + state.w * dt);
}

function simulateScenario(scenario, mode) {
  const modeSpec = typeof mode === 'string'
    ? TRAJECTORY_BENCHMARK_MODES.find((item) => item.mode === mode) ?? TRAJECTORY_BENCHMARK_MODES[0]
    : mode;
  const trajectory = new FirmwareLikeTrajectory(scenario.path, modeSpec.trajectory);
  const pose = { ...scenario.initialPose };
  const dynamics = { v: 0, w: 0 };
  const finalPoint = scenario.path[scenario.path.length - 1];
  const controllers = {
    firmware_current: controllerCurrent,
    fl_regularized: controllerFlRegularized,
  };
  const controller = controllers[modeSpec.controller] || controllerCurrent;
  const rows = [];
  let lastTargetIndex = trajectory.targetIndex;
  let finalTurn = false;
  let done = false;

  for (let time = 0; time <= scenario.maxTime; time += NAV.dt) {
    const ref = trajectory.sample(NAV.dt);
    const isFinalSegment = ref.targetIndex >= scenario.path.length - 1;
    const segmentChanged = ref.targetIndex !== lastTargetIndex;
    lastTargetIndex = ref.targetIndex;
    let sample;

    if (trajectory.done && distance(pose, finalPoint) <= NAV.goalTolerance) {
      if (Number.isFinite(scenario.finalHeading)) {
        finalTurn = true;
        const finalErr = normalizeAngle(scenario.finalHeading - pose.theta);
        if (Math.abs(finalErr) < 0.052) {
          sample = { ...bodyErrors(pose, { ...ref, theta: scenario.finalHeading }), cmdLinear: 0, cmdAngular: 0, refV: 0, refW: 0 };
          done = true;
        } else {
          const turnSpeed = Math.abs(finalErr) < 0.26 ? 0.3 : NAV.turnSpeed;
          sample = {
            ...bodyErrors(pose, { ...ref, theta: scenario.finalHeading }),
            cmdLinear: 0,
            cmdAngular: finalErr > 0 ? turnSpeed : -turnSpeed,
            refV: 0,
            refW: 0,
          };
        }
      } else {
        sample = { ...bodyErrors(pose, ref), cmdLinear: 0, cmdAngular: 0, refV: 0, refW: 0 };
        done = true;
      }
    } else {
      sample = controller(pose, ref, finalPoint, isFinalSegment, segmentChanged);
    }

    rows.push({
      t: time,
      ...sample,
      refX: ref.x,
      refY: ref.y,
      refTheta: ref.theta,
      profileType: finalTurn ? 'final_turn' : ref.profileType,
      profileMode: ref.profileMode,
      segmentTime: ref.segmentTime,
      tTotal: ref.tTotal,
      progress: ref.progress,
      poseX: pose.x,
      poseY: pose.y,
      poseTheta: pose.theta,
    });

    if (done) break;
    applyRobotDynamics(pose, sample, dynamics, NAV.dt);
  }

  return summarizeRows(scenario, modeSpec.mode, rows, done);
}

function summarizeRows(scenario, mode, rows, completed) {
  const finalPoint = scenario.path[scenario.path.length - 1];
  const last = rows[rows.length - 1];
  const rms = (selector) => Math.sqrt(rows.reduce((sum, row) => sum + selector(row) ** 2, 0) / rows.length);
  const maxAbs = (selector) => rows.reduce((max, row) => Math.max(max, Math.abs(selector(row))), 0);
  const saturationCount = rows.filter((row) => Math.abs(row.cmdLinear) >= NAV.maxLinear - 1e-6 || Math.abs(row.cmdAngular) >= NAV.turnSpeed - 1e-6).length;
  const profiles = [...new Set(rows.map((row) => row.profileType))].join(',');
  const profileModes = [...new Set(rows.map((row) => row.profileMode).filter(Boolean))].join(',');
  const finalDistance = Math.hypot(last.poseX - finalPoint.x, last.poseY - finalPoint.y);
  const finalHeadingError = Number.isFinite(scenario.finalHeading)
    ? Math.abs(normalizeAngle(scenario.finalHeading - last.poseTheta))
    : Math.abs(last.eTheta);

  return {
    scenario: scenario.name,
    mode,
    completed,
    duration: last.t,
    samples: rows.length,
    rmsBodyEy: rms((row) => row.bodyEy),
    rmsYaw: rms((row) => row.eTheta),
    maxAbsBodyEy: maxAbs((row) => row.bodyEy),
    maxAbsYaw: maxAbs((row) => row.eTheta),
    maxAbsV: maxAbs((row) => row.cmdLinear),
    maxAbsW: maxAbs((row) => row.cmdAngular),
    saturationPct: (100 * saturationCount) / rows.length,
    finalDistance,
    finalHeadingError,
    profiles,
    profileModes,
  };
}

function formatNumber(value, digits = 3) {
  return Number.isFinite(value) ? value.toFixed(digits) : 'n/a';
}

const PLANNER_GRID = {
  width: 28,
  height: 22,
  resolution: 0.05,
};

const PLANNER_SCENARIOS = [
  {
    name: 'open_diagonal',
    start: { x: 2, y: 2 },
    goal: { x: 24, y: 18 },
    blocks: [],
  },
  {
    name: 'wall_gap',
    start: { x: 2, y: 11 },
    goal: { x: 25, y: 11 },
    blocks: Array.from({ length: 22 }, (_, y) => ({ x: 13, y })).filter((p) => p.y !== 10 && p.y !== 11),
  },
  {
    name: 'offset_corridor',
    start: { x: 3, y: 3 },
    goal: { x: 24, y: 17 },
    blocks: [
      ...Array.from({ length: 18 }, (_, x) => ({ x: x + 5, y: 8 })),
      ...Array.from({ length: 18 }, (_, x) => ({ x: x + 5, y: 14 })),
    ].filter((p) => !(p.x >= 11 && p.x <= 13 && p.y === 8) && !(p.x >= 17 && p.x <= 19 && p.y === 14)),
  },
];

function plannerKey(p) {
  return `${p.x},${p.y}`;
}

function buildPlannerGrid(scenario) {
  const blocked = new Set(scenario.blocks.map(plannerKey));
  return {
    ...PLANNER_GRID,
    blocked,
    isBlocked(x, y) {
      return x < 0 || y < 0 || x >= this.width || y >= this.height || blocked.has(`${x},${y}`);
    },
  };
}

function lineOfSight(grid, a, b) {
  let x = a.x;
  let y = a.y;
  const dx = Math.abs(b.x - a.x);
  const dy = Math.abs(b.y - a.y);
  const sx = a.x < b.x ? 1 : -1;
  const sy = a.y < b.y ? 1 : -1;
  let err = dx - dy;

  for (;;) {
    if (grid.isBlocked(x, y)) return false;
    if (x === b.x && y === b.y) return true;
    const e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

function astarRawPath(grid, start, goal) {
  const dirs = [
    [1, 0], [0, 1], [-1, 0], [0, -1],
    [1, 1], [-1, 1], [-1, -1], [1, -1],
  ];
  const open = [{ p: start, g: 0, f: distance(start, goal), parent: null }];
  const best = new Map([[plannerKey(start), open[0]]]);

  while (open.length > 0) {
    open.sort((a, b) => a.f - b.f);
    const current = open.shift();
    if (current.p.x === goal.x && current.p.y === goal.y) {
      const path = [];
      for (let node = current; node; node = node.parent) path.push(node.p);
      return path.reverse();
    }

    for (const [dx, dy] of dirs) {
      const next = { x: current.p.x + dx, y: current.p.y + dy };
      if (grid.isBlocked(next.x, next.y)) continue;
      if (dx !== 0 && dy !== 0 && (grid.isBlocked(current.p.x + dx, current.p.y) || grid.isBlocked(current.p.x, current.p.y + dy))) continue;

      const step = dx !== 0 && dy !== 0 ? Math.SQRT2 : 1;
      const g = current.g + step;
      const key = plannerKey(next);
      const old = best.get(key);
      if (!old || g < old.g) {
        const node = { p: next, g, f: g + distance(next, goal), parent: current };
        best.set(key, node);
        open.push(node);
      }
    }
  }
  return [];
}

function smoothThetaLos(grid, raw) {
  if (raw.length <= 2) return raw;
  const out = [raw[0]];
  let anchor = 0;
  while (anchor < raw.length - 1) {
    let farthest = anchor + 1;
    for (let j = raw.length - 1; j > anchor + 1; j--) {
      if (lineOfSight(grid, raw[anchor], raw[j])) {
        farthest = j;
        break;
      }
    }
    out.push(raw[farthest]);
    anchor = farthest;
  }
  return out;
}

function pathLengthCells(pathPoints) {
  let total = 0;
  for (let i = 1; i < pathPoints.length; i++) total += distance(pathPoints[i - 1], pathPoints[i]);
  return total;
}

function countTurns(pathPoints) {
  let turns = 0;
  let prev = null;
  for (let i = 1; i < pathPoints.length; i++) {
    const dx = Math.sign(pathPoints[i].x - pathPoints[i - 1].x);
    const dy = Math.sign(pathPoints[i].y - pathPoints[i - 1].y);
    const dir = `${dx},${dy}`;
    if (prev && dir !== prev) turns++;
    prev = dir;
  }
  return turns;
}

function minObstacleClearance(grid, pathPoints) {
  if (grid.blocked.size === 0 || pathPoints.length === 0) return Infinity;
  let best = Infinity;
  const obstacles = [...grid.blocked].map((key) => {
    const [x, y] = key.split(',').map(Number);
    return { x, y };
  });

  const samples = [pathPoints[0]];
  for (let i = 1; i < pathPoints.length; i++) {
    const a = pathPoints[i - 1];
    const b = pathPoints[i];
    const steps = Math.max(Math.abs(b.x - a.x), Math.abs(b.y - a.y), 1);
    for (let s = 1; s <= steps; s++) {
      const t = s / steps;
      samples.push({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });
    }
  }

  for (const p of samples) {
    for (const obs of obstacles) best = Math.min(best, distance(p, obs));
  }
  return best * grid.resolution;
}

export function runPlannerBenchmarks() {
  return PLANNER_SCENARIOS.flatMap((scenario) => {
    const grid = buildPlannerGrid(scenario);
    const raw = astarRawPath(grid, scenario.start, scenario.goal);
    const theta = smoothThetaLos(grid, raw);
    return [
      { scenario: scenario.name, mode: 'astar', path: raw, raw },
      { scenario: scenario.name, mode: 'theta_los', path: theta, raw },
    ].map((row) => ({
      scenario: row.scenario,
      mode: row.mode,
      success: row.path.length > 0,
      pathLengthM: pathLengthCells(row.path) * grid.resolution,
      waypointCount: row.path.length,
      rawCount: row.raw.length,
      turnCount: countTurns(row.path),
      clearanceM: minObstacleClearance(grid, row.path),
    }));
  });
}

export function runBenchmarks() {
  return SCENARIOS.flatMap((scenario) => (
    TRAJECTORY_BENCHMARK_MODES.map((mode) => simulateScenario(scenario, mode))
  ));
}

export function runProfileBenchmarks() {
  const cases = [
    { name: 'adaptive_type_i_short', distanceM: 0.02, expectedProfile: 'type_i' },
    { name: 'adaptive_type_iii_medium', distanceM: 0.20, expectedProfile: 'type_iii' },
    { name: 'adaptive_type_iv_long', distanceM: 0.60, expectedProfile: 'type_iv' },
  ];

  return cases.map((item) => {
    const trajectory = new FirmwareLikeTrajectory(
      [{ x: 0, y: 0 }, { x: item.distanceM, y: 0 }],
      ADAPTIVE_SCURVE_TRAJECTORY,
    );
    const rows = [];
    for (let t = 0; t <= 20 && !trajectory.done; t += NAV.dt) {
      const ref = trajectory.sample(NAV.dt);
      rows.push({ t, ...ref });
    }
    const movingRows = rows.filter((row) => row.profileType !== 'done');
    const profileType = movingRows.find((row) => row.profileType !== 'idle')?.profileType ?? 'done';
    const maxV = movingRows.reduce((max, row) => Math.max(max, Math.abs(row.v)), 0);
    const maxProgress = rows.reduce((max, row) => Math.max(max, row.progress), 0);
    const last = rows[rows.length - 1] ?? trajectory.ref;
    const plan = trajectory.lastPlan ?? {};

    return {
      name: item.name,
      distanceM: item.distanceM,
      expectedProfile: item.expectedProfile,
      profileType,
      completed: trajectory.done,
      duration: last.tTotal || last.segmentTime || 0,
      maxV,
      maxProgress,
      finalX: last.x,
      plan,
    };
  });
}

export function renderMarkdown(results, plannerResults = [], profileResults = []) {
  const lines = [
    '# Trajectory Tracking Offline Benchmark',
    '',
    'Generated by `npm.cmd run benchmark:trajectory`.',
    '',
    'This benchmark is a firmware-safe research tool. It mirrors the current ESP32-S3 v2 trajectory sampler, keeps `legacy_ramp` as the default reference generator, and compares the opt-in `adaptive_scurve` mode before any real-robot rollout.',
    '',
    '## Scenarios',
    '',
    '| Scenario | Purpose |',
    '| --- | --- |',
    ...SCENARIOS.map((scenario) => `| ${scenario.name} | ${scenario.description} |`),
    '',
    '## Results',
    '',
    '| Scenario | Mode | Done | Time (s) | RMS bodyEy (m) | RMS yaw (rad) | Max bodyEy (m) | Max yaw (rad) | Max v | Max w | Sat % | Final dist (m) | Profile modes | Profiles |',
    '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |',
    ...results.map((row) => [
      row.scenario,
      row.mode,
      row.completed ? 'yes' : 'no',
      formatNumber(row.duration, 2),
      formatNumber(row.rmsBodyEy),
      formatNumber(row.rmsYaw),
      formatNumber(row.maxAbsBodyEy),
      formatNumber(row.maxAbsYaw),
      formatNumber(row.maxAbsV),
      formatNumber(row.maxAbsW),
      formatNumber(row.saturationPct, 1),
      formatNumber(row.finalDistance),
      row.profileModes,
      row.profiles,
    ].join(' | ')).map((line) => `| ${line} |`),
    '',
    '## Reading Guide',
    '',
    '- `firmware_current` mirrors the controller form used in `Navigator`: feed-forward reference velocity plus body-frame error feedback.',
    '- `firmware_current_adaptive_scurve` uses the same controller with the opt-in `adaptive_scurve` trajectory mode, so rollout risk stays isolated to the reference generator.',
    '- `fl_regularized` mirrors the upgraded firmware mode: lateral decay when yaw error is large, Kx gain scheduling by reference speed, and angular-correction limiting on segment transitions.',
    '- A useful candidate should finish all scenarios, reduce RMS body-frame error, avoid persistent saturation, and keep final distance within the firmware goal tolerance before any real-robot test.',
    '',
  ];

  if (profileResults.length > 0) {
    lines.push(
      '## Adaptive S-Curve Profile Checks',
      '',
      '| Case | Expected | Actual | Done | Distance (m) | Duration (s) | Max v | Peak a | Plan tJ | Plan tA | Plan tV |',
      '| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |',
      ...profileResults.map((row) => `| ${row.name} | ${row.expectedProfile} | ${row.profileType} | ${row.completed ? 'yes' : 'no'} | ${formatNumber(row.distanceM)} | ${formatNumber(row.duration, 2)} | ${formatNumber(row.maxV)} | ${formatNumber(row.plan.aPeak)} | ${formatNumber(row.plan.tJ)} | ${formatNumber(row.plan.tA)} | ${formatNumber(row.plan.tV)} |`),
      '',
    );
  }

  if (plannerResults.length > 0) {
    lines.push(
      '## Planner Debug Benchmark',
      '',
      'This lightweight grid benchmark mirrors the new firmware option: `astar` keeps the raw A* node chain, while `theta_los` applies line-of-sight waypoint reduction after A* and should fall back safely when no raw path exists.',
      '',
      '| Scenario | Mode | Success | Path length (m) | Waypoints | Raw nodes | Turns | Min clearance (m) |',
      '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |',
      ...plannerResults.map((row) => `| ${row.scenario} | ${row.mode} | ${row.success ? 'yes' : 'no'} | ${formatNumber(row.pathLengthM)} | ${row.waypointCount} | ${row.rawCount} | ${row.turnCount} | ${formatNumber(row.clearanceM)} |`),
      '',
    );
  }

  return `${lines.join('\n')}\n`;
}

function main() {
  const results = runBenchmarks();
  const plannerResults = runPlannerBenchmarks();
  const profileResults = runProfileBenchmarks();
  const markdown = renderMarkdown(results, plannerResults, profileResults);
  const outputPath = path.join(process.cwd(), 'docs', '03_Research', 'trajectory_tracking_benchmark.md');
  fs.mkdirSync(path.dirname(outputPath), { recursive: true });
  fs.writeFileSync(outputPath, markdown, 'utf8');
  console.log(markdown);
  console.log(`Wrote ${outputPath}`);
}

const thisFile = fileURLToPath(import.meta.url);
if (process.argv[1] && path.resolve(process.argv[1]) === thisFile) {
  main();
}
