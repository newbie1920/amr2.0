import assert from 'node:assert/strict';
import test from 'node:test';

import { RobotConnection } from '../src/core/robotProtocol.js';

test('RobotConnection stores nav_path debug telemetry for onboard RViz drawing', () => {
  const robot = new RobotConnection('127.0.0.1', 81, 'test');
  let callbackTelemetry = null;
  robot.onTelemetry = (telemetry) => {
    callbackTelemetry = telemetry;
  };

  robot._handleNavPathMessage({
    type: 'nav_path',
    planner: 'theta_los',
    ok: true,
    reason: 'ok',
    planId: 42,
    goal: [1.2, -0.4],
    waypoints: [[0, 0], [0.5, 0.1], { x: 1.2, y: -0.4 }],
    raw: [[0, 0], ['bad', 0], [0.1, 0], [1.2, -0.4]],
    rawTotal: 4,
    rawStride: 2,
    rawCount: 3,
    debugPath: true,
  });

  assert.equal(callbackTelemetry, robot.telemetry);
  assert.equal(robot.telemetry.planner.mode, 'theta_los');
  assert.equal(robot.telemetry.planner.debugPath, true);
  assert.equal(robot.telemetry.pathDebug.planId, 42);
  assert.equal(robot.telemetry.pathDebug.ok, true);
  assert.deepEqual(robot.telemetry.pathDebug.goal, { x: 1.2, y: -0.4 });
  assert.deepEqual(robot.telemetry.path, [
    { x: 0, y: 0 },
    { x: 0.5, y: 0.1 },
    { x: 1.2, y: -0.4 },
  ]);
  assert.deepEqual(robot.telemetry.pathDebug.raw, [
    { x: 0, y: 0 },
    { x: 0.1, y: 0 },
    { x: 1.2, y: -0.4 },
  ]);
});

test('RobotConnection preserves planner config when nav_path raw debug is disabled', () => {
  const robot = new RobotConnection('127.0.0.1', 81, 'test');

  robot._handleNavPathMessage({
    type: 'nav_path',
    planner: 'astar',
    ok: false,
    reason: 'no_path',
    planId: 7,
    goal: { x: 2, y: 3 },
    waypoints: [],
    raw: [],
    rawTotal: 0,
    rawStride: 1,
    rawCount: 0,
    debugPath: false,
  });

  assert.equal(robot.telemetry.planner.mode, 'astar');
  assert.equal(robot.telemetry.planner.debugPath, false);
  assert.equal(robot.telemetry.pathDebug.ok, false);
  assert.equal(robot.telemetry.pathDebug.reason, 'no_path');
  assert.deepEqual(robot.telemetry.pathDebug.goal, { x: 2, y: 3 });
  assert.deepEqual(robot.telemetry.path, []);
});
