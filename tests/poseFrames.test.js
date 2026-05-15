import assert from 'node:assert/strict';
import test from 'node:test';

import { resolveMapFramePose } from '../src/core/poseFrames.js';
import { buildDirectPath, isDirectPathClear } from '../src/core/navigationPathPolicy.js';

function makeGrid({ blocked = [], lethal = [] } = {}) {
  const width = 10;
  const height = 10;
  const logOdds = new Float32Array(width * height);
  const costmap = new Uint8Array(width * height);

  for (const [gx, gy] of blocked) {
    logOdds[gy * width + gx] = 1;
  }
  for (const [gx, gy, cost = 253] of lethal) {
    costmap[gy * width + gx] = cost;
  }

  return {
    width,
    height,
    resolution: 1,
    originX: 0,
    originY: 0,
    logOdds,
    costmap,
    inBounds(gx, gy) {
      return gx >= 0 && gx < width && gy >= 0 && gy < height;
    },
    isOccupied(gx, gy) {
      return logOdds[gy * width + gx] > 0.3;
    },
    worldToGrid(wx, wy) {
      return { gx: Math.floor(wx), gy: Math.floor(wy) };
    },
  };
}

test('resolveMapFramePose returns telemetry pose for SimBot', () => {
  const robot = {
    _sim: true,
    telemetry: { x: 1.2, y: 2.3, headingRad: 0.4, architecture: 'pc_slam' },
  };

  assert.deepEqual(resolveMapFramePose(robot, {}, 'sim-1'), {
    x: 1.2,
    y: 2.3,
    theta: 0.4,
    source: 'odom',
    frame: 'odom',
  });
});

test('resolveMapFramePose prefers real PC SLAM map pose', () => {
  const robot = {
    connection: { architectureProfile: 'pc_slam' },
    telemetry: {
      x: 1,
      y: 2,
      headingRad: 0.1,
      slamMapX: 4,
      slamMapY: 5,
      slamMapTheta: 0.7,
    },
  };

  const pose = resolveMapFramePose(robot, { mapToOdom: { r1: { dx: 10, dy: 10, dTheta: 1 } } }, 'r1');
  assert.equal(pose.x, 4);
  assert.equal(pose.y, 5);
  assert.equal(pose.theta, 0.7);
  assert.equal(pose.source, 'slam');
  assert.equal(pose.frame, 'map');
});

test('resolveMapFramePose falls back to mapToOdom for real PC SLAM', () => {
  const robot = {
    adapter: { architectureProfile: 'pc_slam' },
    telemetry: { x: 1, y: 2, headingRad: 0.25 },
  };

  const pose = resolveMapFramePose(robot, { mapToOdom: { r1: { dx: 3, dy: -1, dTheta: 0.5 } } }, 'r1');
  assert.equal(pose.x, 4);
  assert.equal(pose.y, 1);
  assert.equal(pose.theta, 0.75);
  assert.equal(pose.source, 'mapToOdom');
});

test('isDirectPathClear allows clear and inflated-only PC goal paths', () => {
  const grid = makeGrid({ lethal: [[3, 3, 100]] });
  assert.equal(isDirectPathClear(grid, { x: 1.5, y: 1.5 }, { x: 5.5, y: 5.5 }), true);
});

test('isDirectPathClear rejects occupied or lethal cells on the segment', () => {
  const occupiedGrid = makeGrid({ blocked: [[3, 3]] });
  assert.equal(isDirectPathClear(occupiedGrid, { x: 1.5, y: 1.5 }, { x: 5.5, y: 5.5 }), false);

  const lethalGrid = makeGrid({ lethal: [[3, 3, 253]] });
  assert.equal(isDirectPathClear(lethalGrid, { x: 1.5, y: 1.5 }, { x: 5.5, y: 5.5 }), false);
});

test('buildDirectPath inserts intermediate waypoints for a clear straight goal', () => {
  const path = buildDirectPath({ x: 0, y: 0 }, { x: 1, y: 0 }, { spacing: 0.25 });

  assert.equal(path.length, 5);
  assert.deepEqual(path[0], { x: 0, y: 0 });
  assert.deepEqual(path[path.length - 1], { x: 1, y: 0 });
  assert.deepEqual(path.slice(1, -1), [
    { x: 0.25, y: 0 },
    { x: 0.5, y: 0 },
    { x: 0.75, y: 0 },
  ]);
});
