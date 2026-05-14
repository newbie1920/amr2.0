import test from 'node:test';
import assert from 'node:assert/strict';

import { clamp, degToRad, distance, normalizeAngle, radToDeg } from '../src/core/mathUtils.js';

test('normalizeAngle keeps angles inside [-PI, PI]', () => {
  assert.equal(normalizeAngle(0), 0);
  assert.ok(normalizeAngle(3 * Math.PI) <= Math.PI);
  assert.ok(normalizeAngle(-3 * Math.PI) >= -Math.PI);
  assert.ok(Math.abs(normalizeAngle((5 * Math.PI) / 2) - Math.PI / 2) < 1e-12);
});

test('distance and angle conversion helpers are stable', () => {
  assert.equal(distance(0, 0, 3, 4), 5);
  assert.ok(Math.abs(degToRad(180) - Math.PI) < 1e-12);
  assert.ok(Math.abs(radToDeg(Math.PI / 2) - 90) < 1e-12);
});

test('clamp bounds values inclusively', () => {
  assert.equal(clamp(5, 0, 10), 5);
  assert.equal(clamp(-1, 0, 10), 0);
  assert.equal(clamp(11, 0, 10), 10);
});
