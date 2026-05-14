import test from 'node:test';
import assert from 'node:assert/strict';

import { VelocityMux, VEL_SOURCE } from '../src/core/velocityMux.js';

function quietMuxLogs(fn) {
  const originalLog = console.log;
  console.log = () => {};
  try {
    fn();
  } finally {
    console.log = originalLog;
  }
}

test('VelocityMux routes the highest-priority active source', () => {
  quietMuxLogs(() => {
    const mux = new VelocityMux();
    const emitted = [];
    mux.onVelocityChanged = (linear, angular, source) => emitted.push({ linear, angular, source });

    mux.send(VEL_SOURCE.NAVIGATION, 0.2, 0.1);
    mux.send(VEL_SOURCE.MANUAL, 0.0, 0.5);

    assert.equal(mux.activeSource, VEL_SOURCE.MANUAL);
    assert.deepEqual(emitted.at(-1), { linear: 0.0, angular: 0.5, source: VEL_SOURCE.MANUAL });
  });
});

test('VelocityMux falls back when a higher-priority source is released', () => {
  quietMuxLogs(() => {
    const mux = new VelocityMux();
    const emitted = [];
    mux.onVelocityChanged = (linear, angular, source) => emitted.push({ linear, angular, source });

    mux.send(VEL_SOURCE.NAVIGATION, 0.2, 0.1);
    mux.send(VEL_SOURCE.MANUAL, 0.0, 0.5);
    mux.release(VEL_SOURCE.MANUAL);

    assert.equal(mux.activeSource, VEL_SOURCE.NAVIGATION);
    assert.deepEqual(emitted.at(-1), { linear: 0.2, angular: 0.1, source: VEL_SOURCE.NAVIGATION });
  });
});

test('VelocityMux tick expires stale commands', () => {
  quietMuxLogs(() => {
    const mux = new VelocityMux();
    mux.timeoutMs = 1;
    mux.send(VEL_SOURCE.NAVIGATION, 0.2, 0.1);

    const nav = mux.sources.get(VEL_SOURCE.NAVIGATION);
    nav.timestamp = Date.now() - 10;
    mux.tick();

    assert.equal(mux.activeSource, null);
    assert.equal(mux.lastLinear, 0);
    assert.equal(mux.lastAngular, 0);
  });
});

test('VelocityMux emergencyStop clears sources and emits stop command', () => {
  quietMuxLogs(() => {
    const mux = new VelocityMux();
    const emitted = [];
    mux.onVelocityChanged = (linear, angular, source) => emitted.push({ linear, angular, source });

    mux.send(VEL_SOURCE.NAVIGATION, 0.2, 0.1);
    mux.emergencyStop();

    assert.equal(mux.sources.size, 0);
    assert.equal(mux.activeSource, null);
    assert.deepEqual(emitted.at(-1), { linear: 0, angular: 0, source: 'emergency_stop' });
  });
});
