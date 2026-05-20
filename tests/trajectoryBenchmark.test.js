import assert from 'node:assert/strict';
import test from 'node:test';
import { renderMarkdown, runBenchmarks, runPlannerBenchmarks, runProfileBenchmarks } from '../scripts/benchmark_trajectory_tracking.js';

test('trajectory benchmark produces finite paired results', () => {
  const results = runBenchmarks();
  assert.equal(results.length, 18);

  for (const row of results) {
    assert.equal(Number.isFinite(row.duration), true);
    assert.equal(Number.isFinite(row.rmsBodyEy), true);
    assert.equal(Number.isFinite(row.rmsYaw), true);
    assert.equal(Number.isFinite(row.maxAbsV), true);
    assert.equal(Number.isFinite(row.maxAbsW), true);
  }

  const markdown = renderMarkdown(results);
  assert.match(markdown, /firmware_current/);
  assert.match(markdown, /firmware_current_adaptive_scurve/);
  assert.match(markdown, /fl_regularized/);
  assert.match(markdown, /short_straight_omega_zero/);

  const flRows = results.filter((row) => row.mode === 'fl_regularized');
  assert.equal(flRows.every((row) => row.completed), true);
});

test('adaptive scurve classifies Type I III IV profiles without overshoot', () => {
  const results = runProfileBenchmarks();
  assert.equal(results.length, 3);

  for (const row of results) {
    assert.equal(row.completed, true);
    assert.equal(row.profileType, row.expectedProfile);
    assert.ok(row.maxV <= 0.300001);
    assert.ok(row.plan.aPeak <= 0.300001);
    assert.ok(row.plan.tTotal > 0);
    assert.ok(row.maxProgress <= 1.000001);
    assert.ok(Math.abs(row.finalX - row.distanceM) < 0.001);
  }
});

test('planner benchmark compares astar and theta_los debug modes', () => {
  const plannerResults = runPlannerBenchmarks();
  assert.equal(plannerResults.length, 6);

  for (const row of plannerResults) {
    assert.equal(row.success, true);
    assert.equal(Number.isFinite(row.pathLengthM), true);
    assert.equal(Number.isFinite(row.clearanceM) || row.clearanceM === Infinity, true);
    assert.ok(row.waypointCount > 0);
    assert.ok(row.rawCount >= row.waypointCount || row.mode === 'theta_los');
  }

  const astarRows = plannerResults.filter((row) => row.mode === 'astar');
  const thetaRows = plannerResults.filter((row) => row.mode === 'theta_los');
  assert.equal(astarRows.length, thetaRows.length);
  assert.ok(thetaRows.some((row, index) => row.waypointCount < astarRows[index].waypointCount));

  const markdown = renderMarkdown(runBenchmarks(), plannerResults);
  assert.match(markdown, /Planner Debug Benchmark/);
  assert.match(markdown, /theta_los/);
});
