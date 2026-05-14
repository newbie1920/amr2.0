import assert from 'node:assert/strict';
import test from 'node:test';
import OccupancyGrid from '../src/core/lidarMapper.js';

function makeRleFrame({
  viewW = 1,
  viewH = 1,
  resolution = 0.1,
  robotX = 0,
  robotY = 0,
  robotHeading = 0,
  originX = 0,
  originY = 0,
  viewStartX = 0,
  viewStartY = 0,
  fullSize = 8,
  runs = [[50, 1]],
} = {}) {
  const payloadBytes = runs.length * 2;
  const buffer = new ArrayBuffer(35 + payloadBytes);
  const view = new DataView(buffer);
  let offset = 0;

  view.setUint8(offset, 0x01); offset += 1;
  view.setUint16(offset, viewW, true); offset += 2;
  view.setUint16(offset, viewH, true); offset += 2;
  view.setFloat32(offset, resolution, true); offset += 4;
  view.setFloat32(offset, robotX, true); offset += 4;
  view.setFloat32(offset, robotY, true); offset += 4;
  view.setFloat32(offset, robotHeading, true); offset += 4;
  view.setFloat32(offset, originX, true); offset += 4;
  view.setFloat32(offset, originY, true); offset += 4;
  view.setUint16(offset, viewStartX, true); offset += 2;
  view.setUint16(offset, viewStartY, true); offset += 2;
  view.setUint16(offset, fullSize, true); offset += 2;

  for (const [value, count] of runs) {
    view.setUint8(offset++, value);
    view.setUint8(offset++, count);
  }

  return buffer;
}

test('updateFromRLEWindow clears stale cells when ESP grid origin shifts', () => {
  const firstFrame = makeRleFrame({
    originX: 0,
    originY: 0,
    viewStartX: 1,
    viewStartY: 1,
    runs: [[100, 1]],
  });

  const grid = OccupancyGrid.updateFromRLEWindow(firstFrame);
  const oldOccupiedIdx = 1 * grid.width + 1;
  assert.ok(grid.logOdds[oldOccupiedIdx] > 0);

  const shiftedFrame = makeRleFrame({
    originX: 1,
    originY: 0,
    viewStartX: 2,
    viewStartY: 2,
    runs: [[50, 1]],
  });

  const shiftedGrid = OccupancyGrid.updateFromRLEWindow(shiftedFrame, grid);
  assert.equal(shiftedGrid.logOdds[oldOccupiedIdx], 0);
  assert.equal(shiftedGrid.originX, 1);
});
