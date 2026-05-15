export const PC_GOAL_DIRECT_BLOCK_COST = 200;

function hasGridMethods(grid) {
  return !!(
    grid &&
    typeof grid.worldToGrid === 'function' &&
    typeof grid.inBounds === 'function'
  );
}

function isCellBlocked(grid, gx, gy, blockCost = PC_GOAL_DIRECT_BLOCK_COST) {
  if (!grid.inBounds(gx, gy)) return true;

  if (typeof grid.isOccupied === 'function' && grid.isOccupied(gx, gy)) {
    return true;
  }

  const idx = gy * grid.width + gx;
  if (grid.logOdds && grid.logOdds[idx] > 0.3) {
    return true;
  }

  if (grid.costmap && grid.costmap[idx] >= blockCost) {
    return true;
  }

  return false;
}

export function isDirectPathClear(grid, start, goal, options = {}) {
  if (!hasGridMethods(grid)) return true;

  const blockCost = options.blockCost ?? PC_GOAL_DIRECT_BLOCK_COST;
  const sg = grid.worldToGrid(start.x, start.y);
  const eg = grid.worldToGrid(goal.x, goal.y);

  let x0 = sg.gx;
  let y0 = sg.gy;
  const x1 = eg.gx;
  const y1 = eg.gy;
  const dx = Math.abs(x1 - x0);
  const dy = Math.abs(y1 - y0);
  const sx = x0 < x1 ? 1 : -1;
  const sy = y0 < y1 ? 1 : -1;
  let err = dx - dy;

  while (true) {
    if (isCellBlocked(grid, x0, y0, blockCost)) return false;
    if (x0 === x1 && y0 === y1) break;
    const e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  return true;
}

export function buildDirectPath(start, goal, options = {}) {
  const spacing = Math.max(0.05, options.spacing ?? 0.35);
  const dx = goal.x - start.x;
  const dy = goal.y - start.y;
  const distance = Math.hypot(dx, dy);
  const steps = Math.max(1, Math.ceil(distance / spacing));
  const path = [];

  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    path.push({
      x: start.x + dx * t,
      y: start.y + dy * t,
    });
  }

  return path;
}
