import { OccupancyGrid } from '../src/core/lidarMapper.js';
import findPathOnGrid from '../src/core/lidarPathfinder.js';
import { SimWorld } from '../src/core/sim/simWorld.js';
import { injectTrafficIntoGridData } from '../src/core/trafficManager.js';

// Create simulated world
const simWorld = new SimWorld();

// Create occupancy grid from segments
const grid = OccupancyGrid.createFromSegments(simWorld.staticSegments, 0.25, 1.0);

console.log("Grid size:", grid.width, grid.height);

const startX = 5.0;
const startY = 1.5;
const goalX = 1.5;
const goalY = 0.5;

console.log(`Finding path from (${startX}, ${startY}) to (${goalX}, ${goalY})...`);

const result = findPathOnGrid(grid, startX, startY, goalX, goalY, {
    allowUnknown: true,
    maxIterations: 15000,
    useCostmap: true
});

console.log("Pathfinding result:", result.success, "waypoints:", result.path.length);
if (!result.success) {
    const startG = grid.worldToGrid(startX, startY);
    const goalG = grid.worldToGrid(goalX, goalY);
    console.log("Start cell:", startG, "cost:", grid.getCost(startG.gx, startG.gy));
    console.log("Goal cell:", goalG, "cost:", grid.getCost(goalG.gx, goalG.gy));
}
