import { ROBOT_RADIUS } from './warehouse.js';

/**
 * Injects the current positions and future planned paths of OTHER robots
 * into the given gridData as lethal obstacles (cost=254, logOdds=5.0).
 * This ensures the pathfinder routes around them.
 * 
 * @param {Object} gridData - Serialized OccupancyGrid data
 * @param {string} currentRobotId - ID of the robot requesting a path
 * @param {Object} allRobots - robotStore.robots object
 * @param {Object} navSessions - navStore.appNavigationSessions object
 * @returns {Object} A cloned gridData with traffic injected
 */
export function injectTrafficIntoGridData(gridData, currentRobotId, allRobots, navSessions) {
  if (!gridData || !gridData.costmap || !gridData.logOdds) return gridData;

  // Clone arrays to avoid modifying the original map in the store
  const trafficGridData = { 
    ...gridData, 
    costmap: new Uint8Array(gridData.costmap),
    logOdds: new Float32Array(gridData.logOdds)
  };
  
  const { width, height, resolution, originX, originY } = trafficGridData;

  // Traffic Configuration
  const LOOKAHEAD_DIST = 4.0; // Predict up to 4 meters into the future path
  const OBSTACLE_RADIUS = ROBOT_RADIUS + 0.15; // Robot radius + 15cm safety padding
  const radiusCells = Math.ceil(OBSTACLE_RADIUS / resolution);

  // Helper to mark a world coordinate as a lethal obstacle
  const applyLethalCost = (wx, wy) => {
    const gx = Math.floor((wx - originX) / resolution);
    const gy = Math.floor((wy - originY) / resolution);
    
    // Fill a circle of radius `radiusCells`
    for (let dy = -radiusCells; dy <= radiusCells; dy++) {
      for (let dx = -radiusCells; dx <= radiusCells; dx++) {
        const nx = gx + dx;
        const ny = gy + dy;
        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
          // Check circular distance
          if (Math.hypot(dx, dy) <= radiusCells) {
            const idx = ny * width + nx;
            trafficGridData.costmap[idx] = 254; // Lethal obstacle
            trafficGridData.logOdds[idx] = Math.max(trafficGridData.logOdds[idx], 5.0); // High confidence obstacle
          }
        }
      }
    }
  };

  for (const robotId of Object.keys(allRobots)) {
    if (robotId === currentRobotId) continue;

    const robot = allRobots[robotId];
    if (!robot || !robot.telemetry) continue;

    // 1. Mark the other robot's CURRENT position as an obstacle
    const cx = robot.telemetry.x || 0;
    const cy = robot.telemetry.y || 0;
    applyLethalCost(cx, cy);

    // 2. Mark the other robot's FUTURE PATH as an obstacle (if navigating)
    const session = navSessions[robotId];
    if (session && session.active && session.path && session.path.length > 0) {
      let accumDist = 0;
      let lastWp = { x: cx, y: cy };
      
      // Start from their current waypoint index
      const startIdx = session.currentWaypointIndex || 0;

      for (let i = startIdx; i < session.path.length; i++) {
        const wp = session.path[i];
        const segDist = Math.hypot(wp.x - lastWp.x, wp.y - lastWp.y);
        
        // We only reserve the next LOOKAHEAD_DIST meters to avoid blocking the whole map unnecessarily
        if (accumDist + segDist > LOOKAHEAD_DIST) {
          break; // Stop injecting path after Lookahead distance
        }

        applyLethalCost(wp.x, wp.y);
        
        accumDist += segDist;
        lastWp = wp;
      }
    }
  }

  return trafficGridData;
}
