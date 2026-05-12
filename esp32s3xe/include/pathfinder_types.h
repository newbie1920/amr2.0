#ifndef PATHFINDER_TYPES_H
#define PATHFINDER_TYPES_H

/**
 * GoToRequest — queued from network_comm or exploration → pathfinderTask
 * Used via xQueueSend(pathfinderQueue, &req, ...)
 */
struct GoToRequest {
    float startX, startY;
    float goalX,  goalY;
    float finalHeading;
};

#endif // PATHFINDER_TYPES_H
