/**
 * Robot State — Global instances
 */

#include "robot_state.h"
#include "occupancy_grid.h"

RobotState state;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
OccupancyGridMapper gridMapper;
