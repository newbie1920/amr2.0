/**
 * AMR 2.0 v2 — Network Communication
 * WiFi + WebSocket + OTA + Telemetry
 * Uses RobotState centralized struct — zero extern globals.
 */

#ifndef NETWORK_COMM_H
#define NETWORK_COMM_H

#include <Arduino.h>

struct PathPlanDebug;
struct Waypoint;

void init_network();
void update_network();
void flush_network();  // Extra webSocket.loop() to flush TX immediately
void broadcast_telemetry();
void broadcast_nav_path(const PathPlanDebug& debug, const Waypoint* waypoints, int waypointCount);
void send_occupancy_grid();
void setArchitectureProfile(const char* profile);

#endif // NETWORK_COMM_H
