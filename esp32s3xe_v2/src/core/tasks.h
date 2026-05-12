/**
 * FreeRTOS Tasks — Core task definitions
 * controlTask    (Core 1, 50Hz) — Motor PID + odometry + navigator + DWA
 * lidarTask      (Core 0)       — LiDAR scan + SLAM
 * pathfinderTask (Core 0)       — A* path planning (on-demand)
 * explorationTask(Core 0)       — Frontier-based exploration
 */

#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include <freertos/queue.h>

extern QueueHandle_t pathfinderQueue;

void tasks_create();
void controlTask(void* pvParameters);
void lidarTask(void* pvParameters);
void pathfinderTask(void* pvParameters);
void explorationTask(void* pvParameters);

#endif

