/**
 * LiDAR Driver — RPLidar A1M8
 * Handles UART init, motor PWM, start/stop scan, device detection.
 */

#ifndef LIDAR_A1M8_H
#define LIDAR_A1M8_H

#include <Arduino.h>
#include <RPLidar.h>

bool lidar_init();            // Full init: serial + motor + device check + startScan
bool lidar_read_point(float& angle, float& distance, uint8_t& quality);
void lidar_reset();           // Recovery: stop + flush + re-detect + startScan
void lidar_motor_set(uint8_t pwm);  // Direct motor PWM control

extern HardwareSerial lidarSerial;
extern RPLidar lidarDevice;

#endif
