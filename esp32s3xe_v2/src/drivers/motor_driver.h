/**
 * Motor Driver — L298N H-Bridge
 * Encapsulates PWM setup, direction control, and motor inversion.
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

enum MotorSide { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 };

void motor_init();
void motor_set(MotorSide side, float pwm);  // pwm: -255..+255
void motor_brake(MotorSide side);            // Active H-bridge brake (IN1=HIGH, IN2=HIGH)

#endif
