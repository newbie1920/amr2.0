/**
 * Quadrature Encoder Driver
 * ISR-based tick counting for differential drive.
 */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>

void encoder_init();

// Direct access to tick counters (volatile, ISR-updated)
extern volatile long encoderLeftTicks;
extern volatile long encoderRightTicks;
extern volatile int lastEncodedLeft;
extern volatile int lastEncodedRight;

#endif
