/**
 * Quadrature Encoder Driver — ISR Implementation
 */

#include "encoder_driver.h"
#include "config.h"
#include "log.h"

volatile long encoderLeftTicks  = 0;
volatile long encoderRightTicks = 0;
volatile int lastEncodedLeft  = 0;
volatile int lastEncodedRight = 0;

// ── ISR Handlers (IRAM) ─────────────────────────────────────

void IRAM_ATTR leftEncoderISR() {
    int MSB = digitalRead(ENCODER_LEFT_A);
    int LSB = digitalRead(ENCODER_LEFT_B);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncodedLeft << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        INVERT_LEFT_ENCODER ? encoderLeftTicks-- : encoderLeftTicks++;
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        INVERT_LEFT_ENCODER ? encoderLeftTicks++ : encoderLeftTicks--;

    lastEncodedLeft = encoded;
}

void IRAM_ATTR rightEncoderISR() {
    int MSB = digitalRead(ENCODER_RIGHT_A);
    int LSB = digitalRead(ENCODER_RIGHT_B);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncodedRight << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        INVERT_RIGHT_ENCODER ? encoderRightTicks-- : encoderRightTicks++;
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        INVERT_RIGHT_ENCODER ? encoderRightTicks++ : encoderRightTicks--;

    lastEncodedRight = encoded;
}

// ── Init ─────────────────────────────────────────────────────

void encoder_init() {
    pinMode(ENCODER_LEFT_A,  INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B,  INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A),  leftEncoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B),  leftEncoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), rightEncoderISR, CHANGE);

    LOG_I("ENC", "Quadrature encoders initialized (LA=%d LB=%d RA=%d RB=%d)",
          ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
}
