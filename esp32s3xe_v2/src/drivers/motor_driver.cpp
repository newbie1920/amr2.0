/**
 * Motor Driver — L298N Implementation
 */

#include "motor_driver.h"
#include "config.h"
#include "log.h"

// Internal pin config per side
struct MotorPins {
    uint8_t en, in1, in2;
    uint8_t pwmChannel;
    bool invert;
};

static const MotorPins motors[2] = {
    { MOTOR_LEFT_EN,  MOTOR_LEFT_IN1,  MOTOR_LEFT_IN2,  0, INVERT_LEFT_MOTOR  },
    { MOTOR_RIGHT_EN, MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, 1, INVERT_RIGHT_MOTOR },
};

void motor_init() {
    for (int i = 0; i < 2; i++) {
        pinMode(motors[i].in1, OUTPUT);
        pinMode(motors[i].in2, OUTPUT);
        pinMode(motors[i].en, OUTPUT);
        analogWrite(motors[i].en, 0);
    }
    LOG_I("MOTOR", "L298N initialized (L_EN=%d, R_EN=%d)", MOTOR_LEFT_EN, MOTOR_RIGHT_EN);
}

void motor_set(MotorSide side, float pwm) {
    const MotorPins& m = motors[side];
    if (m.invert) pwm = -pwm;

    int pwmInt = constrain((int)pwm, -255, 255);

    if (pwmInt > 0) {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, LOW);
    } else if (pwmInt < 0) {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, HIGH);
        pwmInt = -pwmInt;
    } else {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, LOW);
    }

    analogWrite(m.en, pwmInt);
}

void motor_brake(MotorSide side) {
    const MotorPins& m = motors[side];
    // H-bridge short-brake: IN1=HIGH, IN2=HIGH, PWM=MAX
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, HIGH);
    analogWrite(m.en, 255);
}
