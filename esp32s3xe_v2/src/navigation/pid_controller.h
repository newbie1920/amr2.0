/**
 * PID Controller — Wheel velocity control with feedforward.
 * Identical logic to v1 WheelPID, now separated into h/cpp.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class WheelPID {
private:
    float Kp, Ki, Kd, FF;
    float error_sum = 0.0f;
    float last_error = 0.0f;
    float last_output = 0.0f;
    float dt;
    float max_integral;
    float deadzone_pwm;

public:
    WheelPID(float kp, float ki, float kd, float ff, float sample_time,
             float max_i = 5.0f, float dzone = 0.0f)
        : Kp(kp), Ki(ki), Kd(kd), FF(ff), dt(sample_time),
          max_integral(max_i), deadzone_pwm(dzone) {}

    float update(float velocity_meas, float velocity_ref) {
        if (fabsf(velocity_ref) < 0.01f && fabsf(velocity_meas) < 0.01f) {
            error_sum = 0.0f;
            last_error = 0.0f;
            last_output = 0.0f;
            return 0.0f;
        }

        float error = velocity_ref - velocity_meas;

        // Integral with Anti-windup
        error_sum += error * dt;
        error_sum = constrain(error_sum, -max_integral, max_integral);

        // Derivative
        float d_error = (error - last_error) / dt;

        // Control Output = FF + PID
        float u_out = (FF * velocity_ref) + (Kp * error) + (Ki * error_sum) + (Kd * d_error);

        // Deadzone compensation
        if (velocity_ref > 0.01f) u_out += deadzone_pwm;
        else if (velocity_ref < -0.01f) u_out -= deadzone_pwm;

        last_error = error;
        last_output = u_out;
        return u_out;
    }

    void reset() {
        error_sum = 0.0f;
        last_error = 0.0f;
        last_output = 0.0f;
    }

    float getLastError() const { return last_error; }
    float getIntegral() const { return error_sum; }
    float getLastOutput() const { return last_output; }
    float getKp() const { return Kp; }
    float getKi() const { return Ki; }
    float getKd() const { return Kd; }
    float getFf() const { return FF; }
};

#endif // PID_CONTROLLER_H
