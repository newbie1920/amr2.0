#ifndef WHEEL_PID_H
#define WHEEL_PID_H

#include <Arduino.h>

class WheelPID {
private:
    float Kp, Ki, Kd, FF;
    float error_sum = 0.0;
    float last_error = 0.0;
    float dt;
    
    // Limits
    float max_integral;
    float deadzone_pwm;

public:
    WheelPID(float kp, float ki, float kd, float ff, float sample_time, float max_i = 5.0f, float dzone = 0.0f) 
        : Kp(kp), Ki(ki), Kd(kd), FF(ff), dt(sample_time), max_integral(max_i), deadzone_pwm(dzone) {}

    float update(float velocity_meas, float velocity_ref) {
        // Skip control if reference is zero to allow free spin / brake to kick in
        if (fabs(velocity_ref) < 0.01f && fabs(velocity_meas) < 0.01f) {
            error_sum = 0.0;
            last_error = 0.0;
            return 0.0;
        }

        float error = velocity_ref - velocity_meas;
        
        // Integral with Anti-windup
        error_sum += error * dt;
        if (error_sum > max_integral) error_sum = max_integral;
        if (error_sum < -max_integral) error_sum = -max_integral;

        // Derivative
        float d_error = (error - last_error) / dt;

        // Control Output
        float u_out = (FF * velocity_ref) + (Kp * error) + (Ki * error_sum) + (Kd * d_error);

        // Deadzone compensation
        if (velocity_ref > 0.01f) {
            u_out += deadzone_pwm;
        } else if (velocity_ref < -0.01f) {
            u_out -= deadzone_pwm;
        }

        last_error = error;
        
        return u_out;
    }

    void reset() {
        error_sum = 0.0;
        last_error = 0.0;
    }
};

#endif // WHEEL_PID_H
