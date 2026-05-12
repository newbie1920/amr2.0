pub struct WheelPid {
    kp: f32,
    ki: f32,
    kd: f32,
    max_pwm: f32,
    integral: f32,
    prev_error: f32,
}

impl WheelPid {
    pub fn new(kp: f32, ki: f32, kd: f32, max_pwm: f32) -> Self {
        Self {
            kp, ki, kd, max_pwm,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn update(&mut self, current_vel: f32, target_vel: f32, dt: f32) -> f32 {
        let error = target_vel - current_vel;
        self.integral += error * dt;
        
        // Anti-windup
        if self.integral > self.max_pwm {
            self.integral = self.max_pwm;
        } else if self.integral < -self.max_pwm {
            self.integral = -self.max_pwm;
        }

        let derivative = (error - self.prev_error) / dt;
        self.prev_error = error;

        // Feedforward
        let mut feedforward = 0.0;
        if target_vel > 0.01 {
            feedforward = 50.0 + target_vel * 150.0;
        } else if target_vel < -0.01 {
            feedforward = -50.0 + target_vel * 150.0;
        }

        let mut output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative) + feedforward;

        if output > self.max_pwm {
            output = self.max_pwm;
        } else if output < -self.max_pwm {
            output = -self.max_pwm;
        }

        output
    }
}
