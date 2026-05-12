// ============================================================
//   AMR 2.0 — Motor Controller (L298N + PWM)
// ============================================================

use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::gpio::PinDriver;
use anyhow::Result;

pub struct Motor<'a> {
    pwm: LedcDriver<'a>,
    in1: PinDriver<'a, esp_idf_hal::gpio::Output>,
    in2: PinDriver<'a, esp_idf_hal::gpio::Output>,
    inverted: bool,
}

impl<'a> Motor<'a> {
    pub fn new(
        pwm: LedcDriver<'a>,
        in1: PinDriver<'a, esp_idf_hal::gpio::Output>,
        in2: PinDriver<'a, esp_idf_hal::gpio::Output>,
        inverted: bool,
    ) -> Self {
        Self { pwm, in1, in2, inverted }
    }

    /// Set motor speed (-255.0 to 255.0)
    pub fn set_speed(&mut self, mut speed: f32, brake: bool) -> Result<()> {
        if self.inverted {
            speed = -speed;
        }

        if speed > 255.0 { speed = 255.0; }
        if speed < -255.0 { speed = -255.0; }

        let pwr = speed.abs() as u32;

        if speed > 0.0 {
            self.in1.set_high()?;
            self.in2.set_low()?;
            self.pwm.set_duty(pwr)?;
        } else if speed < 0.0 {
            self.in1.set_low()?;
            self.in2.set_high()?;
            self.pwm.set_duty(pwr)?;
        } else {
            if brake {
                self.in1.set_high()?;
                self.in2.set_high()?;
                self.pwm.set_duty(255)?;
            } else {
                self.in1.set_low()?;
                self.in2.set_low()?;
                self.pwm.set_duty(0)?;
            }
        }
        Ok(())
    }
}
