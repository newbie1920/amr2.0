// ============================================================
//   AMR 2.0 — Battery ADC Monitor
// ============================================================

use esp_idf_hal::adc::oneshot::config::AdcChannelConfig;
use esp_idf_hal::adc::oneshot::{AdcDriver, AdcChannelDriver};
use esp_idf_hal::adc::{ADC1, ADCU1, ADCCH1, attenuation};
use esp_idf_hal::gpio::Gpio2;
use anyhow::Result;

pub struct BatteryMonitor<'a> {
    channel: AdcChannelDriver<'a, ADCCH1<ADCU1>, AdcDriver<'a, ADCU1>>,
}

impl<'a> BatteryMonitor<'a> {
    pub fn new(
        adc_peri: ADC1<'a>,
        pin: Gpio2<'a>,
    ) -> Result<Self> {
        let adc = AdcDriver::new(adc_peri)?;
        let config = AdcChannelConfig {
            attenuation: attenuation::DB_12,
            ..Default::default()
        };
        let channel = AdcChannelDriver::new(adc, pin, &config)?;
        Ok(Self { channel })
    }

    pub fn read_voltage(&mut self) -> Result<f32> {
        let raw = self.channel.read()?;
        // Atten12dB max voltage is ~3.1V (on ESP32S3 it's 3.1V to 3.3V)
        let voltage = (raw as f32 / 1000.0) * crate::config::BATT_SCALE_FACTOR;
        Ok(voltage)
    }

    pub fn get_percentage(&mut self) -> Result<u8> {
        let v = self.read_voltage()?;
        let mut pct = (v - crate::config::BATT_MIN_V) / (crate::config::BATT_MAX_V - crate::config::BATT_MIN_V) * 100.0;
        if pct < 0.0 { pct = 0.0; }
        if pct > 100.0 { pct = 100.0; }
        Ok(pct as u8)
    }
}

