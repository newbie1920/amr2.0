// ============================================================
//   AMR 2.0 — INA3221 Power Monitor
// ============================================================

use esp_idf_hal::i2c::I2cDriver;
use anyhow::Result;

pub struct Ina3221;

impl Ina3221 {
    pub const ADDRESS: u8 = 0x40; // Default address for INA3221

    // Register mapping for channels 1, 2, 3
    const CH1_SHUNT_V: u8 = 0x01;
    const CH1_BUS_V: u8 = 0x02;
    const CH2_SHUNT_V: u8 = 0x03;
    const CH2_BUS_V: u8 = 0x04;
    const CH3_SHUNT_V: u8 = 0x05;
    const CH3_BUS_V: u8 = 0x06;

    // Reads a 16-bit register (Big Endian)
    fn read_reg(i2c: &mut I2cDriver<'_>, reg: u8) -> Result<i16> {
        let mut buf = [0u8; 2];
        i2c.write_read(Self::ADDRESS, &[reg], &mut buf, 1000)?;
        Ok(i16::from_be_bytes(buf))
    }

    /// Read bus voltage in Volts (V)
    pub fn read_bus_voltage(i2c: &mut I2cDriver<'_>, channel: usize) -> Result<f32> {
        let reg = match channel {
            1 => Self::CH1_BUS_V,
            2 => Self::CH2_BUS_V,
            3 => Self::CH3_BUS_V,
            _ => return Err(anyhow::anyhow!("Invalid INA3221 channel (must be 1-3)")),
        };
        let raw = Self::read_reg(i2c, reg)?;
        
        // INA3221 Bus Voltage: bits 14..3 (13 bits), LSB = 8mV
        // The value is signed but bus voltage is typically positive.
        let val = (raw >> 3) as f32 * 0.008;
        Ok(val)
    }

    /// Read shunt voltage in Volts (V)
    pub fn read_shunt_voltage(i2c: &mut I2cDriver<'_>, channel: usize) -> Result<f32> {
        let reg = match channel {
            1 => Self::CH1_SHUNT_V,
            2 => Self::CH2_SHUNT_V,
            3 => Self::CH3_SHUNT_V,
            _ => return Err(anyhow::anyhow!("Invalid INA3221 channel (must be 1-3)")),
        };
        let raw = Self::read_reg(i2c, reg)?;
        
        // INA3221 Shunt Voltage: bits 14..3 (13 bits), signed, LSB = 40uV
        let val = (raw >> 3) as f32 * 0.00004;
        Ok(val)
    }

    /// Calculate current in Amperes (A) given a shunt resistor in Ohms
    pub fn read_current(i2c: &mut I2cDriver<'_>, channel: usize, shunt_resistor_ohms: f32) -> Result<f32> {
        let shunt_v = Self::read_shunt_voltage(i2c, channel)?;
        Ok(shunt_v / shunt_resistor_ohms)
    }
}
