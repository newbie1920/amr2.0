// ============================================================
//   AMR 2.0 — MPU6050 IMU Driver (I2C)
//   Ported from imu_sensor.h/cpp
// ============================================================

use esp_idf_hal::i2c::I2cDriver;
use anyhow::Result;

const MPU6050_ADDR: u8 = 0x68;
const WHO_AM_I_REG: u8 = 0x75;
const PWR_MGMT_1: u8 = 0x6B;
const GYRO_ZOUT_H: u8 = 0x47;

pub struct Mpu6050;

impl Mpu6050 {
    /// Initialize MPU6050: wake up, verify WHO_AM_I
    pub fn init(i2c: &mut I2cDriver<'_>) -> Result<()> {
        // Read WHO_AM_I to verify device
        let mut buf = [0u8; 1];
        i2c.write_read(MPU6050_ADDR, &[WHO_AM_I_REG], &mut buf, 100)?;
        
        if buf[0] != 0x68 && buf[0] != 0x98 {
            anyhow::bail!("MPU6050 WHO_AM_I mismatch: got 0x{:02X}", buf[0]);
        }

        // Wake up (clear sleep bit)
        i2c.write(MPU6050_ADDR, &[PWR_MGMT_1, 0x00], 100)?;
        
        log::info!("[IMU] MPU6050 initialized (WHO_AM_I=0x{:02X})", buf[0]);
        Ok(())
    }

    /// Read gyroscope Z-axis angular velocity (rad/s)
    pub fn read_gyro_z(i2c: &mut I2cDriver<'_>) -> Result<f32> {
        let mut buf = [0u8; 2];
        i2c.write_read(MPU6050_ADDR, &[GYRO_ZOUT_H], &mut buf, 100)?;
        
        let raw = i16::from_be_bytes([buf[0], buf[1]]);
        // Default FS_SEL=0 → 131 LSB/°/s → convert to rad/s
        let dps = raw as f32 / 131.0;
        Ok(dps * std::f32::consts::PI / 180.0)
    }
}
