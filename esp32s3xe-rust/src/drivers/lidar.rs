// ============================================================
//   AMR 2.0 — RPLidar A1M8 UART Driver
//   Custom Rust parser for RPLIDAR protocol
// ============================================================

use esp_idf_hal::uart::UartDriver;
use anyhow::Result;

/// RPLIDAR command bytes
const SYNC_BYTE: u8 = 0xA5;
const CMD_GET_INFO: u8 = 0x50;
const CMD_STOP: u8 = 0x25;
const CMD_SCAN: u8 = 0x20;
const CMD_FORCE_SCAN: u8 = 0x21;

/// A single LiDAR scan point
#[derive(Clone, Copy, Default)]
pub struct ScanPoint {
    pub angle: f32,     // degrees (0-360)
    pub distance: f32,  // mm
    pub quality: u8,
    pub start_flag: bool,
}

/// RPLidar A1M8 driver
pub struct RpLidar;

impl RpLidar {
    /// Send stop command to halt any ongoing scan
    pub fn stop(uart: &mut UartDriver<'_>) -> Result<()> {
        uart.write(&[SYNC_BYTE, CMD_STOP])?;
        std::thread::sleep(std::time::Duration::from_millis(100));
        // Flush RX buffer
        let mut trash = [0u8; 64];
        while uart.read(&mut trash, 10).unwrap_or(0) > 0 {}
        Ok(())
    }

    /// Send GET_INFO and verify device responds
    pub fn get_device_info(uart: &mut UartDriver<'_>) -> Result<bool> {
        uart.write(&[SYNC_BYTE, CMD_GET_INFO])?;
        std::thread::sleep(std::time::Duration::from_millis(200));

        let mut resp = [0u8; 27]; // Response descriptor (7) + data (20)
        let n = uart.read(&mut resp, 500).unwrap_or(0);
        
        if n >= 7 && resp[0] == 0xA5 && resp[1] == 0x5A {
            log::info!("[LIDAR] Device info OK ({} bytes received)", n);
            if n >= 27 {
                log::info!("[LIDAR] Model: {} FW: {}.{} HW: {}",
                    resp[7], resp[9], resp[8], resp[10]);
            }
            Ok(true)
        } else {
            log::warn!("[LIDAR] No valid response ({} bytes, first: 0x{:02X})", n, 
                if n > 0 { resp[0] } else { 0 });
            Ok(false)
        }
    }

    /// Start standard scan mode
    pub fn start_scan(uart: &mut UartDriver<'_>) -> Result<()> {
        uart.write(&[SYNC_BYTE, CMD_SCAN])?;
        // Wait for response descriptor (7 bytes)
        std::thread::sleep(std::time::Duration::from_millis(100));
        let mut desc = [0u8; 7];
        let _ = uart.read(&mut desc, 200);
        log::info!("[LIDAR] Scan started");
        Ok(())
    }

    /// Read one scan point from UART (5-byte cabin format)
    /// Returns None if no data available or parse error
    pub fn read_point(uart: &mut UartDriver<'_>) -> Option<ScanPoint> {
        let mut buf = [0u8; 5];
        if uart.read(&mut buf, 20).unwrap_or(0) < 5 {
            return None;
        }

        let quality = buf[0] >> 2;
        let start_flag = (buf[0] & 0x01) != 0;
        let check_bit = (buf[1] & 0x01) != 0;

        if !check_bit {
            return None; // Invalid packet
        }

        let angle_raw = ((buf[2] as u16) << 7) | ((buf[1] as u16) >> 1);
        let angle = angle_raw as f32 / 64.0;

        let distance_raw = ((buf[4] as u16) << 8) | (buf[3] as u16);
        let distance = distance_raw as f32 / 4.0;

        Some(ScanPoint {
            angle,
            distance,
            quality,
            start_flag,
        })
    }
}
