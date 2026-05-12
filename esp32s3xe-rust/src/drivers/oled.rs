// ============================================================
//   AMR 2.0 — SSD1306 OLED Driver (I2C)
//   Implemented using ssd1306 and embedded-graphics
// ============================================================

use esp_idf_hal::i2c::I2cDriver;
use anyhow::Result;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use core::fmt::Write;
use heapless::String;

pub struct Oled;

impl Oled {
    /// Initialize SSD1306 display
    pub fn init(i2c: &mut I2cDriver<'_>) -> Result<()> {
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().map_err(|_| anyhow::anyhow!("OLED init failed"))?;
        display.clear(BinaryColor::Off).map_err(|_| anyhow::anyhow!("OLED clear failed"))?;
        display.flush().map_err(|_| anyhow::anyhow!("OLED flush failed"))?;
        log::info!("[OLED] SSD1306 initialized");
        Ok(())
    }

    /// Draw status to SSD1306 display
    pub fn draw_status(i2c: &mut I2cDriver<'_>, ip: &str, batt: u8, nav_state: &str, imu_ok: bool, speed: f32, heading: f32) -> Result<()> {
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        
        display.clear(BinaryColor::Off).map_err(|_| anyhow::anyhow!("OLED clear failed"))?;
        
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();
            
        let mut buf = String::<64>::new();
        
        write!(&mut buf, "AMR {} ON", ip).unwrap();
        Text::with_baseline(&buf, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display).map_err(|_| anyhow::anyhow!("OLED draw failed"))?;
            
        buf.clear();
        write!(&mut buf, "Bat:{}% IMU:{}", batt, if imu_ok { "OK" } else { "--" }).unwrap();
        Text::with_baseline(&buf, Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut display).map_err(|_| anyhow::anyhow!("OLED draw failed"))?;

        buf.clear();
        write!(&mut buf, "NAV:{}", nav_state).unwrap();
        Text::with_baseline(&buf, Point::new(0, 32), text_style, Baseline::Top)
            .draw(&mut display).map_err(|_| anyhow::anyhow!("OLED draw failed"))?;

        buf.clear();
        write!(&mut buf, "Spd:{:.2}m/s H:{:.0}", speed, heading).unwrap();
        Text::with_baseline(&buf, Point::new(0, 48), text_style, Baseline::Top)
            .draw(&mut display).map_err(|_| anyhow::anyhow!("OLED draw failed"))?;

        display.flush().map_err(|_| anyhow::anyhow!("OLED flush failed"))?;
        Ok(())
    }
    /// Draw QR code to SSD1306 display
    pub fn draw_qr_url(i2c: &mut I2cDriver<'_>, url: &str) -> Result<()> {
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        
        display.clear(BinaryColor::Off).map_err(|_| anyhow::anyhow!("OLED clear failed"))?;
        
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();
            
        // Text instructions on the left
        Text::with_baseline("AMR 2.0", Point::new(0, 5), text_style, Baseline::Top)
            .draw(&mut display).map_err(|_| anyhow::anyhow!("OLED draw failed"))?;
        Text::with_baseline("SETUP", Point::new(0, 18), text_style, Baseline::Top)
            .draw(&mut display).map_err(|_| anyhow::anyhow!("OLED draw failed"))?;
        Text::with_baseline("Scan QR ->", Point::new(0, 35), text_style, Baseline::Top)
            .draw(&mut display).map_err(|_| anyhow::anyhow!("OLED draw failed"))?;
        
        // Generate QR code
        if let Ok(qr) = qrcodegen::QrCode::encode_text(url, qrcodegen::QrCodeEcc::Low) {
            let size = qr.size();
            let module_size = if size <= 25 { 2 } else { 1 }; // Scale 2x for small QR, 1x for large
            
            // Center QR code on the right side of the screen (x offset ~64, y offset centered)
            let qr_pixel_size = size * module_size;
            let offset_x = 128 - qr_pixel_size - 5; // 5 pixels padding from right
            let offset_y = (64 - qr_pixel_size) / 2;
            
            // Draw a white background for the QR code for better scanning contrast
            for y in -2..qr_pixel_size + 2 {
                for x in -2..qr_pixel_size + 2 {
                    display.set_pixel((offset_x + x) as u32, (offset_y + y) as u32, false);
                }
            }

            for y in 0..size {
                for x in 0..size {
                    if qr.get_module(x, y) {
                        for dy in 0..module_size {
                            for dx in 0..module_size {
                                let px = offset_x + x * module_size + dx;
                                let py = offset_y + y * module_size + dy;
                                display.set_pixel(px as u32, py as u32, true);
                            }
                        }
                    }
                }
            }
        }

        display.flush().map_err(|_| anyhow::anyhow!("OLED flush failed"))?;
        Ok(())
    }
}
