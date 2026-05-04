// ============================================================
//   AMR 2.0 — SSD1306 OLED Driver (I2C)
//   Minimal init to verify display is present
// ============================================================

use esp_idf_hal::i2c::I2cDriver;
use anyhow::Result;

const SSD1306_ADDR: u8 = 0x3C;

pub struct Oled;

impl Oled {
    /// Initialize SSD1306 display
    pub fn init(i2c: &mut I2cDriver<'_>) -> Result<()> {
        // SSD1306 init sequence (minimal)
        let init_cmds: &[u8] = &[
            0x00,  // Command stream
            0xAE,  // Display OFF
            0xD5, 0x80, // Set display clock
            0xA8, 0x3F, // Set multiplex ratio (64 lines)
            0xD3, 0x00, // Set display offset
            0x40,       // Set start line
            0x8D, 0x14, // Charge pump ON
            0x20, 0x00, // Horizontal addressing mode
            0xA1,       // Segment re-map
            0xC8,       // COM output scan direction
            0xDA, 0x12, // COM pins
            0x81, 0xCF, // Contrast
            0xD9, 0xF1, // Pre-charge period
            0xDB, 0x40, // VCOMH deselect
            0xA4,       // Display from RAM
            0xA6,       // Normal display (not inverted)
            0xAF,       // Display ON
        ];

        i2c.write(SSD1306_ADDR, init_cmds, 100)?;
        log::info!("[OLED] SSD1306 initialized");
        Ok(())
    }
}
