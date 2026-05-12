// ============================================================
//   AMR 2.0 — Quadrature Encoder Driver
// ============================================================

use esp_idf_hal::gpio::InputPin;
use esp_idf_hal::pcnt::{
    config::{ChannelConfig, ChannelEdgeAction, ChannelLevelAction},
    PcntUnitDriver,
};
use anyhow::Result;

pub struct Encoder<'d> {
    unit: PcntUnitDriver<'d>,
    inverted: bool,
}

impl<'d> Encoder<'d> {
    pub fn new(
        mut unit: PcntUnitDriver<'d>,
        pin_a: impl InputPin + 'd,
        pin_b: impl InputPin + 'd,
        inverted: bool,
    ) -> Result<Self> {
        let ch_a = unit.add_channel(Some(pin_a), Some(pin_b), &ChannelConfig::default())?;
        ch_a.set_edge_action(ChannelEdgeAction::Decrease, ChannelEdgeAction::Increase)?;
        ch_a.set_level_action(ChannelLevelAction::Keep, ChannelLevelAction::Inverse)?;

        unit.enable()?;
        unit.clear_count()?;
        unit.start()?;

        Ok(Self { unit, inverted })
    }

    pub fn get_count(&self) -> Result<i32> {
        let count = self.unit.get_count()?;
        Ok(if self.inverted { -count } else { count })
    }
}
