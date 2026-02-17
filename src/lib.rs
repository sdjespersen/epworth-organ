#![cfg_attr(not(test), no_std)]
pub mod command;
pub mod debouncer;
pub mod decoder;
pub mod encoder;
pub mod event;
pub mod stops;

use defmt_rtt as _;
use serde::{Deserialize, Serialize}; // global logger

#[derive(Clone, Copy, Serialize, Deserialize)]
pub enum Division {
    Swell,
    Great,
    Choir,
    Pedal,
}

impl Division {
    const fn to_u8(self) -> u8 {
        match self {
            Division::Swell => 0,
            Division::Great => 1,
            Division::Choir => 2,
            Division::Pedal => 3,
        }
    }
}
