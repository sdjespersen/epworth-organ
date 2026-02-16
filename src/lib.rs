#![no_std]
pub mod command;
pub mod debouncer;
pub mod decoder_state;
pub mod encoder;
pub mod event;

use defmt_rtt as _;
use serde::{Deserialize, Serialize}; // global logger

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash, Serialize, Deserialize)]
pub enum Division {
    Swell,
    Great,
    Choir,
    Pedal,
}

impl Into<u8> for Division {
    fn into(self) -> u8 {
        match self {
            Division::Swell => 0,
            Division::Great => 1,
            Division::Choir => 2,
            Division::Pedal => 3,
        }
    }
}

impl From<u8> for Division {
    fn from(val: u8) -> Self {
        match val {
            0 => Division::Swell,
            1 => Division::Great,
            2 => Division::Choir,
            3 => Division::Pedal,
            _ => panic!("Invalid value found for division!"),
        }
    }
}
