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

const fn div_to_u8(div: Division) -> u8 {
    match div {
        Division::Swell => 0,
        Division::Great => 1,
        Division::Choir => 2,
        Division::Pedal => 3,
    }
}

impl Into<u8> for Division {
    fn into(self) -> u8 {
        div_to_u8(self)
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
