#![no_std]
pub mod command;
pub mod debouncer;
pub mod event;
pub mod midi;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
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
