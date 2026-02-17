use serde::{Deserialize, Serialize};

use crate::Division;
use crate::stops::Stop;

#[derive(Clone, Copy, Serialize, Deserialize)]
pub enum Event {
    NoteOff(Division, u8),
    NoteOn(Division, u8),
    StopOff(Stop),
    StopOn(Stop),
    PresetRecalled(u8, u64),
    GeneralCancel(),
    Expression(Division, u8),
    Crescendo(u8),
}
