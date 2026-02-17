use crate::stops::Stop;
use serde::{Deserialize, Serialize};

use crate::Division;

#[derive(PartialEq)]
pub enum CommandSource {
    External,
    Local,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash, Serialize, Deserialize)]
pub enum Command {
    NoteOff(Division, u8),
    NoteOn(Division, u8),
    StopOff(Stop),
    StopOn(Stop),
    StopToggle(Stop),
    RecallPreset(u8),
    GeneralCancel(),
    EnableSave(bool),
    Expression(Division, u8),
    Crescendo(u8),
}
