use crate::Division;

#[derive(PartialEq)]
pub enum CommandSource {
    External,
    Local,
}

pub enum Command {
    KeyUp(Division, u8),
    KeyDown(Division, u8),
    StopOff(Division, u8),
    StopOn(Division, u8),
    StopToggle(Division, u8),
    RecallPreset(u8),
    GeneralCancel(),
    EnableSave(bool),
    Expression(Division, u8),
    Crescendo(u8),
}
