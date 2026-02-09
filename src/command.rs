use crate::Division;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum CommandType {
    KeyUp(Division, u8),
    KeyDown(Division, u8),
    StopOff(Division, u8),
    StopOn(Division, u8),
    StopToggle(Division, u8),
    GeneralCancel(),
    RecallPreset(u8),
    EnableSave(bool),
    Expression(Division, u8),
    Crescendo(u8),
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub struct Command {
    pub cmd: CommandType,
    pub external: bool,
}

impl Command {
    pub fn local(cmd: CommandType) -> Self {
        Self {
            cmd: cmd,
            external: false,
        }
    }
}
