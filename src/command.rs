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

impl Command {
    pub fn parse(buf: &[u8]) -> Option<Self> {
        let cmd_type = buf[0] & 0xF0;
        let div = Division::from(buf[0] & 0x0F);
        match cmd_type {
            0x00 => Some(Self::KeyUp(div, buf[1])),
            0x10 => Some(Self::KeyDown(div, buf[1])),
            0x20 => Some(Self::StopOff(div, buf[1])),
            0x30 => Some(Self::StopOn(div, buf[1])),
            0x40 => Some(Self::StopToggle(div, buf[1])),
            0x50 => Some(Self::RecallPreset(buf[1])),
            0x60 => Some(Self::GeneralCancel()),
            0x70 => Some(Self::EnableSave(buf[1] > 63)),
            0x80 => Some(Self::Expression(div, buf[1])),
            0x90 => Some(Self::Crescendo(buf[1])),
            _ => None,
        }
    }
}
