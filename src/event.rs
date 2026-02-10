use crate::Division;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum Event {
    NoteOff(Division, u8),
    NoteOn(Division, u8),
    StopOff(Division, u8),
    StopOn(Division, u8),
    PresetRecalled(u8),
    GeneralCancel(),
    Expression(Division, u8),
    Crescendo(u8),
}

impl Event {
    /// Serializes the event to 2 bytes.
    pub fn serialize(self) -> [u8; 2] {
        match self {
            Self::NoteOff(div, value) => [0x00 | div as u8, value],
            Self::NoteOn(div, value) => [0x10 | div as u8, value],
            Self::StopOff(div, value) => [0x20 | div as u8, value],
            Self::StopOn(div, value) => [0x30 | div as u8, value],
            Self::PresetRecalled(value) => [0x40, value],
            Self::GeneralCancel() => [0x50, 0x00],
            Self::Expression(div, value) => [0x60 | div as u8, value],
            Self::Crescendo(value) => [0x70, value],
        }
    }

    pub fn parse(buf: &[u8]) -> Option<Self> {
        let ev_type = buf[0] & 0xF0;
        let div = Division::from(buf[0] & 0x0F);
        match ev_type {
            0x00 => Some(Self::NoteOff(div, buf[1])),
            0x10 => Some(Self::NoteOn(div, buf[1])),
            0x20 => Some(Self::StopOff(div, buf[1])),
            0x30 => Some(Self::StopOn(div, buf[1])),
            0x40 => Some(Self::PresetRecalled(buf[1])),
            0x50 => Some(Self::GeneralCancel()),
            0x60 => Some(Self::Expression(div, buf[1])),
            0x70 => Some(Self::Crescendo(buf[1])),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_serialize_parse() {
        let ssc = Event::StopStateChanged(0xF0E1D2C3B4A59687);
        let mut buf = [0u8; 10];
        let written = ssc.serialize(&mut buf);
        let actual = Event::parse(&buf[..written]).unwrap();
        assert_eq!(actual, ssc);
    }
}
