use crate::Division;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum Event {
    NoteOff(Division, u8),
    NoteOn(Division, u8),
    StopOff(Division, u8),
    StopOn(Division, u8),
    StopStateChanged(u64),
    PresetRecalled(u8),
    GeneralCancel(),
    Expression(Division, u8),
    Crescendo(u8),
}

impl Event {
    /// Serializes the event to the provided buffer and returns the number of bytes written.
    pub fn serialize(self, buf: &mut [u8]) -> usize {
        // Can we enforce the buffer size?
        match self {
            Self::NoteOff(div, value) => {
                buf[0] = 0x00 | div as u8;
                buf[1] = value;
            }
            Self::NoteOn(div, value) => {
                buf[0] = 0x10 | div as u8;
                buf[1] = value;
            }
            Self::StopOff(div, value) => {
                buf[0] = 0x20 | div as u8;
                buf[1] = value;
            }
            Self::StopOn(div, value) => {
                buf[0] = 0x30 | div as u8;
                buf[1] = value;
            }
            Self::StopStateChanged(value) => {
                buf[0] = 0x40;
                for i in 0..8 {
                    buf[i + 1] = ((value >> 8 * i) & 0xFF) as u8;
                }
                return 9;
            }
            Self::PresetRecalled(value) => {
                buf[0] = 0x50;
                buf[1] = value;
            }
            Self::GeneralCancel() => {
                buf[0] = 0x60;
                return 1;
            }
            Self::Expression(div, value) => {
                buf[0] = 0x70 | div as u8;
                buf[1] = value;
            }
            Self::Crescendo(value) => {
                buf[0] = 0x80;
                buf[1] = value;
            }
        };
        // Events take up exactly 2 bytes, unless otherwise specified.
        2
    }

    pub fn parse(buf: &[u8]) -> Option<Self> {
        let ev_type = buf[0] & 0xF0;
        let div = Division::from(buf[0] & 0x0F);
        match ev_type {
            0x00 => Some(Self::NoteOff(div, buf[1])),
            0x10 => Some(Self::NoteOn(div, buf[1])),
            0x20 => Some(Self::StopOff(div, buf[1])),
            0x30 => Some(Self::StopOn(div, buf[1])),
            0x40 => {
                let mut value = 0u64;
                for i in 1..=7 {
                    value |= buf[i] as u64;
                    value <<= 8;
                }
                value |= buf[8] as u64;
                Some(Self::StopStateChanged(value))
            }
            0x50 => Some(Self::PresetRecalled(buf[1])),
            0x60 => Some(Self::GeneralCancel()),
            0x70 => Some(Self::Expression(div, buf[1])),
            0x80 => Some(Self::Crescendo(buf[1])),
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
