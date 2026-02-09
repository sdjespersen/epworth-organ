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
    pub fn serialize(self, buf: &mut [u8]) -> usize {
        // Can we enforce the buffer size?
        match self {
            Event::NoteOff(div, value) => {
                buf[0] = 0x00 | div as u8;
                buf[1] = value;
            }
            Event::NoteOn(div, value) => {
                buf[0] = 0x10 | div as u8;
                buf[1] = value;
            }
            Event::StopOff(div, value) => {
                buf[0] = 0x20 | div as u8;
                buf[1] = value;
            }
            Event::StopOn(div, value) => {
                buf[0] = 0x30 | div as u8;
                buf[1] = value;
            }
            Event::StopStateChanged(value) => {
                buf[0] = 0x40;
                for i in 0..8 {
                    buf[i+1] = ((value >> 8 * i) & 0xFF) as u8;
                }
                return 9
            }
            Event::PresetRecalled(value) => {
                buf[0] = 0x50;
                buf[1] = value;
            }
            Event::GeneralCancel() => {
                buf[0] = 0x60;
            }
            Event::Expression(div, value) => {
                buf[0] = 0x70 | div as u8;
                buf[1] = value;
            }
            Event::Crescendo(value) => {
                buf[0] = 0x80;
                buf[1] = value;
            }
        };
        2
    }

    pub fn parse(buf: &[u8]) -> Self {
        // Enforce buffer size? All sorts of scary slice access here.
        let ev_type = buf[0] & 0xF0;
        let div = Division::from(buf[0] & 0x0F);
        match ev_type {
            0x00 => Event::NoteOff(div, buf[1]),
            0x10 => Event::NoteOn(div, buf[1]),
            0x20 => Event::StopOff(div, buf[1]),
            0x30 => Event::StopOn(div, buf[1]),
            0x40 => {
                let mut val = 0u64;
                for i in 0..7 {
                    val |= buf[i+1] as u64;
                    val <<= 8;
                }
                val |= buf[8] as u64;
                Event::StopStateChanged(val)
            }
            0x50 => Event::PresetRecalled(buf[1]),
            0x60 => Event::GeneralCancel(),
            0x70 => Event::Expression(div, buf[1]),
            0x80 => Event::Crescendo(buf[1]),
            _ => panic!("Invalid event type!")
        }
    }
}
