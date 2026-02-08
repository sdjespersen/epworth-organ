#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum Division {
    Swell,
    Great,
    Choir,
    Pedal,
}

impl Into<usize> for Division {
    fn into(self) -> usize {
        match self {
            Division::Swell => 0,
            Division::Great => 1,
            Division::Choir => 2,
            Division::Pedal => 3,
        }
    }
}

impl Into<Division> for usize {
    fn into(self) -> Division {
        match self {
            0 => Division::Swell,
            1 => Division::Great,
            2 => Division::Choir,
            3 => Division::Pedal,
            // TODO: Define a proper type for this so we know it's always valid.
            _ => panic!("Invalid division index: {}", self),
        }
    }
}

impl Into<u8> for Division {
    fn into(self) -> u8 {
        let v: usize = self.into();
        v as u8
    }
}

impl Into<Division> for u8 {
    fn into(self) -> Division {
        let v: usize = self.into();
        v.into()
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum EventSource {
    Local,
    External,
}

// Represents an event on the organ. Most of these map pretty directly to MIDI messages, but some (like `StopToggle` and
// `EnableSave`) do not. The proprietary nature allows us to (a) pass messages that don't have a clear analog in the
// MIDI spec and (b) pack every message we'll ever use into only 2 bytes.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum Event {
    NoteOff(Division, u8),
    NoteOn(Division, u8),
    StopOff(Division, u8),
    StopOn(Division, u8),
    StopToggle(Division, u8),
    GeneralCancel(),
    RecallPreset(u8),
    EnableSave(bool),
    Expression(Division, u8),
    Crescendo(u8),
}

impl Event {
    pub fn serialize(&self, buf: &mut [u8]) {
        match self {
            Event::NoteOff(div, key) => {
                buf[0] = 0x00 | *div as u8;
                buf[1] = *key;
            }
            Event::NoteOn(div, key) => {
                buf[0] = 0x10 | *div as u8;
                buf[1] = *key;
            }
            Event::StopOff(div, idx) => {
                buf[0] = 0x20 | *div as u8;
                buf[1] = *idx;
            }
            Event::StopOn(div, idx) => {
                buf[0] = 0x30 | *div as u8;
                buf[1] = *idx;
            }
            Event::StopToggle(div, idx) => {
                buf[0] = 0x40 | *div as u8;
                buf[1] = *idx;
            }
            Event::GeneralCancel() => {
                buf[0] = 0x50;
                buf[1] = 0x00;
            }
            Event::RecallPreset(idx) => {
                buf[0] = 0x60;
                buf[1] = *idx;
            }
            Event::EnableSave(value) => {
                buf[0] = 0x70;
                buf[1] = if *value { 1 } else { 0 };
            }
            Event::Expression(div, value) => {
                buf[0] = 0x80 | *div as u8;
                buf[1] = *value;
            }
            Event::Crescendo(value) => {
                buf[0] = 0x90;
                buf[1] = *value;
            }
        }
    }

    pub fn parse(status: u8, value: u8) -> Event {
        let div = (status & 0x0F).into();
        match status & 0xF0 {
            0x00 => Event::NoteOff(div, value),
            0x10 => Event::NoteOn(div, value),
            0x20 => Event::StopOff(div, value),
            0x30 => Event::StopOn(div, value),
            0x40 => Event::StopToggle(div, value),
            0x50 => Event::GeneralCancel(),
            0x60 => Event::RecallPreset(value),
            0x70 => Event::EnableSave(value == 1),
            0x80 => Event::Expression(div, value),
            0x90 => Event::Crescendo(value),
            _ => panic!("Invalid event status: {:02X}", status),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub struct EventIn(pub EventSource, pub Event);
