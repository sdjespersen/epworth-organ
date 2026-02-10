/// USB MIDI interface for the Epworth organ.
use crate::event::Event;

pub struct MidiPacketStream {
    stop_state: u64,
    counter: u8,
}

impl Iterator for MidiPacketStream {
    type Item = [u8; 4];

    fn next(&mut self) -> Option<Self::Item> {
        if self.counter < 60 {
            // Then, all the stop tab changes
            let div = self.counter / 15;
            let idx = self.counter % 15 + 1;
            let shift = div * 16 + idx;
            let val: u8 = if self.stop_state >> shift & 1 == 1 {
                127
            } else {
                0
            };
            self.counter += 1;
            Some([0x0B, div | 0xB0, 117 - idx, val])
        } else {
            None
        }
    }

    // TODO: Consider implementing size_hint
}

impl MidiPacketStream {
    pub fn for_stop_state(stop_state: u64) -> Self {
        Self {
            stop_state,
            counter: 0,
        }
    }
}

impl Event {
    pub fn to_usb_midi_packet(self) -> Option<[u8; 4]> {
        match self {
            Event::StopOn(div, idx) => Some([0x0B, div as u8 | 0xB0, 117 - idx, 0xFF]),
            Event::StopOff(div, idx) => Some([0x0B, div as u8 | 0xB0, 117 - idx, 0x00]),
            Event::PresetRecalled(idx) => Some([0x0C, 0xC4, idx, 0x00]),
            _ => None,
        }
    }
}
