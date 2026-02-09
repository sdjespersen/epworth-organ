use midly::MidiMessage;
use midly::live::LiveEvent;
use midly::num::{u4, u7};

use crate::event::Event;

fn _stop_tab_midi_cc_message(div: usize, i: u8, div_stop_state: u16) -> LiveEvent<'static> {
    let mut midi_val: u7 = 0.into();
    if div_stop_state >> i & 1 == 1 {
        midi_val = u7::max_value();
    }
    LiveEvent::Midi {
        channel: u4::from(div as u8),
        message: MidiMessage::Controller {
            controller: (117 - i).into(),
            value: midi_val,
        },
    }
}

// Logic for preset recall
// let program_change = LiveEvent::Midi {
//     channel: 5.into(),
//     message: MidiMessage::ProgramChange {
//         program: idx.into(),
//     },
// };
// for div in 0..4 {
//     let div_state = ((stop_state >> (16 * div)) & 0xFFFF) as u16;
//     // Now we need to emit 15 MIDI CCs...skipping i = 0 (LSB) which doesn't correspond to physical hardware
//     for i in 1..=15 {
//         let msg = stop_tab_midi_cc_message(div, i, div_state);
//         let _ = USB_EVENT_BUS.try_send(msg); // yes, yes, this could return error
//     }
// }

impl Event {
    pub fn to_usb_midi(self) -> [u8; 4] {
        match self {
            Event::StopOn(div, idx) => {
                [0x0B, div as u8 | 0xB0, 117 - idx, 0xFF]
            }
            Event::StopOff(div, idx) => {
                [0x0B, div as u8 | 0xB0, 117 - idx, 0x00]
            }
            _ => [0x00; 4],
        }
    }
}
