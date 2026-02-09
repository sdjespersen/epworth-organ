/// USB MIDI interface for the Epworth organ.
use core::iter;

use crate::event::Event;

enum MidiPacketStreamInner {
    Once(iter::Once<[u8; 4]>),
    ForStopStateChanged { stop_state: u64, counter: u8 },
}

impl Iterator for MidiPacketStreamInner {
    type Item = [u8; 4];

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Once(inner) => inner.next(),
            Self::ForStopStateChanged {
                stop_state,
                counter,
            } => {
                if *counter < 60 {
                    // Then, all the stop tab changes
                    let div = *counter / 15;
                    let idx = *counter % 15 + 1;
                    let shift = div * 16 + idx;
                    let val: u8 = if *stop_state >> shift & 1 == 1 {
                        127
                    } else {
                        0
                    };
                    *counter += 1;
                    Some([0x0B, div | 0xB0, 117 - idx, val])
                } else {
                    None
                }
            }
        }
    }

    // TODO: Consider implementing size_hint
}

// A custom type that implements Iterator<[u8; 4]>. Useful because certain events result in multiple MIDI packets.
pub struct MidiPacketStream(MidiPacketStreamInner);

impl MidiPacketStream {
    pub fn of(packet: [u8; 4]) -> Self {
        Self(MidiPacketStreamInner::Once(iter::once(packet)))
    }

    pub fn for_stop_state_changed(stop_state: u64) -> Self {
        Self(MidiPacketStreamInner::ForStopStateChanged {
            stop_state,
            counter: 0,
        })
    }
}

impl Iterator for MidiPacketStream {
    type Item = [u8; 4];

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl Event {
    pub fn to_usb_midi_packets(self) -> MidiPacketStream {
        match self {
            Event::StopOn(div, idx) => {
                MidiPacketStream::of([0x0B, div as u8 | 0xB0, 117 - idx, 0xFF])
            }
            Event::StopOff(div, idx) => {
                MidiPacketStream::of([0x0B, div as u8 | 0xB0, 117 - idx, 0x00])
            }
            Event::PresetRecalled(idx) => MidiPacketStream::of([0x0C, 0xC4, idx, 0x00]),
            Event::StopStateChanged(stop_state) => {
                MidiPacketStream::for_stop_state_changed(stop_state)
            }
            _ => MidiPacketStream::of([0x00; 4]),
        }
    }
}
