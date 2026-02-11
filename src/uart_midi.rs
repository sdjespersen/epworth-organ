use embassy_rp::uart;
use embedded_io_async::Write;
use midly::MidiMessage;
use midly::io::Cursor;
use midly::live::{LiveEvent, SystemCommon, SystemRealtime};
use midly::num::{u4, u7};

use crate::{Division, command::Command, event::Event};

fn into_u4(div: Division) -> u4 {
    match div {
        Division::Swell => u4::from(0),
        Division::Great => u4::from(1),
        Division::Choir => u4::from(2),
        Division::Pedal => u4::from(3),
    }
}

fn div_from_u4(val: u4) -> Division {
    Division::from(u8::from(val))
}

// TODO: Follow the compiler's advice and "desugar" this async fn into a std fn.
pub trait UartWritable {
    async fn write_to_uart<'d>(self, sender: &mut uart::BufferedUartTx);
}

impl UartWritable for Event {
    async fn write_to_uart<'d>(self: Event, sender: &mut uart::BufferedUartTx) {
        // SysEx constrains data bytes to 0-127 (top bit 0). We have 8 bytes that use the full range 0-255, so
        // we need to expand them to 10 bytes.
        let mut buf: [u7; 11] = [0.into(); 11];
        let midi_event = match self {
            Event::NoteOff(div, note) => Some(LiveEvent::Midi {
                channel: into_u4(div),
                message: MidiMessage::NoteOff {
                    key: note.into(),
                    vel: 64.into(),
                },
            }),
            Event::NoteOn(div, note) => Some(LiveEvent::Midi {
                channel: into_u4(div),
                message: MidiMessage::NoteOn {
                    key: note.into(),
                    vel: 64.into(),
                },
            }),
            Event::StopOff(div, idx) => Some(LiveEvent::Midi {
                channel: into_u4(div),
                message: MidiMessage::Controller {
                    controller: (117 - idx).into(),
                    value: 0.into(),
                },
            }),
            Event::StopOn(div, idx) => Some(LiveEvent::Midi {
                channel: into_u4(div),
                message: MidiMessage::Controller {
                    controller: (117 - idx).into(),
                    value: 127.into(),
                },
            }),
            Event::Expression(div, value) => Some(LiveEvent::Midi {
                channel: into_u4(div),
                message: MidiMessage::Controller {
                    controller: 11.into(),
                    value: value.into(),
                },
            }),
            Event::Crescendo(value) => Some(LiveEvent::Midi {
                channel: 4.into(), // "whole organ", not a specific division
                message: MidiMessage::Controller {
                    controller: 11.into(),
                    value: value.into(),
                },
            }),
            Event::PresetRecalled(idx, stop_state) => {
                buf[0] = idx.into(); // preset number
                for i in 0..10 {
                    buf[i + 1] = ((stop_state >> 7 * i) as u8 & 0x7F).into();
                }
                Some(LiveEvent::Common(SystemCommon::SysEx(&buf)))
            }
            Event::GeneralCancel() => Some(LiveEvent::Realtime(SystemRealtime::Reset)),
        };
        if let Some(e) = midi_event {
            let mut buf = [0u8; 16];
            let mut cursor = Cursor::new(&mut buf);
            let _ = e.write(&mut cursor);
            let _ = sender.write_all(cursor.written()).await;
        }
    }
}

pub fn live_event_to_command(event: LiveEvent) -> Option<Command> {
    match event {
        LiveEvent::Midi { channel, message } => {
            match message {
                MidiMessage::NoteOff { key, vel: _ } => {
                    Some(Command::NoteOff(div_from_u4(channel), key.into()))
                }
                MidiMessage::NoteOn { key, vel: _ } => {
                    Some(Command::NoteOn(div_from_u4(channel), key.into()))
                }
                MidiMessage::Controller { controller, value } => {
                    // between 102-117, stop changes
                    if 102 <= controller && controller <= 117 {
                        if value < 64 {
                            Some(Command::StopOff(
                                div_from_u4(channel),
                                117 - u8::from(controller),
                            ))
                        } else {
                            Some(Command::StopOn(
                                div_from_u4(channel),
                                117 - u8::from(controller),
                            ))
                        }
                    } else if controller == 11 {
                        if channel == 4 {
                            Some(Command::Crescendo(value.into()))
                        } else {
                            Some(Command::Expression(div_from_u4(channel), value.into()))
                        }
                    } else {
                        None
                    }
                    // Some(Command::StopOff(div_from_u4(channel), key.into()))
                }
                _ => None,
            }
        }
        LiveEvent::Realtime(msg) => match msg {
            SystemRealtime::Reset => return Some(Command::GeneralCancel()),
            _ => None,
        },
        LiveEvent::Common(_) => None,
    }
}

// TODO: Maybe this is a smell that we should collapse Command and Event together?
pub fn live_event_to_event(event: LiveEvent) -> Option<Event> {
    match event {
        LiveEvent::Midi { channel, message } => {
            match message {
                MidiMessage::NoteOff { key, vel: _ } => {
                    Some(Event::NoteOff(div_from_u4(channel), key.into()))
                }
                MidiMessage::NoteOn { key, vel: _ } => {
                    Some(Event::NoteOn(div_from_u4(channel), key.into()))
                }
                MidiMessage::Controller { controller, value } => {
                    // between 102-117, stop changes
                    if 102 <= controller && controller <= 117 {
                        if value < 64 {
                            Some(Event::StopOff(
                                div_from_u4(channel),
                                117 - u8::from(controller),
                            ))
                        } else {
                            Some(Event::StopOn(
                                div_from_u4(channel),
                                117 - u8::from(controller),
                            ))
                        }
                    } else if controller == 11 {
                        if channel == 4 {
                            Some(Event::Crescendo(value.into()))
                        } else {
                            Some(Event::Expression(div_from_u4(channel), value.into()))
                        }
                    } else {
                        None
                    }
                    // Some(Event::StopOff(div_from_u4(channel), key.into()))
                }
                _ => None,
            }
        }
        LiveEvent::Realtime(msg) => match msg {
            SystemRealtime::Reset => return Some(Event::GeneralCancel()),
            _ => None,
        },
        LiveEvent::Common(msg) => {
            match msg {
                SystemCommon::SysEx(data) => {
                    // So far the only thing this equates to is a PresetRecall
                    let idx = u8::from(data[0]);
                    let mut stop_state = 0u64;
                    for i in 0..10 {
                        stop_state |= (u8::from(data[i + 1]) as u64) << (7 * i);
                    }
                    Some(Event::PresetRecalled(idx, stop_state))
                }
                _ => None,
            }
        }
    }
}
