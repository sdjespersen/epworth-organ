use usbd_midi::Message;
use usbd_midi::message::channel::Channel;

use crate::Division;

use embassy_rp::{peripherals::USB, usb::Driver};
use embassy_usb::class::midi::Sender;

/// USB MIDI interface for the Epworth organ.
use epworth_organ::command::Command;
use epworth_organ::event::Event;

fn div_cc_bufs(stop_state: u64) -> [[u8; 60]; 4] {
    let mut bufs = [[0u8; 60]; 4];
    for div in 0..4 {
        for i in 0..15 {
            let shift = 16 * div + i;
            let midi_val = if stop_state >> shift & 1 == 1 { 127 } else { 0 };
            let start = 4 * i;
            bufs[div][start] = 0x0B;
            bufs[div][start + 1] = div as u8 | 0xB0;
            bufs[div][start + 2] = 102 + i as u8;
            bufs[div][start + 3] = midi_val;
        }
    }
    bufs
}

pub async fn write_event_to_usb<'d>(event: Event, sender: &mut Sender<'d, Driver<'d, USB>>) {
    // Certain events require quite different handling.
    if let Event::PresetRecalled(idx, stop_state) = event {
        let _ = sender.write_packet(&[0x0C, 0xC5, idx, 0x00]).await;
        for buf in div_cc_bufs(stop_state) {
            let _ = sender.write_packet(&buf).await;
        }
    }
    if let Event::GeneralCancel() = event {
        // GC NRPN: Channel 5 (not tied to a particular division), NRPN 0, value 127 to indicate pushed
        let gc_nrpn = [
            0x0B, 0xB4, 0x63, 0x00, 0x0B, 0xB4, 0x62, 0x01, 0x0B, 0xB4, 0x06, 0x7F,
        ];
        let _ = sender.write_packet(&gc_nrpn).await;
        // Send MIDI events to turn all stops off.
        for buf in div_cc_bufs(0) {
            let _ = sender.write_packet(&buf).await;
        }
    }
    // The rest of the events map to a single 4-byte packet and have uniform handling.
    let some_packet = match event {
        Event::NoteOff(div, note) => Some([0x08, div as u8 | 0x80, note, 0x80]),
        Event::NoteOn(div, note) => Some([0x08, div as u8 | 0x90, note, 0x80]),
        Event::StopOff(div, idx) => Some([0x0B, div as u8 | 0xB0, 102 + idx, 0x00]),
        Event::StopOn(div, idx) => Some([0x0B, div as u8 | 0xB0, 102 + idx, 0x7F]),
        Event::Expression(div, value) => Some([0x0B, div as u8 | 0xB0, 0x0B, value]),
        Event::Crescendo(value) => Some([0x0B, 0xB4, 0x0B, value]),
        Event::PresetRecalled(_, _) => None, // already handled above
        Event::GeneralCancel() => None,      // already handled above
    };
    if let Some(packet) = some_packet {
        let _ = sender.write_packet(&packet).await;
    }
}

fn div_from_channel(channel: Channel) -> Option<Division> {
    match channel {
        Channel::Channel1 => Some(Division::Swell),
        Channel::Channel2 => Some(Division::Great),
        Channel::Channel3 => Some(Division::Choir),
        Channel::Channel4 => Some(Division::Pedal),
        _ => None,
    }
}

pub fn midi_message_to_command(message: Message) -> Option<Command> {
    match message {
        Message::NoteOff(channel, note, _) => {
            if let Some(div) = div_from_channel(channel) {
                Some(Command::NoteOff(div, note.into()))
            } else {
                None
            }
        }
        Message::NoteOn(channel, note, _) => {
            if let Some(div) = div_from_channel(channel) {
                Some(Command::NoteOn(div, note.into()))
            } else {
                None
            }
        }
        Message::ControlChange(channel, control_fn, value) => {
            // between 102-117, stop changes
            let controller: u8 = control_fn.0.into();
            let some_div = div_from_channel(channel);
            if 102 <= controller
                && controller <= 117
                && let Some(div) = some_div
            {
                if u8::from(value) < 64 {
                    Some(Command::StopOff(div, controller - 102))
                } else {
                    Some(Command::StopOn(div, controller - 102))
                }
            } else if controller == 11 {
                if channel == Channel::Channel5 {
                    Some(Command::Crescendo(value.into()))
                } else if let Some(div) = some_div {
                    Some(Command::Expression(div, value.into()))
                } else {
                    None
                }
            } else {
                None
            }
        }
        Message::Reset => Some(Command::GeneralCancel()),
        _ => None,
    }
}
