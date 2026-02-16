use crate::presets::PresetStore;
use crate::{command::Command, event::Event};

pub struct EncoderState<'d> {
    stop_state: u64,
    awaiting_save: bool,
    preset_store: &'d mut PresetStore<'d>,
}

pub struct CommandResult(pub Option<Event>, pub bool);

impl<'d> EncoderState<'d> {
    pub fn new(preset_store: &'d mut PresetStore<'d>) -> Self {
        Self {
            stop_state: 0u64,
            awaiting_save: false,
            preset_store: preset_store,
        }
    }

    pub fn get_stop_state(&self) -> u64 {
        self.stop_state
    }

    pub async fn command(&mut self, cmd: Command) -> CommandResult {
        // This is the guts of the primary encoder MCU. All commands issued to the organ go through this event loop in
        // order to affect any state on the organ. Some commands pass through essentially untouched to the decoder,
        // while others may result in numerous or even zero outbound events. This first match applies state-related
        // logic to incoming encoder commands (e.g. "toggle" messages converted to "on" or "off" depending on state).
        match cmd {
            Command::NoteOff(div, value) => CommandResult(Some(Event::NoteOff(div, value)), false),
            Command::NoteOn(div, value) => CommandResult(Some(Event::NoteOn(div, value)), false),
            Command::StopOff(div, idx) => {
                self.stop_state &= !(1 << 16 * div as u8 + idx);
                CommandResult(Some(Event::StopOff(div, idx)), true)
            }
            Command::StopOn(div, idx) => {
                self.stop_state |= 1 << 16 * div as u8 + idx;
                CommandResult(Some(Event::StopOn(div, idx)), true)
            }
            Command::StopToggle(div, idx) => {
                let shift = 16 * div as u8 + idx;
                self.stop_state ^= 1 << shift;
                if self.stop_state >> shift & 1 == 1 {
                    CommandResult(Some(Event::StopOn(div, idx)), true)
                } else {
                    CommandResult(Some(Event::StopOff(div, idx)), true)
                }
            }
            Command::GeneralCancel() => {
                self.stop_state = 0;
                CommandResult(Some(Event::GeneralCancel()), true)
            }
            Command::RecallPreset(idx) => {
                if self.awaiting_save {
                    // Saving a preset is internal; it emits no events.
                    self.preset_store.save(idx, &self.stop_state).await;
                    // Safety: Disengage save mode if we've already written one preset.
                    self.awaiting_save = false;
                    CommandResult(None, false)
                } else {
                    self.stop_state = self.preset_store.load(idx).await;
                    CommandResult(Some(Event::PresetRecalled(idx, self.stop_state)), true)
                }
            }
            Command::EnableSave(val) => {
                self.awaiting_save = val;
                CommandResult(None, false)
            }
            Command::Expression(div, value) => {
                CommandResult(Some(Event::Expression(div, value)), false)
            }
            Command::Crescendo(value) => CommandResult(Some(Event::Crescendo(value)), false),
        }
    }
}
