use crate::{command::Command, event::Event};

pub trait PresetStore {
    async fn save(&mut self, key: u8, value: &u64);
    async fn load(&mut self, key: u8) -> u64;
}

pub struct Encoder<'d, T: PresetStore> {
    stops: u64,
    awaiting_save: bool,
    preset_store: &'d mut T,
}

pub struct CommandResult(pub Option<Event>, pub bool);

impl<'d, T: PresetStore> Encoder<'d, T> {
    pub fn new(preset_store: &'d mut T) -> Self {
        Self {
            stops: 0u64,
            awaiting_save: false,
            preset_store: preset_store,
        }
    }

    pub fn get_stop_state(&self) -> u64 {
        self.stops
    }

    pub async fn handle(&mut self, cmd: Command) -> CommandResult {
        // This is the guts of the primary encoder MCU. All commands issued to the organ go through this event loop in
        // order to affect any state on the organ. Some commands pass through essentially untouched to the decoder,
        // while others may result in numerous or even zero outbound events. This first match applies state-related
        // logic to incoming encoder commands (e.g. "toggle" messages converted to "on" or "off" depending on state).
        match cmd {
            Command::NoteOff(div, value) => CommandResult(Some(Event::NoteOff(div, value)), false),
            Command::NoteOn(div, value) => CommandResult(Some(Event::NoteOn(div, value)), false),
            Command::StopOff(stop) => {
                self.stops &= !(1 << stop.offset());
                CommandResult(Some(Event::StopOff(stop)), true)
            }
            Command::StopOn(stop) => {
                self.stops |= 1 << stop.offset();
                CommandResult(Some(Event::StopOn(stop)), true)
            }
            Command::StopToggle(stop) => {
                let mask = 1 << stop.offset();
                self.stops ^= mask;
                if self.stops & mask != 0 {
                    CommandResult(Some(Event::StopOn(stop)), true)
                } else {
                    CommandResult(Some(Event::StopOff(stop)), true)
                }
            }
            Command::GeneralCancel() => {
                self.stops = 0;
                CommandResult(Some(Event::GeneralCancel()), true)
            }
            Command::RecallPreset(idx) => {
                if self.awaiting_save {
                    // Saving a preset is internal; it emits no events.
                    self.preset_store.save(idx, &self.stops).await;
                    // Safety: Disengage save mode if we've already written one preset.
                    self.awaiting_save = false;
                    CommandResult(None, false)
                } else {
                    self.stops = self.preset_store.load(idx).await;
                    CommandResult(Some(Event::PresetRecalled(idx, self.stops)), true)
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
