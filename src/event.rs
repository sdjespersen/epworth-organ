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
    RecallPreset(u8),
    Expression(Division, u8),
    Crescendo(u8),
}
