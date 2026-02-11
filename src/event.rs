use crate::Division;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum Event {
    NoteOff(Division, u8),
    NoteOn(Division, u8),
    StopOff(Division, u8),
    StopOn(Division, u8),
    PresetRecalled(u8, u64),
    GeneralCancel(),
    Expression(Division, u8),
    Crescendo(u8),
}
