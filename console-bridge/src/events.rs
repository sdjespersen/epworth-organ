#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum Division {
    Swell,
    Great,
    Choir,
    Pedal,
    General,
}

// impl Division {
//     pub fn from(i: usize) -> Self {

//     }
// }

impl Into<usize> for Division {
    fn into(self) -> usize {
        match self {
            Division::Swell => 0,
            Division::Great => 1,
            Division::Choir => 2,
            Division::Pedal => 3,
            Division::General => 4,
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
            4 => Division::General,
            // TODO: Define a proper type for this so we know it's always valid.
            _ => panic!("Invalid division index: {}", self),
        }
    }
}

// #[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
// pub enum EventType {
//     NoteOff,
//     NoteOn,
//     StopOff,
//     StopOn,
//     StopToggle,
//     RecallPreset,
//     GeneralCancel,
//     EnableSave,
//     Expression,
// }

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum EventSource {
    Indigenous,
    Usb,
    Uart,
}

// Represents an event on the organ. Most of these map pretty directly to MIDI messages, but some (like `StopToggle` and
// `EnableSave`) do not.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
pub enum Event {
    NoteOff {
        src: EventSource,
        div: Division,
        key: u8,
        vel: u8,
    },
    NoteOn {
        src: EventSource,
        div: Division,
        key: u8,
        vel: u8,
    },
    StopOff {
        src: EventSource,
        div: Division,
        idx: u8,
    },
    StopOn {
        src: EventSource,
        div: Division,
        idx: u8,
    },
    StopToggle {
        src: EventSource,
        div: Division,
        idx: u8,
    },
    RecallPreset {
        src: EventSource,
        idx: u8,
    },
    GeneralCancel {
        src: EventSource,
    },
    EnableSave {
        src: EventSource,
        val: bool,
    },
    Expression {
        src: EventSource,
        div: Division,
        value: u8,
    },
}

// impl Event {
//     pub fn new(source: EventSource, division: Division, event_type: EventType, value: u8) -> Self {
//         Self {
//             source,
//             division,
//             event_type,
//             value,
//         }
//     }

//     pub fn div_usize(&self) -> usize {
//         self.division.to_usize()
//     }
// }

// pub enum Event {
//     /// Stop playing a note.
//     NoteOff {
//         /// The MIDI key to stop playing.
//         key: u7,
//         /// The velocity with which to stop playing it.
//         vel: u7,
//     },
//     /// Start playing a note.
//     NoteOn {
//         /// The key to start playing.
//         key: u7,
//         /// The velocity (strength) with which to press it.
//         ///
//         /// Note that by convention a `NoteOn` message with a velocity of 0 is equivalent to a
//         /// `NoteOff`.
//         vel: u7,
//     },
//     /// Modify the velocity of a note after it has been played.
//     Aftertouch {
//         /// The key for which to modify its velocity.
//         key: u7,
//         /// The new velocity for the key.
//         vel: u7,
//     },
//     /// Modify the value of a MIDI controller.
//     Controller {
//         /// The controller to modify.
//         ///
//         /// See the MIDI spec for the meaning of each index.
//         controller: u7,
//         /// The value to set it to.
//         value: u7,
//     },
//     /// Change the program (also known as instrument) for a channel.
//     ProgramChange {
//         /// The new program (instrument) to use for the channel.
//         program: u7,
//     },
//     /// Change the note velocity of a whole channel at once, without starting new notes.
//     ChannelAftertouch {
//         /// The new velocity for all notes currently playing in the channel.
//         vel: u7,
//     },
//     /// Set the pitch bend value for the entire channel.
//     PitchBend {
//         /// The new pitch-bend value.
//         bend: PitchBend,
//     },
// }

// #[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
// pub struct SourcedEvent {
//     pub
//     pub event: Event,
// }
