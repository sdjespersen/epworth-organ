use crate::stops;
use crate::{Division, event::Event};

const CRESCENDO_STEPS: usize = 39;
const CRESCENDO_PEDAL_ORDER: [stops::Stop; CRESCENDO_STEPS - 1] = [
    stops::SWELL_TO_PEDAL_8,
    stops::SWELL_TO_PEDAL_4,
    stops::GREAT_TO_PEDAL_8,
    stops::CHOIR_TO_PEDAL_8,
    stops::CHOIR_TO_CHOIR_4,
    stops::SWELL_TO_GREAT_8,
    stops::CHOIR_TO_GREAT_8,
    stops::CHOIR_TO_GREAT_4,
    stops::GREAT_DULCIANA_8,
    stops::SWELL_GEDECKT_FLUTE_8,
    stops::GREAT_FLUTE_OUVERTE_4,
    stops::GREAT_BOURDON_8,
    stops::PEDAL_GEDECKT_16,
    stops::SWELL_SALICIONAL_8,
    stops::CHOIR_GEMSHORN_8,
    stops::SWELL_VOIX_CELESTE_8,
    stops::SWELL_HARMONIC_FLUTE_4,
    stops::SWELL_BLOCK_FLUTE_2,
    stops::CHOIR_GEMSHORN_4,
    stops::PEDAL_GEMSHORN_16,
    stops::PEDAL_PRINCIPAL_8,
    stops::CHOIR_KOPPEL_FLUTE_4,
    stops::CHOIR_KOPPEL_FLUTE_8,
    stops::GREAT_PRINCIPAL_8,
    stops::GREAT_OCTAVE_4,
    stops::GREAT_FIFTEENTH_2,
    stops::CHOIR_KOPPEL_FLUTE_2,
    stops::SWELL_GEIGEN_PRINCIPAL_8,
    stops::PEDAL_OPEN_DIAPASON_16,
    stops::GREAT_FOURNITURE_IV,
    stops::PEDAL_FLUTE_4,
    stops::SWELL_CLARION_4,
    stops::PEDAL_CONTRA_TROMPETTE_16,
    stops::SWELL_TO_GREAT_4,
    stops::SWELL_TO_SWELL_4,
    stops::CHOIR_TO_CHOIR_16,
    stops::SWELL_TROMPETTE_8,
    stops::SWELL_SCHARF_III_IV,
];
// A running accumulation of all the crescendo steps.
const CRESCENDO_INDUCED_STATES: [u64; CRESCENDO_STEPS] = {
    let mut states = [0u64; CRESCENDO_STEPS];
    // This is essentially a for loop, but we can't use `Range` in const expressions.
    let mut i = 0;
    while i < CRESCENDO_STEPS - 1 {
        states[i + 1] = states[i];
        states[i + 1] |= 1 << CRESCENDO_PEDAL_ORDER[i].offset();
        i += 1;
    }
    states
};

const ALL_DIVISIONS: [Option<Division>; 4] = [
    Some(Division::Swell),
    Some(Division::Great),
    Some(Division::Choir),
    Some(Division::Pedal),
];

const fn coupled_divisions(div: Division) -> [Option<Division>; 4] {
    match div {
        // Couplers on the swell can affect sounding notes only on the swell division.
        Division::Swell => [Some(Division::Swell), None, None, None], //true, false, false, false],
        // Couplers on the great can affect sounding notes on any division except the pedal.
        Division::Great => [
            Some(Division::Swell),
            Some(Division::Great),
            Some(Division::Choir),
            None,
        ],
        // Couplers on the choir can affect sounding notes on the choir or swell division.
        Division::Choir => [Some(Division::Swell), None, Some(Division::Choir), None],
        // Couplers on the pedal can affect sounding notes on any division.
        Division::Pedal => ALL_DIVISIONS,
    }
}

pub struct Decoder {
    // Which stops are explicitly toggled on or off.
    stops: u64,
    // Which discrete "level" the crescendo pedal is at.
    crescendo_idx: usize,
    // Which keys are currently pressed. (No couplings applied.)
    keys_down: [u64; 4],
    // Which notes are currently sounding. (Couplings applied.)
    notes_on: [u64; 4],
}

// TODO: Reconsider this data structure. Do i really want to return values? Or just bools with what needs updating?
// Note that for now, we don't have the hardware in place to update notes, so this is OK for now.
pub struct EventHandlerResult {
    pub stop_state: Option<u64>,
}

impl Decoder {
    pub fn new() -> Self {
        Self {
            stops: 0u64,
            crescendo_idx: 0,
            keys_down: [0u64; 4],
            notes_on: [0u64; 4],
        }
    }

    pub fn handle(&mut self, event: Event) -> EventHandlerResult {
        // Most events will trigger a recomputation of the note state with couplings applied.
        let mut recompute_note_state: [Option<Division>; 4] = [None; 4];

        let result = match event {
            Event::NoteOff(div, value) => {
                let div_idx: u8 = div.into();
                self.keys_down[div_idx as usize] &= !(1 << value);
                recompute_note_state = coupled_divisions(div);
                EventHandlerResult { stop_state: None }
            }
            Event::NoteOn(div, value) => {
                let div_idx: u8 = div.into();
                self.keys_down[div_idx as usize] |= 1 << value;
                recompute_note_state = coupled_divisions(div);
                EventHandlerResult { stop_state: None }
            }
            Event::StopOff(stop) => {
                self.stops &= !(1 << stop.offset());
                if let stops::StopKind::Coupler = stop.kind() {
                    recompute_note_state = coupled_divisions(stop.div)
                }
                EventHandlerResult {
                    stop_state: Some(self.stop_state()),
                }
            }
            Event::StopOn(stop) => {
                self.stops |= 1 << stop.offset();
                if let stops::StopKind::Coupler = stop.kind() {
                    recompute_note_state = coupled_divisions(stop.div)
                }
                EventHandlerResult {
                    stop_state: Some(self.stop_state()),
                }
            }
            Event::PresetRecalled(_, new_stop_state) => {
                self.stops = new_stop_state;
                // Without going over things with a fine-toothed comb, we should probably assume everything needs
                // updating.
                recompute_note_state = ALL_DIVISIONS;
                EventHandlerResult {
                    stop_state: Some(self.stop_state()),
                }
            }
            Event::GeneralCancel() => {
                // TODO: Do i want GC to clear all notes, actually? Could be a good panic button.
                self.stops = 0;
                EventHandlerResult {
                    stop_state: Some(self.stop_state()),
                }
            }
            Event::Expression(_, _) => {
                // Expression pedals do not interact with notes or couplers.
                EventHandlerResult { stop_state: None }
            }
            Event::Crescendo(value) => {
                // By contract, value is guaranteed 0-127; map it down to 0-38.
                // This is a little hacky, but i don't have stdlib for round/floor/ceil.
                self.crescendo_idx = ((value & 0x7F) as u16 * 3 / 10) as usize;
                // The crescendo pedal has wide-ranging coupling effects. Don't try to be fancy here; update everything.
                recompute_note_state = ALL_DIVISIONS;
                EventHandlerResult {
                    stop_state: Some(self.stop_state()),
                }
            }
        };

        for maybe_div in recompute_note_state {
            if let Some(div) = maybe_div {
                match div {
                    Division::Swell => {
                        self.notes_on[0] = self.keys_down[0];
                        // Need to handle unison off first for correctness.
                        if self.stops & (1 << stops::SWELL_UNISON_OFF.offset()) != 0 {
                            self.notes_on[0] = 0;
                        }
                        if self.stops & (1 << stops::SWELL_TO_SWELL_16.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[0] >> 12;
                        }
                        if self.stops & (1 << stops::SWELL_TO_SWELL_4.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[0] << 12;
                        }
                        if self.stops & (1 << stops::SWELL_TO_GREAT_16.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[1] >> 12;
                        }
                        if self.stops & (1 << stops::SWELL_TO_GREAT_8.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[1];
                        }
                        if self.stops & (1 << stops::SWELL_TO_GREAT_4.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[1] << 12;
                        }
                        if self.stops & (1 << stops::SWELL_TO_CHOIR_8.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[2];
                        }
                        if self.stops & (1 << stops::SWELL_TO_CHOIR_4.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[2] << 12;
                        }
                        if self.stops & (1 << stops::SWELL_TO_PEDAL_8.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[3];
                        }
                        if self.stops & (1 << stops::SWELL_TO_PEDAL_4.offset()) != 0 {
                            self.notes_on[0] |= self.keys_down[3] << 12;
                        }
                    }
                    Division::Great => {
                        self.notes_on[1] = self.keys_down[1];
                        if self.stops & (1 << stops::GREAT_TO_GREAT_4.offset()) != 0 {
                            self.notes_on[1] |= self.keys_down[1] << 12;
                        }
                        if self.stops & (1 << stops::GREAT_TO_PEDAL_8.offset()) != 0 {
                            self.notes_on[1] |= self.keys_down[3];
                        }
                    }
                    Division::Choir => {
                        self.notes_on[2] = self.keys_down[2];
                        // Need to handle unison off first for correctness.
                        if self.stops & (1 << stops::CHOIR_UNISON_OFF.offset()) != 0 {
                            self.notes_on[2] = 0;
                        }
                        if self.stops & (1 << stops::CHOIR_TO_CHOIR_16.offset()) != 0 {
                            self.notes_on[2] |= self.keys_down[2] >> 12;
                        }
                        if self.stops & (1 << stops::CHOIR_TO_CHOIR_4.offset()) != 0 {
                            self.notes_on[2] |= self.keys_down[2] << 12;
                        }
                        if self.stops & (1 << stops::CHOIR_TO_GREAT_16.offset()) != 0 {
                            self.notes_on[2] |= self.keys_down[1] >> 12;
                        }
                        if self.stops & (1 << stops::CHOIR_TO_GREAT_8.offset()) != 0 {
                            self.notes_on[2] |= self.keys_down[1];
                        }
                        if self.stops & (1 << stops::CHOIR_TO_GREAT_4.offset()) != 0 {
                            self.notes_on[2] |= self.keys_down[1] << 12;
                        }
                        if self.stops & (1 << stops::CHOIR_TO_PEDAL_8.offset()) != 0 {
                            self.notes_on[2] |= self.keys_down[3];
                        }
                    }
                    Division::Pedal => {
                        self.notes_on[3] = self.keys_down[3];
                    }
                }
            }
        }

        result
    }

    pub fn stop_state(&self) -> u64 {
        self.stops | CRESCENDO_INDUCED_STATES[self.crescendo_idx]
    }

    pub fn note_state(&self) -> [u64; 4] {
        self.notes_on
    }
}

#[cfg(test)]
mod tests {
    use crate::{Division, decoder::Decoder, event::Event, stops};

    #[test]
    fn test_couplings() {
        let mut dec = Decoder::new();

        let _ = dec.handle(Event::StopOn(stops::SWELL_UNISON_OFF));
        let _ = dec.handle(Event::StopOn(stops::SWELL_TO_GREAT_8));
        let _ = dec.handle(Event::StopOn(stops::CHOIR_TO_GREAT_16));
        let _ = dec.handle(Event::StopOn(stops::GREAT_TO_GREAT_4));
        let _ = dec.handle(Event::StopOn(stops::SWELL_TO_PEDAL_4));
        let _ = dec.handle(Event::StopOn(stops::GREAT_TO_PEDAL_8));
        let _ = dec.handle(Event::StopOn(stops::CHOIR_TO_PEDAL_8));

        // TODO: Figure out what i want the return type to be, then assert on it.
        // Playing a note on the swell should have no effect, because swell unison is off.
        let _ = dec.handle(Event::NoteOn(Division::Swell, 25));
        assert_eq!(dec.note_state(), [0u64, 0u64, 0u64, 0u64]);
        // The great is targeted by a few couplers.
        let _ = dec.handle(Event::NoteOn(Division::Great, 33));
        assert_eq!(
            dec.note_state(),
            [1 << 33, 1 << 33 | 1 << 45, 1 << 21, 0u64]
        );
        // The choir is not targeted by any couplers currently.
        let _ = dec.handle(Event::NoteOn(Division::Choir, 30));
        assert_eq!(
            dec.note_state(),
            [1 << 33, 1 << 33 | 1 << 45, 1 << 21 | 1 << 30, 0u64]
        );
        // The pedal has a handful of couplers.
        let _ = dec.handle(Event::NoteOn(Division::Pedal, 20));
        assert_eq!(
            dec.note_state(),
            [
                1 << 33 | 1 << 32,
                1 << 33 | 1 << 45 | 1 << 20,
                1 << 21 | 1 << 30 | 1 << 20,
                1 << 20
            ]
        );
        // Turning off couplers should affect the note state.
        let _ = dec.handle(Event::StopOff(stops::SWELL_UNISON_OFF));
        assert_eq!(
            dec.note_state(),
            [
                1 << 25 | 1 << 33 | 1 << 32,
                1 << 33 | 1 << 45 | 1 << 20,
                1 << 21 | 1 << 30 | 1 << 20,
                1 << 20
            ]
        );
        let _ = dec.handle(Event::StopOff(stops::SWELL_TO_PEDAL_4));
        let _ = dec.handle(Event::StopOff(stops::GREAT_TO_PEDAL_8));
        assert_eq!(
            dec.note_state(),
            [
                1 << 25 | 1 << 33,
                1 << 33 | 1 << 45,
                1 << 21 | 1 << 30 | 1 << 20,
                1 << 20
            ]
        );
        // Releasing a note should release the coupled notes.
        let _ = dec.handle(Event::NoteOff(Division::Great, 33));
        assert_eq!(dec.note_state(), [1 << 25, 0, 1 << 30 | 1 << 20, 1 << 20]);
        // Turning on a coupler while notes are held should affect coupled notes.
        let _ = dec.handle(Event::StopOn(stops::SWELL_TO_CHOIR_4));
        let _ = dec.handle(Event::StopOn(stops::CHOIR_TO_CHOIR_16));
        let _ = dec.handle(Event::StopOn(stops::CHOIR_UNISON_OFF));
        assert_eq!(
            dec.note_state(),
            [1 << 25 | 1 << 42, 0, 1 << 18 | 1 << 20, 1 << 20]
        );
    }
}
