use crate::event::Event;

const CRESCENDO_STEPS: usize = 39;
const CRESCENDO_PEDAL_ORDER: [[u8; 2]; 38] = [
    [3, 10], // Swell to Pedal 8'
    [3, 11], // Swell to Pedal 4'
    [3, 12], // Great to Pedal 8'
    [3, 13], // Choir to Pedal 8'
    [2, 14], // Choir to Choir 4'
    [1, 9],  // Swell to Great 8'
    [1, 12], // Choir to Great 8'
    [1, 13], // Choir to Great 4'
    [1, 2],  // Dulciana 8'
    [0, 0],  // Gedeckt Flute 8'
    [1, 4],  // Flute Ouverte 4'
    [1, 1],  // Bourdon 8'
    [3, 1],  // Gedekct 16'
    [0, 1],  // Salicional 8'
    [2, 1],  // Gemshorn 8'
    [0, 2],  // Voix Celeste 8'
    [0, 4],  // Harmonic Flute 4'
    [0, 5],  // Block Flute 2'
    [2, 4],  // Gemshorn 4'
    [3, 2],  // Gemshorn 16'
    [3, 3],  // Principal 8'
    [2, 3],  // Koppel Flute 4'
    [2, 0],  // Koppel Flute 8'
    [1, 0],  // Principal 8'
    [1, 3],  // Octave 4'
    [1, 5],  // Fifteenth 2'
    [2, 6],  // Koppel Flute 2'
    [0, 3],  // Geigen Principal 8'
    [3, 0],  // Open Diapason 16'
    [1, 6],  // Plein Jeu IV (Fourniture)
    [3, 7],  // Flute 4'
    [0, 8],  // Clarion 4'
    [3, 8],  // Contra Trompette 16'
    [1, 10], // Swell to Great 4'
    [0, 13], // Swell to Swell 4'
    [2, 12], // Choir to Choir 16'
    [0, 7],  // Trompette 8'
    [0, 6],  // Scharf III-IV
];

const CRESCENDO_INDUCED_STATES: [u64; CRESCENDO_STEPS] = {
    let mut states = [0u64; CRESCENDO_STEPS];
    // This is essentially a for loop, but we can't use `Range` in const expressions.
    let mut i = 0;
    while i < CRESCENDO_STEPS - 1 {
        states[i + 1] = states[i];
        let div = CRESCENDO_PEDAL_ORDER[i][0] as usize;
        let stop_bit = 16 * div + CRESCENDO_PEDAL_ORDER[i][1] as usize;
        states[i + 1] |= 1 << stop_bit;
        i += 1;
    }
    states
};

pub struct DecoderState {
    stop_state: u64,
    crescendo_idx: usize,
}

pub struct EventHandlerResult {
    pub stop_state: Option<u64>,
}

impl DecoderState {
    pub fn new() -> Self {
        Self {
            stop_state: 0u64,
            crescendo_idx: 0,
        }
    }

    pub fn handle(&mut self, event: Event) -> EventHandlerResult {
        match event {
            Event::NoteOff(div, value) => {
                defmt::info!("Received NoteOff div {:?} value {:?}", div as u8, value);
                EventHandlerResult { stop_state: None }
            }
            Event::NoteOn(div, value) => {
                defmt::info!("Received NoteOn div {:?} value {:?}", div as u8, value);
                EventHandlerResult { stop_state: None }
            }
            Event::StopOff(div, idx) => {
                self.stop_state &= !(1 << 16 * div as u8 + idx);
                EventHandlerResult {
                    stop_state: Some(self.effective_stop_state()),
                }
            }
            Event::StopOn(div, idx) => {
                self.stop_state |= 1 << 16 * div as u8 + idx;
                EventHandlerResult {
                    stop_state: Some(self.effective_stop_state()),
                }
            }
            Event::PresetRecalled(_, new_stop_state) => {
                self.stop_state = new_stop_state;
                EventHandlerResult {
                    stop_state: Some(self.effective_stop_state()),
                }
            }
            Event::GeneralCancel() => {
                self.stop_state = 0;
                EventHandlerResult {
                    stop_state: Some(self.effective_stop_state()),
                }
            }
            Event::Expression(div, value) => {
                defmt::info!("Received Expression div {:?} value {:?}", div as u8, value);
                EventHandlerResult { stop_state: None }
            }
            Event::Crescendo(value) => {
                // By contract, value is guaranteed 0-127; map it down to 0-38.
                // This is a little hacky, but i don't have stdlib for round/floor/ceil.
                self.crescendo_idx = ((value & 0x7F) as u16 * 3 / 10) as usize;
                EventHandlerResult {
                    stop_state: Some(self.effective_stop_state()),
                }
            }
        }
    }

    pub fn effective_stop_state(&self) -> u64 {
        self.stop_state | CRESCENDO_INDUCED_STATES[self.crescendo_idx]
    }
}
