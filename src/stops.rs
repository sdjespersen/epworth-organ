use crate::Division;
use serde::{Deserialize, Serialize};

// Swell speaking stops
pub const SWELL_GEDECKT_FLUTE_8: Stop = Stop::new(Division::Swell, 0);
pub const SWELL_SALICIONAL_8: Stop = Stop::new(Division::Swell, 1);
pub const SWELL_VOIX_CELESTE_8: Stop = Stop::new(Division::Swell, 2);
pub const SWELL_GEIGEN_PRINCIPAL_8: Stop = Stop::new(Division::Swell, 3);
pub const SWELL_HARMONIC_FLUTE_4: Stop = Stop::new(Division::Swell, 4);
pub const SWELL_BLOCK_FLUTE_2: Stop = Stop::new(Division::Swell, 5);
pub const SWELL_SCHARF_III_IV: Stop = Stop::new(Division::Swell, 6);
pub const SWELL_TROMPETTE_8: Stop = Stop::new(Division::Swell, 7);
pub const SWELL_CLARION_4: Stop = Stop::new(Division::Swell, 8);
pub const SWELL_TREMOLO: Stop = Stop::new(Division::Swell, 9);
pub const SWELL_UNMARKED_2: Stop = Stop::new(Division::Swell, 10);
// Swell couplers
pub const SWELL_TO_SWELL_16: Stop = Stop::new(Division::Swell, 11);
pub const SWELL_UNISON_OFF: Stop = Stop::new(Division::Swell, 12);
pub const SWELL_TO_SWELL_4: Stop = Stop::new(Division::Swell, 13);
// Great speaking stops
pub const GREAT_PRINCIPAL_8: Stop = Stop::new(Division::Great, 0);
pub const GREAT_BOURDON_8: Stop = Stop::new(Division::Great, 1);
pub const GREAT_DULCIANA_8: Stop = Stop::new(Division::Great, 2);
pub const GREAT_OCTAVE_4: Stop = Stop::new(Division::Great, 3);
pub const GREAT_FLUTE_OUVERTE_4: Stop = Stop::new(Division::Great, 4);
pub const GREAT_FIFTEENTH_2: Stop = Stop::new(Division::Great, 5);
pub const GREAT_FOURNITURE_IV: Stop = Stop::new(Division::Great, 6);
pub const GREAT_CHIMES: Stop = Stop::new(Division::Great, 7);
// Great couplers
pub const SWELL_TO_GREAT_16: Stop = Stop::new(Division::Great, 8);
pub const SWELL_TO_GREAT_8: Stop = Stop::new(Division::Great, 9);
pub const SWELL_TO_GREAT_4: Stop = Stop::new(Division::Great, 10);
pub const CHOIR_TO_GREAT_16: Stop = Stop::new(Division::Great, 11);
pub const CHOIR_TO_GREAT_8: Stop = Stop::new(Division::Great, 12);
pub const CHOIR_TO_GREAT_4: Stop = Stop::new(Division::Great, 13);
pub const GREAT_TO_GREAT_4: Stop = Stop::new(Division::Great, 14);
// Choir speaking stops
pub const CHOIR_KOPPEL_FLUTE_8: Stop = Stop::new(Division::Choir, 0);
pub const CHOIR_GEMSHORN_8: Stop = Stop::new(Division::Choir, 1);
pub const CHOIR_GEMSHORN_CELESTE_8: Stop = Stop::new(Division::Choir, 2);
pub const CHOIR_KOPPEL_FLUTE_4: Stop = Stop::new(Division::Choir, 3);
pub const CHOIR_GEMSHORN_4: Stop = Stop::new(Division::Choir, 4);
pub const CHOIR_WALD_FLUTE_223: Stop = Stop::new(Division::Choir, 5);
pub const CHOIR_KOPPEL_FLUTE_2: Stop = Stop::new(Division::Choir, 6);
pub const CHOIR_CLARINET_8: Stop = Stop::new(Division::Choir, 7);
pub const CHOIR_TREMOLO: Stop = Stop::new(Division::Choir, 8);
// Choir couplers
pub const SWELL_TO_CHOIR_16: Stop = Stop::new(Division::Choir, 9);
pub const SWELL_TO_CHOIR_8: Stop = Stop::new(Division::Choir, 10);
pub const SWELL_TO_CHOIR_4: Stop = Stop::new(Division::Choir, 11);
pub const CHOIR_TO_CHOIR_16: Stop = Stop::new(Division::Choir, 12);
pub const CHOIR_UNISON_OFF: Stop = Stop::new(Division::Choir, 13);
pub const CHOIR_TO_CHOIR_4: Stop = Stop::new(Division::Choir, 14);
// Pedal speaking stops
pub const PEDAL_OPEN_DIAPASON_16: Stop = Stop::new(Division::Pedal, 0);
pub const PEDAL_GEDECKT_16: Stop = Stop::new(Division::Pedal, 1);
pub const PEDAL_GEMSHORN_16: Stop = Stop::new(Division::Pedal, 2);
pub const PEDAL_PRINCIPAL_8: Stop = Stop::new(Division::Pedal, 3);
pub const PEDAL_FLUTE_8: Stop = Stop::new(Division::Pedal, 4);
pub const PEDAL_GEMSHORN_8: Stop = Stop::new(Division::Pedal, 5);
pub const PEDAL_CHORAL_BASS_4: Stop = Stop::new(Division::Pedal, 6);
pub const PEDAL_FLUTE_4: Stop = Stop::new(Division::Pedal, 7);
pub const PEDAL_CONTRA_TROMPETTE_16: Stop = Stop::new(Division::Pedal, 8);
pub const PEDAL_UNTER_SATZ_32: Stop = Stop::new(Division::Pedal, 9);
// Pedal couplers
pub const SWELL_TO_PEDAL_8: Stop = Stop::new(Division::Pedal, 10);
pub const SWELL_TO_PEDAL_4: Stop = Stop::new(Division::Pedal, 11);
pub const GREAT_TO_PEDAL_8: Stop = Stop::new(Division::Pedal, 12);
pub const CHOIR_TO_PEDAL_8: Stop = Stop::new(Division::Pedal, 13);

pub enum StopKind {
    SpeakingStop,
    Coupler,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct Stop {
    offset: u8,
}

pub const fn offset_to_div(offset: u8) -> Division {
    if offset < 16 {
        Division::Swell
    } else if offset < 32 {
        Division::Great
    } else if offset < 48 {
        Division::Choir
    } else {
        Division::Pedal
    }
}

impl Stop {
    const fn new(div: Division, idx: u8) -> Self {
        Self {
            offset: 16 * div.to_u8() + idx,
        }
    }

    pub const fn try_from(div: Division, idx: u8) -> Option<Self> {
        // Avoids constructing invalid stops
        let max_idx = match div {
            Division::Swell | Division::Pedal => 13u8,
            Division::Great | Division::Choir => 14u8,
        };
        if idx <= max_idx {
            return Some(Self::new(div, idx));
        }
        None
    }

    pub const fn kind(&self) -> StopKind {
        if (11 <= self.offset && self.offset <= 13)
            || (24 <= self.offset && self.offset <= 30)
            || (41 <= self.offset && self.offset <= 46)
            || (58 <= self.offset && self.offset <= 61)
        {
            StopKind::Coupler
        } else {
            StopKind::SpeakingStop
        }
    }

    pub const fn div(&self) -> Division {
        offset_to_div(self.offset)
    }

    pub const fn idx(&self) -> u8 {
        self.offset % 16
    }

    pub const fn offset(&self) -> u8 {
        self.offset
    }
}
