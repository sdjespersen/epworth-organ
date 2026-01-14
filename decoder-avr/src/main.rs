#![no_std]
#![no_main]

use panic_halt as _;

use embedded_midi::MidiIn;
use embedded_midi::MidiMessage;

const FLUSH_STOP_STATE_ITVL_TICKS: u32 = 1_000;
const LED_TOGGLE_ITVL_TICKS: u32 = 50_000;
const CRESCENDO_STEPS: usize = 39;

// Bit manipulation constants
const B: [u32; 4] = [0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF];
const S: [u32; 4] = [1, 2, 4, 8];
const NIBBLE_REVERSE_LOOKUP: [u8; 16] = [
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
];

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

// Evaluated at compile time
const CRESCENDO_INDUCED_STATES: [[u16; 4]; CRESCENDO_STEPS] = {
    let mut states = [[0u16; 4]; CRESCENDO_STEPS];
    let mut i = 0;
    while i < 38 {
        let mut j = 0;
        while j < 4 {
            states[i + 1][j] = states[i][j];
            j += 1;
        }
        let division = CRESCENDO_PEDAL_ORDER[i][0] as usize;
        let stop_bit = CRESCENDO_PEDAL_ORDER[i][1];
        states[i + 1][division] |= 1 << stop_bit;
        i += 1;
    }
    states
};

fn interleave_bits(a: u16, b: u16) -> u32 {
    let mut x = a as u32;
    let mut y = b as u32;

    for i in (0..4).rev() {
        x = (x | (x << S[i])) & B[i];
        y = (y | (y << S[i])) & B[i];
    }
    x | (y << 1)
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 31250);
    let (rx, _tx) = serial.split();
    let mut midi_in = MidiIn::new(rx);

    let mut onboard_led = pins.d13.into_output();

    // For controlling the shift registers
    let mut oe = pins.d5.into_output();
    let mut clk = pins.d4.into_output();
    let mut latch = pins.d3.into_output();
    let mut dta = pins.d2.into_output();

    oe.set_high(); // Disable outputs initially

    let mut stop_state = [0u16; 4];
    let mut crescendo_induced_stop_state = [0u16; 4];
    let mut pending_changes = true;

    let mut ticks = 0u32;

    oe.set_low();

    loop {
        match midi_in.read() {
            Ok(message) => {
                match message {
                    MidiMessage::ControlChange(channel, control_number, control_value) => {
                        let p_channel = u8::from(channel);
                        let p_control_number = u8::from(control_number);
                        if p_channel == 4 && p_control_number == 11 {
                            // Crescendo pedal
                            // control_value is guaranteed 0-127; map it down to 0-38
                            // This is a little hacky, but i don't have stdlib for round/floor/ceil
                            let raw_mapped = ((u8::from(control_value) as u16) * 3 / 10) as usize;
                            for i in 0..4 {
                                crescendo_induced_stop_state[i] =
                                    CRESCENDO_INDUCED_STATES[raw_mapped][i];
                                pending_changes = true;
                            }
                        } else if p_channel < 4
                            && p_control_number >= 102
                            && p_control_number <= 117
                        {
                            // Stop tab change
                            let stop_bit = p_control_number - 102; // is this L to R, or R to L? (MSBfirst, LSBfirst?)
                            if u8::from(control_value) > 63 {
                                stop_state[p_channel as usize] |= 1 << stop_bit;
                            } else {
                                stop_state[p_channel as usize] &= !(1 << stop_bit);
                            }
                            pending_changes = true;
                        }
                    }
                    // Everything else unhandled right now
                    _ => {}
                }
            }
            Err(_) => {} // probably should do something with this eventually
        }

        if pending_changes && (ticks % FLUSH_STOP_STATE_ITVL_TICKS == 0) {
            pending_changes = false;

            // Interleave States
            let left_half = interleave_bits(
                stop_state[1] | crescendo_induced_stop_state[1],
                stop_state[2] | crescendo_induced_stop_state[2],
            );
            let right_half = interleave_bits(
                stop_state[3] | crescendo_induced_stop_state[3],
                stop_state[0] | crescendo_induced_stop_state[0],
            );

            // Shift Out Right Half then Left Half
            for val in [right_half, left_half] {
                for i in 0..4 {
                    let mut to_write = ((val >> (8 * i)) & 0xFF) as u8;
                    // Reverse lower nibble
                    to_write =
                        (to_write & 0xF0) | NIBBLE_REVERSE_LOOKUP[(to_write & 0x0F) as usize];

                    // Manual ShiftOut
                    for bit in 0..8 {
                        if (to_write >> bit) & 1 == 1 {
                            dta.set_high();
                        } else {
                            dta.set_low();
                        }
                        clk.set_high();
                        clk.set_low();
                    }
                }
            }

            latch.set_high();
            latch.set_low();
        }

        if ticks % LED_TOGGLE_ITVL_TICKS == 0 {
            onboard_led.toggle();
        }

        // I'd love to use async rust, but i'm not really sure how to do that on AVR yet. In the meantime, we'll just
        // keep a tick counter without regard to real time. (I note that the implementation of delay_us and similar is
        // really just a busy loop of machine instructions anyway, so this is not too different in practice.)
        ticks += 1;

        arduino_hal::delay_us(10);
    }
}
