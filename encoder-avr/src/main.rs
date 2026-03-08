#![no_std]
#![no_main]

mod debouncer;
mod mcp23017;

use embedded_hal::i2c::I2c;
use embedded_hal_bus::util::AtomicCell;
use panic_halt as _;

use debouncer::Debouncer;

use embedded_hal_bus::i2c::AtomicDevice;
use mcp23017::{MCP23017, Port};

// Bit manipulation constants
const B: [u32; 4] = [0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF];
const S: [u32; 4] = [1, 2, 4, 8];
const NIBBLE_REVERSE_LOOKUP: [u8; 16] = [
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
];

const SAVE_PISTON_IDX: u8 = 0;
const GC_PISTON_IDX: u8 = 1;

const STOP_TAB_BUTTON_POLL_ITVL_TICKS: u32 = 5;
const PISTON_POLL_ITVL_TICKS: u32 = 5;
const LED_TOGGLE_ITVL_TICKS: u32 = 2_000;
const FLUSH_STOP_STATE_ITVL_TICKS: u32 = 100;

fn interleave_bits(a: u16, b: u16) -> u32 {
    let mut x = a as u32;
    let mut y = b as u32;

    for i in (0..4).rev() {
        x = (x | (x << S[i])) & B[i];
        y = (y | (y << S[i])) & B[i];
    }
    x | (y << 1)
}

fn reverse_byte(a: &u8) -> u8 {
    let mut b = *a;
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    b
}

fn read_stop_tabs<I2C: I2c>(mcp_pair: &mut [MCP23017<I2C>; 2]) -> u16 {
    let mut raw_button_readings = 0x01u16; // LSB always 1, by convention
    raw_button_readings |= ((mcp_pair[0].read_gpio(Port::GPIOA).unwrap() & 0x01) as u16) << 8;
    raw_button_readings |= ((mcp_pair[0].read_gpio(Port::GPIOB).unwrap() & 0x7F) as u16) << 9;
    raw_button_readings |= ((mcp_pair[1].read_gpio(Port::GPIOB).unwrap() & 0x7F) as u16) << 1;
    raw_button_readings
}

fn write_stop_state_to_leds<I2C: I2c>(stop_state: &u16, mcp_pair: &mut [MCP23017<I2C>; 2]) -> () {
    let left_half = reverse_byte(&(!(stop_state >> 8) as u8));
    let right_half = reverse_byte(&(!(stop_state & 0xFF) as u8));
    mcp_pair[0]
        .write_gpio(Port::GPIOA, (left_half & 0x7F) << 1)
        .unwrap();
    mcp_pair[0]
        .write_gpio(Port::GPIOB, left_half & 0x80)
        .unwrap();
    mcp_pair[1]
        .write_gpio(Port::GPIOA, right_half & 0x7F)
        .unwrap();
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let i2c = arduino_hal::i2c::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100_000,
    );
    let i2c_cell = AtomicCell::new(i2c);

    let mut mcps = [
        // [
        //     MCP23017::new(AtomicDevice::new(&i2c_cell), 0x20),
        //     MCP23017::new(AtomicDevice::new(&i2c_cell), 0x21),
        // ],
        [
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x22),
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x23),
        ],
        [
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x24),
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x25),
        ],
        // [
        //     MCP23017::new(AtomicDevice::new(&i2c_cell), 0x26),
        //     MCP23017::new(AtomicDevice::new(&i2c_cell), 0x27),
        // ],
    ];
    for mcp_pair in mcps.iter_mut() {
        mcp_pair[0].init_hardware().unwrap();
        mcp_pair[1].init_hardware().unwrap();
        // Set pin modes (input/output) for MCP23017s based on the schematic
        mcp_pair[0].port_mode(Port::GPIOA, 0x01).unwrap();
        mcp_pair[0].port_mode(Port::GPIOB, 0x7F).unwrap();
        mcp_pair[1].port_mode(Port::GPIOA, 0x00).unwrap();
        mcp_pair[1].port_mode(Port::GPIOB, 0x7F).unwrap();
        // Write all outputs high, inputs low.
        mcp_pair[0].write_gpio(Port::GPIOA, 0xFE).unwrap();
        mcp_pair[0].write_gpio(Port::GPIOB, 0x80).unwrap();
        mcp_pair[1].write_gpio(Port::GPIOA, 0x7F).unwrap();
        mcp_pair[1].write_gpio(Port::GPIOB, 0x00).unwrap();
    }

    let mut onboard_led = pins.d13.into_output();

    let piston_data_pin = pins.d8.into_pull_up_input();
    let mut piston_clock_pin = pins.d9.into_output();
    let mut piston_load_pin = pins.d10.into_output();

    // For controlling the shift registers
    let mut oe = pins.d5.into_output();
    let mut clk = pins.d4.into_output();
    let mut latch = pins.d3.into_output();
    let mut dta = pins.d2.into_output();

    oe.set_high(); // Disable outputs initially

    let mut ticks = 0u32;

    oe.set_low();

    let mut awaiting_save_preset = false;
    let mut presets = [[0u16; 4]; 8];

    let mut stop_tab_button_debouncers = [
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
    ];
    let mut stop_state: [u16; 4] = [0u16; 4];
    let mut pending_changes = true;

    let mut piston_debouncer = Debouncer::<5>::new(0xFFFF);

    loop {
        // Heartbeat indicator
        if ticks % LED_TOGGLE_ITVL_TICKS == 0 {
            onboard_led.toggle();
        }

        // Stop tabs
        if ticks % STOP_TAB_BUTTON_POLL_ITVL_TICKS == 0 {
            // Stop tab buttons for each division are logically separate, so this looks like 4 scans.
            for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                stop_tab_button_debouncers[div].update(read_stop_tabs(mcp_pair));
                stop_tab_button_debouncers[div].for_each_falling_edge(|i| {
                    stop_state[div] ^= (1 << i) as u16;

                    write_stop_state_to_leds(&stop_state[div], mcp_pair);

                    pending_changes = true;
                });
            }
        }

        // Pistons
        if ticks % PISTON_POLL_ITVL_TICKS == 0 {
            // Load piston states into 74HC165 shift register
            piston_load_pin.set_low();
            piston_load_pin.set_high();

            let mut raw_piston_reading = 0u16;
            // Ensure clock pin low first...
            piston_clock_pin.set_low();

            // ...then clock the data into the piston readings.
            for i in 0..8 {
                if piston_data_pin.is_high() {
                    raw_piston_reading |= 1 << i;
                }
                piston_clock_pin.set_high();
                piston_clock_pin.set_low();
            }
            // Nothing is hooked up to 8-15 right now...
            raw_piston_reading |= 0xFF00;

            piston_debouncer.update(raw_piston_reading);

            piston_debouncer.for_each_falling_edge(|i| {
                if i == SAVE_PISTON_IDX {
                    // SAVE piston
                    awaiting_save_preset = true;
                } else if i == GC_PISTON_IDX {
                    // GC piston
                    for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                        stop_state[div] = 0;
                        write_stop_state_to_leds(&stop_state[div], mcp_pair);
                    }
                    pending_changes = true;
                } else {
                    if awaiting_save_preset {
                        // write preset
                        for div in 0..4 {
                            presets[i as usize][div] = stop_state[div];
                        }
                        // Force the user to release and re-engage the save button to save another preset.
                        awaiting_save_preset = false;
                    } else {
                        // recall preset
                        for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                            stop_state[div] = presets[i as usize][div];
                            write_stop_state_to_leds(&stop_state[div], mcp_pair);
                        }
                        pending_changes = true;
                    }
                }
            });

            piston_debouncer.for_each_rising_edge(|i| {
                if i == SAVE_PISTON_IDX {
                    awaiting_save_preset = false;
                }
            });
        }

        if pending_changes && (ticks % FLUSH_STOP_STATE_ITVL_TICKS == 0) {
            pending_changes = false;

            // Interleave States
            let left_half = interleave_bits(stop_state[1], stop_state[2]);
            let right_half = interleave_bits(stop_state[3], stop_state[0]);

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

        // I'd love to use async rust, but i'm not really sure how to do that on AVR yet. In the meantime, we'll just
        // keep a tick counter without regard to real time. (I note that the implementation of delay_us and similar is
        // really just a busy loop of machine instructions anyway, so this is not too different in practice.)
        ticks += 1;

        // This delay throttles the pin polling a bit, and allows us to roughly approximate the interval of time at
        // which we scan things.
        arduino_hal::delay_us(10);
    }
}
