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
const NIBBLE_REVERSE_LOOKUP: [u8; 16] = [
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
];

const SAVE_PISTON_IDX: u8 = 0;
const GC_PISTON_IDX: u8 = 1;

const MAX_BUS_RESET_RETRIES: u32 = 16;

const STOP_TAB_BUTTON_POLL_ITVL_TICKS: u32 = 5;
const PISTON_POLL_ITVL_TICKS: u32 = 5;
const LED_TOGGLE_ITVL_TICKS: u32 = 1_000;
const FLUSH_STOP_STATE_ITVL_TICKS: u32 = 100;

fn interleave_bytes(a: u16, b: u16) -> u32 {
    let mut x = a as u32;
    let mut y = b as u32;
    x = (x & 0xFF00) << 16 | (x & 0x00FF) << 8;
    y = (y & 0xFF00) << 8 | (y & 0x00FF);
    x | y
}

fn reverse_byte(a: &u8) -> u8 {
    let mut b = *a;
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    b
}

fn read_stop_tabs<I2C: I2c>(mcp_pair: &mut [MCP23017<I2C>; 2]) -> Result<u16, ()> {
    let mut raw_button_readings = 0x01u16; // LSB always 1, by convention
    raw_button_readings |=
        ((mcp_pair[0].read_gpio(Port::GPIOA).map_err(|_| ())? & 0x01) as u16) << 8;
    raw_button_readings |=
        ((mcp_pair[0].read_gpio(Port::GPIOB).map_err(|_| ())? & 0x7F) as u16) << 9;
    raw_button_readings |=
        ((mcp_pair[1].read_gpio(Port::GPIOB).map_err(|_| ())? & 0x7F) as u16) << 1;
    Ok(raw_button_readings)
}

fn write_stop_state_to_leds<I2C: I2c>(
    stop_state: &u16,
    mcp_pair: &mut [MCP23017<I2C>; 2],
) -> Result<(), ()> {
    let left_half = reverse_byte(&(!(stop_state >> 8) as u8));
    let right_half = reverse_byte(&(!(stop_state & 0xFF) as u8));
    mcp_pair[0]
        .write_gpio(Port::GPIOA, (left_half & 0x7F) << 1)
        .map_err(|_| ())?;
    mcp_pair[0]
        .write_gpio(Port::GPIOB, left_half & 0x80)
        .map_err(|_| ())?;
    mcp_pair[1]
        .write_gpio(Port::GPIOA, right_half & 0x7F)
        .map_err(|_| ())?;
    Ok(())
}

fn config_mcp_pair<I2C: I2c>(mcp_pair: &mut [MCP23017<I2C>; 2]) -> Result<(), ()> {
    mcp_pair[0].init_hardware().map_err(|_| ())?;
    mcp_pair[1].init_hardware().map_err(|_| ())?;
    mcp_pair[0].port_mode(Port::GPIOA, 0x01).map_err(|_| ())?;
    mcp_pair[0].port_mode(Port::GPIOB, 0x7F).map_err(|_| ())?;
    mcp_pair[1].port_mode(Port::GPIOA, 0x00).map_err(|_| ())?;
    mcp_pair[1].port_mode(Port::GPIOB, 0x7F).map_err(|_| ())?;
    Ok(())
}

// Hardware bypass recovery for Arduino Nano (ATmega328P)
fn recover_i2c_bus() {
    // Yeah, this is kinda ridiculous. Gemini told me to do it. Yikes?
    unsafe {
        let twcr = 0xBC as *mut u8; // TWI Control Register
        let portc = 0x28 as *mut u8; // PORTC Data Register
        let ddrc = 0x27 as *mut u8; // PORTC Direction Register

        // 1. Disable TWI hardware by clearing TWEN
        core::ptr::write_volatile(twcr, core::ptr::read_volatile(twcr) & !(1 << 2));

        // 2. Set PC5 (SCL pin) as Output
        core::ptr::write_volatile(ddrc, core::ptr::read_volatile(ddrc) | (1 << 5));

        // 3. 9-Pulse clock sequence to free hung slaves
        for _ in 0..9 {
            core::ptr::write_volatile(portc, core::ptr::read_volatile(portc) & !(1 << 5)); // Low
            arduino_hal::delay_us(10);
            core::ptr::write_volatile(portc, core::ptr::read_volatile(portc) | (1 << 5)); // High
            arduino_hal::delay_us(10);
        }

        // 4. Set SCL pin back to pull-up input
        core::ptr::write_volatile(ddrc, core::ptr::read_volatile(ddrc) & !(1 << 5));
        core::ptr::write_volatile(portc, core::ptr::read_volatile(portc) | (1 << 5));

        // 5. Re-enable TWI hardware
        core::ptr::write_volatile(twcr, core::ptr::read_volatile(twcr) | (1 << 2));
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut oe = pins.d5.into_output();
    let mut latch = pins.d4.into_output();
    let mut dta = pins.d3.into_output();
    let mut clk = pins.d2.into_output();

    oe.set_high(); // Disable outputs initially

    arduino_hal::delay_ms(1000);

    let i2c = arduino_hal::i2c::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100_000,
    );
    let i2c_cell = AtomicCell::new(i2c);

    let mut mcps = [
        [
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x20),
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x21),
        ],
        [
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x22),
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x23),
        ],
        [
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x24),
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x25),
        ],
        [
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x26),
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x27),
        ],
    ];
    for mcp_pair in mcps.iter_mut() {
        // Safe to unwrap here: if it fails on cold boot, it's a hard hardware error
        config_mcp_pair(mcp_pair).unwrap();
        mcp_pair[0].write_gpio(Port::GPIOA, 0xFE).unwrap();
        mcp_pair[0].write_gpio(Port::GPIOB, 0x80).unwrap();
        mcp_pair[1].write_gpio(Port::GPIOA, 0x7F).unwrap();
        mcp_pair[1].write_gpio(Port::GPIOB, 0x00).unwrap();
    }

    let mut onboard_led = pins.d13.into_output();

    let piston_data_pin = pins.d8.into_pull_up_input();
    let mut piston_clock_pin = pins.d9.into_output();
    let mut piston_load_pin = pins.d10.into_output();

    let mut ticks = 0u32;
    let mut bus_reset_retry_attempts = 0u32;

    let mut awaiting_save_preset = false;
    let mut presets = [[0u16; 4]; 8];

    let mut stop_tab_button_debouncers = [
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
    ];
    let mut stop_state: [u16; 4] = [0u16; 4];
    let mut piston_debouncer = Debouncer::<5>::new(0xFFFF);

    // Global flag to track if the bus was corrupted during this tick
    let mut bus_hang_detected = false;

    // Now that we're done with setup, enable shift register outputs.
    oe.set_low();

    loop {
        // --- Heartbeat ---
        if ticks % LED_TOGGLE_ITVL_TICKS == 0 {
            onboard_led.toggle();
        }

        // --- Stop Tabs ---
        if ticks % STOP_TAB_BUTTON_POLL_ITVL_TICKS == 0 {
            for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                match read_stop_tabs(mcp_pair) {
                    Ok(readings) => {
                        stop_tab_button_debouncers[div].update(readings);
                        stop_tab_button_debouncers[div].for_each_falling_edge(|i| {
                            stop_state[div] ^= (1 << i) as u16;

                            if write_stop_state_to_leds(&stop_state[div], mcp_pair).is_err() {
                                bus_hang_detected = true;
                            }
                        });
                    }
                    Err(_) => {
                        bus_hang_detected = true;
                    }
                }
            }
        }

        // --- Pistons ---
        if ticks % PISTON_POLL_ITVL_TICKS == 0 {
            piston_load_pin.set_low();
            piston_load_pin.set_high();

            let mut raw_piston_reading = 0u16;
            piston_clock_pin.set_low();

            for i in 0..8 {
                if piston_data_pin.is_high() {
                    raw_piston_reading |= 1 << i;
                }
                piston_clock_pin.set_high();
                piston_clock_pin.set_low();
            }
            raw_piston_reading |= 0xFF00;

            piston_debouncer.update(raw_piston_reading);

            piston_debouncer.for_each_falling_edge(|i| {
                if i == SAVE_PISTON_IDX {
                    awaiting_save_preset = true;
                } else if i == GC_PISTON_IDX {
                    for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                        stop_state[div] = 0;
                        if write_stop_state_to_leds(&stop_state[div], mcp_pair).is_err() {
                            bus_hang_detected = true;
                        }
                    }
                } else {
                    if awaiting_save_preset {
                        for div in 0..4 {
                            presets[i as usize][div] = stop_state[div];
                        }
                        awaiting_save_preset = false;
                    } else {
                        for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                            stop_state[div] = presets[i as usize][div];
                            if write_stop_state_to_leds(&stop_state[div], mcp_pair).is_err() {
                                bus_hang_detected = true;
                            }
                        }
                    }
                }
            });

            piston_debouncer.for_each_rising_edge(|i| {
                if i == SAVE_PISTON_IDX {
                    awaiting_save_preset = false;
                }
            });
        }

        // --- Shift Register Flushing ---
        // We flush the state regardless of whether or not it's fresh, because it's possible that
        // it becomes desynced from EMI generated by playing notes or by toggling stops.
        if ticks % FLUSH_STOP_STATE_ITVL_TICKS == 0 {
            let left_half = interleave_bytes(stop_state[3], stop_state[0]);
            let right_half = interleave_bytes(stop_state[2], stop_state[1]);

            for val in [right_half, left_half] {
                for i in 0..4 {
                    let mut to_write = ((val >> (8 * i)) & 0xFF) as u8;
                    to_write =
                        (to_write & 0xF0) | NIBBLE_REVERSE_LOOKUP[(to_write & 0x0F) as usize];

                    for bit in 0..8 {
                        if (to_write >> bit) & 1 == 1 {
                            dta.set_high();
                        } else {
                            dta.set_low();
                        }
                        clk.set_high();
                        // Try to counter EMI...
                        arduino_hal::delay_us(5);
                        clk.set_low();
                    }
                }
            }
            latch.set_high();
            arduino_hal::delay_us(5);
            latch.set_low();
        }

        // i2c bus recovery
        if bus_hang_detected && bus_reset_retry_attempts < MAX_BUS_RESET_RETRIES {
            // Un-stick the bus via hardware bypass
            recover_i2c_bus();

            // Loop through all chips, restore their IODIR configs, and write the current state back.
            // After a certain number of retries we will give up!
            for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                let res1 = config_mcp_pair(mcp_pair);
                let res2 = write_stop_state_to_leds(&stop_state[div], mcp_pair);
                if res1.is_ok() && res2.is_ok() {
                    bus_hang_detected = false;
                } else {
                    bus_reset_retry_attempts += 1;
                }
            }
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
