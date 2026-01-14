#![no_std]
#![no_main]

mod debouncer;
mod mcp23017;

use embedded_hal::i2c::I2c;
use embedded_hal_bus::util::AtomicCell;
use embedded_midi::{Channel, Control, MidiMessage, MidiOut};
use midi_types::Value7;
use panic_halt as _;

use debouncer::Debouncer;

use embedded_hal_bus::i2c::AtomicDevice;
use mcp23017::{MCP23017, Port};

const STOP_TAB_BUTTON_POLL_ITVL_TICKS: u16 = 5;
const PISTON_POLL_ITVL_TICKS: u16 = 5;
const CRESCENDO_READ_ITVL_TICKS: u16 = 5;
const LED_TOGGLE_ITVL_TICKS: u16 = 2_000;

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

fn stop_state_cc_message(div: usize, div_stop_state: u16, i: u8) -> MidiMessage {
    let mut control_value = 0u8;
    if (div_stop_state >> i) & 1 != 0 {
        control_value = 127
    };

    // This is part of the MIDI contract that needs to be documented: Stop tabs use the undefined
    // MIDI CCs 102-116 (inclusive), with channel equal to 1-indexed division, value 127 for ON, 0
    // for OFF.
    MidiMessage::ControlChange(
        Channel::from(div as u8),
        Control::from(117 - i),
        Value7::from(control_value),
    )
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 31250);
    let (_rx, tx) = serial.split();
    let mut midi_out = MidiOut::new(tx);

    let i2c = arduino_hal::i2c::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100_000,
    );
    let i2c_cell = AtomicCell::new(i2c);

    let mut mcps = [
        [
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x22),
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x23),
        ],
        [
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x26),
            MCP23017::new(AtomicDevice::new(&i2c_cell), 0x27),
        ],
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

    let mut piston_load_pin = pins.d10.into_output();
    let piston_data_pin = pins.d9.into_pull_up_input();
    let mut piston_clock_pin = pins.d8.into_output();

    // Set up ADC and the potentiometer wiper on pin A5
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let crescendo_pin = pins.a3.into_analog_input(&mut adc);

    // TODO: Factor out all the tickers and their values into some sort of scheduler struct?
    let mut since_last_stop_tab_read_ticks = 0u16;
    let mut since_last_crescendo_read_ticks = 0u16;
    let mut since_last_led_toggle_ticks = 0u16;
    let mut since_last_piston_read_ticks = 0u16;

    let mut smoothed_cresc_value = 0u16;
    let mut last_cresc_control_value = 0u8;
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

    loop {
        // Heartbeat indicator
        if since_last_led_toggle_ticks > LED_TOGGLE_ITVL_TICKS {
            since_last_led_toggle_ticks = 0;
            onboard_led.toggle();
        }

        // Stop tabs
        if since_last_stop_tab_read_ticks > STOP_TAB_BUTTON_POLL_ITVL_TICKS {
            since_last_stop_tab_read_ticks = 0;

            // Stop tab buttons for each division are logically separate, so this looks like 4 scans.
            for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                stop_tab_button_debouncers[div].update(read_stop_tabs(mcp_pair));
                stop_tab_button_debouncers[div].for_each_falling_edge(|i| {
                    stop_state[div] ^= (1 << i) as u16;

                    write_stop_state_to_leds(&stop_state[div], mcp_pair);

                    midi_out
                        .write(&stop_state_cc_message(div, stop_state[div], i))
                        .unwrap();
                });
            }
        }

        // Pistons
        if since_last_piston_read_ticks > PISTON_POLL_ITVL_TICKS {
            since_last_piston_read_ticks = 0;

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
                if i == 0 {
                    // SAVE piston
                    awaiting_save_preset = true;
                } else if i == 1 {
                    // GC piston
                    for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                        stop_state[div] = 0;
                        write_stop_state_to_leds(&stop_state[div], mcp_pair);
                        for j in 0..16 {
                            midi_out
                                .write(&stop_state_cc_message(div, 0u16, j))
                                .unwrap();
                        }
                    }
                } else {
                    if awaiting_save_preset {
                        // write preset
                        for div in 0..4 {
                            presets[i as usize][div] = stop_state[div];
                        }
                    } else {
                        // recall preset
                        for (div, mcp_pair) in mcps.iter_mut().enumerate() {
                            stop_state[div] = presets[i as usize][div];
                            write_stop_state_to_leds(&stop_state[div], mcp_pair);
                            for j in 0..15 {
                                midi_out
                                    .write(&stop_state_cc_message(div, stop_state[div], j))
                                    .unwrap();
                            }
                        }
                    }
                }
            });

            piston_debouncer.for_each_rising_edge(|i| {
                if i == 0 {
                    awaiting_save_preset = false;
                }
            });
        }

        // Crescendo pedal
        if since_last_crescendo_read_ticks > CRESCENDO_READ_ITVL_TICKS {
            since_last_crescendo_read_ticks = 0;
            let crescendo_reading = crescendo_pin.analog_read(&mut adc) as u16;

            smoothed_cresc_value = (smoothed_cresc_value * 3 + crescendo_reading) / 4;

            // Should be in [0, 127] since reading is [0, 1023]
            let control_value = ((smoothed_cresc_value / 8) as u8) & 0x7f;

            if control_value != last_cresc_control_value {
                let cc_msg = MidiMessage::ControlChange(
                    Channel::from(4),  // channel 5 ("whole organ")
                    Control::from(11), // expression
                    Value7::from(control_value),
                );
                midi_out.write(&cc_msg).unwrap();

                last_cresc_control_value = control_value;
            }
        }

        // I'd love to use async rust, but i'm not really sure how to do that on AVR yet. In the meantime, we'll just
        // keep a tick counter without regard to real time. (I note that the implementation of delay_us and similar is
        // really just a busy loop of machine instructions anyway, so this is not too different in practice.)
        since_last_led_toggle_ticks += 1;
        since_last_stop_tab_read_ticks += 1;
        since_last_piston_read_ticks += 1;
        since_last_crescendo_read_ticks += 1;

        // This delay throttles the pin polling a bit, and allows us to roughly approximate the interval of time at
        // which we scan things.
        arduino_hal::delay_us(10);
    }
}
