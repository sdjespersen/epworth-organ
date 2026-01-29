#![no_std]
#![no_main]

mod debouncer;
mod mcp23017;

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, Ordering};

use debouncer::Debouncer;
use defmt::warn;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::select::select4;
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{
    Peri,
    gpio::{AnyPin, Input, Level, Output},
    i2c::Config,
};
use embassy_rp::{bind_interrupts, i2c};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, with_timeout};
use embassy_usb::class::midi::MidiClass;
use embassy_usb::{Builder, Config as UsbConfig};
use panic_halt as _;
use static_cell::{ConstStaticCell, StaticCell};

use crate::mcp23017::MCP23017;
use crate::mcp23017::Port;
use midi_types::{Channel as MidiChannel, Control, MidiMessage, Program, Value7};

type I2c0Bus = Mutex<NoopRawMutex, RefCell<i2c::I2c<'static, I2C0, i2c::Blocking>>>;
type I2c0Mcp = MCP23017<I2cDevice<'static, NoopRawMutex, i2c::I2c<'static, I2C0, i2c::Blocking>>>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

const GC_MESSAGE: MidiMessage = MidiMessage::ProgramChange(
    MidiChannel::new(4), // should this be 5?
    Program::new(0),
);

macro_rules! atomicu16_array {
    ($val:expr) => {{
        [
            AtomicU16::new($val),
            AtomicU16::new($val),
            AtomicU16::new($val),
            AtomicU16::new($val),
        ]
    }};
}

static STOP_STATE: [AtomicU16; 4] = atomicu16_array!(0);
static STOP_STATE_CHANGED: [Signal<CriticalSectionRawMutex, bool>; 4] =
    [Signal::new(), Signal::new(), Signal::new(), Signal::new()];
// TODO: This should be persistent somehow
static PRESETS: [[AtomicU16; 4]; 8] = [
    atomicu16_array!(0),
    atomicu16_array!(0),
    atomicu16_array!(0),
    atomicu16_array!(0),
    atomicu16_array!(0),
    atomicu16_array!(0),
    atomicu16_array!(0),
    atomicu16_array!(0),
];
static OUTBOUND_MIDI_EVENT_BUS: Channel<CriticalSectionRawMutex, MidiMessage, 16> = Channel::new();
static INTERNAL_MIDI_EVENT_BUS: Channel<CriticalSectionRawMutex, MidiMessage, 16> = Channel::new();

static USB_CONFIG_DESCRIPTOR: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static USB_BOS_DESCRIPTOR: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static USB_CONTROL_BUF: ConstStaticCell<[u8; 64]> = ConstStaticCell::new([0; 64]);

fn reverse_byte(a: &u8) -> u8 {
    let mut b = *a;
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    b
}

fn handle_control_change(channel: u8, control_number: u8, value: u8) {
    let i = 117 - control_number;
    if value > 63 {
        STOP_STATE[channel as usize].fetch_or(1 << i, Ordering::SeqCst);
    } else {
        STOP_STATE[channel as usize].fetch_and(!(1 << i), Ordering::SeqCst);
    }
    STOP_STATE_CHANGED[channel as usize].signal(true);
}

fn handle_program_change(value: u8) {
    for div in 0..4 {
        let div_preset = PRESETS[value as usize][div].load(Ordering::SeqCst);
        STOP_STATE[div].store(div_preset, Ordering::SeqCst);
        STOP_STATE_CHANGED[div].signal(true);
    }
}

fn get_mcps(i2c_bus: &'static I2c0Bus) -> [[I2c0Mcp; 2]; 4] {
    [[0x20, 0x21], [0x22, 0x23], [0x24, 0x25], [0x26, 0x27]].map(|[addr0, addr1]| {
        [
            MCP23017::new(I2cDevice::new(i2c_bus), addr0),
            MCP23017::new(I2cDevice::new(i2c_bus), addr1),
        ]
    })
}

async fn enqueue_midi_out_and_in(message: MidiMessage) {
    OUTBOUND_MIDI_EVENT_BUS.sender().send(message).await;
    INTERNAL_MIDI_EVENT_BUS.sender().send(message).await;
}

#[embassy_executor::task]
async fn midi_event_listener() {
    loop {
        let event = INTERNAL_MIDI_EVENT_BUS.receive().await;
        match event {
            MidiMessage::ProgramChange(_channel, prog) => {
                handle_program_change(prog.into());
            }
            MidiMessage::ControlChange(channel, cc_number, cc_value) => {
                handle_control_change(channel.into(), cc_number.into(), cc_value.into());
            }
            _ => {} // unhandled for now
        }
    }
}

#[embassy_executor::task]
async fn flush_stop_state_to_leds(i2c_bus: &'static I2c0Bus) {
    let mcps = &mut get_mcps(i2c_bus);

    loop {
        // TODO: Think of a more efficient way to indicate which divisions need to be flushed.
        let stop_state_change = select4(
            STOP_STATE_CHANGED[0].wait(),
            STOP_STATE_CHANGED[1].wait(),
            STOP_STATE_CHANGED[2].wait(),
            STOP_STATE_CHANGED[3].wait(),
        )
        .await;
        let idx: usize = {
            if stop_state_change.is_first() {
                0
            } else if stop_state_change.is_second() {
                1
            } else if stop_state_change.is_third() {
                2
            } else {
                3
            }
        };

        let div_stop_state = STOP_STATE[idx].load(Ordering::SeqCst);
        let left_half = reverse_byte(&(!(div_stop_state >> 8) as u8));
        let right_half = reverse_byte(&(!(div_stop_state & 0xFF) as u8));
        mcps[idx][0]
            .write_gpio(Port::GPIOA, (left_half & 0x7F) << 1)
            .unwrap();
        mcps[idx][0]
            .write_gpio(Port::GPIOB, left_half & 0x80)
            .unwrap();
        mcps[idx][1]
            .write_gpio(Port::GPIOA, right_half & 0x7F)
            .unwrap();

        // This throttles updates a bit, so there might be mild lag when recalling presets (>= 4x this delay).
        Timer::after_micros(200).await;
    }
}

#[embassy_executor::task]
async fn scan_stop_tab_buttons(i2c_bus: &'static I2c0Bus) {
    let mut mcps = get_mcps(i2c_bus);
    let mut stop_tab_button_debouncers: [Debouncer<5>; 4] = [
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
        Debouncer::<5>::new(0xFFFF),
    ];

    loop {
        for (div, mcp_pair) in mcps.iter_mut().enumerate() {
            let mut raw_reading = 0x01u16; // LSB always 1, by convention
            raw_reading |= ((mcp_pair[0].read_gpio(Port::GPIOA).unwrap() & 0x01) as u16) << 8;
            raw_reading |= ((mcp_pair[0].read_gpio(Port::GPIOB).unwrap() & 0x7F) as u16) << 9;
            raw_reading |= ((mcp_pair[1].read_gpio(Port::GPIOB).unwrap() & 0x7F) as u16) << 1;
            stop_tab_button_debouncers[div].update(raw_reading);

            // This iterator is almost always going to be size 0 or 1, so acquiring the mutex in the loop is fine.
            for i in stop_tab_button_debouncers[div].falling_edges() {
                let div_stop_state = STOP_STATE[div].load(Ordering::SeqCst);
                let mut midi_val = 0u8;
                if div_stop_state >> i & 1 == 0 {
                    midi_val = 127; // currently off, toggling to on
                }
                enqueue_midi_out_and_in(MidiMessage::ControlChange(
                    MidiChannel::from(div as u8),
                    Control::from(117 - i),
                    Value7::from(midi_val),
                ))
                .await;
            }
        }
        Timer::after_micros(200).await;
    }
}

#[embassy_executor::task]
async fn scan_pistons(
    load_pin: Peri<'static, AnyPin>,
    clock_pin: Peri<'static, AnyPin>,
    data_pin: Peri<'static, AnyPin>,
) {
    let mut piston_debouncer = Debouncer::<5>::new(0xFFFF);
    // TODO: Re-implement the save button using MIDI SysEx, so that it can live on a separate MCU and doesn't require
    // special handling.
    let mut awaiting_save = false;

    // Load piston states into 74HC165 shift register
    let mut piston_load_pin = Output::new(load_pin, Level::Low);
    let mut piston_clock_pin = Output::new(clock_pin, Level::Low);
    let piston_data_pin = Input::new(data_pin, Pull::Up);

    loop {
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

        for i in piston_debouncer.falling_edges() {
            if i == 0 {
                // SAVE piston
                awaiting_save = true;
            } else if i == 1 {
                // GC piston, implemented as a "fixed preset" with all 0s
                enqueue_midi_out_and_in(GC_MESSAGE).await;
            } else {
                if awaiting_save {
                    // write preset
                    for div in 0..4 {
                        PRESETS[i as usize - 1][div]
                            .store(STOP_STATE[div].load(Ordering::SeqCst), Ordering::SeqCst);
                    }
                } else {
                    // recall preset
                    enqueue_midi_out_and_in(MidiMessage::ProgramChange(
                        MidiChannel::from(4), // should this be 5? am i off by one?
                        Program::from(i - 1),
                    ))
                    .await;
                }
            }
        }

        for i in piston_debouncer.rising_edges() {
            if i == 0 {
                awaiting_save = false;
            }
        }

        Timer::after_micros(200).await;
    }
}

#[embassy_executor::task]
async fn run_usb_device(mut usb_device: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb_device.run().await;
}

#[embassy_executor::task]
async fn usb_midi_driver(spawner: Spawner, usb: Peri<'static, USB>) {
    let driver = Driver::new(usb, Irqs);
    let mut config = UsbConfig::new(0xc0de, 0xcafe);
    config.manufacturer = Some("RP Pico 2");
    config.product = Some("Epworth Organ MIDI");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        config,
        USB_CONFIG_DESCRIPTOR.take(),
        USB_BOS_DESCRIPTOR.take(),
        &mut [], // no msos descriptors
        USB_CONTROL_BUF.take(),
    );

    let mut midi_class = MidiClass::new(&mut builder, 1, 1, 64);

    // Build the builder.
    let usb_device = builder.build();

    // Run the USB device forever, in a separate task.
    spawner.spawn(run_usb_device(usb_device)).unwrap();

    midi_class.wait_connection().await;

    loop {
        let message = OUTBOUND_MIDI_EVENT_BUS.receive().await;

        match message {
            MidiMessage::ControlChange(channel, cc_number, cc_value) => {
                let cc_packet = [
                    0x0B,
                    u8::from(channel) + 0xB0,
                    cc_number.into(),
                    cc_value.into(),
                ];
                // Try to send the packet, but give up after 2ms
                // This prevents blocking if the USB host is stalled or disconnected
                let result = with_timeout(
                    Duration::from_millis(2),
                    midi_class.write_packet(&cc_packet),
                )
                .await;

                match result {
                    Ok(Ok(())) => {
                        // Success: Packet sent
                    }
                    Ok(Err(e)) => {
                        // USB Error: Cable unplugged or bus reset?
                        // Log it, but don't panic so the loop keeps running
                        warn!("USB Write Error: {:?}", e);
                    }
                    Err(_) => {
                        // Timeout: The host didn't pick up the data in time.
                        // We drop the packet to keep the system responsive.
                        warn!("USB Write Timeout (Dropping packet)");
                    }
                }
            }
            // MidiMessage::ProgramChange => {},
            _ => {}
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // i2c and MCP23017 setup
    let i2c = embassy_rp::i2c::I2c::new_blocking(p.I2C0, p.PIN_5, p.PIN_4, Config::default());
    static I2C_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c.into()));

    for mcp_pair in get_mcps(i2c_bus).iter_mut() {
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

    // Finally, spawn all tasks.
    spawner
        .spawn(scan_pistons(p.PIN_2.into(), p.PIN_6.into(), p.PIN_3.into()))
        .unwrap();
    spawner.spawn(scan_stop_tab_buttons(i2c_bus)).unwrap();
    spawner.spawn(flush_stop_state_to_leds(i2c_bus)).unwrap();
    spawner.spawn(midi_event_listener()).unwrap();
    spawner.spawn(usb_midi_driver(spawner, p.USB)).unwrap();

    // Heartbeat task comes last, and happens on the main loop, roughly indicating that all tasks spawned successfully.
    let mut led = Output::new(p.PIN_25, Level::Low);

    loop {
        led.toggle();
        Timer::after_secs(1).await;
    }
}
