#![no_std]
#![no_main]

use embassy_sync::signal::Signal;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Level, Output};
use embassy_rp::peripherals::UART0;
use embassy_rp::{Peri, bind_interrupts, uart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embedded_io_async::Read;
use epworth_organ::event::Event;
use postcard::accumulator::{CobsAccumulator, FeedResult};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
});

const B: [u32; 4] = [0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF];
const S: [u32; 4] = [1, 2, 4, 8];
const NIBBLE_REVERSE_LOOKUP: [u8; 16] = [
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
];

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

// Queue for all events on the decoder. The only way events make it onto the decoder is over UART.
static EVENTS: Channel<CriticalSectionRawMutex, Event, 8> = Channel::new();
static STOP_STATE: Signal<CriticalSectionRawMutex, u64> = Signal::new();

fn interleave_bits(a: u16, b: u16) -> u32 {
    let mut x = a as u32;
    let mut y = b as u32;

    for i in (0..4).rev() {
        x = (x | (x << S[i])) & B[i];
        y = (y | (y << S[i])) & B[i];
    }
    x | (y << 1)
}

#[embassy_executor::task]
async fn write_stop_state(
    data_pin: Peri<'static, AnyPin>,
    clock_pin: Peri<'static, AnyPin>,
    latch_pin: Peri<'static, AnyPin>,
) {
    let mut stops_data_pin = Output::new(data_pin, Level::Low);
    let mut stops_clock_pin = Output::new(clock_pin, Level::Low);
    let mut stops_latch_pin = Output::new(latch_pin, Level::Low);

    loop {
        let mut stop_state: u64 = STOP_STATE.wait().await;

        // This is our approach to batching lots of little changes: Wait for a brief interval before reading the stop
        // states again.
        Timer::after_millis(1).await;

        stop_state = STOP_STATE.try_take().unwrap_or(stop_state);

        let left_half = interleave_bits(
            (stop_state >> 16 & 0xFFFF) as u16,
            (stop_state >> 32 & 0xFFFF) as u16,
        );
        let right_half = interleave_bits(
            (stop_state >> 48 & 0xFFFF) as u16,
            (stop_state & 0xFFFF) as u16,
        );

        for val in [right_half, left_half] {
            for i in 0..4 {
                let mut to_write = ((val >> (8 * i)) & 0xFF) as u8;
                // Reverse lower nibble
                to_write = (to_write & 0xF0) | NIBBLE_REVERSE_LOOKUP[(to_write & 0x0F) as usize];

                // Manual ShiftOut
                for bit in 0..8 {
                    if (to_write >> bit) & 1 == 1 {
                        stops_data_pin.set_high();
                    } else {
                        stops_data_pin.set_low();
                    }
                    stops_clock_pin.set_high();
                    stops_clock_pin.set_low();
                }
            }
        }

        stops_latch_pin.set_high();
        stops_latch_pin.set_low();
    }
}

#[embassy_executor::task]
async fn uart_reader(mut uart_rx: uart::BufferedUartRx) {
    let mut raw_buf = [0u8; 32];
    let mut cobs_acc: CobsAccumulator<128> = CobsAccumulator::new();

    loop {
        match uart_rx.read(&mut raw_buf).await {
            Ok(n) if n > 0 => {
                let buf = &raw_buf[..n];
                let mut window = &buf[..];

                'cobs: while !window.is_empty() {
                    window = match cobs_acc.feed::<Event>(&window) {
                        FeedResult::Consumed => break 'cobs,
                        FeedResult::OverFull(new_wind) => new_wind,
                        FeedResult::DeserError(new_wind) => new_wind,
                        FeedResult::Success { data, remaining } => {
                            EVENTS.send(data).await;
                            remaining
                        }
                    }
                }
            }
            Err(e) => {
                defmt::error!("Error reading from UART: {:?}", e);
                // Handle errors or 0-byte reads (disconnects)
            }
            Ok(_) => {}
        }
    }
}

#[embassy_executor::task]
async fn main_event_handler() {
    let mut stop_state: u64 = 0u64;
    let mut crescendo_induced_stop_state: u64 = 0u64;

    // Signal the initial stop state.
    STOP_STATE.signal(stop_state);

    loop {
        let event = EVENTS.receive().await;
        match event {
            Event::NoteOff(div, value) => {
                defmt::info!("Received NoteOff div {:?} value {:?}", div as u8, value);
            }
            Event::NoteOn(div, value) => {
                defmt::info!("Received NoteOn div {:?} value {:?}", div as u8, value);
            }
            Event::StopOff(div, idx) => {
                stop_state &= !(1 << 16 * div as u8 + idx);
                STOP_STATE.signal(stop_state | crescendo_induced_stop_state);
            }
            Event::StopOn(div, idx) => {
                stop_state |= 1 << 16 * div as u8 + idx;
                STOP_STATE.signal(stop_state | crescendo_induced_stop_state);
            }
            Event::PresetRecalled(_, new_stop_state) => {
                stop_state = new_stop_state;
                STOP_STATE.signal(stop_state | crescendo_induced_stop_state);
            }
            Event::GeneralCancel() => {
                stop_state = 0;
                STOP_STATE.signal(crescendo_induced_stop_state);
            }
            Event::Expression(div, value) => {
                defmt::info!("Received Expression div {:?} value {:?}", div as u8, value);
            }
            Event::Crescendo(value) => {
                // By contract, value is guaranteed 0-127; map it down to 0-38.
                // This is a little hacky, but i don't have stdlib for round/floor/ceil.
                let raw_mapped = (value as u16 * 3 / 10) as usize;
                crescendo_induced_stop_state = CRESCENDO_INDUCED_STATES[raw_mapped];
                STOP_STATE.signal(stop_state | crescendo_induced_stop_state);
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Disable stops output until writer task is ready.
    let mut stops_oe_pin = Output::new(p.PIN_21, Level::High);

    spawner.spawn(main_event_handler()).unwrap();

    static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 16])[..];
    static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 16])[..];
    let uart_txrx = uart::BufferedUart::new(
        p.UART0,
        p.PIN_16,
        p.PIN_17,
        Irqs,
        tx_buf,
        rx_buf,
        uart::Config::default(),
    );
    let (_uart_tx, uart_rx) = uart_txrx.split();
    spawner.spawn(uart_reader(uart_rx)).unwrap();
    // spawner.spawn(uart_writer(uart_tx)).unwrap();

    spawner
        .spawn(write_stop_state(
            p.PIN_18.into(),
            p.PIN_20.into(),
            p.PIN_19.into(),
        ))
        .unwrap();
    // Enable stops output now that the writer task is ready.
    stops_oe_pin.set_low();

    // Heartbeat task comes last, and happens on the main loop, roughly indicating that all tasks spawned successfully.
    let mut led = Output::new(p.PIN_25, Level::Low);

    loop {
        led.toggle();
        Timer::after_secs(1).await;
    }
}
