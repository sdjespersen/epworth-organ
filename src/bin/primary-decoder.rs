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
use epworth_organ::event::Event;

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::InterruptHandler<UART0>;
});

const B: [u32; 4] = [0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF];
const S: [u32; 4] = [1, 2, 4, 8];
const NIBBLE_REVERSE_LOOKUP: [u8; 16] = [
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
];

// Queue for all events on the decoder. The only way events make it onto the decoder is over UART.
static EVENT_BUS: Channel<CriticalSectionRawMutex, Event, 128> = Channel::new();
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
async fn write_stop_state_to_stops(
    data_pin: Peri<'static, AnyPin>,
    clock_pin: Peri<'static, AnyPin>,
    latch_pin: Peri<'static, AnyPin>,
    oe_pin: Peri<'static, AnyPin>,
) {
    let mut stops_data_pin = Output::new(data_pin, Level::Low);
    let mut stops_clock_pin = Output::new(clock_pin, Level::Low);
    let mut stops_latch_pin = Output::new(latch_pin, Level::Low);
    let mut stops_oe_pin = Output::new(oe_pin, Level::High);

    stops_oe_pin.set_low();

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
async fn uart_reader(_uart_rx: uart::UartRx<'static, uart::Async>) {
    loop {
        // All events on UART are proprietary 2-byte messages, as defined in the Event struct.
        // FIXME: This is no longer true. We are going to have some bigger ones (e.g. StopStateChange)
        // let mut buf: [u8; 2] = [0; 2];

        // match uart_rx.read(&mut buf).await {
        //     Ok(_) => {
        //         if let Some(e) = Event::parse(&buf) {
        //             EVENT_BUS.send(e).await;
        //         }
        //     }
        //     Err(err) => {
        //         defmt::warn!("Error reading from UART: {:?}", err);
        //     }
        // }
    }
}

#[embassy_executor::task]
async fn main_event_handler() {
    loop {
        let event = EVENT_BUS.receive().await;
        match event {
            Event::GeneralCancel() => {
                // TODO: All notes off
            }
            // TODO: NoteOff, NoteOn, PresetRecalled, Crescendo, Expression
            _ => {}
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner.spawn(main_event_handler()).unwrap();

    let uart_txrx = uart::Uart::new(
        p.UART0,
        p.PIN_12,
        p.PIN_13,
        Irqs,
        p.DMA_CH0,
        p.DMA_CH1,
        uart::Config::default(),
    );
    let (_, uart_rx) = uart_txrx.split();
    spawner.spawn(uart_reader(uart_rx)).unwrap();

    spawner
        .spawn(write_stop_state_to_stops(
            p.PIN_21.into(),
            p.PIN_19.into(),
            p.PIN_20.into(),
            p.PIN_18.into(),
        ))
        .unwrap();

    // Heartbeat task comes last, and happens on the main loop, roughly indicating that all tasks spawned successfully.
    let mut led = Output::new(p.PIN_25, Level::Low);

    loop {
        led.toggle();
        Timer::after_secs(1).await;
    }
}
