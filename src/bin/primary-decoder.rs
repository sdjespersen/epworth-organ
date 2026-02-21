#![no_std]
#![no_main]

use embassy_sync::signal::Signal;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Level, Output};
use embassy_rp::peripherals::{SPI0, UART0};
use embassy_rp::spi;
use embassy_rp::{Peri, bind_interrupts, uart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use epworth_organ::decoder::Decoder;
use epworth_organ::event::Event;
use epworth_organ::uart::UartReader;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
});

const B: [u32; 4] = [0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF];
const S: [u32; 4] = [1, 2, 4, 8];
const NIBBLE_REVERSE_LOOKUP: [u8; 16] = [
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
];

// Queue for all events on the decoder. The only way events make it onto the decoder is over UART.
static EVENTS: Channel<CriticalSectionRawMutex, Event, 8> = Channel::new();
static STOP_STATE: Signal<CriticalSectionRawMutex, u64> = Signal::new();

fn interleave_bits(mut x: u32, mut y: u32) -> u32 {
    for i in (0..4).rev() {
        x = (x | (x << S[i])) & B[i];
        y = (y | (y << S[i])) & B[i];
    }
    x | (y << 1)
}

#[embassy_executor::task]
async fn write_stop_state(
    mut spi: spi::Spi<'static, SPI0, spi::Async>,
    latch_pin: Peri<'static, AnyPin>,
) {
    let mut stops_latch_pin = Output::new(latch_pin, Level::Low);

    loop {
        let mut stop_state: u64 = STOP_STATE.wait().await;

        // This is our approach to batching lots of little changes: Wait for a brief interval before reading the stop
        // states again.
        Timer::after_millis(1).await;

        stop_state = STOP_STATE.try_take().unwrap_or(stop_state);

        // There's a lot going on the in the next few paragraphs.
        //
        // First off, the stop state saved in this program is ordered logically in terms of manuals: swell, great,
        // choir, and finally pedal. However, the way the stops are electrically wired in the combination action box is
        // completely different. From organist's left to right, it goes swell, pedal, swell, pedal, ... until the swell
        // and pedal stops are exhausted, and then it picks up with great, choir, great, choir, etc. This means that
        // we need to "interleave" the bits of the swell stop state with the bits of the pedal stop state, and same for
        // the great and the choir.
        //
        // Second, the wiring of the outputs on the MIC5891YN line drivers necessitates a further reshuffling. Because
        // of the pinout on the chip, we need to remap the bit positions in each byte as follows:
        // 0 -> 4, 1 -> 5, 2 -> 6, 3 -> 7, 4 -> 3, 5 -> 2, 6 -> 1, 7 -> 0
        // Reversing the high nibble and then swapping high and low nibbles accomplishes this.
        let left_half = interleave_bits(
            (stop_state >> 16 & 0xFFFF) as u32,
            (stop_state >> 32 & 0xFFFF) as u32,
        );
        let right_half = interleave_bits(
            (stop_state >> 48 & 0xFFFF) as u32,
            (stop_state & 0xFFFF) as u32,
        );

        let mut buf: [u8; 8] = ((left_half as u64) << 32 | (right_half as u64)).to_le_bytes();
        for i in 0..buf.len() {
            // Reverse high nibble, then swap nibbles
            buf[i] = ((buf[i] & 0x0F) << 4) | NIBBLE_REVERSE_LOOKUP[((buf[i] & 0xF0) >> 4) as usize];
        }
        let _ = spi.write(&buf).await;

        stops_latch_pin.set_high();
        stops_latch_pin.set_low();
    }
}

#[embassy_executor::task]
async fn uart_reader(uart_rx: uart::BufferedUartRx) {
    let mut uart_rdr = UartReader::new(uart_rx);

    loop {
        uart_rdr
            .feed::<Event>(async |event| {
                EVENTS.send(event).await;
            })
            .await;
    }
}

#[embassy_executor::task]
async fn main_event_handler() {
    let mut decoder = Decoder::new();

    // Signal the initial stop state.
    STOP_STATE.signal(decoder.stop_state());

    loop {
        let event = EVENTS.receive().await;
        let result = decoder.handle(event);
        if let Some(stop_state) = result.stop_state {
            STOP_STATE.signal(stop_state);
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
        p.PIN_0,
        p.PIN_1,
        Irqs,
        tx_buf,
        rx_buf,
        uart::Config::default(),
    );
    let (_uart_tx, uart_rx) = uart_txrx.split();
    spawner.spawn(uart_reader(uart_rx)).unwrap();
    // spawner.spawn(uart_writer(uart_tx)).unwrap();

    let spi = spi::Spi::new_txonly(
        p.SPI0,
        p.PIN_18,
        p.PIN_19,
        p.DMA_CH0,
        spi::Config::default(),
    );
    spawner
        .spawn(write_stop_state(spi, p.PIN_20.into()))
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
