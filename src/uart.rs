use embedded_io_async::Read;
use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde::Deserialize;

pub struct UartReader<R: Read> {
    uart_rx: R,
    raw_buf: [u8; 32],
    cobs_acc: CobsAccumulator<128>,
}

impl<R: Read> UartReader<R> {
    pub fn new(uart_rx: R) -> Self {
        Self {
            uart_rx: uart_rx,
            raw_buf: [0u8; 32],
            cobs_acc: CobsAccumulator::new(),
        }
    }

    pub async fn feed<T>(&mut self, mut handler: impl AsyncFnMut(T))
    where
        T: for<'de> Deserialize<'de>,
    {
        match self.uart_rx.read(&mut self.raw_buf).await {
            Ok(n) if n > 0 => {
                let buf = &self.raw_buf[..n];
                let mut window = &buf[..];

                'cobs: while !window.is_empty() {
                    window = match self.cobs_acc.feed::<T>(&window) {
                        FeedResult::Consumed => break 'cobs,
                        FeedResult::OverFull(new_window) => new_window,
                        FeedResult::DeserError(new_window) => new_window,
                        FeedResult::Success { data, remaining } => {
                            handler(data).await;
                            remaining
                        }
                    }
                }
            }
            Err(_) => {
                defmt::error!("Error reading from UART");
            }
            Ok(_) => {}
        }
    }
}
