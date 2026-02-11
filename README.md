# Epworth Organ

Encoders (`main` being the primary), decoders (`primary-decoder` being the primary), etc. for the Epworth organ,  targeting the [Raspberry Pi Pico 2](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf), written in Rust.

## Setup

The best dev setup for this involves using a [Pico debug probe](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html) and [probe-rs](https://probe.rs). This allows you to get a SWD debugger and UART output, as well as flashing the program to the chip without holding BOOTSEL while plugging in the USB. (Trust me, it gets old!)
