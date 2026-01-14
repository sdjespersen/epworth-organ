# Encoder: Rust on AVR (Atmega328p)

This directory contains a functional partial MIDI encoder for the Epworth organ, targeting the Arduino Nano, written in Rust. It compiles to an ELF that can be flashed straight onto the chip.

## Setup

There were some tooling setup steps required here. I'll update as i run into them again; for now i'm working from memory.

Here are some necessary steps/deps:
- Run `rustup default nightly` so that `cargo` is using the nightly release
- `avrdude`. I would recommend `brew install avrdude`; using the version that lives inside Arduino's install may not work (it didn't for me).
- The JSON in `avr-specs` is key.

## Build + Flash to MCU

```
cargo build --release && ./flashtoboard.sh
```

You may have to change the port depending on where it gets mapped on your machine, `/dev/cu.usbserial-XXXX` or whatever. I don't love having a build script like this. I did try to get `ravedude` working so that `cargo run` would automatically take care of the flashing/bootloading, but i didn't succeed, hence the build script.

## Gotchas

When uploading, you must not have the TX/RX lines on the microcontroller hooked up! Additionally, you may not have any serial monitors connected to the MCU you are flashing.
