# Epworth Organ

Encoder (`encoder-avr`), decoder (`decoder-avr`), etc. for the Epworth organ, targeting the Arduino Nano, written in
Rust. Encoder and decoder compile to an ELF that can be flashed straight onto the chip.

## Setup

There were some tooling setup steps required to get Rust working for these particular targets. I'll update as i run into
them again; for now i'm working from memory.

Some necessary steps/deps:
- Run `rustup default nightly` so that `cargo` is using the nightly release
- `brew install avrdude` (the version that lives inside Arduino's install didn't work for me)
- Don't mess with the JSON in `avr-specs`!

## Build + Flash to MCU

When inside the `encoder-avr` or `decoder-avr` directory:

```
cargo build --release && ./flashtoboard.sh
```

You may have to change the port depending on where it gets mapped on your machine, `/dev/cu.usbserial-XXXX` or whatever.
I don't love having a build script like this. I did try to get `ravedude` working so that `cargo run` would
automatically take care of the flashing/bootloading, but i didn't succeed, hence the build script.

## Gotchas

When uploading, you must not have the TX/RX lines on the microcontroller hooked up! Additionally, you may not have any
serial monitors connected to the MCU you are flashing.
