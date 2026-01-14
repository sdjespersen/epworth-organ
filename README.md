# Epworth Organ

Encoder (`encoder-avr`), decoder (`decoder-avr`), etc. for the Epworth organ, targeting the Arduino Nano, written in
Rust. Encoder and decoder compile to an ELF that can be flashed straight onto the chip.

## Setup

There were some tooling setup steps required to get Rust working for these particular targets. The
[Rust on AVR book](https://book.avr-rust.org/) was helpful here. I'll update as i run into
them again; for now i'm working from memory.

You'll need compiler support for AVR (i'm on a Mac):

```
$ xcode-select --install
$ brew tap osx-cross/avr
$ brew install avr-gcc
```

You'll need nightly `cargo`:

```
$ rustup toolchain install nightly
$ rustup override set nightly
```

You'll also need the utility `avrdude` to flash the built executable (`ELF`):

```
$ brew install avrdude
```

The version that lives inside Arduino's install didn't work for me.

Also, don't mess with the JSON in `avr-specs`!

## Build + Flash to MCU

When inside the `encoder-avr` or `decoder-avr` directory, your typical `cargo build` or `cargo run` commands will work
(optionally targeting the `--release` profile). `cargo run` will build and also flash to the MCU using `avrdude`.

You may have to change the USB port depending on where it gets mapped on your
machine, `/dev/cu.usbserial-XXXX` or whatever; this setting is found in `.cargo/config.toml`.

## Gotchas

When uploading, you must not have the TX/RX lines on the microcontroller hooked up! Additionally, you may not have any
serial monitors connected to the MCU you are flashing.
