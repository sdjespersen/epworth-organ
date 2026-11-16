// Note: This is heavily modified from https://github.com/circuitry-maker/mcp23017.
// I needed a different API than was offered, and there were some conflicts with the latest embedded-hal crate (1.0.0).

//! Manages an MCP23017, a 16-Bit I2C I/O Expander with Serial Interface module.
//!
//! This operates the chip in `IOCON.BANK=0` mode, i.e. the registers are mapped sequentially.
//! This driver does not set `IOCON.BANK`, but the factory default is `0` and this driver does
//! not change that value.
//!
//! See [the datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf) for more
//! information on the device.

use embedded_hal::i2c::I2c;

/// Struct for an MCP23017.
/// See the crate-level documentation for general info on the device and the operation of this
/// driver.
#[derive(Clone, Copy, Debug)]
pub struct MCP23017<I2C: I2c> {
    com: I2C,
    /// The I2C slave address of this device.
    pub address: u8,
}

impl<I2C: I2c> MCP23017<I2C> {
    /// Creates an expander with specific address.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { com: i2c, address }
    }

    /// Initiates hardware with basic setup.
    pub fn init_hardware(&mut self) -> Result<(), I2C::Error> {
        // set all inputs to defaults on port A and B
        self.write_register(Register::IODIRA, 0xff)?;
        self.write_register(Register::IODIRB, 0xff)?;

        Ok(())
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut data: [u8; 1] = [0];
        self.com.write_read(self.address, &[reg as u8], &mut data)?;
        Ok(data[0])
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), I2C::Error> {
        self.com.write(self.address, &[reg as u8, byte])
    }

    /// Sets all pins of a port to the given modes. Bit value 1 means input, 0 means output.
    pub fn port_mode(&mut self, port: Port, mode: u8) -> Result<(), I2C::Error> {
        let ioreg = match port {
            Port::GPIOA => Register::IODIRA,
            Port::GPIOB => Register::IODIRB,
        };
        let gppureg: Register = match port {
            Port::GPIOA => Register::GPPUA,
            Port::GPIOB => Register::GPPUB,
        };
        let ipolreg: Register = match port {
            Port::GPIOA => Register::IPOLA,
            Port::GPIOB => Register::IPOLB,
        };
        self.write_register(gppureg, 0xFF).ok(); // eek
        self.write_register(ipolreg, 0x00).ok(); // hacky
        self.write_register(ioreg, mode)
    }

    /// Reads a single port, A or B, and returns its current 8 bit value.
    pub fn read_gpio(&mut self, port: Port) -> Result<u8, I2C::Error> {
        let reg = match port {
            Port::GPIOA => Register::GPIOA,
            Port::GPIOB => Register::GPIOB,
        };
        self.read_register(reg)
    }

    /// Writes all the pins of one port with the value at the same time.
    pub fn write_gpio(&mut self, port: Port, value: u8) -> Result<(), I2C::Error> {
        let reg = match port {
            Port::GPIOA => Register::GPIOA,
            Port::GPIOB => Register::GPIOB,
        };
        self.write_register(reg, value)
    }
}

/// Generic port definitions.
#[derive(Debug, Copy, Clone)]
pub enum Port {
    /// Represent port A.
    GPIOA,
    /// Represent port B.
    GPIOB,
}

#[derive(Debug, Copy, Clone)]
enum Register {
    IODIRA = 0x00,
    IPOLA = 0x02,
    _GPINTENA = 0x04,
    _DEFVALA = 0x06,
    _INTCONA = 0x08,
    _IOCONA = 0x0A,
    GPPUA = 0x0C,
    _INTFA = 0x0E,
    _INTCAPA = 0x10,
    GPIOA = 0x12,
    _OLATA = 0x14,
    IODIRB = 0x01,
    IPOLB = 0x03,
    _GPINTENB = 0x05,
    _DEFVALB = 0x07,
    _INTCONB = 0x09,
    _IOCONB = 0x0B,
    GPPUB = 0x0D,
    _INTFB = 0x0F,
    _INTCAPB = 0x11,
    GPIOB = 0x13,
    _OLATB = 0x15,
}
