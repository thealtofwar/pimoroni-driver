#![no_std]

use core::fmt::{Arguments, Display};

use embassy_time::{Duration};
use embedded_hal::digital::v2::InputPin;
use embedded_hal_async::i2c::I2c;

/// Trackball constants
pub const I2C_ADDRESS: u8 = 0x0A;
pub const I2C_ADDRESS_ALTERNATIVE: u8 = 0x0B;

pub const CHIP_ID: u16 = 0xBA11;
pub const VERSION: u8 = 1;

// Registers
pub const REG_LED_RED: u8 = 0x00;
pub const REG_LED_GRN: u8 = 0x01;
pub const REG_LED_BLU: u8 = 0x02;
pub const REG_LED_WHT: u8 = 0x03;

pub const REG_LEFT: u8 = 0x04;
pub const REG_RIGHT: u8 = 0x05;
pub const REG_UP: u8 = 0x06;
pub const REG_DOWN: u8 = 0x07;
pub const REG_SWITCH: u8 = 0x08;
pub const MSK_SWITCH_STATE: u8 = 0b1000_0000;

pub const REG_INT: u8 = 0xF9;
pub const MSK_INT_TRIGGERED: u8 = 0b0000_0001;
pub const MSK_INT_OUT_EN: u8 = 0b0000_0010;

pub const REG_CHIP_ID_L: u8 = 0xFA;
pub const REG_CHIP_ID_H: u8 = 0xFB;
pub const REG_VERSION: u8 = 0xFC;
pub const REG_I2C_ADDR: u8 = 0xFD;
pub const REG_CTRL: u8 = 0xFE;

pub struct Trackball<'d, T: I2c, Pin: InputPin> {
    i2c: &'d mut T,
    address: u8,
    interrupt_pin: Option<Pin>
}

pub enum DriverError {
    InvalidChipId {found: u16, expected: u16}
}

impl Display for DriverError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DriverError::InvalidChipId { found, expected } => {
                format_args!("Invalid chip ID: 0x{:04X}, expected 0x{:04X}", found, expected).fmt(f)
            },
        }
    }
}

pub enum Error<T: I2c> {
    I2cError(T::Error),
    DriverError(DriverError)
}


impl<'d, T: I2c, Pin: InputPin> Trackball<'d, T, Pin> {
    pub async fn new(i2c: &'d mut T, address: u8, interrupt_pin: Option<Pin>) -> Result<Self, Error<T>> {
        // Read chip ID (2 bytes)
        let mut buf = [0u8; 2];
        if let Err(e) = i2c.write_read(address, &[REG_CHIP_ID_L], &mut buf).await {
            return Err(Error::I2cError(e));
        }
        let chip_id = u16::from_le_bytes(buf);

        if chip_id != CHIP_ID {
            return Err(Error::DriverError(DriverError::InvalidChipId { found: chip_id, expected: CHIP_ID }));
        }

        Ok(Self {
            i2c,
            address,
            interrupt_pin
        })
    }

    async fn i2c_write(&mut self, data: &[u8]) -> Result<(), T::Error> {
        self.i2c.write(self.address, data).await
    }

    async fn i2c_write_read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), T::Error> {
        self.i2c.write_read(self.address, &[reg], buf).await
    }

    pub async fn set_rgbw(&mut self, r: u8, g: u8, b: u8, w: u8) -> Result<(), T::Error> {
        self.i2c_write(&[REG_LED_RED, r, g, b, w]).await
    }

    pub async fn set_red(&mut self, value: u8) -> Result<(), T::Error> {
        self.i2c_write(&[REG_LED_RED, value]).await
    }
    pub async fn set_green(&mut self, value: u8) -> Result<(), T::Error> {
        self.i2c_write(&[REG_LED_GRN, value]).await
    }
    pub async fn set_blue(&mut self, value: u8) -> Result<(), T::Error> {
        self.i2c_write(&[REG_LED_BLU, value]).await
    }
    pub async fn set_white(&mut self, value: u8) -> Result<(), T::Error> {
        self.i2c_write(&[REG_LED_WHT, value]).await
    }

    pub async fn read_motion(&mut self) -> Result<(u8, u8, u8, u8, u8, bool), T::Error> {
        let mut buf = [0u8; 5];
        self.i2c_write_read(REG_LEFT, &mut buf).await?;
        let left = buf[0];
        let right = buf[1];
        let up = buf[2];
        let down = buf[3];
        let switch = buf[4];
        let switch_val = switch & !MSK_SWITCH_STATE;
        let switch_state = (switch & MSK_SWITCH_STATE) > 0;
        Ok((up, down, left, right, switch_val, switch_state))
    }

    pub async fn get_interrupt(&mut self) -> Result<bool, T::Error> {
        let mut buf = [0u8; 1];
        self.i2c_write_read(REG_INT, &mut buf).await?;
        Ok((buf[0] & MSK_INT_TRIGGERED) != 0)
    }

    pub async fn enable_interrupt(&mut self, enable: bool) -> Result<(), T::Error> {
        let mut buf = [0u8; 1];
        self.i2c_write_read(REG_INT, &mut buf).await?;
        let mut value = buf[0] & !MSK_INT_OUT_EN;
        if enable {
            value |= MSK_INT_OUT_EN;
        }
        self.i2c_write(&[REG_INT, value]).await
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        
    }
}
