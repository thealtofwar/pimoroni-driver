#![no_std]

use embassy_time::{Duration, Instant, Timer};
use embedded_hal::digital::v2::InputPin;
use embedded_hal_async::i2c::I2c;

pub const I2C_ADDRESS: u8 = 0x0A;
pub const I2C_ADDRESS_ALTERNATIVE: u8 = 0x0B;

pub const CHIP_ID: u16 = 0xBA11;
pub const VERSION: u8 = 1;

pub const REG_LED_RED: u8 = 0x00;
pub const REG_LED_GRN: u8 = 0x01;
pub const REG_LED_BLU: u8 = 0x02;
pub const REG_LED_WHT: u8 = 0x03;

pub const REG_LEFT: u8 = 0x04;
pub const REG_RIGHT: u8 = 0x05;
pub const REG_UP: u8 = 0x06;
pub const REG_DOWN: u8 = 0x07;
pub const REG_SWITCH: u8 = 0x08;
pub const MSK_SWITCH_STATE: u8 = 0b10000000;

pub const REG_USER_FLASH: u8 = 0xD0;
pub const REG_FLASH_PAGE: u8 = 0xF0;
pub const REG_INT: u8 = 0xF9;
pub const MSK_INT_TRIGGERED: u8 = 0b00000001;
pub const MSK_INT_OUT_EN: u8 = 0b00000010;
pub const REG_CHIP_ID_L: u8 = 0xFA;
pub const RED_CHIP_ID_H: u8 = 0xFB;
pub const REG_VERSION: u8 = 0xFC;
pub const REG_I2C_ADDR: u8 = 0xFD;
pub const REG_CTRL: u8 = 0xFE;
pub const MSK_CTRL_SLEEP: u8 = 0b00000001;
pub const MSK_CTRL_RESET: u8 = 0b00000010;
pub const MSK_CTRL_FREAD: u8 = 0b00000100;
pub const MSK_CTRL_FWRITE: u8 = 0b00001000;

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    InvalidChipId(u16),
    Timeout,
}

pub struct TrackBall<I2C, Pin> {
    i2c: I2C,
    address: u8,
    interrupt_pin: Option<Pin>,
    timeout: Duration,
}

impl<I2C, Pin> TrackBall<I2C, Pin>
where
    I2C: I2c,
    Pin: InputPin,
{
    pub async fn new(
        i2c: I2C,
        interrupt_pin: Option<Pin>,
        address: Option<u8>,
        timeout_secs: Option<u64>,
    ) -> Result<Self, Error<I2C::Error>> {
        let mut trackball = Self {
            i2c,
            address: address.unwrap_or(I2C_ADDRESS),
            interrupt_pin,
            timeout: Duration::from_secs(timeout_secs.unwrap_or(5)),
        };

        let mut chip_id_bytes = [0u8; 2];
        trackball
            .i2c_rdwr(&[REG_CHIP_ID_L], &mut chip_id_bytes)
            .await?;
        let chip_id = u16::from_le_bytes(chip_id_bytes);

        if chip_id != CHIP_ID {
            return Err(Error::InvalidChipId(chip_id));
        }

        trackball.enable_interrupt(true).await?;

        Ok(trackball)
    }

    /// Write a new I2C address into flash.
    pub async fn change_address(&mut self, new_address: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c_rdwr(&[REG_I2C_ADDR, new_address], &mut [])
            .await?;
        self.wait_for_flash().await
    }

    async fn wait_for_flash(&mut self) -> Result<(), Error<I2C::Error>> {
        let start = Instant::now();
        while self.get_interrupt().await? {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout);
            }
            Timer::after(Duration::from_millis(1)).await;
        }

        let start = Instant::now();
        while !self.get_interrupt().await? {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout);
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        
        Ok(())
    }

    /// Enable/disable GPIO interrupt pin.
    pub async fn enable_interrupt(&mut self, interrupt: bool) -> Result<(), Error<I2C::Error>> {
        let mut value = [0u8; 1];
        self.i2c_rdwr(&[REG_INT], &mut value).await?;
        
        let mut val = value[0] & !MSK_INT_OUT_EN;
        if interrupt {
            val |= MSK_INT_OUT_EN;
        }

        self.i2c_rdwr(&[REG_INT, val], &mut []).await
    }

    /// Write and optionally read I2C data.
    async fn i2c_rdwr(&mut self, write_data: &[u8], read_buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, write_data)
            .await
            .map_err(Error::I2c)?;

        if !read_buf.is_empty() {
            Timer::after(Duration::from_millis(20)).await;
            self.i2c
                .read(self.address, read_buf)
                .await
                .map_err(Error::I2c)?;
        }

        Ok(())
    }

    /// Get the trackball interrupt status
    pub async fn get_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
        if let Some(pin) = &self.interrupt_pin {
            Ok(pin.is_low().unwrap_or(false))
        } else {
            let mut value = [0u8; 1];
            self.i2c_rdwr(&[REG_INT], &mut value).await?;
            Ok((value[0] & MSK_INT_TRIGGERED) != 0)
        }
    }
    /// Set all LED brightness as RGBW.
    pub async fn set_rgbw(&mut self, r: u8, g: u8, b: u8, w: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c_rdwr(&[REG_LED_RED, r, g, b, w], &mut []).await
    }

    /// Set brightness of trackball red LED.
    pub async fn set_red(&mut self, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c_rdwr(&[REG_LED_RED, value], &mut []).await
    }

    /// Set brightness of trackball green LED.
    pub async fn set_green(&mut self, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c_rdwr(&[REG_LED_GRN, value], &mut []).await
    }

    /// Set brightness of trackball blue LED.
    pub async fn set_blue(&mut self, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c_rdwr(&[REG_LED_BLU, value], &mut []).await
    }

    /// Set brightness of trackball white LED.
    pub async fn set_white(&mut self, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c_rdwr(&[REG_LED_WHT, value], &mut []).await
    }

    /// Read up, down, left, right and switch data from trackball.
    pub async fn read(&mut self) -> Result<(u8, u8, u8, u8, u8, bool), Error<I2C::Error>> {
        let mut buf = [0u8; 5];
        self.i2c_rdwr(&[REG_LEFT], &mut buf).await?;
        let left = buf[0];
        let right = buf[1];
        let up = buf[2];
        let down = buf[3];
        let switch_raw = buf[4];

        let switch = switch_raw & !MSK_SWITCH_STATE;
        let switch_state = (switch_raw & MSK_SWITCH_STATE) > 0;

        Ok((up, down, left, right, switch, switch_state))
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        
    }
}
