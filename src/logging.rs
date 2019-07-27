use ehal;
use nb;

use crate::utils;
use core::fmt;

pub struct SerialLogger<Wr> {
    tx: Wr,
}

impl<Wr> SerialLogger<Wr>
    where Wr: ehal::serial::Write<u8> + Sized
{
    pub fn new(tx: Wr) -> Self {
        Self { tx}
    }

    pub fn blink(&mut self) {
        // self.beeper.set_high();
        // XXX: use Delay and ms
        utils::tick_delay(32000);
        // let _ = self.beeper.set_low();
    }
}

impl<Wr> fmt::Write for SerialLogger<Wr>
    where Wr: ehal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            match self.write_char(c) {
                Ok(_) => {},
                Err(_) => {},
            }
        }
        match self.tx.flush() {
            Ok(_) => {},
            Err(_) => {},
        };
        Ok(())
    }

    fn write_char(&mut self, s: char) -> fmt::Result {
        match nb::block!(self.tx.write(s as u8)) {
            Ok(_) => {},
            Err(_) => {},
        }
        Ok(())
    }
}
