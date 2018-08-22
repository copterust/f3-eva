use ehal;
use nb;

use core::fmt;
use crate::utils;

pub struct SerialLogger<Wr, Op>
    where Wr: ehal::serial::Write<u8>,
          Op: ehal::digital::OutputPin
{
    tx: Wr,
    beeper: Op,
}

impl<Wr, Op> SerialLogger<Wr, Op>
    where Wr: ehal::serial::Write<u8> + Sized,
          Op: ehal::digital::OutputPin
{
    pub fn new(tx: Wr, beeper: Op) -> Self {
        Self { tx,
               beeper, }
    }

    pub fn blink(&mut self) {
        self.beeper.set_high();
        // XXX: use Delay and ms
        utils::tick_delay(32000);
        self.beeper.set_low();
    }
}

impl<Wr, Op> fmt::Write for SerialLogger<Wr, Op>
    where Wr: ehal::serial::Write<u8>,
          Op: ehal::digital::OutputPin
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
