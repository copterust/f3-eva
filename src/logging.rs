use ehal;
use nb;

use core::fmt;
use utils;

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
        utils::tick_delay(100000);
        self.beeper.set_low();
    }

    fn write_one(&mut self, data: u8) {
        let b = data.into();
        match nb::block!(self.tx.write(b)) {
            Ok(_) => {},
            Err(_) => self.blink(),
        }
    }

    fn write_many(&mut self, data: &[u8]) {
        for b in data.iter() {
            self.write_one(b.clone());
        }
        match self.tx.flush() {
            Ok(_) => {},
            Err(_) => self.blink(),
        };
    }
}

impl<Wr, Op> fmt::Write for SerialLogger<Wr, Op>
    where Wr: ehal::serial::Write<u8>,
          Op: ehal::digital::OutputPin
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_many(s.as_bytes());
        Ok(())
    }

    fn write_char(&mut self, s: char) -> fmt::Result {
        self.write_one(s as u8);
        Ok(())
    }
}
