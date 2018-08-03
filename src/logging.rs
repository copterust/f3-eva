use ehal;
use nb;

use beeper;
use core::fmt;
use utils;

pub struct SerialLogger<Wr>
    where Wr: ehal::serial::Write<u8>
{
    tx: Wr,
    beeper: beeper::Beeper,
}

impl<Wr> SerialLogger<Wr> where Wr: ehal::serial::Write<u8> + Sized
{
    pub fn new(tx: Wr, beeper: beeper::Beeper) -> Self {
        Self { tx,
               beeper, }
    }

    pub fn blink(&mut self) {
        self.beeper.on();
        // XXX: use Delay and ms
        utils::tick_delay(100000);
        self.beeper.off();
    }

    fn write_one(&mut self, data: u8) {
        let b = data.into();
        match nb::block!(self.tx.write(b)) {
            Ok(_) => {},
            Err(_) => {
                self.blink();
            },
        }
    }

    fn write_many(&mut self, data: &[u8]) {
        for b in data.iter() {
            self.write_one(b.clone());
        }
        match self.tx.flush() {
            Ok(_) => {},
            Err(_) => {},
        };
    }
}

impl<Wr> fmt::Write for SerialLogger<Wr> where Wr: ehal::serial::Write<u8>
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
