use ehal;
use nb;

use beeper;
use utils;

pub struct SerialLogger<Wr>
    where Wr: ehal::serial::Write<u8> + Sized
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

    fn write_one<I: Into<u8>>(&mut self, data: I) {
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

impl<'a, Wr> Logger<'a> for SerialLogger<Wr>
    where Wr: ehal::serial::Write<u8> + Sized
{
    type Underlying = u8;

    fn debug(&mut self, data: impl Into<Debuggable<'a, u8>>) {
        match data.into() {
            Debuggable::One(byte) => self.write_one(byte),
            Debuggable::Many(bytes) => self.write_many(bytes),
        }
    }

    fn error(&mut self, data: impl Into<Debuggable<'a, u8>>) {
        self.blink();
        self.debug(data);
    }
}

pub trait Logger<'a> {
    type Underlying: 'a;
    fn debug(&mut self, data: impl Into<Debuggable<'a, Self::Underlying>>);
    fn error(&mut self, data: impl Into<Debuggable<'a, Self::Underlying>>);
}

pub enum Debuggable<'a, T: 'a> {
    One(T),
    Many(&'a [T]),
}

impl<'a> From<u8> for Debuggable<'a, u8> {
    fn from(arg: u8) -> Self {
        Debuggable::One(arg)
    }
}

impl<'a> From<char> for Debuggable<'a, u8> {
    fn from(arg: char) -> Self {
        Debuggable::One(arg as u8)
    }
}

impl<'a> From<&'a str> for Debuggable<'a, u8> {
    fn from(arg: &'a str) -> Self {
        Debuggable::Many(arg.as_bytes())
    }
}

impl<'a> From<&'a [u8]> for Debuggable<'a, u8> {
    fn from(arg: &'a [u8]) -> Self {
        Debuggable::Many(arg)
    }
}
