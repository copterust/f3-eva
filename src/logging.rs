use ehal;
use nb;

use beeper;

pub struct DebugWriter<Wr>
    where Wr: ehal::serial::Write<u8> + Sized
{
    tx: Wr,
    beeper: beeper::Beeper,
}

impl<Wr> DebugWriter<Wr> where Wr: ehal::serial::Write<u8> + Sized
{
    pub fn new(tx: Wr, beeper: beeper::Beeper) -> Self {
        Self { tx,
               beeper, }
    }

    // fn delay(&mut self) {
    //     self.timer.start(self.err_delay);
    //     while let Err(nb::Error::WouldBlock) = self.timer.wait() {}
    // }

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

impl<Wr> Debugger for DebugWriter<Wr> where Wr: ehal::serial::Write<u8> + Sized
{
    fn debug<'a>(&mut self, data: impl Into<Debuggable<'a>>) {
        match data.into() {
            Debuggable::One(byte) => self.write_one(byte),
            Debuggable::Many(bytes) => self.write_many(bytes),
        }
    }
}

impl<Wr> ErrorReporter for DebugWriter<Wr>
    where Wr: ehal::serial::Write<u8> + Sized
{
    fn blink(&mut self) {
        self.beeper.on();
        // self.delay();
        self.beeper.off();
    }
}

pub trait Debugger: Sized {
    fn debug<'a>(&mut self, data: impl Into<Debuggable<'a>>);
}

pub trait ErrorReporter: Debugger {
    fn blink(&mut self);
    fn error<'a>(&mut self, data: impl Into<Debuggable<'a>>) {
        self.debug(data);
        self.blink();
    }
}

pub enum Debuggable<'a> {
    One(u8),
    Many(&'a [u8]),
}

impl<'a> From<u8> for Debuggable<'a> {
    fn from(arg: u8) -> Self {
        Debuggable::One(arg)
    }
}

impl<'a> From<char> for Debuggable<'a> {
    fn from(arg: char) -> Self {
        Debuggable::One(arg as u8)
    }
}

impl<'a> From<&'a str> for Debuggable<'a> {
    fn from(arg: &'a str) -> Self {
        Debuggable::Many(arg.as_bytes())
    }
}

impl<'a> From<&'a [u8]> for Debuggable<'a> {
    fn from(arg: &'a [u8]) -> Self {
        Debuggable::Many(arg)
    }
}
