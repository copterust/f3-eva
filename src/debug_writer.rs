use ehal;
use nb;

use beeper;
use hal::time::Hertz;

pub struct DebugWriter<Wr, CountDown>
    where CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
          Wr: ehal::serial::Write<u8> + Sized
{
    tx: Wr,
    timer: CountDown,
    beeper: beeper::Beeper,
    err_delay: Hertz,
}

impl<Wr, CountDown> DebugWriter<Wr, CountDown>
    where CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
          Wr: ehal::serial::Write<u8> + Sized
{
    pub fn new(tx: Wr,
               timer: CountDown,
               beeper: beeper::Beeper,
               err_delay: Hertz)
               -> Self {
        Self { tx,
               timer,
               beeper,
               err_delay, }
    }

    fn delay(&mut self) {
        self.timer.start(self.err_delay);
        while let Err(nb::Error::WouldBlock) = self.timer.wait() {}
    }
}

impl<Wr, CountDown> Debugger for DebugWriter<Wr, CountDown>
    where CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
          Wr: ehal::serial::Write<u8> + Sized
{
    fn write_one<I: Into<u8>>(&mut self, data: I) {
        let b = data.into();
        match nb::block!(self.tx.write(b)) {
            Ok(_) => {}
            Err(_) => {
                self.blink();
            }
        }
    }

    fn write_many(&mut self, data: &[u8]) {
        for b in data.iter() {
            self.write_one(b.clone());
        }
        match self.tx.flush() {
            Ok(_) => {}
            Err(_) => {}
        };
    }
}

impl<Wr, CountDown> ErrorReporter for DebugWriter<Wr, CountDown>
    where CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
          Wr: ehal::serial::Write<u8> + Sized
{
    fn blink(&mut self) {
        self.beeper.on();
        self.delay();
        self.beeper.off();
    }
}

pub trait Debugger: Sized {
    fn write_one<I: Into<u8>>(&mut self, data: I);
    fn write_many(&mut self, data: &[u8]);
    fn debug<T: Debuggable>(&mut self, data: T) {
        data.do_debug(self)
    }
}

pub trait ErrorReporter: Debugger {
    fn blink(&mut self);
    fn error<T: Debuggable>(&mut self, data: T) {
        self.debug(data);
        self.blink();
    }
}

pub trait Debuggable: Sized {
    fn do_debug(&self, &mut impl Debugger);
}

impl Debuggable for u8 {
    fn do_debug(&self, dw: &mut impl Debugger) {
        dw.write_one(self.clone())
    }
}

impl Debuggable for char {
    fn do_debug(&self, dw: &mut impl Debugger) {
        dw.write_one(self.clone() as u8)
    }
}

impl Debuggable for &[u8] {
    fn do_debug(&self, dw: &mut impl Debugger) {
        dw.write_many(self)
    }
}

impl Debuggable for &str {
    fn do_debug(&self, dw: &mut impl Debugger) {
        dw.write_many(self.as_bytes())
    }
}
