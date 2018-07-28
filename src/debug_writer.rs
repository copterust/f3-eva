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

    pub fn blink(&mut self) {
        self.beeper.on();
        self.delay();
        self.beeper.off();
    }

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

    pub fn debug<T: Debuggable>(&mut self, data: T) {
        data.do_debug(self)
    }

    pub fn error<T: Debuggable>(&mut self, data: T) {
        self.debug(data);
        self.blink();
    }
}

pub trait Debuggable {
    fn do_debug<Cd, Wr>(&self, &mut DebugWriter<Wr, Cd>)
        where Cd: ehal::timer::CountDown<Time = Hertz> + Sized,
              Wr: ehal::serial::Write<u8> + Sized;
}

impl Debuggable for u8 {
    fn do_debug<Cd, Wr>(&self, dw: &mut DebugWriter<Wr, Cd>)
        where Cd: ehal::timer::CountDown<Time = Hertz> + Sized,
              Wr: ehal::serial::Write<u8> + Sized
    {
        dw.write_one(self.clone())
    }
}

impl Debuggable for char {
    fn do_debug<Cd, Wr>(&self, dw: &mut DebugWriter<Wr, Cd>)
        where Cd: ehal::timer::CountDown<Time = Hertz> + Sized,
              Wr: ehal::serial::Write<u8> + Sized
    {
        dw.write_one(self.clone() as u8)
    }
}

impl Debuggable for &[u8] {
    fn do_debug<Cd, Wr>(&self, dw: &mut DebugWriter<Wr, Cd>)
        where Cd: ehal::timer::CountDown<Time = Hertz> + Sized,
              Wr: ehal::serial::Write<u8> + Sized
    {
        dw.write_many(self)
    }
}

impl Debuggable for &str {
    fn do_debug<Cd, Wr>(&self, dw: &mut DebugWriter<Wr, Cd>)
        where Cd: ehal::timer::CountDown<Time = Hertz> + Sized,
              Wr: ehal::serial::Write<u8> + Sized
    {
        dw.write_many(self.as_bytes())
    }
}
