
extern crate nb;
extern crate embedded_hal as ehal;

use beeper;
use hal::time::Hertz;

pub struct DebugWriter<Wr, CountDown>
    where CountDown: ehal::timer::CountDown<Time=Hertz> + Sized,
          Wr: ehal::serial::Write<u8> + Sized
{
    tx: Wr,
    timer: CountDown,
    beeper: beeper::Beeper,
    err_delay: Hertz,
}

impl<Wr, CountDown> DebugWriter<Wr, CountDown>
    where CountDown: ehal::timer::CountDown<Time=Hertz> + Sized,
          Wr: ehal::serial::Write<u8> + Sized
{
    pub fn new(tx: Wr,
               timer: CountDown,
               beeper: beeper::Beeper,
               err_delay: Hertz) -> Self {
        Self { tx, timer, beeper, err_delay }
    }

    pub fn beeper(&mut self) -> &mut beeper::Beeper {
        &mut self.beeper
    }

    fn err_beep(&mut self) {
        self.beeper.on();
        self.delay();
        self.beeper.off();
    }

    fn delay(&mut self) {
        self.timer.start(self.err_delay);
        while let Err(nb::Error::WouldBlock) = self.timer.wait() { }
    }

}

impl<Wr, CountDown> DebugWrite<u8> for DebugWriter<Wr, CountDown>
    where CountDown: ehal::timer::CountDown<Time=Hertz> + Sized,
          Wr: ehal::serial::Write<u8> + Sized {
    fn debug(&mut self, data: u8) {
        match self.tx.write(data) {
            Ok(_) => {}
            Err(_) => { self.err_beep(); }
        }
    }
    fn error(&mut self, data: u8) {
        self.debug(data);
        self.err_beep();
    }
}

impl<Wr, CountDown> DebugWrite<char> for DebugWriter<Wr, CountDown>
    where CountDown: ehal::timer::CountDown<Time=Hertz> + Sized,
          Wr: ehal::serial::Write<u8> + Sized {
    fn debug(&mut self, data: char) {
        match self.tx.write(data as u8) {
            Ok(_) => {}
            Err(_) => { self.err_beep(); }
        }
    }

    fn error(&mut self, data: char) {
        self.debug(data);
        self.err_beep();
    }
}

pub trait DebugWrite<Data> {
    fn debug(&mut self, data: Data);
    fn error(&mut self, data: Data);
}


// fn delay<CD, T, A>(t: &mut CD, timeout: T)
//     where CD: ehal::timer::CountDown<Time=T>,
//           A: Into<T> {
//     t.start(timeout);
//     while let Err(nb::Error::WouldBlock) = t.wait() {
//     }
// }
