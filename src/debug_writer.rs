use ehal;
use nb;

use beeper;
use hal::time::Hertz;

pub trait DebugWrite<Data> {
    fn debug(&mut self, data: Data);
    fn error(&mut self, data: Data);
}

pub struct DebugWriter<Wr, CountDown>
where
    CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
    Wr: ehal::serial::Write<u8> + Sized,
{
    tx: Wr,
    timer: CountDown,
    beeper: beeper::Beeper,
    err_delay: Hertz,
}

impl<Wr, CountDown> DebugWriter<Wr, CountDown>
where
    CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
    Wr: ehal::serial::Write<u8> + Sized,
{
    pub fn new(tx: Wr, timer: CountDown, beeper: beeper::Beeper, err_delay: Hertz) -> Self {
        Self {
            tx,
            timer,
            beeper,
            err_delay,
        }
    }

    pub fn blink(&mut self) {
        self.beeper.on();
        self.delay();
        self.beeper.off();
    }

    fn delay(&mut self) {
        self.timer.start(self.err_delay);
        while let Err(nb::Error::WouldBlock) = self.timer.wait() {}
    }
}

impl<Wr, CountDown> DebugWrite<u8> for DebugWriter<Wr, CountDown>
where
    CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
    Wr: ehal::serial::Write<u8> + Sized,
{
    fn debug(&mut self, data: u8) {
        match nb::block!(self.tx.write(data)) {
            Ok(_) => {}
            Err(_) => {
                self.blink();
            }
        }
    }
    fn error(&mut self, data: u8) {
        self.debug(data);
        self.blink();
    }
}

impl<Wr, CountDown> DebugWrite<char> for DebugWriter<Wr, CountDown>
where
    CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
    Wr: ehal::serial::Write<u8> + Sized,
{
    fn debug(&mut self, data: char) {
        match self.tx.write(data as u8) {
            Ok(_) => {}
            Err(_) => {
                self.blink();
            }
        }
    }

    fn error(&mut self, data: char) {
        self.debug(data);
        self.blink();
    }
}

impl<'a, Wr, CountDown> DebugWrite<&'a [u8]> for DebugWriter<Wr, CountDown>
where
    CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
    Wr: ehal::serial::Write<u8> + Sized,
{
    fn debug(&mut self, data: &[u8]) {
        for b in data.iter() {
            self.debug(b.clone());
        }
        match self.tx.flush() {
            Ok(_) => {}
            Err(_) => {}
        };
    }

    fn error(&mut self, data: &[u8]) {
        self.debug(data);
        self.blink();
    }
}

impl<'a, Wr, CountDown> DebugWrite<&'a str> for DebugWriter<Wr, CountDown>
where
    CountDown: ehal::timer::CountDown<Time = Hertz> + Sized,
    Wr: ehal::serial::Write<u8> + Sized,
{
    fn debug(&mut self, data: &str) {
        self.debug(data.as_bytes());
    }

    fn error(&mut self, data: &str) {
        self.debug(data);
        self.blink();
    }
}

// fn delay<CD, T, A>(t: &mut CD, timeout: T)
//     where CD: ehal::timer::CountDown<Time=T>,
//           A: Into<T> {
//     t.start(timeout);
//     while let Err(nb::Error::WouldBlock) = t.wait() {
//     }
// }
