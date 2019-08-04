use asm_delay::CyclesToTime;

use cortex_m::peripheral::DCB;
use cortex_m::peripheral::DWT;
use hal::rcc::Clocks;
use hal::time::*;

pub type T = impl Chrono;

pub fn stopwatch(dwt: DWT, dcb: DCB, clocks: Clocks) -> T {
    let m = MonoTimer::new(dwt, dcb, clocks);
    Stopwatch::new(m)
}

pub trait Chrono: Sized {
    type Time;
    /// Get the last measurements without updating state
    fn last(&self) -> Self::Time;

    /// Starts new cycle
    fn reset(&mut self) {
        self.split_time_ms();
    }

    /// Get elapsed time (ms) since last measurement and start new cycle
    fn split_time_ms(&mut self) -> f32;

    /// Get elapsed time (s) since last measurement and start new cycle
    fn split_time_s(&mut self) -> f32 {
        self.split_time_ms() / 1000.
    }
}

pub struct Stopwatch {
    cc: CyclesToTime,
    last: Instant,
    mt: MonoTimer,
}

impl Stopwatch {
    pub fn new(mt: MonoTimer) -> Self {
        let f = mt.frequency();
        let last = mt.now();
        Stopwatch { mt, cc: CyclesToTime::new(f), last }
    }
}

impl Chrono for Stopwatch {
    type Time = Instant;

    fn last(&self) -> Self::Time {
        self.last
    }

    fn split_time_ms(&mut self) -> f32 {
        let duration = self.last.elapsed();
        self.last = self.mt.now();
        self.cc.to_ms(duration)
    }
}
