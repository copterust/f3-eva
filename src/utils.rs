use cortex_m::asm;
use ehal::blocking::delay::DelayMs;
use hal::delay::Delay;

pub fn tick_delay(ticks: usize) {
    (0..ticks).for_each(|_| asm::nop());
}

pub trait WallClockDelay: DelayMs<u32> {
    const MAX_STEP_DELAY: u32;
    fn wc_delay_ms(&mut self, ms: u32);
}

impl WallClockDelay for Delay {
    // for 64.mhz()
    const MAX_STEP_DELAY: u32 = 250;

    fn wc_delay_ms(&mut self, ms: u32) {
        let steps = ms / Self::MAX_STEP_DELAY;
        let step = (ms / Self::MAX_STEP_DELAY) as u32;
        (0..steps).for_each(|_| self.delay_ms(step));
    }
}
