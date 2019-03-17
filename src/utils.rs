use cortex_m::asm;
use ehal::blocking::delay::DelayMs;
use hal::delay::Delay;
use nalgebra::Vector3;

// use asm_delay
pub fn tick_delay(ticks: usize) {
    (0..ticks).for_each(|_| asm::nop());
}

// use asm_delay
// XXX: migrate to Duration?
pub trait WallClockDelay: DelayMs<u32> {
    const MAX_STEP_DELAY: u32;
    fn wc_delay_ms(&mut self, ms: u32);
}

impl WallClockDelay for Delay {
    const MAX_STEP_DELAY: u32 = 100;

    fn wc_delay_ms(&mut self, ms: u32) {
        let steps = ms / Self::MAX_STEP_DELAY;
        let step = (ms / Self::MAX_STEP_DELAY) as u32;
        (0..steps).for_each(|_| self.delay_ms(step));
    }
}

// TODO: generify
pub fn vec_to_tuple<T: nalgebra::base::Scalar>(inp: &Vector3<T>) -> (T, T, T) {
    (inp.x, inp.y, inp.z)
}

pub fn parse<T, E>(bytes: &[u8]) -> Result<T, E>
    where T: core::str::FromStr<Err = E>
{
    let v = unsafe { core::str::from_utf8_unchecked(bytes) };
    T::from_str(v)
}
