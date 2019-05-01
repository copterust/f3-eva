use hal::time::{Bps, Hertz, KiloHertz};

pub const BAUD_RATE: Bps<u32> = Bps(460800);

pub const TIM_FREQ: KiloHertz<u32> = KiloHertz(32);
#[allow(unused)]
pub const TICK_TIMEOUT: Hertz<u32> = Hertz(1);
#[allow(unused)]
pub const TICK_PERIOD: i8 = 5;

pub const NSAMPLES: u16 = 388;
