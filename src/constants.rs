use hal::time::{Bps, Hertz, KiloHertz};

pub const BAUD_RATE: Bps = Bps(9600);

pub const TIM_FREQ: KiloHertz = KiloHertz(32);
#[allow(unused)]
pub const TICK_TIMEOUT: Hertz = Hertz(1);
#[allow(unused)]
pub const TICK_PERIOD: i8 = 5;

pub const NSAMPLES: u16 = 388;
