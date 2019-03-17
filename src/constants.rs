use hal::time::{Bps, Hertz, KiloHertz};

pub const BAUD_RATE: Bps<u32> = Bps(9600);

pub const TIM_FREQ: KiloHertz<u32> = KiloHertz(32);

pub const NSAMPLES: u16 = 388;
