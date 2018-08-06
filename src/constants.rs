use hal::time::{Bps, Hertz, MegaHertz};

pub const BAUD_RATE: Bps = Bps(9600);

pub const TIM_TIMEOUT: MegaHertz = MegaHertz(1);
#[allow(unused)]
pub const TICK_TIMEOUT: Hertz = Hertz(1);
#[allow(unused)]
pub const TICK_PERIOD: i8 = 5;

pub const NSAMPLES: u16 = 256;

pub mod messages {
    pub const RESET: u8 = 'r' as u8;
    pub const BOOTLOADER: u8 = 'R' as u8;
    pub const MOTOR: u8 = '0' as u8;
}
