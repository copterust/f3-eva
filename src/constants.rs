use hal::time::{Bps, Hertz, KiloHertz};

pub const BAUD_RATE: Bps = Bps(115200);

pub const TIM_FREQ: KiloHertz = KiloHertz(32);
#[allow(unused)]
pub const TICK_TIMEOUT: Hertz = Hertz(1);
#[allow(unused)]
pub const TICK_PERIOD: i8 = 5;

pub const NSAMPLES: u16 = 388;

pub mod messages {
    pub const RESET: u8 = 'r' as u8;
    pub const BOOTLOADER: u8 = 'R' as u8;
    pub const MOTOR: u8 = '0' as u8;

    pub const PLUS_T: u8 = 'u' as u8;
    pub const MINUS_T: u8 = 'd' as u8;

    pub const PLUS_KY: u8 = '+' as u8;
    pub const MINUS_KY: u8 = '-' as u8;

    pub const CTL_1: u8 = 'z' as u8;
    pub const CTL_5: u8 = 'f' as u8;
}
