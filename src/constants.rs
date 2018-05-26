extern crate stm32f30x_hal as hal;

use hal::time::Hertz;

pub const BAUD_RATE: hal::time::Bps = hal::time::Bps(9600);

pub const DEBUG_TIMEOUT: Hertz = Hertz(1);
pub const TICK_TIMEOUT: Hertz = Hertz(1);
pub const TICK_PERIOD: i8 = 5;

pub mod messages {
    pub const INIT: &'static str = "init\r\n";
    pub const TICK: &'static str = "tick\r\n";
    // pub const TOCK: char = 'o';

    pub const RESET: u8 = 'r' as u8;
    pub const BOOTLOADER: u8 = 'R' as u8;
    pub const MOTOR: u8 = '0' as u8;

    pub const FRAMING_ERROR: char = 'f';
    pub const PARITY_ERROR: char = 'p';
    pub const NOISE: char = 'n';
    pub const UNKNOWN_ERROR: char = 'u';
    pub const ERROR: char = 'e';

    pub const HARD_FAULT: char = 'h';
    pub const DEFAULT_INTERRUPT: char = 'd';
}
