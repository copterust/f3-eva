#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]

extern crate cortex_m;

extern crate embedded_hal as ehal;
extern crate stm32f30x_hal as hal;

mod beeper;

use hal::prelude::*;
use hal::stm32f30x;
use hal::delay::Delay;

fn main() {
    let dp = stm32f30x::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let gpioc = dp.GPIOC.split(&mut rcc.ahb);

    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut beep = beeper::Beeper::new(gpioc);
    loop {
        delay.delay_ms(1_000_u16);
        beep.on();
        delay.delay_ms(1_000_u16);
        beep.off();
    }
}
