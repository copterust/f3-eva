#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]

extern crate cortex_m;

extern crate embedded_hal as ehal;
extern crate stm32f30x_hal as hal;

mod beeper;

use hal::prelude::*;
use hal::stm32f30x;
// use hal::stm32f30x::GpioExt;
//use cortex_m::asm;

fn main() {
    let dp = stm32f30x::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let gpioc = dp.GPIOC.split(&mut rcc.ahb);

    let mut beep = beeper::Beeper::new(gpioc);
    loop {
        beep.on();
        beep.off();
    }
}
