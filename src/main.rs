#![no_std]

extern crate stm32f103xx;
extern crate cortex_m;

use stm32f103xx::GPIOC;
use cortex_m::asm;

fn main()  {
    let per = stm32f103xx::Peripherals::take().unwrap();

    // Enable port c in the register clock controll
    per.RCC.apb2enr.write(|w| w.iopcen().enabled());

    let gpioc = per.GPIOC;

    // Change PIN13 mode to output
    gpioc.crh.write(|w| w.mode13().output());
    
    loop {
        // Blink 10 times quickly (250ms on 250ms off)
        for _ in 0..10 {
            blink(&gpioc);
        }
        delay_ms(1000);
    }

}

fn blink(gpioc: &GPIOC) {
    // Turn led on
    gpioc.bsrr.write(|w| w.br13().set_bit());
    delay_ms(250);
    // Turn led off
    gpioc.bsrr.write(|w| w.bs13().set());
    delay_ms(250);
}

fn delay_ms(ms: usize) {
    tick_delay(1000 * ms);
}

fn tick_delay(ticks: usize) {
    (0..ticks).for_each(|_| asm::nop());
}