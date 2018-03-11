#![no_std]

extern crate cortex_m;

extern crate embedded_hal as ehal;
extern crate stm32f103xx_hal as hal;

use hal::prelude::*;
use hal::stm32f103xx;
use cortex_m::asm;


fn main() {
    let p = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // TIM2
    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

    // TIM3
    // let c1 = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
    // let c2 = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    // let c3 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    // let c4 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);

    // TIM4
    // let c1 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    // let c2 = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
    // let c3 = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
    // let c4 = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

    let mut pwms = p.TIM2
        .pwm((c1, c2, c3, c4),
            &mut afio.mapr,
            1.khz(),
            clocks,
            &mut rcc.apb1,
        );

    loop {
        for i in 0..4 {
            match i {
                0 => {control(&mut pwms.0);}
                1 => {control(&mut pwms.1);}
                2 => {control(&mut pwms.2);}
                3 => {control(&mut pwms.3);}
                _ => {}
            };

            delay_ms(1000);
        }
    }



}

fn control<'a, P:ehal::PwmPin<Duty=u16>>(pwm: &'a mut P)  {
    let max = pwm.get_max_duty();
    pwm.enable();
    for i in 1..max {
        pwm.set_duty(i);
        delay_ms(5);
    }
    for i in max..0 {
        pwm.set_duty(i);
        delay_ms(5);
    }
    pwm.disable()
}

fn delay_ms(ms: usize) {
    tick_delay(1000 * ms);
}

fn tick_delay(ticks: usize) {
    (0..ticks).for_each(|_| asm::nop());
}
