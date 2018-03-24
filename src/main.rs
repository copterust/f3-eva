#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]

extern crate cortex_m;

extern crate embedded_hal as ehal;
extern crate stm32f30x_hal as hal;
extern crate mpu9250;

mod beeper;

use hal::prelude::*;
use hal::stm32f30x;
use hal::delay::Delay;
use mpu9250::Mpu9250;
use hal::spi::Spi;


fn main() {
    let dp = stm32f30x::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    // GPs
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let gpioc = dp.GPIOC.split(&mut rcc.ahb);

    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(cp.SYST, clocks);

    let tx = gpioa
            .pa9
            .into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rx = gpioa
            .pa10
            .into_af7(&mut gpioa.moder, &mut gpioa.afrh);

    let _uart = hal::serial::Serial::usart1(dp.USART1, (tx, rx), hal::time::Bps(9600), clocks, &mut rcc.apb2);

    // SPI1
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl) ;//.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        mpu9250::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut mpu9250 = Mpu9250::marg(spi, nss, &mut delay).unwrap();

    assert_eq!(mpu9250.who_am_i().unwrap(), 0x71);
    assert_eq!(mpu9250.ak8963_who_am_i().unwrap(), 0x48);

    let mut beep = beeper::Beeper::new(gpioc);
    loop {
        delay.delay_ms(1_000_u16);
        beep.on();
        delay.delay_ms(1_000_u16);
        beep.off();
    }
}
