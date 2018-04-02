//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]

extern crate cortex_m;

extern crate embedded_hal as ehal;
extern crate stm32f30x_hal as hal;
extern crate mpu9250;

mod beeper;

use hal::prelude::*;
use hal::stm32f30x;
use cortex_m::peripheral::syst::SystClkSource;
use stm32f30x::NVIC_PRIO_BITS;
//use hal::delay::Delay;
//use mpu9250::Mpu9250;
//use hal::spi::Spi;
//use hal::serial::*;

fn init_system() {
    let dp = stm32f30x::Peripherals::take().unwrap();
    dp.RCC.apb1enr.modify(|_, w| w.pwren().enabled());
    dp.RCC.apb2enr.modify(|_, w| w.syscfgen().enabled());
    dp.RCC.ahbenr.modify(|_, w| w.iopeen().enabled());
    dp.RCC.ahbenr.modify(|_, w| w.iopaen().enabled());

    // USB is on PA 11/12 USB_DM/DP
    // in push-pull mode
    // in AF
    // without pull-up resistor
    // running at 50 MHz
    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let _dm = gpioa.pa11
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
        .into_af14(&mut gpioa.moder, &mut gpioa.afrh)
        .into_highspeed(&mut gpioa.ospeedr);
}

fn init_systick() {
    let dp = stm32f30x::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let freq = clocks.hclk();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut sys_tick = cp.SYST;
    // configure systick
    // TODO if tick > mask...
    match freq {
        hal::time::Hertz(i) => sys_tick.set_reload((i / 100) - 1)
    }

    let mut scb = cp.SCB;
    let priority = (1 << NVIC_PRIO_BITS) - 1;
    unsafe {
        // SysTick IRQ
        scb.shpr[11].write((priority << (8 - NVIC_PRIO_BITS)) & 0xff);
    };

    sys_tick.clear_current();
    sys_tick.set_clock_source(SystClkSource::Core);
    sys_tick.enable_interrupt();
    sys_tick.enable_counter();
}

fn main() {
    init_systick();
    init_system();
    /*
    let dp = stm32f30x::Peripherals::take().unwrap();
    // USB stuff
    let rss = dp.RSS.constrain();

    let mut rcc = dp.RCC.constrain();
    // GPs
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let gpioc = dp.GPIOC.split(&mut rcc.ahb);

    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(cp.SYST, clocks);

    let txpin = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rxpin = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);

    let uart = hal::serial::Serial::usart1(
        dp.USART1,
        (txpin, rxpin),
        hal::time::Bps(9600), clocks, &mut rcc.apb2);
    let (mut tx, _rx) = uart.split();

    // SPI1
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
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

    // assert_eq!(mpu9250.who_am_i().unwrap(), 0x71);
    // assert_eq!(mpu9250.ak8963_who_am_i().unwrap(), 0x48);

    let mut _beep = beeper::Beeper::new(gpioc);
    loop {
        let _res1  = tx.write(55);
        delay.delay_ms(1_000_u16);
        let _res2 = tx.flush();
    }
    */
}
