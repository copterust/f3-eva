//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]

extern crate nb;
extern crate cortex_m;
extern crate panic_abort;

extern crate embedded_hal as ehal;
extern crate stm32f30x_hal as hal;
extern crate mpu9250;

mod beeper;

use hal::prelude::*;
use hal::stm32f30x;
use hal::delay::Delay;
// use mpu9250::Mpu9250;
use hal::serial;


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

    let txpin = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rxpin = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);

    let mut serial = hal::serial::Serial::usart1(
        dp.USART1,
        (txpin, rxpin),
        hal::time::Bps(9600), clocks, &mut rcc.apb2);
    serial.listen(serial::Event::Rxne);
    let (mut tx, mut rx) = serial.split();

    // SPI1
    // let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    // let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    // let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    // let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    // let spi = Spi::spi1(
    //     dp.SPI1,
    //     (sck, miso, mosi),
    //     mpu9250::MODE,
    //     1.mhz(),
    //     clocks,
    //     &mut rcc.apb2,
    // );

    // let mut mpu9250 = Mpu9250::marg(spi, nss, &mut delay).unwrap();

    // assert_eq!(mpu9250.who_am_i().unwrap(), 0x71);
    // assert_eq!(mpu9250.ak8963_who_am_i().unwrap(), 0x48);

    let mut beep = beeper::Beeper::new(gpioc);
    let rb = &mut beep;
    let rd = &mut delay;
    loop {
        match rx.read() {
            Ok(b) => {
                wrt(&mut tx, b, rb, rd, 2000);
            }
            Err(nb::Error::Other(e)) => {
                match e {
                    serial::Error::Framing => {
                        wrtc(&mut tx, 'f', rb, rd, 2000);
                        err(rb, rd, 2000);
                    }
                    serial::Error::Overrun => {
                        rx.clear_overrun_error();
                    }
                    serial::Error::Parity => {
                        wrtc(&mut tx, 'p', rb, rd, 2000);
                        err(rb, rd, 2000);
                    }
                    serial::Error::Noise => {
                        wrtc(&mut tx, 'n', rb, rd, 2000);
                        err(rb, rd, 2000);
                    }
                    _ => {
                        wrtc(&mut tx, 'u', rb, rd, 2000);
                        err(rb, rd, 2000);
                    }
                }
            }
            Err(nb::Error::WouldBlock) => {
            }
        };
    }
}

fn wrt(tx: &mut hal::serial::Tx<hal::stm32f30x::USART1>,
       data: u8,
       b: &mut beeper::Beeper, d: &mut Delay,
       t: u16) {
    match tx.write(data) {
        Ok(_) => {}
        Err(_) => { err(b, d, t); }
    }
}

fn wrtc(tx: &mut hal::serial::Tx<hal::stm32f30x::USART1>,
        data: char,
        b: &mut beeper::Beeper, d: &mut Delay,
        t: u16) {
    wrt(tx, data as u8, b, d, t);
}


fn err(b: &mut beeper::Beeper, d: &mut Delay, t: u16) {
    b.on();
    d.delay_ms(t);
    b.off();
}
