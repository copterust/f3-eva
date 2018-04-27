//#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![feature(proc_macro)]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate panic_abort;
extern crate embedded_hal as ehal;
extern crate stm32f30x_hal as hal;

extern crate mpu9250;

extern crate nb;


mod beeper;
mod itoa;

use hal::prelude::*;
use hal::stm32f30x;
use hal::delay::Delay;
use hal::serial;
use hal::spi::Spi;
use hal::serial::{Serial, Tx, Rx};
use hal::gpio::gpiob;
use hal::gpio::{Output, PushPull, AF5};

use mpu9250::Mpu9250;
use rtfm::{app, Threshold};
use cortex_m::register::msp;

const BAUD_RATE: hal::time::Bps = hal::time::Bps(9600);

type MPU9250 = mpu9250::Mpu9250<
        Spi<hal::stm32f30x::SPI1, (gpiob::PB3<AF5>,
                                   gpiob::PB4<AF5>,
                                   gpiob::PB5<AF5>)>,
    gpiob::PB9<Output<PushPull>>,
    mpu9250::Imu>;

app!{
    device: stm32f30x,

    resources: {
        static TX: Tx<hal::stm32f30x::USART1>;
        static RX: Rx<hal::stm32f30x::USART1>;
        static BEEPER: beeper::Beeper;
        static DELAY: hal::delay::Delay;
        static MPU: MPU9250;
    },

    tasks: {
        USART1_EXTI25: {
            path: echo,
            resources: [TX, RX, BEEPER, DELAY, MPU],
        },
        SYS_TICK: {
            path: tick,
            resources: [MPU, BEEPER, DELAY, TX],
        },
    }
}

fn init(p: init::Peripherals) -> init::LateResources {
    let mut rcc = p.device.RCC.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = p.device.GPIOB.split(&mut rcc.ahb);
    let gpioc = p.device.GPIOC.split(&mut rcc.ahb);

    let mut flash = p.device.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(p.core.SYST, clocks);
    let txpin = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rxpin = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let mut serial = Serial::usart1(
        p.device.USART1,
        (txpin, rxpin),
        BAUD_RATE, clocks, &mut rcc.apb2);
    serial.listen(serial::Event::Rxne);
    let (tx, rx) = serial.split();
    let beep = beeper::Beeper::new(gpioc);

    // SPI1
    let nss = gpiob.pb9.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let sck = gpiob.pb3.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
    let miso = gpiob.pb4.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
    let mosi = gpiob.pb5.into_af5(&mut gpiob.moder, &mut gpiob.afrl);

    let spi = Spi::spi1(
        p.device.SPI1,
        (sck, miso, mosi),
        mpu9250::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mpu9250 = Mpu9250::imu(spi, nss, &mut delay).unwrap();

    init::LateResources { TX: tx,
                          RX: rx,
                          BEEPER: beep,
                          DELAY: delay,
                          MPU: mpu9250 }
}

// IDLE LOOP
fn idle() -> ! {
    // Sleep
    loop {
        rtfm::wfi();
    }
}

fn system_reset() {
    let scb = cortex_m::peripheral::SCB::ptr();
    unsafe {
        (*scb).aircr.write(0x05FA0000 | 0x04u32);
    }
}

fn tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    let rb = &mut r.BEEPER;
    rb.on();
    let rd = &mut r.DELAY;
    let mut tx = r.TX;
    let gyrox = match r.MPU.gx() {
        Ok(x) => {
            x
        }
        Err(_) => {
            0
        }
    };
    let bb = itoa::itoa_i16(gyrox);
    for b in bb.iter() {
        wrt(&mut tx, b.clone(), rb, rd, 2000);
    }
    rb.off();
}

// TASKS
// Send back the received byte
fn echo(_t: &mut Threshold, mut r: USART1_EXTI25::Resources) {
    let rb = &mut r.BEEPER;
    let rd = &mut r.DELAY;
    let mut rx = r.RX;
    let mut tx = r.TX;
    rb.off();
    match rx.read() {
        Ok(b) => {
            if b == 'r' as u8 {
                rb.on();
                system_reset();
            }
            if b == 'b' as u8 {
                // TODO enable irq
                unsafe {
                    msp::write(0x1FFFD800);
                    let f = 0x1FFFD804u32 as *const fn();
                    (*f)();
                }
                loop {}
            }
            wrt(&mut tx, b, rb, rd, 2000);
        }
        Err(nb::Error::Other(e)) => {
            rb.on();
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
