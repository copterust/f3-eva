//#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![feature(proc_macro)]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate nb;
extern crate panic_abort;
extern crate embedded_hal as ehal;
extern crate stm32f30x_hal as hal;

// extern crate mpu9250;

mod beeper;

use hal::prelude::*;
use hal::stm32f30x;
use hal::delay::Delay;
// use mpu9250::Mpu9250;
use hal::serial;
use rtfm::{app, Threshold};

const BAUD_RATE: hal::time::Bps = hal::time::Bps(9600);

app!{
    device: stm32f30x,

    resources: {
        static TX: hal::serial::Tx<hal::stm32f30x::USART1>;
        static RX: hal::serial::Rx<hal::stm32f30x::USART1>;
        static BEEPER: beeper::Beeper;
        static DELAY: hal::delay::Delay;
    },

    tasks: {
        USART1_EXTI25: {
            path: echo,
            resources: [TX, RX, BEEPER, DELAY],
        },
    }
}

fn init(p: init::Peripherals) -> init::LateResources {
    let mut rcc = p.device.RCC.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.ahb);
    let gpioc = p.device.GPIOC.split(&mut rcc.ahb);

    let mut flash = p.device.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let delay = Delay::new(p.core.SYST, clocks);
    let txpin = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rxpin = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let mut serial = hal::serial::Serial::usart1(
        p.device.USART1,
        (txpin, rxpin),
        BAUD_RATE, clocks, &mut rcc.apb2);
    serial.listen(serial::Event::Rxne);
    let (tx, rx) = serial.split();
    let beep = beeper::Beeper::new(gpioc);
    init::LateResources { TX: tx, RX: rx, BEEPER: beep, DELAY: delay }
}

// IDLE LOOP
fn idle() -> ! {
    // Sleep
    loop {
        rtfm::wfi();
    }
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
                // TODO
                // movs r3, #0
                // ldr r3, [r3, #0]
                // MSR msp, r3
                unsafe {
                    let f = 0x00000004 as *const fn();
                    (*f)();
                }
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
