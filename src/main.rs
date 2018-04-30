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
mod debug_writer;
mod itoa;

use debug_writer::DebugWrite;

use hal::prelude::*;
use hal::stm32f30x;
use hal::delay::Delay;
use hal::serial;
use hal::spi::Spi;
use hal::serial::{Serial, Tx, Rx};
use hal::gpio::gpiob;
use hal::gpio::{Output, PushPull, AF5};
use hal::timer::{self, Timer};
use hal::time::Hertz;

use mpu9250::Mpu9250;
use rtfm::{app, Threshold};

// Bootloader request stuff
use cortex_m::register::msp;

const BOOTLOADER_REQUEST:u32 = 1;

const BAUD_RATE: hal::time::Bps = hal::time::Bps(9600);
const FREQ: u32 = 1024;
const BEEP_TIMEOUT: Hertz = Hertz(2);

type MPU9250 = mpu9250::Mpu9250<
        Spi<hal::stm32f30x::SPI1, (gpiob::PB3<AF5>,
                                   gpiob::PB4<AF5>,
                                   gpiob::PB5<AF5>)>,
        gpiob::PB9<Output<PushPull>>,
        mpu9250::Imu>;

type DW = debug_writer::DebugWriter<
        Tx<hal::stm32f30x::USART1>,
        hal::timer::Timer<cortex_m::peripheral::SYST>>;

app!{
    device: stm32f30x,

    resources: {
        static DW: DW;
        static RX: Rx<hal::stm32f30x::USART1>;
        static MPU: MPU9250;
    },

    tasks: {
        USART1_EXTI25: {
            path: echo,
            resources: [DW, RX],
        },
        SYS_TICK: {
            path: tick,
            resources: [MPU, DW],
        },
    }
}

fn check_bootloader_request() {
    let bkp0r = unsafe { &(*stm32f30x::RTC::ptr()).bkp0r };
    if bkp0r.read().bits() == BOOTLOADER_REQUEST {
        bkp0r.write(|w| unsafe { w.bits(0) });
        unsafe {
            // TODO __enable_irq();
            msp::write(0x1FFFD800);
            let f = 0x1FFFD804u32 as *const fn();
            (*f)();
        }
        loop {}
    }
}

fn init(p: init::Peripherals) -> init::LateResources {
    check_bootloader_request();

    let mut rcc = p.device.RCC.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = p.device.GPIOB.split(&mut rcc.ahb);
    let gpioc = p.device.GPIOC.split(&mut rcc.ahb);

    let mut flash = p.device.FLASH.constrain();
    let clocks = rcc.cfgr
        .sysclk(64.mhz())
        .pclk1(32.mhz())
        .freeze(&mut flash.acr);

    let txpin = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rxpin = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let mut serial = Serial::usart1(
        p.device.USART1,
        (txpin, rxpin),
        BAUD_RATE, clocks, &mut rcc.apb2);
    serial.listen(serial::Event::Rxne);
    let (mut tx, rx) = serial.split();
    // COBS frame
    tx.write(0x00).unwrap();


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

    let mut delay = Delay::new(p.core.SYST, clocks);
    let mpu9250 = Mpu9250::imu(spi, nss, &mut delay).unwrap();
    let mut timer = Timer::syst(delay.free(), FREQ.hz(), clocks);
    timer.listen(timer::Event::TimeOut);
    let beep = beeper::Beeper::new(gpioc);

    let dw = debug_writer::DebugWriter::new(tx, timer, beep, BEEP_TIMEOUT);

    init::LateResources { DW: dw,
                          RX: rx,
                          MPU: mpu9250 }
}

// IDLE LOOP
fn idle() -> ! {
    // Sleep
    loop {
        rtfm::wfi();
    }
}

fn reset_to_bootloader() {
    // write cookie to backup register and reset
    // bkp0r::write(BOOTLOADER_REQUEST);
    system_reset();
}

fn system_reset() {
    let scb = cortex_m::peripheral::SCB::ptr();
    unsafe {
        (*scb).aircr.write(0x05FA0000 | 0x04u32);
    }
}

fn tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    let dw = &mut r.DW;
    let gyrox = match r.MPU.gx() {
        Ok(x) => { x }
        Err(_) => { 0 }
    };
    let data = itoa::itoa_i16(gyrox);
    dw.debug(&data[..]);
}

// TASKS
// Send back the received byte
fn echo(_t: &mut Threshold, r: USART1_EXTI25::Resources) {
    let mut rx = r.RX;
    let mut dw = r.DW;
    dw.beeper().off();
    match rx.read() {
        Ok(b) => {
            dw.beeper().on();

            if b == 'r' as u8 {
                system_reset();
            }

            if b == 'R' as u8 {
                reset_to_bootloader();
            }

            dw.debug(b);
        }
        Err(nb::Error::Other(e)) => {
            dw.beeper().on();
            match e {
                serial::Error::Framing => {
                    dw.error('f');
                }
                serial::Error::Overrun => {
                    rx.clear_overrun_error();
                }
                serial::Error::Parity => {
                    dw.error('p');
                }
                serial::Error::Noise => {
                    dw.error('n');
                }
                _ => {
                    dw.error('u');
                }
            }
        }
        Err(nb::Error::WouldBlock) => {
        }
    };
}
