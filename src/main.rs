//#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]
#![feature(proc_macro)]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate nb;
extern crate panic_abort;

extern crate embedded_hal as ehal;
extern crate mpu9250;
#[macro_use]
extern crate stm32f30x;
extern crate stm32f30x_hal as hal;

mod beeper;
mod bootloader;
mod debug_writer;
// mod itoa;
// mod motors;

use bootloader::Bootloader;
use debug_writer::DebugWrite;
// use motors::Motor;

// use hal::gpio::gpiob;
// use hal::gpio::{AF5, Output, PushPull};
use hal::prelude::*;
use hal::serial;
use hal::serial::{Rx, Serial, Tx};
// use hal::spi::Spi;
use hal::time::Hertz;
use hal::timer::{self, Timer};

// use cortex_m::asm;
use rt::ExceptionFrame;
use stm32f30x::Interrupt;
// use mpu9250::Mpu9250;

const BAUD_RATE: hal::time::Bps = hal::time::Bps(9600);
const FREQ: u32 = 1024;
const BEEP_TIMEOUT: Hertz = Hertz(2);

// type MPU9250 = mpu9250::Mpu9250<
//     Spi<hal::stm32f30x::SPI1, (gpiob::PB3<AF5>, gpiob::PB4<AF5>, gpiob::PB5<AF5>)>,
//     gpiob::PB9<Output<PushPull>>,
//     mpu9250::Imu,
// >;

type DW =
    debug_writer::DebugWriter<Tx<hal::stm32f30x::USART1>, hal::timer::Timer<hal::stm32f30x::TIM2>>;

static mut DW: Option<DW> = None;
static mut RX: Option<Rx<hal::stm32f30x::USART1>> = None;
static mut BOOTLOADER: Option<bootloader::stm32f30x::Bootloader> = None;
//         static MPU: MPU9250;
//         static MOTORS: motors::f3evo::MotorPWM;

entry!(main);

fn main() -> ! {
    let device = hal::stm32f30x::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();
    let bootloader = bootloader::stm32f30x::Bootloader::new();

    let mut rcc = device.RCC.constrain();
    let mut gpioa = device.GPIOA.split(&mut rcc.ahb);
    // let mut gpiob = device.GPIOB.split(&mut rcc.ahb);
    let gpioc = device.GPIOC.split(&mut rcc.ahb);

    let mut flash = device.FLASH.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(64.mhz())
        .pclk1(32.mhz())
        .freeze(&mut flash.acr);

    let txpin = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rxpin = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let mut serial = Serial::usart1(
        device.USART1,
        (txpin, rxpin),
        BAUD_RATE,
        clocks,
        &mut rcc.apb2,
    );
    serial.listen(serial::Event::Rxne);
    let (mut tx, rx) = serial.split();
    // COBS frame
    tx.write(0x00).unwrap();

    // SPI1
    // let nss = gpiob
    //     .pb9
    //     .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    // let sck = gpiob.pb3.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
    // let miso = gpiob.pb4.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
    // let mosi = gpiob.pb5.into_af5(&mut gpiob.moder, &mut gpiob.afrl);

    // let spi = Spi::spi1(
    //     device.SPI1,
    //     (sck, miso, mosi),
    //     mpu9250::MODE,
    //     1.mhz(),
    //     clocks,
    //     &mut rcc.apb2,
    // );

    // let mpu9250 = Mpu9250::imu(spi, nss, &mut delay).unwrap();

    let tim2 = device.TIM2;
    let mut timer = Timer::tim2(tim2, FREQ.hz(), clocks, &mut rcc.apb1);
    timer.listen(timer::Event::TimeOut);
    let beep = beeper::Beeper::new(gpioc);

    let mut dw = debug_writer::DebugWriter::new(tx, timer, beep, BEEP_TIMEOUT);
    dw.debug('i');
    dw.err_beep();
    // let motor = motors::f3evo::MotorPWM::new();

    unsafe {
        DW = Some(dw);
        BOOTLOADER = Some(bootloader);
        RX = Some(rx);
    }

    unsafe { cortex_m::interrupt::enable() };
    let mut nvic = core.NVIC;
    let prio_bits = stm32f30x::NVIC_PRIO_BITS;
    let hw = ((1 << prio_bits) - 1u8) << (8 - prio_bits);
    unsafe { nvic.set_priority(Interrupt::USART1_EXTI25, hw) };
    nvic.enable(Interrupt::USART1_EXTI25);

    loop {}
}

unsafe fn extract<T>(opt: &'static mut Option<T>) -> &'static mut T {
    match opt {
        Some(ref mut x) => &mut *x,
        None => panic!(),
    }
}

interrupt!(USART1_EXTI25, echo);
// Send back the received byte
fn echo() {
    let rx = unsafe { extract(&mut RX) };
    let dw = unsafe { extract(&mut DW) };
    let bootloader = unsafe { extract(&mut BOOTLOADER) };
    // let mut motor = r.MOTORS;
    match rx.read() {
        Ok(b) => {
            if b == 'r' as u8 {
                // motor.write();
                bootloader.system_reset();
            }
            if b == 'R' as u8 {
                bootloader.to_bootloader();
            }
            dw.debug(b);
        }
        Err(nb::Error::Other(e)) => match e {
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
        },
        Err(nb::Error::WouldBlock) => {}
    };
}

exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}

// fn tick(_t: &mut Threshold, _r: SYS_TICK::Resources) {
//     // let dw = &mut r.DW;
//     // let gyrox = match r.MPU.gx() {
//     //     Ok(x) => x,
//     //     Err(_) => 0,
//     // };
//     // let data = itoa::itoa_i16(gyrox);
//     // dw.debug(&data[..]);
// }
