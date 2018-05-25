//#![deny(unsafe_code)]
// #![deny(warnings)]
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
extern crate stm32f30x_hal as hal;

mod beeper;
mod bootloader;
mod debug_writer;
mod itoa;
mod motors;

use bootloader::Bootloader;
use debug_writer::DebugWrite;
use motors::Motor;

use hal::gpio::gpiob;
use hal::gpio::{AF5, Output, PushPull};
use hal::prelude::*;
use hal::serial;
use hal::serial::{Rx, Serial, Tx};
use hal::spi::Spi;
use hal::stm32f30x;
use hal::time::Hertz;
use hal::timer::{self, Timer};

use cortex_m::asm;
use rt::ExceptionFrame;
// use mpu9250::Mpu9250;

const BAUD_RATE: hal::time::Bps = hal::time::Bps(9600);
const FREQ: u32 = 1024;
const BEEP_TIMEOUT: Hertz = Hertz(2);

type MPU9250 = mpu9250::Mpu9250<
    Spi<hal::stm32f30x::SPI1, (gpiob::PB3<AF5>, gpiob::PB4<AF5>, gpiob::PB5<AF5>)>,
    gpiob::PB9<Output<PushPull>>,
    mpu9250::Imu,
>;

type DW = debug_writer::DebugWriter<
    Tx<hal::stm32f30x::USART1>,
    hal::timer::Timer<cortex_m::peripheral::SYST>,
>;

// app!{
//     device: stm32f30x,

//     resources: {
//         static DW: DW;
//         static RX: Rx<hal::stm32f30x::USART1>;
//         // can't use bootloader trait here as the size of the resource
//         // should be known at compile time
//         static BOOTLOADER: bootloader::stm32f30x::Bootloader;
//         static MPU: MPU9250;
//         static MOTORS: motors::f3evo::MotorPWM;
//     },

//     tasks: {
//         USART1_EXTI25: {
//             path: echo,
//             resources: [DW, RX, BOOTLOADER, MOTORS],
//         },
//         SYS_TICK: {
//             path: tick,
//             resources: [MPU, DW],
//         },
//     }
// }

entry!(main);

fn main() -> ! {
    let device = stm32f30x::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();
    let bootloader = bootloader::stm32f30x::Bootloader::new();

    let mut rcc = device.RCC.constrain();
    let mut gpioa = device.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = device.GPIOB.split(&mut rcc.ahb);
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
    let nss = gpiob
        .pb9
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let sck = gpiob.pb3.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
    let miso = gpiob.pb4.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
    let mosi = gpiob.pb5.into_af5(&mut gpiob.moder, &mut gpiob.afrl);

    let spi = Spi::spi1(
        device.SPI1,
        (sck, miso, mosi),
        mpu9250::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    // let mpu9250 = Mpu9250::imu(spi, nss, &mut delay).unwrap();

    let tim2 = device.TIM2;
    let mut timer = Timer::tim2(tim2, FREQ.hz(), clocks, &mut rcc.apb1);
    timer.listen(timer::Event::TimeOut);
    let beep = beeper::Beeper::new(gpioc);

    let mut dw = debug_writer::DebugWriter::new(tx, timer, beep, BEEP_TIMEOUT);
    dw.debug('i');
    let motor = motors::f3evo::MotorPWM::new();

    loop {
        asm::bkpt();
    }
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

// // TASKS
// // Send back the received byte
// fn echo(_t: &mut Threshold, r: USART1_EXTI25::Resources) {
//     let mut rx = r.RX;
//     let mut dw = r.DW;
//     let mut bootloader = r.BOOTLOADER;
//     let mut motor = r.MOTORS;
//     match rx.read() {
//         Ok(b) => {
//             if b == 'r' as u8 {
//                 motor.write();
//                 bootloader.system_reset();
//             }

//             if b == 'R' as u8 {
//                 bootloader.to_bootloader();
//             }
//             dw.debug(b);
//         }
//         Err(nb::Error::Other(e)) => match e {
//             serial::Error::Framing => {
//                 dw.error('f');
//             }
//             serial::Error::Overrun => {
//                 rx.clear_overrun_error();
//             }
//             serial::Error::Parity => {
//                 dw.error('p');
//             }
//             serial::Error::Noise => {
//                 dw.error('n');
//             }
//             _ => {
//                 dw.error('u');
//             }
//         },
//         Err(nb::Error::WouldBlock) => {}
//     };
// }
