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
mod constants;
mod debug_writer;
mod esc;
mod itoa;
mod motor;

use esc::pwm::Controller as ESC;
use motor::brushed::Coreless as CorelessMotor;
use motor::Motor;

use bootloader::Bootloader;
use debug_writer::DebugWrite;

use hal::delay::Delay;
use hal::gpio::gpiob;
use hal::gpio::{AF5, Output, PushPull};
use hal::prelude::*;
use hal::serial;
use hal::serial::{Rx, Serial, Tx};
use hal::spi::Spi;
use hal::timer::{self, Timer};

// use cortex_m::asm;
use mpu9250::Mpu9250;
use rt::ExceptionFrame;
use stm32f30x::Interrupt;

type MPU9250 = mpu9250::Mpu9250<
    Spi<hal::stm32f30x::SPI1, (gpiob::PB3<AF5>, gpiob::PB4<AF5>, gpiob::PB5<AF5>)>,
    gpiob::PB9<Output<PushPull>>,
    mpu9250::Imu,
>;

type DW =
    debug_writer::DebugWriter<Tx<hal::stm32f30x::USART1>, hal::timer::Timer<hal::stm32f30x::TIM2>>;

static mut DW: Option<DW> = None;
static mut RX: Option<Rx<hal::stm32f30x::USART1>> = None;
static mut BOOTLOADER: Option<bootloader::stm32f30x::Bootloader> = None;
static mut ESC: Option<ESC> = None;
static mut MOTORS: Option<CorelessMotor> = None;
static mut MPU: Option<MPU9250> = None;

entry!(main);

fn main() -> ! {
    let device = hal::stm32f30x::Peripherals::take().unwrap();
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
        constants::BAUD_RATE,
        clocks,
        &mut rcc.apb2,
    );
    serial.listen(serial::Event::Rxne);
    let (mut tx, rx) = serial.split();
    // COBS frame
    tx.write(0x00).unwrap();

    // SPI1
    let ncs = gpiob
        .pb9
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let scl_sck = gpiob.pb3.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
    let sda_sdi_mosi = gpiob.pb5.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
    let ad0_sdo_miso = gpiob.pb4.into_af5(&mut gpiob.moder, &mut gpiob.afrl);

    let spi = Spi::spi1(
        device.SPI1,
        (scl_sck, ad0_sdo_miso, sda_sdi_mosi),
        mpu9250::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut delay = Delay::new(core.SYST, clocks);
    let mut mpu9250 = Mpu9250::imu(spi, ncs, &mut delay).unwrap();
    mpu9250.a_scale(mpu9250::FSScale::_01).unwrap();
    mpu9250.g_scale(mpu9250::FSScale::_01).unwrap();

    let mut timer2 = Timer::tim2(device.TIM2, constants::DEBUG_TIMEOUT, clocks, &mut rcc.apb1);
    timer2.listen(timer::Event::TimeOut);
    let beep = beeper::Beeper::new(gpioc);
    let dw = debug_writer::DebugWriter::new(tx, timer2, beep, constants::DEBUG_TIMEOUT);

    let esc = ESC::new();
    let motor = CorelessMotor::new();

    unsafe {
        DW = Some(dw);
        BOOTLOADER = Some(bootloader);
        RX = Some(rx);
        ESC = Some(esc);
        MOTORS = Some(motor);
        MPU = Some(mpu9250);
    }

    unsafe { cortex_m::interrupt::enable() };
    let mut nvic = core.NVIC;
    let prio_bits = stm32f30x::NVIC_PRIO_BITS;
    let hw = ((1 << prio_bits) - 1u8) << (8 - prio_bits);
    unsafe { nvic.set_priority(Interrupt::USART1_EXTI25, hw) };
    nvic.enable(Interrupt::USART1_EXTI25);
    nvic.enable(Interrupt::EXTI0);

    let mut timer3 = Timer::tim3(device.TIM3, constants::TICK_TIMEOUT, clocks, &mut rcc.apb1);
    timer3.listen(timer::Event::TimeOut);

    let dw = unsafe { extract(&mut DW) };
    dw.debug(constants::messages::INIT);
    dw.blink();

    let mut c = 0;
    loop {
        timer3.start(constants::TICK_TIMEOUT);
        while let Err(nb::Error::WouldBlock) = timer3.wait() {}
        c += 1;
        if c == 10 {
            dw.debug(constants::messages::TICK);
            // trigger the `EXTI0` interrupt
            nvic.set_pending(Interrupt::EXTI0);
            c = 0;
        }
    }
}

unsafe fn extract<T>(opt: &'static mut Option<T>) -> &'static mut T {
    match opt {
        Some(ref mut x) => &mut *x,
        None => panic!("extract"),
    }
}

interrupt!(EXTI0, exti0);
fn exti0() {
    let dw = unsafe { extract(&mut DW) };
    let mpu = unsafe { extract(&mut MPU) };
    match mpu.who_am_i() {
        Ok(g) => {
            let data = itoa::itoa_u8(g);
            dw.debug(data.as_ref());
        }
        Err(_) => {
            dw.debug(constants::messages::ERROR);
        }
    };
}

interrupt!(USART1_EXTI25, usart1_exti25);
// Send back the received byte
fn usart1_exti25() {
    let rx = unsafe { extract(&mut RX) };
    let dw = unsafe { extract(&mut DW) };
    let bootloader = unsafe { extract(&mut BOOTLOADER) };
    let motor = unsafe { extract(&mut MOTORS) };
    match rx.read() {
        Ok(b) => {
            if b == constants::messages::RESET {
                bootloader.system_reset();
            }
            if b == constants::messages::BOOTLOADER {
                bootloader.to_bootloader();
            }
            if b == constants::messages::MOTOR {
                motor.set_rpm(1.0);
            }
            dw.debug(b);
        }
        Err(nb::Error::Other(e)) => match e {
            serial::Error::Framing => {
                dw.error(constants::messages::FRAMING_ERROR);
            }
            serial::Error::Overrun => {
                rx.clear_overrun_error();
            }
            serial::Error::Parity => {
                dw.error(constants::messages::PARITY_ERROR);
            }
            serial::Error::Noise => {
                dw.error(constants::messages::NOISE);
            }
            _ => {
                dw.error(constants::messages::UNKNOWN_ERROR);
            }
        },
        Err(nb::Error::WouldBlock) => {}
    };
}

exception!(HardFault, hard_fault);
fn hard_fault(ef: &ExceptionFrame) -> ! {
    let dw = unsafe { extract(&mut DW) };
    dw.error(constants::messages::HARD_FAULT);
    panic!("HardFault at {:#?}", ef);
}

exception!(*, default_handler);
fn default_handler(irqn: i16) {
    let dw = unsafe { extract(&mut DW) };
    let is = itoa::itoa_i16(irqn);
    dw.debug(constants::messages::DEFAULT_INTERRUPT);
    dw.debug(is.as_ref());
    panic!("Unhandled exception (IRQn = {})", irqn);
}
