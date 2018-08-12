#![deny(warnings)]
#![no_std]
#![no_main]

// used to provide panic_implementation
#[allow(unused)]
use panic_abort;

// internal
mod ahrs;
mod bootloader;
mod constants;
mod esc;
mod logging;
mod motor;
mod utils;

// internal imports
use crate::bootloader::Bootloader;
use crate::esc::pwm::Controller as ESC;
use crate::motor::{brushed::Coreless as CorelessMotor, Motor};
use crate::utils::WallClockDelay;

// rust std/core
use core::fmt::Write;

// external
use hal::delay::Delay;
use hal::gpio::{gpiob, gpioc};
use hal::gpio::{AltFn, AF5, AF7};
use hal::gpio::{LowSpeed, MediumSpeed, Output, PullNone, PullUp, PushPull};
use hal::prelude::*;
use hal::serial::{self, Rx, Serial, Tx};
use hal::spi::Spi;
use hal::timer;
use mpu9250::Mpu9250;
use rt::{entry, exception, ExceptionFrame};
use stm32f30x::interrupt;

#[allow(unused)]
type SPI = Spi<hal::stm32f30x::SPI1,
               (gpiob::PB3<PullNone, AltFn<AF5, PushPull, LowSpeed>>,
               gpiob::PB4<PullNone, AltFn<AF5, PushPull, LowSpeed>>,
               gpiob::PB5<PullNone, AltFn<AF5, PushPull, LowSpeed>>)>;
#[allow(unused)]
type MPU9250 = mpu9250::Mpu9250<SPI,
                                gpiob::PB9<PullNone,
                                           Output<PushPull, LowSpeed>>,
                                mpu9250::Marg>;

type L = logging::SerialLogger<Tx<hal::stm32f30x::USART1>,
                               gpioc::PC14<PullNone,
                                           Output<PushPull, LowSpeed>>>;

static mut L: Option<L> = None;
static mut RX: Option<Rx<hal::stm32f30x::USART1>> = None;
static mut BOOTLOADER: Option<bootloader::stm32f30x::Bootloader> = None;
static mut ESC: Option<ESC> = None;
static mut MOTORS: Option<CorelessMotor> = None;

entry!(main);

fn main() -> ! {
    let device = hal::stm32f30x::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();
    let bootloader = bootloader::stm32f30x::Bootloader::new();

    let mut rcc = device.RCC.constrain();
    let gpioa = device.GPIOA.split(&mut rcc.ahb);
    let gpiob = device.GPIOB.split(&mut rcc.ahb);
    let gpioc = device.GPIOC.split(&mut rcc.ahb);

    let mut flash = device.FLASH.constrain();
    let clocks = rcc.cfgr
                    .sysclk(64.mhz())
                    .pclk1(32.mhz())
                    .pclk2(36.mhz())
                    .freeze(&mut flash.acr);

    let txpin = gpioa.pa9.alternating(AF7);
    let rxpin = gpioa.pa10.alternating(AF7);
    let mut serial = Serial::usart1(device.USART1,
                                    (txpin, rxpin),
                                    constants::BAUD_RATE,
                                    clocks,
                                    &mut rcc.apb2);
    serial.listen(serial::Event::Rxne);
    let (mut tx, rx) = serial.split();
    // COBS frame
    tx.write(0x00).unwrap();

    // XXX: split delay and pass to logger as well?
    let mut delay = Delay::new(core.SYST, clocks);

    let beeper = gpioc.pc14.output().pull_type(PullNone);
    let mut l = logging::SerialLogger::new(tx, beeper);
    write!(l, "Logger ok\r\n");

    // SPI1
    let ncs = gpiob.pb9.output().push_pull();
    let scl_sck = gpiob.pb3.alternating(AF5);
    let sda_sdi_mosi = gpiob.pb5.alternating(AF5);
    let ad0_sdo_miso = gpiob.pb4.alternating(AF5);
    let spi = Spi::spi1(device.SPI1,
                        (scl_sck, ad0_sdo_miso, sda_sdi_mosi),
                        mpu9250::MODE,
                        1.mhz(),
                        clocks,
                        &mut rcc.apb2);
    // MPU
    // XXX: catch error result, print and panic
    let mpu9250 = Mpu9250::marg(spi, ncs, &mut delay).unwrap();
    let loop_delay_ms: u32 = 50;
    let filter_freq = loop_delay_ms as f32 / 1000.;
    let mut ahrs = ahrs::AHRS::create_calibrated(mpu9250,
                                                 filter_freq,
                                                 constants::NSAMPLES,
                                                 &mut delay).unwrap();
    write!(l, "Calibration done, biases {:?}\r\n", ahrs.gyro_biases());

    // MOTORS:
    // pa0 -- pa3
    let (ch1, ch2, ch3, ch4, mut timer2) =
        timer::tim2::Timer::new(device.TIM2,
                                constants::TIM_TIMEOUT,
                                clocks,
                                &mut rcc.apb1).take_all();
    let mut motor_pa0 = gpioa.pa0.pull_type(PullUp).to_pwm(ch1, MediumSpeed);
    let mut motor_pa1 = gpioa.pa1.pull_type(PullUp).to_pwm(ch2, MediumSpeed);
    let mut motor_pa2 = gpioa.pa2.pull_type(PullUp).to_pwm(ch3, MediumSpeed);
    let mut motor_pa3 = gpioa.pa3.pull_type(PullUp).to_pwm(ch4, MediumSpeed);
    motor_pa0.enable();
    motor_pa1.enable();
    motor_pa2.enable();
    motor_pa3.enable();
    timer2.enable();

    let esc = ESC::new();
    let motor = CorelessMotor::new();

    unsafe {
        L = Some(l);
        BOOTLOADER = Some(bootloader);
        RX = Some(rx);
        ESC = Some(esc);
        MOTORS = Some(motor);
    }
    let l = unsafe { extract(&mut L) };

    write!(l, "init done\r\n");
    l.blink();

    delay.wc_delay_ms(5000);
    write!(l, "safety off\r\n");

    for i in 10..200 {
        motor_pa0.set_duty(i);
        motor_pa1.set_duty(i);
        motor_pa2.set_duty(i);
        motor_pa3.set_duty(i);
        utils::tick_delay(25000);
    }
    write!(l, "lift off\r\n");

    delay.wc_delay_ms(5000);

    for i in 10..200 {
        motor_pa0.set_duty(200 - i);
        motor_pa1.set_duty(200 - i);
        motor_pa2.set_duty(200 - i);
        motor_pa3.set_duty(200 - i);
        utils::tick_delay(25000);
    }
    write!(l, "take down\r\n");

    // unsafe { cortex_m::interrupt::enable() };
    // let mut nvic = core.NVIC;
    // let prio_bits = stm32f30x::NVIC_PRIO_BITS;
    // let hw = ((1 << prio_bits) - 1u8) << (8 - prio_bits);
    // unsafe { nvic.set_priority(Interrupt::USART1_EXTI25, hw) };
    // nvic.enable(Interrupt::USART1_EXTI25);

    // nvic.enable(Interrupt::EXTI0);
    // let mut timer4 =
    //     timer::tim4::Timer::new(device.TIM4, 8888.hz(), clocks, &mut
    // rcc.apb1);

    write!(l, "starting loop\r\n");
    loop {
        match ahrs.read() {
            Ok(q) => {
                write!(l, "quat: {:?}\r\n", q);
            },
            Err(e) => {
                write!(l, "ahrs error: {:?}\r\n", e);
            },
        }
        // delay.delay_ms(loop_delay_ms);
        // timer4.start(constants::TICK_TIMEOUT);
        // while let Err(nb::Error::WouldBlock) = timer4.wait() {}
        // c = (c + 1) % constants::TICK_PERIOD;
        // if c == 0 {
        //     write!(l, "tick\r\n");
        //     nvic.set_pending(Interrupt::EXTI0);
        // }
    }
}

unsafe fn extract<T>(opt: &'static mut Option<T>) -> &'static mut T {
    match opt {
        Some(ref mut x) => &mut *x,
        None => panic!("extract"),
    }
}

// interrupt!(EXTI0, exti0);
// fn exti0() {
// }

interrupt!(USART1_EXTI25, usart1_exti25);
fn usart1_exti25() {
    let rx = unsafe { extract(&mut RX) };
    let l = unsafe { extract(&mut L) };
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
            // echo byte as is
            l.write_one(b);
        },
        Err(nb::Error::Other(e)) => match e {
            serial::Error::Overrun => {
                rx.clear_overrun_error();
            },
            _ => {
                l.blink();
                write!(l, "read error: {:?}", e);
            },
        },
        Err(nb::Error::WouldBlock) => {},
    };
}

exception!(HardFault, hard_fault);
fn hard_fault(ef: &ExceptionFrame) -> ! {
    let l = unsafe { extract(&mut L) };
    l.blink();
    write!(l, "hard fault at {:?}", ef);
    panic!("HardFault at {:#?}", ef);
}

exception!(*, default_handler);
fn default_handler(irqn: i16) {
    let l = unsafe { extract(&mut L) };
    write!(l, "Interrupt: {}", irqn);
    panic!("Unhandled exception (IRQn = {})", irqn);
}
