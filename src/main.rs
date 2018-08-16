#![deny(warnings)]
#![no_std]
#![no_main]
#![allow(unused)]

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
use hal::gpio::{self, AltFn, AF5, AF7};
use hal::gpio::{LowSpeed, MediumSpeed, Output, PullNone, PullUp, PushPull};
use hal::prelude::*;
use hal::serial::{self, Rx, Serial, Tx};
use hal::spi::Spi;
use hal::timer;
use mpu9250::Mpu9250;
use rt::{entry, exception, ExceptionFrame};
use stm32f30x::{interrupt, Interrupt};

type USART = hal::stm32f30x::USART2;

type SPI = Spi<hal::stm32f30x::SPI1,
               (gpio::PB3<PullNone, AltFn<AF5, PushPull, LowSpeed>>,
               gpio::PB4<PullNone, AltFn<AF5, PushPull, LowSpeed>>,
               gpio::PB5<PullNone, AltFn<AF5, PushPull, LowSpeed>>)>;
type MPU9250 = mpu9250::Mpu9250<SPI,
                                gpio::PB9<PullNone,
                                          Output<PushPull, LowSpeed>>,
                                mpu9250::Marg>;

type L = logging::SerialLogger<Tx<USART>,
                               gpio::PC14<PullNone,
                                          Output<PushPull, LowSpeed>>>;

static mut L: Option<L> = None;
static mut RX: Option<Rx<USART>> = None;
static mut BOOTLOADER: Option<bootloader::stm32f30x::Bootloader> = None;
static mut ESC: Option<ESC> = None;
static mut MOTORS: Option<CorelessMotor> = None;

static mut KOEFF: f32 = 10.;
static mut TOTAL_THRUST: f32 = 0.;
static mut CTL_STEP: f32 = 10.;

const X_THRUST: f32 = 0.;
const Z_THRUST: f32 = 0.;

entry!(main);
fn main() -> ! {
    let device = hal::stm32f30x::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();
    let bootloader = bootloader::stm32f30x::Bootloader::new(core.SCB);

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

    let txpin = gpioa.pa14.alternating(AF7);
    let rxpin = gpioa.pa15.alternating(AF7);
    let mut serial = Serial::usart2(device.USART2,
                                    (txpin, rxpin),
                                    constants::BAUD_RATE,
                                    clocks,
                                    &mut rcc.apb1);
    serial.listen(serial::Event::Rxne);
    let (mut tx, rx) = serial.split();
    // COBS frame
    tx.write(0x00).unwrap();

    let beeper = gpioc.pc14.output().pull_type(PullNone);
    let mut l = logging::SerialLogger::new(tx, beeper);
    write!(l, "Logger ok\r\n");
    // XXX: split delay and pass to logger as well?
    let mut delay = Delay::new(core.SYST, clocks);
    delay.wc_delay_ms(5000);

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
    write!(l, "spi ok\r\n");
    // MPU
    // XXX: catch error result, print and panic
    let mut mpu9250 = Mpu9250::imu_default(spi, ncs, &mut delay).unwrap();
    write!(l, "mpu ok\r\n");
    mpu9250.sample_rate_divisor(4);
    let gyro_biases =
        ahrs::calibrate(&mut mpu9250, constants::NSAMPLES, &mut delay).unwrap();
    write!(l, "Calibration done, biases {:?}\r\n", gyro_biases);
    let loop_delay_ms: u32 = 50;
    // let filter_freq = loop_delay_ms as f32 / 1000.;
    // let ahrs = ahrs::AHRS::create_calibrated(mpu9250,
    //                                          filter_freq,
    //                                          constants::NSAMPLES,
    //                                          &mut delay).unwrap();
    // write!(l, "Calibration done, biases {:?}\r\n", ahrs.gyro_biases());

    // MOTORS:
    // pa0 -- pa3
    let (ch1, ch2, ch3, ch4, mut timer2) =
        timer::tim2::Timer::new(device.TIM2,
                                constants::TIM_TIMEOUT,
                                clocks,
                                &mut rcc.apb1).take_all();
    let mut m_rear_right = gpioa.pa0.pull_type(PullUp).to_pwm(ch1, MediumSpeed);
    let mut m2_front_right =
        gpioa.pa1.pull_type(PullUp).to_pwm(ch2, MediumSpeed);
    let mut m3_rear_left = gpioa.pa2.pull_type(PullUp).to_pwm(ch3, MediumSpeed);
    let mut m4_front_left =
        gpioa.pa3.pull_type(PullUp).to_pwm(ch4, MediumSpeed);
    m_rear_right.enable();
    m2_front_right.enable();
    m3_rear_left.enable();
    m4_front_left.enable();
    timer2.enable();
    write!(l, "motors ok\r\n");

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
    // delay.wc_delay_ms(5000);
    write!(l, "safety off\r\n");
    // for i in 10..200 {
    //     motor_pa0.set_duty(i);
    //     motor_pa1.set_duty(i);
    //     motor_pa2.set_duty(i);
    //     motor_pa3.set_duty(i);
    //     utils::tick_delay(25000);
    // }
    write!(l, "lift off\r\n");
    // delay.wc_delay_ms(5000);
    // for i in 10..200 {
    //     motor_pa0.set_duty(200 - i);
    //     motor_pa1.set_duty(200 - i);
    //     motor_pa2.set_duty(200 - i);
    //     motor_pa3.set_duty(200 - i);
    //     utils::tick_delay(25000);
    // }
    write!(l, "take down\r\n");

    unsafe { cortex_m::interrupt::enable() };
    let mut nvic = core.NVIC;
    let prio_bits = stm32f30x::NVIC_PRIO_BITS;
    let hw = ((1 << prio_bits) - 1u8) << (8 - prio_bits);
    unsafe { nvic.set_priority(Interrupt::USART2_EXTI26, hw) };
    nvic.enable(Interrupt::USART2_EXTI26);

    // nvic.enable(Interrupt::EXTI0);
    // let mut timer4 =
    //     timer::tim4::Timer::new(device.TIM4, 8888.hz(), clocks, &mut
    // rcc.apb1);

    write!(l, "starting loop\r\n");

    loop {
        match mpu9250.gyro() {
            Ok(g) => {
                let corrected = mpu9250::F32x3 { x: g.x - gyro_biases.x,
                                                 y: g.y - gyro_biases.y,
                                                 z: g.z - gyro_biases.z, };
                // let x_err = 0. - corrected.x;
                let y_err = 0. - corrected.y;
                // let z_err = 0. - corrected.z;
                let k = koef();
                let x_corr = 0.;
                let y_corr = y_err * k;
                let z_corr = 0.;
                let t = total_thrust();
                let front_left = t + x_corr + y_corr - z_corr;
                let front_right = t - x_corr + y_corr + z_corr;
                let rear_left = t + x_corr - y_corr + z_corr;
                let rear_right = t - x_corr - y_corr - z_corr;
                m_rear_right.set_duty(rear_right as u32);
                m2_front_right.set_duty(front_right as u32);
                m3_rear_left.set_duty(rear_left as u32);
                m4_front_left.set_duty(front_left as u32);
            },
            Err(e) => {
                write!(l, "ahrs error: {:?}\r\n", e);
            },
        }

        // match ahrs.read() {
        //     Ok(q) => {
        //         write!(l, "quat: {:?}\r\n", q);
        //     },
        //     Err(e) => {
        //         write!(l, "ahrs error: {:?}\r\n", e);
        //     },
        // }
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

fn total_thrust() -> f32 {
    unsafe { TOTAL_THRUST }
}

fn ctl_step() -> f32 {
    unsafe { CTL_STEP }
}

fn koef() -> f32 {
    unsafe { KOEFF }
}

interrupt!(USART2_EXTI26, usart_exti25);
fn usart_exti25() {
    let rx = unsafe { extract(&mut RX) };
    let l = unsafe { extract(&mut L) };
    let bootloader = unsafe { extract(&mut BOOTLOADER) };
    let motor = unsafe { extract(&mut MOTORS) };
    let mut t = false;
    match rx.read() {
        Ok(b) => {
            if b == constants::messages::RESET {
                bootloader.system_reset();
            } else if b == constants::messages::BOOTLOADER {
                bootloader.to_bootloader();
            } else if b == constants::messages::PLUS_T {
                unsafe {
                    TOTAL_THRUST += ctl_step();
                }
                t = !t;
            } else if b == constants::messages::MINUS_T {
                unsafe {
                    if (TOTAL_THRUST > 0.0) {
                        TOTAL_THRUST -= ctl_step();
                    }
                }
                t = !t;
            } else if b == constants::messages::PLUS_KY {
                unsafe {
                    KOEFF += ctl_step();
                }
                t = !t;
            } else if b == constants::messages::MINUS_KY {
                unsafe {
                    if (KOEFF > 0.0) {
                        KOEFF -= ctl_step();
                    }
                }
                t = !t;
            } else if b == constants::messages::CTL_1 {
                unsafe {
                    CTL_STEP = 1.0;
                }
                t = !t;
            } else if b == constants::messages::CTL_5 {
                unsafe {
                    CTL_STEP = 5.0;
                }
                t = !t;
            }

            if t {
                write!(l,
                       "TTHRUST: {:?}; KY: {:?}; C: {:?}\r\n",
                       total_thrust(),
                       koef(),
                       ctl_step());
                t = false;
            } else {
                // echo byte as is
                l.write_one(b);
            }
        },
        Err(nb::Error::WouldBlock) => {},
        Err(nb::Error::Other(e)) => {
            match e {
                serial::Error::Overrun => {
                    rx.clear_overrun_error();
                },
                serial::Error::Framing => {
                    rx.clear_framing_error();
                },
                serial::Error::Noise => {
                    rx.clear_noise_error();
                },
                _ => {
                    l.blink();
                    write!(l, "read error: {:?}", e);
                },
            }
        },
    };
}

exception!(HardFault, |ef| {
    let l = unsafe { extract(&mut L) };
    l.blink();
    write!(l, "hard fault at {:?}", ef);
    panic!("HardFault at {:#?}", ef);
});

exception!(*, |irqn| {
    let l = unsafe { extract(&mut L) };
    write!(l, "Interrupt: {}", irqn);
    panic!("Unhandled exception (IRQn = {})", irqn);
});
