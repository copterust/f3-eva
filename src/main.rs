#![deny(warnings)]
#![no_std]
#![no_main]
#![feature(core_intrinsics)]
#![feature(panic_handler)]
#![feature(asm)]
#![feature(fn_traits, unboxed_closures)]
#![allow(unused)]

// internal
mod ahrs;
mod bootloader;
mod cmd;
mod constants;
mod esc;
mod logging;
mod motor;
#[macro_use]
mod toolbox;
mod utils;

// internal imports
use crate::bootloader::Bootloader;
use crate::cmd::Cmd;
use crate::constants as c;
use crate::esc::pwm::Controller as ESC;
use crate::motor::{brushed::Coreless as CorelessMotor, Motor};
use crate::utils::{vec_to_tuple, WallClockDelay};

// rust std/core
use core::fmt::Write;
use core::intrinsics;
use core::panic::PanicInfo;

// external
use hal::delay::Delay;
use hal::gpio::{self, AltFn, AF5, AF7};
use hal::gpio::{LowSpeed, MediumSpeed, Output, PullNone, PullUp, PushPull};
use hal::prelude::*;
use hal::serial::{self, Rx, Serial, Tx};
use hal::spi::Spi;
use hal::stm32f30x::{self, interrupt, Interrupt};
use hal::timer;

use mpu9250::Mpu9250;
use nalgebra::clamp;
use nalgebra::geometry::Quaternion;
use nalgebra::Vector3;
use rt::{entry, exception, ExceptionFrame};

#[cfg(feature = "usart1")]
use_serial!(USART1, usart_int, state: Option<Cmd> = None);
#[cfg(feature = "usart2")]
use_serial!(USART2, usart_int, state: Option<Cmd> = None);

type L = logging::SerialLogger<Tx<USART>,
                               gpio::PC14<PullNone,
                                          Output<PushPull, LowSpeed>>>;

static mut L: Option<L> = None;
static mut RX: Option<Rx<USART>> = None;
static mut BOOTLOADER: Option<bootloader::stm32f30x::Bootloader> = None;
static mut ESC: Option<ESC> = None;
static mut MOTORS: Option<CorelessMotor> = None;

static mut P_KOEFF: f32 = 0.;
static mut I_KOEFF: f32 = 0.;
static mut D_KOEFF: f32 = 0.;
static mut TOTAL_THRUST: f32 = 0.;

static mut NOW_MS: u32 = 0;
static mut PITCH_PWM: u32 = 0;
static mut STATUS_REQ: bool = false;

fn now_ms() -> u32 {
    unsafe { core::ptr::read_volatile(&NOW_MS as *const u32) }
}

exception!(SysTick, || {
    NOW_MS = NOW_MS.wrapping_add(1);
});

entry!(main);
fn main() -> ! {
    // first things first
    let mut bootloader = bootloader::stm32f30x::Bootloader::new();
    bootloader.check_request();
    let core = cortex_m::Peripherals::take().unwrap();
    let device = hal::stm32f30x::Peripherals::take().unwrap();

    let mut rcc = device.RCC.constrain();
    let gpioa = device.GPIOA.split(&mut rcc.ahb);
    let gpiob = device.GPIOB.split(&mut rcc.ahb);
    let gpioc = device.GPIOC.split(&mut rcc.ahb);

    let mut flash = device.FLASH.constrain();
    let clocks = rcc.cfgr
                    .sysclk(64.mhz())
                    .pclk1(32.mhz())
                    .pclk2(32.mhz())
                    .freeze(&mut flash.acr);

    let mut serial = init_serial!(device, gpioa, c::BAUD_RATE, clocks);
    let serial_int = serial.get_interrupt();
    serial.listen(serial::Event::Rxne);
    let (mut tx, rx) = serial.split();
    // COBS frame
    tx.write(0x00).unwrap();

    let beeper = gpioc.pc14.output().pull_type(PullNone);
    let mut l = logging::SerialLogger::new(tx, beeper);
    info!(l, "Logger ok\r\n");
    // XXX: split delay and pass to logger as well?
    let mut delay = Delay::new(core.SYST, clocks);

    // SPI1
    let ncs = gpiob.pb9.output().push_pull();
    let scl_sck = gpiob.pb3;
    let sda_sdi_mosi = gpiob.pb5;
    let ad0_sdo_miso = gpiob.pb4;
    let spi = device.SPI1.spi((scl_sck, ad0_sdo_miso, sda_sdi_mosi),
                              mpu9250::MODE,
                              1.mhz(),
                              clocks);
    info!(l, "spi ok\r\n");
    // MPU
    let mut mpu9250 =
        Mpu9250::imu_default(spi, ncs, &mut delay).expect("mpu error");
    let mut ahrs =
        ahrs::AHRS::create_calibrated(mpu9250, &mut delay, now_ms).expect("ahrs error");
    // MOTORS:
    // pa0 -- pa3
    let (ch1, ch2, ch3, ch4, mut timer2) =
        timer::tim2::Timer::new(device.TIM2,
                                c::TIM_FREQ,
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
    info!(l, "motors ok\r\n");

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

    info!(l, "init done\r\n");
    l.blink();
    unsafe { cortex_m::interrupt::enable() };
    let mut nvic = core.NVIC;
    nvic.enable(serial_int);

    delay.wc_delay_ms(2000);
    let max_duty = m_rear_right.get_max_duty();

    // Set systick to fire every ms
    let mut syst = delay.free();
    let reload = (clocks.sysclk().0 / 1000) - 1;
    syst.set_reload(reload);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();

    info!(l, "max duty (arr): {}\r\n", max_duty);
    loop {
        let mut delta = 0.;
        let mut prev_err = 0.;
        match ahrs.estimate() {
            Ok((dcm, dt_s)) => {
                // let x_err = 0. - g.x;
                // let z_err = 0. - g.z;
                debug!(l, "typr,{},{},{},{}\r\n", dt_s,
                       dcm.yaw, dcm.roll, dcm.pitch);
                let pk = pkoef();
                let ik = ikoef();
                let dk = dkoef();
                let y_err = 0. - dcm.pitch;
                let i_comp = 0.;
                delta = y_err - prev_err;
                let y_corr = y_err * pk + i_comp * ik + dk * delta;
                prev_err = y_err;
                let x_corr = 0.;
                let z_corr = 0.;
                let t = total_thrust();
                let front_left = t + x_corr + y_corr - z_corr;
                let front_right = t - x_corr + y_corr + z_corr;
                let rear_left = t + x_corr - y_corr + z_corr;
                let rear_right = t - x_corr - y_corr - z_corr;
                let pitch_cmd = pitch_pwm();
                if 0 == pitch_cmd {
                    let mmax = m_rear_right.get_max_duty() as f32;
                    m_rear_right.set_duty(clamp(rear_right, 0.0, mmax) as u32);
                    m2_front_right.set_duty(clamp(front_right, 0.0, mmax)
                                            as u32);
                    m3_rear_left.set_duty(clamp(rear_left, 0.0, mmax) as u32);
                    m4_front_left.set_duty(clamp(front_left, 0.0, mmax) as u32);
                } else {
                    m_rear_right.set_duty(pitch_cmd);
                    m3_rear_left.set_duty(pitch_cmd);
                }
                unsafe {
                    if STATUS_REQ == true {
                        STATUS_REQ = false;
                        info!(l, "Pitch: {}\r\n", dcm.pitch);
                        info!(l,
                               "Motors: {}, {}, {}, {} max {}\r\n",
                               m_rear_right.get_duty(),
                               m2_front_right.get_duty(),
                               m3_rear_left.get_duty(),
                               m4_front_left.get_duty(),
                               m4_front_left.get_max_duty());
                        info!(l,
                               "Tthrust: {}; dk: {}\r\n",
                               total_thrust(),
                               dkoef());
                    }
                }
            },
            Err(e) => {
                info!(l, "ahrs error: {:?}\r\n", e);
            },
        }
    }
}

unsafe fn extract<T>(opt: &'static mut Option<T>) -> &'static mut T {
    match opt {
        Some(ref mut x) => &mut *x,
        None => panic!("extract"),
    }
}

fn pitch_pwm() -> u32 {
    unsafe { PITCH_PWM }
}

fn total_thrust() -> f32 {
    unsafe { TOTAL_THRUST }
}

fn pkoef() -> f32 {
    unsafe { P_KOEFF }
}

fn ikoef() -> f32 {
    unsafe { I_KOEFF }
}

fn dkoef() -> f32 {
    unsafe { D_KOEFF }
}

fn process_cmd(cmd: &mut cmd::Cmd) {
    let rx = unsafe { extract(&mut RX) };
    let l = unsafe { extract(&mut L) };
    let bootloader = unsafe { extract(&mut BOOTLOADER) };
    let motor = unsafe { extract(&mut MOTORS) };
    match rx.read() {
        Ok(b) => {
            // first echo
            l.write_char(b as char);
            if let Some(word) = cmd.push(b) {
                // koeffs are parsed as i32 for simplicity
                parse!(word:
                       ["pitch_pwm=", pitch:i32] => {
                           unsafe {
                               PITCH_PWM = pitch as u32;
                           };
                       },
                       ["thrust=", thrust:i32] => {
                           unsafe {
                               TOTAL_THRUST = thrust as f32
                           };
                       },
                       ["dk=", dk:i32] => {
                           unsafe {
                               D_KOEFF = dk as f32
                           };
                       },
                       ["pk=", pk:i32] => {
                           unsafe {
                               P_KOEFF = pk as f32
                           };
                       },
                       ["ik=", ik:i32] => {
                           unsafe {
                               I_KOEFF = ik as f32
                           };
                       },
                       ["boot"] => {
                           bootloader.to_bootloader();
                       },
                       ["reset"] => {
                           bootloader.system_reset();
                       },
                       ["status"] => {
                           unsafe { STATUS_REQ = true; }
                       }
                );
            }
        },
        Err(nb::Error::WouldBlock) => {},
        Err(nb::Error::Other(e)) => {
            match e {
                serial::Error::Overrun => {
                    info!(l, "read error: {:?}\r\n", e);
                    rx.clear_overrun_error();
                },
                serial::Error::Framing => {
                    info!(l, "read error: {:?}\r\n", e);
                    rx.clear_framing_error();
                },
                serial::Error::Noise => {
                    info!(l, "read error: {:?}\r\n", e);
                    rx.clear_noise_error();
                },
                _ => {
                    l.blink();
                    info!(l, "read error: {:?}\r\n", e);
                },
            }
        },
    };
}

fn usart_int(state: &mut Option<cmd::Cmd>) {
    if state.is_none() {
        *state = Some(Cmd::new());
    }
    if let Some(cmd) = state.as_mut() {
        process_cmd(cmd)
    }
}

exception!(HardFault, |ef| {
    panic!("HardFault at {:#?}", ef);
});

exception!(*, |irqn| {
    panic!("Unhandled exception (IRQn = {})", irqn);
});

#[panic_handler]
fn panic(panic_info: &PanicInfo) -> ! {
    match unsafe { &mut L } {
        Some(ref mut l) => {
            let payload = panic_info.payload().downcast_ref::<&str>();
            match (panic_info.location(), payload) {
                (Some(location), Some(msg)) => {
                    error!(l,
                           "\r\npanic in file '{}' at line {}: {:?}\r\n",
                           location.file(),
                           location.line(),
                           msg);
                },
                (Some(location), None) => {
                    error!(l,
                           "panic in file '{}' at line {}",
                           location.file(),
                           location.line());
                },
                (None, Some(msg)) => {
                    error!(l, "panic: {:?}", msg);
                },
                (None, None) => {
                    error!(l, "panic occured, no info available");
                },
            }
        },
        None => {},
    }
    unsafe { intrinsics::abort() }
}
