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
#[macro_use]
mod toolbox;
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
use hal::stm32f30x::{self, interrupt, Interrupt};
use hal::timer;
use libm::F32Ext;
use mpu9250::Mpu9250;
use nalgebra::geometry::Quaternion;
use rt::{entry, exception, ExceptionFrame};
// conditional support
use cfg_if::cfg_if;

// usart1 is default, so should be last
cfg_if! {
    if #[cfg(feature = "usart2")] {
        use_serial!(USART2, usart_int);
    } else if #[cfg(feature = "usart1")] {
        use_serial!(USART1, usart_int);
    }
}

type L = logging::SerialLogger<Tx<USART>,
                               gpio::PC14<PullNone,
                                          Output<PushPull, LowSpeed>>>;

static mut L: Option<L> = None;
static mut RX: Option<Rx<USART>> = None;
static mut BOOTLOADER: Option<bootloader::stm32f30x::Bootloader> = None;
static mut ESC: Option<ESC> = None;
static mut MOTORS: Option<CorelessMotor> = None;

static mut P_KOEFF: f32 = 1000.;
static mut I_KOEFF: f32 = 0.;
static mut D_KOEFF: f32 = 0.;

static mut TOTAL_THRUST: f32 = 0.;
static mut CTL_STEP: f32 = 100.;

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
                    .pclk2(32.mhz())
                    .freeze(&mut flash.acr);

    let mut serial = init_serial!(device, gpioa, constants::BAUD_RATE, clocks);
    let serial_int = serial.get_interrupt();
    serial.listen(serial::Event::Rxne);
    let (mut tx, rx) = serial.split();
    // COBS frame
    tx.write(0x00).unwrap();

    let beeper = gpioc.pc14.output().pull_type(PullNone);
    let mut l = logging::SerialLogger::new(tx, beeper);
    write!(l, "Logger ok\r\n");
    // XXX: split delay and pass to logger as well?
    let mut delay = Delay::new(core.SYST, clocks);

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
    write!(l, "mpu ok, will proceed to calibration...\r\n");
    // mpu9250.calibrate_at_rest(&mut delay).unwrap();
    write!(l, "Calibration done, biases are stored\r\n");
    // let filter_freq = 4. / 1000.;
    // let mut ahrs = ahrs::AHRS::new(mpu9250, filter_freq);
    // write!(l, "Calibration done, biases {:?}\r\n", ahrs.gyro_biases());

    // MOTORS:
    // pa0 -- pa3
    let (ch1, ch2, ch3, ch4, mut timer2) =
        timer::tim2::Timer::new(device.TIM2,
                                constants::TIM_FREQ,
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
    unsafe { cortex_m::interrupt::enable() };
    let mut nvic = core.NVIC;
    nvic.enable(serial_int);

    delay.wc_delay_ms(2000);
    write!(l, "starting loop\r\n");
    let max_duty = m_rear_right.get_max_duty();
    write!(l, "max duty (arr): {}\r\n", max_duty);
    loop {
        // match ahrs.read() {
        //     Ok(q) => {
        //         write!(l, "w={};i={};j={};k={}\r\n", q.w, q.i, q.j, q.k);
        //         let (roll, yaw, pitch) = to_euler(l, &q);
        //         write!(l, "roll={},yaw={},pitch={}\r\n", roll, yaw, pitch);
        //     },
        //     Err(e) => {
        //         write!(l, "AHRS error: {:?}\r\n", e);
        //     },
        // }
        let mut delta = 0.;
        let mut prev_err = 0.;
        match mpu9250.all() {
            Ok(meas) => {
                let g = meas.gyro;
                let accel = meas.accel;
                // let x_err = 0. - g.x;
                let y_err = 0. - g.y;
                // let z_err = 0. - g.z;
                let pk = pkoef();
                let ik = ikoef();
                let dk = dkoef();

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
                m_rear_right.set_duty(rear_right as u32);
                m2_front_right.set_duty(front_right as u32);
                m3_rear_left.set_duty(rear_left as u32);
                m4_front_left.set_duty(front_left as u32);
            },
            Err(e) => {
                write!(l, "ahrs error: {:?}\r\n", e);
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

fn total_thrust() -> f32 {
    unsafe { TOTAL_THRUST }
}

fn ctl_step() -> f32 {
    unsafe { CTL_STEP }
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

fn usart_int() {
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
                        if (TOTAL_THRUST < 0.0) {
                            TOTAL_THRUST = 0.0;
                        }
                    }
                }
                t = !t;
            } else if b == constants::messages::PLUS_KY {
                unsafe {
                    D_KOEFF += (ctl_step() / 4.);
                }
                t = !t;
            } else if b == constants::messages::MINUS_KY {
                unsafe {
                    D_KOEFF -= (ctl_step() / 4.);
                }
                t = !t;
            }

            if t {
                write!(l,
                       "TTHRUST: {:?}; DK: {:?}; C: {:?}\r\n",
                       total_thrust(),
                       dkoef(),
                       ctl_step());
                t = false;
            } else {
                // echo byte as is
                l.write_char(b as char);
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
                    write!(l, "read error: {:?}\r\n", e);
                },
            }
        },
    };
}

fn to_euler(l: &mut L, q: &Quaternion<f32>) -> (f32, f32, f32) {
    let sqw = q.w * q.w;
    let sqx = q.i * q.i;
    let sqy = q.j * q.j;
    let sqz = q.k * q.k;
    let pitch_inp = -2. * (q.i * q.k - q.j * q.w);
    let pitch = pitch_inp.asin();
    let m = q.i * q.j + q.k * q.w;
    let mut roll;
    let mut yaw;
    if (m - 0.5).abs() < 1e-8 {
        roll = 0.;
        yaw = 2. * q.i.atan2(q.w);
    } else if (m + 0.5).abs() < 1e-8 {
        roll = -2. * q.i.atan2(q.w);
        yaw = 0.;
    } else {
        let roll_inp = 2. * (q.i * q.j + q.k * q.w);
        roll = roll_inp.atan2(sqx - sqy - sqz + sqw);
        let yaw_inp = 2. * (q.j * q.k + q.i * q.w);
        yaw = yaw_inp.atan2(-sqx - sqy + sqz + sqw);
    }
    (roll, pitch, yaw)
}

exception!(HardFault, |ef| {
    let l = unsafe { extract(&mut L) };
    l.blink();
    write!(l, "hard fault at {:?}\r\n", ef);
    panic!("HardFault at {:#?}", ef);
});

exception!(*, |irqn| {
    let l = unsafe { extract(&mut L) };
    write!(l, "Interrupt: {}\r\n", irqn);
    panic!("Unhandled exception (IRQn = {})", irqn);
});
