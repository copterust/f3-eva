//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_std]
#![no_main]
#![feature(use_extern_macros)]
#![feature(in_band_lifetimes)]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate nb;
extern crate panic_abort;

extern crate embedded_hal as ehal;
extern crate mpu9250;
#[macro_use]
extern crate stm32f30x;
extern crate alt_stm32f30x_hal as hal;
extern crate libm;

mod beeper;
mod bootloader;
mod constants;
mod debug_writer;
mod esc;
mod itoa;
mod kalman;
mod motor;

use bootloader::Bootloader;
use esc::pwm::Controller as ESC;
use kalman::Kalman;
use motor::brushed::Coreless as CorelessMotor;
use motor::Motor;

use core::f32::consts::PI;
use hal::delay::Delay;
use hal::gpio::{gpiob, gpioc};
use hal::gpio::{AF2, AF5, AF7, AltFn};
use hal::gpio::{LowSpeed, MediumSpeed, Output, PullNone, PushPull};
use hal::prelude::*;
use hal::pwm::PwmBinding;
use hal::serial::{self, Rx, Serial, Tx};
use hal::spi::Spi;
use hal::timer;
use libm::F32Ext;
use mpu9250::Mpu9250;
use rt::ExceptionFrame;
use stm32f30x::Interrupt;

type MPU9250 =
    mpu9250::Mpu9250<Spi<hal::stm32f30x::SPI1,
                         (gpiob::PB3<PullNone,
                                     AltFn<AF5, PushPull, LowSpeed>>,
                         gpiob::PB4<PullNone,
                                     AltFn<AF5, PushPull, LowSpeed>>,
                         gpiob::PB5<PullNone,
                                     AltFn<AF5, PushPull, LowSpeed>>)>,
                     gpiob::PB9<PullNone, Output<PushPull, LowSpeed>>,
                     mpu9250::Imu>;

type DW = debug_writer::DebugWriter<Tx<hal::stm32f30x::USART1>,
                                    timer::tim2::Timer<timer::ChannelFree,
                                                       timer::ChannelFree,
                                                       timer::ChannelFree,
                                                       timer::ChannelFree>>;

type PWM1 = PwmBinding<gpioc::PC6<PullNone, AltFn<AF2, PushPull, MediumSpeed>>,
                       timer::tim3::Channel<timer::CH1, timer::Pwm1>,
                       AF2>;

static mut DW: Option<DW> = None;
static mut RX: Option<Rx<hal::stm32f30x::USART1>> = None;
static mut BOOTLOADER: Option<bootloader::stm32f30x::Bootloader> = None;
static mut ESC: Option<ESC> = None;
static mut MOTORS: Option<CorelessMotor> = None;
static mut MPU: Option<MPU9250> = None;
static mut PWM1: Option<PWM1> = None;
static mut KALMAN: Option<Kalman> = None;

// gyroscope sensitivity
// XXX: move to KALMAN or constants
const K_G: f32 = 250. / (1 << 15) as f32;
const K_A: f32 = 2. / (1 << 15) as f32;

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

    let mut delay = Delay::new(core.SYST, clocks);
    let mut mpu9250 = Mpu9250::imu(spi, ncs, &mut delay).unwrap();
    mpu9250.a_scale(mpu9250::FSScale::_01).unwrap();
    mpu9250.g_scale(mpu9250::FSScale::_01).unwrap();

    // CALIBRATION & KALMAN FILTER INITIALIZATION
    // TODO: abstract mpu9250 as Accelerometer/Gyro and perform
    //       initialization elsewhere
    let (mut gx, mut ary, mut arz) = (0, 0, 0);
    const NSAMPLES: i32 = 128;
    for _ in 0..NSAMPLES {
        let (ary_, arz_, _, gx_) = mpu9250.aryz_t_gx().ok().unwrap();
        ary += ary_ as i32;
        arz += arz_ as i32;
        gx += gx_ as i32;
        delay.delay_ms(1_u8);
    }

    // average
    gx /= NSAMPLES;
    ary /= NSAMPLES;
    arz /= NSAMPLES;

    let gyro_bias = gx as f32 * K_G;
    let angle = (ary as f32 * K_A).atan2(arz as f32 * K_A) * 180. / PI;

    let kalman = Kalman::new(angle, gyro_bias);
    // XXX^ move above bullshit somewhere

    let timer2 = timer::tim2::Timer::new(device.TIM2,
                                         constants::DEBUG_TIMEOUT,
                                         clocks,
                                         &mut rcc.apb1);

    let dw = debug_writer::DebugWriter::new(tx,
                                            timer2,
                                            beeper::Beeper::new(gpioc.pc14),
                                            constants::DEBUG_TIMEOUT);

    let esc = ESC::new();
    let motor = CorelessMotor::new();

    let tim3 = timer::tim3::Timer::new(device.TIM3,
                                       constants::PWM_SPEED,
                                       clocks,
                                       &mut rcc.apb1);
    let (ch1, mut tim3) = tim3.take_ch1();
    tim3.enable();
    let pwm = PwmBinding::<gpioc::PC6<_, _>,
                         timer::tim3::Channel<timer::CH1, _>,
                         AF2>::new(gpioc.pc6, ch1);
    unsafe {
        DW = Some(dw);
        BOOTLOADER = Some(bootloader);
        RX = Some(rx);
        ESC = Some(esc);
        MOTORS = Some(motor);
        MPU = Some(mpu9250);
        PWM1 = Some(pwm);
        KALMAN = Some(kalman);
    }

    unsafe { cortex_m::interrupt::enable() };
    let mut nvic = core.NVIC;
    let prio_bits = stm32f30x::NVIC_PRIO_BITS;
    let hw = ((1 << prio_bits) - 1u8) << (8 - prio_bits);
    unsafe { nvic.set_priority(Interrupt::USART1_EXTI25, hw) };
    nvic.enable(Interrupt::USART1_EXTI25);
    nvic.enable(Interrupt::EXTI0);

    let mut timer4 = timer::tim4::Timer::new(device.TIM4,
                                             constants::TICK_TIMEOUT,
                                             clocks,
                                             &mut rcc.apb1);
    timer4.listen(timer::Event::TimeOut);

    let dw = unsafe { extract(&mut DW) };
    dw.debug(constants::messages::INIT);
    dw.blink();

    let mut c = -1;
    loop {
        timer4.start(constants::TICK_TIMEOUT);
        while let Err(nb::Error::WouldBlock) = timer4.wait() {}
        c = (c + 1) % constants::TICK_PERIOD;
        if c == 0 {
            dw.debug(constants::messages::TICK);
            // trigger the `EXTI0` interrupt
            nvic.set_pending(Interrupt::EXTI0);
        }
    }
}

unsafe fn extract<T>(opt: &'static mut Option<T>) -> &'static mut T {
    match opt {
        Some(ref mut x) => &mut *x,
        None => panic!("extract"),
    }
}

fn print_gyro(dw: &mut DW, mpu: &mut MPU9250) {
    match mpu.gyro() {
        Ok(g) => {
            dw.debug("gx:");
            dw.debug(itoa::itoa_i16(g.x).as_ref());
            dw.debug("; gy:");
            dw.debug(itoa::itoa_i16(g.y).as_ref());
            dw.debug("; gz:");
            dw.debug(itoa::itoa_i16(g.z).as_ref());
            dw.debug("\r\n");
        }
        Err(_) => {
            dw.debug(constants::messages::ERROR);
        }
    };
}

fn print_accel(dw: &mut DW, mpu: &mut MPU9250) {
    match mpu.accel() {
        Ok(a) => {
            dw.debug("ax:");
            dw.debug(itoa::itoa_i16(a.x).as_ref());
            dw.debug("; ay:");
            dw.debug(itoa::itoa_i16(a.y).as_ref());
            dw.debug("; az:");
            dw.debug(itoa::itoa_i16(a.z).as_ref());
            dw.debug("\r\n");
        }
        Err(_) => {
            dw.debug(constants::messages::ERROR);
        }
    };
}

fn do_pwm(dw: &mut DW, pwm: &mut PWM1) {
    dw.debug("c0:");
    dw.debug(itoa::itoa_u32(pwm.get_max_duty()).as_ref());
    pwm.set_duty(20000);
    dw.debug("set\r\n");
}

interrupt!(EXTI0, exti0);
fn exti0() {
    let mut dw = unsafe { extract(&mut DW) };
    let mut mpu = unsafe { extract(&mut MPU) };
    let mut pwm1 = unsafe { extract(&mut PWM1) };
    let kalman = unsafe { extract(&mut KALMAN) };

    let (ary, arz, _, gx) = mpu.aryz_t_gx().ok().unwrap();
    let omega = (gx as f32) * K_G;
    let angle = (ary as f32 * K_A).atan2(arz as f32 * K_A) * 180. / PI;
    let _estimate = kalman.update(angle, omega);
    print_gyro(&mut dw, &mut mpu);
    print_accel(&mut dw, &mut mpu);
    do_pwm(&mut dw, &mut pwm1);
    dw.debug("\r\n");
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
