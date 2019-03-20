#![deny(warnings)]
#![no_std]
#![no_main]
#![feature(core_intrinsics)]
#![feature(asm)]
#![feature(const_fn)]
#![feature(fn_traits, unboxed_closures)]
#![feature(impl_trait_in_bindings)]
#![feature(existential_type)]
#![allow(unused)]

// internal
mod bootloader;
mod chrono;
mod cmd;
mod constants;
mod esc;
mod logging;
mod motor;
mod utils;
#[macro_use]
mod toolbox;
mod ahrs;
mod altitude;
mod controller;
mod mixer;

// internal imports
use crate::bootloader::Bootloader;
use crate::chrono::{Chrono, RtfmClock};
use crate::cmd::Cmd;
use crate::constants as c;
use crate::esc::pwm::Controller as ESC;
use crate::mixer::{Map6, MotorMixer};
use crate::motor::{brushed::Coreless as CorelessMotor, Motor};
use crate::utils::{vec_to_tuple, WallClockDelay};

// rust std/core
use core::fmt::Write;
use core::intrinsics;
use core::panic::PanicInfo;

// external
use asm_delay::AsmDelay;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use ehal;
use hal;
use hal::delay::Delay;
use hal::gpio::{self, AltFn, AF1, AF2, AF5, AF7};
use hal::gpio::{HighSpeed, LowSpeed, MediumSpeed};
use hal::gpio::{Output, PullNone, PullUp, PushPull};
use hal::prelude::*;
use hal::pwm::PwmBinding;
use hal::serial::{self, Rx, Serial, Tx};
use hal::spi::Spi;
use hal::stm32f30x::{self, interrupt, Interrupt};
use hal::timer::{self, tim2, tim3, Pwm1};
use nalgebra::{self, clamp};
use nb;
use rtfm::{app, Instant};

// devices
// use bmp280::{self, BMP280};
use mpu9250::Mpu9250;
// use lsm303c::Lsm303c;
// use shared_bus::CortexMBusManager as SharedBus;
use vl53l0x;

#[cfg(feature = "usart1")]
use_serial!(USART1, process_cmd);
#[cfg(feature = "usart2")]
use_serial!(USART2, process_cmd);

pub type Vector3 = nalgebra::Vector3<f32>;
pub type Vector2 = nalgebra::Vector2<f32>;

type MotorPins =
    (PwmBinding<gpio::PA0<PullUp, AltFn<AF1, PushPull, MediumSpeed>>,
                tim2::Channel<timer::CH1, Pwm1>,
                AF1>,
     PwmBinding<gpio::PA1<PullUp, AltFn<AF1, PushPull, MediumSpeed>>,
                tim2::Channel<timer::CH2, Pwm1>,
                AF1>,
     PwmBinding<gpio::PA2<PullUp, AltFn<AF1, PushPull, MediumSpeed>>,
                tim2::Channel<timer::CH3, Pwm1>,
                AF1>,
     PwmBinding<gpio::PA3<PullUp, AltFn<AF1, PushPull, MediumSpeed>>,
                tim2::Channel<timer::CH4, Pwm1>,
                AF1>,
     PwmBinding<gpio::PA6<PullUp, AltFn<AF2, PushPull, MediumSpeed>>,
                tim3::Channel<timer::CH1, Pwm1>,
                AF2>,
     PwmBinding<gpio::PA7<PullUp, AltFn<AF2, PushPull, MediumSpeed>>,
                tim3::Channel<timer::CH2, Pwm1>,
                AF2>);
type MotorMixerT = MotorMixer<Map6, MotorPins>;

pub type L = logging::SerialLogger<Tx<USART>,
                                   gpio::PC14<PullNone,
                                              Output<PushPull, LowSpeed>>>;

type SPI = Spi<hal::stm32f30x::SPI1,
               (gpio::PB3<PullNone, AltFn<AF5, PushPull, HighSpeed>>,
                gpio::PB4<PullNone, AltFn<AF5, PushPull, HighSpeed>>,
                gpio::PB5<PullNone, AltFn<AF5, PushPull, HighSpeed>>)>;
type MpuDev =
    mpu9250::SpiDevice<SPI, gpio::PB9<PullNone, Output<PushPull, LowSpeed>>>;

pub const START_THRUST: f32 = 1350.;
pub const MAX_THRUST: f32 = 1950.;
pub const G: f32 = 9.81;
pub const RHO: f32 = 1.292;

// XXX: temporary for RTFM transition
// XXX: replace with proper controllers already!
struct ContCoeffs {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub pitch_p: f32,
    pub yaw_p: f32,
    pub roll_p: f32,
    pub total_thrust: f32,
}

impl ContCoeffs {
    const fn new() -> Self {
        ContCoeffs { p: 0.,
                     i: 0.,
                     d: 0.,
                     pitch_p: 0.,
                     yaw_p: 0.,
                     roll_p: 0.,
                     total_thrust: 0. }
    }
}

#[app(device = stm32f30x)]
const APP: () = {
    static mut CMD: Cmd = ();
    static mut L: L = ();
    static mut RX: Rx<USART> = ();
    static mut BOOTLOADER: bootloader::stm32f30x::Bootloader = ();
    // static mut ESC: ESC = ();
    // static mut MOTORS: CorelessMotor = ();
    static mut MOTOR_MIXER: MotorMixerT = ();
    static mut AHRS: ahrs::AHRS<MpuDev, chrono::T> = ();

    static mut CONT_COEFFS: ContCoeffs = ContCoeffs::new();
    static mut STATUS_REQ: bool = false;

    static mut TAKEOFF: bool = false;
    static mut TAKEOFF_DURATION_S: f32 = 3.0;
    static mut TAKEOFF_TO: f32 = 0.10;

    static mut ALT_CONTROLLER: controller::Controller = ();
    static mut TOF: vl53l0x::VL53L0x<I2C> = ();
    static mut INITIAL_ALTITUDE: f32 = ();
    static mut ALTITUDE: f32 = 0.;
    static mut VERT_VELOCITY: f32 = 0.;
    static mut ALT_CLOCK: chrono::T = ();
    static mut ELAPSED: f32 = 0.;

    #[init]
    fn init() -> init::LateResources {
        // first things first
        let mut bootloader = bootloader::stm32f30x::Bootloader::new();
        bootloader.check_request();

        // core.DCB.enable_trace(); // required for DWT cycle clounter to work
        // when not connected to the                          //
        // debugger core.DWT.enable_cycle_counter();

        let freq = 64.mhz();
        // let core = cortex_m::Peripherals::take().unwrap();
        let device: stm32f30x::Peripherals = device;

        let mut rcc = device.RCC.constrain();
        let gpioa = device.GPIOA.split(&mut rcc.ahb);
        let gpiob = device.GPIOB.split(&mut rcc.ahb);
        let gpioc = device.GPIOC.split(&mut rcc.ahb);

        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr
                        .sysclk(freq)
                        .pclk1(32.mhz())
                        .pclk2(32.mhz())
                        .freeze(&mut flash.acr);

        let mut serial = init_serial!(device, gpioa, c::BAUD_RATE, clocks);
        let serial_int = serial.get_interrupt();
        serial.listen(serial::Event::Rxne);
        let (mut tx, rx) = serial.split();
        // // COBS frame
        // tx.write(0x00).unwrap();

        let beeper = gpioc.pc14.output().pull_type(PullNone);
        let mut l = logging::SerialLogger::new(tx, beeper);
        info!(l, "Logger ok\r\n");
        // XXX: split delay and pass to logger as well?
        let mut delay = AsmDelay::new(freq);

        // SPI1
        let ncs = gpiob.pb9.output().push_pull();
        let scl_sck = gpiob.pb3;
        let ad0_sdo_miso = gpiob.pb4;
        let sda_sdi_mosi = gpiob.pb5;
        let spi = device.SPI1.spi((scl_sck, ad0_sdo_miso, sda_sdi_mosi),
                                  mpu9250::MODE,
                                  1.mhz(),
                                  clocks);
        info!(l, "spi ok\r\n");
        // MPU
        let mut mpu9250 =
            Mpu9250::imu_default(spi, ncs, &mut delay).expect("mpu error");
        let mut ahrs = ahrs::AHRS::create_calibrated(mpu9250,
                                                     &mut delay,
                                                     chrono::rtfm_stopwatch(freq))
            .expect("ahrs error");
        // i2c stuff, sensors
        let i2c = init_i2c!(device, gpioa, 400.khz(), clocks);
        info!(l, "i2c ok\r\n");
        // tof
        let mut tof = vl53l0x::VL53L0x::new(i2c).expect("vl");
        info!(l, "vl/tof ok\r\n");
        tof.set_measurement_timing_budget(200000).expect("timbudg");
        info!(l, "meas budget set; start cont \r\n");
        tof.start_continuous(0).expect("start cont");
        // MOTORS:
        // pa0 -- pa3
        let (ch1, ch2, ch3, ch4, mut timer2) =
            timer::tim2::Timer::new(device.TIM2,
                                    c::TIM_FREQ,
                                    clocks,
                                    &mut rcc.apb1).take_all();
        let mut m1_rear_right =
            gpioa.pa0.pull_type(PullUp).to_pwm(ch1, MediumSpeed);
        let mut m2_front_right =
            gpioa.pa1.pull_type(PullUp).to_pwm(ch2, MediumSpeed);
        let mut m3_rear_left =
            gpioa.pa2.pull_type(PullUp).to_pwm(ch3, MediumSpeed);
        let mut m4_front_left =
            gpioa.pa3.pull_type(PullUp).to_pwm(ch4, MediumSpeed);
        m1_rear_right.enable();
        m2_front_right.enable();
        m3_rear_left.enable();
        m4_front_left.enable();
        timer2.enable();

        let (ch5, ch6, _, _, mut timer3) =
            timer::tim3::Timer::new(device.TIM3,
                                    c::TIM_FREQ,
                                    clocks,
                                    &mut rcc.apb1).take_all();
        let mut m5_left = gpioa.pa6.pull_type(PullUp).to_pwm(ch5, MediumSpeed);
        let mut m6_right = gpioa.pa7.pull_type(PullUp).to_pwm(ch6, MediumSpeed);
        m5_left.enable();
        m6_right.enable();
        timer3.enable();

        info!(l, "motors ok\r\n");
        #[cfg_attr(rustfmt, rustfmt_skip)]
        let map = Map6::from_row_slice(&[
            0.567, -0.815, -1.0, 1.0, /* rear left */
            0.567, 0.815, -1.0, 1.0, /* front right */
            -0.567, -0.815, 1.0, 1.0, /* rear left */
            -0.567, 0.815, 1.0, 1.0, /* front left */
            -1.0, -0.0, -1.0, 1.0, /* left */
            1.0, -0.0, 1.0, 1.0 /* right */]);
        let mut motor_mixer = MotorMixer::new6(map,
                                               (m1_rear_right,
                                                m2_front_right,
                                                m3_rear_left,
                                                m4_front_left,
                                                m5_left,
                                                m6_right),
                                               m1_rear_right.get_max_duty()
                                               as f32);

        // let esc = ESC::new();
        // let motor = CorelessMotor::new();

        info!(l, "init done\r\n");
        l.blink();

        let initial_altitude: f32 =
            (tof.read_range_mm().expect("initial tof read error") as f32)
            / 1000.;
        // info!("inital altitude: {:?}\r\n", initial_altitude);

        init::LateResources { CMD: Cmd::new(),
                              L: l,
                              BOOTLOADER: bootloader,
                              RX: rx,
                              // ESC: esc,
                              // MOTORS: motor,
                              MOTOR_MIXER: motor_mixer,
                              AHRS: ahrs,
                              TOF: tof,
                              INITIAL_ALTITUDE: initial_altitude,
                              ALT_CONTROLLER: controller::Controller::new(),
                              ALT_CLOCK: chrono::rtfm_stopwatch(freq) }
    }

    #[idle(resources = [STATUS_REQ,
                        L,
                        AHRS,
                        ALT_CONTROLLER,
                        TOF,
                        ALT_CLOCK,
                        ELAPSED,
                        TAKEOFF,
                        TAKEOFF_DURATION_S,
                        TAKEOFF_TO,
                        ALTITUDE,
                        INITIAL_ALTITUDE,
                        VERT_VELOCITY,
                        CONT_COEFFS,
                        MOTOR_MIXER])]
    fn idle() -> ! {
        let alt_controller = resources.ALT_CONTROLLER;
        let ahrs = resources.AHRS;
        let mut l = resources.L;
        let tof = resources.TOF;
        let alt_clock = resources.ALT_CLOCK;
        let takeoff_to = *resources.TAKEOFF_TO;
        let takeoff_duration_s = *resources.TAKEOFF_DURATION_S;
        let initial_altitude = *resources.INITIAL_ALTITUDE;

        let motor_mixer = resources.MOTOR_MIXER;
        let c = resources.CONT_COEFFS;

        loop {
            let mut delta_y = 0.;
            let mut delta_x = 0.;
            let mut delta_z = 0.;
            let mut prev_err_x = 0.;
            let mut prev_err_y = 0.;
            let mut prev_err_z = 0.;
            match ahrs.estimate(&mut l) {
                Ok((dcm, biased_gyro, dt_s)) => {
                    if *resources.TAKEOFF {
                        match tof.read_range_mm() {
                            Ok(v) => {
                                // if >= 8092...
                                let time_diff_s = alt_clock.split_time_s();
                                *resources.ELAPSED += time_diff_s;
                                let (h0, h1, t0, t1) = if *resources.ELAPSED
                                                          < takeoff_duration_s
                                {
                                    (0.0, takeoff_to, 0.0, takeoff_duration_s)
                                } else {
                                    (takeoff_to, takeoff_to, 0.0, 1.0)
                                };
                                let current_altitude =
                                    (v as f32) / 1000. - initial_altitude;
                                let traversed =
                                    current_altitude - *resources.ALTITUDE;
                                *resources.ALTITUDE = current_altitude;
                                *resources.VERT_VELOCITY =
                                    traversed / time_diff_s;
                                let target_alt = h0
                                                 + (h1 - h0)
                                                   * (*resources.ELAPSED - t0)
                                                   / (t1 - t0);
                                let target_vertical_veloctity =
                                    (h1 - h0) / (t1 - t0);
                                let alt_thrust = alt_controller.altitude(
                                    target_alt,
                                    target_vertical_veloctity,
                                    *resources.ALTITUDE,
                                    *resources.VERT_VELOCITY,
                                    dcm,
                                    G);
                                let thrust = alt_thrust * START_THRUST;
                                c.total_thrust = thrust;
                            },
                            Err(_) => {},
                        };
                    }

                    // let x_err = 0. - g.x;
                    // let z_err = 0. - g.z;
                    // body rate ctrl
                    // pitch-roll ctrl
                    let pitch_err = 0. - dcm.pitch;
                    let pitch_u = pitch_err * c.pitch_p;
                    let yaw_err = 0. - dcm.yaw;
                    let yaw_u = yaw_err * c.yaw_p;
                    let roll_err = 0. - dcm.roll;
                    let roll_u = roll_err * c.roll_p;
                    let x_err = roll_u - biased_gyro.x;
                    let y_err = pitch_u - biased_gyro.y;
                    let z_err = yaw_u - biased_gyro.z;
                    let i_comp = 0.;
                    delta_x = x_err - prev_err_x;
                    delta_y = y_err - prev_err_y;
                    delta_z = z_err - prev_err_z;
                    let x_corr = x_err * c.p + i_comp * c.i + c.d * delta_x;
                    let y_corr = y_err * c.p + i_comp * c.i + c.d * delta_y;
                    let z_corr = 0.; // z_err * pk + i_comp * ik + dk * delta_z;
                    prev_err_x = x_err;
                    prev_err_y = y_err;
                    prev_err_z = z_err;

                    motor_mixer.set_duty(x_corr,
                                         y_corr,
                                         z_corr,
                                         c.total_thrust);

                    unsafe {
                        if *resources.STATUS_REQ {
                            *resources.STATUS_REQ = false;
                            info!(l, "Altitude: {:?}; vspeed: {:?}; elapsed: {:?}\r\n",
                                  resources.ALTITUDE,
                                  resources.VERT_VELOCITY,
                                  resources.ELAPSED);
                            info!(l,
                                  "dt: {:?}; dcm: {:?}; gyro: {:?};\r\n",
                                  dt_s,
                                  dcm,
                                  biased_gyro);
                            // info!(l,
                            //       "Motors: {:?}; max {}\r\n",
                            //       ctrl.get_duty(),
                            //       ctrl.max_duty);
                            info!(l,
                                  "Tthrust: {}; pk: {}; pipk: {}; ypk: {}; rpk: {}\r\n",
                                  c.total_thrust,
                                  c.p,
                                  c.pitch_p,
                                  c.yaw_p,
                                  c.roll_p);
                        }
                    }
                },
                Err(e) => {
                    info!(l, "ahrs error: {:?}\r\n", e);
                },
            }
            // interrupts are serviced here
        }
    }

    #[interrupt(resources = [CMD, L, RX, BOOTLOADER])]
    fn USART1_EXTI25() {}

    #[interrupt(resources = [CMD, L, RX, BOOTLOADER])]
    fn USART2_EXTI26() {}
};

fn process_cmd(cmd: &mut Cmd,
               l: &mut L,
               rx: &mut Rx<USART>,
               bootloader: &mut bootloader::stm32f30x::Bootloader) {
    match rx.read() {
        Ok(b) => {
            // first echo
            l.write_char(b as char);
            if let Some(word) = cmd.push(b) {
                // koeffs are parsed as i32 for simplicity
                // parse!(word:
                //        ["thrust=", thrust:i32] => {
                //            unsafe {
                //                TOTAL_THRUST = thrust as f32
                //            };
                //        },
                //        ["takeoff"] => {
                //            unsafe {
                //                TAKEOFF = true;
                //                PITCH_KOEFF = 5.;
                //                ROLL_KOEFF = 5.;
                //                P_KOEFF = 200.;
                //            };
                //        },
                //        ["dk=", dk:i32] => {
                //            unsafe {
                //                D_KOEFF = dk as f32
                //            };
                //        },
                //        ["pk=", pk:i32] => {
                //            unsafe {
                //                P_KOEFF = pk as f32
                //            };
                //        },
                //        ["pipk=", pipk:i32] => {
                //            unsafe {
                //                PITCH_KOEFF = pipk as f32
                //            };
                //        },
                //        ["ypk=", ypk:i32] => {
                //            unsafe {
                //                YAW_KOEFF = ypk as f32
                //            };
                //        },
                //        ["rpk=", rpk:i32] => {
                //            unsafe {
                //                ROLL_KOEFF = rpk as f32
                //            };
                //        },
                //        ["ik=", ik:i32] => {
                //            unsafe {
                //                I_KOEFF = ik as f32
                //            };
                //        },
                //        ["boot"] => {
                //            bootloader.to_bootloader();
                //        },
                //        ["reset"] => {
                //            bootloader.system_reset();
                //        },
                //        ["status"] => {
                //            unsafe { STATUS_REQ = true; }
                //        }
                // );
            }
        },
        Err(nb::Error::WouldBlock) => {},
        Err(nb::Error::Other(e)) => match e {
            serial::Error::Overrun => {
                info!(l, "read error: {:?}\r\n", e);
            },
            serial::Error::Framing => {
                info!(l, "read error: {:?}\r\n", e);
            },
            serial::Error::Noise => {
                info!(l, "read error: {:?}\r\n", e);
            },
            _ => {
                l.blink();
                info!(l, "read error: {:?}\r\n", e);
            },
        },
    };
}

// #[exception]
// fn HardFault(ef: &ExceptionFrame) -> ! {
//     panic!("HardFault at {:#?}", ef);
// }

// #[exception]
// fn DefaultHandler(irqn: i16) {
//     panic!("Unhandled exception (IRQn = {})", irqn);
// }

#[panic_handler]
fn panic(_panic_info: &PanicInfo) -> ! {
    // match unsafe { &mut L } {
    //     Some(ref mut l) => {
    //         let payload = panic_info.payload().downcast_ref::<&str>();
    //         match (panic_info.location(), payload) {
    //             (Some(location), Some(msg)) => {
    //                 error!(l,
    //                        "\r\npanic in file '{}' at line {}: {:?}\r\n",
    //                        location.file(),
    //                        location.line(),
    //                        msg);
    //             },
    //             (Some(location), None) => {
    //                 error!(l,
    //                        "panic in file '{}' at line {}",
    //                        location.file(),
    //                        location.line());
    //             },
    //             (None, Some(msg)) => {
    //                 error!(l, "panic: {:?}", msg);
    //             },
    //             (None, None) => {
    //                 error!(l, "panic occured, no info available");
    //             },
    //         }
    //     },
    //     None => {},
    // }
    unsafe { intrinsics::abort() }
}
