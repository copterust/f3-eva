use hal::gpio::{AltFn, Gpioa, HighSpeed, PullNone, PushPull, AF7, PA10, PA9};
use hal::prelude::*;
use hal::serial::Serial;
use hal::stm32f30x;

// pub trait UsartInit<GPIO> {
//     type Output;
//     fn toto(device: stm32f30x::Peripherals,
//             gpio: GPIO,
//             baud_rate: hal::time::Bps,
//             clocks: hal::rcc::Clocks)
//             -> (stm32f30x::Peripherals, GPIO, Self::Output);
// }

// impl UsartInit for (stm32f30x::USART1,
//                     stm32f30x::Peripherals,
//                     Gpioa) {
//     type Output = Serial<stm32f30x::USART1,
//                          (PA9<PullNone, AltFn<AF7, PushPull, HighSpeed>>,
//                          PA10<PullNone, AltFn<AF7, PushPull, HighSpeed>>)>;

//     fn toto(baud_rate: hal::time::Bps,
//             clocks: hal::rcc::Clocks)
//             -> (stm32f30x::Peripherals, Gpioa, Self::Output) {
//         let us = device.USART1;
//         let tx = gpio.pa9;
//         let rx = gpio.pa10;
//         let ser = us.serial((tx, rx), baud_rate, clocks);
//         (device, gpio, ser)
//     }
// }
macro_rules! _init_serial {
    (USART1,
     $device:ident,
     $gp: ident,
     $br: expr,
     $clocks: expr) => {
        $device.USART1.serial(($gp.pa9, $gp.pa10), $br, $clocks)
    };
    (USART2,
     $device:ident,
     $gp: ident,
     $br: expr,
     $clocks: expr) => {
        $device.USART2.serial(($gp.pa14, $gp.pa15), $br, $clocks)
    };
}

macro_rules! _inter {
    (USART1, $p: path) => {
        interrupt!(USART1_EXTI25, usart_int);
    };
    (USART2, $p: path) => {
        interrupt!(USART2_EXTI26, usart_int);
    };
}

macro_rules! use_serial {
    ($u: ident, $int: path
    ) => {
        type USART = stm32f30x::$u;
        _inter!($u, $int);
        macro_rules! init_serial {
            ($device:ident,
                                     $gp:ident,
                                     $br: expr,
                                     $clocks: expr
                                    ) => {
                _init_serial!($u, $device, $gp, $br, $clocks)
            };
        }
    };
}

// macro_rules! init_serial {
//     ($usart:ty,
//      $device:expr,
//      $gpio:expr,
//      $br: expr,
//      $clocks: expr) => {
//         <$usart as UsartInit>::toto($device, $gpio, $br, $clocks)
//     };
// }
