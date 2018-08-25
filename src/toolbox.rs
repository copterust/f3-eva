use hal::gpio::{AltFn, Gpioa, HighSpeed, PullNone, PushPull, AF7, PA10, PA9};
use hal::prelude::*;
use hal::serial::Serial;
use hal::stm32f30x;

// Initialization support

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

#[cfg_attr(rustfmt, rustfmt_skip)]
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
