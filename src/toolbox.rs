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
    (USART1, $p: path, $st: ty, $is: expr) => {
        interrupt!(USART1_EXTI25, usart_int, state: $st = $is);
    };
    (USART2, $p: path, $s: expr) => {
        interrupt!(USART2_EXTI26, usart_int, state: $st = $is);
    };
}

#[cfg_attr(rustfmt, rustfmt_skip)]
macro_rules! use_serial {
    ($u: ident, $int: path, state: $State:ty = $initial_state:expr
    ) => {
        type USART = stm32f30x::$u;
        _inter!($u, $int, $State, $initial_state);
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

macro_rules! when {
    (
        $input: ident.startswith():
        $($cmdvar:expr, $name:pat => $code:expr),+
    ) => {
        $(
            if $input.starts_with($cmdvar.as_bytes()) {
                let $name = &$input[$cmdvar.len()..];
                $code
            } else
        )+
            { }
    }
}
