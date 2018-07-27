//! On-board pad for beeper

use hal::prelude::*;

use hal::gpio::gpioc::PC14;
use hal::gpio::{LowSpeed, Output, PinMode, PullNone, PullType, PushPull};

pub struct Beeper {
    pin: PC14<PullNone, Output<PushPull, LowSpeed>>,
}

impl Beeper {
    pub fn new<PT, PM>(pin: PC14<PT, PM>) -> Self
    where
        PT: PullType,
        PM: PinMode,
    {
        let pin = pin.output().pull_type(PullNone);
        Beeper { pin }
    }

    pub fn off(&mut self) {
        self.pin.set_low()
    }

    pub fn on(&mut self) {
        self.pin.set_high()
    }
}
