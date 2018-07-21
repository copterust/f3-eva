//! On-board pad for beeper

use hal::prelude::*;

use hal::gpio::gpioc::{PC14, MODER, OTYPER};
use hal::gpio::{Floating, Input, Output, PushPull};

pub struct Beeper {
    pin: PC14<Output<PushPull>>,
}

impl Beeper {
    pub fn new(pin: PC14<Input<Floating>>, moder: &mut MODER, otyper: &mut OTYPER) -> Self {
        let p = pin.into_push_pull_output(moder, otyper);
        Beeper { pin: p }
    }

    pub fn off(&mut self) {
        self.pin.set_low()
    }

    pub fn on(&mut self) {
        self.pin.set_high()
    }
}
