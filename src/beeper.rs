//! On-board pad for beeper

use hal::prelude::*;

use hal::gpio::gpioc::{PC15};
use hal::gpio::{Output, PushPull};

pub type BEEPER = PC15<Output<PushPull>>;

pub struct Beeper {
    pin: PC15<Output<PushPull>>,
}

impl Beeper {
    pub fn new(mut gpioc: gpioc::Parts) -> Self {
        let p = gpioc
            .pc15
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
        Beeper { pin: p }
    }

    pub fn off(&mut self) {
        self.pin.set_low()
    }

    pub fn on(&mut self) {
        self.pin.set_high()
    }
}
