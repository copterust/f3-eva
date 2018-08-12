pub trait Bootloader: Sized {
    fn to_bootloader(&mut self);
    fn system_reset(&mut self);
}

pub mod stm32f30x {
    use core;
    use cortex_m::{self, interrupt, register::msp};

    use super::Bootloader as BootloaderTrait;

    pub struct Bootloader {
        scb: cortex_m::peripheral::SCB,
    }

    impl Bootloader {
        pub fn new(scb: cortex_m::peripheral::SCB) -> Self {
            Bootloader { scb, }
        }
    }

    impl BootloaderTrait for Bootloader {
        fn to_bootloader(&mut self) {
            unsafe {
                interrupt::enable();
                let n = core::ptr::read(0x1FFFD800u32 as *const u32);
                msp::write(n);
                let bootloader_address = 0x1FFFD804u32 as *const fn();
                (*bootloader_address)();
            }
            loop {}
        }

        fn system_reset(&mut self) {
            self.scb.system_reset();
        }
    }
}
