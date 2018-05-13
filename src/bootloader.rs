pub trait Bootloader: Sized + Send {
    fn to_bootloader(&mut self);
    fn system_reset(&mut self);
}

pub mod stm32f30x {
    extern crate cortex_m;
    extern crate stm32f30x_hal as hal;

    use core;
    use cortex_m::interrupt;
    use cortex_m::peripheral;
    use cortex_m::register::msp;

    use super::Bootloader as BootloaderTrait;

    pub struct Bootloader {
        scb: peripheral::SCB,
    }

    impl Bootloader {
        pub fn new(scb: peripheral::SCB) -> Self {
            Bootloader { scb }
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
            unsafe {
                self.scb.aircr.write(0x05FA0000 | 0x04u32);
            }
        }
    }
}
