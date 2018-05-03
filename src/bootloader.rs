
pub trait Bootloader : Sized + Send {
    fn check_request(&mut self);
    fn to_bootloader(&mut self);
    fn system_reset(&mut self);
}

pub mod stm32f30x {
    extern crate stm32f30x_hal as hal;
    extern crate cortex_m;
    use cortex_m::register::msp;

    use super::Bootloader as BootloaderTrait;

    const BOOTLOADER_REQUEST:u32 = 1;
    const STM32_RESET_FN_ADDRESS:u32 = 0x1FFFD804u32;
    const STM32_BOOTLOADER_ADDRESS:u32 = 0x1FFFD800;

    pub struct Bootloader {
        rtc: hal::stm32f30x::RTC
    }

    impl Bootloader {
        pub fn new(rtc: hal::stm32f30x::RTC) -> Self {
            Bootloader { rtc }
        }
    }

    impl BootloaderTrait for Bootloader {
        fn check_request(&mut self) {
            let bkp0r = &(*self.rtc).bkp0r;
            if bkp0r.read().bits() == BOOTLOADER_REQUEST {
                bkp0r.write(|w| unsafe { w.bits(0) });
                unsafe {
                    // TODO __enable_irq();
                    msp::write(STM32_BOOTLOADER_ADDRESS);
                    let f = STM32_RESET_FN_ADDRESS as *const fn();
                    (*f)();
                }
                loop {}
            }
        }

        fn to_bootloader(&mut self) {
            {
                let bkp0r = &(*self.rtc).bkp0r;
                // write cookie to backup register and reset
                bkp0r.write(|w| unsafe { w.bits(BOOTLOADER_REQUEST) });
            }
            self.system_reset();
        }

        fn system_reset(&mut self) {
            // TODO: pass to constsructor?
            let scb = cortex_m::peripheral::SCB::ptr();
            unsafe {
                (*scb).aircr.write(0x05FA0000 | 0x04u32);
            }
        }

    }
}
