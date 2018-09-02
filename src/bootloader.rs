pub trait Bootloader: Sized {
    fn check_request(&mut self);
    fn to_bootloader(&mut self);
    fn system_reset(&mut self);
}

pub mod stm32f30x {
    use core;
    use cortex_m::{self, interrupt, register::msp};
    use hal::stm32f30x::{RTC, PWR, RCC};
    use cortex_m::peripheral::SCB;

    use super::Bootloader as BootloaderTrait;

    const BOOTLOADER_REQUEST:u32 = 93;
    const STM32_RESET_FN_ADDRESS:u32 = 0x1FFFD804u32;
    const STM32_BOOTLOADER_ADDRESS:u32 = 0x1FFFD800;

    pub struct Bootloader;

    impl Bootloader {
        pub fn new() -> Self {
            Bootloader { }
        }

        fn enable_bkp(&mut self) {
            let rcc = unsafe {&*RCC::ptr()};
            let pwr = unsafe {&*PWR::ptr()};
            let rtc = unsafe {&*RTC::ptr()};
            // enable bkp registers
            &(*rcc).apb1enr.modify(|r, w| {
                w.pwren().bit(true)
            });
            // clear data protection
            &(*pwr).cr.modify(|r, w| w.dbp().bit(true));
        }
    }

    impl BootloaderTrait for Bootloader {
        fn check_request(&mut self) {
            self.enable_bkp();
            let rtc = unsafe {&*RTC::ptr()};
            let bkp0r = &(*rtc).bkp0r;
            if bkp0r.read().bits() == BOOTLOADER_REQUEST {
                cortex_m::asm::dsb();
                unsafe {
                    asm!("
        cpsie i\n
        movw r0, 0xd800\n
        movt r0, 0x1fff\n
        ldr r0, [r0]\n
        msr MSP, r0\n" ::: "r0" : "volatile");
                    let f = 0x1FFFD804u32 as *const fn();
                    (*f)();
                }
                cortex_m::asm::dsb();
                loop {
                    cortex_m::asm::nop(); // avoid rust-lang/rust#28728
                }
            }
        }

        fn to_bootloader(&mut self) {
            self.enable_bkp();
            let rtc = unsafe {&*RTC::ptr()};
            // write cookie to backup register and reset
            &(*rtc).bkp0r.write(|w| unsafe { w.bits(BOOTLOADER_REQUEST) });
            cortex_m::asm::dsb();
            self.system_reset();
        }

        fn system_reset(&mut self) {
            let scb = cortex_m::peripheral::SCB::ptr();
            cortex_m::asm::dsb();
            unsafe {
                (*scb).aircr.write(0x05FA0000 | 0x04u32);
            }
            cortex_m::asm::dsb();
            loop { // wait for the reset
                cortex_m::asm::nop(); // avoid rust-lang/rust#28728
            }
        }
    }
}
