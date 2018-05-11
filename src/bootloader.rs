pub trait Bootloader: Sized + Send {
    fn check_request(&mut self);
    fn to_bootloader(&mut self);
    fn system_reset(&mut self);
}

pub mod stm32f30x {
    extern crate cortex_m;
    extern crate stm32f30x_hal as hal;

    use core;
    use cortex_m::peripheral;
    use cortex_m::register::msp;
    use cortex_m::interrupt;
    use stm32f30x::RCC;

    use super::Bootloader as BootloaderTrait;

    const BOOTLOADER_REQUEST: u32 = 1;
    const STM32_RESET_FN_ADDRESS: u32 = 0x1FFFD804u32;
    const STM32_BOOTLOADER_ADDRESS: u32 = 0x1FFFD800;
    static mut USER_RESET: Option<extern "C" fn()> = None;

    pub struct Bootloader {
        rtc: hal::stm32f30x::RTC,
        scb: peripheral::SCB,
    }

    impl Bootloader {
        pub fn new(rtc: hal::stm32f30x::RTC, scb: peripheral::SCB) -> Self {
            Bootloader { rtc, scb }
        }
    }

    impl BootloaderTrait for Bootloader {
        fn check_request(&mut self) {
            let bkp0r = &(*self.rtc).bkp0r;
            if bkp0r.read().bits() == BOOTLOADER_REQUEST {
                bkp0r.write(|w| unsafe { w.bits(0) });
                unsafe {
                    // TODO __enable_irq();
                    self.scb.vtor.write(STM32_BOOTLOADER_ADDRESS);
                    msp::write(STM32_BOOTLOADER_ADDRESS);
                    let f = STM32_RESET_FN_ADDRESS as *const fn();
                    USER_RESET = Some(core::mem::transmute(f));
                    (USER_RESET.unwrap())();
                }
                loop {}
            }
        }

        fn to_bootloader(&mut self) {
            unsafe {
                interrupt::disable();
                // Set clock configuration to the reset state
                let rcc = &*RCC::ptr();
                rcc.cr.write(|w| w.hsion().set_bit());
                rcc.cr.write(|w| w.hsitrim().bits(0x80));
                rcc.cfgr.write(|w| w.sw().bits(0x00));
                rcc.cfgr.write(|w| w.hpre().bits(0x00));
                rcc.cfgr.write(|w| w.ppre1().bits(0x00));
                rcc.cfgr.write(|w| w.ppre2().bits(0x00));
                rcc.cfgr.write(|w| w.mco().bits(0x00));
                rcc.cr.write(|w| w.pllon().clear_bit());
                rcc.cr.write(|w| w.csson().clear_bit());
                rcc.cr.write(|w| w.hseon().clear_bit());
                rcc.cr.write(|w| w.hsebyp().clear_bit());
                rcc.cfgr.write(|w| w.bits(0x0));
                rcc.cfgr2.write(|w| w.bits(0x0));
                rcc.cfgr3.write(|w| w.bits(0x0));
                rcc.cir.write(|w| w.bits(0x0));
                // Deinitialize perepherials
                rcc.apb1rstr.write(|w| w.bits(0xFFFFFFFFu32));
                rcc.apb1rstr.write(|w| w.bits(0x00000000u32));
                rcc.apb2rstr.write(|w| w.bits(0xFFFFFFFFu32));
                rcc.apb2rstr.write(|w| w.bits(0x00000000u32));
                rcc.ahbenr.write(|w| w.bits(0xFFFFFFFFu32));
                rcc.ahbenr.write(|w| w.bits(0x00000000u32));
                // Jump to bootloader
                interrupt::enable();
                msp::write(0x1FFFD800);
                let bootloader_address = 0x1FFFD804u32 as *const fn();
                (*bootloader_address)();
            }
            loop {}
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
