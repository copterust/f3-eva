//#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_std]

extern crate cortex_m;

extern crate embedded_hal as ehal;
extern crate stm32f30x_hal as hal;
extern crate mpu9250;

mod beeper;

use hal::prelude::*;
use hal::stm32f30x;
use cortex_m::peripheral::syst::SystClkSource;
use stm32f30x::*;
//use hal::delay::Delay;
//use mpu9250::Mpu9250;
//use hal::spi::Spi;
//use hal::serial::*;

fn init_system() {
    let dp = stm32f30x::Peripherals::take().unwrap();
    // enable the PWR clock
    dp.RCC.apb1enr.modify(|_, w| w.pwren().enabled());
    // enable the SYSCFG clock (used for USB disconnect)
    dp.RCC.apb2enr.modify(|_, w| w.syscfgen().enabled());
    // enable the usb disconnect GPIO clock
    dp.RCC.ahbenr.modify(|_, w| w.iopeen().enabled());
    // enable GIOA clock
    dp.RCC.ahbenr.modify(|_, w| w.iopaen().enabled());
    // USB is on PA 11/12 USB_DM/DP
    // in push-pull mode
    // in AF
    // without pull-up resistor
    // running at 50 MHz
    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    // XXX: in C we can do smth like: "GIO_Pin = Pin_11 | Pin_12a" and then
    //      roll with it.
    let _dm = gpioa.pa11
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
        .into_af14(&mut gpioa.moder, &mut gpioa.afrh)
        .into_highspeed(&mut gpioa.ospeedr);
    let _dp = gpioa.pa12
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
        .into_af14(&mut gpioa.moder, &mut gpioa.afrh)
        .into_highspeed(&mut gpioa.ospeedr);
    // XXX: check for PullUp/PullDown is not set?
    // XXX: button?

    let mut cp = cortex_m::Peripherals::take().unwrap();
    cp.NVIC.clear_pending(Interrupt::USB_WKUP_EXTI);
    // XXX: get rid of `unsafe` by setting individual bits?
    dp.SYSCFG.exticr1.write(|w| unsafe {
        w.exti1().bits(0x04)
    });
    // clear exti line
    dp.EXTI.imr1.write(|w| { w.mr18().set_bit()});
    dp.EXTI.emr1.write(|w| { w.mr18().set_bit()});
    // select the trigger for the selected interrupts
    dp.EXTI.rtsr1.write(|w| { w.tr18().set_bit() });
    dp.EXTI.ftsr1.write(|w| { w.tr18().clear_bit() });
}

fn init_usb_clock() {
    let dp = stm32f30x::Peripherals::take().unwrap();
    // USB prescaler 1:5
    dp.RCC.cfgr.modify(|_, w| w.usbpres().clear_bit());
    dp.RCC.apb1enr.modify(|_, w| w.usben().enabled());
}

fn init_systick() {
    let dp = stm32f30x::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let freq = clocks.hclk();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut sys_tick = cp.SYST;
    // configure systick
    // TODO if tick > mask...
    match freq {
        hal::time::Hertz(i) => sys_tick.set_reload((i / 100) - 1)
    }

    let scb = cp.SCB;
    let priority = (1 << NVIC_PRIO_BITS) - 1;
    unsafe {
        // SysTick IRQ
        scb.shpr[11].write((priority << (8 - NVIC_PRIO_BITS)) & 0xff);
    };

    sys_tick.clear_current();
    sys_tick.set_clock_source(SystClkSource::Core);
    sys_tick.enable_interrupt();
    sys_tick.enable_counter();
}

fn enable_nvic(irq_channel: usize, preemption_priority: u8, sub_priority: u8) {
    let cp = cortex_m::Peripherals::take().unwrap();
    let ar_value = cp.SCB.aircr.read();
    let mut tmp_priority = (0x700 - (ar_value & 0x700)) >> 0x08;
    let tmp_pre = 0x4 - tmp_priority;
    let tmpsub = 0x0f >> tmp_priority;
    tmp_priority = (preemption_priority as u32) << tmp_pre;
    tmp_priority = tmp_priority | ((sub_priority as u32) & tmpsub);
    tmp_priority = tmp_priority << 0x04;
    unsafe { cp.NVIC.ipr[irq_channel].write(tmp_priority as u8) };
    unsafe { cp.NVIC.iser[irq_channel >> 0x05]
             .write(0x01 << (irq_channel & 0x1f)) };
}

fn setup_usb_interrupts() {
    let cp = cortex_m::Peripherals::take().unwrap();
    // XXX: why this is unsafe? volatile-register shouldn't be
    unsafe { cp.SCB.aircr.write(0x05FA0000 | 0x500) };
    enable_nvic(20, 2, 0);
    enable_nvic(42, 1, 0);
    enable_nvic(6, 0, 0);
}

fn init_usb() {

}

fn main() {
    init_systick();
    init_system();
    init_usb_clock();
    setup_usb_interrupts();
    init_usb();
    /*
    let dp = stm32f30x::Peripherals::take().unwrap();
    // USB stuff
    let rss = dp.RSS.constrain();

    let mut rcc = dp.RCC.constrain();
    // GPs
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let gpioc = dp.GPIOC.split(&mut rcc.ahb);

    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(cp.SYST, clocks);

    let txpin = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rxpin = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);

    let uart = hal::serial::Serial::usart1(
        dp.USART1,
        (txpin, rxpin),
        hal::time::Bps(9600), clocks, &mut rcc.apb2);
    let (mut tx, _rx) = uart.split();

    // SPI1
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        mpu9250::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut mpu9250 = Mpu9250::marg(spi, nss, &mut delay).unwrap();

    // assert_eq!(mpu9250.who_am_i().unwrap(), 0x71);
    // assert_eq!(mpu9250.ak8963_who_am_i().unwrap(), 0x48);

    let mut _beep = beeper::Beeper::new(gpioc);
    loop {
        let _res1  = tx.write(55);
        delay.delay_ms(1_000_u16);
        let _res2 = tx.flush();
    }
    */
}
