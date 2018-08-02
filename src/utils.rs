use super::ehal;
use super::nb;
use cortex_m::asm;
use hal::time::{Hertz, U32Ext};

pub fn tick_delay(ticks: usize) {
    (0..ticks).for_each(|_| asm::nop());
}

pub fn time_delay<C>(c: &mut C, sec: u8)
    where C: ehal::timer::CountDown<Time = Hertz>
{
    for _i in 0..sec {
        c.start(1.hz());
        while let Err(nb::Error::WouldBlock) = c.wait() {}
    }
}
