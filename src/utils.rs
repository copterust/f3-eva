use cortex_m::asm;

pub fn tick_delay(ticks: usize) {
    (0..ticks).for_each(|_| asm::nop());
}
