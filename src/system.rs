//! A trait for system-level functions

trait System {
    fn new() -> Self;
    fn reset(&self);
    fn invoke_bootloader(&self);
}

