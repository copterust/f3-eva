pub trait ESC {
    fn get_max_throttle(&self) -> u32; // or u16?
    fn set_throttle(&mut self, throttle: u32);
}

pub mod pwm {
    use super::ESC as ESCTrait;

    pub struct Controller {}

    impl Controller {
        pub fn new() -> Self {
            Controller {}
        }
    }

    impl ESCTrait for Controller {
        fn get_max_throttle(&self) -> u32 {
            return 0;
        }

        fn set_throttle(&mut self, _throttle: u32) {}
    }
}
