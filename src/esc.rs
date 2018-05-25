pub trait ESC {
    fn get_max_throttle() -> u32; // or u16?
    fn set_throttle(throttle: u32);
}

pub mod PWM {
    use super::ESC as ESCTrait;

    pub struct Controller {

    }

    impl Controller {
        pub fn new() -> Self {
            Controller {}
        }
    }

    impl ESCTrait for Controller {
        fn get_max_throttle() -> u32 {
            return 0;
        }

        fn set_throttle(throttle: u32) {

        }
    }
}