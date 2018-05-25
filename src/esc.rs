pub trait ESC {
    pub fn get_max_throttle() -> u32; // or u16?
    pub fn set_throttle(throttle: u32);
}

mod PWM {
    use super::ESC as ESCTrait;

    struct ESC {

    }

    impl ESCTrait for ESC {
        fn get_max_throttle() -> u32 {
            return 0;
        }

        fn set_throttle(throttle: u32) {
            
        }
    }
}