pub trait Motor: Sized + Send {
    fn write(&mut self);
}

pub mod stm32f30x {
    use super::Motor as MotorTrait;

    pub struct MotorPWM {

    }

    impl MotorPWM {
        pub fn new() -> Self {
            MotorPWM { }
        }
    }

    impl MotorTrait for MotorPWM {
        fn write(&mut self) {

        }
    }
}

