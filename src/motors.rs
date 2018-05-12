pub trait Motor {
    fn write(&mut Self);
}

pub mod stm32f30x {
    pub struct MotorPWM {

    }

    impl MotorPWM {
        fn new() -> Self {
            MotorPWM { }
        }
    }

    impl MotorTrait for MotorPWM {
        fn write(&mut Self) {

        }
    }
}

