pub trait Motor: Sized + Send {
    fn write(&mut self);
}

pub mod f3evo {
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
/*
    pub struct MotorDSHOT {

    }

    impl MotorDSHOT {
        pub fn new() -> Self {
            MotorDSHOT { }
        }
    }

    impl MotorTrait for MotorDSHOT {
        fn write(&mut self) {

        }
    }
*/
}

