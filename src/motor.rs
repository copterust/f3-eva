pub trait Motor {
    fn set_rpm(rpm: f32);
}

pub mod Brushed {
    use super::Motor as MotorTrait;

    pub struct Coreless {
    }

    impl Coreless {
        pub fn new() -> Self {
            Coreless {}
        }
    }

    impl MotorTrait for Coreless {
        fn set_rpm(rpm: f32) {
            
        }
    }
}

//pub mod Brushless {
//
//}
