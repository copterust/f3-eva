pub trait Motor: Sized {
    fn set_rpm(&mut self, rpm: f32);
}

pub mod brushed {
    use super::Motor as MotorTrait;

    pub struct Coreless {}

    impl Coreless {
        pub fn new() -> Self {
            Coreless {}
        }
    }

    impl MotorTrait for Coreless {
        fn set_rpm(&mut self, _rpm: f32) {}
    }
}

//pub mod Brushless {
//
//}
