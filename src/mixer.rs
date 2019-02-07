use nalgebra::clamp;

pub struct MotorCtrl<M, P> {
    pub map: M,
    pub pin: P,
    pub max_duty: f32,
}

pub type Ctrl = nalgebra::Vector4<f32>;
pub type Map4 = nalgebra::Matrix4<f32>;
pub type Map6 = nalgebra::Matrix6x4<f32>;

macro_rules! impl_motor_ctrl {
    ($map:ident, $num:expr, $($pin:ident $nr:tt)+) => (
        impl<$($pin),+> MotorCtrl<$map, ($($pin),+)>
        where $($pin: ehal::PwmPin<Duty = u32>),+
        {
            pub fn set_duty(&mut self, x: f32, y: f32, z: f32, thrust: f32) {
                let duty = self.map * Ctrl::new(x, y, z, thrust);
                let max_duty = self.max_duty;
                $( self.pin.$nr.set_duty(clamp(duty[$nr], 0.0, max_duty) as u32); )+
            }

            pub fn get_duty(&mut self) -> [u32; $num] {
                [ $( self.pin.$nr.get_duty() ),+ ]
            }
        }
    )
}

impl_motor_ctrl!(Map4, 4, A 0 B 1 C 2 D 3);
impl_motor_ctrl!(Map6, 6, A 0 B 1 C 2 D 3 E 4 F 5);
