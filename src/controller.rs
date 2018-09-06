use core::f32::consts::PI;

struct Controller {
    KpYaw: f32
}

impl Controller {
    /// Yaw controller.
    /// Returns: target yaw rate (in rads/s) based on current and desired yaw
    pub fn yaw_control(self, target, current) -> f32 {
        let cmd = target % (2.0 * PI);
        let mut err = cmd - current;
        if err > PI {
            err = err - 2.0 * PI;
        } else if err < -PI {
            err = err + 2.0 * PI;
        }
        err * self.KpYaw
    }
}