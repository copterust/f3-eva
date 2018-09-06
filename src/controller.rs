use core::f32::consts::PI;
use nalgebra::{Vector3, norm};

struct Controller {
    KpYaw: f32,
    KpPQR: Vector3,
    max_torque: f32,
    MOI: Vector3,
}

impl Controller {
    /// Body-rate controller.
    /// Gets: target body rates and actual ones
    /// Returns: roll, pitch and yaw moments in N*m
    pub fn body_rate(self, target: Vector3, current: Vector3) -> Vector3 {
        let err = target - current;
        let mut cmd = self.MOI * (self.KpPQR * err);
        let norm = norm(cmd);
        if norm > self.max_torque {
            cmd = cmd * self.max_torque / norm;
        }
        cmd
    }

    /// Yaw controller.
    /// Returns: target yaw rate (in rads/s) based on current and desired yaw
    pub fn yaw(self, target, current) -> f32 {
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