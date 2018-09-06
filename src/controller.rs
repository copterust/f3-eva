use core::f32::consts::PI;
use nalgebra::{Vector2, Vector3, norm};

/// Nonlinear drone controller
struct Controller {
    /// P-coefficient for yaw controller
    KpYaw: f32,
    /// P-coefficients for body-rate controller
    KpPQR: Vector3,
    /// Maximal torque
    max_torque: f32,
    /// Vehicle's moment of inertia about three axes
    MOI: Vector3,
}

impl Controller {
    /// Roll-pitch controller.
    /// cmd: target acceleration (north, east)
    /// attitude: roll, pitch, yaw
    /// Returns pitch and roll rates for desired north/east acceleration
    pub fn roll_pitch(self, cmd: Vector2, attitude: Vector3, thrust: f32) -> Vector2 {

    }

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