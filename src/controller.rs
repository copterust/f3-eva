#![allow(non_snake_case)]

use core::f32::consts::PI;
use nalgebra::{norm, clamp};
use libm::F32Ext;

use crate::{Vector2, Vector3};

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
    /// P-coefficient for altitude controller
    KpAlt: f32,
    /// Maximal descent rate
    MaxDescentRate: f32,
    /// Maximal ascent rate
    MaxAscentRate: f32,
    /// P-coefficient for vertical acceleration controller
    KpAltAcc: f32,
    /// Drone mass
    Mass: f32,
    /// Maximal thrust
    MaxThrust: f32,
}

impl Controller {
    /// Altitude controller
    /// Return thrust to set given current state
    /// attitude in form roll/pitch/yaw
    pub fn altitude(
        self,
        target_alt: f32,
        target_vertical_velocity: f32,
        altitude: f32,
        vertical_velocity: f32,
        attitude: Vector3,
        feed_forward_acceleration: f32) -> f32 {

        let mut v_cmd = self.KpAlt * (target_alt - altitude);
        v_cmd += target_vertical_velocity;
        v_cmd = clamp(v_cmd, -self.MaxDescentRate, self.MaxAscentRate);
        let acc_cmd = feed_forward_acceleration + self.KpAltAcc * (v_cmd - vertical_velocity);
        let thrust = self.Mass * acc_cmd / (attitude[0].cos() * attitude[1].cos());
        clamp(thrust, 0.0, self.MaxThrust)
    }

    /// Roll-pitch controller.
    /// cmd: target acceleration (north, east)
    /// attitude: roll, pitch, yaw
    /// Returns pitch and roll rates for desired north/east acceleration
    pub fn roll_pitch(self, cmd: Vector2, attitude: Vector3, thrust: f32) -> Vector2 {
        unimplemented!();
    }

    /// Body-rate controller.
    /// Gets: target body rates and actual ones
    /// Returns: roll, pitch and yaw moments in N * m
    pub fn body_rate(self, target: Vector3, current: Vector3) -> Vector3 {
        let err = target - current;
        let mut cmd = self.MOI.clone();
        cmd.component_mul_assign(&self.KpPQR);
        cmd.component_mul_assign(&err);
        let norm = norm(&cmd);
        if norm > self.max_torque {
            cmd = cmd * self.max_torque / norm;
        }
        cmd
    }

    /// Yaw controller.
    /// Returns: target yaw rate (in rads/s) based on current and desired yaw
    pub fn yaw(self, target: f32 , current: f32) -> f32 {
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
