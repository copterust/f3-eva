use crate::chrono::Chrono;

use crate::utils::vec_to_tuple;

use dcmimu::DCMIMU;
use ehal::blocking::delay::DelayMs;
use ehal::blocking::spi;
use ehal::digital::OutputPin;
use libm::{asinf, atan2f, atanf, fabsf, sqrtf};
use mpu9250::Mpu9250;
use nalgebra::geometry::Quaternion;
use nalgebra::Vector3;

mod kalman;

// Magnetometer calibration parameters
// NOTE you need to use the right parameters for *your* magnetometer
// You can use the `log-sensors` example to calibrate your magnetometer. The
// producer is explained in https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration

// const M_BIAS_X: f32 = 150.;
// const M_SCALE_X: f32 = 0.9;
// const M_BIAS_Y: f32 = -60.;
// const M_SCALE_Y: f32 = 0.9;
// const M_BIAS_Z: f32 = -220.;
// const M_SCALE_Z: f32 = 1.2;

// TODO: make generic over Imu/Marg
pub struct AHRS<DEV, T> {
    mpu: Mpu9250<DEV, mpu9250::Imu>,
    dcmimu: DCMIMU,
    // accel_biases: Vector3<f32>,
    timer_ms: T,
    angle_x: f32,
    angle_y: f32,
    kalman_x: kalman::AngularKalman,
    kalman_y: kalman::AngularKalman,
}

impl<DEV, E, T> AHRS<DEV, T>
    where DEV: mpu9250::Device<Error = E>,
          T: Chrono
{
    pub fn create_calibrated<D>(mut mpu: Mpu9250<DEV, mpu9250::Imu>,
                                delay: &mut D,
                                timer_ms: T)
                                -> Result<Self, mpu9250::Error<E>>
        where D: DelayMs<u8>
    {
        // let mut accel_biases = mpu.calibrate_at_rest(delay)?;
        // Accel biases contain compensation for Earth gravity,
        // so when we will adjust measurements with those biases, gravity will
        // be cancelled. This is helpful for some algos, but not the
        // others. For DCMIMU we need gravity, so we will add it back
        // to measurements, by adjusting biases once.
        // TODO: find real Z axis.
        // accel_biases.z -= mpu9250::G;
        let dcmimu = DCMIMU::new();
        let ky = kalman::AngularKalman{
            q_a: 0.001,
            q_b: 0.003,
            r: 0.03,
            angle: 0.0,
            bias: 0.0,
            rate: 0.0,
            p: [[0.0, 0.0], [0.0, 0.0]],
            k: [0.0, 0.0],
            y: 0.0,
            s: 0.0
        };

        let kx = kalman::AngularKalman{
            q_a: 0.001,
            q_b: 0.003,
            r: 0.03,
            angle: 0.0,
            bias: 0.0,
            rate: 0.0,
            p: [[0.0, 0.0], [0.0, 0.0]],
            k: [0.0, 0.0],
            y: 0.0,
            s: 0.0
        };
        Ok(AHRS { mpu,
                  dcmimu,
                  // accel_biases,
                  timer_ms,
                  angle_x: 0.0,
                  angle_y: 0.0,
                  kalman_x: kx,
                  kalman_y: ky,
                  })
    }

    pub fn setup_time(&mut self) {
        self.timer_ms.reset();
    }

    pub fn estimate<W>(&mut self,
                       l: &mut W)
                       -> Result<(dcmimu::EulerAngles, Vector3<f32>, f32), E>
        where W: core::fmt::Write
    {
        let meas = self.mpu.all()?;
        let dt_s = self.timer_ms.split_time_s();
        let accel = meas.accel; //
                                // - self.accel_biases;
        let mut gyro = meas.gyro;


        // New filter
        let roll  = atan2f(accel[1], sqrtf(accel[0] * accel[0] + accel[2] * accel[2]));
        let pitch = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));

        let gyro_x = gyro[0];
        let mut gyro_y = gyro[1];

        if ((roll < -1.5707963267948966 && self.angle_x > 1.5707963267948966) || (roll > 1.5707963267948966 && self.angle_x < -1.5707963267948966)) {
            self.kalman_x.set_angle(roll);
            self.angle_x = roll;
        } else {
            self.angle_x = self.kalman_x.step(roll, gyro_x, dt_s);
        }
        if (fabsf(self.angle_x) > 1.5707963267948966) {
            gyro_y = -gyro_y;
        }
        self.angle_y = self.kalman_y.step(pitch, gyro_y, dt_s);
        // retlif weN
        //
        let (dcm, gyro_biases) =
            self.dcmimu.update(vec_to_tuple(&gyro), vec_to_tuple(&accel), dt_s);
        let gyro_biases =
            Vector3::new(self.kalman_x.bias, self.kalman_y.bias, gyro_biases.z);
        gyro = gyro - gyro_biases;
        debug!(l,
               "typrxyz,{},{},{},{},{},{},{}\r\n",
               dt_s,
               dcm.yaw,
               self.angle_y,
               self.angle_x,
               gyro.x,
               gyro.y,
               gyro.z);
        Ok((dcm, gyro, dt_s))
    }
}

pub fn to_euler(q: &Quaternion<f32>) -> (f32, f32, f32) {
    let sqw = q.w * q.w;
    let sqx = q.i * q.i;
    let sqy = q.j * q.j;
    let sqz = q.k * q.k;
    let pitch = asinf(-2. * (q.i * q.k - q.j * q.w));
    let m = q.i * q.j + q.k * q.w;
    let mut roll;
    let mut yaw;
    if fabsf((m - 0.5)) < 1e-8 {
        roll = 0.;
        yaw = 2. * atan2f(q.i, q.w);
    } else if fabsf(m + 0.5) < 1e-8 {
        roll = -2. * atan2f(q.i, q.w);
        yaw = 0.;
    } else {
        roll = atan2f(2. * (q.i * q.j + q.k * q.w), sqx - sqy - sqz + sqw);
        yaw = atan2f(2. * (q.j * q.k + q.i * q.w), -sqx - sqy + sqz + sqw);
    }
    (roll, pitch, yaw)
}
