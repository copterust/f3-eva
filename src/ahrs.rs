use crate::utils::vec_to_tuple;

use dcmimu::DCMIMU;
use ehal::blocking::delay::DelayMs;
use ehal::blocking::spi;
use ehal::digital::OutputPin;
use libm::{asinf, atan2f, fabsf};
use mpu9250::Mpu9250;
use nalgebra::geometry::Quaternion;
use nalgebra::Vector3;

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
pub struct AHRS<SPI, NCS> {
    mpu: Mpu9250<SPI, NCS, mpu9250::Imu>,
    dcmimu: DCMIMU,
    accel_biases: Vector3<f32>,
}

impl<SPI, NCS, E> AHRS<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    pub fn create_calibrated<D>(mut mpu: Mpu9250<SPI, NCS, mpu9250::Imu>,
                                delay: &mut D)
                                -> Result<Self, mpu9250::Error<E>>
        where D: DelayMs<u8>
    {
        let mut accel_biases = mpu.calibrate_at_rest(delay)?;
        // Accel biases contain compensation for Earth gravity,
        // so when we will adjust measurements with those biases, gravity will
        // be cancelled. This is helpful for some algos, but not the
        // others. For DCMIMU we need gravity, so we will add it back
        // to measurements, by adjusting biases once.
        // TODO: find real Z axis.
        accel_biases.z -= mpu9250::G;
        let dcmimu = DCMIMU::new();
        Ok(AHRS { mpu, dcmimu, accel_biases })
    }

    pub fn estimate(&mut self,
                    dt_s: f32)
                    -> Result<dcmimu::TaitBryanAngles, E> {
        let meas = self.mpu.all()?;
        let accel = meas.accel - self.accel_biases;
        let gyro = meas.gyro;

        let res =
            self.dcmimu.update(vec_to_tuple(&gyro), vec_to_tuple(&accel), dt_s);
        Ok(res)
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
