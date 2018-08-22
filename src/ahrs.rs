use ehal::blocking::delay::DelayMs;
use ehal::blocking::spi;
use ehal::digital::OutputPin;
use madgwick::{Madgwick, Quaternion, Vector3};
use mpu9250::Mpu9250;

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

// Madgwick filter parameters
// In the original Madgwick study, beta of 0.041 (corresponding to
// GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
const BETA: f32 = 1e-3;

// TODO: make generic over Imu/Marg
pub struct AHRS<SPI, NCS> {
    mpu: Mpu9250<SPI, NCS, mpu9250::Imu>,
    madgwick: Madgwick<madgwick::Imu>,
}

impl<SPI, NCS, E> AHRS<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    pub fn new(mut mpu: Mpu9250<SPI, NCS, mpu9250::Imu>,
               freq_sec: f32)
               -> Self {
        let madgwick = Madgwick::imu(BETA, freq_sec);
        AHRS { mpu,
               madgwick, }
    }

    pub fn read(&mut self) -> Result<Quaternion<f32>, E> {
        let meas = self.mpu.all()?;
        // let mag = self.mpu.mag()?;
        let accel = meas.accel;
        let gyro = meas.gyro;

        // Fix the X Y Z components of the magnetometer so they match the gyro
        // axes
        // let mag = Vector3::new(mag.y, -mag.x, mag.z);
        // Fix the X Y Z components of the accelerometer so they match the gyro
        // axes
        let accel = Vector3::new(accel.y, -accel.x, accel.z);
        let quat = self.madgwick.update(gyro, accel);
        Ok(quat)
    }
}
