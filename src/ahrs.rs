use ehal::blocking::delay::DelayMs;
use ehal::blocking::spi;
use ehal::digital::OutputPin;
use madgwick::{F32x3, Marg};
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

pub struct AHRS<SPI, NCS> {
    mpu: Mpu9250<SPI, NCS, mpu9250::Marg>,
    marg: madgwick::Marg,
    gyro_biases: F32x3,
}

impl<SPI, NCS> AHRS<SPI, NCS> {
    pub fn gyro_biases(&self) -> F32x3 {
        self.gyro_biases
    }
}

impl<SPI, NCS, E> AHRS<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    pub fn create_calibrated(mut mpu: Mpu9250<SPI, NCS, mpu9250::Marg>,
                             freq_sec: f32,
                             nsamples: u16,
                             delay: &mut impl DelayMs<u32>)
                             -> Result<Self, E> {
        // mpu.a_scale(mpu9250::FSScale::_01)?;
        // mpu.g_scale(mpu9250::FSScale::_01)?;
        let mut gyro_bias_x = 0.;
        let mut gyro_bias_y = 0.;
        let mut gyro_bias_z = 0.;
        for _ in 0..nsamples {
            let ar = mpu.gyro()?;
            gyro_bias_x += ar.x;
            gyro_bias_y += ar.y;
            gyro_bias_z += ar.z;
            delay.delay_ms(5);
        }
        let n = nsamples as f32;
        let gyro_biases = F32x3 { x: gyro_bias_x / n,
                                  y: gyro_bias_y / n,
                                  z: gyro_bias_z / n, };

        let marg = Marg::new(BETA, freq_sec);

        Ok(AHRS { mpu,
                  marg,
                  gyro_biases, })
    }

    pub fn read(&mut self) -> Result<madgwick::Quaternion, E> {
        let mag = self.mpu.mag()?;
        let accel = self.mpu.accel()?;
        let gyro = self.mpu.gyro()?;

        // Fix the X Y Z components of the magnetometer so they match the gyro
        // axes
        let mag = F32x3 { x: mag.y,
                          y: -mag.x,
                          z: mag.z, };

        // TODO: Add core::ops::* impls to madgwick repo
        let gyro_x = gyro.x - self.gyro_biases.x;
        let gyro_y = gyro.y - self.gyro_biases.y;
        let gyro_z = gyro.z - self.gyro_biases.z;
        let gyro = F32x3 { x: gyro_x,
                           y: gyro_y,
                           z: gyro_z, };

        // Fix the X Y Z components of the accelerometer so they match the gyro
        // axes
        let accel = F32x3 { x: accel.y,
                            y: -accel.x,
                            z: accel.z, };
        let quat = self.marg.update(mag, gyro, accel);
        Ok(quat)
    }
}
