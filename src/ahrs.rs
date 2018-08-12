use core::f32::consts::PI;

use ehal::blocking::delay::DelayMs;
use ehal::blocking::spi;
use ehal::digital::OutputPin;
use madgwick::{F32x3, Marg};
use mpu9250::Mpu9250;

// Magnetometer calibration parameters
// NOTE you need to use the right parameters for *your* magnetometer
// You can use the `log-sensors` example to calibrate your magnetometer. The
// producer is explained in https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration

const M_BIAS_X: f32 = 150.;
const M_SCALE_X: f32 = 0.9;
const M_BIAS_Y: f32 = -60.;
const M_SCALE_Y: f32 = 0.9;
const M_BIAS_Z: f32 = -220.;
const M_SCALE_Z: f32 = 1.2;
// Sensitivities of the accelerometer and gyroscope, respectively
const K_ACCEL: f32 = 2. / (1 << 15) as f32; // LSB -> g
const K_GYRO: f32 = 8.75e-3 * PI / 180.; // LSB -> rad/s

// Madgwick filter parameters
const BETA: f32 = 1e-3;

pub struct AHRS<SPI, NCS> {
    mpu: Mpu9250<SPI, NCS, mpu9250::Marg>,
    marg: madgwick::Marg,
    gyro_bias_x: i16,
    gyro_bias_y: i16,
    gyro_bias_z: i16,
}

impl<SPI, NCS> AHRS<SPI, NCS> {
    pub fn gyro_biases(&self) -> (i16, i16, i16) {
        (self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z)
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
        let nsamples = nsamples as i32;
        let mut gyro_bias_x = 0;
        let mut gyro_bias_y = 0;
        let mut gyro_bias_z = 0;
        for _ in 0..nsamples {
            let ar = mpu.gyro()?;
            gyro_bias_x += ar.x as i32;
            gyro_bias_y += ar.y as i32;
            gyro_bias_z += ar.z as i32;
            delay.delay_ms(5);
        }
        let gyro_bias_x = (gyro_bias_x / nsamples) as i16;
        let gyro_bias_y = (gyro_bias_y / nsamples) as i16;
        let gyro_bias_z = (gyro_bias_z / nsamples) as i16;
        let marg = Marg::new(BETA, freq_sec);

        Ok(AHRS { mpu,
                  marg,
                  gyro_bias_x,
                  gyro_bias_y,
                  gyro_bias_z, })
    }

    pub fn read(&mut self) -> Result<madgwick::Quaternion, E> {
        let mag = self.mpu.mag()?;
        let accel = self.mpu.accel()?;
        let gyro = self.mpu.gyro()?;

        let mag_x = ((mag.x as f32) - M_BIAS_X) / M_SCALE_X;
        let mag_y = ((mag.y as f32) - M_BIAS_Y) / M_SCALE_Y;
        let mag_z = ((mag.z as f32) - M_BIAS_Z) / M_SCALE_Z;

        // Fix the X Y Z components of the magnetometer so they match the gyro
        // axes
        let mag = F32x3 { x: mag_y,
                          y: -mag_x,
                          z: mag_z, };

        let gyro_x = ((gyro.x - self.gyro_bias_x) as f32) * K_GYRO;
        let gyro_y = ((gyro.y - self.gyro_bias_y) as f32) * K_GYRO;
        let gyro_z = ((gyro.z - self.gyro_bias_z) as f32) * K_GYRO;
        let gyro = F32x3 { x: gyro_x,
                           y: gyro_y,
                           z: gyro_z, };

        // Fix the X Y Z components of the accelerometer so they match the gyro
        // axes
        let accel_x = (accel.x as f32) * K_ACCEL;
        let accel_y = (accel.y as f32) * K_ACCEL;
        let accel_z = (accel.z as f32) * K_ACCEL;
        let accel = F32x3 { x: accel_y,
                            y: -accel_x,
                            z: accel_z, };
        let quat = self.marg.update(mag, gyro, accel);
        Ok(quat)
    }
}
