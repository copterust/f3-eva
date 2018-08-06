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
const M_BIAS_X: f32 = -34.;
const M_SCALE_X: f32 = 650.;
const M_BIAS_Y: f32 = -70.;
const M_SCALE_Y: f32 = 636.;
const M_BIAS_Z: f32 = -37.5;
const M_SCALE_Z: f32 = 589.5;
// Sensitivities of the accelerometer and gyroscope, respectively
const K_G: f32 = 2. / (1 << 15) as f32; // LSB -> g
const K_AR: f32 = 8.75e-3 * PI / 180.; // LSB -> rad/s

// Madgwick filter parameters
const SAMPLE_FREQ: f32 = 220.;
const BETA: f32 = 1e-3;

pub struct AHRS<SPI, NCS> {
    mpu: Mpu9250<SPI, NCS, mpu9250::Marg>,
    marg: madgwick::Marg,
    ar_bias_x: i16,
    ar_bias_y: i16,
    ar_bias_z: i16,
}

impl<SPI, NCS> AHRS<SPI, NCS> {
    pub fn ar_biases(&self) -> (i16, i16, i16) {
        (self.ar_bias_x, self.ar_bias_y, self.ar_bias_z)
    }
}

impl<SPI, NCS, E> AHRS<SPI, NCS>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          NCS: OutputPin
{
    pub fn create_calibrated(mut mpu: Mpu9250<SPI, NCS, mpu9250::Marg>,
                             nsamples: u16,
                             delay: &mut impl DelayMs<u32>)
                             -> Result<Self, E> {
        mpu.a_scale(mpu9250::FSScale::_01)?;
        mpu.g_scale(mpu9250::FSScale::_01)?;
        let nsamples = nsamples as i32;
        let mut ar_bias_x = 0;
        let mut ar_bias_y = 0;
        let mut ar_bias_z = 0;
        for _ in 0..nsamples {
            let ar = mpu.gyro()?;
            ar_bias_x += ar.x as i32;
            ar_bias_y += ar.y as i32;
            ar_bias_z += ar.z as i32;
            delay.delay_ms(5);
        }
        let ar_bias_x = (ar_bias_x / nsamples) as i16;
        let ar_bias_y = (ar_bias_y / nsamples) as i16;
        let ar_bias_z = (ar_bias_z / nsamples) as i16;
        let marg = Marg::new(BETA, 1. / SAMPLE_FREQ);

        Ok(AHRS { mpu,
                  marg,
                  ar_bias_x,
                  ar_bias_y,
                  ar_bias_z, })
    }

    pub fn read(&mut self) -> Result<madgwick::Quaternion, E> {
        let m = self.mpu.mag()?;
        let ar = self.mpu.gyro()?;
        let g = self.mpu.accel()?;

        let m_x = ((m.x as f32) - M_BIAS_X) / M_SCALE_X;
        let m_y = ((m.y as f32) - M_BIAS_Y) / M_SCALE_Y;
        let m_z = ((m.z as f32) - M_BIAS_Z) / M_SCALE_Z;

        // Fix the X Y Z components of the magnetometer so they match the gyro
        // axes
        let m = F32x3 { x: m_y,
                        y: -m_x,
                        z: m_z, };

        let ar_x = ((ar.x - self.ar_bias_x) as f32) * K_AR;
        let ar_y = ((ar.y - self.ar_bias_y) as f32) * K_AR;
        let ar_z = ((ar.z - self.ar_bias_z) as f32) * K_AR;
        let ar = F32x3 { x: ar_x,
                         y: ar_y,
                         z: ar_z, };

        // Fix the X Y Z components of the accelerometer so they match the gyro
        // axes
        let g_x = (g.x as f32) * K_G;
        let g_y = (g.y as f32) * K_G;
        let g_z = (g.z as f32) * K_G;
        let g = F32x3 { x: g_y,
                        y: -g_x,
                        z: g_z, };
        let quat = self.marg.update(m, ar, g);
        Ok(quat)
    }
}
