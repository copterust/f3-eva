use DT;

// Accelerometer angle (process) variance
const Q_ANGLE: f32 = 1e-3;
// const Q_ANGLE: f32 = 0.00249289093265333; // measured tilt variance

// Gyroscope bias (process) variance
const Q_BIAS: f32 = 3e-3;

// Observation (measurement) noise
// const R_OBS: f32 = 3e-2;
const R_OBS: f32 = 3e-2;
// const R_OBS: f32 = 4.945245206941632e-06; // gyro variance

pub struct Kalman {
    // estimate covariance
    p: [[f32; 2]; 2],
    // estimated pitch angle
    angle: f32,
    // gyroscope bias
    bias: f32,
}

impl Kalman {
    pub fn new(angle: f32, bias: f32) -> Self {
        Kalman {
            p: [[0.; 2]; 2],
            angle,
            bias,
        }
    }

    // - `angle` estimated using the accelerometer (rad)
    // - `omega` measured angular rate (rad / s)
    pub fn update(&mut self, angle: f32, omega: f32) -> f32 {
        let p = &mut self.p;

        // a priori estimate
        self.angle += (omega - self.bias) * DT;
        // no estimate for the bias

        // a priori P
        p[0][0] += DT * (DT * p[1][1] - p[0][1] - p[1][0] + Q_ANGLE);
        p[0][1] -= DT * p[1][1];
        p[1][0] -= DT * p[1][1];
        p[1][1] += DT * Q_BIAS;

        // innovation
        let y = angle - self.angle;

        // innovation covariance
        let s = p[0][0] + R_OBS;

        // Kalman gain
        let k = [p[0][0] / s, p[1][0] / s];

        // a posteriori estimate
        self.angle += k[0] * y;
        self.bias += k[1] * y;

        // a posteriori P
        let p00 = p[0][0];
        let p01 = p[0][1];

        p[0][0] -= k[0] * p00;
        p[0][1] -= k[0] * p01;
        p[1][0] -= k[1] * p00;
        p[1][1] -= k[1] * p01;

        self.angle
    }
}
