pub struct PID {
    T: f32,
    Kp: f32,
    Ki: f32,
    Kd: f32,
    u1: f32,
    e1: f32,
    e2: f32,
    g0: f32,
    g1: f32,
    g2: f32,
}

pub fn new(period: f32) -> PID {
    PID {
        T: period,
        Kp: 0.0,
        Ki: 0.0,
        Kd: 0.0,
        u1: 0.0,
        e1: 0.0,
        e2: 0.0,
        g0: 0.0,
        g1: 0.0,
        g2: 0.0
    }
}

impl PID {
    pub fn set(&mut self, p: f32, i: f32, d: f32) {
        self.g0 = self.Kp + self.Ki * self.T / 2.0 + self.Kd / self.T;
        self.g1 = -self.Kp + self.Ki * self.T / 2.0 - 2.0 * self.Kd / self.T;
        self.g2 = self.Kd / self.T;
    }

    pub fn output(&mut self, e: f32) -> f32 {
        let u = self.u1 + self.g0 * e + self.g1 * self.e1 + self.g2 * self.e2;
        self.u1 = u;
        u
    }
}
