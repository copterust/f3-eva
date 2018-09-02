const BUFFER_SIZE: usize = 32;
const CR: u8 = '\r' as u8;
const LF: u8 = '\n' as u8;

pub struct Cmd {
    buffer: [u8; BUFFER_SIZE],
    pos: usize,
}

impl Cmd {
    pub fn new() -> Cmd {
        Cmd { buffer: [0; BUFFER_SIZE], pos: 0 }
    }

    pub fn push(&mut self, b: u8) -> Option<&[u8]> {
        if b == CR || b == LF {
            if (self.pos == 0) {
                None
            } else {
                let result = &self.buffer[0..self.pos];
                self.pos = 0;
                Some(result)
            }
        } else {
            self.buffer[self.pos] = b;
            self.pos = (self.pos + 1) & (BUFFER_SIZE - 1);
            None
        }
    }
}
