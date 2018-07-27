#![allow(unused)]
macro_rules! itoa_impl {
    ($num:expr, $num_digits:expr, $buffer:ident) => {
        let mut x = $num;
        $buffer[$num_digits - 1] = '0' as u8;
        let mut i = $num_digits - 1;
        while x > 0 {
            let digit = (x % 10) as u8;
            x = x / 10;
            $buffer[i] = ('0' as u8 + digit) as u8;
            i -= 1;
        }
    };
}

pub fn itoa_u32(num: u32) -> [u8; 10] {
    const NUM_DIGITS: usize = 10;
    let mut buffer: [u8; NUM_DIGITS] = [' ' as u8; NUM_DIGITS];
    itoa_impl!(num, NUM_DIGITS, buffer);
    return buffer;
}

pub fn itoa_i16(num: i16) -> [u8; 6] {
    const NUM_DIGITS: usize = 6;
    let mut buffer: [u8; NUM_DIGITS] = [' ' as u8; NUM_DIGITS];
    itoa_impl!(num, NUM_DIGITS, buffer);
    return buffer;
}

pub fn itoa_u16(num: u16) -> [u8; 5] {
    const NUM_DIGITS: usize = 5;
    let mut buffer: [u8; NUM_DIGITS] = [' ' as u8; NUM_DIGITS];
    itoa_impl!(num, NUM_DIGITS, buffer);
    return buffer;
}

pub fn itoa_u8(num: u8) -> [u8; 3] {
    const NUM_DIGITS: usize = 3;
    let mut buffer: [u8; NUM_DIGITS] = [' ' as u8; NUM_DIGITS];
    itoa_impl!(num, NUM_DIGITS, buffer);
    return buffer;
}
