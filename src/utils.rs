/** Join two bytes into word. */
pub fn u8_u8_to_u16(msb: u8, lsb: u8) -> u16 {
    ((msb as u16) << 8) | (lsb as u16)
}

/** Split word into two bytes. */
pub fn u16_to_u8_u8(word: u16) -> (u8, u8) {
    (((word >> 8) & 0xFF) as u8, (word & 0xFF) as u8)
}

/** Reset given bit b in r to 0. */
pub fn res_r8(b: u8, r: u8) -> u8 {
    r & !(1 << b)
}

/** Set given bit b in r to 1. */
pub fn set_r8(b: u8, r: u8) -> u8 {
    r | (1 << b)
}

/** Update given bit b in r according to given boolean value. */
pub fn update_r8(b: u8, r: u8, value: bool) -> u8 {
    (r & !(1 << b)) | ((value as u8) << b)
}

/** Take two bytes, join them into word, decrement by 1 and return the two bytes. */
pub fn dec_r16(msb: u8, lsb: u8) -> (u8, u8) {
    u16_to_u8_u8(u8_u8_to_u16(msb, lsb).wrapping_sub(1))
}

/** Take two bytes, join them into word, increment by 1 and return the two bytes. */
pub fn inc_r16(msb: u8, lsb: u8) -> (u8, u8) {
    u16_to_u8_u8(u8_u8_to_u16(msb, lsb).wrapping_add(1))
}
