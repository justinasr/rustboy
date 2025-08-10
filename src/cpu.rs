use super::memory::Memory;
use super::utils::*;

// macro_rules! cpulogln {
//     ($($arg:tt)*) => (print!($($arg)*);print!("\n"));
// }
// macro_rules! cpulog {
//     ($($arg:tt)*) => (print!($($arg)*));
// }

macro_rules! cpulogln {
    ($($arg:tt)*) => {};
}
macro_rules! cpulog {
    ($($arg:tt)*) => {};
}

pub struct CPU {
    pc: u16, // Program Counter (PC)
    sp: u16, // Stack Pointer (SP)

    a: u8, // Accumulator (A)
    // Flag register (F) bits:
    // 7 6 5 4 3 2 1 0
    // Z N H C 0 0 0 0
    f: u8, // Flags (F)

    b: u8, // Register (B)
    c: u8, // Register (C)

    d: u8, // Register (D)
    e: u8, // Register (E)

    h: u8, // Register (H)
    l: u8, // Register (L)

    ime: bool,
    should_enable_ime: bool,
    total_cycles: u16,
    halt: bool,
}

impl CPU {
    pub fn new() -> CPU {
        return CPU {
            pc: 0x100, // After bootrom PC is pointing to 0x100
            sp: 0,
            a: 0,
            f: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
            ime: true,
            should_enable_ime: false,
            total_cycles: 0,
            halt: false,
        };
    }

    // Flags

    fn set_zero_flag(&mut self, value: bool) {
        if value {
            self.f |= 1 << 7;
        } else {
            self.f &= !(1 << 7);
        }
    }

    fn set_subtraction_flag(&mut self, value: bool) {
        if value {
            self.f |= 1 << 6;
        } else {
            self.f &= !(1 << 6);
        }
    }

    fn set_half_carry_flag(&mut self, value: bool) {
        if value {
            self.f |= 1 << 5;
        } else {
            self.f &= !(1 << 5);
        }
    }

    fn set_carry_flag(&mut self, value: bool) {
        if value {
            self.f |= 1 << 4;
        } else {
            self.f &= !(1 << 4);
        }
    }

    fn get_zero_flag(&self) -> bool {
        return self.f & (1 << 7) != 0;
    }

    fn get_subtraction_flag(&self) -> bool {
        return self.f & (1 << 6) != 0;
    }

    fn get_half_carry_flag(&self) -> bool {
        return self.f & (1 << 5) != 0;
    }

    fn get_carry_flag(&self) -> bool {
        return self.f & (1 << 4) != 0;
    }

    // Instructions

    /** Decrement 8-bit value. Set Z, H and N flags. */
    fn dec_r8(&mut self, r: u8) -> u8 {
        let rr = r.wrapping_sub(1);
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(true);
        // If lower nibble is 0, it will carry from upper nibble
        self.set_half_carry_flag((r & 0x0F) == 0);
        rr
    }

    /** Increment 8-bit value. Set Z, H and N flags. */
    fn inc_r8(&mut self, r: u8) -> u8 {
        let rr = r.wrapping_add(1);
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(false);
        // If lower nibble is 0xF, adding 1 will move to upper nibble
        self.set_half_carry_flag((r & 0x0F) == 0x0F);
        rr
    }

    /** Rotate bits in given value left, through the carry flag. */
    fn rl_r8(&mut self, r: u8) -> u8 {
        let rr = (r << 1) | if self.get_carry_flag() { 1 } else { 0 };
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        self.set_carry_flag(r & 0x80 != 0);
        rr
    }

    /** Rotate bits in given value right, through the carry flag. */
    fn rr_r8(&mut self, r: u8) -> u8 {
        let rr = (r >> 1) | if self.get_carry_flag() { 1 << 7 } else { 0 };
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        self.set_carry_flag(r & 0x01 != 0);
        rr
    }

    /** Shift left into carry, LSB is set to 0. */
    fn sla_r8(&mut self, r: u8) -> u8 {
        let rr = (r << 1) & 0xFE;
        self.set_zero_flag(rr == 0);
        self.set_half_carry_flag(false);
        self.set_subtraction_flag(false);
        self.set_carry_flag(r & 0x80 != 0);
        rr
    }

    /** Shift right into carry, MSB does not change. */
    fn sra_r8(&mut self, r: u8) -> u8 {
        let rr = (r >> 1) | (r & 0x80);
        self.set_zero_flag(rr == 0);
        self.set_half_carry_flag(false);
        self.set_subtraction_flag(false);
        self.set_carry_flag(r & 0x01 != 0);
        rr
    }

    /** Test bit b in given value, set the zero flag accordingly. */
    fn bit_r8(&mut self, b: u8, r: u8) {
        self.set_zero_flag(r & (1 << b) == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(true);
    }

    /** Subtract given value from register A value. */
    fn sub_r8(&mut self, r: u8) -> u8 {
        let a = self.a.wrapping_sub(r);
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(true);
        self.set_half_carry_flag((self.a & 0x0F) < (r & 0x0F));
        self.set_carry_flag(self.a < r);
        a
    }

    /** Subtract given value from register A value with carry. */
    fn sbc_r8(&mut self, r: u8) -> u8 {
        let c = if self.get_carry_flag() { 1 } else { 0 };
        let a = self.a.wrapping_sub(r).wrapping_sub(c);
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(true);
        self.set_half_carry_flag((self.a & 0x0F) < ((r & 0x0F) + c));
        self.set_carry_flag((self.a as u16) < (r as u16) + (c as u16));
        a
    }

    /** Add given value to register A value. */
    fn add_r8(&mut self, r: u8) -> u8 {
        let a = self.a.wrapping_add(r);
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag((self.a & 0x0F) + (r & 0x0F) > 0x0F);
        self.set_carry_flag((self.a as u16) + (r as u16) > 0xFF);
        a
    }

    /** Add given word to HL register value. */
    fn add_r16(&mut self, rr: u16) -> u16 {
        let hl = self.get_hl().wrapping_add(rr);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag((self.get_hl() & 0x0FFF) + (rr & 0x0FFF) > 0x0FFF);
        self.set_carry_flag((self.get_hl() as u32) + (rr as u32) > 0xFFFF);
        hl
    }

    /** Add given value with carry to register A value. */
    fn adc_r8(&mut self, r: u8) -> u8 {
        let c = if self.get_carry_flag() { 1 } else { 0 };
        let a = self.a.wrapping_add(r).wrapping_add(c);
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag((self.a & 0x0F) + (r & 0x0F) + c > 0x0F);
        self.set_carry_flag((self.a as u16) + (r as u16) + (c as u16) > 0xFF);
        a
    }

    /** Compare given value with register A value and set the flags. */
    fn cp_r8(&mut self, r: u8) {
        let a = self.a;
        self.set_zero_flag(a == r);
        self.set_subtraction_flag(true);
        self.set_half_carry_flag((a & 0x0F) < (r & 0x0F));
        self.set_carry_flag(a < r);
    }

    /** Bitwise AND given value with register A value. */
    fn and_r8(&mut self, r: u8) -> u8 {
        let a = self.a & r;
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(true);
        self.set_carry_flag(false);
        a
    }

    /** Bitwise OR given value with register A value. */
    fn or_r8(&mut self, r: u8) -> u8 {
        let a = self.a | r;
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        self.set_carry_flag(false);
        a
    }

    /** XOR given value with A register. Set Z flag. Reset N, H, C flags. */
    fn xor(&mut self, r: u8) -> u8 {
        let a = self.a ^ r;
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        self.set_carry_flag(false);
        a
    }

    /** Swap the nibbles of given value and set the flags. */
    fn swap_r8(&mut self, r: u8) -> u8 {
        self.set_zero_flag(r == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        self.set_carry_flag(false);
        (r << 4) & (0xF0) | (r >> 4) & (0x0F)
    }

    fn rlc_r8(&mut self, r: u8) -> u8 {
        self.set_carry_flag(r & 0b1000_0000 != 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        let nn = (r << 1) | ((r >> 7) & 0x01);
        self.set_zero_flag(nn == 0);
        nn
    }

    fn rrc_r8(&mut self, r: u8) -> u8 {
        self.set_carry_flag(r & 0b0000_0001 != 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        let nn = (r >> 1) | ((r << 7) & 0b1000_0000);
        self.set_zero_flag(nn == 0);
        nn
    }

    fn srl_r8(&mut self, r: u8) -> u8 {
        self.set_carry_flag(r & 0b0000_0001 != 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        let nn = (r >> 1) | ((r << 7) & 0b0111_1111);
        self.set_zero_flag(nn == 0);
        nn
    }

    // Instruction/operand loading

    fn load_byte(&mut self, memory: &Memory) -> u8 {
        let nn = memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        nn
    }

    fn load_word(&mut self, memory: &Memory) -> u16 {
        let nn = memory.read_word(self.pc);
        self.pc = self.pc.wrapping_add(2);
        nn
    }

    // Stack operations

    fn pop_word(&mut self, memory: &Memory) -> u16 {
        let nn = memory.read_word(self.sp);
        self.sp = self.sp.wrapping_add(2);
        nn
    }

    fn push_word(&mut self, memory: &mut Memory, nn: u16) {
        self.sp = self.sp.wrapping_sub(2);
        memory.write_word(self.sp, nn);
    }

    // Register pair

    fn get_bc(&self) -> u16 {
        return u8_u8_to_u16(self.b, self.c);
    }

    fn set_bc(&mut self, bc: u16) {
        (self.b, self.c) = u16_to_u8_u8(bc);
    }

    fn get_de(&self) -> u16 {
        return u8_u8_to_u16(self.d, self.e);
    }

    fn set_de(&mut self, de: u16) {
        (self.d, self.e) = u16_to_u8_u8(de);
    }

    fn get_hl(&self) -> u16 {
        return u8_u8_to_u16(self.h, self.l);
    }

    fn set_hl(&mut self, hl: u16) {
        (self.h, self.l) = u16_to_u8_u8(hl);
    }

    fn get_af(&self) -> u16 {
        return u8_u8_to_u16(self.a, self.f);
    }

    fn set_af(&mut self, af: u16) {
        (self.a, self.f) = u16_to_u8_u8(af & 0xFFF0);
    }

    // Register getter

    fn get_mut_reg(&mut self, i: u8) -> &mut u8 {
        match i {
            0 => &mut self.b,
            1 => &mut self.c,
            2 => &mut self.d,
            3 => &mut self.e,
            4 => &mut self.h,
            5 => &mut self.l,
            6 => panic!("(HL) is memory, not register"),
            7 => &mut self.a,
            _ => unreachable!("Unexpected register index {i}"),
        }
    }

    fn get_reg(&self, memory: &Memory, i: u8) -> u8 {
        let hl = self.get_hl();
        match i {
            0 => self.b,
            1 => self.c,
            2 => self.d,
            3 => self.e,
            4 => self.h,
            5 => self.l,
            6 => memory.read_byte(hl),
            7 => self.a,
            _ => unreachable!("Unexpected register index {i}"),
        }
    }

    fn get_reg_name(&self, i: u8) -> &str {
        match i {
            0 => "B",
            1 => "C",
            2 => "D",
            3 => "E",
            4 => "H",
            5 => "L",
            6 => "(HL)",
            7 => "A",
            _ => unreachable!("Unexpected register index {i}"),
        }
    }

    fn execute_instruction(&mut self, memory: &mut Memory) -> u8 {
        let opcode = self.load_byte(&memory);
        if opcode != 0xCB {
            cpulog!(
                "[CPU] PC = {:#06x} | {:#04x}   | ",
                self.pc.wrapping_sub(1),
                opcode
            );
        }
        let cycles: u8;

        match opcode {
            0x00 => {
                cpulogln!("NOP");
                cycles = 4;
            }
            0x01 => {
                let nn = self.load_word(memory);
                cpulogln!("LD BC, d16     | LD BC {:#06x}", nn);
                self.set_bc(nn);
                cycles = 12;
            }
            0x02 => {
                let addr = self.get_bc();
                cpulogln!("LD (BC), A     | LD ({:#06x}), {:#04x}", addr, self.a);
                memory.write_byte(addr, self.a);
                cycles = 8;
            }
            0x03 => {
                cpulogln!("INC BC         | INC {:#06x}", self.get_bc());
                (self.b, self.c) = inc_r16(self.b, self.c);
                cycles = 8;
            }
            0x04 => {
                cpulogln!("INC B          | INC {:#04x}", self.b);
                self.b = self.inc_r8(self.b);
                cycles = 4;
            }
            0x05 => {
                cpulogln!("DEC B          | DEC {:#04x}", self.b);
                self.b = self.dec_r8(self.b);
                cycles = 4;
            }
            0x06 => {
                self.b = self.load_byte(memory);
                cpulogln!("LD B, d8       | LD B, {:#04x}", self.b);
                cycles = 8;
            }
            0x07 => {
                cpulogln!("RLCA");
                self.a = self.rlc_r8(self.a);
                self.set_zero_flag(false);
                cycles = 4;
            }
            0x08 => {
                let addr = self.load_word(memory);
                cpulogln!("LD (a16), SP   | LD ({:#06x}), {:#06x}", addr, self.sp);
                memory.write_word(addr, self.sp);
                cycles = 20;
            }
            0x09 => {
                cpulogln!("ADD HL, BC     | ADD HL, {:#06x}", self.get_bc());
                let hl = self.add_r16(self.get_bc());
                self.set_hl(hl);
                cycles = 8;
            }
            0x0A => {
                let addr = self.get_bc();
                cpulogln!("LD A, (BC)     | LD A, ({:#06x})", addr);
                self.a = memory.read_byte(addr);
                cycles = 8;
            }
            0x0B => {
                cpulogln!("DEC BC         | DEC {:#06x}", self.get_bc());
                (self.b, self.c) = dec_r16(self.b, self.c);
                cycles = 8;
            }
            0x0C => {
                cpulogln!("INC C          | INC {:#04x}", self.c);
                self.c = self.inc_r8(self.c);
                cycles = 4;
            }
            0x0D => {
                cpulogln!("DEC C          | DEC C {:#04x}", self.c);
                self.c = self.dec_r8(self.c);
                cycles = 4;
            }
            0x0E => {
                self.c = self.load_byte(memory);
                cpulogln!("LD C, d8       | LD C, {:#04x}", self.c);
                cycles = 8;
            }
            0x0F => {
                cpulogln!("RRCA");
                self.a = self.rrc_r8(self.a);
                self.set_zero_flag(false);
                cycles = 4;
            }
            0x10 => {
                cpulogln!("STOP");
                self.halt = true;
                cycles = 4;
            }
            0x11 => {
                let nn = self.load_word(memory);
                cpulogln!("LD DE, d16     | LD DE, {:#06x}", nn);
                self.set_de(nn);
                cycles = 12;
            }
            0x12 => {
                let addr = self.get_de();
                cpulogln!("LD (DE), A     | LD ({:#06x}), {:#04x}", addr, self.a);
                memory.write_byte(addr, self.a);
                cycles = 8;
            }
            0x13 => {
                cpulogln!("DEC DE         | INC DE {:#06x}", self.get_de());
                (self.d, self.e) = inc_r16(self.d, self.e);
                cycles = 8;
            }
            0x14 => {
                cpulogln!("INC D          | INC {:#04x}", self.d);
                self.d = self.inc_r8(self.d);
                cycles = 4;
            }
            0x15 => {
                cpulogln!("DEC D          | DEC {:#04x}", self.d);
                self.d = self.dec_r8(self.d);
                cycles = 4;
            }
            0x16 => {
                self.d = self.load_byte(memory);
                cpulogln!("LD D, d8       | LD D, {:#04x}", self.d);
                cycles = 8;
            }
            0x17 => {
                cpulogln!("RLA");
                self.a = self.rl_r8(self.a);
                self.set_zero_flag(false);
                cycles = 4;
            }
            0x18 => {
                let nn = self.load_byte(memory);
                cpulogln!("JR r8          | JR {:#04x}", nn);
                self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                cycles = 12;
            }
            0x19 => {
                cpulogln!("ADD HL, DE     | ADD HL, {:#06x}", self.get_hl());
                let hl = self.add_r16(self.get_de());
                self.set_hl(hl);
                cycles = 8;
            }
            0x1A => {
                cpulogln!("LD A, (DE)     | LD A, ({:#06x})", self.get_de());
                let addr = self.get_de();
                self.a = memory.read_byte(addr);
                cycles = 8;
            }
            0x1B => {
                cpulogln!("DEC DE         | DEC {:#06x}", self.get_de());
                (self.d, self.e) = dec_r16(self.d, self.e);
                cycles = 8;
            }
            0x1C => {
                cpulogln!("INC E          | INC {:#04x}", self.e);
                self.e = self.inc_r8(self.e);
                cycles = 4;
            }
            0x1D => {
                cpulogln!("DEC E          | DEC {:#04x}", self.e);
                self.e = self.dec_r8(self.e);
                cycles = 4;
            }
            0x1E => {
                self.e = self.load_byte(memory);
                cpulogln!("LD E, d8       | LD D, {:#04x}", self.e);
                cycles = 8;
            }
            0x1F => {
                cpulogln!("RRA");
                self.a = self.rr_r8(self.a);
                self.set_zero_flag(false);
                cycles = 4;
            }
            0x20 => {
                let nn = self.load_byte(memory);
                if !self.get_zero_flag() {
                    cpulogln!("JR NZ, r8      | JR {:#04x}", nn);
                    self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                    cycles = 12;
                } else {
                    cpulogln!("JR NZ, r8      | <no jump>");
                    cycles = 8;
                }
            }
            0x21 => {
                let nn = self.load_word(memory);
                cpulogln!("LD HL, d16     | LD HL, {:#06x}", nn);
                self.set_hl(nn);
                cycles = 12;
            }
            0x22 => {
                let addr = self.get_hl();
                cpulogln!("LD (HL+), A    | LD ({:#06x}), {:#04x}", addr, self.a);
                memory.write_byte(addr, self.a);
                (self.h, self.l) = inc_r16(self.h, self.l);
                cycles = 8;
            }
            0x23 => {
                cpulogln!("INC HL         | INC {:#06x}", self.get_hl());
                (self.h, self.l) = inc_r16(self.h, self.l);
                cycles = 8;
            }
            0x24 => {
                cpulogln!("INC H          | INC {:#04x}", self.h);
                self.h = self.inc_r8(self.h);
                cycles = 4;
            }
            0x25 => {
                cpulogln!("DEC H          | DEC {:#04x}", self.h);
                self.h = self.dec_r8(self.h);
                cycles = 4;
            }
            0x26 => {
                self.h = self.load_byte(memory);
                cpulogln!("LD H, d8       | LD H, {:#04x}", self.h);
                cycles = 8;
            }
            0x27 => {
                cpulogln!("DAA");
                if self.get_subtraction_flag() {
                    if self.get_carry_flag() {
                        self.a = self.a.wrapping_sub(0x60);
                    }
                    if self.get_half_carry_flag() {
                        self.a = self.a.wrapping_sub(0x06);
                    }
                } else {
                    if self.get_carry_flag() || (self.a & 0xFF) > 0x99 {
                        self.a = self.a.wrapping_add(0x60);
                        self.set_carry_flag(true);
                    }
                    if self.get_half_carry_flag() || (self.a & 0x0F) > 0x09 {
                        self.a = self.a.wrapping_add(0x06);
                    }
                }

                self.set_zero_flag(self.a == 0);
                self.set_half_carry_flag(false);
                cycles = 4;
            }
            0x28 => {
                let nn = self.load_byte(memory);
                if self.get_zero_flag() {
                    cpulogln!("JR Z, r8       | JR {:#04x}", nn);
                    self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                    cycles = 12;
                } else {
                    cpulogln!("JR Z, r8       | <no jump>");
                    cycles = 8;
                }
            }
            0x29 => {
                cpulogln!("ADD HL, HL     | ADD HL, {:#06x}", self.get_hl());
                let hl = self.add_r16(self.get_hl());
                self.set_hl(hl);
                cycles = 8;
            }
            0x2A => {
                let addr = self.get_hl();
                cpulogln!("LD A, (HL+)    | LD A, ({:#06x})", addr);
                self.a = memory.read_byte(addr);
                (self.h, self.l) = inc_r16(self.h, self.l);
                cycles = 8;
            }
            0x2B => {
                cpulogln!("DEC HL         | DEC {:#06x}", self.get_hl());
                (self.h, self.l) = dec_r16(self.h, self.l);
                cycles = 8;
            }
            0x2C => {
                cpulogln!("INC L          | INC {:#04x}", self.l);
                self.l = self.inc_r8(self.l);
                cycles = 4;
            }
            0x2D => {
                cpulogln!("DEC L          | DEC {:#04x}", self.l);
                self.l = self.dec_r8(self.l);
                cycles = 4;
            }
            0x2E => {
                cpulogln!("LD L, d8       | LD L, {:#04x}", self.l);
                self.l = self.load_byte(memory);
                cycles = 8;
            }
            0x2F => {
                cpulogln!("CPL");
                self.a = !self.a;
                self.set_subtraction_flag(true);
                self.set_half_carry_flag(true);
                cycles = 4;
            }
            0x30 => {
                let nn = self.load_byte(memory);
                if !self.get_carry_flag() {
                    cpulogln!("JR NC, r8      | JR {:#04x}", nn);
                    self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                    cycles = 12;
                } else {
                    cpulogln!("JR NC, r8      | <no jump>");
                    cycles = 8;
                }
            }
            0x31 => {
                self.sp = self.load_word(memory);
                cpulogln!("LD SP, d16     | LD SP, {:#06x}", self.sp);
                cycles = 12;
            }
            0x32 => {
                let addr = self.get_hl();
                memory.write_byte(addr, self.a);
                (self.h, self.l) = dec_r16(self.h, self.l);
                cpulogln!("LD (HL-), A    | LD ({:#06x}), {:#04x}", addr, self.a);
                cycles = 8;
            }
            0x33 => {
                cpulogln!("INC SP         | INC {:#06x}", self.sp);
                self.sp = self.sp.wrapping_add(1);
                cycles = 8;
            }
            0x34 => {
                let addr = self.get_hl();
                cpulogln!("INC (HL)       | INC ({:#06x})", addr);
                let nn = self.inc_r8(memory.read_byte(addr));
                memory.write_byte(addr, nn);
                cycles = 4;
            }
            0x35 => {
                let addr = self.get_hl();
                cpulogln!("DEC (HL)       | DEC ({:#06x})", addr);
                let nn = self.dec_r8(memory.read_byte(addr));
                memory.write_byte(addr, nn);
                cycles = 4;
            }
            0x36 => {
                let nn = self.load_byte(memory);
                let addr = self.get_hl();
                cpulogln!("LD (HL), d8    | LD ({:#06x}), {:#04x}", addr, nn);
                memory.write_byte(addr, nn);
                cycles = 12;
            }
            0x37 => {
                cpulogln!("SCF");
                self.set_subtraction_flag(false);
                self.set_half_carry_flag(false);
                self.set_carry_flag(true);
                cycles = 4;
            }
            0x38 => {
                let nn = self.load_byte(memory);
                if self.get_carry_flag() {
                    cpulogln!("JR C, r8       | JR {:#04x}", nn);
                    self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                    cycles = 12;
                } else {
                    cpulogln!("JR C, r8       | <no jump>");
                    cycles = 8;
                }
            }
            0x39 => {
                cpulogln!("ADD HL, SP     | ADD HL, {:#06x}", self.sp);
                let hl = self.add_r16(self.sp);
                self.set_hl(hl);
                cycles = 8;
            }
            0x3A => {
                let addr = self.get_hl();
                cpulogln!("LD A, (HL-)    | LD A, ({:#06x})", addr);
                self.a = memory.read_byte(addr);
                (self.h, self.l) = dec_r16(self.h, self.l);
                cycles = 8;
            }
            0x3B => {
                cpulogln!("DEC SP         | DEC {:#06x}", self.sp);
                self.sp = self.sp.wrapping_sub(1);
                cycles = 8;
            }
            0x3C => {
                cpulogln!("INC A          | INC {:#04x}", self.a);
                self.a = self.inc_r8(self.a);
                cycles = 4;
            }
            0x3D => {
                cpulogln!("DEC A          | DEC {:#04x}", self.a);
                self.a = self.dec_r8(self.a);
                cycles = 4;
            }
            0x3E => {
                self.a = self.load_byte(memory);
                cpulogln!("LD A, d8       | LD A, {:#04x}", self.a);
                cycles = 8;
            }
            0x3F => {
                cpulogln!("CCF");
                self.set_subtraction_flag(false);
                self.set_half_carry_flag(false);
                self.set_carry_flag(!self.get_carry_flag());
                cycles = 4;
            }
            0x40..=0x75 | 0x77..=0x7F => {
                let lhs_i = (opcode >> 3) & 0b0111; // Upper nibble % 8
                let rhs_i = opcode & 0b0111; // Lower nibble % 8
                cpulogln!(
                    "LD {:<4}, {:<4}  | LD {}, {:#04x}",
                    self.get_reg_name(lhs_i),
                    self.get_reg_name(rhs_i),
                    self.get_reg_name(lhs_i),
                    self.get_reg(memory, rhs_i)
                );
                if lhs_i == 6 {
                    memory.write_byte(self.get_hl(), self.get_reg(memory, rhs_i));
                } else {
                    *self.get_mut_reg(lhs_i) = self.get_reg(memory, rhs_i);
                }
                cycles = if lhs_i == 6 || rhs_i == 6 { 8 } else { 4 };
            }
            0x76 => {
                cpulogln!("HALT");
                self.halt = true;
                cycles = 4;
            }
            0x80..=0x87 => {
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                cpulogln!("ADD A, {:<4}    | ADD A, {:#04x}", self.get_reg_name(i), nn);
                self.a = self.add_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0x88..=0x8F => {
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                cpulogln!("ADC A, {:<4}    | ADC A, {:#04x}", self.get_reg_name(i), nn);
                self.a = self.adc_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0x90..=0x97 => {
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                cpulogln!("SUB {:<4}       | SUB {:#04x}", self.get_reg_name(i), nn);
                self.a = self.sub_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0x98..=0x9F => {
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                cpulogln!("SBC A, {:<4}    | SBC A, {:#04x}", self.get_reg_name(i), nn);
                self.a = self.sbc_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xA0..=0xA7 => {
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                cpulogln!("AND {:<4}       | AND {:#04x}", self.get_reg_name(i), nn);
                self.a = self.and_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xA8..=0xAF => {
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                cpulogln!("XOR {:<4}       | XOR {:#04x}", self.get_reg_name(i), nn);
                self.a = self.xor(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xB0..=0xB7 => {
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                cpulogln!("OR {:<4}        | OR {:#04x}", self.get_reg_name(i), nn);
                self.a = self.or_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xB8..=0xBF => {
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                cpulogln!("CP {:<4}          | CP {:#04x}", self.get_reg_name(i), nn);
                self.cp_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xC0 => {
                if !self.get_zero_flag() {
                    cpulogln!("RET NZ         | RET {:#06x}", self.pc);
                    self.pc = self.pop_word(memory);
                    cycles = 20;
                } else {
                    cpulogln!("RET NZ         | <no return>");
                    cycles = 8;
                }
            }
            0xC1 => {
                let nn = self.pop_word(memory);
                cpulogln!("POP BC         | BC = {:#06x}", nn);
                self.set_bc(nn);
                cycles = 12;
            }
            0xC2 => {
                let nn = self.load_word(memory);
                if !self.get_zero_flag() {
                    cpulogln!("JR NZ, a16     | JR NZ, {:#06x}", nn);
                    self.pc = nn;
                    cycles = 16;
                } else {
                    cpulogln!("JR NZ, a16     | <no jump>");
                    cycles = 12
                }
            }
            0xC3 => {
                self.pc = self.load_word(memory);
                cpulogln!("JP a16         | JP {:#06x}", self.pc);
                cycles = 16;
            }
            0xC4 => {
                cpulogln!(" CALL NZ, a16");
                let nn = self.load_word(memory);
                if !self.get_zero_flag() {
                    cpulogln!("CALL NZ, a16   | CALL {:#06x}", nn);
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    cycles = 24;
                } else {
                    cpulogln!("CALL NZ, a16   | <no call>");
                    cycles = 12;
                }
            }
            0xC5 => {
                cpulogln!("PUSH BC        | PUSH {:#06x}", self.get_bc());
                self.push_word(memory, self.get_bc());
                cycles = 16;
            }
            0xC6 => {
                let nn = self.load_byte(memory);
                cpulogln!("ADD A, d8      | ADD A, {:#04x}", nn);
                self.a = self.add_r8(nn);
                cycles = 8;
            }
            0xC7 => {
                cpulogln!("RST 00H");
                self.push_word(memory, self.pc);
                self.pc = 0x0000;
                cycles = 16;
            }
            0xC8 => {
                if self.get_zero_flag() {
                    self.pc = self.pop_word(memory);
                    cpulogln!("RET Z          | RET {:#06x}", self.pc);
                    cycles = 20;
                } else {
                    cpulogln!("RET Z          | <no return>");
                    cycles = 8;
                }
            }
            0xC9 => {
                self.pc = self.pop_word(memory);
                cpulogln!("RET            | RET {:#06x}", self.pc);
                cycles = 16;
            }
            0xCA => {
                let nn = self.load_word(memory);
                if self.get_zero_flag() {
                    cpulogln!("JP Z, a16      | JP {:#06x}", nn);
                    self.pc = nn;
                    cycles = 16;
                } else {
                    cpulogln!("JP Z, a16      | <no jump>");
                    cycles = 12
                }
            }
            0xCB => {
                let cb_opcode = self.load_byte(memory);
                cpulog!(
                    "[CPU] PC = {:#06x} | {:#04x}{:02x} | ",
                    self.pc.wrapping_sub(1),
                    opcode,
                    cb_opcode,
                );
                let i = cb_opcode & 0b0111;
                match cb_opcode {
                    0x00..=0x07 => {
                        cpulogln!("RLC {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), self.rlc_r8(nn));
                        } else {
                            *self.get_mut_reg(i) = self.rlc_r8(nn);
                        }
                    }
                    0x08..=0x0F => {
                        cpulogln!("RRC {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), self.rrc_r8(nn));
                        } else {
                            *self.get_mut_reg(i) = self.rrc_r8(nn);
                        }
                    }
                    0x10..=0x17 => {
                        cpulogln!("RL {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), self.rl_r8(nn));
                        } else {
                            *self.get_mut_reg(i) = self.rl_r8(nn);
                        }
                    }
                    0x18..=0x1F => {
                        cpulogln!("RR {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), self.rr_r8(nn));
                        } else {
                            *self.get_mut_reg(i) = self.rr_r8(nn);
                        }
                    }
                    0x20..=0x27 => {
                        cpulogln!("SLA {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), self.sla_r8(nn));
                        } else {
                            *self.get_mut_reg(i) = self.sla_r8(nn);
                        }
                    }
                    0x28..=0x2F => {
                        cpulogln!("SRA {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), self.sra_r8(nn));
                        } else {
                            *self.get_mut_reg(i) = self.sra_r8(nn);
                        }
                    }
                    0x30..=0x37 => {
                        cpulogln!("SWAP {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), self.swap_r8(nn));
                        } else {
                            *self.get_mut_reg(i) = self.swap_r8(nn);
                        }
                    }
                    0x38..=0x3F => {
                        cpulogln!("SRL {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), self.srl_r8(nn));
                        } else {
                            *self.get_mut_reg(i) = self.srl_r8(nn);
                        }
                    }
                    0x40..=0x7F => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        cpulogln!("BIT {bit}, {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        self.bit_r8(bit, nn);
                    }
                    0x80..=0xBF => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        cpulogln!("RES {bit}, {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), res_r8(bit, nn));
                        } else {
                            *self.get_mut_reg(i) = res_r8(bit, nn);
                        }
                    }
                    0xC0..=0xFF => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        cpulogln!("SET {bit}, {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            memory.write_byte(self.get_hl(), set_r8(bit, nn));
                        } else {
                            *self.get_mut_reg(i) = set_r8(bit, nn);
                        }
                    }
                }
                cycles = if i == 6 { 16 } else { 8 }
            }
            0xCC => {
                let nn = self.load_word(memory);
                if self.get_zero_flag() {
                    cpulogln!("CALL Z, a16    | CALL {:#06x}", nn);
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    cycles = 24;
                } else {
                    cpulogln!("CALL Z, a16    | <no call>");
                    cycles = 12;
                }
            }
            0xCD => {
                // CALL nn: Call function
                // Unconditional function call to the absolute address specified by the 16-bit operand nn.
                let nn = self.load_word(memory);
                cpulogln!("CALL a16       | CALL {:#06x}", nn);
                self.push_word(memory, self.pc);
                self.pc = nn;
                cycles = 24;
            }
            0xCE => {
                let nn = self.load_byte(memory);
                cpulogln!("ADC A, d8      | ADC A, {:#04x}", nn);
                self.a = self.adc_r8(nn);
                cycles = 8;
            }
            0xCF => {
                cpulogln!("RST 08H");
                self.push_word(memory, self.pc);
                self.pc = 0x0008;
                cycles = 16;
            }
            0xD0 => {
                if !self.get_carry_flag() {
                    self.pc = self.pop_word(memory);
                    cpulogln!("RET NC         | RET {:#06x}", self.pc);
                    cycles = 20;
                } else {
                    cpulogln!("RET NC         | <no return>");
                    cycles = 8;
                }
            }
            0xD1 => {
                let nn = self.pop_word(memory);
                cpulogln!("POP DE         | DE = {:#06x}", nn);
                self.set_de(nn);
                cycles = 12;
            }
            0xD2 => {
                let nn = self.load_word(memory);
                if !self.get_carry_flag() {
                    cpulogln!("JP NC, a16     | JP {:#06x}", nn);
                    self.pc = nn;
                    cycles = 16;
                } else {
                    cpulogln!("JP NC, a16     | <no jump>");
                    cycles = 12
                }
            }
            0xD4 => {
                let nn = self.load_word(memory);
                if !self.get_carry_flag() {
                    cpulogln!("CALL NC, a16   | CALL {:#06x}", nn);
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    cycles = 24;
                } else {
                    cpulogln!("CALL NC, a16   | <no call>");
                    cycles = 12;
                }
            }
            0xD5 => {
                cpulogln!("PUSH DE        | PUSH {:#06x}", self.get_de());
                self.push_word(memory, self.get_de());
                cycles = 16;
            }
            0xD6 => {
                let nn = self.load_byte(memory);
                cpulogln!("SUB A, d8      | SUB A, {:#04x}", nn);
                self.a = self.sub_r8(nn);
                cycles = 8;
            }
            0xD7 => {
                cpulogln!("RST 10H");
                self.push_word(memory, self.pc);
                self.pc = 0x0010;
                cycles = 16;
            }
            0xD8 => {
                if self.get_carry_flag() {
                    self.pc = self.pop_word(memory);
                    cpulogln!("RET C          | RET {:#06x}", self.pc);
                    cycles = 20;
                } else {
                    cpulogln!("RET NC         | <no return>");
                    cycles = 8;
                }
            }
            0xD9 => {
                self.pc = self.pop_word(memory);
                cpulogln!("RETI           | RETI {:#06x}", self.pc);
                self.ime = true;
                cycles = 16;
            }
            0xDA => {
                let nn = self.load_word(memory);
                if self.get_carry_flag() {
                    cpulogln!("JP C, a16      | JP {:#06x}", nn);
                    self.pc = nn;
                    cycles = 16;
                } else {
                    cpulogln!("JP C, a16      | <no jump>");
                    cycles = 12
                }
            }
            0xDC => {
                let nn = self.load_word(memory);
                if self.get_carry_flag() {
                    cpulogln!("CALL C, a16    | CALL {:#06x}", nn);
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    cycles = 24;
                } else {
                    cpulogln!("CALL C, a16    | <no call>");
                    cycles = 12;
                }
            }
            0xDE => {
                let nn = self.load_byte(memory);
                cpulogln!("SBC A, d8      | SBC A, {:#04x}", nn);
                self.a = self.sbc_r8(nn);
                cycles = 8;
            }
            0xDF => {
                cpulogln!("RST 18H");
                self.push_word(memory, self.pc);
                self.pc = 0x0018;
                cycles = 16;
            }
            0xE0 => {
                // LD (n), A: Load from accumulator (direct 0xFF00+n)
                // Load to the address specified by the 8-bit immediate data n, data from the 8-bit A register. The
                // full 16-bit absolute address is obtained by setting the most significant byte to 0xFF and the
                // least significant byte to the value of n, so the possible range is 0xFF00-0xFFFF.
                let nn = self.load_byte(memory);
                cpulogln!("LD (a8), A     | LD ({:#04x}) {:#04x}", nn, self.a);
                let addr = 0xFF00 | nn as u16;
                memory.write_byte(addr, self.a);
                cycles = 12;
            }
            0xE1 => {
                let nn = self.pop_word(memory);
                cpulogln!("POP HL         | HL = {:#06x}", nn);
                self.set_hl(nn);
                cycles = 12;
            }
            0xE2 => {
                cpulogln!("LD (C), A      | LD ({:#04x}) {:#04x}", self.c, self.a);
                let addr = 0xFF00 | (self.c as u16);
                memory.write_byte(addr, self.a);
                cycles = 8;
            }
            0xE5 => {
                cpulogln!("PUSH HL        | PUSH {:#06x}", self.get_hl());
                self.push_word(memory, self.get_hl());
                cycles = 16;
            }
            0xE6 => {
                let nn = self.load_byte(memory);
                cpulogln!("AND d8         | AND {:#04x}", nn);
                self.a &= nn;
                self.set_zero_flag(self.a == 0);
                self.set_subtraction_flag(false);
                self.set_half_carry_flag(true);
                self.set_carry_flag(false);
                cycles = 8;
            }
            0xE7 => {
                cpulogln!("RST 20H");
                self.push_word(memory, self.pc);
                self.pc = 0x0020;
                cycles = 16;
            }
            0xE8 => {
                let nn = self.load_byte(memory) as i8;
                cpulogln!("ADD SP, r8     | ADD SP, {:#04x}", nn);
                self.set_half_carry_flag((self.sp & 0x0F) + (nn as u16 & 0x0F) > 0x0F);
                self.set_carry_flag((self.sp & 0xFF) + (nn as u16 & 0xFF) > 0xFF);
                self.sp = self.sp.wrapping_add(nn as u16);
                self.set_zero_flag(false);
                self.set_subtraction_flag(false);
                cycles = 16;
            }
            0xE9 => {
                cpulogln!("JP HL          | JP {:#06x}", self.get_hl());
                self.pc = self.get_hl();
                cycles = 4;
            }
            0xEA => {
                let nn = self.load_word(memory);
                cpulogln!("LD (a16), A    | LD ({:#06x}), {:#04x}", nn, self.a);
                memory.write_byte(nn, self.a);
                cycles = 16;
            }
            0xEE => {
                let nn = self.load_byte(memory);
                cpulogln!("XOR d8         | XOR {:#04x}", nn);
                self.a = self.xor(nn);
                cycles = 8;
            }
            0xEF => {
                cpulogln!("RST 28H");
                self.push_word(memory, self.pc);
                self.pc = 0x0028;
                cycles = 16;
            }
            0xF0 => {
                let nn = self.load_byte(memory);
                let addr = 0xFF00 | nn as u16;
                self.a = memory.read_byte(addr);
                cpulogln!("LD A, (a8)     | LD A, {:#04x}", self.a);
                cycles = 12;
            }
            0xF1 => {
                let nn = self.pop_word(memory);
                cpulogln!("POP AF         | AF = {:#06x}", nn);
                self.set_af(nn);
                cycles = 12;
            }
            0xF2 => {
                let addr = 0xFF00 | (self.c as u16);
                self.a = memory.read_byte(addr);
                cpulogln!("LD A, (C)      | LD A, {:#04x}", self.a);
                cycles = 8;
            }
            0xF3 => {
                cpulogln!("DI");
                self.ime = false;
                cycles = 4;
            }
            0xF5 => {
                cpulogln!("PUSH AF        | PUSH {:#06x}", self.get_af());
                self.push_word(memory, self.get_af());
                cycles = 16;
            }
            0xF6 => {
                let nn = self.load_byte(memory);
                cpulogln!("OR d8          | OR {:#04x}", nn);
                self.a = self.or_r8(nn);
                cycles = 8;
            }
            0xF7 => {
                cpulogln!("RST 30H");
                self.push_word(memory, self.pc);
                self.pc = 0x0030;
                cycles = 16;
            }
            0xF8 => {
                let nn = self.load_byte(memory) as i8;
                self.set_half_carry_flag((self.sp & 0x0F) + (nn as u16 & 0x0F) > 0x0F);
                self.set_carry_flag((self.sp & 0xFF) + (nn as u16 & 0xFF) > 0xFF);
                let hl = self.sp.wrapping_add(nn as u16);
                cpulogln!("LD HL, SP+r8   | LD HL, {:#06x}", hl);
                self.set_hl(hl);
                self.set_zero_flag(false);
                self.set_subtraction_flag(false);
                cycles = 16;
            }
            0xF9 => {
                self.sp = self.get_hl();
                cpulogln!("LD SP, HL      | LD SP, {:#06x}", self.sp);
                cycles = 8;
            }
            0xFA => {
                let nn = self.load_word(memory);
                self.a = memory.read_byte(nn);
                cpulogln!("LD A, (a16)    | LD A, {:#04x}", self.a);
                cycles = 16;
            }
            0xFB => {
                cpulogln!("EI");
                self.should_enable_ime = true;
                cycles = 4;
            }
            0xFE => {
                let nn = self.load_byte(memory);
                cpulogln!("CP d8          | CP {:#04x}", nn);
                self.cp_r8(nn);
                cycles = 8;
            }
            0xFF => {
                cpulogln!("RST 38H");
                self.push_word(memory, self.pc);
                self.pc = 0x0038;
                cycles = 16;
            }
            _ => {
                cpulogln!(" ???");
                panic!("Opcode not implemented {:#04x}", opcode);
            }
        }
        cycles
    }

    fn handle_interrupts(&mut self, memory: &mut Memory) {
        // FFFF - IE: Interrupt enable
        let interrupt_enable = memory.read_byte(0xFFFF) & 0b0001_1111;
        // FF0F — IF: Interrupt flag
        let interrupt_flag = memory.read_byte(0xFF0F) & 0b0001_1111;

        if self.halt && interrupt_enable & interrupt_flag != 0 {
            self.halt = false;
        }
        if self.ime {
            if (interrupt_flag & interrupt_enable & 0b0000_0001) != 0 {
                // V-Blank interrupt requested
                memory.write_byte(0xFF0F, interrupt_flag & !0b0000_0001);
                self.ime = false;
                self.push_word(memory, self.pc);
                self.pc = 0x0040;
            } else if (interrupt_flag & interrupt_enable & 0b0000_0010) != 0 {
                // LCD STAT interrupt requested
                memory.write_byte(0xFF0F, interrupt_flag & !0b0000_0010);
                self.ime = false;
                self.push_word(memory, self.pc);
                self.pc = 0x0048;
            } else if (interrupt_flag & interrupt_enable & 0b0000_0100) != 0 {
                // Timer interrupt requested
                memory.write_byte(0xFF0F, interrupt_flag & !0b0000_0100);
                self.ime = false;
                self.push_word(memory, self.pc);
                self.pc = 0x0050;
            } else if (interrupt_flag & interrupt_enable & 0b0000_1000) != 0 {
                // Serial interrupt requested
                memory.write_byte(0xFF0F, interrupt_flag & !0b0000_1000);
                self.ime = false;
                self.push_word(memory, self.pc);
                self.pc = 0x0058;
            } else if (interrupt_flag & interrupt_enable & 0b0001_0000) != 0 {
                // Joypad interrupt requested
                memory.write_byte(0xFF0F, interrupt_flag & !0b0001_0000);
                self.ime = false;
                self.push_word(memory, self.pc);
                self.pc = 0x0060;
            }
        }
    }

    fn update_timer(&mut self, memory: &mut Memory, cycles: u8) {
        // FF04 - DIV: Divider register
        let div = memory.read_byte(0xFF04);
        memory.write_byte(0xFF04, div.wrapping_add(cycles * 4));
        // FF05 - TIMA: Timer counter
        let tima = memory.read_byte(0xFF05);
        // FF06 - TMA: Timer modulo
        let tma = memory.read_byte(0xFF06);
        // FF07 - TAC: Timer control
        let tac = memory.read_byte(0xFF07);

        let increment_every: u16 = if tac & 0b0000_0011 == 0b0000_0000 {
            // Increment every 256 machine-cycles
            256
        } else if tac & 0b0000_0011 == 0b0000_0001 {
            // Increment every 4 machine-cycles
            4
        } else if tac & 0b0000_0011 == 0b0000_0010 {
            // Increment every 16 machine-cycles
            16
        } else {
            // Increment every 64 machine-cycles
            64
        };

        while self.total_cycles >= increment_every {
            if tac & 0b0000_0100 != 0 {
                // If increment is enabled
                if tima == 0xFF {
                    // Overflow - reset TIMA to TMA
                    memory.write_byte(0xFF05, tma);
                    // Request Timer interrupt
                    let mut nn = memory.read_byte(0xFF0F);
                    nn |= 0b0000_0100; // Bit 2
                    memory.write_byte(0xFF0F, nn);
                } else {
                    memory.write_byte(0xFF05, tima + 1);
                }
            }
            self.total_cycles -= increment_every;
        }
    }

    // Main function

    pub fn tick(&mut self, memory: &mut Memory) -> u8 {
        self.handle_interrupts(memory);

        let cycles = if self.halt {
            4
        } else {
            self.execute_instruction(memory)
        };
        self.total_cycles += cycles as u16;

        self.update_timer(memory, cycles);

        if self.should_enable_ime {
            self.ime = true;
            self.should_enable_ime = false;
        }

        cycles
    }
}
