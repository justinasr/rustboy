use super::memory::Memory;
use super::utils::*;

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
            pc: 0,
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

    fn dec_r8(&mut self, r: u8) -> u8 {
        // If lower nibble is 0, it will carry from upper nibble
        self.set_half_carry_flag((r & 0x0F) == 0);
        let rr = r.wrapping_sub(1);
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(true);
        rr
    }

    fn inc_r8(&mut self, r: u8) -> u8 {
        // If lower nibble is 0b1111, adding 1 will move to upper nibble
        self.set_half_carry_flag((r & 0x0F) == 0x0F);
        let rr = r.wrapping_add(1);
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(false);
        rr
    }

    /** XOR given value with A register. */
    fn xor(&mut self, r: u8) -> u8 {
        let a = self.a ^ r;
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        self.set_carry_flag(false);
        a
    }

    /** Rotate bits in given register left, through the carry flag. */
    fn rl_r8(&mut self, r: u8) -> u8 {
        let c = if self.get_carry_flag() { 1 } else { 0 };
        self.set_carry_flag(r & 0b1000_0000 != 0);
        let mut rr = r << 1;
        rr |= c;
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        rr
    }

    fn rr_r8(&mut self, r: u8) -> u8 {
        let c = if self.get_carry_flag() { 1 } else { 0 };
        self.set_carry_flag(r & 0b0000_0001 != 0);
        let mut rr = r >> 1;
        rr |= c << 7;
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        rr
    }

    /** Shift left into carry, LSB is set to 0. */
    fn sla_r8(&mut self, r: u8) -> u8 {
        self.set_carry_flag(r & 0b1000_0000 != 0);
        let rr = (r << 1) & 0b1111_1110;
        self.set_zero_flag(rr == 0);
        self.set_half_carry_flag(false);
        self.set_subtraction_flag(false);
        rr
    }

    /** Shift right into carry, MSB does not change */
    fn sra_r8(&mut self, r: u8) -> u8 {
        self.set_carry_flag(r & 0b0000_0001 != 0);
        let rr = (r >> 1) | (r & 0b1000_0000);
        self.set_zero_flag(rr == 0);
        self.set_half_carry_flag(false);
        self.set_subtraction_flag(false);
        rr
    }

    /** Test bit b in given value, set the zero flag accordingly. */
    fn bit_r8(&mut self, b: u8, r: u8) {
        self.set_zero_flag(r & (1 << b) == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(true);
    }

    fn sub_r8(&mut self, r: u8) -> u8 {
        let mut a = self.a;
        self.set_half_carry_flag((a & 0x0F) < (r & 0x0F));
        self.set_carry_flag(a < r);
        a = a.wrapping_sub(r);
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(true);
        a
    }

    fn sbc_r8(&mut self, r: u8) -> u8 {
        let mut a = self.a;
        let c = if self.get_carry_flag() { 1 } else { 0 };
        self.set_half_carry_flag((a & 0x0F) < ((r & 0x0F) + c));
        self.set_carry_flag((a as u16) < (r as u16) + (c as u16));
        a = a.wrapping_sub(r).wrapping_sub(c);
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(true);
        a
    }

    fn add_r8(&mut self, r: u8) -> u8 {
        let mut a = self.a;
        self.set_half_carry_flag((a & 0x0F) + (r & 0x0F) > 0x0F);
        self.set_carry_flag((a as u16) + (r as u16) > 0xFF);
        a = a.wrapping_add(r);
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        a
    }

    fn add_r16(&mut self, rr: u16) -> u16 {
        let mut hl = self.get_hl();
        self.set_half_carry_flag((hl & 0x0FFF) + (rr & 0x0FFF) > 0x0FFF);
        self.set_carry_flag((hl as u32) + (rr as u32) > 0xFFFF);
        hl = hl.wrapping_add(rr);
        self.set_subtraction_flag(false);
        hl
    }

    fn adc_r8(&mut self, r: u8) -> u8 {
        let mut a = self.a;
        let c = if self.get_carry_flag() { 1 } else { 0 };
        self.set_half_carry_flag((a & 0x0F) + (r & 0x0F) + c > 0x0F);
        self.set_carry_flag((a as u16) + (r as u16) + (c as u16) > 0xFF);
        a = a.wrapping_add(r).wrapping_add(c);
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        a
    }

    fn cp_r8(&mut self, r: u8) {
        let a = self.a;
        self.set_zero_flag(a == r);
        self.set_subtraction_flag(true);
        self.set_half_carry_flag((a & 0x0F) < (r & 0x0F));
        self.set_carry_flag(a < r);
    }

    fn and_r8(&mut self, r: u8) -> u8 {
        let a = self.a & r;
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(true);
        self.set_carry_flag(false);
        a
    }

    fn or_r8(&mut self, r: u8) -> u8 {
        let a = self.a | r;
        self.set_zero_flag(a == 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        self.set_carry_flag(false);
        a
    }

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
        // print!("[CPU] PC = {:#06x} | Opcode = {:#04x} |", self.pc, opcode);
        let cycles: u8;

        match opcode {
            0x00 => {
                // println!(" NOP");
                cycles = 4;
            }
            0x01 => {
                // println!(" LD BC, d16");
                let nn = self.load_word(memory);
                self.set_bc(nn);
                cycles = 12;
            }
            0x02 => {
                // println!(" LD (BC), A");
                let addr = self.get_bc();
                memory.write_byte(addr, self.a);
                cycles = 8;
            }
            0x03 => {
                // println!(" INC BC");
                (self.b, self.c) = inc_r16(self.b, self.c);
                cycles = 8;
            }
            0x04 => {
                // println!(" INC B");
                self.b = self.inc_r8(self.b);
                cycles = 4;
            }
            0x05 => {
                // println!(" DEC B");
                self.b = self.dec_r8(self.b);
                cycles = 4;
            }
            0x06 => {
                // print!(" LD B, d8");
                self.b = self.load_byte(memory);
                // println!("    | B = {:#04x}", self.b);
                cycles = 8;
            }
            0x07 => {
                // println!(" RLCA");
                self.a = self.rlc_r8(self.a);
                self.set_zero_flag(false);
                cycles = 4;
            }
            0x08 => {
                // println!(" LD (a16), SP");
                let addr = self.load_word(memory);
                memory.write_word(addr, self.sp);
                cycles = 20;
            }
            0x09 => {
                // println!(" ADD HL, BC");
                let hl = self.add_r16(self.get_bc());
                self.set_hl(hl);
                cycles = 8;
            }
            0x0A => {
                // println!(" LD A, (BC)");
                let addr = self.get_bc();
                self.a = memory.read_byte(addr);
                cycles = 8;
            }
            0x0B => {
                // println!(" DEC BC");
                (self.b, self.c) = dec_r16(self.b, self.c);
                cycles = 8;
            }
            0x0C => {
                // println!(" INC C");
                self.c = self.inc_r8(self.c);
                cycles = 4;
            }
            0x0D => {
                // println!(" DEC C");
                self.c = self.dec_r8(self.c);
                cycles = 4;
            }
            0x0E => {
                // println!(" LD C, d8");
                self.c = self.load_byte(memory);
                cycles = 8;
            }
            0x0F => {
                // println!(" RRCA");
                self.a = self.rrc_r8(self.a);
                self.set_zero_flag(false);
                cycles = 4;
            }
            0x11 => {
                // println!(" LD DE, d16");
                let nn = self.load_word(memory);
                self.set_de(nn);
                cycles = 12;
            }
            0x12 => {
                // println!(" LD (DE), A");
                let addr = self.get_de();
                memory.write_byte(addr, self.a);
                cycles = 8;
            }
            0x13 => {
                // println!(" INC DE");
                (self.d, self.e) = inc_r16(self.d, self.e);
                cycles = 8;
            }
            0x14 => {
                // println!(" INC D");
                self.d = self.inc_r8(self.d);
                cycles = 4;
            }
            0x15 => {
                // println!(" DEC D");
                self.d = self.dec_r8(self.d);
                cycles = 4;
            }
            0x16 => {
                // println!(" LD D, d8");
                self.d = self.load_byte(memory);
                cycles = 8;
            }
            0x17 => {
                // println!(" RLA");
                self.a = self.rl_r8(self.a);
                self.set_zero_flag(false);
                cycles = 4;
            }
            0x18 => {
                // print!(" JR r8");
                // Relative jump
                let nn = self.load_byte(memory) as i8 as i16;
                // println!("     | JR {nn}");
                self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                cycles = 12;
            }
            0x19 => {
                // println!(" ADD HL, DE");
                let hl = self.add_r16(self.get_de());
                self.set_hl(hl);
                cycles = 8;
            }
            0x1A => {
                // println!(" LD A, (DE)");
                let addr = self.get_de();
                self.a = memory.read_byte(addr);
                cycles = 8;
            }
            0x1B => {
                // println!(" DEC DE");
                (self.d, self.e) = dec_r16(self.d, self.e);
                cycles = 8;
            }
            0x1C => {
                // println!(" INC E");
                self.e = self.inc_r8(self.e);
                cycles = 4;
            }
            0x1D => {
                // println!(" DEC E");
                self.e = self.dec_r8(self.e);
                cycles = 4;
            }
            0x1E => {
                // println!(" LD E, d8");
                self.e = self.load_byte(memory);
                cycles = 8;
            }
            0x1F => {
                // println!(" RRA");
                self.a = self.rr_r8(self.a);
                self.set_zero_flag(false);
                cycles = 4;
            }
            0x20 => {
                // println!(" JR NZ, r8");
                let nn = self.load_byte(memory) as i8 as i16;
                if !self.get_zero_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                    cycles = 12;
                } else {
                    cycles = 8;
                }
            }
            0x21 => {
                // println!(" LD HL, d16");
                let nn = self.load_word(memory);
                self.set_hl(nn);
                cycles = 12;
            }
            0x22 => {
                // println!(" LD (HL+), A");
                let addr = self.get_hl();
                memory.write_byte(addr, self.a);
                (self.h, self.l) = inc_r16(self.h, self.l);
                cycles = 8;
            }
            0x23 => {
                // println!(" INC HL");
                (self.h, self.l) = inc_r16(self.h, self.l);
                cycles = 8;
            }
            0x24 => {
                // println!(" INC H");
                self.h = self.inc_r8(self.h);
                cycles = 4;
            }
            0x25 => {
                // println!(" DEC H");
                self.h = self.dec_r8(self.h);
                cycles = 4;
            }
            0x26 => {
                // println!(" LD H, d8");
                self.h = self.load_byte(memory);
                cycles = 8;
            }
            0x27 => {
                // println!(" DAA");
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
                // println!(" JR Z, r8");
                let nn = self.load_byte(memory) as i8 as i16;
                if self.get_zero_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                    cycles = 12;
                } else {
                    cycles = 8;
                }
            }
            0x29 => {
                // println!(" ADD HL, HL");
                let hl = self.add_r16(self.get_hl());
                self.set_hl(hl);
                cycles = 8;
            }
            0x2A => {
                // println!(" LD A, (HL+)");
                let addr = self.get_hl();
                self.a = memory.read_byte(addr);
                (self.h, self.l) = inc_r16(self.h, self.l);
                cycles = 8;
            }
            0x2B => {
                // println!(" DEC HL");
                (self.h, self.l) = dec_r16(self.h, self.l);
                cycles = 8;
            }
            0x2C => {
                // println!(" INC L");
                self.l = self.inc_r8(self.l);
                cycles = 4;
            }
            0x2D => {
                // println!(" DEC L");
                self.l = self.dec_r8(self.l);
                cycles = 4;
            }
            0x2E => {
                // println!(" LD L, d8");
                self.l = self.load_byte(memory);
                cycles = 8;
            }
            0x2F => {
                // println!(" CPL");
                self.a = !self.a;
                self.set_subtraction_flag(true);
                self.set_half_carry_flag(true);
                cycles = 4;
            }
            0x30 => {
                // println!(" JR NC, r8");
                let nn = self.load_byte(memory) as i8 as i16;
                if !self.get_carry_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                    cycles = 12;
                } else {
                    cycles = 8;
                }
            }
            0x31 => {
                // println!(" LD SP, d16");
                self.sp = self.load_word(memory);
                cycles = 12;
            }
            0x32 => {
                // println!(" LD (HL+), A");
                let addr = self.get_hl();
                memory.write_byte(addr, self.a);
                (self.h, self.l) = dec_r16(self.h, self.l);
                cycles = 8;
            }
            0x33 => {
                // println!(" INC SP");
                self.sp = self.sp.wrapping_add(1);
                cycles = 8;
            }
            0x34 => {
                // println!(" INC (HL)");
                let addr = self.get_hl();
                let nn = self.inc_r8(memory.read_byte(addr));
                memory.write_byte(addr, nn);
                cycles = 4;
            }
            0x35 => {
                // println!(" DEC (HL)");
                let addr = self.get_hl();
                let nn = self.dec_r8(memory.read_byte(addr));
                memory.write_byte(addr, nn);
                cycles = 4;
            }
            0x36 => {
                // println!(" LD (HL), d8");
                let nn = self.load_byte(memory);
                let addr = self.get_hl();
                memory.write_byte(addr, nn);
                cycles = 12;
            }
            0x37 => {
                // println!(" SCF");
                self.set_subtraction_flag(false);
                self.set_half_carry_flag(false);
                self.set_carry_flag(true);
                cycles = 4;
            }
            0x38 => {
                // println!(" JR C, r8");
                let nn = self.load_byte(memory) as i8 as i16;
                if self.get_carry_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                    cycles = 12;
                } else {
                    cycles = 8;
                }
            }
            0x39 => {
                // println!(" ADD HL, SP");
                let hl = self.add_r16(self.sp);
                self.set_hl(hl);
                cycles = 8;
            }
            0x3A => {
                // println!(" LD A, (HL-)");
                let addr = self.get_hl();
                self.a = memory.read_byte(addr);
                (self.h, self.l) = dec_r16(self.h, self.l);
                cycles = 8;
            }
            0x3B => {
                // println!(" DEC SP");
                self.sp = self.sp.wrapping_sub(1);
                cycles = 8;
            }
            0x3C => {
                // println!(" INC A");
                self.a = self.inc_r8(self.a);
                cycles = 4;
            }
            0x3D => {
                // println!(" DEC A");
                self.a = self.dec_r8(self.a);
                cycles = 4;
            }
            0x3E => {
                // println!(" LD A, d8");
                self.a = self.load_byte(memory);
                cycles = 8;
            }
            0x3F => {
                // println!(" CCF");
                self.set_subtraction_flag(false);
                self.set_half_carry_flag(false);
                self.set_carry_flag(!self.get_carry_flag());
                cycles = 4;
            }
            0x40..=0x75 | 0x77..=0x7F => {
                let lhs_i = (opcode >> 3) & 0b0111; // Upper nibble divided by 8
                let rhs_i = opcode & 0b0111; // Lower nibble
                // println!(" LD {}, {}",self.get_reg_name(lhs_i),self.get_reg_name(rhs_i));
                if lhs_i == 6 {
                    *memory.get_cell(self.get_hl()) = self.get_reg(memory, rhs_i);
                } else {
                    *self.get_mut_reg(lhs_i) = self.get_reg(memory, rhs_i);
                }
                cycles = if lhs_i == 6 || rhs_i == 6 { 8 } else { 4 };
            }
            0x76 => {
                // println!(" HALT");
                self.halt = true;
                cycles = 4;
            }
            0x80..=0x87 => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" ADD A, {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.add_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0x88..=0x8F => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" ADC A, {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.adc_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0x90..=0x97 => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" SUB {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.sub_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0x98..=0x9F => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" SBC A, {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.sbc_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xA0..=0xA7 => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" AND {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.and_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xA8..=0xAF => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" XOR {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.xor(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xB0..=0xB7 => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" OR {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.or_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xB8..=0xBF => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" CP {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.cp_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0xC0 => {
                // println!(" RET NZ");
                if !self.get_zero_flag() {
                    self.pc = self.pop_word(memory);
                    cycles = 20;
                } else {
                    cycles = 8;
                }
            }
            0xC1 => {
                // println!(" POP BC");
                let nn = self.pop_word(memory);
                self.set_bc(nn);
                cycles = 12;
            }
            0xC2 => {
                // println!(" JP NZ, a16");
                let nn = self.load_word(memory);
                if !self.get_zero_flag() {
                    self.pc = nn;
                    cycles = 16;
                } else {
                    cycles = 12
                }
            }
            0xC3 => {
                // println!(" JP a16");
                self.pc = self.load_word(memory);
                cycles = 16;
            }
            0xC4 => {
                // println!(" CALL NZ, a16");
                let nn = self.load_word(memory);
                if !self.get_zero_flag() {
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    cycles = 24;
                } else {
                    cycles = 12;
                }
            }
            0xC5 => {
                // print!(" PUSH BC");
                self.push_word(memory, self.get_bc());
                // println!("     | PUSH({:#06x})", self.get_bc());
                cycles = 16;
            }
            0xC6 => {
                // println!(" ADD A, d8");
                let nn = self.load_byte(memory);
                self.a = self.add_r8(nn);
                cycles = 8;
            }
            0xC7 => {
                // println!(" RST 00H");
                self.push_word(memory, self.pc);
                self.pc = 0x0000;
                cycles = 16;
            }
            0xC8 => {
                // println!(" RET Z");
                if self.get_zero_flag() {
                    self.pc = self.pop_word(memory);
                    cycles = 20;
                } else {
                    cycles = 8;
                }
            }
            0xC9 => {
                // println!(" RET");
                self.pc = self.pop_word(memory);
                cycles = 16;
            }
            0xCA => {
                // println!(" JP Z, a16");
                let nn = self.load_word(memory);
                if self.get_zero_flag() {
                    self.pc = nn;
                    cycles = 16;
                } else {
                    cycles = 12
                }
            }
            0xCB => {
                let cb_opcode = self.load_byte(memory);
                // print!(" CB Opcode = {:#04x} |", cb_opcode);
                let i = cb_opcode & 0b0111;
                match cb_opcode {
                    0x00..=0x07 => {
                        // println!(" RLC {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.rlc_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.rlc_r8(nn);
                        }
                    }
                    0x08..=0x0F => {
                        // println!(" RRC {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.rrc_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.rrc_r8(nn);
                        }
                    }
                    0x10..=0x17 => {
                        // println!(" RL {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.rl_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.rl_r8(nn);
                        }
                    }
                    0x18..=0x1F => {
                        // println!(" RR {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.rr_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.rr_r8(nn);
                        }
                    }
                    0x20..=0x27 => {
                        // println!(" SLA {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.sla_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.sla_r8(nn);
                        }
                    }
                    0x28..=0x2F => {
                        // println!(" SRA {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.sra_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.sra_r8(nn);
                        }
                    }
                    0x30..=0x37 => {
                        // println!(" SWAP {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.swap_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.swap_r8(nn);
                        }
                    }
                    0x38..=0x3F => {
                        // println!(" SRL {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.srl_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.srl_r8(nn);
                        }
                    }
                    0x40..=0x7F => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        // println!(" BIT {bit}, {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        self.bit_r8(bit, nn);
                    }
                    0x80..=0xBF => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        // println!(" RES {bit}, {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = res_r8(bit, nn);
                        } else {
                            *self.get_mut_reg(i) = res_r8(bit, nn);
                        }
                    }
                    0xC0..=0xFF => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        // println!(" SET {bit}, {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = set_r8(bit, nn);
                        } else {
                            *self.get_mut_reg(i) = set_r8(bit, nn);
                        }
                    }
                }
                cycles = if i == 6 { 16 } else { 8 }
            }
            0xCC => {
                // println!(" CALL Z, a16");
                let nn = self.load_word(memory);
                if self.get_zero_flag() {
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    cycles = 24;
                } else {
                    cycles = 12;
                }
            }
            0xCD => {
                // println!(" CALL a16");
                // CALL nn: Call function
                // Unconditional function call to the absolute address specified by the 16-bit operand nn.
                let nn = self.load_word(memory);
                self.push_word(memory, self.pc);
                self.pc = nn;
                cycles = 24;
            }
            0xCE => {
                // println!(" ADC A, d8");
                let nn = self.load_byte(memory);
                self.a = self.adc_r8(nn);
                cycles = 8;
            }
            0xCF => {
                // println!(" RST 08H");
                self.push_word(memory, self.pc);
                self.pc = 0x0008;
                cycles = 16;
            }
            0xD0 => {
                // println!(" RET NC");
                if !self.get_carry_flag() {
                    self.pc = self.pop_word(memory);
                    cycles = 20;
                } else {
                    cycles = 8;
                }
            }
            0xD1 => {
                // println!(" POP DE");
                let nn = self.pop_word(memory);
                self.set_de(nn);
                cycles = 12;
            }
            0xD2 => {
                // println!(" JP NC, a16");
                let nn = self.load_word(memory);
                if !self.get_carry_flag() {
                    self.pc = nn;
                    cycles = 16;
                } else {
                    cycles = 12
                }
            }
            0xD4 => {
                // println!(" CALL NC, a16");
                let nn = self.load_word(memory);
                if !self.get_carry_flag() {
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    cycles = 24;
                } else {
                    cycles = 12;
                }
            }
            0xD5 => {
                // println!(" PUSH DE");
                self.push_word(memory, self.get_de());
                cycles = 16;
            }
            0xD6 => {
                // println!(" SUB A, d8");
                let nn = self.load_byte(memory);
                self.a = self.sub_r8(nn);
                cycles = 8;
            }
            0xD7 => {
                // println!(" RST 10H");
                self.push_word(memory, self.pc);
                self.pc = 0x0010;
                cycles = 16;
            }
            0xD8 => {
                // println!(" RET C");
                if self.get_carry_flag() {
                    self.pc = self.pop_word(memory);
                    cycles = 20;
                } else {
                    cycles = 8;
                }
            }
            0xD9 => {
                // println!(" RETI");
                self.pc = self.pop_word(memory);
                self.ime = true;
                cycles = 16;
            }
            0xDA => {
                // println!(" JP C, a16");
                let nn = self.load_word(memory);
                if self.get_carry_flag() {
                    self.pc = nn;
                    cycles = 16;
                } else {
                    cycles = 12
                }
            }
            0xDC => {
                // println!(" CALL C, a16");
                let nn = self.load_word(memory);
                if self.get_carry_flag() {
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    cycles = 24;
                } else {
                    cycles = 12;
                }
            }
            0xDE => {
                // println!(" SBC A, d8");
                let nn = self.load_byte(memory);
                self.a = self.sbc_r8(nn);
                cycles = 8;
            }
            0xDF => {
                // println!(" RST 18H");
                self.push_word(memory, self.pc);
                self.pc = 0x0018;
                cycles = 16;
            }
            0xE0 => {
                // println!(" LD (a8), A");
                // LD (n), A: Load from accumulator (direct 0xFF00+n)
                // Load to the address specified by the 8-bit immediate data n, data from the 8-bit A register. The
                // full 16-bit absolute address is obtained by setting the most significant byte to 0xFF and the
                // least significant byte to the value of n, so the possible range is 0xFF00-0xFFFF.
                let nn = self.load_byte(memory);
                let addr = 0xFF00 | nn as u16;
                memory.write_byte(addr, self.a);
                cycles = 12;
            }
            0xE1 => {
                // println!(" POP HL");
                let nn = self.pop_word(memory);
                self.set_hl(nn);
                cycles = 12;
            }
            0xE2 => {
                // println!(" LD (C), A");
                let addr = 0xFF00 | (self.c as u16);
                memory.write_byte(addr, self.a);
                cycles = 8;
            }
            0xE5 => {
                // println!(" PUSH HL");
                self.push_word(memory, self.get_hl());
                cycles = 16;
            }
            0xE6 => {
                // println!(" AND d8");
                let nn = self.load_byte(memory);
                self.a &= nn;
                self.set_zero_flag(self.a == 0);
                self.set_subtraction_flag(false);
                self.set_half_carry_flag(true);
                self.set_carry_flag(false);
                cycles = 8;
            }
            0xE7 => {
                // println!(" RST 20H");
                self.push_word(memory, self.pc);
                self.pc = 0x0020;
                cycles = 16;
            }
            0xE8 => {
                // println!(" ADD SP, r8");
                let nn = self.load_byte(memory) as i8;
                self.set_half_carry_flag((self.sp & 0x0F) + (nn as u16 & 0x0F) > 0x0F);
                self.set_carry_flag((self.sp & 0xFF) + (nn as u16 & 0xFF) > 0xFF);
                self.sp = self.sp.wrapping_add(nn as u16);
                self.set_zero_flag(false);
                self.set_subtraction_flag(false);
                cycles = 16;
            }
            0xE9 => {
                // println!(" JP (HL)");
                self.pc = self.get_hl();
                cycles = 4;
            }
            0xEA => {
                // println!(" LD (a16), A");
                let nn = self.load_word(memory);
                memory.write_byte(nn, self.a);
                cycles = 16;
            }
            0xEE => {
                // println!(" XOR d8");
                let nn = self.load_byte(memory);
                self.a = self.xor(nn);
                cycles = 8;
            }
            0xEF => {
                // println!(" RST 28H");
                self.push_word(memory, self.pc);
                self.pc = 0x0028;
                cycles = 16;
            }
            0xF0 => {
                // print!(" LD A, (a8)");
                let nn = self.load_byte(memory);
                let addr = 0xFF00 | nn as u16;
                self.a = memory.read_byte(addr);
                // println!(" | LD A, ({:#06x}) => LD A, {:#04x}", addr, self.a);
                cycles = 12;
            }
            0xF1 => {
                // println!(" POP AF");
                let nn = self.pop_word(memory);
                self.set_af(nn);
                cycles = 12;
            }
            0xF2 => {
                // println!(" LD A, (C)");
                let addr = 0xFF00 | (self.c as u16);
                self.a = memory.read_byte(addr);
                cycles = 8;
            }
            0xF3 => {
                // println!(" DI");
                self.ime = false;
                cycles = 4;
            }
            0xF5 => {
                // println!(" PUSH AF");
                self.push_word(memory, self.get_af());
                cycles = 16;
            }
            0xF6 => {
                // println!(" OR d8");
                let nn = self.load_byte(memory);
                self.a = self.or_r8(nn);
                cycles = 8;
            }
            0xF7 => {
                // println!(" RST 30H");
                self.push_word(memory, self.pc);
                self.pc = 0x0030;
                cycles = 16;
            }
            0xF8 => {
                // println!(" LD HL, SP+r8");
                let nn = self.load_byte(memory) as i8;
                self.set_half_carry_flag((self.sp & 0x0F) + (nn as u16 & 0x0F) > 0x0F);
                self.set_carry_flag((self.sp & 0xFF) + (nn as u16 & 0xFF) > 0xFF);
                self.set_hl(self.sp.wrapping_add(nn as u16));
                self.set_zero_flag(false);
                self.set_subtraction_flag(false);
                cycles = 16;
            }
            0xF9 => {
                // println!(" LD SP, HL");
                self.sp = self.get_hl();
                cycles = 8;
            }
            0xFA => {
                // println!(" LD A, (a16)");
                let nn = self.load_word(memory);
                self.a = memory.read_byte(nn);
                cycles = 16;
            }
            0xFB => {
                // println!(" EI");
                self.should_enable_ime = true;
                cycles = 4;
            }
            0xFE => {
                // print!(" CP d8");
                let nn = self.load_byte(memory);
                // println!("      | CP {:#04x}", nn);
                self.cp_r8(nn);
                cycles = 8;
            }
            0xFF => {
                // println!(" RST 38H");
                self.push_word(memory, self.pc);
                self.pc = 0x0038;
                cycles = 16;
            }
            _ => {
                // println!(" ???");
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
        // println!("IME={} IE={:#010b} IF={:#010b} IE&IF={:#010b}", self.ime, interrupt_enable, interrupt_flag, interrupt_enable & interrupt_flag);

        if self.halt && interrupt_enable & interrupt_flag != 0 {
            self.halt = false;
        }
        if self.ime && (interrupt_flag & interrupt_enable & 0b0000_0001) != 0 {
            // V-Blank interrupt requested
            // println!("Handling VBlank interrupt");
            memory.write_byte(0xFF0F, interrupt_flag & !0b0000_0001);
            self.ime = false;
            self.push_word(memory, self.pc);
            self.pc = 0x0040;
        } else if self.ime && (interrupt_flag & interrupt_enable & 0b0000_0010) != 0 {
            // LCD STAT interrupt requested
            // println!("Handling STAT interrupt");
            memory.write_byte(0xFF0F, interrupt_flag & !0b0000_0010);
            self.ime = false;
            self.push_word(memory, self.pc);
            self.pc = 0x0048;
        } else if self.ime && (interrupt_flag & interrupt_enable & 0b0000_0100) != 0 {
            // Timer interrupt requested
            // println!("Handling timer interrupt");
            memory.write_byte(0xFF0F, interrupt_flag & !0b0000_0100);
            self.ime = false;
            self.push_word(memory, self.pc);
            self.pc = 0x0050;
        } else if self.ime && (interrupt_flag & interrupt_enable & 0b0000_1000) != 0 {
            // Serial interrupt requested
            // println!("Handling serial interrupt");
            memory.write_byte(0xFF0F, interrupt_flag & !0b0000_1000);
            self.ime = false;
            self.push_word(memory, self.pc);
            self.pc = 0x0058;
        } else if self.ime && (interrupt_flag & interrupt_enable & 0b0001_0000) != 0 {
            // Joypad interrupt requested
            // println!("Handling joypad interrupt");
            memory.write_byte(0xFF0F, interrupt_flag & !0b0001_0000);
            self.ime = false;
            self.push_word(memory, self.pc);
            self.pc = 0x0060;
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

        let cycles = if !self.halt {
            self.execute_instruction(memory)
        } else {
            4
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
