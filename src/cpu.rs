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

    // Interrupt Master Enable
    ime: bool,
    // Whether IME should be enabled during next tick
    enable_ime: bool,

    // CPU cycles for timer
    cycles: u16,

    // Whether CPU is halted
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
            enable_ime: false,
            cycles: 0,
            halt: false,
        };
    }

    // Flags

    fn set_zero_flag(&mut self, value: bool) {
        self.f = update_r8(7, self.f, value)
    }

    fn set_subtraction_flag(&mut self, value: bool) {
        self.f = update_r8(6, self.f, value)
    }

    fn set_half_carry_flag(&mut self, value: bool) {
        self.f = update_r8(5, self.f, value)
    }

    fn set_carry_flag(&mut self, value: bool) {
        self.f = update_r8(4, self.f, value)
    }

    fn get_zero_flag(&self) -> bool {
        self.f & (1 << 7) != 0
    }

    fn get_subtraction_flag(&self) -> bool {
        self.f & (1 << 6) != 0
    }

    fn get_half_carry_flag(&self) -> bool {
        self.f & (1 << 5) != 0
    }

    fn get_carry_flag(&self) -> bool {
        self.f & (1 << 4) != 0
    }

    // Instruction helpers

    /** Decrement 8-bit value. */
    fn dec_r8(&mut self, r: u8) -> u8 {
        let rr = r.wrapping_sub(1);
        self.set_zero_flag(rr == 0);
        self.set_subtraction_flag(true);
        // If lower nibble is 0, it will carry from upper nibble
        self.set_half_carry_flag((r & 0x0F) == 0);
        rr
    }

    /** Increment 8-bit value. */
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

    /** Rotate left circular, set carry flag to MSB. */
    fn rlc_r8(&mut self, r: u8) -> u8 {
        self.set_carry_flag(r & 0b1000_0000 != 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        let nn = (r << 1) | ((r >> 7) & 0x01);
        self.set_zero_flag(nn == 0);
        nn
    }

    /** Rotate right circular, set carry flag to LSB. */
    fn rrc_r8(&mut self, r: u8) -> u8 {
        self.set_carry_flag(r & 0b0000_0001 != 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        let nn = (r >> 1) | ((r << 7) & 0b1000_0000);
        self.set_zero_flag(nn == 0);
        nn
    }

    /** Shift right logical. MSB is set to 0, carry flag to LSB. */
    fn srl_r8(&mut self, r: u8) -> u8 {
        self.set_carry_flag(r & 0b0000_0001 != 0);
        self.set_subtraction_flag(false);
        self.set_half_carry_flag(false);
        let nn = (r >> 1) | ((r << 7) & 0b0111_1111);
        self.set_zero_flag(nn == 0);
        nn
    }

    // Instruction/operand loading

    /** Load and return byte at PC, increment PC by 1. */
    fn load_byte(&mut self, memory: &Memory) -> u8 {
        let nn = memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        nn
    }

    /** Load and return word at PC, increment PC by 2. */
    fn load_word(&mut self, memory: &Memory) -> u16 {
        let nn = memory.read_word(self.pc);
        self.pc = self.pc.wrapping_add(2);
        nn
    }

    // Stack operations

    /** Decrement SP by 2. Push word to stack at SP. */
    fn push_word(&mut self, memory: &mut Memory, nn: u16) {
        self.sp = self.sp.wrapping_sub(2);
        memory.write_word(self.sp, nn);
    }

    /** Increment SP by 2. Pop and return word from stack at SP. */
    fn pop_word(&mut self, memory: &Memory) -> u16 {
        let nn = memory.read_word(self.sp);
        self.sp = self.sp.wrapping_add(2);
        nn
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

    /** Main function that reads the next instruction and executes it.
     * Return number of cycles taken by the instruction.*/
    fn execute_instruction(&mut self, memory: &mut Memory) -> u8 {
        let opcode = self.load_byte(&memory);
        match opcode {
            0x00 => {
                // NOP
                4
            }
            0x01 => {
                // LD BC, d16
                let nn = self.load_word(memory);
                self.set_bc(nn);
                12
            }
            0x02 => {
                // LD (BC), A
                let addr = self.get_bc();
                memory.write_byte(addr, self.a);
                8
            }
            0x03 => {
                // INC BC
                (self.b, self.c) = inc_r16(self.b, self.c);
                8
            }
            0x04 => {
                // INC B
                self.b = self.inc_r8(self.b);
                4
            }
            0x05 => {
                // DEC B
                self.b = self.dec_r8(self.b);
                4
            }
            0x06 => {
                // LD B, d8
                self.b = self.load_byte(memory);
                8
            }
            0x07 => {
                // RLCA
                self.a = self.rlc_r8(self.a);
                self.set_zero_flag(false);
                4
            }
            0x08 => {
                // LD (a16), SP
                let addr = self.load_word(memory);
                memory.write_word(addr, self.sp);
                20
            }
            0x09 => {
                // ADD HL, BC
                let hl = self.add_r16(self.get_bc());
                self.set_hl(hl);
                8
            }
            0x0A => {
                // LD A, (BC)
                let addr = self.get_bc();
                self.a = memory.read_byte(addr);
                8
            }
            0x0B => {
                // DEC BC
                (self.b, self.c) = dec_r16(self.b, self.c);
                8
            }
            0x0C => {
                // INC C
                self.c = self.inc_r8(self.c);
                4
            }
            0x0D => {
                // DEC C
                self.c = self.dec_r8(self.c);
                4
            }
            0x0E => {
                // LD C, d8
                self.c = self.load_byte(memory);
                8
            }
            0x0F => {
                // RRCA
                self.a = self.rrc_r8(self.a);
                self.set_zero_flag(false);
                4
            }
            0x10 => {
                // STOP
                self.halt = true;
                4
            }
            0x11 => {
                // LD DE, d16
                let nn = self.load_word(memory);
                self.set_de(nn);
                12
            }
            0x12 => {
                // LD (DE), A
                let addr = self.get_de();
                memory.write_byte(addr, self.a);
                8
            }
            0x13 => {
                // DEC DE
                (self.d, self.e) = inc_r16(self.d, self.e);
                8
            }
            0x14 => {
                // INC D
                self.d = self.inc_r8(self.d);
                4
            }
            0x15 => {
                // DEC D
                self.d = self.dec_r8(self.d);
                4
            }
            0x16 => {
                // LD D, d8
                self.d = self.load_byte(memory);
                8
            }
            0x17 => {
                // RLA
                self.a = self.rl_r8(self.a);
                self.set_zero_flag(false);
                4
            }
            0x18 => {
                // JR r8
                let nn = self.load_byte(memory);
                self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                12
            }
            0x19 => {
                // ADD HL, DE
                let hl = self.add_r16(self.get_de());
                self.set_hl(hl);
                8
            }
            0x1A => {
                // LD A, (DE)
                self.a = memory.read_byte(self.get_de());
                8
            }
            0x1B => {
                // DEC DE
                (self.d, self.e) = dec_r16(self.d, self.e);
                8
            }
            0x1C => {
                // INC E
                self.e = self.inc_r8(self.e);
                4
            }
            0x1D => {
                // DEC E
                self.e = self.dec_r8(self.e);
                4
            }
            0x1E => {
                // LD E, d8
                self.e = self.load_byte(memory);
                8
            }
            0x1F => {
                // RRA
                self.a = self.rr_r8(self.a);
                self.set_zero_flag(false);
                4
            }
            0x20 => {
                // JR NZ, r8
                let nn = self.load_byte(memory);
                if !self.get_zero_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                    12
                } else {
                    8
                }
            }
            0x21 => {
                // LD HL, d16
                let nn = self.load_word(memory);
                self.set_hl(nn);
                12
            }
            0x22 => {
                // LD (HL+), A
                let addr = self.get_hl();
                memory.write_byte(addr, self.a);
                (self.h, self.l) = inc_r16(self.h, self.l);
                8
            }
            0x23 => {
                // INC HL
                (self.h, self.l) = inc_r16(self.h, self.l);
                8
            }
            0x24 => {
                // INC H
                self.h = self.inc_r8(self.h);
                4
            }
            0x25 => {
                // DEC H
                self.h = self.dec_r8(self.h);
                4
            }
            0x26 => {
                // LD H, d8
                self.h = self.load_byte(memory);
                8
            }
            0x27 => {
                // DAA
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
                4
            }
            0x28 => {
                // JR Z, r8
                let nn = self.load_byte(memory);
                if self.get_zero_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                    12
                } else {
                    8
                }
            }
            0x29 => {
                // ADD HL, HL
                let hl = self.add_r16(self.get_hl());
                self.set_hl(hl);
                8
            }
            0x2A => {
                // LD A, (HL+)
                let addr = self.get_hl();
                self.a = memory.read_byte(addr);
                (self.h, self.l) = inc_r16(self.h, self.l);
                8
            }
            0x2B => {
                // DEC HL
                (self.h, self.l) = dec_r16(self.h, self.l);
                8
            }
            0x2C => {
                // INC L
                self.l = self.inc_r8(self.l);
                4
            }
            0x2D => {
                // DEC L
                self.l = self.dec_r8(self.l);
                4
            }
            0x2E => {
                // LD L, d8
                self.l = self.load_byte(memory);
                8
            }
            0x2F => {
                // CPL
                self.a = !self.a;
                self.set_subtraction_flag(true);
                self.set_half_carry_flag(true);
                4
            }
            0x30 => {
                // JR NC, r8
                let nn = self.load_byte(memory);
                if !self.get_carry_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                    12
                } else {
                    8
                }
            }
            0x31 => {
                // LD SP, d16
                self.sp = self.load_word(memory);
                12
            }
            0x32 => {
                // LD (HL-), A
                let addr = self.get_hl();
                memory.write_byte(addr, self.a);
                (self.h, self.l) = dec_r16(self.h, self.l);
                8
            }
            0x33 => {
                // INC SP
                self.sp = self.sp.wrapping_add(1);
                8
            }
            0x34 => {
                // INC (HL)
                let addr = self.get_hl();
                let nn = self.inc_r8(memory.read_byte(addr));
                memory.write_byte(addr, nn);
                4
            }
            0x35 => {
                // DEC (HL)
                let addr = self.get_hl();
                let nn = self.dec_r8(memory.read_byte(addr));
                memory.write_byte(addr, nn);
                4
            }
            0x36 => {
                // LD (HL), d8
                let nn = self.load_byte(memory);
                let addr = self.get_hl();
                memory.write_byte(addr, nn);
                12
            }
            0x37 => {
                // SCF
                self.set_subtraction_flag(false);
                self.set_half_carry_flag(false);
                self.set_carry_flag(true);
                4
            }
            0x38 => {
                // JR C, r8
                let nn = self.load_byte(memory);
                if self.get_carry_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn as i8 as i16) as u16;
                    12
                } else {
                    8
                }
            }
            0x39 => {
                // ADD HL, SP
                let hl = self.add_r16(self.sp);
                self.set_hl(hl);
                8
            }
            0x3A => {
                // LD A, (HL-)
                let addr = self.get_hl();
                self.a = memory.read_byte(addr);
                (self.h, self.l) = dec_r16(self.h, self.l);
                8
            }
            0x3B => {
                // DEC SP
                self.sp = self.sp.wrapping_sub(1);
                8
            }
            0x3C => {
                // INC A
                self.a = self.inc_r8(self.a);
                4
            }
            0x3D => {
                // DEC A
                self.a = self.dec_r8(self.a);
                4
            }
            0x3E => {
                // LD A, d8
                self.a = self.load_byte(memory);
                8
            }
            0x3F => {
                // CCF
                self.set_subtraction_flag(false);
                self.set_half_carry_flag(false);
                self.set_carry_flag(!self.get_carry_flag());
                4
            }
            0x40..=0x75 | 0x77..=0x7F => {
                // LD X, Y
                // Where X and Y are 8-bit registers
                let lhs_i = (opcode >> 3) & 0b0111; // Upper nibble % 8
                let rhs_i = opcode & 0b0111; // Lower nibble % 8
                if lhs_i == 6 {
                    memory.write_byte(self.get_hl(), self.get_reg(memory, rhs_i));
                } else {
                    *self.get_mut_reg(lhs_i) = self.get_reg(memory, rhs_i);
                }
                if lhs_i == 6 || rhs_i == 6 { 8 } else { 4 }
            }
            0x76 => {
                // HALT
                self.halt = true;
                4
            }
            0x80..=0x87 => {
                // ADD A, X
                // Where X is an 8-bit register
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                self.a = self.add_r8(nn);
                if i == 6 { 8 } else { 4 }
            }
            0x88..=0x8F => {
                // ADC A, X
                // Where X is an 8-bit register
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                self.a = self.adc_r8(nn);
                if i == 6 { 8 } else { 4 }
            }
            0x90..=0x97 => {
                // SUB X
                // Where X is an 8-bit register
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                self.a = self.sub_r8(nn);
                if i == 6 { 8 } else { 4 }
            }
            0x98..=0x9F => {
                // SBC A, X
                // Where X is an 8-bit register
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                self.a = self.sbc_r8(nn);
                if i == 6 { 8 } else { 4 }
            }
            0xA0..=0xA7 => {
                // AND X
                // Where X is an 8-bit register
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                self.a = self.and_r8(nn);
                if i == 6 { 8 } else { 4 }
            }
            0xA8..=0xAF => {
                // XOR X
                // Where X is an 8-bit register
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                self.a = self.xor(nn);
                if i == 6 { 8 } else { 4 }
            }
            0xB0..=0xB7 => {
                // OR X
                // Where X is an 8-bit register
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                self.a = self.or_r8(nn);
                if i == 6 { 8 } else { 4 }
            }
            0xB8..=0xBF => {
                // CP X
                // Where X is an 8-bit register
                let i = opcode & 0b0111; // Lower nibble
                let nn = self.get_reg(memory, i);
                self.cp_r8(nn);
                if i == 6 { 8 } else { 4 }
            }
            0xC0 => {
                // RET NZ
                if !self.get_zero_flag() {
                    self.pc = self.pop_word(memory);
                    20
                } else {
                    8
                }
            }
            0xC1 => {
                // POP BC
                let nn = self.pop_word(memory);
                self.set_bc(nn);
                12
            }
            0xC2 => {
                // JR NZ, a16
                let nn = self.load_word(memory);
                if !self.get_zero_flag() {
                    self.pc = nn;
                    16
                } else {
                    12
                }
            }
            0xC3 => {
                // JP a16
                self.pc = self.load_word(memory);
                16
            }
            0xC4 => {
                // CALL NZ, a16
                let nn = self.load_word(memory);
                if !self.get_zero_flag() {
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    24
                } else {
                    12
                }
            }
            0xC5 => {
                // PUSH BC
                self.push_word(memory, self.get_bc());
                16
            }
            0xC6 => {
                // ADD A, d8
                let nn = self.load_byte(memory);
                self.a = self.add_r8(nn);
                8
            }
            0xC7 => {
                // RST 00H
                self.push_word(memory, self.pc);
                self.pc = 0x0000;
                16
            }
            0xC8 => {
                // RET Z
                if self.get_zero_flag() {
                    self.pc = self.pop_word(memory);
                    20
                } else {
                    8
                }
            }
            0xC9 => {
                // RET
                self.pc = self.pop_word(memory);
                16
            }
            0xCA => {
                // JP Z, a16
                let nn = self.load_word(memory);
                if self.get_zero_flag() {
                    self.pc = nn;
                    16
                } else {
                    12
                }
            }
            0xCB => {
                // Prefix CB
                let cb_opcode = self.load_byte(memory);
                let i = cb_opcode & 0b0111;
                let bit = (cb_opcode >> 3) & 0b0111;
                let nn = self.get_reg(memory, i);
                if 0x40 <= cb_opcode && cb_opcode <= 0x7F {
                    self.bit_r8(bit, nn)
                } else {
                    let res = match cb_opcode {
                        0x00..=0x07 => {
                            // RLC
                            self.rlc_r8(nn)
                        }
                        0x08..=0x0F => {
                            // RRC
                            self.rrc_r8(nn)
                        }
                        0x10..=0x17 => {
                            // RL
                            self.rl_r8(nn)
                        }
                        0x18..=0x1F => {
                            // RR
                            self.rr_r8(nn)
                        }
                        0x20..=0x27 => {
                            // SLA
                            self.sla_r8(nn)
                        }
                        0x28..=0x2F => {
                            // SRA
                            self.sra_r8(nn)
                        }
                        0x30..=0x37 => {
                            // SWAP
                            self.swap_r8(nn)
                        }
                        0x38..=0x3F => {
                            // SRL
                            self.srl_r8(nn)
                        }
                        0x40..=0x7F => {
                            unreachable!();
                        }
                        0x80..=0xBF => {
                            // RES
                            res_r8(bit, nn)
                        }
                        0xC0..=0xFF => {
                            // SET
                            set_r8(bit, nn)
                        }
                    };
                    if i == 6 {
                        memory.write_byte(self.get_hl(), res);
                    } else {
                        *self.get_mut_reg(i) = res;
                    }
                }
                if i == 6 { 16 } else { 8 }
            }
            0xCC => {
                // CALL Z, a16
                let nn = self.load_word(memory);
                if self.get_zero_flag() {
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    24
                } else {
                    12
                }
            }
            0xCD => {
                // CALL a16
                let nn = self.load_word(memory);
                self.push_word(memory, self.pc);
                self.pc = nn;
                24
            }
            0xCE => {
                // ADC A, d8
                let nn = self.load_byte(memory);
                self.a = self.adc_r8(nn);
                8
            }
            0xCF => {
                // RST 08H
                self.push_word(memory, self.pc);
                self.pc = 0x0008;
                16
            }
            0xD0 => {
                // RET NC
                if !self.get_carry_flag() {
                    self.pc = self.pop_word(memory);
                    20
                } else {
                    8
                }
            }
            0xD1 => {
                // POP DE
                let nn = self.pop_word(memory);
                self.set_de(nn);
                12
            }
            0xD2 => {
                // JP NC, a16
                let nn = self.load_word(memory);
                if !self.get_carry_flag() {
                    self.pc = nn;
                    16
                } else {
                    12
                }
            }
            0xD4 => {
                // CALL NC, a16
                let nn = self.load_word(memory);
                if !self.get_carry_flag() {
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    24
                } else {
                    12
                }
            }
            0xD5 => {
                // PUSH DE
                self.push_word(memory, self.get_de());
                16
            }
            0xD6 => {
                // SUB A, d8
                let nn = self.load_byte(memory);
                self.a = self.sub_r8(nn);
                8
            }
            0xD7 => {
                // RST 10H
                self.push_word(memory, self.pc);
                self.pc = 0x0010;
                16
            }
            0xD8 => {
                // RET C
                if self.get_carry_flag() {
                    self.pc = self.pop_word(memory);
                    20
                } else {
                    8
                }
            }
            0xD9 => {
                // RETI
                self.pc = self.pop_word(memory);
                self.ime = true;
                16
            }
            0xDA => {
                // JP C, a16
                let nn = self.load_word(memory);
                if self.get_carry_flag() {
                    self.pc = nn;
                    16
                } else {
                    12
                }
            }
            0xDC => {
                // CALL C, a16
                let nn = self.load_word(memory);
                if self.get_carry_flag() {
                    self.push_word(memory, self.pc);
                    self.pc = nn;
                    24
                } else {
                    12
                }
            }
            0xDE => {
                // SBC A, d8
                let nn = self.load_byte(memory);
                self.a = self.sbc_r8(nn);
                8
            }
            0xDF => {
                // RST 18H
                self.push_word(memory, self.pc);
                self.pc = 0x0018;
                16
            }
            0xE0 => {
                // LD (a8), A
                let nn = self.load_byte(memory);
                let addr = 0xFF00 | nn as u16;
                memory.write_byte(addr, self.a);
                12
            }
            0xE1 => {
                // POP HL
                let nn = self.pop_word(memory);
                self.set_hl(nn);
                12
            }
            0xE2 => {
                // LD (C), A
                let addr = 0xFF00 | (self.c as u16);
                memory.write_byte(addr, self.a);
                8
            }
            0xE5 => {
                // PUSH HL
                self.push_word(memory, self.get_hl());
                16
            }
            0xE6 => {
                // AND d8
                let nn = self.load_byte(memory);
                self.a = self.and_r8(nn);
                8
            }
            0xE7 => {
                // RST 20H
                self.push_word(memory, self.pc);
                self.pc = 0x0020;
                16
            }
            0xE8 => {
                // ADD SP, r8
                let nn = self.load_byte(memory) as i8;
                self.set_half_carry_flag((self.sp & 0x0F) + (nn as u16 & 0x0F) > 0x0F);
                self.set_carry_flag((self.sp & 0xFF) + (nn as u16 & 0xFF) > 0xFF);
                self.sp = self.sp.wrapping_add(nn as u16);
                self.set_zero_flag(false);
                self.set_subtraction_flag(false);
                16
            }
            0xE9 => {
                // JP HL
                self.pc = self.get_hl();
                4
            }
            0xEA => {
                // LD (a16), A
                let nn = self.load_word(memory);
                memory.write_byte(nn, self.a);
                16
            }
            0xEE => {
                // XOR d8 XOR {:#04x}", nn);
                let nn = self.load_byte(memory);
                self.a = self.xor(nn);
                8
            }
            0xEF => {
                // RST 28H
                self.push_word(memory, self.pc);
                self.pc = 0x0028;
                16
            }
            0xF0 => {
                // LD A, (a8)
                let nn = self.load_byte(memory);
                let addr = 0xFF00 | nn as u16;
                self.a = memory.read_byte(addr);
                12
            }
            0xF1 => {
                // POP AF
                let nn = self.pop_word(memory);
                self.set_af(nn);
                12
            }
            0xF2 => {
                // LD A, (C)
                let addr = 0xFF00 | (self.c as u16);
                self.a = memory.read_byte(addr);
                8
            }
            0xF3 => {
                // DI
                self.ime = false;
                4
            }
            0xF5 => {
                // PUSH AF
                self.push_word(memory, self.get_af());
                16
            }
            0xF6 => {
                // OR d8
                let nn = self.load_byte(memory);
                self.a = self.or_r8(nn);
                8
            }
            0xF7 => {
                // RST 30H
                self.push_word(memory, self.pc);
                self.pc = 0x0030;
                16
            }
            0xF8 => {
                // LD HL, SP+r8
                let nn = self.load_byte(memory) as i8;
                self.set_half_carry_flag((self.sp & 0x0F) + (nn as u16 & 0x0F) > 0x0F);
                self.set_carry_flag((self.sp & 0xFF) + (nn as u16 & 0xFF) > 0xFF);
                let hl = self.sp.wrapping_add(nn as u16);
                self.set_hl(hl);
                self.set_zero_flag(false);
                self.set_subtraction_flag(false);
                16
            }
            0xF9 => {
                // LD SP, HL
                self.sp = self.get_hl();
                8
            }
            0xFA => {
                // LD A, (a16)
                let nn = self.load_word(memory);
                self.a = memory.read_byte(nn);
                16
            }
            0xFB => {
                // EI
                self.enable_ime = true;
                4
            }
            0xFE => {
                // CP d8
                let nn = self.load_byte(memory);
                self.cp_r8(nn);
                8
            }
            0xFF => {
                // RST 38H
                self.push_word(memory, self.pc);
                self.pc = 0x0038;
                16
            }
            _ => {
                panic!("Opcode not implemented {:#04x}", opcode);
            }
        }
    }

    fn handle_interrupts(&mut self, memory: &mut Memory) {
        // FFFF - IE: Interrupt enable
        let interrupt_enable = memory.read_byte(0xFFFF) & 0b0001_1111;
        // FF0F — IF: Interrupt flag
        let interrupt_flag = memory.read_byte(0xFF0F) & 0b0001_1111;

        if interrupt_enable & interrupt_flag == 0 {
            return;
        }

        if self.halt {
            self.halt = false;
        }

        if self.ime {
            self.ime = false;
            memory.write_byte(0xFF0F, interrupt_flag & !interrupt_enable);
            self.push_word(memory, self.pc);

            if (interrupt_enable & 0b0000_0001) != 0 {
                // V-Blank interrupt requested
                self.pc = 0x0040;
            } else if (interrupt_enable & 0b0000_0010) != 0 {
                // LCD STAT interrupt requested
                self.pc = 0x0048;
            } else if (interrupt_enable & 0b0000_0100) != 0 {
                // Timer interrupt requested
                self.pc = 0x0050;
            } else if (interrupt_enable & 0b0000_1000) != 0 {
                // Serial interrupt requested
                self.pc = 0x0058;
            } else if (interrupt_enable & 0b0001_0000) != 0 {
                // Joypad interrupt requested
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

        while self.cycles >= increment_every {
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
            self.cycles -= increment_every;
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
        self.cycles += cycles as u16;

        self.update_timer(memory, cycles);

        if self.enable_ime {
            self.ime = true;
            self.enable_ime = false;
        }

        cycles
    }
}
