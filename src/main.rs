struct Memory {
    bootrom: Vec<u8>,
    cartridge: Vec<u8>,
    memory: Vec<u8>,
}

impl Memory {
    pub fn read_byte(&self, addr: u16) -> u8 {
        if self.memory[0xFF50] == 0 && (addr as usize) < self.bootrom.len() {
            return self.bootrom[addr as usize];
        }
        if addr < 0x8000 {
            return self.cartridge[addr as usize];
        }
        self.memory[addr as usize]
    }

    pub fn read_word(&self, addr: u16) -> u16 {
        let lsb = self.read_byte(addr) as u16;
        let msb = self.read_byte(addr + 1) as u16;
        (msb << 8) | lsb
    }

    pub fn write_byte(&mut self, addr: u16, value: u8) {
        if addr < 0x8000 {
            // I guess it is not supposed to write anything there?
            panic!("Trying to write {:#04x} to {:#06x}", value, addr)
        }
        self.memory[addr as usize] = value;
    }

    pub fn write_word(&mut self, addr: u16, value: u16) {
        self.write_byte(addr, (value & 0xFF) as u8);
        self.write_byte(addr + 1, ((value >> 8) & 0xFF) as u8);
    }

    pub fn get_cell(&mut self, addr: u16) -> &mut u8 {
        &mut (self.memory[addr as usize])
    }
}

struct CPU {
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
        return self.f & 0b1000_0000 != 0;
    }

    fn get_carry_flag(&self) -> bool {
        return self.f & 0b0001_0000 != 0;
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

    fn add_r8(&mut self, r: u8) -> u8 {
        let mut a = self.a;
        self.set_half_carry_flag((a & 0x0F) + (r & 0x0F) > 0x0F);
        self.set_carry_flag((a as u16) + (r as u16) > 0xFF);
        a = a.wrapping_add(r);
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
        self.set_half_carry_flag(false);
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

    // Main function

    pub fn tick(&mut self, memory: &mut Memory) -> u8 {
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
            0x08 => {
                // println!(" LD (a16), SP");
                let addr = self.load_word(memory);
                memory.write_word(addr, self.sp);
                cycles = 20;
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
                // println!(" JR r8");
                // Relative jump
                let nn = self.load_byte(memory) as i8 as i16;
                self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                cycles = 12;
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
            0x20 => {
                // print!(" JR NZ, r8");
                let nn = self.load_byte(memory) as i8 as i16;
                if !self.get_zero_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                    // println!("  | JR NZ, {:#04x}    => JR NZ, {:#06x}", nn as u8, self.pc);
                    cycles = 12;
                } else {
                    // println!();
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
                panic!("HALT");
            }
            0x80..=0x87 => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" ADD A, {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.add_r8(nn);
                cycles = if i == 6 { 8 } else { 4 };
            }
            0x90..=0x97 => {
                let i = opcode & 0b0111; // Lower nibble
                // println!(" SUB {}", self.get_reg_name(i));
                let nn = self.get_reg(memory, i);
                self.a = self.sub_r8(nn);
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
            0xC1 => {
                // println!(" POP BC");
                let nn = self.pop_word(memory);
                self.set_bc(nn);
                cycles = 12;
            }
            0xC3 => {
                // println!(" JP a16");
                self.pc = self.load_word(memory);
                cycles = 16;
            }
            0xC5 => {
                // print!(" PUSH BC");
                self.push_word(memory, self.get_bc());
                // println!("     | PUSH({:#06x})", self.get_bc());
                cycles = 16;
            }
            0xC9 => {
                // println!(" RET");
                self.pc = self.pop_word(memory);
                cycles = 16;
            }
            0xCB => {
                let cb_opcode = self.load_byte(memory);
                // print!(" CB Opcode = {:#04x} |", cb_opcode);
                let i = cb_opcode & 0b0111;
                match cb_opcode {
                    0x10..=0x17 => {
                        // println!(" RL {}", self.get_reg_name(i));
                        let nn = self.get_reg(memory, i);
                        if i == 6 {
                            *memory.get_cell(self.get_hl()) = self.rl_r8(nn);
                        } else {
                            *self.get_mut_reg(i) = self.rl_r8(nn);
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
                    _ => {
                        panic!("CB-opcode not implemented {:#04x}", cb_opcode);
                    }
                }
                cycles = if i == 6 { 16 } else { 8 }
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
            0xD1 => {
                // println!(" POP DE");
                let nn = self.pop_word(memory);
                self.set_de(nn);
                cycles = 12;
            }
            0xD5 => {
                // println!(" PUSH DE");
                self.push_word(memory, self.get_de());
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
            0xF0 => {
                // print!(" LD A, (a8)");
                let nn = self.load_byte(memory);
                let addr = 0xFF00 | nn as u16;
                self.a = memory.read_byte(addr);
                // println!(" | LD A, ({:#06x}) => LD A, {:#04x}", addr, self.a);
                cycles = 12;
            }
            0xF2 => {
                // println!(" LD A, (C)");
                let addr = 0xFF00 | (self.c as u16);
                self.a = memory.read_byte(addr);
                cycles = 8;
            }
            0xF5 => {
                // println!(" PUSH AF");
                self.push_word(memory, self.get_af());
                cycles = 16;
            }
            0xFE => {
                // print!(" CP d8");
                let nn = self.load_byte(memory);
                // println!("      | CP {:#04x}", nn);
                self.cp_r8(nn);
                cycles = 8;
            }
            _ => {
                // println!(" ???");
                panic!("Opcode not implemented {:#04x}", opcode);
            }
        }

        cycles
    }
}

struct PPU {
    cycles: u64,
    pixels: Vec<Vec<u8>>,
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            cycles: 0,
            pixels: vec![vec![0; 160]; 154],
        }
    }

    pub fn tick(&mut self, memory: &mut Memory, cycles: u8) {
        // println!("[PPU] Cycles {cycles} | LY: {}", memory.read_byte(0xFF44));

        self.cycles += cycles as u64;

        if self.cycles >= 456 {
            // End of scanline
            self.cycles -= 456;
            let nn = (memory.read_byte(0xFF44) + 1) % 154;
            memory.write_byte(0xFF44, nn);
        }
    }
}

/** Join two bytes into word. */
fn u8_u8_to_u16(msb: u8, lsb: u8) -> u16 {
    ((msb as u16) << 8) | (lsb as u16)
}

/** Split word into two bytes. */
fn u16_to_u8_u8(word: u16) -> (u8, u8) {
    (((word >> 8) & 0xFF) as u8, (word & 0xFF) as u8)
}

/** Reset given bit b in r to 0. */
fn res_r8(b: u8, r: u8) -> u8 {
    r & !(1 << b)
}

/** Set given bit b in r to 1. */
fn set_r8(b: u8, r: u8) -> u8 {
    r | (1 << b)
}

/** Take two bytes, join them into word, decrement by 1 and return the two bytes. */
fn dec_r16(msb: u8, lsb: u8) -> (u8, u8) {
    u16_to_u8_u8(u8_u8_to_u16(msb, lsb).wrapping_sub(1))
}

/** Take two bytes, join them into word, increment by 1 and return the two bytes. */
fn inc_r16(msb: u8, lsb: u8) -> (u8, u8) {
    u16_to_u8_u8(u8_u8_to_u16(msb, lsb).wrapping_add(1))
}

fn dump_vram(memory: &Memory) {
    for i in 0x8000..=(0x9FFF as u16) {
        if i == 0x8000 {
            println!("Tile data:");
        } else if i == 0x9800 {
            println!("Tile map 1:");
        } else if i == 0x9C00 {
            println!("Tile map 2:");
        }
        if i % 64 == 0 {
            print!("{:#06x}   ", i);
        }
        print!("{:02x} ", memory.read_byte(i));
        if i % 64 == 63 {
            println!();
        }
    }
}

fn dump_tile_data(memory: &Memory) {
    for tile_i in 0..384 {
        let tile_addr = 0x8000 + tile_i * 16;
        let tile_data: Vec<u16> = (0..8)
            .map(|i| memory.read_word(tile_addr + i * 2))
            .collect();
        if !tile_data.iter().any(|i| *i > 0) {
            continue;
        }
        println!("Tile {}", tile_i);
        for tile_word in tile_data.iter() {
            for pixel_i in 0..8 {
                let pixel = ((tile_word >> (15 - pixel_i)) & 0b01)
                    | (((tile_word >> (7 - pixel_i)) << 1) & 0b10);
                match pixel {
                    0 => print!(" "),
                    1 => print!("░"),
                    2 => print!("▒"),
                    3 => print!("▓"),
                    _ => unreachable!(),
                };
            }
            println!();
        }
        println!();
    }
}

fn main() {
    // RAM
    let ram: Vec<u8> = vec![0; 0xFFFF + 1];

    let bootrom_path = "boot.bin";
    let bootrom: Vec<u8> = std::fs::read(bootrom_path).unwrap();
    let cartridge_path = "tetris.gb";
    let cartridge: Vec<u8> = std::fs::read(cartridge_path).unwrap();

    let mut memory = Memory {
        bootrom,
        cartridge,
        memory: ram,
    };

    // Set LCD Y coordinate to 144 which indicates VBlank period
    // https://gbdev.io/pandocs/STAT.html#ff44--ly-lcd-y-coordinate-read-only
    memory.write_byte(0xFF44, 144);

    // CPU
    let mut cpu = CPU::new();

    // PPU
    let mut ppu = PPU::new();

    let mut total_cycles: u64 = 0;
    while total_cycles < 20_000_000 {
        let cycles = cpu.tick(&mut memory);
        ppu.tick(&mut memory, cycles);
        total_cycles += cycles as u64;

        // Wait for ROM to be done
        // if memory.read_byte(0xFF50) != 0 {
        //     println!("Bootrom done! Cycles: {total_cycles}");
        //     break;
        // }
    }

    dump_vram(&memory);
    dump_tile_data(&memory);

    let lcdc = memory.read_byte(0xFF40);
    let mut pixels = vec![vec![' '; 256]; 256];
    let tile_data_start = if lcdc & 0b0001_0000 != 0 {
        0x8000 as u16
    } else {
        0x9000 as u16
    };
    // bit-3 BG tile map: 0 = 9800–9BFF; 1 = 9C00–9FFF
    let tile_map_start = if lcdc & 0b0000_1000 != 0 {
        0x9C00 as u16
    } else {
        0x9800 as u16
    };
    for bg_tile_row in 0..32 {
        for bg_tile_col in 0..32 {
            let tile_i = memory.read_byte(tile_map_start + bg_tile_row * 32 + bg_tile_col);
            let tile_addr = (tile_data_start as i32
                + if lcdc & 0b0001_0000 != 0 {
                    ((tile_i as u16) * 16) as i32
                } else {
                    ((tile_i as i8 as i8) * 16) as i32
                }) as u16;
            for tile_row in 0..8 {
                let tile_word = memory.read_word(tile_addr + tile_row * 2);
                for tile_col in 0..8 {
                    let pixel = ((tile_word >> (15 - tile_col)) & 0b01)
                        | (((tile_word >> (7 - tile_col)) << 1) & 0b10);
                    pixels[(bg_tile_row * 8 + tile_row) as usize]
                        [(bg_tile_col * 8 + tile_col) as usize] = match pixel {
                        0 => ' ',
                        1 => '░',
                        2 => '▒',
                        3 => '▓',
                        _ => unreachable!(),
                    };
                }
            }
        }
        println!();
    }

    for row in 0..256 {
        for col in 0..256 {
            print!("{}", pixels[row as usize][col as usize]);
        }
        println!();
    }
}
