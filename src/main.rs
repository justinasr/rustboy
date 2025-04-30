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

struct CPU<'a> {
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

    memory: &'a mut Memory,
}

impl<'a> CPU<'a> {
    pub fn new(memory: &'a mut Memory) -> CPU<'a> {
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
            memory,
        };
    }

    // Flags

    fn set_zero_flag(&mut self, value: bool) {
        if value {
            self.f |= 0b1000_0000;
        } else {
            self.f &= 0b0111_1111;
        }
    }

    fn set_subtraction_flag(&mut self, value: bool) {
        if value {
            self.f |= 0b0100_0000;
        } else {
            self.f &= 0b1011_1111;
        }
    }

    fn set_half_carry_flag(&mut self, value: bool) {
        if value {
            self.f |= 0b0010_0000;
        } else {
            self.f &= 0b1100_1111;
        }
    }

    fn set_carry_flag(&mut self, value: bool) {
        if value {
            self.f |= 0b0001_0000;
        } else {
            self.f &= 0b1110_1111;
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

    // Instruction/operand loading

    fn load_byte(&mut self) -> u8 {
        let nn = self.memory.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        nn
    }

    fn load_word(&mut self) -> u16 {
        let nn = self.memory.read_word(self.pc);
        self.pc = self.pc.wrapping_add(2);
        nn
    }

    // Stack operations

    fn pop_word(&mut self) -> u16 {
        let nn = self.memory.read_word(self.sp);
        self.sp = self.sp.wrapping_add(2);
        nn
    }

    fn push_word(&mut self, nn: u16) {
        self.sp = self.sp.wrapping_sub(2);
        self.memory.write_word(self.sp, nn);
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
        let hl = self.get_hl();
        match i {
            0 => &mut self.b,
            1 => &mut self.c,
            2 => &mut self.d,
            3 => &mut self.e,
            4 => &mut self.h,
            5 => &mut self.l,
            6 => self.memory.get_cell(hl),
            7 => &mut self.a,
            _ => unreachable!("Unexpected register index {i}"),
        }
    }

    fn get_reg(&mut self, i: u8) -> u8 {
        let hl = self.get_hl();
        match i {
            0 => self.b,
            1 => self.c,
            2 => self.d,
            3 => self.e,
            4 => self.h,
            5 => self.l,
            6 => self.memory.read_byte(hl),
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

    pub fn tick(&mut self) {
        let opcode = self.load_byte();
        print!("PC = {:#06x} | Opcode = {:#04x} |", self.pc, opcode);

        match opcode {
            0x00 => {
                println!(" NOP");
            }
            0x01 => {
                println!(" LD BC, d16");
                let nn = self.load_word();
                self.set_bc(nn)
            }
            0x02 => {
                println!(" LD (BC), A");
                let addr = self.get_bc();
                self.memory.write_byte(addr, self.a);
            }
            0x03 => {
                println!(" INC BC");
                (self.b, self.c) = inc_r16(self.b, self.c);
            }
            0x04 => {
                println!(" INC B");
                self.b = self.inc_r8(self.b);
            }
            0x05 => {
                println!(" DEC B");
                self.b = self.dec_r8(self.b);
            }
            0x06 => {
                println!(" LD B, d8");
                self.b = self.load_byte();
            }
            0x08 => {
                println!(" LD (a16), SP");
                let addr = self.load_word();
                self.memory.write_word(addr, self.sp);
            }
            0x0A => {
                println!(" LD A, (BC)");
                let addr = self.get_bc();
                self.a = self.memory.read_byte(addr);
            }
            0x0B => {
                println!(" DEC BC");
                (self.b, self.c) = dec_r16(self.b, self.c);
            }
            0x0C => {
                println!(" INC C");
                self.c = self.inc_r8(self.c);
            }
            0x0D => {
                println!(" DEC C");
                self.c = self.dec_r8(self.c);
            }
            0x0E => {
                println!(" LD C, d8");
                self.c = self.load_byte();
            }
            0x11 => {
                println!(" LD DE, d16");
                let nn = self.load_word();
                self.set_de(nn);
            }
            0x12 => {
                println!(" LD (DE), A");
                let addr = self.get_de();
                self.memory.write_byte(addr, self.a);
            }
            0x13 => {
                println!(" INC DE");
                (self.d, self.e) = inc_r16(self.d, self.e);
            }
            0x14 => {
                println!(" INC D");
                self.d = self.inc_r8(self.d);
            }
            0x15 => {
                println!(" DEC D");
                self.d = self.dec_r8(self.d);
            }
            0x16 => {
                println!(" LD D, d8");
                self.d = self.load_byte();
            }
            0x17 => {
                println!(" RLA");
                self.a = self.rl_r8(self.a);
                self.set_zero_flag(false);
            }
            0x18 => {
                println!(" JR r8");
                // Relative jump
                let nn = self.load_byte() as i8 as i16;
                self.pc = (self.pc as i16).wrapping_add(nn) as u16;
            }
            0x1A => {
                println!(" LD A, (DE)");
                let addr = self.get_de();
                self.a = self.memory.read_byte(addr);
            }
            0x1B => {
                println!(" DEC DE");
                (self.d, self.e) = dec_r16(self.d, self.e);
            }
            0x1C => {
                println!(" INC E");
                self.e = self.inc_r8(self.e);
            }
            0x1D => {
                println!(" DEC E");
                self.e = self.dec_r8(self.e);
            }
            0x1E => {
                println!(" LD E, d8");
                self.e = self.load_byte();
            }
            0x20 => {
                println!(" JR NZ, r8");
                let nn = self.load_byte() as i8 as i16;
                if !self.get_zero_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                }
            }
            0x21 => {
                println!(" LD HL, d16");
                let nn = self.load_word();
                self.set_hl(nn);
            }
            0x22 => {
                println!(" LD (HL+), A");
                let addr = self.get_hl();
                self.memory.write_byte(addr, self.a);
                (self.h, self.l) = inc_r16(self.h, self.l);
            }
            0x23 => {
                println!(" INC HL");
                (self.h, self.l) = inc_r16(self.h, self.l);
            }
            0x24 => {
                println!(" INC H");
                self.h = self.inc_r8(self.h);
            }
            0x25 => {
                println!(" DEC H");
                self.h = self.dec_r8(self.h);
            }
            0x26 => {
                println!(" LD H, d8");
                self.h = self.load_byte();
            }
            0x28 => {
                println!(" JR Z, r8");
                let nn = self.load_byte() as i8 as i16;
                if self.get_zero_flag() {
                    self.pc = (self.pc as i16).wrapping_add(nn) as u16;
                }
            }
            0x2A => {
                println!(" LD A, (HL+)");
                let addr = self.get_hl();
                self.a = self.memory.read_byte(addr);
                (self.h, self.l) = inc_r16(self.h, self.l);
            }
            0x2B => {
                println!(" DEC HL");
                (self.h, self.l) = dec_r16(self.h, self.l);
            }
            0x2C => {
                println!(" INC L");
                self.l = self.inc_r8(self.l);
            }
            0x2D => {
                println!(" DEC L");
                self.l = self.dec_r8(self.l);
            }
            0x2E => {
                println!(" LD L, d8");
                self.l = self.load_byte();
            }
            0x31 => {
                println!(" LD SP, d16");
                self.sp = self.load_word();
            }
            0x32 => {
                println!(" LD (HL+), A");
                let addr = self.get_hl();
                self.memory.write_byte(addr, self.a);
                (self.h, self.l) = dec_r16(self.h, self.l);
            }
            0x33 => {
                println!(" INC SP");
                self.sp = self.sp.wrapping_add(1);
            }
            0x34 => {
                println!(" INC (HL)");
                let addr = self.get_hl();
                let nn = self.inc_r8(self.memory.read_byte(addr));
                self.memory.write_byte(addr, nn);
            }
            0x35 => {
                println!(" DEC (HL)");
                let addr = self.get_hl();
                let nn = self.dec_r8(self.memory.read_byte(addr));
                self.memory.write_byte(addr, nn);
            }
            0x3A => {
                println!(" LD A, (HL-)");
                let addr = self.get_hl();
                self.a = self.memory.read_byte(addr);
                (self.h, self.l) = dec_r16(self.h, self.l);
            }
            0x3B => {
                println!(" DEC SP");
                self.sp = self.sp.wrapping_sub(1);
            }
            0x3D => {
                println!(" DEC A");
                self.a = self.dec_r8(self.a);
            }
            0x3C => {
                println!(" INC A");
                self.a = self.inc_r8(self.a);
            }
            0x3E => {
                println!(" LD A, d8");
                self.a = self.load_byte();
            }
            0x40..=0x75 | 0x77..=0x7F => {
                let lhs_i = (opcode >> 3) & 0b0111; // Upper nibble divided by 8
                let rhs_i = opcode & 0b0111; // Lower nibble
                let lhs_reg_name = self.get_reg_name(lhs_i);
                let rhs_reg_name = self.get_reg_name(rhs_i);
                println!(" LD {lhs_reg_name}, {rhs_reg_name}");
                *self.get_mut_reg(lhs_i) = self.get_reg(rhs_i);
            }
            0x76 => {
                println!(" HALT");
                todo!("HALT");
            }
            0x80..=0x87 => {
                let i = opcode & 0b0111; // Lower nibble
                let reg_name = self.get_reg_name(i);
                println!(" ADD A, {reg_name}");
                let nn = self.get_reg(i);
                self.a = self.add_r8(nn);
            }
            0x90..=0x97 => {
                let i = opcode & 0b0111; // Lower nibble
                let reg_name = self.get_reg_name(i);
                println!(" SUB {reg_name}");
                let nn = self.get_reg(i);
                self.a = self.sub_r8(nn);
            }
            0xA8..=0xAF => {
                let i = opcode & 0b0111; // Lower nibble
                let reg_name = self.get_reg_name(i);
                println!(" XOR {reg_name}");
                let nn = self.get_reg(i);
                self.a = self.xor(nn);
            }
            0xB8..=0xBF => {
                let i = opcode & 0b0111; // Lower nibble
                let reg_name = self.get_reg_name(i);
                println!(" CP {reg_name}");
                let nn = self.get_reg(i);
                self.cp_r8(nn);
            }
            0xC1 => {
                println!(" POP BC");
                let nn = self.pop_word();
                self.set_bc(nn);
            }
            0xC3 => {
                println!(" JP a16");
                self.pc = self.load_word();
            }
            0xC9 => {
                println!(" RET");
                self.pc = self.pop_word();
            }
            0xCB => {
                let cb_opcode = self.load_byte();
                print!(" CB Opcode = {:#04x} |", cb_opcode);
                let i = cb_opcode & 0b0111;
                let reg_name = self.get_reg_name(i);
                match cb_opcode {
                    0x10..=0x17 => {
                        println!(" RL {reg_name}");
                        let nn = self.get_reg(i);
                        *self.get_mut_reg(i) = self.rl_r8(nn);
                    }
                    0x40..=0x7F => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        println!(" BIT {bit}, {reg_name}");
                        let nn = self.get_reg(i);
                        self.bit_r8(bit, nn);
                    }
                    0x80..=0xBF => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        println!(" RES {bit}, {reg_name}");
                        let nn = self.get_reg(i);
                        *self.get_mut_reg(i) = res_r8(bit, nn);
                    }
                    0xC0..=0xFF => {
                        let bit = (cb_opcode >> 3) & 0b0111;
                        println!(" SET {bit}, {reg_name}");
                        let nn = self.get_reg(i);
                        *self.get_mut_reg(i) = set_r8(bit, nn);
                    }
                    _ => {
                        panic!("CB-opcode not implemented {:#04x}", cb_opcode);
                    }
                }
            }
            0xC5 => {
                println!(" PUSH BC");
                self.push_word(self.get_bc());
            }
            0xCD => {
                println!(" CALL a16");
                // CALL nn: Call function
                // Unconditional function call to the absolute address specified by the 16-bit operand nn.
                let nn = self.load_word();
                self.push_word(self.pc);
                self.pc = nn;
            }
            0xD1 => {
                println!(" POP DE");
                let nn = self.pop_word();
                self.set_de(nn);
            }
            0xD5 => {
                println!(" PUSH DE");
                self.push_word(self.get_de());
            }
            0xE0 => {
                println!(" LD (a8), A");
                // LD (n), A: Load from accumulator (direct 0xFF00+n)
                // Load to the address specified by the 8-bit immediate data n, data from the 8-bit A register. The
                // full 16-bit absolute address is obtained by setting the most significant byte to 0xFF and the
                // least significant byte to the value of n, so the possible range is 0xFF00-0xFFFF.
                let nn = self.load_byte();
                let addr = 0xFF00 | nn as u16;
                self.memory.write_byte(addr, self.a);
            }
            0xE1 => {
                println!(" POP HL");
                let nn = self.pop_word();
                self.set_hl(nn);
            }
            0xE2 => {
                println!(" LD (C), A");
                let addr = 0xFF00 | (self.c as u16);
                self.memory.write_byte(addr, self.a);
            }
            0xE5 => {
                println!(" PUSH HL");
                self.push_word(self.get_hl());
            }
            0xEA => {
                println!(" LD (a16), A");
                let nn = self.load_word();
                self.memory.write_byte(nn, self.a);
            }
            0xEE => {
                println!(" XOR d8");
                let nn = self.load_byte();
                self.a = self.xor(nn);
            }
            0xF0 => {
                println!(" LD A, (a8)");
                let nn = self.load_byte();
                let addr = 0xFF00 | nn as u16;
                self.a = self.memory.read_byte(addr);
            }
            0xF2 => {
                println!(" LD A, (C)");
                let addr = 0xFF00 | (self.c as u16);
                self.a = self.memory.read_byte(addr);
            }
            0xF5 => {
                println!(" PUSH AF");
                self.push_word(self.get_af());
            }
            0xFE => {
                println!(" CP d8");
                let nn = self.load_byte();
                self.cp_r8(nn);
            }
            _ => {
                println!(" ???");
                panic!("Opcode not implemented {:#04x}", opcode);
            }
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

fn main() {
    // RAM
    let ram: Vec<u8> = vec![0; 0xFFFF + 1];

    let bootrom_path = "boot.bin";
    let bootrom: Vec<u8> = std::fs::read(bootrom_path).unwrap();
    let cartridge_path = "zelda.gb";
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
    let mut cpu = CPU::new(&mut memory);

    let mut iter = 0;
    while iter < 30_000 {
        cpu.tick();
        iter += 1;
    }
}
