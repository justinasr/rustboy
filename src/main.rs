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
        return self.memory[addr as usize];
    }

    pub fn read_word(&self, addr: u16) -> u16 {
        let lsb = self.read_byte(addr) as u16;
        let msb = self.read_byte(addr + 1) as u16;
        return (msb << 8) | lsb;
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
        return &mut (self.memory[addr as usize]);
    }
}

fn set_zero_flag(f: &mut u8, value: bool) {
    if value {
        *f |= 0b1000_0000;
    } else {
        *f &= 0b0111_1111;
    }
}

fn set_subtraction_flag(f: &mut u8, value: bool) {
    if value {
        *f |= 0b0100_0000;
    } else {
        *f &= 0b1011_1111;
    }
}

fn set_half_carry_flag(f: &mut u8, value: bool) {
    if value {
        *f |= 0b0010_0000;
    } else {
        *f &= 0b1100_1111;
    }
}

fn set_carry_flag(f: &mut u8, value: bool) {
    if value {
        *f |= 0b0001_0000;
    } else {
        *f &= 0b1110_1111;
    }
}

fn get_zero_flag(f: u8) -> bool {
    return f & 0b1000_0000 != 0;
}

fn get_carry_flag(f: u8) -> bool {
    return f & 0b0001_0000 != 0;
}

fn get_lsb(r: u16) -> u8 {
    return (r & 0xFF) as u8;
}

fn get_msb(r: u16) -> u8 {
    return ((r >> 8) & 0xFF) as u8;
}

fn u8_u8_to_u16(msb: u8, lsb: u8) -> u16 {
    return ((msb as u16) << 8) | (lsb as u16);
}

fn u16_to_r8_r8(value: u16, msb: &mut u8, lsb: &mut u8) {
    *msb = get_msb(value);
    *lsb = get_lsb(value);
}

// Instructions

fn dec_r8(r: &mut u8, f: &mut u8) {
    *r = if *r == 0 { 255 } else { *r - 1 };
    set_zero_flag(f, *r == 0);
    set_subtraction_flag(f, true);
    set_half_carry_flag(f, (*r & 0b0000_1000) == 0);
}

fn inc_r8(r: &mut u8, f: &mut u8) {
    *r = if *r == 255 { 0 } else { *r + 1 };
    set_zero_flag(f, *r == 0);
    set_subtraction_flag(f, false);
    set_half_carry_flag(f, (*r & 0b0000_1000) == 0);
}

fn dec_r16(msb: &mut u8, lsb: &mut u8) {
    let mut x: u16 = u8_u8_to_u16(*msb, *lsb);
    x = if x == 0 { 0xFFFF } else { x - 1 };
    *msb = ((x >> 8) & 0xFF) as u8;
    *lsb = ((x >> 0) & 0xFF) as u8;
}

fn inc_r16(msb: &mut u8, lsb: &mut u8) {
    let mut x: u16 = u8_u8_to_u16(*msb, *lsb);
    x = if x == 0xFFFF { 0 } else { x + 1 };
    *msb = ((x >> 8) & 0xFF) as u8;
    *lsb = ((x >> 0) & 0xFF) as u8;
}

fn xor_r8_a(a: u8, r: &mut u8, f: &mut u8) {
    *r = *r ^ a;
    set_zero_flag(f, *r == 0);
    set_subtraction_flag(f, false);
    set_half_carry_flag(f, false);
    set_carry_flag(f, false);
}

fn rl_r8(r: &mut u8, f: &mut u8) {
    // Rotate bits in register r left, through the carry flag.
    set_carry_flag(f, *r & 0b1000_0000 != 0);
    *r <<= 1;
    *r |= if get_carry_flag(*f) { 1 } else { 0 };
    set_zero_flag(f, *r == 0);
    set_subtraction_flag(f, false);
    set_half_carry_flag(f, false);
}

fn bit_r8(b: u8, r: u8, f: &mut u8) {
    // Test bit b in register r, set the zero flag if bit is 0.
    set_zero_flag(f, r & (1 << b) == 0);
    set_subtraction_flag(f, false);
    set_half_carry_flag(f, true);
}

fn sub_r8(a: &mut u8, r: u8, f: &mut u8) {
    *a = if *a < r { 255 - (r - *a) } else { *a - r };
    set_zero_flag(f, *a == 0);
    set_subtraction_flag(f, true);
    set_half_carry_flag(f, (*a & 0b0000_1000) == 0);
    set_carry_flag(f, *a & 0b1000_0000 != 0);
}

fn add_r8(a: &mut u8, r: u8, f: &mut u8) {
    *a = ((*a as u16 + r as u16) & 0x00FF) as u8;
    set_zero_flag(f, *a == 0);
    set_subtraction_flag(f, false);
    set_half_carry_flag(f, (*a & 0b0000_1000) == 0);
    set_carry_flag(f, *a & 0b1000_0000 != 0);
}

fn cp_r8(a: u8, r: u8, f: &mut u8) {
    set_zero_flag(f, a == r);
    set_subtraction_flag(f, true);
    set_half_carry_flag(f, (a & 0xF) < (r & 0xF));
    set_carry_flag(f, a < r);
}

fn main() {
    // Registers
    let mut pc: u16 = 0; // Program Counter (PC)
    let mut sp: u16 = 0; // Stack Pointer (SP)
    let mut a: u8 = 0; // Accumulator (A)

    let mut b: u8 = 0; // Register (B)
    let mut c: u8 = 0; // Register (C)

    let mut d: u8 = 0; // Register (D)
    let mut e: u8 = 0; // Register (E)

    let mut h: u8 = 0; // Register (H)
    let mut l: u8 = 0; // Register (L)

    // Flag register (F) bits:
    // 7 6 5 4 3 2 1 0
    // Z N H C 0 0 0 0
    let mut f: u8 = 0; // Flags (F)

    let mut ir: u8 = 0; // Instruction Register (IR)
    let mut ie: u8 = 0; // Interrupt Enable (IE)

    // RAM
    let mut ram: Vec<u8> = vec![0; 0xFFFF + 1];

    // Set LCD Y coordinate to 144 which indicates VBlank period
    // https://gbdev.io/pandocs/STAT.html#ff44--ly-lcd-y-coordinate-read-only
    ram[0xFF44] = 0x90;

    let bootrom_path = "boot.bin";
    let bootrom: Vec<u8> = std::fs::read(bootrom_path).unwrap();
    let cartridge_path = "zelda.gb";
    let cartridge: Vec<u8> = std::fs::read(cartridge_path).unwrap();

    let mut memory = Memory {
        bootrom,
        cartridge,
        memory: ram,
    };

    let mut iter = 0;
    while iter < 100_000 {
        let opcode = memory.read_byte(pc);
        print!("[{:8}] PC = {:#06x} | Opcode = {:#04x} |", iter, pc, opcode);
        pc += 1;

        match opcode {
            0x00 => {
                println!(" NOP");
            }
            0x01 => {
                println!(" LD BC, d16");
                let nn = memory.read_word(pc);
                pc += 2;
                b = get_msb(nn);
                c = get_lsb(nn);
            }
            0x02 => {
                println!(" LD (BC), A");
                let addr = u8_u8_to_u16(b, c);
                memory.write_byte(addr, a);
            }
            0x03 => {
                println!(" INC BC");
                inc_r16(&mut b, &mut c);
            }
            0x04 => {
                println!(" INC B");
                inc_r8(&mut b, &mut f);
            }
            0x05 => {
                println!(" DEC B");
                dec_r8(&mut b, &mut f);
            }
            0x06 => {
                println!(" LD B, d8");
                b = memory.read_byte(pc);
                pc += 1;
            }
            0x08 => {
                println!(" LD (a16), SP");
                let addr = memory.read_word(pc);
                pc += 2;
                memory.write_word(addr, sp);
            }
            0x0A => {
                println!(" LD A, (BC)");
                let addr = u8_u8_to_u16(b, c);
                a = memory.read_byte(addr);
            }
            0x0B => {
                println!(" DEC BC");
                dec_r16(&mut b, &mut c);
            }
            0x0C => {
                println!(" INC C");
                inc_r8(&mut c, &mut f);
            }
            0x0D => {
                println!(" DEC C");
                dec_r8(&mut c, &mut f);
            }
            0x0E => {
                println!(" LD C, d8");
                c = memory.read_byte(pc);
                pc += 1;
            }
            0x11 => {
                println!(" LD DE, d16");
                let nn = memory.read_word(pc);
                pc += 2;
                d = get_msb(nn);
                e = get_lsb(nn);
            }
            0x12 => {
                println!(" LD (DE), A");
                let addr = u8_u8_to_u16(d, e);
                memory.write_byte(addr, a);
            }
            0x13 => {
                println!(" INC DE");
                inc_r16(&mut d, &mut e);
            }
            0x14 => {
                println!(" INC D");
                inc_r8(&mut d, &mut f);
            }
            0x15 => {
                println!(" DEC D");
                dec_r8(&mut d, &mut f);
            }
            0x16 => {
                println!(" LD D, d8");
                d = memory.read_byte(pc);
                pc += 1;
            }
            0x17 => {
                println!(" RLA");
                // Rotate bits in register A left, through the carry flag.
                set_carry_flag(&mut f, a & 0b1000_0000 != 0);
                a <<= 1;
                a |= if get_carry_flag(f) { 1 } else { 0 };
                set_zero_flag(&mut f, false);
                set_subtraction_flag(&mut f, false);
                set_half_carry_flag(&mut f, false);
            }
            0x18 => {
                println!(" JR r8");
                // Relative jump
                let nn = memory.read_byte(pc) as i8;
                pc += 1;
                pc = pc.checked_add_signed(nn as i16).unwrap();
            }
            0x1A => {
                println!(" LD A, (DE)");
                let addr = u8_u8_to_u16(d, e);
                a = memory.read_byte(addr);
            }
            0x1B => {
                println!(" DEC DE");
                dec_r16(&mut d, &mut e);
            }
            0x1C => {
                println!(" INC E");
                inc_r8(&mut e, &mut f);
            }
            0x1D => {
                println!(" DEC E");
                dec_r8(&mut e, &mut f);
            }
            0x1E => {
                println!(" LD E, d8");
                e = memory.read_byte(pc);
                pc += 1;
            }
            0x20 => {
                println!(" JR NZ, r8");
                // Relative jump
                let nn = memory.read_byte(pc) as i8;
                pc += 1;
                // Z flag not set - NotZero
                if !get_zero_flag(f) {
                    pc = pc.checked_add_signed(nn as i16).unwrap();
                }
            }
            0x21 => {
                println!(" LD HL, d16");
                let nn = memory.read_word(pc);
                pc += 2;
                h = get_msb(nn);
                l = get_lsb(nn);
            }
            0x22 => {
                println!(" LD (HL+), A");
                let addr = u8_u8_to_u16(h, l);
                memory.write_byte(addr, a);
                inc_r16(&mut h, &mut l);
            }
            0x23 => {
                println!(" INC HL");
                inc_r16(&mut h, &mut l);
            }
            0x24 => {
                println!(" INC H");
                inc_r8(&mut h, &mut f);
            }
            0x25 => {
                println!(" DEC H");
                dec_r8(&mut h, &mut f);
            }
            0x26 => {
                println!(" LD H, d8");
                h = memory.read_byte(pc);
                pc += 1;
            }
            0x28 => {
                println!(" JR Z, r8");
                // Relative jump
                let nn = memory.read_byte(pc) as i8;
                pc += 1;
                // Z flag set - Zero
                if get_zero_flag(f) {
                    pc = pc.checked_add_signed(nn as i16).unwrap();
                }
            }
            0x2A => {
                println!(" LD A, (HL+)");
                let addr = u8_u8_to_u16(h, l);
                a = memory.read_byte(addr);
                inc_r16(&mut h, &mut l);
            }
            0x2B => {
                println!(" DEC HL");
                dec_r16(&mut h, &mut l);
            }
            0x2C => {
                println!(" INC L");
                inc_r8(&mut l, &mut f);
            }
            0x2D => {
                println!(" DEC L");
                dec_r8(&mut l, &mut f);
            }
            0x2E => {
                println!(" LD L, d8");
                l = memory.read_byte(pc);
                pc += 1;
            }
            0x31 => {
                println!(" LD SP, d16");
                let nn = memory.read_word(pc);
                pc += 2;
                sp = nn;
            }
            0x32 => {
                println!(" LD (HL-), A");
                let addr = u8_u8_to_u16(h, l);
                memory.write_byte(addr, a);
                dec_r16(&mut h, &mut l);
            }
            0x33 => {
                println!(" INC SP");
                sp += 1;
            }
            0x34 => {
                println!(" INC (HL)");
                let addr = u8_u8_to_u16(h, l);
                inc_r8(memory.get_cell(addr), &mut f);
            }
            0x35 => {
                println!(" DEC (HL)");
                let addr = u8_u8_to_u16(h, l);
                dec_r8(memory.get_cell(addr), &mut f);
            }
            0x3A => {
                println!(" LD A, (HL-)");
                let addr = u8_u8_to_u16(h, l);
                a = memory.read_byte(addr);
                dec_r16(&mut h, &mut l);
            }
            0x3B => {
                println!(" DEC SP");
                sp -= 1;
            }
            0x3D => {
                println!(" DEC A");
                dec_r8(&mut a, &mut f);
            }
            0x3C => {
                println!(" INC A");
                inc_r8(&mut a, &mut f);
            }
            0x3E => {
                println!(" LD A, d8");
                a = memory.read_byte(pc);
                pc += 1;
            }
            0x40 => {
                println!(" LD B, B");
                b = b;
            }
            0x41 => {
                println!(" LD B, C");
                b = c;
            }
            0x42 => {
                println!(" LD B, D");
                b = d;
            }
            0x43 => {
                println!(" LD B, E");
                b = e;
            }
            0x44 => {
                println!(" LD B, H");
                b = h;
            }
            0x45 => {
                println!(" LD B, L");
                b = l;
            }
            0x46 => {
                println!(" LD B, (HL)");
                b = memory.read_byte(u8_u8_to_u16(h, l));
            }
            0x47 => {
                println!(" LD B, A");
                b = a;
            }
            0x48 => {
                println!(" LD C, B");
                c = b;
            }
            0x49 => {
                println!(" LD C, C");
                c = c;
            }
            0x4A => {
                println!(" LD C, D");
                c = d;
            }
            0x4B => {
                println!(" LD C, E");
                c = e;
            }
            0x4C => {
                println!(" LD C, H");
                c = h;
            }
            0x4D => {
                println!(" LD C, L");
                c = l;
            }
            0x4E => {
                println!(" LD C, (HL)");
                c = memory.read_byte(u8_u8_to_u16(h, l));
            }
            0x4F => {
                println!(" LD C, A");
                c = a;
            }
            0x50 => {
                println!(" LD D, B");
                d = b;
            }
            0x51 => {
                println!(" LD D, C");
                d = c;
            }
            0x52 => {
                println!(" LD D, D");
                d = d;
            }
            0x53 => {
                println!(" LD D, E");
                d = e;
            }
            0x54 => {
                println!(" LD D, H");
                d = h;
            }
            0x55 => {
                println!(" LD D, L");
                d = l;
            }
            0x56 => {
                println!(" LD D, (HL)");
                d = memory.read_byte(u8_u8_to_u16(h, l));
            }
            0x57 => {
                println!(" LD D, A");
                d = a;
            }
            0x58 => {
                println!(" LD E, B");
                e = b;
            }
            0x59 => {
                println!(" LD E, C");
                e = c;
            }
            0x5A => {
                println!(" LD E, D");
                e = d;
            }
            0x5B => {
                println!(" LD E, E");
                e = e;
            }
            0x5C => {
                println!(" LD E, H");
                e = h;
            }
            0x5D => {
                println!(" LD E, L");
                e = l;
            }
            0x5E => {
                println!(" LD E, (HL)");
                e = memory.read_byte(u8_u8_to_u16(h, l));
            }
            0x5F => {
                println!(" LD E, A");
                e = a;
            }
            0x60 => {
                println!(" LD H, B");
                h = b;
            }
            0x61 => {
                println!(" LD H, C");
                h = c;
            }
            0x62 => {
                println!(" LD H, D");
                h = d;
            }
            0x63 => {
                println!(" LD H, E");
                h = e;
            }
            0x64 => {
                println!(" LD H, H");
                h = h;
            }
            0x65 => {
                println!(" LD H, L");
                h = l;
            }
            0x66 => {
                println!(" LD H, (HL)");
                h = memory.read_byte(u8_u8_to_u16(h, l));
            }
            0x67 => {
                println!(" LD H, A");
                h = a;
            }
            0x68 => {
                println!(" LD L, B");
                l = b;
            }
            0x69 => {
                println!(" LD L, C");
                l = c;
            }
            0x6A => {
                println!(" LD L, D");
                l = d;
            }
            0x6B => {
                println!(" LD L, E");
                l = e;
            }
            0x6C => {
                println!(" LD L, H");
                l = h;
            }
            0x6D => {
                println!(" LD L, L");
                l = l;
            }
            0x6E => {
                println!(" LD L, (HL)");
                l = memory.read_byte(u8_u8_to_u16(h, l));
            }
            0x6F => {
                println!(" LD L, A");
                l = a;
            }
            0x70 => {
                println!(" LD (HL), B");
                memory.write_byte(u8_u8_to_u16(h, l), b);
            }
            0x71 => {
                println!(" LD (HL), C");
                memory.write_byte(u8_u8_to_u16(h, l), c);
            }
            0x72 => {
                println!(" LD (HL), D");
                memory.write_byte(u8_u8_to_u16(h, l), d);
            }
            0x73 => {
                println!(" LD (HL), E");
                memory.write_byte(u8_u8_to_u16(h, l), e);
            }
            0x74 => {
                println!(" LD (HL), H");
                memory.write_byte(u8_u8_to_u16(h, l), h);
            }
            0x75 => {
                println!(" LD (HL), L");
                memory.write_byte(u8_u8_to_u16(h, l), l);
            }
            0x77 => {
                println!(" LD (HL), A");
                memory.write_byte(u8_u8_to_u16(h, l), a);
            }
            0x78 => {
                println!(" LD A, B");
                a = b;
            }
            0x79 => {
                println!(" LD A, C");
                a = c;
            }
            0x7A => {
                println!(" LD A, D");
                a = d;
            }
            0x7B => {
                println!(" LD A, E");
                a = e;
            }
            0x7C => {
                println!(" LD A, H");
                a = h;
            }
            0x7D => {
                println!(" LD A, L");
                a = l;
            }
            0x7E => {
                println!(" LD A, (HL)");
                a = memory.read_byte(u8_u8_to_u16(h, l));
            }
            0x7F => {
                println!(" LD A, A");
                a = a;
            }
            0x80 => {
                println!(" ADD A, B");
                add_r8(&mut a, b, &mut f);
            }
            0x81 => {
                println!(" ADD A, C");
                add_r8(&mut a, c, &mut f);
            }
            0x82 => {
                println!(" ADD A, D");
                add_r8(&mut a, d, &mut f);
            }
            0x83 => {
                println!(" ADD A, E");
                add_r8(&mut a, e, &mut f);
            }
            0x84 => {
                println!(" ADD A, H");
                add_r8(&mut a, h, &mut f);
            }
            0x85 => {
                println!(" ADD A, L");
                add_r8(&mut a, l, &mut f);
            }
            0x86 => {
                println!(" ADD A, (HL)");
                let addr = u8_u8_to_u16(h, l);
                add_r8(&mut a, memory.read_byte(addr), &mut f);
            }
            0x87 => {
                println!(" ADD A, A");
                let aa = a;
                sub_r8(&mut a, aa, &mut f);
            }
            0x90 => {
                println!(" SUB B");
                sub_r8(&mut a, b, &mut f);
            }
            0x91 => {
                println!(" SUB C");
                sub_r8(&mut a, c, &mut f);
            }
            0x92 => {
                println!(" SUB D");
                sub_r8(&mut a, d, &mut f);
            }
            0x93 => {
                println!(" SUB E");
                sub_r8(&mut a, e, &mut f);
            }
            0x94 => {
                println!(" SUB H");
                sub_r8(&mut a, h, &mut f);
            }
            0x95 => {
                println!(" SUB L");
                sub_r8(&mut a, l, &mut f);
            }
            0x96 => {
                println!(" SUB (HL)");
                let addr = u8_u8_to_u16(h, l);
                sub_r8(&mut a, memory.read_byte(addr), &mut f);
            }
            0x97 => {
                println!(" SUB A");
                let aa = a;
                sub_r8(&mut a, aa, &mut f);
            }
            0xA8 => {
                println!(" XOR B");
                xor_r8_a(a, &mut b, &mut f);
            }
            0xA9 => {
                println!(" XOR C");
                xor_r8_a(a, &mut c, &mut f);
            }
            0xAA => {
                println!(" XOR D");
                xor_r8_a(a, &mut d, &mut f);
            }
            0xAB => {
                println!(" XOR E");
                xor_r8_a(a, &mut e, &mut f);
            }
            0xAC => {
                println!(" XOR H");
                xor_r8_a(a, &mut h, &mut f);
            }
            0xAD => {
                println!(" XOR L");
                xor_r8_a(a, &mut l, &mut f);
            }
            0xAE => {
                println!(" XOR (HL)");
                let addr = u8_u8_to_u16(h, l);
                xor_r8_a(a, memory.get_cell(addr), &mut f);
            }
            0xAF => {
                println!(" XOR A");
                xor_r8_a(a, &mut a, &mut f);
            }
            0xB8 => {
                println!(" CP B");
                cp_r8(a, b, &mut f);
            }
            0xB9 => {
                println!(" CP C");
                cp_r8(a, c, &mut f);
            }
            0xBA => {
                println!(" CP D");
                cp_r8(a, d, &mut f);
            }
            0xBB => {
                println!(" CP E");
                cp_r8(a, e, &mut f);
            }
            0xBC => {
                println!(" CP H");
                cp_r8(a, h, &mut f);
            }
            0xBD => {
                println!(" CP L");
                cp_r8(a, l, &mut f);
            }
            0xBE => {
                println!(" CP (HL)");
                let addr = u8_u8_to_u16(h, l);
                cp_r8(a, memory.read_byte(addr), &mut f);
            }
            0xBF => {
                println!(" CP A");
                cp_r8(a, a, &mut f);
            }
            0xC1 => {
                println!(" POP BC");
                let nn = memory.read_word(sp);
                sp += 2;
                u16_to_r8_r8(nn, &mut b, &mut c);
            }
            0xC3 => {
                println!(" JP a16");
                let nn = memory.read_word(pc);
                pc += 2;
                pc = nn;
            }
            0xC9 => {
                println!(" RET");
                pc = memory.read_word(sp);
                sp += 2;
            }
            0xCB => {
                let cb_opcode = memory.read_byte(pc);
                print!(" CB Opcode = {:#04x} |", cb_opcode);
                pc += 1;
                match cb_opcode {
                    0x10 => {
                        println!(" RL B");
                        rl_r8(&mut b, &mut f);
                    }
                    0x11 => {
                        println!(" RL C");
                        rl_r8(&mut c, &mut f);
                    }
                    0x12 => {
                        println!(" RL D");
                        rl_r8(&mut d, &mut f);
                    }
                    0x13 => {
                        println!(" RL E");
                        rl_r8(&mut e, &mut f);
                    }
                    0x14 => {
                        println!(" RL H");
                        rl_r8(&mut h, &mut f);
                    }
                    0x15 => {
                        println!(" RL L");
                        rl_r8(&mut l, &mut f);
                    }
                    0x16 => {
                        println!(" RL (HL)");
                        let addr = u8_u8_to_u16(h, l);
                        rl_r8(memory.get_cell(addr), &mut f);
                    }
                    0x17 => {
                        println!(" RL A");
                        rl_r8(&mut a, &mut f);
                    }

                    0x40 => {
                        println!(" BIT 0, B");
                        bit_r8(0, b, &mut f);
                    }
                    0x41 => {
                        println!(" BIT 0, C");
                        bit_r8(0, c, &mut f);
                    }
                    0x42 => {
                        println!(" BIT 0, D");
                        bit_r8(0, d, &mut f);
                    }
                    0x43 => {
                        println!(" BIT 0, E");
                        bit_r8(0, e, &mut f);
                    }
                    0x44 => {
                        println!(" BIT 0, H");
                        bit_r8(0, h, &mut f);
                    }
                    0x45 => {
                        println!(" BIT 0, L");
                        bit_r8(0, l, &mut f);
                    }
                    0x46 => {
                        println!(" BIT 0, (HL)");
                        bit_r8(0, memory.read_byte(u8_u8_to_u16(h, l)), &mut f);
                    }
                    0x47 => {
                        println!(" BIT 1, A");
                        bit_r8(1, a, &mut f);
                    }
                    0x48 => {
                        println!(" BIT 1, B");
                        bit_r8(1, b, &mut f);
                    }
                    0x49 => {
                        println!(" BIT 1, C");
                        bit_r8(1, c, &mut f);
                    }
                    0x4A => {
                        println!(" BIT 1, D");
                        bit_r8(1, d, &mut f);
                    }
                    0x4B => {
                        println!(" BIT 1, E");
                        bit_r8(1, e, &mut f);
                    }
                    0x4C => {
                        println!(" BIT 1, H");
                        bit_r8(1, h, &mut f);
                    }
                    0x4D => {
                        println!(" BIT 1, L");
                        bit_r8(1, l, &mut f);
                    }
                    0x4E => {
                        println!(" BIT 1, (HL)");
                        bit_r8(1, memory.read_byte(u8_u8_to_u16(h, l)), &mut f);
                    }
                    0x4F => {
                        println!(" BIT 1, A");
                        bit_r8(1, a, &mut f);
                    }
                    0x50 => {
                        println!(" BIT 2, B");
                        bit_r8(2, b, &mut f);
                    }
                    0x51 => {
                        println!(" BIT 2, C");
                        bit_r8(2, c, &mut f);
                    }
                    0x52 => {
                        println!(" BIT 2, D");
                        bit_r8(2, d, &mut f);
                    }
                    0x53 => {
                        println!(" BIT 2, E");
                        bit_r8(2, e, &mut f);
                    }
                    0x54 => {
                        println!(" BIT 2, H");
                        bit_r8(2, h, &mut f);
                    }
                    0x55 => {
                        println!(" BIT 2, L");
                        bit_r8(2, l, &mut f);
                    }
                    0x56 => {
                        println!(" BIT 2, (HL)");
                        bit_r8(2, memory.read_byte(u8_u8_to_u16(h, l)), &mut f);
                    }
                    0x57 => {
                        println!(" BIT 2, A");
                        bit_r8(2, a, &mut f);
                    }
                    0x58 => {
                        println!(" BIT 3, B");
                        bit_r8(3, b, &mut f);
                    }
                    0x59 => {
                        println!(" BIT 3, C");
                        bit_r8(3, c, &mut f);
                    }
                    0x5A => {
                        println!(" BIT 3, D");
                        bit_r8(3, d, &mut f);
                    }
                    0x5B => {
                        println!(" BIT 3, E");
                        bit_r8(3, e, &mut f);
                    }
                    0x5C => {
                        println!(" BIT 3, H");
                        bit_r8(3, h, &mut f);
                    }
                    0x5D => {
                        println!(" BIT 3, L");
                        bit_r8(3, l, &mut f);
                    }
                    0x5E => {
                        println!(" BIT 3, (HL)");
                        bit_r8(3, memory.read_byte(u8_u8_to_u16(h, l)), &mut f);
                    }
                    0x5F => {
                        println!(" BIT 3, A");
                        bit_r8(3, a, &mut f);
                    }
                    0x60 => {
                        println!(" BIT 4, B");
                        bit_r8(4, b, &mut f);
                    }
                    0x61 => {
                        println!(" BIT 4, C");
                        bit_r8(4, c, &mut f);
                    }
                    0x62 => {
                        println!(" BIT 4, D");
                        bit_r8(4, d, &mut f);
                    }
                    0x63 => {
                        println!(" BIT 4, E");
                        bit_r8(4, e, &mut f);
                    }
                    0x64 => {
                        println!(" BIT 4, H");
                        bit_r8(4, h, &mut f);
                    }
                    0x65 => {
                        println!(" BIT 4, L");
                        bit_r8(4, l, &mut f);
                    }
                    0x66 => {
                        println!(" BIT 4, (HL)");
                        bit_r8(4, memory.read_byte(u8_u8_to_u16(h, l)), &mut f);
                    }
                    0x67 => {
                        println!(" BIT 4, A");
                        bit_r8(4, a, &mut f);
                    }
                    0x68 => {
                        println!(" BIT 5, B");
                        bit_r8(5, b, &mut f);
                    }
                    0x69 => {
                        println!(" BIT 5, C");
                        bit_r8(5, c, &mut f);
                    }
                    0x6A => {
                        println!(" BIT 5, D");
                        bit_r8(5, d, &mut f);
                    }
                    0x6B => {
                        println!(" BIT 5, E");
                        bit_r8(5, e, &mut f);
                    }
                    0x6C => {
                        println!(" BIT 5, H");
                        bit_r8(5, h, &mut f);
                    }
                    0x6D => {
                        println!(" BIT 5, L");
                        bit_r8(5, l, &mut f);
                    }
                    0x6E => {
                        println!(" BIT 5, (HL)");
                        bit_r8(5, memory.read_byte(u8_u8_to_u16(h, l)), &mut f);
                    }
                    0x6F => {
                        println!(" BIT51, A");
                        bit_r8(5, a, &mut f);
                    }
                    0x70 => {
                        println!(" BIT 6, B");
                        bit_r8(6, b, &mut f);
                    }
                    0x71 => {
                        println!(" BIT 6, C");
                        bit_r8(6, c, &mut f);
                    }
                    0x72 => {
                        println!(" BIT 6, D");
                        bit_r8(6, d, &mut f);
                    }
                    0x73 => {
                        println!(" BIT 6, E");
                        bit_r8(6, e, &mut f);
                    }
                    0x74 => {
                        println!(" BIT 6, H");
                        bit_r8(6, h, &mut f);
                    }
                    0x75 => {
                        println!(" BIT 6, L");
                        bit_r8(6, l, &mut f);
                    }
                    0x76 => {
                        println!(" BIT 6, (HL)");
                        bit_r8(6, memory.read_byte(u8_u8_to_u16(h, l)), &mut f);
                    }
                    0x77 => {
                        println!(" BIT 6, A");
                        bit_r8(6, a, &mut f);
                    }
                    0x78 => {
                        println!(" BIT 7, B");
                        bit_r8(7, b, &mut f);
                    }
                    0x79 => {
                        println!(" BIT 7, C");
                        bit_r8(7, c, &mut f);
                    }
                    0x7A => {
                        println!(" BIT 7, D");
                        bit_r8(7, d, &mut f);
                    }
                    0x7B => {
                        println!(" BIT 7, E");
                        bit_r8(7, e, &mut f);
                    }
                    0x7C => {
                        println!(" BIT 7, H");
                        bit_r8(7, h, &mut f);
                    }
                    0x7D => {
                        println!(" BIT 7, L");
                        bit_r8(7, l, &mut f);
                    }
                    0x7E => {
                        println!(" BIT 7, (HL)");
                        bit_r8(7, memory.read_byte(u8_u8_to_u16(h, l)), &mut f);
                    }
                    0x7F => {
                        println!(" BIT57, A");
                        bit_r8(7, a, &mut f);
                    }
                    _ => {
                        panic!("CB-opcode not implemented {:#04x}", cb_opcode);
                    }
                }
            }
            0xC5 => {
                println!(" PUSH BC");
                sp -= 2;
                memory.write_word(sp, u8_u8_to_u16(b, c));
            }
            0xCD => {
                println!(" CALL a16");
                // CALL nn: Call function
                // Unconditional function call to the absolute address specified by the 16-bit operand nn.
                let nn = memory.read_word(pc);
                pc += 2;
                sp -= 2;
                memory.write_word(sp, pc);
                pc = nn;
            }
            0xD1 => {
                println!(" POP DE");
                let nn = memory.read_word(sp);
                sp += 2;
                u16_to_r8_r8(nn, &mut d, &mut e);
            }
            0xD5 => {
                println!(" PUSH DE");
                sp -= 2;
                memory.write_word(sp, u8_u8_to_u16(d, e));
            }
            0xE0 => {
                println!(" LDH (a8),A");
                // LDH (n), A: Load from accumulator (direct 0xFF00+n)
                // Load to the address specified by the 8-bit immediate data n, data from the 8-bit A register. The
                // full 16-bit absolute address is obtained by setting the most significant byte to 0xFF and the
                // least significant byte to the value of n, so the possible range is 0xFF00-0xFFFF.
                let nn = memory.read_byte(pc);
                pc += 1;
                let addr = 0xFF00 | nn as u16;
                memory.write_byte(addr, a);
            }
            0xE1 => {
                println!(" POP HL");
                let nn = memory.read_word(sp);
                sp += 2;
                u16_to_r8_r8(nn, &mut h, &mut l);
            }
            0xE2 => {
                println!(" LD (C), A");
                let addr = 0xFF00 | (c as u16);
                memory.write_byte(addr, a);
            }
            0xE5 => {
                println!(" PUSH HL");
                sp -= 2;
                memory.write_word(sp, u8_u8_to_u16(h, l));
            }
            0xEA => {
                println!(" LD (a16), A");
                let nn = memory.read_word(pc);
                pc += 2;
                memory.write_byte(nn, a);
            }
            0xEE => {
                println!(" XOR d8");
                let nn = memory.read_byte(pc);
                pc += 1;
                xor_r8_a(nn, &mut a, &mut f);
            }
            0xF0 => {
                println!(" LDH A, (a8)");
                let nn = memory.read_byte(pc);
                pc += 1;
                let addr = 0xFF00 | nn as u16;
                a = memory.read_byte(addr);
            }
            0xF2 => {
                println!(" LD A, (C)");
                let addr = 0xFF00 | (c as u16);
                a = memory.read_byte(addr);
            }
            0xF5 => {
                println!(" PUSH AF");
                sp -= 2;
                memory.write_word(sp, u8_u8_to_u16(a, f));
            }
            0xFE => {
                println!(" CP d8");
                let nn = memory.read_byte(pc);
                pc += 1;
                cp_r8(a, nn, &mut f);
            }
            _ => {
                println!(" ???");
                panic!("Opcode not implemented {:#04x}", opcode);
            }
        }

        iter += 1;
    }
}
