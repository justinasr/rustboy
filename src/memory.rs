use crate::cartridge::Cartridge;

pub struct Memory {
    cartridge: Cartridge,
    memory: Vec<u8>,
}

impl Memory {
    pub fn new(cartridge: Cartridge) -> Memory {
        Memory {
            cartridge,
            memory: vec![0; 0xFFFF + 1],
        }
    }

    pub fn read_byte(&self, addr: u16) -> u8 {
        // 0x0000 - 0x8000 - cartridge ROM
        // 0xA000 - 0xBFFF - cartridge RAM
        if addr < 0x8000 || addr >= 0xA000 && addr <= 0xBFFF {
            return self.cartridge.read_byte(addr);
        }
        self.memory[addr as usize]
    }

    pub fn read_word(&self, addr: u16) -> u16 {
        let lsb = self.read_byte(addr) as u16;
        let msb = self.read_byte(addr + 1) as u16;
        (msb << 8) | lsb
    }

    pub fn write_byte(&mut self, addr: u16, value: u8) {
        // 0x0000 - 0x8000 - cartridge ROM
        // 0xA000 - 0xBFFF - cartridge RAM
        if addr < 0x8000 || addr >= 0xA000 && addr <= 0xBFFF {
            self.cartridge.write_byte(addr, value);
            return;
        }
        // Writing XX to 0xFF46 starts a data transfer from (0xXX00..0xXX9F) to (0xFE00..0xFE9F).
        // This is cycle-inaccurate implementation of that.
        if addr == 0xFF46 {
            let source = (value as u16) << 8;
            for i in 0..=0x9F {
                let byte = self.read_byte(source + i);
                self.memory[0xFE00 + i as usize] = byte;
            }
            return;
        }
        self.memory[addr as usize] = value;
    }

    pub fn write_word(&mut self, addr: u16, value: u16) {
        self.write_byte(addr, (value & 0xFF) as u8);
        self.write_byte(addr + 1, ((value >> 8) & 0xFF) as u8);
    }
}
