pub struct Memory {
    bootrom: Vec<u8>,
    cartridge: Vec<u8>,
    memory: Vec<u8>,
}

impl Memory {
    pub fn new(bootrom: Vec<u8>, cartridge: Vec<u8>) -> Memory {
        Memory {
            bootrom,
            cartridge,
            memory: vec![0; 0xFFFF + 1],
        }
    }

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
            // panic!("Trying to write {:#04x} to {:#06x}", value, addr)
            return;
        }
        if addr == 0xFF00 {
            let nn = (value & 0xF0) | (self.memory[addr as usize] & 0x0F);
            self.memory[addr as usize] = nn;
            return
        }
        if addr == 0xFF46 {
            // Very cycle-inaccurate DMA transfer.
            let start_addr = (value as u16) << 8;
            for i in 0..=0x9F {
                self.write_byte(0xFE00 + i, self.read_byte(start_addr + i));
            }
        }
        self.memory[addr as usize] = value;
    }

    pub fn write_byte_(&mut self, addr: u16, value: u8) {
        self.memory[addr as usize] = value;
    }

    pub fn write_word(&mut self, addr: u16, value: u16) {
        self.write_byte(addr, (value & 0xFF) as u8);
        self.write_byte(addr + 1, ((value >> 8) & 0xFF) as u8);
    }
}
