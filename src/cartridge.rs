pub struct Cartridge {
    data: Vec<u8>,
    ram: Vec<u8>,
    rom_bank: u8,
    ram_bank: u8,
    rom_type: u8,
    ram_type: u8,
}

impl Cartridge {
    pub fn new(data: Vec<u8>) -> Cartridge {
        let rom_type = data[0x148];
        let ram_type = data[0x149];
        let ram_size = match ram_type {
            0x00 => 0,
            0x02 => 8192,
            _ => panic!("Unsupported RAM type {:#04x}", ram_type),
        };

        let ram = vec![0; ram_size];
        return Cartridge {
            data,
            ram,
            rom_bank: 0,
            ram_bank: 0,
            rom_type,
            ram_type,
        };
    }

    pub fn read_byte(&self, addr: u16) -> u8 {
        if addr < 0x4000 {
            // ROM Bank 00
            return self.data[addr as usize];
        }
        if addr < 0x8000 {
            if self.rom_type == 0x00 {
                // No banking
                return self.data[addr as usize];
            } else {
                // With banking - use ROM bank for offset
                let offset_in_bank = (addr - 0x4000) as usize;
                let bank_offset = (self.rom_bank as usize * 0x4000) as usize;
                return self.data[bank_offset + offset_in_bank];
            }
        }
        if addr >= 0xA000 && addr < 0xC000 {
            if self.ram_type == 0x00 {
                // No cartridge RAM
                panic!("No cartridge RAM")
            } else if self.ram_type == 0x02 {
                // 1 bank
                return self.ram[(addr - 0xA000) as usize];
            } else {
                unreachable!();
            }
        }
        panic!("Unexpected read from cartridge address {:#06x}", addr);
    }

    pub fn write_byte(&mut self, addr: u16, value: u8) {
        if addr < 0x2000 {
            // RAM Enable
            // TODO!
        } else if addr < 0x4000 {
            // ROM Bank number
            self.rom_bank = value & 0x1F;
        } else if addr < 0x6000 {
            // RAM Bank number
            self.ram_bank = value & 0x03;
        } else if addr < 0x8000 {
            // Banking mode select
            // TODO!
        } else if addr >= 0xA000 && addr < 0xC000 {
            if self.ram_type == 0x00 {
                // No cartridge RAM
                panic!("No cartridge RAM")
            } else if self.ram_type == 0x02 {
                // 1 bank
                self.ram[(addr - 0xA000) as usize] = value;
            } else {
                unreachable!();
            }
        } else {
            panic!("Unexpected write to cartridge address {:#06x}", addr);
        }
    }
}
