use super::memory::Memory;

pub struct PPU {
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

        let mut ly = memory.read_byte(0xFF44);
        self.cycles += cycles as u64;

        if self.cycles >= 456 {
            // End of scanline
            self.cycles -= 456;
            ly = (ly + 1) % 154;
            memory.write_byte(0xFF44, ly);
        }
        if ly == 144 {
            // Entering VBlank - request V-Blank interrupt
            let mut nn = memory.read_byte(0xFF0F);
            nn |= 0b0000_0001; // Bit 0
            memory.write_byte(0xFF0F, nn);
        }
    }
}
