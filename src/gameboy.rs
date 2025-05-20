use super::cartridge::Cartridge;
use super::cpu::CPU;
use super::memory::Memory;
use super::ppu::PPU;

pub struct Gameboy {
    pub memory: Memory,
    pub cpu: CPU,
    pub ppu: PPU,
}

impl Gameboy {
    pub fn new(cartridge_path: &str) -> Self {
        let bootrom_path = "boot.bin";
        let bootrom: Vec<u8> = std::fs::read(bootrom_path).unwrap();
        let cartridge_data: Vec<u8> = std::fs::read(cartridge_path).unwrap();
        let cartridge = Cartridge::new(cartridge_data.clone());

        // Memory
        let memory = Memory::new(bootrom, cartridge);

        // CPU
        let cpu = CPU::new();

        // PPU
        let ppu = PPU::new();

        Self { memory, cpu, ppu }
    }

    pub fn tick(&mut self) {
        let cycles = self.cpu.tick(&mut self.memory);
        self.ppu.tick(&mut self.memory, cycles);
    }
}
