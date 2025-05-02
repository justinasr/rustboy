mod cpu;
mod debug;
mod memory;
mod ppu;
mod utils;

use cpu::CPU;
use debug::*;
use memory::Memory;
use ppu::PPU;

fn main() {
    // RAM
    let args: Vec<String> = std::env::args().collect();

    let bootrom_path = "boot.bin";
    let bootrom: Vec<u8> = std::fs::read(bootrom_path).unwrap();
    let cartridge_path = &args[1];
    let cartridge: Vec<u8> = std::fs::read(cartridge_path).unwrap();

    let mut memory = Memory::new(bootrom, cartridge);

    // Set LCD Y coordinate to 144 which indicates VBlank period
    // https://gbdev.io/pandocs/STAT.html#ff44--ly-lcd-y-coordinate-read-only
    memory.write_byte(0xFF44, 144);

    // CPU
    let mut cpu = CPU::new();

    // PPU
    let mut ppu = PPU::new();

    let mut total_cycles: u64 = 0;
    while total_cycles < 200_000_000 {
        let cycles = cpu.tick(&mut memory);
        ppu.tick(&mut memory, cycles);
        total_cycles += cycles as u64;

        // Wait for ROM to be done
        // if memory.read_byte(0xFF50) != 0 {
        //     println!("Bootrom done! Cycles: {total_cycles}");
        //     break;
        // }
    }

    // dump_vram(&memory);

    dump_background(&memory);
}
