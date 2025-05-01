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

    let bootrom_path = "boot.bin";
    let bootrom: Vec<u8> = std::fs::read(bootrom_path).unwrap();
    let cartridge_path = "tetris.gb";
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
