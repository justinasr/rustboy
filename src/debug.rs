use super::memory::Memory;

pub fn dump_memory_range(memory: &Memory, from: u16, to: u16, break_every: u16) {
    for i in from..=to {
        if i % break_every == 0 {
            print!("{:#06x}   ", i);
        }
        print!("{:02x} ", memory.read_byte(i));
        if i % break_every == break_every - 1 {
            println!();
        }
    }
}

pub fn dump_vram(memory: &Memory) {
    println!("Tile data:");
    dump_memory_range(memory, 0x8000, 0x97FF, 64);
    println!("Tile map 1:");
    dump_memory_range(memory, 0x9800, 0x9BFF, 32);
    println!("Tile map 2:");
    dump_memory_range(memory, 0x9C00, 0x9FFF, 32);
}

pub fn dump_tile(memory: &Memory, addr: u16) {
    for row in 0..8 {
        let word = memory.read_word(addr + row * 2);
        for col in 0..8 {
            let pixel = ((word >> (15 - col)) & 0b01) | (((word >> (7 - col)) << 1) & 0b10);
            match pixel {
                0 => print!(" "),
                1 => print!("░"),
                2 => print!("▒"),
                3 => print!("▓"),
                _ => unreachable!(),
            }
        }
        println!();
    }
}

fn get_tile_data_start(memory: &Memory) -> u16 {
    let lcdc = memory.read_byte(0xFF40);
    //  Bit 4 - BG & Window Tile Data Select
    //  0: $8800-$97FF
    //  1: $8000-$8FFF <- Same area as OBJ
    if lcdc & 0b0001_0000 == 0 {
        0x9000 as u16
    } else {
        0x8000 as u16
    }
}

fn get_tile_map_start(memory: &Memory) -> u16 {
    let lcdc = memory.read_byte(0xFF40);
    //  Bit 3 - BG Tile Map Display Select
    //  0: $9800-$9BFF
    //  1: $9C00-$9FFF
    if lcdc & 0b0000_1000 == 0 {
        0x9800 as u16
    } else {
        0x9C00 as u16
    }
}

pub fn dump_background(memory: &Memory) {
    let lcdc = memory.read_byte(0xFF40);
    let mut pixels = vec![vec![' '; 256]; 256];

    let tile_data_start = get_tile_data_start(memory);
    let tile_map_start = get_tile_map_start(memory);
    for bg_tile_row in 0..32 {
        for bg_tile_col in 0..32 {
            let tile_i = memory.read_byte(tile_map_start + bg_tile_row * 32 + bg_tile_col);
            let tile_addr = (tile_data_start as i32
                + if lcdc & 0b0001_0000 == 0 {
                    ((tile_i as i32) * 16) as i32
                } else {
                    ((tile_i as u8 as i32) * 16) as i32
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
    }

    for row in 0..256 {
        if !pixels[row as usize].iter().any(|i| *i != ' ') {
            continue;
        }
        for col in 0..256 {
            print!("{}", pixels[row as usize][col as usize]);
        }
        println!();
    }
}

pub fn dump_tile_data(memory: &Memory) {
    let mut pixels = vec![vec![' '; 128]; 192];
    for tile_row in 0..24 as u8 {
        for tile_col in 0..16 as u8 {
            let tile_addr = tile_addr(tile_row as u16 * 16 + tile_col as u16);
            for y in 0..8 as u8 {
                for x in 0..8 as u8 {
                    let xx = tile_col * 8 + x;
                    let yy = tile_row * 8 + y;
                    let pixel_i = tile_pixel_i(memory, tile_addr, x, y);
                    pixels[yy as usize][xx as usize] = match pixel_i {
                        0 => ' ',
                        1 => '░',
                        2 => '▒',
                        3 => '▓',
                        _ => unreachable!(),
                    };
                }
            }
        }
    }

    for row in 0..192 {
        if row % 8 == 0 {
            println!("");
        }
        for col in 0..128 {
            print!("{}", pixels[row as usize][col as usize]);
        }
        println!();
    }
}

pub fn tile_addr(tile_index: u16) -> u16 {
    0x8000 + tile_index * 16
}

pub fn tile_pixel_i(memory: &Memory, tile_addr: u16, x: u8, y: u8) -> u8 {
    let word = memory.read_word(tile_addr + y as u16 * 2);
    ((word >> (15 - x)) & 0b01) as u8 | (((word >> (7 - x)) << 1) & 0b10) as u8
}

pub fn debug_color(pixel: u8) -> [u8; 4] {
    match pixel {
        0 => [255, 255, 255, 255],
        1 => [160, 160, 160, 255],
        2 => [80, 80, 80, 255],
        3 => [0, 0, 0, 255],
        _ => unreachable!(),
    }
}

pub fn dump_oam(memory: &Memory) {
    dump_memory_range(memory, 0xFE00, 0xFE9F, 4);
}