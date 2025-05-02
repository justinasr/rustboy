use super::memory::Memory;

pub fn dump_memory_range(memory: &Memory, from: u16, to: u16) {
    for i in from..=to {
        if i % 64 == 0 {
            print!("{:#06x}   ", i);
        }
        print!("{:02x} ", memory.read_byte(i));
        if i % 64 == 63 {
            println!();
        }
    }
}

pub fn dump_vram(memory: &Memory) {
    println!("Tile data:");
    dump_memory_range(memory, 0x8000, 0x9FFF);
    println!("Tile map 1:");
    dump_memory_range(memory, 0x9800, 0x9BFF);
    println!("Tile map 2:");
    dump_memory_range(memory, 0x9C00, 0x9FFF);
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

pub fn dump_background(memory: &Memory) {
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
        // if !pixels[row as usize].iter().any(|i| *i != ' ') {
        //     continue;
        // }
        for col in 0..256 {
            print!("{}", pixels[row as usize][col as usize]);
        }
        println!();
    }
}