use crate::debug::dump_tile;

use super::memory::Memory;

pub struct PPU {
    cycles: u64,
    pixels: Vec<Vec<u8>>,

    full_bg: Vec<Vec<u8>>,
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            cycles: 0,
            pixels: vec![vec![0; 160]; 154],

            full_bg: vec![vec![0; 256]; 256],
        }
    }

    pub fn get_pixel(&self, x: u8, y: u8) -> [u8; 4] {
        match self.pixels[y as usize][x as usize] {
            0 => [255, 255, 255, 255],
            1 => [160, 160, 160, 255],
            2 => [80, 80, 80, 255],
            3 => [0, 0, 0, 255],
            _ => unreachable!(),
        }
    }

    pub fn tick(&mut self, memory: &mut Memory, cycles: u8) {
        // println!("[PPU] Cycles {cycles} | LY: {}", memory.read_byte(0xFF44));

        let mut ly = memory.read_byte(0xFF44);
        self.cycles += cycles as u64;

        if self.cycles >= 456 {
            // End of scanline
            self.cycles -= 456;

            // Draw a line of background
            // println!("Draw line {ly}");
            let lcdc = memory.read_byte(0xFF40);
            if lcdc & 0b0000_0001 != 0 {
                // Address where tile data start
                let tile_data_start = if lcdc & 0b0001_0000 != 0 {
                    0x8000 as u16
                } else {
                    0x9000 as u16
                };
                // if lcdc & 0b0001_0000 != 0 {
                //     println!("Not Weird stuff {ly}");
                // } else {
                //     println!("Weird stuff {ly}");
                // }
                // Address where tile map starts
                // LCDC bit 3 indicates BG tile map: 0 = 9800–9BFF; 1 = 9C00–9FFF
                let tile_map_start = if lcdc & 0b0000_1000 != 0 {
                    0x9C00 as u16
                } else {
                    0x9800 as u16
                };

                // FF42 - SCY: Background viewport Y position
                // FF43 - SCX: Background viewport X position
                let scy = memory.read_byte(0xFF42);
                let scx = memory.read_byte(0xFF43);

                // Coordinate in the full 256x256 background
                let bg_y = scy.wrapping_add(ly);

                for x in 0..160 {
                    // Coordinate in the full 256x256 background
                    let bg_x = scx.wrapping_add(x);

                    // Tile coordinate
                    let tile_x = bg_x / 8;
                    let tile_y = bg_y / 8;
                    if tile_y as u16 * 32 + tile_x as u16 > 1023 {
                        panic!("tile_y = {tile_y}, tile_x = {tile_x}");
                    }
                    let tile_i =
                        memory.read_byte(tile_map_start + tile_y as u16 * 32 + tile_x as u16);

                    let tile_addr = if lcdc & 0b0001_0000 != 0 {
                        tile_data_start.wrapping_add(tile_i as u16 * 16)
                    } else {
                        tile_data_start.wrapping_add_signed(tile_i as i8 as i16 * 16) as u16
                    };
                    if tile_addr < 0x8000 || tile_addr > 0x97FF {
                        panic!("tile_addr = {:#06x}", tile_addr);
                    }
                    if (tile_addr - 0x8000) % 16 != 0 {
                        panic!("Unexpected tile address")
                    }
                    if x == 0 {
                        dump_tile(memory, tile_addr);
                    }

                    // Pixel coordinate in the tile - 0..8
                    let pixel_x = bg_x - (tile_x * 8);
                    let pixel_y = bg_y - (tile_y * 8);
                    // println!("bg_x = {bg_x} bg_y = {bg_y} | tile_x = {tile_x} tile_y = {tile_y} | pixel_x = {pixel_x} pixel_y = {pixel_y}");
                    let row_addr = tile_addr + pixel_y as u16 * 2;
                    let word = memory.read_word(row_addr);
                    let pixel =
                        ((word >> (15 - pixel_x)) & 0b01) | (((word >> (7 - pixel_x)) << 1) & 0b10);
                    self.pixels[ly as usize][x as usize] = pixel as u8;
                }
            } else {
                // Background disabled
                for x in 0..160 {
                    self.pixels[ly as usize][x as usize] = 0;
                }
            }

            ly = (ly + 1) % 154;
            memory.write_byte(0xFF44, ly);
        }
        if ly == 144 {
            // self.redraw_whole_background(memory);

            // if (memory.read_byte(0xFF50) != 0) {
            //     for i in 0..384 as u16 {
            //         println!("Tile {i}");
            //         dump_tile(memory, 0x8000 + 16 * i);
            //     }
            // }

            // Entering VBlank - request V-Blank interrupt
            let mut nn = memory.read_byte(0xFF0F);
            nn |= 0b0000_0001; // Bit 0
            memory.write_byte(0xFF0F, nn);
        }
    }

    pub fn redraw_whole_background(&mut self, memory: &Memory) {
        let lcdc = memory.read_byte(0xFF40);
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
                let tile_addr = if lcdc & 0b0001_0000 != 0 {
                    tile_data_start.wrapping_add(tile_i as u16 * 16)
                } else {
                    tile_data_start.wrapping_add_signed(tile_i as i8 as i16 * 16) as u16
                };
                for tile_row in 0..8 {
                    let tile_word = memory.read_word(tile_addr + tile_row * 2);
                    for tile_col in 0..8 {
                        let pixel = ((tile_word >> (15 - tile_col)) & 0b01)
                            | (((tile_word >> (7 - tile_col)) << 1) & 0b10);
                        self.full_bg[(bg_tile_row * 8 + tile_row) as usize]
                            [(bg_tile_col * 8 + tile_col) as usize] = pixel as u8;
                    }
                }
            }
        }

        // FF42 - SCY: Background viewport Y position
        // FF43 - SCX: Background viewport X position
        let scy = memory.read_byte(0xFF42);
        let scx = memory.read_byte(0xFF43);
        for y in 0..144 {
            for x in 0..160 {
                self.pixels[y][x] = self.full_bg[scy as usize + y][scx as usize + x];
            }
        }
    }
}
