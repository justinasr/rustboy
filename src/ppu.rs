use super::memory::Memory;

pub struct PPU {
    cycles: u64,
    pixels: Vec<Vec<u8>>,
    color_index: Vec<Vec<u8>>,
    mode: u8,
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            cycles: 0,
            pixels: vec![vec![0; 160]; 154],
            color_index: vec![vec![0; 160]; 154],
            mode: 0,
        }
    }

    pub fn get_pixel(&self, x: u8, y: u8) -> [u8; 4] {
        match self.pixels[y as usize][x as usize] {
            0 => [255, 255, 255, 255],
            1 => [160, 160, 160, 255],
            2 => [80, 80, 80, 255],
            3 => [0, 0, 0, 255],
            _ => unreachable!("Invalid pixel {}", self.pixels[y as usize][x as usize]),
        }
    }

    /** Get x, y pixel of tile starting at tile_addr */
    fn get_tile_pixel(&self, tile_addr: u16, memory: &Memory, x: u8, y: u8) -> u8 {
        // let row_addr = tile_addr + y as u16 * 2;
        // let word = memory.read_word(row_addr);
        // let pixel = ((word >> (15 - x)) & 0b01) | (((word >> (7 - x)) << 1) & 0b10);
        // pixel as u8
        let row_addr = tile_addr + y as u16 * 2;
        let low = memory.read_byte(row_addr);
        let high = memory.read_byte(row_addr + 1);
        let mask = 1 << (7 - x);
        let lo_bit = (low & mask) >> (7 - x);
        let hi_bit = (high & mask) >> (7 - x);
        (hi_bit << 1) | lo_bit
    }

    fn draw_line_of_background(&mut self, memory: &Memory, lcdc: u8, ly: u8) {
        if lcdc & 0b0000_0001 != 0 {
            // println!("Drawing line {ly} of background, lcdc = {:#010b}", lcdc);
            // Background enabled
            // Address where tile data start
            let tile_data_start = if lcdc & 0b0001_0000 == 0 {
                0x9000 as u16
            } else {
                0x8000 as u16
            };
            // Address where tile map starts
            // LCDC bit 3 indicates BG tile map: 0 = 9800–9BFF; 1 = 9C00–9FFF
            let tile_map_start = if lcdc & 0b0000_1000 == 0 {
                0x9800 as u16
            } else {
                0x9C00 as u16
            };

            // FF42 - SCY: Background viewport Y position
            // FF43 - SCX: Background viewport X position
            let scy = memory.read_byte(0xFF42);
            let scx = memory.read_byte(0xFF43);

            // Coordinate in the full 256x256 background
            let bg_y = scy.wrapping_add(ly);

            // Palette
            let palette = memory.read_byte(0xFF47);

            for lx in 0..160 {
                // Coordinate in the full 256x256 background
                let bg_x = scx.wrapping_add(lx);

                // Tile coordinate
                let tile_x = bg_x / 8;
                let tile_y = bg_y / 8;
                // Tile id read from tile map
                let tile_i = memory.read_byte(tile_map_start + tile_y as u16 * 32 + tile_x as u16);
                // Tile data address based on tile id
                let tile_addr = if lcdc & 0b0001_0000 == 0 {
                    tile_data_start.wrapping_add_signed(tile_i as i8 as i16 * 16) as u16
                } else {
                    tile_data_start.wrapping_add(tile_i as u16 * 16)
                };
                if tile_addr < 0x8000 || tile_addr > 0x97FF || (tile_addr - 0x8000) % 16 != 0 {
                    panic!("Unexpected tile address {:#06x}", tile_addr)
                }
                // Pixel coordinate in the tile - 0..8
                let pixel_x = bg_x - (tile_x * 8);
                let pixel_y = bg_y - (tile_y * 8);
                // Color index in palette
                let pixel_i = self.get_tile_pixel(tile_addr, memory, pixel_x, pixel_y) as u8;
                let pixel = (palette >> (pixel_i * 2)) & 0x03;
                self.pixels[ly as usize][lx as usize] = pixel;
                self.color_index[ly as usize][lx as usize] = pixel_i;
            }
        } else {
            // Background disabled
            for x in 0..160 {
                self.pixels[ly as usize][x as usize] = 0;
            }
        }
    }

    fn draw_line_of_objects(&mut self, memory: &Memory, lcdc: u8, ly: u8) {
        let obj_enable = lcdc & 0x01 != 0;
        if !obj_enable {
            return;
        }
        let tile_height = if lcdc & 0x04 != 0 { 16 } else { 8 };
        let tile_size = 16; // Tile size in bytes
        let tile_i_mask = if tile_height == 8 { 0xFF } else { 0xFE };
        let mut count = 0;
        for i in 0..40 as u16 {
            // OAM object address. Each object is 4 bytes.
            let addr = 0xFE00 + i * 4;
            // Object y starts at -16 and ends at (144+16) = 160.
            // Object height can be either 8 or 16 pixels.
            // Screen "starts" at y=16 and ends at y=160.
            // Check if scanline ly is within the object.
            let y = memory.read_byte(addr);
            if y == 0 {
                continue;
            }
            if y <= ly + 16 && ly + 16 < y + tile_height {
                count += 1;
                // Tile x on screen.
                let x = memory.read_byte(addr + 1);
                if x == 0 || x >= 168 {
                    continue;
                }
                // Tile index in tile data memory.
                let tile_i = memory.read_byte(addr + 2) & tile_i_mask;
                // Tile flags
                let flags = memory.read_byte(addr + 3);
                let dmg_palette = (flags >> 4) & 0x01; // 0 or 1
                let x_flip = flags & 0x20 != 0;
                let y_flip = flags & 0x40 != 0;
                let priority = flags & 0x80 != 0;
                // Tile start address. If tile size is 8x16, pick correct 8x8 tile.
                let tile_addr = 0x8000 + tile_i as u16 * tile_size as u16;
                // Y coordinate in tile, regardless of Y flip - 0..16.
                let _tile_y = ly - (y - 16);
                // Y coordinate in tile, taking in consideration Y flip.
                let tile_y = if y_flip {
                    tile_height - 1 - _tile_y
                } else {
                    _tile_y
                };
                // Palette
                // 0xFF48 or 0xFF49
                let palette = memory.read_byte(0xFF48 + dmg_palette as u16);
                // Objects are 8 pixels wide.
                for xx in 0..8 {
                    // Screen x "starts" at 8 and ends at 168.
                    if 8 <= x + xx && x + xx < 168 {
                        // X coordinate in tile.
                        let tile_x = if x_flip { 7 - xx } else { xx };
                        // Color index in palette 0..3
                        let pixel_i = self.get_tile_pixel(tile_addr, memory, tile_x, tile_y);
                        let lx = x + xx - 8;

                        if pixel_i != 0
                            && (!priority || self.color_index[ly as usize][lx as usize] == 0)
                        {
                            let pixel = (palette >> (pixel_i * 2)) & 0x03;
                            self.pixels[ly as usize][lx as usize] = pixel;
                            self.color_index[ly as usize][lx as usize] = pixel_i;
                        }
                    }
                }
            }
            // At most 10 sprites can be drawn in one scanline.
            if count == 10 {
                break;
            }
        }
    }

    pub fn tick(&mut self, memory: &mut Memory, cycles: u8) {
        // 0xFF40 - LCDC: LCD control
        let lcdc = memory.read_byte(0xFF40);
        // 0xFF41 - STAT: LCD status
        let mut stat = memory.read_byte(0xFF41);
        // 0xFF44 - LY: LCD Y coordinate
        let mut ly = memory.read_byte(0xFF44);
        // 0xFF45 - LYC: LY compare
        let lyc = memory.read_byte(0xFF45);

        self.cycles += cycles as u64;

        // // println!(
        //     "[PPU] Cycles +{}={} | LCDC {:#010b} | STAT {:#010b} | LY {} | LYC {} | Mode {}",
        //     cycles, self.cycles, lcdc, stat, ly, lyc, self.mode
        // );

        // Mode Value   Meaning                     Duration (approx)
        // 0    00      HBlank (Horizontal Blank)   ~204 clock cycles
        // 1    01      VBlank (Vertical Blank)     ~4560 clock cycles (10 lines × 456)
        // 2    10      OAM Search (Scanline setup) ~80 clock cycles
        // 3    11      Pixel Transfer (drawing)    ~172 clock cycles
        if self.cycles <= 80 {
            if ly < 144 {
                // OAM Search
                if self.mode != 2 {
                    stat = (stat & 0b1111_1100) | 2;
                    memory.write_byte(0xFF41, stat);
                    if stat & 0b0010_0000 != 0 {
                        // println!("Will change to mode 2 - Request STAT interrupt");
                        // Mode 2 - Request STAT interrupt
                        let mut nn = memory.read_byte(0xFF0F);
                        nn |= 0b0000_0010; // Bit 1
                        memory.write_byte(0xFF0F, nn);
                    }
                    // println!("Setting mode to 2");
                    self.mode = 2;
                }
            }
        } else if self.cycles < 252 {
            if ly < 144 {
                // Pixel Transfer
                if self.mode != 3 {
                    stat = (stat & 0b1111_1100) | 3;
                    memory.write_byte(0xFF41, stat);
                    // println!("Setting mode to 3");
                    self.mode = 3;
                }
            }
        } else if self.cycles < 456 {
            if ly < 144 {
                // HBlank
                if self.mode != 0 {
                    stat = stat & 0b1111_1100;
                    memory.write_byte(0xFF41, stat);
                    if stat & 0b0000_1000 != 0 {
                        // println!("Will change to mode 0 - Request STAT interrupt");
                        // Mode 0 - Request STAT interrupt
                        let mut nn = memory.read_byte(0xFF0F);
                        nn |= 0b0000_0010; // Bit 1
                        memory.write_byte(0xFF0F, nn);
                    }
                    // println!("Setting mode to 0");
                    self.mode = 0;
                }
            }
        } else {
            // End of scanline
            self.cycles -= 456;
            // Draw a line of background
            if ly < 144 {
                self.draw_line_of_background(memory, lcdc, ly);
                self.draw_line_of_objects(memory, lcdc, ly);
            }
            ly = (ly + 1) % 154;
            memory.write_byte(0xFF44, ly);
            if ly == 144 {
                // VBlank
                if self.mode != 1 {
                    stat = (stat & 0b1111_1100) | 1;
                    memory.write_byte(0xFF41, stat);
                    if stat & 0b0001_0000 != 0 {
                        // println!("Will change to mode 1 - Request STAT interrupt");
                        // Mode 1 - Request STAT interrupt
                        let mut nn = memory.read_byte(0xFF0F);
                        nn |= 0b0000_0010; // Bit 1
                        memory.write_byte(0xFF0F, nn);
                    }
                    // println!("Request VBlank interrupt");
                    // Mode 1 - Request VBlank interrupt
                    let mut nn = memory.read_byte(0xFF0F);
                    nn |= 0b0000_0001; // Bit 0
                    memory.write_byte(0xFF0F, nn);
                    // println!("Setting mode to 1");
                    self.mode = 1;
                }
            }
            if ly == lyc {
                let mut nn = memory.read_byte(0xFF41);
                nn |= 0b0000_0100; // Bit 2
                memory.write_byte(0xFF41, nn);
                if stat & 0b0100_0000 != 0 {
                    // println!("LY == LYC - Request STAT interrupt");
                    // LY == LYC - Request STAT interrupt
                    let mut nn = memory.read_byte(0xFF0F);
                    nn |= 0b0000_0010; // Bit 1
                    memory.write_byte(0xFF0F, nn);
                }
            } else {
                let mut nn = memory.read_byte(0xFF41);
                nn &= 0b1111_1011; // Bit 2
                memory.write_byte(0xFF41, nn);
            }
        }
    }
}
