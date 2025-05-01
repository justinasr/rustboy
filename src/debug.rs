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
