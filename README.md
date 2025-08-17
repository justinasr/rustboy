# rustboy - Game Boy Emulator

A Game Boy (DMG) emulator written in Rust just for fun.

![Games](./games.gif)

## ✨ Features

- **CPU Emulation**: Complete Sharp LR35902 CPU implementation with all instructions
- **Memory Management**: Full memory map including cartridge ROM/RAM, work RAM, and I/O registers
- **Timer System**: Hardware-accurate timer implementation with DIV and TIMA registers
- **Interrupt Handling**: Complete interrupt system (V-Blank, LCD STAT, Timer, Serial, Joypad)
- **Cartridge Support**: Basic cartridge loading and memory bank controller support
- **PPU (Picture Processing Unit)**: Graphics rendering pipeline

## 🚀 Building and Running

```bash
# Build the project
cargo build --release

# Run with a ROM file
cargo run --release <rom_file.gb>
```

## 🎮 Controls

| Game Boy | Keyboard   |
| -------- | ---------- |
| D-Pad    | Arrow Keys |
| A        | S          |
| B        | A          |
| Start    | X          |
| Select   | Z          |

## 🧪 Tests

| Test                  | State |
| --------------------- | ----- |
| Blargg cpu_instrs     | ✅    |
| Blargg instr_timing   | ✅    |
| Blargg interrupt_time | ❌    |
| Blargg mem_timing     | ❌    |
| dmg-acid2             | ✅    |

The emulator is also tested with these games: Tetris, The Legend of Zelda: Link's Awakening.

## 📊 Current Status

Current state of the project:

- ✅ Complete CPU instruction set implementation
- ✅ Memory management
- ✅ Timer and interrupt systems
- ✅ PPU implementation
- ❌ Audio processing unit (APU)
- ❌ All MBC support
- ❌ Game saves
- ❌ Emulation speed

## 📚 References and Sources

### 📖 Essential Documentation

- **[Pan Docs](https://gbdev.io/pandocs/)** - The most comprehensive Game Boy technical documentation
- **[Game Boy Complete Technical Reference](https://gekkio.fi/files/gb-docs/gbctr.pdf)** - Detailed hardware reference by Gekkio
- **[DMG-01: How to Emulate a Game Boy](https://rylev.github.io/DMG-01/public/book/introduction.html)** - Emulator tutorial

### 🔧 CPU and Instructions

- **[Opcode Table](https://www.pastraiser.com/cpu/gameboy/gameboy_opcodes.html)** - Complete instruction set reference
- **[Game Boy CPU Manual](http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf)** - Official CPU documentation
- **[Game Boy Bootstrap ROM](https://gbdev.gg8.se/wiki/articles/Gameboy_Bootstrap_ROM)** - Boot sequence analysis

### ⚙️ Specific Implementation Details

- **[DAA Instruction](https://blog.ollien.com/posts/gb-daa/)** - Decimal adjust instruction explanation
- **[DAA Implementation](https://gbdev.gg8.se/wiki/articles/DAA)** - Implementation details for DAA
- **[Interrupts](https://gbdev.gg8.se/wiki/articles/Interrupts)** - Interrupt system documentation

### 🔬 Testing

- **[Blargg's Test ROMs](https://github.com/retrio/gb-test-roms)** - CPU and timing test collection
- **[DMG-ACID2](https://github.com/mattcurrie/dmg-acid2)** - Hardware accuracy test suite
