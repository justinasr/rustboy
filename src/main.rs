#![deny(clippy::all)]
#![forbid(unsafe_code)]

mod cpu;
mod debug;
mod memory;
mod ppu;
mod utils;

use cpu::CPU;
use memory::Memory;
use ppu::PPU;

use error_iter::ErrorIter as _;
use log::error;
use pixels::{Error, Pixels, SurfaceTexture};
use winit::dpi::LogicalSize;
use winit::event::{Event, WindowEvent};
use winit::event_loop::EventLoop;
use winit::keyboard::KeyCode;
use winit::window::WindowBuilder;
use winit_input_helper::WinitInputHelper;

const WIDTH: u32 = 160;
const HEIGHT: u32 = 144;

struct Gameboy {
    memory: Memory,
    cpu: CPU,
    ppu: PPU,
}

fn main() -> Result<(), Error> {
    env_logger::init();
    let event_loop = EventLoop::new().unwrap();
    let mut input = WinitInputHelper::new();
    let window = {
        let size = LogicalSize::new(WIDTH as f64, HEIGHT as f64);
        WindowBuilder::new()
            .with_title("Hello Pixels")
            .with_inner_size(size)
            .with_min_inner_size(size)
            .build(&event_loop)
            .unwrap()
    };

    let mut pixels = {
        let window_size = window.inner_size();
        let surface_texture = SurfaceTexture::new(window_size.width, window_size.height, &window);
        Pixels::new(WIDTH, HEIGHT, surface_texture)?
    };
    let mut gameboy = Gameboy::new();

    let res = event_loop.run(|event, elwt| {
        // Draw the current frame
        if let Event::WindowEvent {
            event: WindowEvent::RedrawRequested,
            ..
        } = event
        {
            gameboy.draw(pixels.frame_mut());
            if let Err(err) = pixels.render() {
                log_error("pixels.render", err);
                elwt.exit();
                return;
            }
        }

        // Handle input events
        if input.update(&event) {
            // Close events
            if input.key_pressed(KeyCode::Escape) || input.close_requested() {
                elwt.exit();
                return;
            }

            // Resize the window
            if let Some(size) = input.window_resized() {
                if let Err(err) = pixels.resize_surface(size.width, size.height) {
                    log_error("pixels.resize_surface", err);
                    elwt.exit();
                    return;
                }
            }

            // Update internal state and request a redraw
            for i in 0..100000 {
                gameboy.update();
            }
            window.request_redraw();
        }
    });
    res.map_err(|e| Error::UserDefined(Box::new(e)))
}

fn log_error<E: std::error::Error + 'static>(method_name: &str, err: E) {
    error!("{method_name}() failed: {err}");
    for source in err.sources().skip(1) {
        error!("  Caused by: {source}");
    }
}

impl Gameboy {
    fn new() -> Self {
        let args: Vec<String> = std::env::args().collect();

        let bootrom_path = "boot.bin";
        let bootrom: Vec<u8> = std::fs::read(bootrom_path).unwrap();
        let cartridge_path = &args[1];
        let cartridge: Vec<u8> = std::fs::read(cartridge_path).unwrap();

        // Memory
        let memory = Memory::new(bootrom, cartridge);

        // CPU
        let cpu = CPU::new();

        // PPU
        let ppu = PPU::new();
        Self { memory, cpu, ppu }
    }

    fn update(&mut self) {
        // if self.memory.read_byte(0xFF50) != 0 {
        //     dump_vram(&self.memory);
        //     panic!();
        // }

        let cycles = self.cpu.tick(&mut self.memory);
        self.ppu.tick(&mut self.memory, cycles);
    }

    fn draw(&self, frame: &mut [u8]) {
        for (i, pixel) in frame.chunks_exact_mut(4).enumerate() {
            let x = (i % WIDTH as usize) as u8;
            let y = (i / WIDTH as usize) as u8;

            let rgba = self.ppu.get_pixel(x, y);

            pixel.copy_from_slice(&rgba);
        }
    }
}
