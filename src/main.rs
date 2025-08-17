mod cartridge;
mod cpu;
mod gameboy;
mod memory;
mod ppu;
mod utils;

use gameboy::Gameboy;

use pixels::{Error, Pixels, SurfaceTexture};
use winit::dpi::LogicalSize;
use winit::event::{Event, WindowEvent};
use winit::event_loop::EventLoop;
use winit::keyboard::KeyCode;
use winit::window::WindowBuilder;
use winit_input_helper::WinitInputHelper;

use utils::update_r8;

const WIDTH: u32 = 160;
const HEIGHT: u32 = 144;
const SCALE: u32 = 3;

fn main() -> Result<(), Error> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Cartridge name is not provided.");
        eprintln!("Usage: {} CARTRIDGE_FILE", args[0]);
        std::process::exit(1);
    }
    let cartridge_name = &args[1];

    let event_loop = EventLoop::new().unwrap();
    let mut input = WinitInputHelper::new();
    // Main window
    let window = {
        let size = LogicalSize::new((WIDTH * SCALE) as f64, (HEIGHT * SCALE) as f64);
        WindowBuilder::new()
            .with_title(format!("rustboy | {}", cartridge_name))
            .with_inner_size(size)
            .with_min_inner_size(size)
            .build(&event_loop)
            .unwrap()
    };
    // Pixel canvas
    let mut pixels = {
        let size = window.inner_size();
        let surface_texture = SurfaceTexture::new(size.width, size.height, &window);
        Pixels::new(WIDTH, HEIGHT, surface_texture)?
    };

    // Gameboy instance
    let mut gameboy = Gameboy::new(cartridge_name);

    event_loop
        .run(|event, elwt| {
            // Draw the current frame
            if let Event::WindowEvent {
                event: WindowEvent::RedrawRequested,
                window_id,
            } = event
            {
                if window_id == window.id() {
                    for (i, pixel) in pixels.frame_mut().chunks_exact_mut(4).enumerate() {
                        let x = (i % WIDTH as usize) as u8;
                        let y = (i / WIDTH as usize) as u8;
                        let rgba = gameboy.ppu.get_pixel(x, y);

                        pixel.copy_from_slice(&rgba);
                    }

                    if let Err(err) = pixels.render() {
                        eprintln!("Error rendering pixels: {}", err);
                        elwt.exit();
                        std::process::exit(2);
                    }
                } else {
                    panic!("Unexpected window ID for redraw.");
                }
            }

            // Handle window resize and update canvas size.
            if let Event::WindowEvent {
                event: WindowEvent::Resized(size),
                window_id,
            } = event
            {
                if window_id == window.id() {
                    if let Err(err) = pixels.resize_surface(size.width, size.height) {
                        eprintln!("Error resizing surface: {}", err);
                        elwt.exit();
                        std::process::exit(2);
                    }
                } else {
                    panic!("Unexpected window ID for resize.");
                }
            }

            // Handle input events.
            if input.update(&event) {
                // Close events
                if input.key_pressed(KeyCode::Escape) || input.close_requested() {
                    elwt.exit();
                    return;
                }

                for _ in 0..10000 {
                    // All input is handled through byte at 0xFF00.
                    // If bit 4 is set to 0, lower nibble contains state of select buttons.
                    // If bit 5 is set to 0, lower nibble contains state of directional buttons.
                    // Note that pressed button is 0 while release button is 1;
                    let mut value = gameboy.memory.read_byte(0xFF00);
                    if value & 0x10 == 0 {
                        // Directional buttons
                        value = update_r8(0, value, !input.key_held(KeyCode::ArrowRight));
                        value = update_r8(1, value, !input.key_held(KeyCode::ArrowLeft));
                        value = update_r8(2, value, !input.key_held(KeyCode::ArrowUp));
                        value = update_r8(3, value, !input.key_held(KeyCode::ArrowDown));
                    } else if value & 0x20 == 0 {
                        // Select buttons
                        value = update_r8(0, value, !input.key_held(KeyCode::KeyS));
                        value = update_r8(1, value, !input.key_held(KeyCode::KeyA));
                        value = update_r8(2, value, !input.key_held(KeyCode::KeyZ));
                        value = update_r8(3, value, !input.key_held(KeyCode::KeyX));
                    } else {
                        // No selector - no buttons are pressed
                        value = (value & 0xF0) | 0x0F;
                    }
                    // Update input state
                    gameboy.memory.write_byte(0xFF00, value);

                    gameboy.tick();
                }
                window.request_redraw();
            }
        })
        .map_err(|e| Error::UserDefined(Box::new(e)))
}
