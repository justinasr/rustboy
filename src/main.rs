mod cpu;
mod gameboy;
mod memory;
mod ppu;
mod utils;
mod cartridge;

use gameboy::Gameboy;

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
const SCALE: u32 = 3;

fn main() -> Result<(), Error> {
    env_logger::init();
    let event_loop = EventLoop::new().unwrap();
    let mut input = WinitInputHelper::new();
    // Main Window
    let window = {
        let size = LogicalSize::new((WIDTH * SCALE) as f64, (HEIGHT * SCALE) as f64);
        WindowBuilder::new()
            .with_title("rustboy")
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

    let args: Vec<String> = std::env::args().collect();
    let mut gameboy = Gameboy::new(&args[1]);
    let mut paused = false;

    let res = event_loop.run(|event, elwt| {
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
                    log_error("pixels.render", err);
                    elwt.exit();
                    return;
                }
            } else {
                panic!();
            }
        }

        if let Event::WindowEvent {
            event: WindowEvent::Resized(size),
            window_id,
        } = event
        {
            if window_id == window.id() {
                if let Err(err) = pixels.resize_surface(size.width, size.height) {
                    log_error("pixels.resize_surface", err);
                    elwt.exit();
                    return;
                }
            } else {
                panic!();
            }
        }

        // Handle input events
        if input.update(&event) {
            // Close events
            if input.key_pressed(KeyCode::Escape) || input.close_requested() {
                elwt.exit();
                return;
            } else if input.key_pressed(KeyCode::Space) {
                paused = !paused;
            }

            // Update internal state and request a redraw
            if !paused {
                for _ in 0..10000 {
                    let mut input_value = gameboy.memory.read_byte(0xFF00);
                    if input_value & 0x10 == 0 {
                        // DPad
                        if input.key_held(KeyCode::ArrowRight) {
                            input_value &= !1;
                        } else {
                            input_value |= 1;
                        }
                        if input.key_held(KeyCode::ArrowLeft) {
                            input_value &= !2;
                        } else {
                            input_value |= 2;
                        }
                        if input.key_held(KeyCode::ArrowUp) {
                            input_value &= !4;
                        } else {
                            input_value |= 4;
                        }
                        if input.key_held(KeyCode::ArrowDown) {
                            input_value &= !8;
                        } else {
                            input_value |= 8;
                        }
                    } else if input_value & 0x20 == 0 {
                        // Select buttons
                        if input.key_held(KeyCode::KeyS) {
                            input_value &= !1;
                        } else {
                            input_value |= 1;
                        }
                        if input.key_held(KeyCode::KeyA) {
                            input_value &= !2;
                        } else {
                            input_value |= 2;
                        }
                        if input.key_held(KeyCode::KeyX) {
                            input_value &= !4;
                        } else {
                            input_value |= 4;
                        }
                        if input.key_held(KeyCode::KeyZ) {
                            input_value &= !8;
                        } else {
                            input_value |= 8;
                        }
                    } else {
                        input_value = (input_value & 0xF0) | 0x0F;
                    }
                    gameboy.memory.write_byte_(0xFF00, input_value);

                    gameboy.tick();
                }
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
