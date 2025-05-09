mod cpu;
mod debug;
mod gameboy;
mod memory;
mod ppu;
mod utils;

use debug::*;
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

// const TILE_DATA_WIDTH: u32 = 128;
// const TILE_DATA_HEIGHT: u32 = 192;

fn main() -> Result<(), Error> {
    env_logger::init();
    let event_loop = EventLoop::new().unwrap();
    let mut input = WinitInputHelper::new();
    // Main Window
    let window = {
        let size = LogicalSize::new(WIDTH as f64 * 2.0, HEIGHT as f64 * 2.0);
        WindowBuilder::new()
            .with_title("My Gameboy Emulator")
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
    // Tile Data Window
    // let tile_data_window = {
    //     let size = LogicalSize::new(TILE_DATA_WIDTH as f64 * 2.0, TILE_DATA_HEIGHT as f64 * 2.0);
    //     WindowBuilder::new()
    //         .with_title("Tile Data")
    //         .with_inner_size(size)
    //         .with_min_inner_size(size)
    //         .build(&event_loop)
    //         .unwrap()
    // };
    // let mut tile_data_pixels = {
    //     let size = tile_data_window.inner_size();
    //     let surface_texture = SurfaceTexture::new(size.width, size.height, &tile_data_window);
    //     Pixels::new(TILE_DATA_WIDTH, TILE_DATA_HEIGHT, surface_texture)?
    // };

    let args: Vec<String> = std::env::args().collect();
    let mut gameboy = Gameboy::new(&args[1]);
    let mut paused = true;

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
            // } else if window_id == tile_data_window.id() {
            //     for (i, pixel) in tile_data_pixels.frame_mut().chunks_exact_mut(4).enumerate() {
            //         let x = (i % TILE_DATA_WIDTH as usize) as u8;
            //         let y = (i / TILE_DATA_WIDTH as usize) as u8;

            //         let tile_x = x / 8;
            //         let tile_y = y / 8;
            //         let rgba = debug_color(tile_pixel_i(
            //             &gameboy.memory,
            //             tile_addr(tile_y as u16 * 16 + tile_x as u16),
            //             x % 8,
            //             y % 8,
            //         ));

            //         pixel.copy_from_slice(&rgba);
            //     }

            //     if let Err(err) = tile_data_pixels.render() {
            //         log_error("tile_data_pixels.render", err);
            //         elwt.exit();
            //         return;
            //     }
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
            // } else if window_id == tile_data_window.id() {
            //     if let Err(err) = tile_data_pixels.resize_surface(size.width, size.height) {
            //         log_error("tile_data_pixels.resize_surface", err);
            //         elwt.exit();
            //         return;
            //     }
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
            } else if input.key_pressed(KeyCode::KeyT) {
                println!("TILES:");
                dump_tile_data(&gameboy.memory);
            } else if input.key_pressed(KeyCode::KeyV) {
                println!("VRAM");
                dump_vram(&gameboy.memory);
            } else if input.key_pressed(KeyCode::KeyB) {
                println!("BACKGROUND");
                dump_background(&gameboy.memory);
            } else if input.key_pressed(KeyCode::KeyQ) {
                gameboy.tick();
            } else if input.key_pressed(KeyCode::KeyW) {
                for _ in 0..10 {
                    gameboy.tick();
                }
            } else if input.key_pressed(KeyCode::KeyE) {
                for _ in 0..100 {
                    gameboy.tick();
                }
            } else if input.key_pressed(KeyCode::KeyR) {
                for _ in 0..1000 {
                    gameboy.tick();
                }
            }

            // Update internal state and request a redraw
            if !paused {
                for _ in 0..10000 {
                    gameboy.tick();
                }
            }
            window.request_redraw();
            // tile_data_window.request_redraw();
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
