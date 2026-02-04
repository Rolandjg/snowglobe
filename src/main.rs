mod verlet_object;

use crate::verlet_object::*;
use cgmath::{InnerSpace, Vector2 as Vec2};
use clap::Parser;
use rand::Rng;
use raylib::prelude::*;

use std::fs;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Particle Size
    #[arg(short, long, default_value_t = 10)]
    particle_size: i32,

    /// Frames directory
    #[arg(short, long, default_value_t = String::from("frames"))]
    directory: String,

    /// Total (max) particles
    #[arg(short, long, default_value_t = 2000)]
    total: i32,

    /// Motion dampening
    #[arg(short, long, default_value_t = 10)]
    motion_dampening: i32,

    /// Simulation substeps
    #[arg(short, long, default_value_t = 8)]
    substeps: i32,

    /// Particle cohesion
    #[arg(short, long, default_value_t = 0.0)]
    cohesion: f32,

    /// Particle repulsion
    #[arg(short, long, default_value_t = 1.0)]
    repulsion: f32,

    /// Particle Size Variance
    #[arg(short, long, default_value_t = 0)]
    variance: i32,
}

const WIDTH: i32 = 800;
const HEIGHT: i32 = 800;

fn main() {
    let args = Args::parse();
    let (mut rl, thread) = raylib::init()
        .size(WIDTH, HEIGHT)
        .title("Digital Snowglobe")
        .resizable()
        .build();

    let audio = raylib::core::audio::RaylibAudio::init_audio_device().unwrap();

    let mut playing = true;
    let particle_size = args.particle_size as f32;
    let total = args.total;
    let movement_dampening = args.motion_dampening as f32;
    let substeps = args.substeps;
    let cohesion = args.cohesion;
    let repulsion = args.repulsion;
    let size_variance = args.variance;
    let direcory = args.directory;

    let mut rng = rand::rng();
    let mut frame_index = 0;

    let mut window_pos = unsafe { ffi::GetWindowPosition() };

    let mut particles: Vec<VerletObject> = Vec::new();
    let mut solver = Solver::new(
        WIDTH,
        HEIGHT,
        substeps,
        cohesion,
        repulsion,
    );
    solver.set_max_particles(total);

    let mut paths: Vec<PathBuf> = fs::read_dir(direcory)
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .collect();

    paths.sort();

    let mut images: Vec<image::DynamicImage> = Vec::new();

    for path in paths {
        match image::open(&path) {
            Ok(img) => {
                println!("{}", path.display());
                images.push(img.resize_exact(
                    solver.width as u32,
                    solver.height as u32,
                    image::imageops::FilterType::Nearest,
                ));
            }
            Err(e) => {
                eprintln!("Failed to open {:?}: {e}", path);
            }
        }
    }

    for _ in 0..5 {
        let x_pos = rng.random_range(0..WIDTH) as f32;
        let y_pos = rng.random_range(0..HEIGHT) as f32 ;

        particles.push(VerletObject::new(
            Vec2::new(x_pos, y_pos),
            Vec2::new(x_pos, y_pos),
            Vec2::new(0.0, 0.0),
            if size_variance != 0 {
                (particle_size + (rng.random_range(-size_variance..size_variance) as f32)).abs()
            } else {
                particle_size
            },
            (255, 255, 255, 255),
            false,
        ));
    }

    rl.set_target_fps(30);
    let music = audio.new_music("audio.wav").expect("Could not open music");
    music.play_stream();

    while !rl.window_should_close() {
        music.update_stream();
        let new_window_pos = unsafe { ffi::GetWindowPosition() };

        if window_pos.x != new_window_pos.x || window_pos.y != new_window_pos.y {
            let old = Vec2::new(window_pos.x, window_pos.y);
            let new = Vec2::new(new_window_pos.x as f32, new_window_pos.y as f32);

            let force_vector = old - new;
            let n = force_vector / force_vector.magnitude();
            solver.apply_arbituary_force(&mut particles, n / movement_dampening);
            window_pos = new_window_pos;
        }

        solver.width = rl.get_screen_width();
        solver.height = rl.get_screen_height();

        rl.set_trace_log(TraceLogLevel::LOG_NONE);

        if !images.is_empty() {
            let image = &images[(frame_index as usize) % images.len()];
            
            if playing {
                solver.update(
                    &mut particles,
                    1.0 / 30.0,
                    (particle_size.powf(1.5) + 1.4) as u32,
                    image
                );
                
                frame_index += 1; // Move to next frame
            }

            let mut d = rl.begin_drawing(&thread);
            d.clear_background(Color::BLACK);

            for p in particles.iter() {
                let col = p.col;
                d.draw_circle(
                    p.position_current.x as i32,
                    p.position_current.y as i32,
                    p.radius,
                    Color::new(col.0, col.1, col.2, col.3),
                );
            }
        } else {
            eprintln!("No images found in ./frames directory!");
            break;
        }

        unsafe {
            if raylib::ffi::IsKeyDown(KeyboardKey::KEY_P as i32) {
                playing = true;
            }

            if raylib::ffi::IsKeyDown(KeyboardKey::KEY_S as i32) {
                playing = false;
            }
        }
    }
}
