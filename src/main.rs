mod verlet_object;

use crate::verlet_object::*;
use cgmath::{InnerSpace, Vector2 as Vec2};
use clap::Parser;
use rand::Rng;
use raylib::prelude::*;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Particle Size
    #[arg(short, long, default_value_t = 10)]
    particle_size: i32,

    /// Motion dampening
    #[arg(short, long, default_value_t = 10)]
    motion_dampening: i32,

    /// Total particles
    #[arg(short, long, default_value_t = 1000)]
    total: i32,

    /// Simulation substeps
    #[arg(short, long, default_value_t = 8)]
    substeps: i32,

    /// Simulation gravity
    #[arg(short, long, default_value_t = 1000)]
    gravity: i32,

    /// Particle cohesion
    #[arg(short, long, default_value_t = 0.0)]
    cohesion: f32,

    /// Particle repulsion
    #[arg(short, long, default_value_t = 0.0)]
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

    let mut playing = true;
    let particle_size = args.particle_size as f32;
    let movement_dampening = args.motion_dampening as f32;
    let total = args.total;
    let substeps = args.substeps;
    let gravity = args.gravity as f32;
    let cohesion = args.cohesion;
    let repulsion = args.repulsion;
    let size_variance = args.variance;
    let mut fall_off = 100.0;

    let mut rng = rand::rng();

    let mut window_pos = unsafe { ffi::GetWindowPosition() };

    let mut particles: Vec<VerletObject> = Vec::new();
    let mut solver = Solver::new(
        Vec2::new(0.0, gravity),
        WIDTH,
        HEIGHT,
        substeps,
        cohesion,
        repulsion,
    );

    for x in 0..((total as f32).sqrt() as i32) {
        for y in 0..((total as f32).sqrt() as i32) {
            let x_pos = (x * particle_size as i32) as f32 * 2.5;
            let y_pos = (y * particle_size as i32) as f32 * 2.5;
            particles.push(VerletObject::new(
                Vec2::new(x_pos + particle_size, y_pos + particle_size),
                Vec2::new(x_pos + particle_size, y_pos + particle_size),
                Vec2::new(0.0, 0.0),
                if size_variance != 0 {
                    (particle_size + (rng.random_range(-size_variance..size_variance) as f32)).abs()
                } else {
                    particle_size
                },
                (255, 255, 255),
                false,
            ));
        }
    }

    while !rl.window_should_close() {
        let new_window_pos = unsafe { ffi::GetWindowPosition() };
        let mouse_x = rl.get_mouse_x();
        let mouse_y = rl.get_mouse_y();

        if window_pos.x != new_window_pos.x || window_pos.y != new_window_pos.y {
            let old = Vec2::new(window_pos.x, window_pos.y);
            let new = Vec2::new(new_window_pos.x as f32, new_window_pos.y as f32);

            let force_vector = old - new;
            let n = force_vector / force_vector.magnitude();
            solver.apply_arbituary_force(&mut particles, n / movement_dampening);
            window_pos = new_window_pos;
        }

        if rl.is_mouse_button_down(raylib::consts::MouseButton::MOUSE_BUTTON_RIGHT) {
            for i in 0..(if fall_off < 0.0 { 10 } else { 1 }) {
                particles.push(VerletObject::new(
                    Vec2::new((mouse_x + i) as f32, (mouse_y + i) as f32),
                    Vec2::new((mouse_x + i) as f32, (mouse_y + i) as f32),
                    Vec2::new(0.0, 0.0),
                    if size_variance != 0 && fall_off < 0.0 {
                        (particle_size + (rng.random_range(-size_variance..size_variance) as f32))
                            .abs()
                    } else {
                        particle_size
                    },
                    (255, 255, 255),
                    fall_off > 0.0,
                ));
            }
        }
        if rl.is_mouse_button_down(raylib::consts::MouseButton::MOUSE_BUTTON_LEFT) {
            solver.apply_point_arbituary_force(
                &mut particles,
                Vec2::new(mouse_x as f32, mouse_y as f32),
                fall_off,
            );
        }

        let scroll = rl.get_mouse_wheel_move();
        fall_off += 5.0 * scroll;

        solver.width = rl.get_screen_width();
        solver.height = rl.get_screen_height();

        rl.set_target_fps(60);
        rl.set_trace_log(TraceLogLevel::LOG_NONE);
        if playing {
            solver.update(
                &mut particles,
                1.0 / 60.0 as f32,
                (particle_size.powf(1.5) + 1.4) as u32,
            );
        }

        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::BLACK);

        for p in particles.iter() {
            let col = p.col;
            d.draw_circle(
                p.position_current.x as i32,
                p.position_current.y as i32,
                p.radius,
                Color::new(col.0, col.1, col.2, 255),
            );
        }

        d.draw_circle_lines(
            mouse_x,
            mouse_y,
            fall_off,
            if fall_off > 0.0 {
                Color::GREEN
            } else {
                Color::RED
            },
        );

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
