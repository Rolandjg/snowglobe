mod verlet_object;

use crate::verlet_object::*;
use cgmath::{InnerSpace, Vector2 as Vec2};
use raylib::prelude::*;
use std::env;

const WIDTH: i32 = 800;
const HEIGHT: i32 = 800;

fn main() {
    let (mut rl, thread) = raylib::init()
        .size(WIDTH, HEIGHT)
        .title("Digital Snowglobe")
        .resizable()
        .build();

    let args: Vec<String> = env::args().collect();
    let particle_size = 10.0;
    if args.len() == 2 {
        let particle_size = args[1]
            .parse::<f32>()
            .expect("Provide a valid int for particle size") as f32;
        println!("{}", particle_size);
    }

    let mut frame_number = 0;
    let mut window_pos = unsafe { ffi::GetWindowPosition() };

    let mut particles: Vec<VerletObject> = Vec::new();
    let mut solver = Solver::new(Vec2::new(0.0, 1000.0), WIDTH, HEIGHT);

    while !rl.window_should_close() {
        let new_window_pos = unsafe { ffi::GetWindowPosition() };

        if window_pos.x != new_window_pos.x || window_pos.y != new_window_pos.y {
            let old = Vec2::new(window_pos.x, window_pos.y);
            let new = Vec2::new(new_window_pos.x as f32, new_window_pos.y as f32);

            let force_vector = old - new;
            let n = force_vector / force_vector.magnitude();
            solver.apply_arbituary_force(&mut particles, n / 10.0);
            window_pos = new_window_pos;
        }

        frame_number += 1;

        solver.width = rl.get_screen_width();
        solver.height = rl.get_screen_height();

        let color = (
            (frame_number / 5 % 100) as u8,
            (frame_number / 5 % 155) as u8,
            255,
        );

        if frame_number % 5 == 0 && particles.len() <= 1500 {
            for i in 0..6 {
                let pos = (25 + i * 20) as f32;
                particles.push(VerletObject::new(
                    Vec2::new(pos, 16.0),
                    Vec2::new(pos - 1.0, 15.0),
                    Vec2::new(0.0, 0.0),
                    particle_size,
                    color,
                ));
            }
        }

        rl.set_target_fps(60);
        solver.update(
            &mut particles,
            1.0 / rl.get_fps() as f32,
            (particle_size.powf(1.5) + 1.0) as u32,
        );

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
    }
}
