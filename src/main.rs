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
    let mut particle_size = 10.0;
    let mut movement_dampening = 10.0;
    let mut total = 1500;
    let mut substeps = 8;
    let mut gravity = 1000.0;
    let mut cohesion = 0.1;
    let mut fall_off = 100.0;

    if args.len() >= 2 {
        particle_size = args[1]
            .parse::<f32>()
            .expect("Provide a valid int for particle size") as f32;
    }
    if args.len() >= 3 {
        movement_dampening = args[2]
            .parse::<f32>()
            .expect("Provide a valid int for window velocity dampening")
            as f32;
    }
    if args.len() >= 4 {
        total = args[3]
            .parse::<f32>()
            .expect("Provide a valid int for total particles") as i32;
    }
    if args.len() >= 5 {
        substeps = args[4]
            .parse::<f32>()
            .expect("Provide a valid int for simulation substeps") as i32;
    }
    if args.len() >= 6 {
        gravity = args[5]
            .parse::<f32>()
            .expect("Provide a valid float for gravity");
    }
    if args.len() >= 7 {
        cohesion = args[6]
            .parse::<f32>()
            .expect("Provide a valid float for cohesion multiplier");
    }

    let mut frame_number = 0;
    let mut window_pos = unsafe { ffi::GetWindowPosition() };

    let mut particles: Vec<VerletObject> = Vec::new();
    let mut solver = Solver::new(Vec2::new(0.0, gravity), WIDTH, HEIGHT, substeps, cohesion);

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

        if rl.is_mouse_button_down(raylib::consts::MouseButton::MOUSE_BUTTON_LEFT) {
            solver.apply_point_arbituary_force(
                &mut particles,
                Vec2::new(mouse_x as f32, mouse_y as f32),
                fall_off,
            );
        }

        let scroll = rl.get_mouse_wheel_move();
        fall_off += 5.0 * scroll;
        frame_number += 1;

        solver.width = rl.get_screen_width();
        solver.height = rl.get_screen_height();

        if frame_number % 5 == 0 && particles.len() <= total as usize {
            for i in 0..20 {
                let pos = (25 + i * 20) as f32;
                particles.push(VerletObject::new(
                    Vec2::new(16.0, pos),
                    Vec2::new(15.0, pos - 1.0),
                    Vec2::new(0.0, 0.0),
                    particle_size,
                    (255, 255, 255),
                ));
            }
        }

        rl.set_target_fps(60);
        solver.update(
            &mut particles,
            1.0 / rl.get_fps() as f32,
            (particle_size.powf(1.5) + 1.4) as u32,
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

        if scroll != 0.0 {
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
        }
    }
}
