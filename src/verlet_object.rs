use cgmath::{Vector2 as Vec2, InnerSpace};
use std::collections::HashMap;
use rayon::prelude::*;

#[derive(PartialEq)]
pub struct VerletObject {
    pub position_current: Vec2<f32>,
    pub position_old: Vec2<f32>,
    pub acceleration: Vec2<f32>,
    pub radius: f32,
    pub col: (u8, u8, u8)
}

pub struct Solver {
    pub gravity: Vec2<f32>,
    pub width: i32,
    pub height: i32
}

impl VerletObject {
    pub fn new(position_current: Vec2<f32>, position_old: Vec2<f32>, acceleration: Vec2<f32>, radius: f32, col: (u8, u8, u8)) -> Self {
        Self { position_current, position_old, acceleration, radius, col}
    }

    pub fn update_position(&mut self, dt: f32) {
        let velocity: Vec2<f32> = self.position_current - self.position_old;
        self.position_old = self.position_current;
        self.position_current = self.position_current + velocity + self.acceleration * dt * dt;

        self.acceleration.x = 0.0;
        self.acceleration.y = 0.0;
    }

    pub fn accelerate(&mut self, acc: Vec2<f32>) {
        self.acceleration += acc;    
    }
}
 
impl Solver {
    pub fn new(gravity: Vec2<f32>, width: i32, height: i32) -> Self {
        Self { gravity, width, height}
    }

    pub fn apply_arbituary_force(&mut self, particles: &mut Vec<VerletObject>, force_vector: Vec2<f32>) {
        particles.par_iter_mut()
            .for_each(|p| {
                p.position_current += force_vector;
            });
    }
        
    fn apply_gravity(&mut self, particles: &mut Vec<VerletObject>) {
        particles.par_iter_mut()
            .for_each(|p| {
                p.accelerate(self.gravity); 
            });
    }

    fn update_positions(&mut self, particles: &mut Vec<VerletObject>, dt: f32) {
        particles.par_iter_mut()
            .for_each(|p| {
                p.update_position(dt); 
            });
    }

    fn apply_constraint(&mut self, particles: &mut Vec<VerletObject>) {
        let w = self.width as f32;
        let h = self.height as f32;
        let restitution = 0.3;   
        let friction = 1.0; // 1.0 is perfect friction

        particles.par_iter_mut()
            .for_each(|p| {
                let mut pos = p.position_current;
                let mut old = p.position_old;
                let mut v   = pos - old;          // Verlet "velocity"

                // X walls
                let mut hit_x = false;
                if pos.x > w - p.radius {
                    pos.x = w - p.radius;
                    v.x = -v.x * restitution;
                    hit_x = true;
                }
                if pos.x < p.radius {
                    pos.x = p.radius;
                    v.x = -v.x * restitution;
                    hit_x = true;
                }
                if hit_x { v.y *= friction; }

                // Y walls
                let mut hit_y = false;
                if pos.y > h - p.radius {
                    pos.y = h - p.radius;
                    v.y = -v.y * restitution;
                    hit_y = true;
                }
                if pos.y < p.radius {
                    pos.y = p.radius;
                    v.y = -v.y * restitution;
                    hit_y = true;
                }
                if hit_y { v.x *= friction; }

                // Preserve v_after
                old = pos - v;

                p.position_current = pos;
                p.position_old = old;
            });
    }

    fn solve_collision(&mut self, a: &mut VerletObject, b: &mut VerletObject) {
        let axis: Vec2<f32> = a.position_current - b.position_current;
        let dist = axis.magnitude();

        if dist < a.radius + b.radius {
            let n: Vec2<f32> = axis / dist;
            let delta = a.radius + b.radius - dist;
            a.position_current += 0.5 * delta * n;
            b.position_current -= 0.5 * delta * n;
        }
    }

    // fn hash_cell(&mut self, x: i32, y: i32) -> f32 {
    //     ((x as f32 * 13.8913)/(y as f32 * 0.9381) * 1000000.0) % 255.0
    // }

    fn compute_spatial_map(&mut self, particles: &mut Vec<VerletObject>, density: u32) -> HashMap<(i32, i32), Vec<i32>> {
        let mut grid: HashMap<(i32, i32), Vec<i32>> = HashMap::new();
        
        for i in 0..particles.len() {
            let p = particles.get_mut(i).unwrap(); // There will always be a particle

            let x = (p.position_current.x / density as f32).floor() as i32;
            let y = (p.position_current.y / density as f32).floor() as i32;

            // Color based on grid
            // p.col = (self.hash_cell(x+1, y+1) as u8, (self.hash_cell(x/2, y*2) + 100.0) as u8, self.hash_cell(y+1, x+1) as u8);

            let arr = grid.get_mut(&(x,y));
            
            match arr {
                Some(v) => v.push(i as i32),
                None => {
                    let mut new_arr: Vec<i32> = Vec::new();
                    new_arr.push(i as i32);
                    grid.insert((x, y), new_arr);
                }
            }
        }
        grid
    }

    fn find_colllisions(&mut self, particles: &mut Vec<VerletObject>, density: u32) {
        let grid = self.compute_spatial_map(particles, density);

        for (&(x, y), cell_particles) in &grid {
            for dx in (-1i32)..=1 {
                for dy in (-1i32)..=1 {
                    if dx < 0 || (dx == 0 && dy < 0) { continue; }

                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx >= 0 && ny >= 0 {
                        if let Some(neighbor_cell_particles) = grid.get(&(nx, ny)) {
                            self.check_cells_collisions(particles, cell_particles, neighbor_cell_particles);
                        }
                    }
                }
            }
        }
    }

    fn check_cells_collisions(&mut self, particles: &mut Vec<VerletObject>, cell_1: &Vec<i32>, cell_2: &Vec<i32>) {
        for p1 in cell_1 {
            for p2 in cell_2 {
                if p1 == p2 { continue };
                if p1 < p2 {
                    let (a, b) = particles.split_at_mut(*p2 as usize);
                    self.solve_collision(&mut a[*p1 as usize], &mut b[0]);
                } else {
                    let (a, b) = particles.split_at_mut(*p1 as usize);
                    self.solve_collision(&mut b[0], &mut a[*p2 as usize]);
                }
            }
        }
    }

    pub fn update(&mut self, particles: &mut Vec<VerletObject>, dt: f32, density: u32) {
        let substeps = 8;
        
        for _ in 0..substeps {
            self.apply_gravity(particles);
            self.update_positions(particles, dt / (substeps as f32));
            self.find_colllisions(particles, density);
            self.apply_constraint(particles);
        }
    }
}
