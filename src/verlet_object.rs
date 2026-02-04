use cgmath::{InnerSpace, Vector2 as Vec2};
use rayon::prelude::*;
use std::collections::HashMap;
use image::DynamicImage;
use rand::Rng;
use std::cmp;

#[derive(PartialEq)]
pub struct VerletObject {
    pub position_current: Vec2<f32>,
    pub position_old: Vec2<f32>,
    pub acceleration: Vec2<f32>,
    pub radius: f32,
    pub col: (u8, u8, u8, u8),
    pub rigid: bool,
    pub target_position: Option<Vec2<f32>>,
}

pub struct Solver {
    pub cohesion_multiplier: f32,
    pub repulsion_multiplier: f32,
    pub width: i32,
    pub height: i32,
    pub substeps: i32,
}

struct Cell {
    pub color: bool,
    pub items: Vec<i32>,
}

fn hue_to_rgb(hue: f32) -> (u8, u8, u8, u8) {
    let h = (hue % 360.0) / 60.0;
    let c = 1.0;
    let x = 1.0 - ((h % 2.0) - 1.0).abs();
    let (r, g, b) = match h as u32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    ((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8, 255)
}

impl Cell {
    pub fn new(color: bool, items: Vec<i32>) -> Self {
        Self { color, items }
    }
}

impl VerletObject {
    pub fn new(
        position_current: Vec2<f32>,
        position_old: Vec2<f32>,
        acceleration: Vec2<f32>,
        radius: f32,
        col: (u8, u8, u8, u8),
        rigid: bool,
    ) -> Self {
        Self {
            position_current,
            position_old,
            acceleration,
            radius,
            col,
            rigid,
            target_position: None,
        }
    }

    pub fn update_position(&mut self, dt: f32) {
        if self.rigid {
            return;
        }
        let velocity: Vec2<f32> = self.position_current - self.position_old;
        self.position_old = self.position_current;
        self.position_current = self.position_current + velocity + self.acceleration * dt * dt;

        let hue = hue_to_rgb(240.0 - velocity.magnitude() / 3.0 * 240.0);
        self.col = hue;

        if velocity.magnitude() > 2.5 {
            self.col = (0, 0, 0, 0);
        }

        self.acceleration.x = 0.0;
        self.acceleration.y = 0.0;
    }
}

impl Solver {
    pub fn new(
        width: i32,
        height: i32,
        substeps: i32,
        cohesion_multiplier: f32,
        repulsion_multiplier: f32,
    ) -> Self {
        Self {
            width,
            height,
            substeps,
            cohesion_multiplier,
            repulsion_multiplier,
        }
    }

    pub fn apply_arbituary_force(
        &mut self,
        particles: &mut Vec<VerletObject>,
        force_vector: Vec2<f32>,
    ) {
        particles.par_iter_mut().for_each(|p| {
            p.position_current += force_vector;
        });
    }

    // Build a list of white pixel coordinates for respawning
    fn get_white_pixels(&self, image: &DynamicImage) -> Vec<(i32, i32)> {
        let rgb_image = image.to_rgb8();
        let img_width = rgb_image.width() as i32;
        let img_height = rgb_image.height() as i32;
        let mut white_pixels = Vec::new();
        
        for y in 0..img_height {
            for x in 0..img_width {
                let pixel = rgb_image.get_pixel(x as u32, y as u32).0;
                if pixel[0] >= 100 && pixel[1] >= 100 && pixel[2] >= 100 {
                    white_pixels.push((x, y));
                }
            }
        }
        
        white_pixels
    }

    fn manage_particles(&self, particles: &mut Vec<VerletObject>, white_count: i32) {
        let mut rng = rand::rng();
        let max_particles = 4000;

        if white_count == 0 {
            return
        }

        if particles.is_empty() {
            let pos_x = rng.random_range(0..800);
            let pos_y = rng.random_range(0..800);

            let pos = Vec2::new(pos_x as f32, pos_y as f32);
            particles.push(VerletObject::new(pos, pos, Vec2::new(0.0, 0.0), particles[0].radius, (0, 0, 0, 0), false));
        }

        let x = cmp::min((white_count) / 100, max_particles);

        if particles.len() > x as usize {
            while particles.len() > x as usize {
                particles.pop();
            }
        } else {
            while particles.len() < x as usize {
                let pos_x = rng.random_range(0..800);
                let pos_y = rng.random_range(0..800);

                let pos = Vec2::new(pos_x as f32, pos_y as f32);
                particles.push(VerletObject::new(pos, pos, Vec2::new(0.0, 0.0), particles[0].radius, (0, 0, 0, 0), false));
            }
        }
        println!("{}", particles.len());
    }

    pub fn calculate_targets(
        &mut self, 
        particles: &mut Vec<VerletObject>, 
        density: u32,
        image: &DynamicImage,
    ) {
        let grid = self.compute_spatial_map(particles, density, image);

        let rgb_image = image.to_rgb8();
        let img_width = rgb_image.width() as i32;
        let img_height = rgb_image.height() as i32;
        let search_radius = (density * 3) as i32;
        
        // Build white pixel list for respawning
        let white_pixels = self.get_white_pixels(image);
        let mut rng = rand::rng();
        
        for cell in grid.values() {
            if cell.color {
                // Clear targets for particles in white cells
                for &idx in &cell.items {
                    particles[idx as usize].target_position = None;
                }
                continue;
            }
            
            for &idx in &cell.items {
                let particle = &particles[idx as usize];
                let px = particle.position_current.x as i32;
                let py = particle.position_current.y as i32;
                
                let mut nearest_dist_sq = f32::INFINITY;
                let mut target: Option<Vec2<f32>> = None;
                
                // Search for nearest white pixel
                for dy in -search_radius..=search_radius {
                    for dx in -search_radius..=search_radius {
                        let x = px + dx;
                        let y = py + dy;
                        
                        if x < 0 || x >= img_width || y < 0 || y >= img_height {
                            continue;
                        }
                        
                        let pixel = rgb_image.get_pixel(x as u32, y as u32).0;
                        if pixel[0] >= 100 && pixel[1] >= 100 && pixel[2] >= 100 {
                            let dist_sq = (dx * dx + dy * dy) as f32;
                            if dist_sq < nearest_dist_sq {
                                nearest_dist_sq = dist_sq;
                                target = Some(Vec2::new(x as f32, y as f32));
                            }
                        }
                    }
                }
                
                // If no white pixel found within search radius, respawn particle
                if target.is_none() && !white_pixels.is_empty() {
                    let random_white = white_pixels[rng.random_range(0..white_pixels.len())];
                    let particle = &mut particles[idx as usize];
                    
                    // Teleport to random white pixel
                    particle.position_current = Vec2::new(random_white.0 as f32, random_white.1 as f32);
                    particle.position_old = particle.position_current; // Reset velocity
                    particle.target_position = None;
                } else {
                    // Store the target in the particle
                    particles[idx as usize].target_position = target;
                }
            }
        }
    }

    // Move particles towards their pre-calculated targets
    pub fn move_to_targets(
        &mut self, 
        particles: &mut Vec<VerletObject>, 
        density: u32,
    ) {
        for particle in particles.iter_mut() {
            if let Some(target) = particle.target_position {
                let direction = target - particle.position_current;
                let dist = direction.magnitude();
                
                if dist > 0.01 {
                    // Move a fraction each substep
                    let move_dist = (density as f32 * 0.5 / self.substeps as f32).min(dist);
                    particle.position_current += (direction / dist) * move_dist;
                }
            }
        }
    }

    fn update_positions(&mut self, particles: &mut Vec<VerletObject>, dt: f32) {
        particles.par_iter_mut().for_each(|p| {
            p.update_position(dt);
        });
    }

    fn apply_constraint(&mut self, particles: &mut Vec<VerletObject>) {
        let w = self.width as f32;
        let h = self.height as f32;
        let restitution = 0.5;
        let friction = 1.0;

        particles.par_iter_mut().for_each(|p| {
            let mut pos = p.position_current;
            let mut old = p.position_old;
            let mut v = pos - old;

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
            if hit_x {
                v.y *= friction;
            }

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
            if hit_y {
                v.x *= friction;
            }

            old = pos - v;

            p.position_current = pos;
            p.position_old = old;
        });
    }

    fn solve_collision(&mut self, a: &mut VerletObject, b: &mut VerletObject) {
        let axis: Vec2<f32> = a.position_current - b.position_current;
        let dist = axis.magnitude();

        if dist < a.radius + b.radius - self.repulsion_multiplier {
            let n: Vec2<f32> = axis / dist;
            let delta = a.radius + b.radius - dist;
            if !a.rigid {
                a.position_current += 0.5 * delta * n;
            }
            if !b.rigid {
                b.position_current -= 0.5 * delta * n;
            }
        }
    }

    fn solve_cohesion(&mut self, a: &mut VerletObject, b: &mut VerletObject) {
        let axis: Vec2<f32> = a.position_current - b.position_current;
        let dist = axis.magnitude();
        let e = self.cohesion_multiplier * 1e-4;

        if dist > a.radius + b.radius {
            let n: Vec2<f32> = axis / dist;
            let delta = a.radius + b.radius - dist;
            if !a.rigid {
                a.position_current += e * delta * n
            }
            if !b.rigid {
                b.position_current -= e * delta * n
            }
        }
    }

    fn compute_spatial_map(
        &mut self,
        particles: &mut Vec<VerletObject>,
        density: u32,
        image: &DynamicImage,
    ) -> HashMap<(i32, i32), Cell> {
        let mut grid: HashMap<(i32, i32), Cell> = HashMap::new();
        let rgb_image = image.to_rgb8();
        let img_width = rgb_image.width() as i32;
        let img_height = rgb_image.height() as i32;

        for i in 0..particles.len() {
            let p = particles.get_mut(i).unwrap();

            let grid_x = (p.position_current.x / density as f32).floor() as i32;
            let grid_y = (p.position_current.y / density as f32).floor() as i32;

            let arr = grid.get_mut(&(grid_x, grid_y));

            match arr {
                Some(v) => {
                    v.items.push(i as i32);
                },
                None => {
                    let mut new_arr: Vec<i32> = Vec::new();
                    new_arr.push(i as i32);

                    let pixel_x = p.position_current.x as i32;
                    let pixel_y = p.position_current.y as i32;
                    
                    let is_white = if pixel_x >= 0 && pixel_x < img_width && pixel_y >= 0 && pixel_y < img_height {
                        let pixel = rgb_image.get_pixel(pixel_x as u32, pixel_y as u32).0;
                        pixel[0] >= 100 && pixel[1] >= 100 && pixel[2] >= 100
                    } else {
                        false
                    };

                    grid.insert((grid_x, grid_y), Cell::new(is_white, new_arr));
                }
            }
        }
        grid
    }

    fn find_colllisions(&mut self, particles: &mut Vec<VerletObject>, density: u32, image: &DynamicImage) {
        let grid = self.compute_spatial_map(particles, density, image);

        for (&(x, y), cell_particles) in &grid {
            for dx in (-1i32)..=1 {
                for dy in (-1i32)..=1 {
                    if dx < 0 || (dx == 0 && dy < 0) {
                        continue;
                    }

                    let nx = x + dx;
                    let ny = y + dy;
                    if nx >= 0 && ny >= 0 {
                        if let Some(neighbor_cell_particles) = grid.get(&(nx, ny)) {
                            self.check_cells_collisions(
                                particles,
                                cell_particles,
                                neighbor_cell_particles,
                            );
                        }
                    }
                }
            }
        }
    }

    fn check_cells_collisions(
        &mut self,
        particles: &mut Vec<VerletObject>,
        cell_1: &Cell,
        cell_2: &Cell,
    ) {
        for &p1 in cell_1.items.iter(){
            for &p2 in cell_2.items.iter() {
                if p1 == p2 {
                    continue;
                };
                if p1 < p2 {
                    let (a, b) = particles.split_at_mut(p2 as usize);
                    self.solve_cohesion(&mut a[p1 as usize], &mut b[0]);
                    self.solve_collision(&mut a[p1 as usize], &mut b[0]);
                } else {
                    let (a, b) = particles.split_at_mut(p1 as usize);
                    self.solve_cohesion(&mut b[0], &mut a[p2 as usize]);
                    self.solve_collision(&mut b[0], &mut a[p2 as usize]);
                }
            }
        }
    }

    pub fn update(&mut self, particles: &mut Vec<VerletObject>, dt: f32, density: u32, image: &DynamicImage) {
        self.calculate_targets(particles, density, image);
        
        for _ in 0..self.substeps {
            self.update_positions(particles, dt / (self.substeps as f32));
            self.find_colllisions(particles, density, image);
            self.apply_constraint(particles);
            self.move_to_targets(particles, density);
        }
        self.manage_particles(particles, self.get_white_pixels(image).len() as i32);
    }
}
