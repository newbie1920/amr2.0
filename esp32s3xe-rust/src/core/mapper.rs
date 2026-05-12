use crate::config::*;
use std::collections::VecDeque;

pub const INSCRIBED_RADIUS: f32 = 0.10; // 10cm - robot size
pub const COST_SCALING_FACTOR: f32 = 10.0;

pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub resolution: f32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub log_odds: Vec<f32>,
    pub costmap: Vec<u8>,
}

impl OccupancyGrid {
    pub fn new() -> Self {
        let size = MAP_WIDTH * MAP_HEIGHT;
        Self {
            width: MAP_WIDTH,
            height: MAP_HEIGHT,
            resolution: MAP_RESOLUTION,
            origin_x: MAP_ORIGIN_X,
            origin_y: MAP_ORIGIN_Y,
            log_odds: vec![0.0; size],
            costmap: vec![0; size],
        }
    }

    pub fn world_to_grid(&self, x: f32, y: f32) -> (isize, isize) {
        let gx = ((x + self.origin_x) / self.resolution).floor() as isize;
        let gy = ((y + self.origin_y) / self.resolution).floor() as isize;
        (gx, gy)
    }

    pub fn grid_to_world(&self, gx: isize, gy: isize) -> (f32, f32) {
        let x = (gx as f32 + 0.5) * self.resolution - self.origin_x;
        let y = (gy as f32 + 0.5) * self.resolution - self.origin_y;
        (x, y)
    }

    pub fn in_bounds(&self, gx: isize, gy: isize) -> bool {
        gx >= 0 && gx < self.width as isize && gy >= 0 && gy < self.height as isize
    }

    pub fn update_point(&mut self, gx: isize, gy: isize, hit: bool) {
        if !self.in_bounds(gx, gy) { return; }
        let idx = (gy as usize) * self.width + (gx as usize);
        let delta = if hit { LOG_ODDS_HIT } else { LOG_ODDS_MISS };
        self.log_odds[idx] = (self.log_odds[idx] + delta).clamp(MIN_LOG_ODDS, MAX_LOG_ODDS);
    }

    pub fn is_occupied(&self, gx: isize, gy: isize) -> bool {
        if !self.in_bounds(gx, gy) { return true; }
        self.log_odds[(gy as usize) * self.width + (gx as usize)] > 0.3
    }

    pub fn get_log_odds(&self, gx: isize, gy: isize) -> f32 {
        if !self.in_bounds(gx, gy) { return 0.0; }
        self.log_odds[(gy as usize) * self.width + (gx as usize)]
    }

    pub fn cast_ray(&mut self, start_x: f32, start_y: f32, angle: f32, range: f32) {
        let (x0, y0) = self.world_to_grid(start_x, start_y);
        
        let target_x = start_x + angle.cos() * range;
        let target_y = start_y + angle.sin() * range;
        let (x1, y1) = self.world_to_grid(target_x, target_y);

        let mut x = x0;
        let mut y = y0;
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;

        loop {
            if x == x1 && y == y1 {
                if range < 12.0 {
                    self.update_point(x, y, true);
                }
                break;
            }

            self.update_point(x, y, false);

            let e2 = 2 * err;
            if e2 > -dy { err -= dy; x += sx; }
            if e2 < dx { err += dx; y += sy; }

            if !self.in_bounds(x, y) { break; }
            // Max ray length check to avoid infinite loops if x1, y1 are far
            if (x - x0).abs() + (y - y0).abs() > (range / self.resolution * 2.0) as isize { break; }
        }
    }

    pub fn inflate_obstacles(&mut self) {
        let width = self.width;
        let height = self.height;
        let size = width * height;
        
        self.costmap.fill(0);
        let mut dist_grid = vec![999.0f32; size];
        let mut queue = VecDeque::new();
        
        let inscribed_cells = (INSCRIBED_RADIUS / self.resolution).ceil() as f32;
        let inflation_radius_cells = (INFLATION_RADIUS / self.resolution).ceil() as f32;

        for y in 0..height {
            for x in 0..width {
                let idx = y * width + x;
                if self.log_odds[idx] > 0.5 {
                    self.costmap[idx] = 254; // Lethal
                    dist_grid[idx] = 0.0;
                    queue.push_back((x as isize, y as isize));
                }
            }
        }

        let dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)];

        while let Some((cx, cy)) = queue.pop_front() {
            let c_idx = (cy as usize) * width + (cx as usize);
            let parent_dist = dist_grid[c_idx];

            for (dx, dy) in dirs.iter() {
                let nx = cx + dx;
                let ny = cy + dy;
                
                if !self.in_bounds(nx, ny) { continue; }
                
                let n_idx = (ny as usize) * width + (nx as usize);
                let step_dist = if dx.abs() + dy.abs() == 2 { 1.414 } else { 1.0 };
                let new_dist = parent_dist + step_dist;

                if new_dist >= dist_grid[n_idx] || new_dist > inflation_radius_cells { continue; }
                
                dist_grid[n_idx] = new_dist;

                let dist_meters = new_dist * self.resolution;
                let cost = if new_dist <= inscribed_cells {
                    253 // Inscribed
                } else {
                    let decay_dist = dist_meters - INSCRIBED_RADIUS;
                    (252.0 * (-COST_SCALING_FACTOR * decay_dist).exp()).round() as u8
                };

                if cost > 0 && cost > self.costmap[n_idx] {
                    self.costmap[n_idx] = cost;
                    queue.push_back((nx, ny));
                }
            }
        }
    }
}
