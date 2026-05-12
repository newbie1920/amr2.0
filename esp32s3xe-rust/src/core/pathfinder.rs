use crate::core::mapper::OccupancyGrid;
use std::collections::{BinaryHeap, VecDeque};
use std::cmp::Ordering;

#[derive(Clone, Copy, Debug, PartialEq)]
struct Node {
    idx: usize,
    f: f32,
    g: f32,
}

impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse for min-heap
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

pub struct Pathfinder;

impl Pathfinder {
    fn heuristic(x1: isize, y1: isize, x2: isize, y2: isize) -> f32 {
        let dx = (x2 - x1).abs() as f32;
        let dy = (y2 - y1).abs() as f32;
        dx.max(dy) + 0.414 * dx.min(dy) // Octile distance
    }

    fn line_of_sight(x0: isize, y0: isize, x1: isize, y1: isize, grid: &OccupancyGrid, safety_threshold: u8) -> bool {
        let mut x = x0;
        let mut y = y0;
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;

        loop {
            if !grid.in_bounds(x, y) { return false; }
            
            let idx = (y as usize) * grid.width + (x as usize);
            if grid.costmap[idx] >= safety_threshold { return false; }
            if grid.log_odds[idx] > 0.3 { return false; }

            if x == x1 && y == y1 { break; }
            let e2 = 2 * err;
            if e2 > -dy { err -= dy; x += sx; }
            if e2 < dx { err += dx; y += sy; }
        }
        true
    }

    fn find_nearest_passable(grid: &OccupancyGrid, start_gx: isize, start_gy: isize, allow_unknown: bool) -> Option<(isize, isize)> {
        let mut queue = VecDeque::new();
        queue.push_back((start_gx, start_gy));
        
        let size = grid.width * grid.height;
        let mut visited = vec![false; size];
        if grid.in_bounds(start_gx, start_gy) {
            visited[(start_gy as usize) * grid.width + (start_gx as usize)] = true;
        }

        let mut max_iter = 500;
        let dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)];

        while let Some((cx, cy)) = queue.pop_front() {
            if max_iter == 0 { break; }
            max_iter -= 1;

            if grid.in_bounds(cx, cy) {
                let idx = (cy as usize) * grid.width + (cx as usize);
                let lo = grid.log_odds[idx];
                
                if lo < -0.3 || (allow_unknown && lo.abs() <= 0.3) {
                    if grid.costmap[idx] < 100 {
                        return Some((cx, cy));
                    }
                }
            }

            for (dx, dy) in dirs.iter() {
                let nx = cx + dx;
                let ny = cy + dy;
                if grid.in_bounds(nx, ny) {
                    let n_idx = (ny as usize) * grid.width + (nx as usize);
                    if !visited[n_idx] {
                        visited[n_idx] = true;
                        queue.push_back((nx, ny));
                    }
                }
            }
        }
        None
    }

    pub fn find_path(grid: &OccupancyGrid, start: Point, goal: Point, allow_unknown: bool) -> Option<Vec<Point>> {
        let (mut sgx, mut sgy) = grid.world_to_grid(start.x, start.y);
        let (mut egx, mut egy) = grid.world_to_grid(goal.x, goal.y);

        // Clamp to grid bounds
        sgx = sgx.clamp(0, grid.width as isize - 1);
        sgy = sgy.clamp(0, grid.height as isize - 1);
        egx = egx.clamp(0, grid.width as isize - 1);
        egy = egy.clamp(0, grid.height as isize - 1);

        // Check if goal is blocked
        let goal_idx = (egy as usize) * grid.width + (egx as usize);
        let goal_blocked = grid.log_odds[goal_idx] > 0.3 || grid.costmap[goal_idx] >= 100;
        
        if goal_blocked {
            if let Some((nx, ny)) = Self::find_nearest_passable(grid, egx, egy, allow_unknown) {
                egx = nx;
                egy = ny;
            } else {
                return None;
            }
        }

        let size = grid.width * grid.height;
        let mut open_set = BinaryHeap::new();
        let mut g_score = vec![f32::INFINITY; size];
        let mut came_from: Vec<Option<usize>> = vec![None; size];
        let mut closed = vec![false; size];

        let start_idx = (sgy as usize) * grid.width + (sgx as usize);
        
        open_set.push(Node { idx: start_idx, f: Self::heuristic(sgx, sgy, egx, egy), g: 0.0 });
        g_score[start_idx] = 0.0;

        let dirs = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ];

        let mut iterations = 0;
        let max_iterations = 15000;

        while let Some(current) = open_set.pop() {
            iterations += 1;
            if iterations > max_iterations { break; }

            let cgx = (current.idx % grid.width) as isize;
            let cgy = (current.idx / grid.width) as isize;

            if closed[current.idx] { continue; }
            closed[current.idx] = true;

            // Goal reached
            if (cgx - egx).abs() <= 1 && (cgy - egy).abs() <= 1 {
                let mut path = Vec::new();
                let mut curr_idx = current.idx;
                
                while let Some(prev_idx) = came_from[curr_idx] {
                    let cx = (curr_idx % grid.width) as isize;
                    let cy = (curr_idx / grid.width) as isize;
                    let (wx, wy) = grid.grid_to_world(cx, cy);
                    path.push(Point { x: wx, y: wy });
                    curr_idx = prev_idx;
                }
                
                let (sx, sy) = grid.grid_to_world(sgx, sgy);
                path.push(Point { x: sx, y: sy });
                path.reverse();

                let mut simplified = Self::los_shortcut(path, grid);
                simplified = Self::rdp_simplify(simplified, 0.08);
                let smoothed = Self::smooth_path(simplified, grid);
                let mut capped = Self::limit_waypoints(smoothed, 60);

                if !capped.is_empty() {
                    capped[0] = Point { x: start.x, y: start.y };
                    let goal_g = grid.world_to_grid(goal.x, goal.y);
                    if grid.in_bounds(goal_g.0, goal_g.1) {
                        let orig_idx = (goal_g.1 as usize) * grid.width + (goal_g.0 as usize);
                        if grid.costmap[orig_idx] < 100 && grid.log_odds[orig_idx] <= 0.3 {
                            *capped.last_mut().unwrap() = Point { x: goal.x, y: goal.y };
                        }
                    }
                }

                return Some(capped);
            }

            for (dx, dy, cost) in dirs.iter() {
                let nx = cgx + dx;
                let ny = cgy + dy;

                if !grid.in_bounds(nx, ny) { continue; }

                let n_idx = (ny as usize) * grid.width + (nx as usize);
                if closed[n_idx] { continue; }

                let lo = grid.log_odds[n_idx];
                if lo > 0.3 { continue; }
                if !allow_unknown && lo.abs() <= 0.3 { continue; }

                // Diagonal safety
                if *dx != 0 && *dy != 0 {
                    let lo1 = grid.get_log_odds(cgx + dx, cgy);
                    let lo2 = grid.get_log_odds(cgx, cgy + dy);
                    if lo1 > 0.3 || lo2 > 0.3 { continue; }
                }

                let cm = grid.costmap[n_idx];
                if cm >= 253 { continue; }

                let mut penalty = 0.0;
                if cm >= 100 {
                    penalty += 100000.0;
                } else if cm > 0 {
                    let norm = cm as f32 / 100.0;
                    penalty += (norm * 5.0).exp() - 1.0;
                }

                if lo.abs() <= 0.3 {
                    penalty += 3.0;
                }

                let parent_idx_opt = came_from[current.idx];
                let mut best_g = current.g + cost + penalty;
                let mut best_parent = current.idx;

                if let Some(p_idx) = parent_idx_opt {
                    let pgx = (p_idx % grid.width) as isize;
                    let pgy = (p_idx / grid.width) as isize;

                    if Self::line_of_sight(pgx, pgy, nx, ny, grid, 10) {
                        let dist = (((nx - pgx).pow(2) + (ny - pgy).pow(2)) as f32).sqrt();
                        let direct_g = g_score[p_idx] + dist + penalty * (dist / (dx.abs() as f32).max(dy.abs() as f32).max(1.0));
                        
                        if direct_g < best_g {
                            best_g = direct_g;
                            best_parent = p_idx;
                        }
                    }
                }

                if best_g < g_score[n_idx] {
                    g_score[n_idx] = best_g;
                    came_from[n_idx] = Some(best_parent);
                    open_set.push(Node {
                        idx: n_idx,
                        f: best_g + Self::heuristic(nx, ny, egx, egy),
                        g: best_g,
                    });
                }
            }
        }

        None
    }

    fn los_shortcut(path: Vec<Point>, grid: &OccupancyGrid) -> Vec<Point> {
        if path.len() <= 2 { return path; }
        let mut result = vec![path[0]];
        let mut current = 0;
        while current < path.len() - 1 {
            let mut farthest = current + 1;
            for ahead in (current + 2..path.len()).rev() {
                let (gx0, gy0) = grid.world_to_grid(path[current].x, path[current].y);
                let (gx1, gy1) = grid.world_to_grid(path[ahead].x, path[ahead].y);
                if Self::line_of_sight(gx0, gy0, gx1, gy1, grid, 10) {
                    farthest = ahead;
                    break;
                }
            }
            current = farthest;
            result.push(path[current]);
        }
        result
    }

    fn rdp_simplify(points: Vec<Point>, epsilon: f32) -> Vec<Point> {
        if points.len() <= 2 { return points; }
        let mut max_dist = 0.0;
        let mut index = 0;
        let start = points[0];
        let end = *points.last().unwrap();

        for i in 1..points.len() - 1 {
            let dist = Self::point_line_dist(points[i], start, end);
            if dist > max_dist {
                max_dist = dist;
                index = i;
            }
        }

        if max_dist > epsilon {
            let mut left = Self::rdp_simplify(points[..=index].to_vec(), epsilon);
            let right = Self::rdp_simplify(points[index..].to_vec(), epsilon);
            left.pop();
            left.extend(right);
            left
        } else {
            vec![start, end]
        }
    }

    fn point_line_dist(p: Point, a: Point, b: Point) -> f32 {
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let len2 = dx * dx + dy * dy;
        if len2 == 0.0 { return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt(); }
        let t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
        let t = t.clamp(0.0, 1.0);
        ((p.x - (a.x + t * dx)).powi(2) + (p.y - (a.y + t * dy)).powi(2)).sqrt()
    }

    fn smooth_path(path: Vec<Point>, grid: &OccupancyGrid) -> Vec<Point> {
        if path.len() <= 2 { return path; }
        let mut smoothed = path;
        for _ in 0..3 {
            let mut next = vec![smoothed[0]];
            for i in 0..smoothed.len() - 1 {
                let p0 = smoothed[i];
                let p1 = smoothed[i + 1];
                let q = Point { x: 0.75 * p0.x + 0.25 * p1.x, y: 0.75 * p0.y + 0.25 * p1.y };
                let r = Point { x: 0.25 * p0.x + 0.75 * p1.x, y: 0.25 * p0.y + 0.75 * p1.y };
                
                let (qgx, qgy) = grid.world_to_grid(q.x, q.y);
                let (rgx, rgy) = grid.world_to_grid(r.x, r.y);
                
                let mut q_blocked = false;
                let mut r_blocked = false;

                if grid.in_bounds(qgx, qgy) {
                    let cost = grid.costmap[(qgy as usize) * grid.width + (qgx as usize)];
                    if cost >= 100 { q_blocked = true; }
                }
                if grid.in_bounds(rgx, rgy) {
                    let cost = grid.costmap[(rgy as usize) * grid.width + (rgx as usize)];
                    if cost >= 100 { r_blocked = true; }
                }

                if !q_blocked && !r_blocked {
                    let mid_x = (q.x + r.x) / 2.0;
                    let mid_y = (q.y + r.y) / 2.0;
                    let (mgx, mgy) = grid.world_to_grid(mid_x, mid_y);
                    if grid.in_bounds(mgx, mgy) {
                        let cost = grid.costmap[(mgy as usize) * grid.width + (mgx as usize)];
                        if cost >= 100 { q_blocked = true; r_blocked = true; }
                    }
                }

                if q_blocked || r_blocked {
                    next.push(p1);
                } else {
                    next.push(q);
                    next.push(r);
                }
            }
            next.push(*smoothed.last().unwrap());
            smoothed = next;
        }
        smoothed
    }

    fn limit_waypoints(path: Vec<Point>, max_waypoints: usize) -> Vec<Point> {
        if path.len() <= max_waypoints { return path; }
        
        let mut scores = vec![0.0f32; path.len()];
        scores[0] = f32::INFINITY;
        scores[path.len() - 1] = f32::INFINITY;
        
        for i in 1..path.len() - 1 {
            let prev = path[i - 1];
            let curr = path[i];
            let next = path[i + 1];
            
            let dx1 = curr.x - prev.x;
            let dy1 = curr.y - prev.y;
            let dx2 = next.x - curr.x;
            let dy2 = next.y - curr.y;
            
            let len1 = (dx1 * dx1 + dy1 * dy1).sqrt();
            let len2 = (dx2 * dx2 + dy2 * dy2).sqrt();
            
            if len1 < 1e-6 || len2 < 1e-6 {
                scores[i] = 0.0;
                continue;
            }
            
            let dot = (dx1 * dx2 + dy1 * dy2) / (len1 * len2);
            scores[i] = 1.0 - dot.clamp(-1.0, 1.0);
        }
        
        let mut indexed: Vec<(usize, f32)> = scores.into_iter().enumerate().collect();
        // Sort descending by score
        indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(Ordering::Equal));
        
        let mut selected: Vec<_> = indexed.into_iter().take(max_waypoints).collect();
        // Sort ascending by index to maintain path order
        selected.sort_by_key(|a| a.0);
        
        selected.into_iter().map(|s| path[s.0]).collect()
    }
}
