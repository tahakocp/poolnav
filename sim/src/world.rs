use crate::pool::{Pool, P};
use crate::rng::Rng;
use navcore::{Config, Drive, Nav, Sensors};
use std::cell::RefCell;
use std::collections::HashMap;

thread_local! {
    static REACH: RefCell<HashMap<String, Vec<bool>>> = RefCell::new(HashMap::new());
}

/// Physical assumptions. Defaults are round numbers in the range of consumer
/// robots (about 0.25 m/s, 40 cm long), not measurements of a specific model.
#[derive(Clone, Debug)]
pub struct Params {
    pub dt: f64,
    pub top_speed: f64,
    pub track_gap: f64,
    pub body_len: f64,
    pub body_wid: f64,
    pub cell: f64,
    /// Fraction of ideal rotation a skid-steered robot really achieves.
    pub turn_efficiency: f64,
    /// Std of the per-run speed error (tracks slip on algae, slopes).
    pub speed_err: f64,
    /// Std of the gyro bias at start, deg/s.
    pub gyro_bias_dps: f64,
    pub gyro_bias_walk_dps: f64,
    pub gyro_noise_dps: f64,
    /// Rate at which pushing into a wall squares the robot to it, deg/s.
    pub align_dps: f64,
}

impl Default for Params {
    fn default() -> Self {
        Params {
            dt: 0.1,
            top_speed: 0.25,
            track_gap: 0.28,
            body_len: 0.40,
            body_wid: 0.30,
            cell: 0.1,
            turn_efficiency: 0.75,
            speed_err: 0.08,
            gyro_bias_dps: 0.5,
            gyro_bias_walk_dps: 0.01,
            gyro_noise_dps: 0.3,
            align_dps: 45.0,
        }
    }
}

pub struct World {
    pub pool: Pool,
    pub p: Params,
    pub nav: Nav,
    rng: Rng,
    pub x: f64,
    pub y: f64,
    pub th: f64,
    pub t: f64,
    pub blocked_steps: u32,
    pub path_len: f64,
    pub grid_w: usize,
    pub grid_h: usize,
    origin: P,
    /// 0 = outside the pool, 1 = dirty, 2 = clean.
    pub grid: Vec<u8>,
    pub total: usize,
    pub clean: usize,
    speed_gain: [f64; 2],
    bias: f64,
    last_blocked: bool,
    last_rate: f64,
}

impl World {
    pub fn new(pool: Pool, cfg: Config, seed: u64, p: Params) -> World {
        let mut rng = Rng::new(seed);
        let (lo, hi) = pool.bounds();
        let grid_w = ((hi.0 - lo.0) / p.cell).ceil() as usize;
        let grid_h = ((hi.1 - lo.1) / p.cell).ceil() as usize;
        let mut grid = vec![0u8; grid_w * grid_h];
        let mut total = 0;
        let reach = Self::reachable(&pool, &p, lo, grid_w, grid_h);
        for (k, &r) in reach.iter().enumerate() {
            if r {
                grid[k] = 1;
                total += 1;
            }
        }
        // Robots are dropped in by hand, anywhere clear of the walls.
        let (mut x, mut y, mut th);
        loop {
            x = rng.range(lo.0, hi.0);
            y = rng.range(lo.1, hi.1);
            th = rng.range(0.0, std::f64::consts::TAU);
            if Self::fits(&pool, &p, x, y, th) {
                break;
            }
        }
        let speed_gain = [
            1.0 + p.speed_err * rng.normal(),
            1.0 + p.speed_err * rng.normal(),
        ];
        let bias = (p.gyro_bias_dps * rng.normal()).to_radians();
        let nav = Nav::new(cfg, (rng.next_u64() >> 32) as u32);
        let mut w = World {
            pool,
            p,
            nav,
            rng,
            x,
            y,
            th,
            t: 0.0,
            blocked_steps: 0,
            path_len: 0.0,
            grid_w,
            grid_h,
            origin: lo,
            grid,
            total,
            clean: 0,
            speed_gain,
            bias,
            last_blocked: false,
            last_rate: 0.0,
        };
        w.paint();
        w
    }

    pub fn origin(&self) -> P {
        self.origin
    }

    fn body_points(p: &Params, x: f64, y: f64, th: f64) -> Vec<P> {
        let (c, s) = (th.cos(), th.sin());
        let (hl, hw) = (p.body_len / 2.0, p.body_wid / 2.0);
        let mut pts = Vec::new();
        let step = 0.08;
        let nl = (p.body_len / step).ceil() as i32;
        let nw = (p.body_wid / step).ceil() as i32;
        for i in 0..=nl {
            let u = -hl + p.body_len * i as f64 / nl as f64;
            pts.push((u, -hw));
            pts.push((u, hw));
        }
        for j in 1..nw {
            let v = -hw + p.body_wid * j as f64 / nw as f64;
            pts.push((-hl, v));
            pts.push((hl, v));
        }
        pts.into_iter()
            .map(|(u, v)| (x + u * c - v * s, y + u * s + v * c))
            .collect()
    }

    /// A pose that clips a wall slightly is pushed back out by the wall, the way
    /// a turning body shoves itself off a side wall. None if it would need a big shove.
    fn pushed_out(pool: &Pool, p: &Params, mut x: f64, mut y: f64, th: f64) -> Option<(f64, f64)> {
        let (x0, y0) = (x, y);
        for _ in 0..6 {
            let bad = Self::body_points(p, x, y, th)
                .into_iter()
                .find(|&q| !pool.contains(q));
            let Some(q) = bad else { return Some((x, y)) };
            let n = pool.wall_inward_normal(q);
            x += n.0 * 0.015;
            y += n.1 * 0.015;
            if (x - x0).hypot(y - y0) > 0.06 {
                return None;
            }
        }
        Self::fits(pool, p, x, y, th).then_some((x, y))
    }

    fn fits(pool: &Pool, p: &Params, x: f64, y: f64, th: f64) -> bool {
        Self::body_points(p, x, y, th)
            .into_iter()
            .all(|q| pool.contains(q))
    }

    /// Calls `f` with the grid index of every cell whose centre is under the body.
    fn stamp(
        p: &Params,
        dims: (P, usize, usize),
        x: f64,
        y: f64,
        th: f64,
        mut f: impl FnMut(usize),
    ) {
        let (origin, gw, gh) = dims;
        let (c, s) = (th.cos(), th.sin());
        let (hl, hw) = (p.body_len / 2.0, p.body_wid / 2.0);
        let r = hl.hypot(hw);
        let i0 = ((x - r - origin.0) / p.cell).floor().max(0.0) as usize;
        let i1 = (((x + r - origin.0) / p.cell).ceil() as usize).min(gw);
        let j0 = ((y - r - origin.1) / p.cell).floor().max(0.0) as usize;
        let j1 = (((y + r - origin.1) / p.cell).ceil() as usize).min(gh);
        for j in j0..j1 {
            for i in i0..i1 {
                let cx = origin.0 + (i as f64 + 0.5) * p.cell - x;
                let cy = origin.1 + (j as f64 + 0.5) * p.cell - y;
                let u = cx * c + cy * s;
                let v = -cx * s + cy * c;
                if u.abs() <= hl && v.abs() <= hw {
                    f(j * gw + i);
                }
            }
        }
    }

    fn paint(&mut self) {
        let (p, o, gw, gh) = (self.p.clone(), self.origin, self.grid_w, self.grid_h);
        let (x, y, th) = (self.x, self.y, self.th);
        let (grid, clean) = (&mut self.grid, &mut self.clean);
        Self::stamp(&p, (o, gw, gh), x, y, th, |k| {
            if grid[k] == 0 {
                return;
            }
            if grid[k] == 1 {
                grid[k] = 2;
                *clean += 1;
            }
        });
    }

    /// Cells some legal pose can sweep. Walls and steps keep a rectangular
    /// body out of tight corners, so those cells would cap every strategy.
    fn reachable(pool: &Pool, p: &Params, origin: P, gw: usize, gh: usize) -> Vec<bool> {
        let key = format!("{}-{}-{}-{}", pool.name, p.cell, p.body_len, p.body_wid);
        if let Some(v) = REACH.with(|m| m.borrow().get(&key).cloned()) {
            return v;
        }
        let mut seen = vec![false; gw * gh];
        let step = 0.04;
        let (hi_x, hi_y) = (origin.0 + gw as f64 * p.cell, origin.1 + gh as f64 * p.cell);
        let mut y = origin.1;
        while y <= hi_y {
            let mut x = origin.0;
            while x <= hi_x {
                if pool.contains((x, y)) {
                    for a in 0..12 {
                        let th = a as f64 * std::f64::consts::PI / 12.0;
                        if Self::fits(pool, p, x, y, th) {
                            Self::stamp(p, (origin, gw, gh), x, y, th, |k| seen[k] = true);
                        }
                    }
                }
                x += step;
            }
            y += step;
        }
        REACH.with(|m| m.borrow_mut().insert(key, seen.clone()));
        seen
    }

    pub fn coverage(&self) -> f64 {
        self.clean as f64 / self.total as f64
    }

    /// Area swept by the body divided by area cleaned: 1.0 means no overlap.
    pub fn overlap(&self) -> f64 {
        let swept = self.path_len * self.p.body_wid;
        swept / (self.clean.max(1) as f64 * self.p.cell * self.p.cell)
    }

    pub fn step(&mut self) {
        let p = self.p.clone();
        let dt = p.dt;
        self.bias += (p.gyro_bias_walk_dps * dt.sqrt()).to_radians() * self.rng.normal();

        // Controller sees the last tick's outcome, like a real control loop.
        let blocked = self.last_blocked;
        let base = 0.22 + 0.04 * self.rng.normal();
        let cur = if blocked {
            0.95 + 0.05 * self.rng.normal()
        } else {
            base
        };
        let gyro_true = self.last_rate;
        let sensors = Sensors {
            gyro_z: (gyro_true + self.bias + (p.gyro_noise_dps).to_radians() * self.rng.normal())
                as f32,
            current_left: cur as f32,
            current_right: (cur + 0.02 * self.rng.normal()) as f32,
        };
        let d: Drive = self.nav.step(&sensors, dt as f32);

        let (l, r) = (
            d.left as f64 * self.speed_gain[0],
            d.right as f64 * self.speed_gain[1],
        );
        let v = (l + r) / 2.0 * p.top_speed;
        let w = (r - l) * p.top_speed / p.track_gap * p.turn_efficiency;
        let (c, s) = (self.th.cos(), self.th.sin());
        let nx = self.x + v * c * dt;
        let ny = self.y + v * s * dt;
        let nth = self.th + w * dt;

        let in_place = v.abs() < 0.02;
        let mut hit = false;
        let mut rate = 0.0;
        if Self::fits(&self.pool, &p, nx, ny, nth) {
            self.path_len += (v * dt).abs();
            self.x = nx;
            self.y = ny;
            self.th = nth;
            rate = w;
        } else if let Some((px, py)) =
            Self::pushed_out(&self.pool, &p, nx, ny, nth).filter(|_| in_place)
        {
            self.x = px;
            self.y = py;
            self.th = nth;
            rate = w;
        } else if in_place {
            hit = w.abs() > 1e-6 || v.abs() > 1e-6;
        } else if Self::fits(&self.pool, &p, nx, ny, self.th) {
            // The turn part would clip a wall, keep driving straight.
            self.path_len += (v * dt).abs();
            self.x = nx;
            self.y = ny;
        } else {
            // Scraping along a wall with the side of the body deflects the robot.
            let front_free = v > 0.0
                && self.pool.contains((
                    nx + (p.body_len / 2.0 + 0.02) * c,
                    ny + (p.body_len / 2.0 + 0.02) * s,
                ));
            let deflect = if front_free {
                [0.3f64, -0.3, 0.6, -0.6]
                    .iter()
                    .map(|d| self.th + d.to_radians())
                    .find(|&t2| Self::fits(&self.pool, &p, nx, ny, t2))
            } else {
                None
            };
            if let Some(t2) = deflect {
                self.path_len += (v * dt).abs();
                rate = (t2 - self.th) / dt;
                self.x = nx;
                self.y = ny;
                self.th = t2;
            } else {
                hit = true;
                if v > 0.0 {
                    let front = (
                        self.x + (p.body_len / 2.0 + 0.04) * c,
                        self.y + (p.body_len / 2.0 + 0.04) * s,
                    );
                    let n = self.pool.wall_inward_normal(front);
                    let want = (-n.1).atan2(-n.0);
                    let mut dl = want - self.th;
                    dl = (dl + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                        - std::f64::consts::PI;
                    let a = p.align_dps.to_radians() * dt;
                    let rot = dl.clamp(-a, a);
                    if Self::fits(&self.pool, &p, self.x, self.y, self.th + rot) {
                        self.th += rot;
                        rate = rot / dt;
                    }
                }
            }
        }
        if hit {
            self.blocked_steps += 1;
        }
        self.last_blocked = hit;
        self.last_rate = rate;
        self.t += dt;
        self.paint();
    }
}
