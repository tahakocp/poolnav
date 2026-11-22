use std::f64::consts::TAU;

pub type P = (f64, f64);

#[derive(Clone)]
pub struct Pool {
    pub name: &'static str,
    pub outer: Vec<P>,
    /// Things standing in the water: ladder bases, steps cut out of the floor area.
    pub holes: Vec<Vec<P>>,
}

pub const NAMES: [&str; 4] = ["rect", "l-shape", "steps", "kidney"];

pub fn by_name(name: &str) -> Option<Pool> {
    let rect = |w: f64, h: f64| vec![(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)];
    let sq = |x: f64, y: f64, s: f64| vec![(x, y), (x + s, y), (x + s, y + s), (x, y + s)];
    match name {
        "rect" => Some(Pool {
            name: "rect",
            outer: rect(8.0, 4.0),
            holes: vec![],
        }),
        "l-shape" => Some(Pool {
            name: "l-shape",
            outer: vec![
                (0.0, 0.0),
                (8.0, 0.0),
                (8.0, 2.5),
                (4.5, 2.5),
                (4.5, 5.0),
                (0.0, 5.0),
            ],
            holes: vec![],
        }),
        // 9 x 4.5 m with a corner staircase and a ladder base near the long wall.
        "steps" => Some(Pool {
            name: "steps",
            outer: vec![
                (0.0, 0.0),
                (9.0, 0.0),
                (9.0, 4.5),
                (1.8, 4.5),
                (1.8, 3.9),
                (1.2, 3.9),
                (1.2, 3.3),
                (0.6, 3.3),
                (0.6, 2.7),
                (0.0, 2.7),
            ],
            holes: vec![sq(5.2, 0.0, 0.5)],
        }),
        "kidney" => {
            let n = 40;
            let outer = (0..n)
                .map(|i| {
                    let t = TAU * i as f64 / n as f64;
                    let bite = 0.9 * (-t.sin()).max(0.0) * (1.0 - (t.cos().abs() * 2.2).min(1.0));
                    (4.5 + 4.5 * t.cos(), 2.4 + 2.0 * t.sin() + bite)
                })
                .collect();
            Some(Pool {
                name: "kidney",
                outer,
                holes: vec![],
            })
        }
        _ => None,
    }
}

fn inside_poly(p: P, poly: &[P]) -> bool {
    let mut c = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        if (yi > p.1) != (yj > p.1) && p.0 < (xj - xi) * (p.1 - yi) / (yj - yi) + xi {
            c = !c;
        }
        j = i;
    }
    c
}

fn seg_nearest(p: P, a: P, b: P) -> (f64, P) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l2 = dx * dx + dy * dy;
    let t = (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0);
    let q = (a.0 + t * dx, a.1 + t * dy);
    (((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2)).sqrt(), q)
}

impl Pool {
    pub fn contains(&self, p: P) -> bool {
        inside_poly(p, &self.outer) && !self.holes.iter().any(|h| inside_poly(p, h))
    }

    pub fn bounds(&self) -> (P, P) {
        let mut lo = (f64::MAX, f64::MAX);
        let mut hi = (f64::MIN, f64::MIN);
        for &(x, y) in &self.outer {
            lo = (lo.0.min(x), lo.1.min(y));
            hi = (hi.0.max(x), hi.1.max(y));
        }
        (lo, hi)
    }

    /// Direction pointing from the nearest wall into the water at `p`.
    pub fn wall_inward_normal(&self, p: P) -> (f64, f64) {
        let mut best = (f64::MAX, (0.0, 0.0), (0.0, 0.0));
        for poly in std::iter::once(&self.outer).chain(self.holes.iter()) {
            for i in 0..poly.len() {
                let a = poly[i];
                let b = poly[(i + 1) % poly.len()];
                let (d, q) = seg_nearest(p, a, b);
                if d < best.0 {
                    best = (d, q, (b.0 - a.0, b.1 - a.1));
                }
            }
        }
        let (_, q, e) = best;
        let l = (e.0 * e.0 + e.1 * e.1).sqrt();
        let n = (-e.1 / l, e.0 / l);
        // Pick the side that is water.
        if self.contains((q.0 + n.0 * 0.05, q.1 + n.1 * 0.05)) {
            n
        } else {
            (-n.0, -n.1)
        }
    }
}
