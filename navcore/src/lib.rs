//! Navigation for a pool robot with no camera, no lidar and no position fix.
//!
//! Inputs per tick: yaw rate from a gyro and the current drawn by each track
//! motor. Output: a left and right track command in -1..1. Everything else is
//! derived: heading by integrating the gyro, wall contact from current spikes,
//! gyro bias from the moments the robot is pinned against a wall.
#![no_std]

use core::f32::consts::{FRAC_PI_2, PI};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Strategy {
    /// Drive until a wall, back up, turn a random angle. What cheap robots do.
    Random,
    /// Find a corner, then sweep lanes wall to wall, re-aligning on every wall.
    Lanes,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub strategy: Strategy,
    /// Normalised motor current above which a track counts as blocked.
    pub contact_current: f32,
    pub contact_debounce_s: f32,
    /// Keep driving into the wall so the tracks sit flush against it.
    pub push_s: f32,
    pub reverse_s: f32,
    /// Seconds of forward travel between two lanes.
    pub shift_s: f32,
    pub turn_speed: f32,
    /// Round the heading to a multiple of 90 degrees after a flush push.
    pub snap_to_wall: bool,
    /// Learn gyro bias while pinned (the true yaw rate is zero then).
    pub bias_from_wall: bool,
    /// Run the lanes a second time at right angles to close the gaps left by
    /// slip and turn error, before wandering.
    pub second_pass: bool,
    /// After the lanes run out, wander until the cycle ends.
    pub wander_after_lanes: bool,
}

impl Config {
    pub const fn new(strategy: Strategy) -> Self {
        Config {
            strategy,
            contact_current: 0.7,
            contact_debounce_s: 0.3,
            push_s: 1.2,
            reverse_s: 0.6,
            shift_s: 1.0,
            turn_speed: 0.8,
            snap_to_wall: true,
            bias_from_wall: true,
            second_pass: true,
            wander_after_lanes: true,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Sensors {
    /// Yaw rate, rad/s, counter-clockwise positive.
    pub gyro_z: f32,
    pub current_left: f32,
    pub current_right: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Drive {
    pub left: f32,
    pub right: f32,
}

/// What a forward run is for. Decides what happens at the next wall.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Run {
    Wander,
    FindWall,
    FindCorner,
    Sweep,
    Shift,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum State {
    Forward(Run),
    Push(Run),
    Back(Run),
    /// Rotation is blocked (a wall is too close): back off, then resume.
    Unjam {
        target: f32,
        then: Run,
    },
    Turn {
        target: f32,
        then: Run,
    },
    Done,
}

pub struct Nav {
    cfg: Config,
    state: State,
    clock: f32,
    in_state: f32,
    blocked_for: f32,
    heading: f32,
    /// Heading to keep while driving straight.
    hold: f32,
    bias: f32,
    wall_ref: Option<f32>,
    /// +1: next lane turn is to the left, -1: to the right.
    lane_dir: f32,
    lanes_done: u16,
    unjam_dir: f32,
    lanes_ok: bool,
    pass: u8,
    /// Turn direction at a corner, and the side the lanes then advance to.
    corner_turn: f32,
    rng: u32,
}

const TURN_TOL: f32 = 0.012;
/// Pool walls this far from square to the first wall mean lanes make no sense.
const MAX_WALL_SKEW: f32 = 0.3;
/// rad/s. Above this a pinned robot is still rotating, so it says nothing about bias.
const MAX_PINNED_RATE: f32 = 0.035;
const MAX_BIAS: f32 = 0.08;
const TURN_TIMEOUT_S: f32 = 8.0;

impl Nav {
    pub fn new(cfg: Config, seed: u32) -> Self {
        let first = match cfg.strategy {
            Strategy::Random => Run::Wander,
            Strategy::Lanes => Run::FindWall,
        };
        Nav {
            cfg,
            state: State::Forward(first),
            clock: 0.0,
            in_state: 0.0,
            blocked_for: 0.0,
            heading: 0.0,
            hold: 0.0,
            bias: 0.0,
            wall_ref: None,
            lane_dir: 1.0,
            lanes_done: 0,
            unjam_dir: -1.0,
            lanes_ok: true,
            pass: 0,
            corner_turn: 1.0,
            rng: seed | 1,
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn heading(&self) -> f32 {
        self.heading
    }

    /// False once a wall turned out not to be square to the first one.
    pub fn lanes_ok(&self) -> bool {
        self.lanes_ok
    }

    pub fn gyro_bias(&self) -> f32 {
        self.bias
    }

    /// Lanes completed so far (each shift that ended without hitting a wall).
    pub fn lanes_done(&self) -> u16 {
        self.lanes_done
    }

    fn enter(&mut self, s: State) {
        if let State::Forward(_) = s {
            self.hold = self.heading;
        }
        self.state = s;
        self.in_state = 0.0;
        self.blocked_for = 0.0;
    }

    fn rand01(&mut self) -> f32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        (x >> 8) as f32 / (1u32 << 24) as f32
    }

    fn turn_by(&mut self, delta: f32, then: Run) {
        let target = self.heading + delta;
        self.enter(State::Turn { target, then });
    }

    /// Rounds the heading to the wall grid. Returns how far off the grid the
    /// robot was, which is large when the wall it just hit is not square to the first.
    fn snap(&mut self) -> f32 {
        let Some(r) = self.wall_ref else { return 0.0 };
        let q = (self.heading - r) / FRAC_PI_2;
        let n = if q >= 0.0 {
            (q + 0.5) as i32
        } else {
            (q - 0.5) as i32
        };
        let grid = r + n as f32 * FRAC_PI_2;
        let off = self.heading - grid;
        if self.cfg.snap_to_wall {
            self.heading = grid;
        }
        off
    }

    /// Called when the robot has backed off a wall after a run of this kind.
    fn after_wall(&mut self, run: Run) {
        let d = self.lane_dir;
        match run {
            Run::Wander => {
                let a = (60.0 + 240.0 * self.rand01()).to_radians();
                let sign = if self.rand01() < 0.5 { 1.0 } else { -1.0 };
                self.turn_by(sign * a, Run::Wander);
            }
            Run::FindWall => self.turn_by(FRAC_PI_2, Run::FindCorner),
            Run::FindCorner => {
                self.lane_dir = self.corner_turn;
                self.turn_by(self.corner_turn * FRAC_PI_2, Run::Sweep);
            }
            Run::Sweep => self.turn_by(d * FRAC_PI_2, Run::Shift),
            Run::Shift => {
                // Ran into the far wall while shifting: the lanes are used up.
                if self.pass == 0 && self.cfg.second_pass && self.lanes_ok {
                    // Head back along this wall to the corner and sweep across the first pass.
                    self.pass = 1;
                    self.corner_turn = -d;
                    self.turn_by(-d * FRAC_PI_2, Run::FindCorner);
                } else if self.cfg.wander_after_lanes {
                    self.turn_by(PI, Run::Wander);
                } else {
                    self.enter(State::Done);
                }
            }
        }
    }

    pub fn step(&mut self, s: &Sensors, dt: f32) -> Drive {
        self.clock += dt;
        self.in_state += dt;
        self.heading += (s.gyro_z - self.bias) * dt;

        let blocked = s.current_left.max(s.current_right) > self.cfg.contact_current;
        match self.state {
            State::Forward(run) => {
                if blocked {
                    self.blocked_for += dt;
                } else {
                    self.blocked_for = 0.0;
                }
                if self.blocked_for >= self.cfg.contact_debounce_s {
                    let next = match run {
                        Run::Wander | Run::Shift => State::Back(run),
                        _ => State::Push(run),
                    };
                    self.enter(next);
                    return Drive::default();
                }
                if run == Run::Shift && self.in_state >= self.cfg.shift_s {
                    self.lanes_done = self.lanes_done.saturating_add(1);
                    let d = self.lane_dir;
                    self.lane_dir = -d;
                    self.turn_by(d * FRAC_PI_2, Run::Sweep);
                    return Drive::default();
                }
                let steer = ((self.hold - self.heading) * 2.0).clamp(-0.3, 0.3);
                Drive {
                    left: 1.0 - steer,
                    right: 1.0 + steer,
                }
            }
            State::Push(run) => {
                // Only trust the gyro once the body has stopped squaring itself
                // to the wall; while it still rotates the rate is real.
                let quiet = (s.gyro_z - self.bias).abs() < MAX_PINNED_RATE;
                if self.cfg.bias_from_wall && quiet && blocked {
                    self.bias =
                        (self.bias + 0.1 * (s.gyro_z - self.bias)).clamp(-MAX_BIAS, MAX_BIAS);
                }
                if self.in_state >= self.cfg.push_s {
                    if run == Run::FindWall && self.wall_ref.is_none() {
                        self.wall_ref = Some(self.heading);
                    }
                    let off = self.snap();
                    let run = if off.abs() > MAX_WALL_SKEW {
                        self.lanes_ok = false;
                        Run::Wander
                    } else {
                        run
                    };
                    self.enter(State::Back(run));
                    return Drive::default();
                }
                Drive {
                    left: 0.6,
                    right: 0.6,
                }
            }
            State::Back(run) => {
                if self.in_state >= self.cfg.reverse_s {
                    self.after_wall(run);
                    return Drive::default();
                }
                Drive {
                    left: -0.6,
                    right: -0.6,
                }
            }
            State::Unjam { target, then } => {
                if blocked {
                    self.blocked_for += dt;
                    if self.blocked_for >= self.cfg.contact_debounce_s {
                        // Something in the way: go the other way, for as long as it takes.
                        self.unjam_dir = -self.unjam_dir;
                        self.blocked_for = 0.0;
                        self.in_state = 0.0;
                    }
                } else {
                    self.blocked_for = 0.0;
                }
                if self.in_state >= self.cfg.reverse_s {
                    self.enter(State::Turn { target, then });
                    return Drive::default();
                }
                Drive {
                    left: 0.6 * self.unjam_dir,
                    right: 0.6 * self.unjam_dir,
                }
            }
            State::Turn { target, then } => {
                if blocked {
                    self.blocked_for += dt;
                    if self.blocked_for >= self.cfg.contact_debounce_s {
                        // Blocked on this side: rotate the long way round instead.
                        let err = target - self.heading;
                        let target = if err > 0.0 {
                            target - 2.0 * PI
                        } else {
                            target + 2.0 * PI
                        };
                        self.enter(State::Unjam { target, then });
                        return Drive::default();
                    }
                } else {
                    self.blocked_for = 0.0;
                }
                let err = target - self.heading;
                if err.abs() < TURN_TOL || self.in_state > TURN_TIMEOUT_S {
                    self.enter(State::Forward(then));
                    return Drive::default();
                }
                let mag = (err.abs() * 1.5).clamp(0.12, self.cfg.turn_speed);
                let v = if err > 0.0 { mag } else { -mag };
                Drive { left: -v, right: v }
            }
            State::Done => Drive::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.1;

    fn free() -> Sensors {
        Sensors {
            gyro_z: 0.0,
            current_left: 0.2,
            current_right: 0.2,
        }
    }

    fn stalled() -> Sensors {
        Sensors {
            gyro_z: 0.0,
            current_left: 0.95,
            current_right: 0.95,
        }
    }

    #[test]
    fn heading_integrates_gyro() {
        let mut n = Nav::new(Config::new(Strategy::Random), 7);
        for _ in 0..10 {
            n.step(
                &Sensors {
                    gyro_z: 0.5,
                    ..free()
                },
                DT,
            );
        }
        assert!((n.heading() - 0.5).abs() < 1e-4);
    }

    #[test]
    fn short_current_spike_is_not_a_wall() {
        let mut n = Nav::new(Config::new(Strategy::Random), 7);
        n.step(&free(), DT);
        n.step(&stalled(), DT);
        n.step(&stalled(), DT);
        n.step(&free(), DT);
        assert_eq!(n.state(), State::Forward(Run::Wander));
    }

    #[test]
    fn sustained_stall_backs_off() {
        let mut n = Nav::new(Config::new(Strategy::Random), 7);
        for _ in 0..5 {
            n.step(&stalled(), DT);
        }
        assert!(matches!(n.state(), State::Back(Run::Wander)));
        let d = n.step(&free(), DT);
        assert!(d.left < 0.0 && d.right < 0.0);
    }

    #[test]
    fn lanes_push_into_the_first_wall_before_turning() {
        let mut n = Nav::new(Config::new(Strategy::Lanes), 7);
        for _ in 0..5 {
            n.step(&stalled(), DT);
        }
        assert!(matches!(n.state(), State::Push(Run::FindWall)));
    }

    #[test]
    fn pinned_gyro_offset_becomes_bias() {
        let mut n = Nav::new(Config::new(Strategy::Lanes), 7);
        for _ in 0..5 {
            n.step(&stalled(), DT);
        }
        for _ in 0..30 {
            n.step(
                &Sensors {
                    gyro_z: 0.01,
                    ..stalled()
                },
                DT,
            );
            if !matches!(n.state(), State::Push(_)) {
                break;
            }
        }
        assert!(
            n.gyro_bias() > 0.002 && n.gyro_bias() <= 0.01,
            "{}",
            n.gyro_bias()
        );
    }

    #[test]
    fn rotation_while_pinned_is_not_mistaken_for_bias() {
        let mut n = Nav::new(Config::new(Strategy::Lanes), 7);
        for _ in 0..5 {
            n.step(&stalled(), DT);
        }
        for _ in 0..8 {
            n.step(
                &Sensors {
                    gyro_z: 0.5,
                    ..stalled()
                },
                DT,
            );
        }
        assert_eq!(n.gyro_bias(), 0.0);
    }

    #[test]
    fn same_seed_same_first_turn() {
        let first_turn = |seed| {
            let mut n = Nav::new(Config::new(Strategy::Random), seed);
            for _ in 0..40 {
                n.step(&stalled(), DT);
                if let State::Turn { target, .. } = n.state() {
                    return target;
                }
            }
            panic!("no turn");
        };
        assert_eq!(first_turn(5), first_turn(5));
        assert_ne!(first_turn(5), first_turn(6));
    }
}
