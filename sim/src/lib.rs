//! 2D simulator for a tracked pool robot. The controller is the real navcore
//! crate, fed only a noisy gyro and motor currents.
pub mod pool;
pub mod rng;
pub mod world;

pub use pool::Pool;
pub use world::{Params, World};

use navcore::{Config, Strategy};

pub const VARIANTS: [&str; 5] = [
    "random",
    "lanes",
    "lanes-one-pass",
    "lanes-no-snap",
    "lanes-no-bias",
];

/// Named controller settings used by the bench and the browser demo.
pub fn variant(name: &str) -> Option<Config> {
    let mut c = Config::new(Strategy::Lanes);
    match name {
        "random" => c = Config::new(Strategy::Random),
        "lanes" => {}
        "lanes-one-pass" => c.second_pass = false,
        "lanes-no-snap" => c.snap_to_wall = false,
        "lanes-no-bias" => c.bias_from_wall = false,
        _ => return None,
    }
    Some(c)
}
