use navcore::{Config, Strategy};
use poolsim::{pool, Params, World};

fn world(name: &str, strat: Strategy, seed: u64) -> World {
    World::new(
        pool::by_name(name).unwrap(),
        Config::new(strat),
        seed,
        Params::default(),
    )
}

#[test]
fn robot_never_leaves_the_water() {
    for name in pool::NAMES {
        let mut w = world(name, Strategy::Lanes, 11);
        for _ in 0..30_000 {
            w.step();
            assert!(
                w.pool.contains((w.x, w.y)),
                "{name}: left the pool at t={}",
                w.t
            );
        }
    }
}

#[test]
fn same_seed_same_run() {
    let mut a = world("rect", Strategy::Lanes, 4);
    let mut b = world("rect", Strategy::Lanes, 4);
    for _ in 0..5_000 {
        a.step();
        b.step();
    }
    assert_eq!((a.x, a.y, a.th, a.clean), (b.x, b.y, b.th, b.clean));
}

fn minutes_to(name: &str, strat: Strategy, seed: u64, target: f64) -> f64 {
    let mut w = world(name, strat, seed);
    while w.coverage() < target && w.t < 7200.0 {
        w.step();
    }
    w.t / 60.0
}

#[test]
fn lanes_beat_random_in_a_rectangle() {
    let seeds = 1..=8u64;
    let lanes: f64 = seeds
        .clone()
        .map(|s| minutes_to("rect", Strategy::Lanes, s, 0.95))
        .sum();
    let random: f64 = seeds
        .map(|s| minutes_to("rect", Strategy::Random, s, 0.95))
        .sum();
    assert!(
        lanes < random * 0.8,
        "lanes {lanes:.0} min total vs random {random:.0}"
    );
}

#[test]
fn lanes_give_up_on_a_curved_pool() {
    let mut w = world("kidney", Strategy::Lanes, 2);
    for _ in 0..20_000 {
        w.step();
    }
    assert!(!w.nav.lanes_ok());
}

#[test]
fn reachable_area_is_close_to_pool_area() {
    let w = world("rect", Strategy::Random, 1);
    // 8 x 4 m at 10 cm cells is 3200 cells; none can lie outside the pool.
    assert!((3100..=3200).contains(&w.total), "{}", w.total);
}
