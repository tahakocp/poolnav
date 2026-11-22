//! Flat C interface so the browser can drive the simulator without glue code.
//! One world at a time; all getters read it.
use navcore::State;
use poolsim::{pool, variant, Params, World};
use std::cell::RefCell;

thread_local! {
    static WORLD: RefCell<Option<World>> = const { RefCell::new(None) };
    static GRID: RefCell<Vec<u8>> = const { RefCell::new(Vec::new()) };
    static POLY: RefCell<Vec<Vec<f64>>> = const { RefCell::new(Vec::new()) };
}

fn with<T>(f: impl FnOnce(&World) -> T, default: T) -> T {
    WORLD.with(|w| w.borrow().as_ref().map(f).unwrap_or(default))
}

/// Returns 0 on success, 1 for an unknown pool, 2 for an unknown variant.
#[no_mangle]
pub extern "C" fn pn_new(
    pool_idx: u32,
    variant_idx: u32,
    seed: u32,
    gyro_bias_dps: f64,
    speed_err: f64,
    align_dps: f64,
) -> u32 {
    let Some(name) = pool::NAMES.get(pool_idx as usize) else {
        return 1;
    };
    let Some(vname) = poolsim::VARIANTS.get(variant_idx as usize) else {
        return 2;
    };
    let p = pool::by_name(name).unwrap();
    let params = Params {
        gyro_bias_dps,
        speed_err,
        align_dps,
        ..Params::default()
    };
    let w = World::new(p.clone(), variant(vname).unwrap(), seed as u64, params);
    POLY.with(|poly| {
        let mut v = vec![p.outer.iter().flat_map(|q| [q.0, q.1]).collect::<Vec<_>>()];
        v.extend(
            p.holes
                .iter()
                .map(|h| h.iter().flat_map(|q| [q.0, q.1]).collect::<Vec<_>>()),
        );
        *poly.borrow_mut() = v;
    });
    WORLD.with(|c| *c.borrow_mut() = Some(w));
    0
}

#[no_mangle]
pub extern "C" fn pn_step(n: u32) {
    WORLD.with(|c| {
        if let Some(w) = c.borrow_mut().as_mut() {
            for _ in 0..n {
                w.step();
            }
        }
    });
}

#[no_mangle]
pub extern "C" fn pn_pool_count() -> u32 {
    pool::NAMES.len() as u32
}

#[no_mangle]
pub extern "C" fn pn_variant_count() -> u32 {
    poolsim::VARIANTS.len() as u32
}

#[no_mangle]
pub extern "C" fn pn_x() -> f64 {
    with(|w| w.x, 0.0)
}

#[no_mangle]
pub extern "C" fn pn_y() -> f64 {
    with(|w| w.y, 0.0)
}

#[no_mangle]
pub extern "C" fn pn_heading() -> f64 {
    with(|w| w.th, 0.0)
}

#[no_mangle]
pub extern "C" fn pn_time() -> f64 {
    with(|w| w.t, 0.0)
}

#[no_mangle]
pub extern "C" fn pn_coverage() -> f64 {
    with(|w| w.coverage(), 0.0)
}

#[no_mangle]
pub extern "C" fn pn_swept_over_cleaned() -> f64 {
    with(|w| w.overlap(), 0.0)
}

#[no_mangle]
pub extern "C" fn pn_body_len() -> f64 {
    with(|w| w.p.body_len, 0.4)
}

#[no_mangle]
pub extern "C" fn pn_body_wid() -> f64 {
    with(|w| w.p.body_wid, 0.3)
}

#[no_mangle]
pub extern "C" fn pn_cell() -> f64 {
    with(|w| w.p.cell, 0.1)
}

#[no_mangle]
pub extern "C" fn pn_origin_x() -> f64 {
    with(|w| w.origin().0, 0.0)
}

#[no_mangle]
pub extern "C" fn pn_origin_y() -> f64 {
    with(|w| w.origin().1, 0.0)
}

#[no_mangle]
pub extern "C" fn pn_grid_w() -> u32 {
    with(|w| w.grid_w as u32, 0)
}

#[no_mangle]
pub extern "C" fn pn_grid_h() -> u32 {
    with(|w| w.grid_h as u32, 0)
}

/// Controller state: 0 forward, 1 push, 2 back, 3 turn, 4 unjam, 5 done.
#[no_mangle]
pub extern "C" fn pn_state() -> u32 {
    with(
        |w| match w.nav.state() {
            State::Forward(_) => 0,
            State::Push(_) => 1,
            State::Back(_) => 2,
            State::Turn { .. } => 3,
            State::Unjam { .. } => 4,
            State::Done => 5,
        },
        0,
    )
}

#[no_mangle]
pub extern "C" fn pn_lanes_ok() -> u32 {
    with(|w| w.nav.lanes_ok() as u32, 1)
}

/// Copies the grid (0 outside, 1 dirty, 2 clean) and returns a pointer to it.
#[no_mangle]
pub extern "C" fn pn_grid_ptr() -> *const u8 {
    GRID.with(|g| {
        let mut g = g.borrow_mut();
        with(|w| *g = w.grid.clone(), ());
        g.as_ptr()
    })
}

#[no_mangle]
pub extern "C" fn pn_poly_count() -> u32 {
    POLY.with(|p| p.borrow().len() as u32)
}

/// Number of f64 values (x, y pairs flattened) in polygon `i`; the first polygon is the wall.
#[no_mangle]
pub extern "C" fn pn_poly_len(i: u32) -> u32 {
    POLY.with(|p| p.borrow().get(i as usize).map_or(0, |v| v.len() as u32))
}

#[no_mangle]
pub extern "C" fn pn_poly_ptr(i: u32) -> *const f64 {
    POLY.with(|p| {
        p.borrow()
            .get(i as usize)
            .map_or(std::ptr::null(), |v| v.as_ptr())
    })
}
