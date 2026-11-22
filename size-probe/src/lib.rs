//! Exists only to measure what the controller costs on a Cortex-M0+ (RP2040).
//! It exposes the one call a firmware main loop would make.
#![no_std]

use core::mem::MaybeUninit;
use navcore::{Config, Nav, Sensors, Strategy};

static mut NAV: MaybeUninit<Nav> = MaybeUninit::uninit();

#[no_mangle]
pub extern "C" fn probe_init(seed: u32) {
    // Single-threaded probe, never linked into anything that calls it twice at once.
    unsafe { (*core::ptr::addr_of_mut!(NAV)).write(Nav::new(Config::new(Strategy::Lanes), seed)) };
}

#[no_mangle]
pub extern "C" fn probe_step(gyro: f32, cur_l: f32, cur_r: f32, out: *mut f32) {
    let nav = unsafe { (*core::ptr::addr_of_mut!(NAV)).assume_init_mut() };
    let d = nav.step(&Sensors { gyro_z: gyro, current_left: cur_l, current_right: cur_r }, 0.1);
    unsafe {
        *out = d.left;
        *out.add(1) = d.right;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
