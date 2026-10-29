//! Compile-time engine constants. Values that scale with the length unit are computed so
//! `set_length_units_per_meter` takes effect.

use core::sync::atomic::{AtomicU32, Ordering};

pub const PI: f32 = 3.14159265359;
pub const DEG_TO_RAD: f32 = 0.01745329251;
pub const RAD_TO_DEG: f32 = 57.2957795131;
pub const MIN_SCALE: f32 = 0.01;
pub const TWO_PI: f32 = 6.283185307;
pub const PI_OVER_TWO: f32 = 1.570796327;
pub const PI_OVER_FOUR: f32 = 0.785398163;
pub const SQRT3: f32 = 1.732050808;

/// Used to indicate null for interfaces that work with indices instead of pointers.
pub const NULL_INDEX: i32 = -1;

/// Maximum parallel workers. Used for some fixed size arrays.
pub const MAX_WORKERS: i32 = 32;
/// Maximum number of tasks queued per world step.
pub const MAX_TASKS: i32 = 256;
/// Maximum number of colors in the constraint graph.
pub const GRAPH_COLOR_COUNT: i32 = 24;
/// Number of contact point buckets.
pub const CONTACT_MANIFOLD_COUNT_BUCKETS: i32 = 8;
pub const MIN_FRICTION_WEIGHT: f32 = 1e-10;
pub const MAX_WORLDS: i32 = 128;
pub const MAX_ROTATION: f32 = 0.25 * PI;
pub const CONTACT_RECYCLE_ANGULAR_DISTANCE: f32 = 0.99240388;
pub const AABB_MARGIN_FRACTION: f32 = 0.125;
pub const TIME_TO_SLEEP: f32 = 0.5;
pub const MAX_RESTITUTION_ITERATIONS: i32 = 63;
pub const MAX_MANIFOLD_POINTS: i32 = 4;
pub const GYROSCOPIC_ITERATIONS: i32 = 1;
pub const MAX_HULL_VERTICES: i32 = 128;
pub const MAX_HULL_FACES: i32 = 128;
pub const MAX_HULL_EDGES: i32 = 128;
pub const PARALLEL_EDGE_TOLERANCE: f32 = 0.005;
pub const MAX_SHAPE_CAST_POINTS: i32 = MAX_HULL_VERTICES;
pub const SHAPE_POWER: i32 = 22;
pub const CHILD_POWER: i32 = 64 - 2 * SHAPE_POWER;
pub const MAX_SHAPES: i32 = 1 << SHAPE_POWER;
pub const MAX_CHILD_SHAPES: i64 = 1 << CHILD_POWER;
pub const MAX_MESH_CONTACT_TRIANGLES: i32 = 512;
pub const ALIGNMENT: usize = 32;
pub const HASH_INIT: u32 = 5381;

pub const DYNAMIC_TREE_VERSION: u64 = 0x1D6F_4C2A_73B8_0E91;
pub const HULL_VERSION: u64 = 0x6B1E_39D4_A87C_25F3;
pub const MESH_VERSION: u64 = 0xAAAB_9A00_F1A8_AAF7;
pub const COMPOUND_VERSION: u64 =
    0x9E4B_17D3_A25C_68F1 ^ MESH_VERSION ^ HULL_VERSION;
pub const HEIGHT_FIELD_VERSION: u64 = 0x8E41_E5FB_0848_48F8;

static LENGTH_UNITS: AtomicU32 = AtomicU32::new(0x3F80_0000); // 1.0f32

pub fn set_length_units_per_meter(units: f32) {
    assert!(units.is_finite() && units > 0.0);
    LENGTH_UNITS.store(units.to_bits(), Ordering::SeqCst);
}

#[inline]
pub fn length_units_per_meter() -> f32 {
    f32::from_bits(LENGTH_UNITS.load(Ordering::SeqCst))
}

#[inline]
pub fn get_length_units_per_meter() -> f32 {
    length_units_per_meter()
}

/// In float mode positions greater than about 16km have precision problems, so 100km is a safe
/// limit.
#[inline]
pub fn huge() -> f32 {
    1.0e5 * length_units_per_meter()
}

/// A small length used as a collision and constraint tolerance.
#[inline]
pub fn linear_slop() -> f32 {
    0.005 * length_units_per_meter()
}

#[inline]
pub fn min_capsule_length() -> f32 {
    linear_slop()
}

#[inline]
pub fn overlap_slop() -> f32 {
    0.1 * linear_slop()
}

#[inline]
pub fn speculative_distance() -> f32 {
    4.0 * linear_slop()
}

#[inline]
pub fn mesh_rest_offset() -> f32 {
    linear_slop()
}

#[inline]
pub fn contact_recycle_distance() -> f32 {
    10.0 * linear_slop()
}

#[inline]
pub fn max_aabb_margin() -> f32 {
    0.05 * length_units_per_meter()
}
