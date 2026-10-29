//! World definition, capacities, profiles, and counters.

use core::ffi::c_void;
use std::sync::Arc;

use crate::base::constants as b3;
use crate::math::types::{Pos, Vec3};
use crate::types::debug_draw::DebugShape;
use crate::types::ids::{ShapeID, WorldID};

pub const DEFAULT_CATEGORY_BITS: u64 = u64::MAX;
pub const DEFAULT_MASK_BITS: u64 = u64::MAX;

#[inline]
pub fn default_category_bits() -> u64 {
    DEFAULT_CATEGORY_BITS
}

#[inline]
pub fn default_mask_bits() -> u64 {
    DEFAULT_MASK_BITS
}

/// A task the scheduler must run on a worker thread exactly once per enqueue.
pub type TaskCallback = Box<dyn Fn(Option<*mut c_void>) + Send + Sync>;

/// Invoke a task system. Returns the user's task object, or `None` to indicate the work was
/// executed serially.
pub type EnqueueTaskCallback =
    Arc<dyn Fn(TaskCallback, Option<*mut c_void>, &str) -> Option<*mut c_void> + Send + Sync>;

/// Finishes a user task object that wraps a task. This must block until the task has completed.
pub type FinishTaskCallback = Arc<dyn Fn(*mut c_void) + Send + Sync>;

/// Create a user debug draw shape. Called when a shape is first drawn.
pub type CreateDebugShapeCallback = Arc<dyn Fn(&DebugShape) -> Option<*mut c_void>>;

/// Destroy a user debug draw shape.
pub type DestroyDebugShapeCallback = Arc<dyn Fn(Option<*mut c_void>)>;

/// Optional friction mixing callback.
pub type FrictionCallback =
    Arc<dyn Fn(f32, u64, f32, u64) -> f32 + Send + Sync>;

/// Optional restitution mixing callback.
pub type RestitutionCallback =
    Arc<dyn Fn(f32, u64, f32, u64) -> f32 + Send + Sync>;

/// Contact filter callback.
pub type CustomFilterCallback = Arc<dyn Fn(ShapeID, ShapeID) -> bool + Send + Sync>;

/// Pre-solve callback.
pub type PreSolveCallback =
    Arc<dyn Fn(ShapeID, ShapeID, Pos, Vec3) -> bool + Send + Sync>;

/// Overlap query callback.
pub type OverlapResultCallback = Arc<dyn Fn(ShapeID) -> bool>;

/// Ray cast callback.
pub type CastResultCallback = Arc<
    dyn Fn(ShapeID, Pos, Vec3, f32, u64, i32, i32) -> f32,
>;

/// Optional world capacities that can be used to avoid run-time allocations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct Capacity {
    pub static_shape_count: i32,
    pub dynamic_shape_count: i32,
    pub static_body_count: i32,
    pub dynamic_body_count: i32,
    pub contact_count: i32,
}

/// World definition used to create a simulation world.
#[derive(Clone)]
pub struct WorldDef {
    pub gravity: Vec3,
    pub restitution_threshold: f32,
    pub restitution_iterations: i32,
    pub enable_restitution_propagation: bool,
    pub hit_event_threshold: f32,
    pub contact_hertz: f32,
    pub contact_damping_ratio: f32,
    pub contact_speed: f32,
    pub maximum_linear_speed: f32,
    pub isotropic_inertia_tolerance: f32,
    pub friction_callback: Option<FrictionCallback>,
    pub restitution_callback: Option<RestitutionCallback>,
    pub custom_filter_callback: Option<CustomFilterCallback>,
    pub pre_solve_callback: Option<PreSolveCallback>,
    pub enable_sleep: bool,
    pub enable_continuous: bool,
    pub worker_count: u32,
    pub enqueue_task: Option<EnqueueTaskCallback>,
    pub finish_task: Option<FinishTaskCallback>,
    pub user_data: Option<*mut c_void>,
    pub create_debug_shape: Option<CreateDebugShapeCallback>,
    pub destroy_debug_shape: Option<DestroyDebugShapeCallback>,
    pub capacity: Capacity,
}

impl Default for WorldDef {
    fn default() -> Self {
        let units = b3::length_units_per_meter();
        Self {
            gravity: Vec3::new(0.0, -10.0, 0.0),
            restitution_threshold: units,
            restitution_iterations: 2,
            enable_restitution_propagation: false,
            hit_event_threshold: units,
            contact_hertz: 30.0,
            contact_damping_ratio: 10.0,
            contact_speed: 3.0 * units,
            maximum_linear_speed: 400.0 * units,
            isotropic_inertia_tolerance: 0.2,
            friction_callback: None,
            restitution_callback: None,
            custom_filter_callback: None,
            pre_solve_callback: None,
            enable_sleep: true,
            enable_continuous: true,
            worker_count: 0,
            enqueue_task: None,
            finish_task: None,
            user_data: None,
            create_debug_shape: None,
            destroy_debug_shape: None,
            capacity: Capacity::default(),
        }
    }
}

/// The explosion definition is used to configure options for explosions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExplosionDef {
    pub mask_bits: u64,
    pub position: Pos,
    pub radius: f32,
    pub falloff: f32,
    pub impulse_per_area: f32,
}

impl Default for ExplosionDef {
    fn default() -> Self {
        Self {
            mask_bits: DEFAULT_MASK_BITS,
            position: Pos::ZERO,
            radius: 0.0,
            falloff: 0.0,
            impulse_per_area: 0.0,
        }
    }
}

/// Profiling data. Times are in milliseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Profile {
    pub step: f32,
    pub pairs: f32,
    pub collide: f32,
    pub solve: f32,
    pub solver_setup: f32,
    pub constraints: f32,
    pub prepare_constraints: f32,
    pub integrate_velocities: f32,
    pub warm_start: f32,
    pub solve_impulses: f32,
    pub integrate_positions: f32,
    pub relax_impulses: f32,
    pub restitution: f32,
    pub store_impulses: f32,
    pub split_islands: f32,
    pub transforms: f32,
    pub sensor_hits: f32,
    pub joint_events: f32,
    pub hit_events: f32,
    pub refit: f32,
    pub bullets: f32,
    pub sleep_islands: f32,
    pub sensors: f32,
    pub soft_bodies: f32,
    pub vehicles: f32,
}

/// Counters that give details of the simulation size.
#[derive(Clone, Copy, Debug)]
pub struct Counters {
    pub byte_count: i64,
    pub body_count: i32,
    pub shape_count: i32,
    pub contact_count: i32,
    pub joint_count: i32,
    pub island_count: i32,
    pub stack_used: i32,
    pub arena_capacity: i32,
    pub static_tree_height: i32,
    pub tree_height: i32,
    pub sat_call_count: i32,
    pub sat_cache_hit_count: i32,
    pub task_count: i32,
    pub color_counts: [i32; 24],
    pub manifold_counts: [i32; 8],
    pub awake_contact_count: i32,
    pub recycled_contact_count: i32,
    pub distance_iterations: i32,
    pub push_back_iterations: i32,
    pub root_iterations: i32,
}

impl Default for Counters {
    fn default() -> Self {
        Self {
            byte_count: 0,
            body_count: 0,
            shape_count: 0,
            contact_count: 0,
            joint_count: 0,
            island_count: 0,
            stack_used: 0,
            arena_capacity: 0,
            static_tree_height: 0,
            tree_height: 0,
            sat_call_count: 0,
            sat_cache_hit_count: 0,
            task_count: 0,
            color_counts: [0; 24],
            manifold_counts: [0; 8],
            awake_contact_count: 0,
            recycled_contact_count: 0,
            distance_iterations: 0,
            push_back_iterations: 0,
            root_iterations: 0,
        }
    }
}

/// Simple holder for the world id used by public query APIs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct WorldIDHolder(pub WorldID);
