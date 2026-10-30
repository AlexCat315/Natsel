//! Quality scenes: soft body query throughput and sparse soft body precision.

use std::sync::atomic::{AtomicI32, AtomicU32, AtomicU64, Ordering};

use natsel::math::types::{Pos, Vec3};
use natsel::simulation::world_runtime::World;
use natsel::soft_body::soft_body_runtime::{SoftBodyDef, SoftBodyID};
use natsel::soft_body::soft_body_settings::{SoftBodyBendType, SoftBodySettings};

/// Swift's `nonisolated(unsafe) private var querySoftBody`. The benchmark harness is single
/// threaded, so relaxed atomics stand in for the file-scope variable without `static mut`.
/// All three identity fields are kept: `soft_body_is_valid` checks `world0` and `generation`
/// as well as the index, so an index alone would not resolve.
static QUERY_SOFT_BODY_INDEX: AtomicI32 = AtomicI32::new(0);
static QUERY_SOFT_BODY_WORLD: AtomicU32 = AtomicU32::new(0);
static QUERY_SOFT_BODY_GENERATION: AtomicU32 = AtomicU32::new(0);
/// Swift's `nonisolated(unsafe) private var softRayChecksum: Double`, held as its bit pattern.
static SOFT_RAY_CHECKSUM: AtomicU64 = AtomicU64::new(0);

fn add_checksum(value: f64) {
    let previous = f64::from_bits(SOFT_RAY_CHECKSUM.load(Ordering::Relaxed));
    SOFT_RAY_CHECKSUM.store((previous + value).to_bits(), Ordering::Relaxed);
}

fn query_soft_body() -> SoftBodyID {
    let index = QUERY_SOFT_BODY_INDEX.load(Ordering::Relaxed);
    if index == 0 {
        return SoftBodyID::NULL;
    }
    SoftBodyID::new(
        index,
        QUERY_SOFT_BODY_WORLD.load(Ordering::Relaxed) as u16,
        QUERY_SOFT_BODY_GENERATION.load(Ordering::Relaxed) as u16,
    )
}

/// A deformable surface with 7,938 faces queried 512 times per step. The BVH is rebuilt lazily
/// after each simulated frame. `NATSEL_SOFT_QUERY_BVH=0` provides the linear-scan baseline.
pub fn create_soft_queries(world: &mut World) {
    world.gravity = Vec3::ZERO;
    world.enable_soft_body_query_acceleration =
        std::env::var("NATSEL_SOFT_QUERY_BVH").unwrap_or_default() != "0";
    let mut settings = SoftBodySettings::make_cloth(
        64,
        64,
        0.1,
        Default::default(),
        SoftBodyBendType::Distance,
        |_, _| false,
    );
    for vertex in settings.vertices.iter_mut() {
        vertex.inv_mass = 0.0;
    }
    let mut def = SoftBodyDef::default();
    def.settings = settings;
    def.enable_sleep = false;
    def.faces_double_sided = true;
    let id = world.create_soft_body(&def);
    QUERY_SOFT_BODY_INDEX.store(id.index1, Ordering::Relaxed);
    QUERY_SOFT_BODY_WORLD.store(id.world0 as u32, Ordering::Relaxed);
    QUERY_SOFT_BODY_GENERATION.store(id.generation as u32, Ordering::Relaxed);
    SOFT_RAY_CHECKSUM.store(0.0f64.to_bits(), Ordering::Relaxed);
}

pub fn step_soft_queries(world: &mut World, step: usize) {
    let id = query_soft_body();
    for i in 0..512usize {
        let x = (((i * 17 + step) % 101) as f64) * 0.06 - 3.0;
        let z = (((i * 31) % 101) as f64) * 0.06 - 3.0;
        let hit = world.cast_ray_soft_body(id, Pos::new(x, 2.0, z), Vec3::new(0.0, -4.0, 0.0));
        if hit.hit {
            add_checksum(hit.fraction as f64 + hit.face_index as f64);
        }
    }
}

pub fn destroy_soft_queries() {
    println!(
        "soft-ray checksum: {}",
        f64::from_bits(SOFT_RAY_CHECKSUM.load(Ordering::Relaxed))
    );
    QUERY_SOFT_BODY_INDEX.store(0, Ordering::Relaxed);
}

/// 2,000 separated soft bodies at a distant origin. Exercises precise grouping without creating
/// quadratic pairs from coarse absolute float bounds.
pub fn create_sparse_soft_bodies(world: &mut World) {
    world.gravity = Vec3::ZERO;
    let settings = SoftBodySettings::make_cloth(
        2,
        2,
        0.1,
        Default::default(),
        SoftBodyBendType::Distance,
        |_, _| false,
    );
    for i in 0..2000 {
        let mut def = SoftBodyDef::default();
        def.settings = settings.clone();
        def.position = Pos::new(1.0e12, i as f64 * 2.0, -1.0e12);
        def.enable_sleep = false;
        def.iteration_count = 1;
        world.create_soft_body(&def);
    }
}
