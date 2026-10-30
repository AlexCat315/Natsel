//! Scene builders used by the Natsel benchmark harness.
//!
//! Ported from `Sources/Benchmarks/Scenes.swift`.

use std::sync::atomic::{AtomicI32, Ordering};

use natsel::collision::hull::make_box_hull;
use natsel::collision::hull_builder::{create_complex_hull, create_cylinder, create_hull};
use natsel::collision::mesh::build::MeshBuildOptions;
use natsel::collision::mesh::procedural::create_wave_mesh;
use natsel::collision::mesh::TriangleMesh;
use natsel::math::functions::{
    compute_cos_sin, cross, inv_rotate_vector, make_quat_from_axis_angle, mul_add, rotate_vector,
};
use natsel::math::types::{Pos, Quat, Vec3};
use natsel::simulation::world_runtime::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::Sphere;
use natsel::types::ids::{BodyID, JointID};
use natsel::types::joint_defs::{FilterJointDef, RevoluteJointDef, SphericalJointDef};
use natsel::types::shape_def::ShapeDef;

// MARK: - Shared storage

/// Body, shape, and contact counts for the last benchmark.
///
/// Swift's `nonisolated(unsafe) var worldCounters` is a mutable file-scope tuple. The harness is
/// single threaded, so three relaxed atomics stand in for it without `static mut`.
pub static WORLD_COUNTER_BODY: AtomicI32 = AtomicI32::new(0);
pub static WORLD_COUNTER_SHAPE: AtomicI32 = AtomicI32::new(0);
pub static WORLD_COUNTER_CONTACT: AtomicI32 = AtomicI32::new(0);

pub fn record_counters(world: &World) {
    WORLD_COUNTER_BODY.store(world.body_count(), Ordering::Relaxed);
    WORLD_COUNTER_SHAPE.store(0, Ordering::Relaxed);
    WORLD_COUNTER_CONTACT.store(world.contact_count(), Ordering::Relaxed);
}

pub fn world_counters() -> (i32, i32, i32) {
    (
        WORLD_COUNTER_BODY.load(Ordering::Relaxed),
        WORLD_COUNTER_SHAPE.load(Ordering::Relaxed),
        WORLD_COUNTER_CONTACT.load(Ordering::Relaxed),
    )
}

// MARK: - joint_grid

pub fn create_joint_grid(world: &mut World) {
    world.enable_sleeping(false);

    let n = 100i32;

    let mut body_def = BodyDef::default();
    body_def.enable_sleep = false;
    let mut shape_def = ShapeDef::default();
    shape_def.filter.category_bits = 2;
    shape_def.filter.mask_bits = !2u64;
    let sphere = Sphere::new(Vec3::ZERO, 0.4);

    let mut joint_def = SphericalJointDef::default();
    let mut bodies: Vec<BodyID> = vec![BodyID::NULL; (n * n) as usize];
    let mut index = 0usize;

    for k in 0..n {
        for i in 0..n {
            let fk = k as f32;
            let fi = i as f32;

            body_def.type_ = if i == 0 {
                BodyType::StaticBody
            } else {
                BodyType::DynamicBody
            };
            body_def.position = Pos::new(fk as f64, -fi as f64, 0.0);

            let body = world.create_body(&body_def);
            let _ = world.create_sphere_shape(body, &shape_def, sphere);

            if i > 0 {
                joint_def.base.body_id_a = bodies[index - 1];
                joint_def.base.body_id_b = body;
                joint_def.base.local_frame_a.p = Vec3::new(0.0, -0.5, 0.0);
                joint_def.base.local_frame_b.p = Vec3::new(0.0, 0.5, 0.0);
                let _ = world.create_spherical_joint(&joint_def);
            }

            if k > 0 {
                joint_def.base.body_id_a = bodies[index - n as usize];
                joint_def.base.body_id_b = body;
                joint_def.base.local_frame_a.p = Vec3::new(0.5, 0.0, 0.0);
                joint_def.base.local_frame_b.p = Vec3::new(-0.5, 0.0, 0.0);
                let _ = world.create_spherical_joint(&joint_def);
            }

            bodies[index] = body;
            index += 1;
        }
    }
    record_counters(world);
}

// MARK: - large_pyramid

pub fn create_large_pyramid(world: &mut World) {
    world.enable_sleeping(false);

    let base_count = 100i32;

    {
        let mut body_def = BodyDef::default();
        body_def.position = Pos::new(0.0, -1.0, 0.0);
        let ground = world.create_body(&body_def);
        let shape_def = ShapeDef::default();
        let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(400.0, 1.0, 400.0));
    }

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;

    let mut shape_def = ShapeDef::default();
    shape_def.density = 100.0;

    let h: f32 = 0.5;
    let box_hull = make_box_hull(h, h, h);
    let shift = h;

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * ((j - i) as f32) * shift
                - h * base_count as f32;
            body_def.position = Pos::new(x as f64, y as f64, 0.0);
            let body = world.create_body(&body_def);
            let _ = world.create_hull_shape(body, &shape_def, &box_hull);
        }
    }
    record_counters(world);
}

// MARK: - many_pyramids

fn create_small_pyramid(
    world: &mut World,
    base_count: i32,
    extent: f32,
    center_x: f32,
    base_z: f32,
) {
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.enable_sleep = false;

    let mut shape_def = ShapeDef::default();
    shape_def.density = 100.0;

    let box_hull = make_box_hull(extent, extent, extent);

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * extent;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent + center_x - 0.5;
            body_def.position = Pos::new(x as f64, y as f64, base_z as f64);
            let body = world.create_body(&body_def);
            let _ = world.create_hull_shape(body, &shape_def, &box_hull);
        }
    }
}

pub fn create_many_pyramids(world: &mut World) {
    let base_count = 10i32;
    let extent: f32 = 0.5;
    let row_count = 14i32;
    let column_count = 14i32;
    let ground_extent = extent * column_count as f32 * (base_count as f32 + 1.0);

    {
        let mut body_def = BodyDef::default();
        body_def.position = Pos::new(0.0, -1.0, 0.0);
        let ground = world.create_body(&body_def);
        let shape_def = ShapeDef::default();
        let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(ground_extent, 1.0, ground_extent));
    }

    let base_width = 2.0 * extent * base_count as f32;
    let base_z = -ground_extent + 2.0 * extent;
    let delta_z = 2.0 * (ground_extent - 2.0 * extent) / (row_count - 1) as f32;

    let mut z = base_z;
    for _ in 0..row_count {
        for j in 0..column_count {
            let center_x = -ground_extent + j as f32 * (base_width + 2.0 * extent) + 2.0 * extent;
            create_small_pyramid(world, base_count, extent, center_x, z);
        }
        z += delta_z;
    }
    record_counters(world);
}

// MARK: - washer

pub fn create_washer(world: &mut World) {
    let ground_id: BodyID;
    {
        let mut body_def = BodyDef::default();
        body_def.position.y = -1.0;
        ground_id = world.create_body(&body_def);
        let shape_def = ShapeDef::default();
        let _ = world.create_hull_shape(ground_id, &shape_def, &make_box_hull(60.0, 1.0, 60.0));
    }

    {
        let motor_speed: f32 = 25.0;

        let mut body_def = BodyDef::default();
        body_def.position = Pos::new(0.0, 21.0, 0.0);
        body_def.type_ = BodyType::KinematicBody;
        body_def.angular_velocity = Vec3::new(0.0, 0.0, (scene_pi() / 180.0) * motor_speed);
        body_def.linear_velocity = Vec3::new(0.001, -0.002, 0.0);

        let body_id = world.create_body(&body_def);
        let shape_def = ShapeDef::default();

        let r0: f32 = 14.0;
        let r1: f32 = 16.0;
        let r2: f32 = 18.0;
        let nd = Vec3::new(0.0, 0.0, -10.0);
        let pd = Vec3::new(0.0, 0.0, 10.0);

        let angle = scene_pi() / 18.0;
        let q = make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), angle);
        let qo = make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.1 * angle);
        let mut u1 = Vec3::new(1.0, 0.0, 0.0);
        for i in 0..36 {
            let u2: Vec3;
            if i == 35 {
                u2 = Vec3::new(1.0, 0.0, 0.0);
            } else {
                u2 = rotate_vector(q, u1);
            }

            {
                let a1 = inv_rotate_vector(qo, u1);
                let a2 = rotate_vector(qo, u2);
                let p1 = mul_add(nd, r1, a1);
                let p2 = mul_add(nd, r2, a1);
                let p3 = mul_add(nd, r1, a2);
                let p4 = mul_add(nd, r2, a2);
                let p5 = mul_add(pd, r1, a1);
                let p6 = mul_add(pd, r2, a1);
                let p7 = mul_add(pd, r1, a2);
                let p8 = mul_add(pd, r2, a2);
                if let Some(hull) = create_hull(&[p1, p2, p3, p4, p5, p6, p7, p8], 8) {
                    let _ = world.create_hull_shape(body_id, &shape_def, &hull);
                }
            }

            if i % 9 == 0 {
                let p1 = mul_add(nd, r0, u1);
                let p2 = mul_add(nd, r1, u1);
                let p3 = mul_add(nd, r0, u2);
                let p4 = mul_add(nd, r1, u2);
                let p5 = mul_add(pd, r0, u1);
                let p6 = mul_add(pd, r1, u1);
                let p7 = mul_add(pd, r0, u2);
                let p8 = mul_add(pd, r1, u2);
                if let Some(hull) = create_hull(&[p1, p2, p3, p4, p5, p6, p7, p8], 8) {
                    let _ = world.create_hull_shape(body_id, &shape_def, &hull);
                }
            }

            u1 = u2;
        }
    }

    let grid_count = 20i32;
    let a: f32 = 0.2;

    let cube = make_box_hull(a, a, a);
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    let shape_def = ShapeDef::default();

    let mut x = -2.0 * a * grid_count as f32;
    for _ in 0..grid_count {
        let mut y = -2.0 * a * grid_count as f32 + 21.0;
        for _ in 0..grid_count {
            let mut z = -2.0 * a * grid_count as f32;
            for _ in 0..grid_count {
                body_def.position = Pos::new(x as f64, y as f64, z as f64);
                let body_id = world.create_body(&body_def);
                let _ = world.create_hull_shape(body_id, &shape_def, &cube);
                z += 4.0 * a;
            }
            y += 4.0 * a;
        }
        x += 4.0 * a;
    }
    record_counters(world);
}

// MARK: - convex_pile

pub fn create_convex_pile(world: &mut World) {
    {
        let mut body_def = BodyDef::default();
        body_def.position = Pos::new(0.0, -1.0, 0.0);
        let ground = world.create_body(&body_def);
        let shape_def = ShapeDef::default();
        let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(250.0, 1.0, 250.0));
    }

    let count_x = 8i32;
    let count_z = 8i32;
    let layers = 80i32;
    let amplitude: f32 = 2.0;
    let scatter = 2.0 * amplitude;

    let Some(convex) = create_complex_hull(amplitude) else {
        return;
    };

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    let shape_def = ShapeDef::default();

    for layer in 0..layers {
        for z in 0..count_z {
            for x in 0..count_x {
                let pos_x = (x as f32 - 0.5 * count_x as f32) * scatter;
                let pos_z = (z as f32 - 0.5 * count_z as f32) * scatter;
                let pos_y = amplitude + 2.0 * amplitude * layer as f32;
                body_def.position = Pos::new(pos_x as f64, pos_y as f64, pos_z as f64);
                let body_id = world.create_body(&body_def);
                let _ = world.create_hull_shape(body_id, &shape_def, &convex);
            }
        }
    }
    record_counters(world);
}

// MARK: - spinner

/// Swift's file-scope `nonisolated(unsafe)` globals. Edition 2024 forbids `&mut` on a `static
/// mut`, so each value is wrapped in an `UnsafeCell` like the Swift variable it replaces. The
/// harness is single threaded: a scene's `create` runs to completion before any reader.
struct BenchGlobal<T>(std::cell::UnsafeCell<T>);

// SAFETY: every access is on the benchmark main thread, never concurrently.
unsafe impl<T> Sync for BenchGlobal<T> {}

impl<T> BenchGlobal<T> {
    const fn new(value: T) -> Self {
        Self(std::cell::UnsafeCell::new(value))
    }

    /// SAFETY: the caller must not hold another reference to the same global at the same time.
    #[allow(clippy::mut_from_ref)]
    unsafe fn get_mut(&self) -> &mut T {
        unsafe { &mut *self.0.get() }
    }

    /// SAFETY: the caller must not hold a mutable reference to the same global at the same time.
    unsafe fn get_ref(&self) -> &T {
        unsafe { &*self.0.get() }
    }
}

/// Swift's `nonisolated(unsafe) private var spinnerMesh`. The scene owns the mesh for as long as
/// the world references it, so the handle is kept alive here until `destroy_spinner`.
static SPINNER_MESH: BenchGlobal<Option<TriangleMesh>> = BenchGlobal::new(None);

/// Swift's `nonisolated(unsafe) var spinnerProbeBodies: [BodyID]`.
static SPINNER_PROBE_BODIES: BenchGlobal<[BodyID; 2]> = BenchGlobal::new([BodyID::NULL; 2]);

pub fn spinner_probe_body(index: usize) -> BodyID {
    // SAFETY: the harness is single threaded; every write happens in `create_spinner` before any
    // reader runs.
    unsafe { SPINNER_PROBE_BODIES.get_ref()[index] }
}

fn create_inward_cylinder_mesh(
    radius: f32,
    half_depth: f32,
    y_center: f32,
    segment_count: usize,
) -> Option<TriangleMesh> {
    let vertex_count = 2 * segment_count + 2;
    let triangle_count = 4 * segment_count;

    let mut vertices = vec![Vec3::ZERO; vertex_count];
    let mut indices = vec![0i32; 3 * triangle_count];

    for i in 0..segment_count {
        let angle = 2.0 * scene_pi() * i as f32 / segment_count as f32;
        let cs = compute_cos_sin(angle);
        let x = radius * cs.cosine;
        let y = radius * cs.sine + y_center;
        vertices[2 * i] = Vec3::new(x, y, -half_depth);
        vertices[2 * i + 1] = Vec3::new(x, y, half_depth);
    }

    let center_negative = 2 * segment_count;
    let center_positive = 2 * segment_count + 1;
    vertices[center_negative] = Vec3::new(0.0, y_center, -half_depth);
    vertices[center_positive] = Vec3::new(0.0, y_center, half_depth);

    let mut index = 0usize;
    for i in 0..segment_count {
        let j = (i + 1) % segment_count;

        indices[index] = (2 * i) as i32;
        indices[index + 1] = (2 * i + 1) as i32;
        indices[index + 2] = (2 * j + 1) as i32;
        index += 3;

        indices[index] = (2 * i) as i32;
        indices[index + 1] = (2 * j + 1) as i32;
        indices[index + 2] = (2 * j) as i32;
        index += 3;

        indices[index] = center_negative as i32;
        indices[index + 1] = (2 * i) as i32;
        indices[index + 2] = (2 * j) as i32;
        index += 3;

        indices[index] = center_positive as i32;
        indices[index + 1] = (2 * j + 1) as i32;
        indices[index + 2] = (2 * i + 1) as i32;
        index += 3;
    }

    let mut options = MeshBuildOptions::default();
    options.identify_edges = true;
    TriangleMesh::new(&vertices, &indices, None, options)
}

pub fn create_spinner(world: &mut World) {
    let radius: f32 = 12.0;
    let half_depth: f32 = 1.0;
    let y_center: f32 = radius;

    let ground_id: BodyID;
    {
        let body_def = BodyDef::default();
        ground_id = world.create_body(&body_def);

        let Some(mesh) = create_inward_cylinder_mesh(radius, half_depth, y_center, 90) else {
            return;
        };
        // SAFETY: single threaded harness; the mesh must outlive every world reference taken
        // from it, and Swift kept it alive in the same file-scope variable.
        unsafe {
            *SPINNER_MESH.get_mut() = Some(mesh.clone());
        }

        let mut shape_def = ShapeDef::default();
        shape_def.base_material.friction = 0.1;
        let _ = world.create_mesh_shape(ground_id, &shape_def, mesh, Vec3::ONE);
    }

    {
        let half_length = 0.5 * radius;

        let mut body_def = BodyDef::default();
        body_def.type_ = BodyType::DynamicBody;
        body_def.position = Pos::new(0.0, (y_center - half_length) as f64, 0.0);
        body_def.enable_sleep = false;

        let spinner_id = world.create_body(&body_def);
        // SAFETY: see `spinner_probe_body`; written once during scene creation.
        unsafe {
            *SPINNER_PROBE_BODIES.get_mut() = [ground_id, spinner_id];
        }

        let paddle = make_box_hull(0.3, half_length + 0.1, half_depth + 0.1);
        let mut shape_def = ShapeDef::default();
        shape_def.base_material.friction = 0.0;
        let _ = world.create_hull_shape(spinner_id, &shape_def, &paddle);

        let mut joint_def = RevoluteJointDef::default();
        joint_def.base.body_id_a = ground_id;
        joint_def.base.body_id_b = spinner_id;
        joint_def.base.local_frame_a.p = Vec3::new(0.0, y_center - half_length, 0.0);
        joint_def.enable_motor = true;
        joint_def.motor_speed = 8.0;
        joint_def.max_motor_torque = f32::MAX;
        let _ = world.create_revolute_joint(&joint_def);
    }

    let sphere = Sphere::new(Vec3::ZERO, 0.2);

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = 0.1;
    shape_def.base_material.restitution = 0.1;
    shape_def.density = 0.25;

    let body_count = 1500i32;
    let spacing: f32 = 0.55;
    let y_start: f32 = 2.0;
    let d = y_center - y_start;
    let chord_half_width = (radius * radius - d * d).sqrt();
    let x_limit = chord_half_width - 2.0 * spacing;
    let z_limit = half_depth - 2.0 * sphere.radius;

    let mut x = -x_limit;
    let mut y = y_start;
    let mut z = -z_limit;

    for _ in 0..body_count {
        body_def.position = Pos::new(x as f64, y as f64, z as f64);
        let body_id = world.create_body(&body_def);
        let _ = world.create_sphere_shape(body_id, &shape_def, sphere);

        z += spacing;
        if z > z_limit {
            z = -z_limit;
            x += spacing;
            if x > x_limit {
                x = -x_limit;
                y += spacing;
            }
        }
    }
    record_counters(world);
}

pub fn destroy_spinner() {
    // SAFETY: single threaded harness; nothing borrows the mesh at this point because the world
    // that referenced it has already been dropped by the caller loop.
    unsafe {
        *SPINNER_MESH.get_mut() = None;
    }
}

// MARK: - sleep

const SLEEP_PYRAMID_COUNT: i32 = 10;

/// Swift's `nonisolated(unsafe) private var sleepBodyA` / `sleepBodyB`.
static SLEEP_BODY_A: BenchGlobal<[BodyID; SLEEP_PYRAMID_COUNT as usize]> =
    BenchGlobal::new([BodyID::NULL; SLEEP_PYRAMID_COUNT as usize]);
static SLEEP_BODY_B: BenchGlobal<[BodyID; SLEEP_PYRAMID_COUNT as usize]> =
    BenchGlobal::new([BodyID::NULL; SLEEP_PYRAMID_COUNT as usize]);

fn create_sleep_pyramid(
    world: &mut World,
    base_count: i32,
    extent: f32,
    center_x: f32,
    index: usize,
) {
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.sleep_threshold = 1.0;

    let shape_def = ShapeDef::default();
    let box_hull = make_box_hull(extent, extent, extent);

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * extent;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent + center_x;
            body_def.position = Pos::new(x as f64, y as f64, 0.0);
            let body_id = world.create_body(&body_def);
            let _ = world.create_hull_shape(body_id, &shape_def, &box_hull);

            if i == 0 && j == 0 {
                // SAFETY: single threaded scene construction writes each slot once.
                unsafe {
                    SLEEP_BODY_A.get_mut()[index] = body_id;
                }
            } else if i == 1 && j == 1 {
                // SAFETY: see above.
                unsafe {
                    SLEEP_BODY_B.get_mut()[index] = body_id;
                }
            }
        }
    }
}

pub fn create_sleep(world: &mut World) {
    // SAFETY: single threaded harness; the tables are reset before the scene records into them.
    unsafe {
        *SLEEP_BODY_A.get_mut() = [BodyID::NULL; SLEEP_PYRAMID_COUNT as usize];
        *SLEEP_BODY_B.get_mut() = [BodyID::NULL; SLEEP_PYRAMID_COUNT as usize];
    }

    let base_count = 20i32;
    let extent: f32 = 0.5;

    let base_width = 2.0 * extent * base_count as f32;
    let pitch = base_width + 8.0 * extent;
    let span = pitch * (SLEEP_PYRAMID_COUNT - 1) as f32;

    {
        let mut body_def = BodyDef::default();
        body_def.position = Pos::new(0.0, -1.0, 0.0);
        let ground = world.create_body(&body_def);
        let shape_def = ShapeDef::default();
        let _ = world.create_hull_shape(
            ground,
            &shape_def,
            &make_box_hull(0.5 * span + base_width, 1.0, 4.0),
        );
    }

    for i in 0..SLEEP_PYRAMID_COUNT {
        let center_x = -0.5 * span + i as f32 * pitch - 0.5 * base_width;
        create_sleep_pyramid(world, base_count, extent, center_x, i as usize);
    }
    record_counters(world);
}

pub fn step_sleep(world: &mut World, _step_count: usize) {
    for i in 0..SLEEP_PYRAMID_COUNT as usize {
        // SAFETY: the tables were written during scene creation and only read here.
        let (body_a, body_b) =
            unsafe { (SLEEP_BODY_A.get_ref()[i], SLEEP_BODY_B.get_ref()[i]) };
        if !world.body_is_awake(body_a) {
            let mut joint_def = FilterJointDef::default();
            joint_def.base.body_id_a = body_a;
            joint_def.base.body_id_b = body_b;
            let joint_id: JointID = world.create_filter_joint(&joint_def);
            // Swift recovered the low 32 bits of `stored` as the raw joint index.
            world.destroy_joint((joint_id.stored() >> 32) as u32 as i32 - 1, true);
            break;
        }
    }
}

// MARK: - trees

/// Swift's `nonisolated(unsafe) private var treeMesh: TriangleMesh?`.
static TREE_MESH: BenchGlobal<Option<TriangleMesh>> = BenchGlobal::new(None);

fn create_trees(world: &mut World, scale: i32) {
    // SAFETY: single threaded harness; the previous mesh is released before the new one is stored.
    unsafe {
        *TREE_MESH.get_mut() = None;
    }

    let tilt: f32 = 0.0;
    let mut body_def = BodyDef::default();
    body_def.position = Pos::new(0.0, 0.0, 0.0);
    body_def.rotation = make_quat_from_axis_angle(Vec3::new(1.0, 0.0, 0.0), tilt);
    let ground_id = world.create_body(&body_def);

    let x_count = scale * 150;
    let z_count = scale * 200;

    let cell_width = 1.0 / scale as f32;
    let amplitude: f32 = 0.4;
    let row_hz: f32 = 0.05;
    let column_hz: f32 = 0.1;

    let Some(mesh) = create_wave_mesh(x_count, z_count, cell_width, amplitude, row_hz, column_hz)
    else {
        return;
    };
    // SAFETY: see above.
    unsafe {
        *TREE_MESH.get_mut() = Some(mesh.clone());
    }
    let mut shape_def = ShapeDef::default();
    let _ = world.create_mesh_shape(ground_id, &shape_def, mesh, Vec3::ONE);

    body_def.type_ = BodyType::DynamicBody;
    body_def.sleep_threshold = 0.2;
    body_def.rotation = Quat::IDENTITY;

    let body_count = 50i32;

    shape_def.base_material.friction = 0.9;
    shape_def.base_material.rolling_resistance = 0.05;
    shape_def.update_body_mass = false;
    shape_def.density = 1.0;

    let hull_count = 22i32;
    let mut hulls: Vec<natsel::collision::hull::Hull> = Vec::new();

    let mut y: f32 = 1.0;
    let mut r: f32 = 0.75;
    let l: f32 = 1.5;
    for _ in 0..hull_count {
        if let Some(hull) = create_cylinder(l + 2.0 * r, r, y - r, 6) {
            hulls.push(hull);
        }
        y += l + 2.0 * r;
        r = 0.95 * r;
    }

    let mut angular_velocity: f32 = -0.5;
    let mut z: f32 = -70.0;
    let cs = compute_cos_sin(tilt);
    let y_tilt = cs.sine / cs.cosine;
    for body_index in 0..body_count {
        body_def.position = Pos::new(0.0, (1.0 - z * y_tilt) as f64, z as f64);
        let body_id = world.create_body(&body_def);

        for shape_index in 0..22usize {
            if shape_index < hulls.len() {
                let _ = world.create_hull_shape(body_id, &shape_def, &hulls[shape_index]);
            }
        }

        let velocity_scale = 0.5 + (0.5 * body_index as f32) / body_count as f32;
        world.apply_mass_from_shapes(body_id);
        let center = world.body_world_center(body_id);
        let omega = Vec3::new(0.0, 0.0, velocity_scale * angular_velocity);
        let v = cross(omega, sub_pos_to_vec3(center, body_def.position));
        world.set_body_angular_velocity(body_id, omega);
        world.set_body_linear_velocity(body_id, v);

        z += 3.0;
        angular_velocity = -angular_velocity;
    }
    record_counters(world);
}

/// Swift's `center - bodyDef.position` on a `Pos - Pos` pair, which yields a local `Vec3`.
fn sub_pos_to_vec3(a: Pos, b: Pos) -> Vec3 {
    Vec3::new(
        (a.x - b.x) as f32,
        (a.y - b.y) as f32,
        (a.z - b.z) as f32,
    )
}

pub fn create_trees25(world: &mut World) {
    create_trees(world, 4);
}

pub fn destroy_trees() {
    // SAFETY: single threaded harness; the world referencing the mesh is already gone.
    unsafe {
        *TREE_MESH.get_mut() = None;
    }
}

/// Nearest-rounded pi for scene construction; `f32::PI` rounds toward zero.
#[inline]
pub fn scene_pi() -> f32 {
    3.14159265359
}
