//! Ported from Tests/NatselTests/Simulation/DeterminismScene.swift.
//!
//! Shared deterministic scenes for the repeatability suites. Every item is `pub(crate)` because
//! the sibling determinism/equivalence test modules (`DeterminismTests`, `ImpulseDeterminismTests`,
//! `RagdollDeterminismTests`, `WideEquivalenceTests`) draw on the same scene builders and hashers.

use crate::base::hash::{hash, hash_bytes};
use crate::collision::height_field::factories::create_wave_height_field;
use crate::collision::height_field::HeightField;
use crate::collision::hull::{make_box_hull, Hull};
use crate::collision::hull_builder::create_rock;
use crate::collision::mesh::procedural::create_wave_mesh;
use crate::collision::mesh::storage::TriangleMesh;
use crate::math::functions::{compute_cos_sin, offset_pos};
use crate::math::types::{AABB, Pos, Quat, Vec3, WorldTransform};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::ids::BodyID;
use crate::types::queries::{ShapeCastInput, ShapeProxy};
use crate::types::shape_def::{QueryFilter, ShapeDef};

pub(crate) struct SceneRandom {
    pub seed: u32,
}

impl SceneRandom {
    pub(crate) const fn new(seed: u32) -> Self {
        Self { seed }
    }

    /// XorShift32, mapped to `[0, 32767]`.
    pub(crate) fn next_int(&mut self) -> usize {
        let mut x = self.seed;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.seed = x;
        (x % 32_768) as usize
    }

    pub(crate) fn float_range(&mut self, lo: f32, hi: f32) -> f32 {
        let mut r = (self.next_int() & 32_767) as f32;
        r /= 32_767.0;
        r = (hi - lo) * r + lo;
        r
    }

    pub(crate) fn vec3_uniform(&mut self, lo: f32, hi: f32) -> Vec3 {
        Vec3::new(
            self.float_range(lo, hi),
            self.float_range(lo, hi),
            self.float_range(lo, hi),
        )
    }

    pub(crate) fn unit_vector(&mut self) -> Vec3 {
        let u1 = self.float_range(0.0, 1.0);
        let u2 = self.float_range(0.0, 2.0 * crate::base::constants::PI);
        let u3 = self.float_range(0.0, 2.0 * crate::base::constants::PI);
        let sqrt1_minus_u1 = (1.0 - u1).sqrt();
        let sqrt_u1 = u1.sqrt();
        let cs2 = compute_cos_sin(u2);
        let cs3 = compute_cos_sin(u3);
        Vec3::new(
            sqrt1_minus_u1 * cs2.sine,
            sqrt1_minus_u1 * cs2.cosine,
            sqrt_u1 * cs3.sine,
        )
    }

    pub(crate) fn quat(&mut self) -> Quat {
        let u1 = self.float_range(0.0, 1.0);
        let u2 = self.float_range(0.0, 2.0 * crate::base::constants::PI);
        let u3 = self.float_range(0.0, 2.0 * crate::base::constants::PI);
        let sqrt1_minus_u1 = (1.0 - u1).sqrt();
        let sqrt_u1 = u1.sqrt();
        let cs2 = compute_cos_sin(u2);
        let cs3 = compute_cos_sin(u3);
        Quat::new(
            Vec3::new(
                sqrt1_minus_u1 * cs2.sine,
                sqrt1_minus_u1 * cs2.cosine,
                sqrt_u1 * cs3.sine,
            ),
            sqrt_u1 * cs3.cosine,
        )
    }
}

// MARK: - Transform hashing

/// Hash a `WorldTransform` by its raw bytes, matching Swift's
/// `withUnsafeBytes(of: &transform) { hash(hash, $0.baseAddress!, Int32($0.count)) }`.
pub(crate) fn hash_transform(hash: u32, transform: &WorldTransform) -> u32 {
    hash_bytes(hash, transform)
}

// MARK: - Wave pile

pub(crate) struct WavePileScene {
    pub bodies: Vec<BodyID>,
    pub height_field: HeightField,
    pub random: SceneRandom,
}

impl WavePileScene {
    pub(crate) const BODY_COUNT: usize = 100;
}

pub(crate) fn create_wave_pile(world: &mut World) -> WavePileScene {
    let field_count = 21usize;
    let field_scale = Vec3::new(1.0, 0.6, 1.0);
    let height_field = create_wave_height_field(
        field_count,
        field_count,
        field_scale,
        0.08,
        0.06,
        false,
    )
    .expect("wave height field");
    let mut scene = WavePileScene {
        bodies: Vec::new(),
        height_field: height_field.clone(),
        random: SceneRandom::new(52_977),
    };

    let extent = field_scale.x * (field_count - 1) as f32;
    let mut ground_def = BodyDef::default();
    ground_def.position = Pos::new(
        f64::from(-0.5 * extent),
        0.0,
        f64::from(-0.5 * extent),
    );
    let ground = world.create_body(&ground_def);
    let _ = world.create_height_field_shape(ground, &ShapeDef::default(), height_field);

    let rock: Hull = create_rock(0.55).expect("rock hull");
    let box_hull = make_box_hull(0.45, 0.3, 0.55);
    let sphere = Sphere::new(Vec3::ZERO, 0.5);
    let capsule = Capsule::new(
        Vec3::new(0.0, -0.3, 0.0),
        Vec3::new(0.0, 0.3, 0.0),
        0.35,
    );

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.rolling_resistance = 0.3;

    let spacing: f32 = 1.7;
    let mut index = 0usize;
    for layer in 0..4usize {
        for i in 0..5usize {
            for j in 0..5usize {
                let jitter = scene.random.vec3_uniform(-0.3, 0.3);
                body_def.position.x = f64::from(spacing * (i as f32 - 0.5 * 4.0) + jitter.x);
                body_def.position.y =
                    f64::from(2.5 + 1.6 * layer as f32 + 0.3 * jitter.y);
                body_def.position.z = f64::from(spacing * (j as f32 - 0.5 * 4.0) + jitter.z);
                body_def.rotation = scene.random.quat();

                let body = world.create_body(&body_def);
                scene.bodies.push(body);

                match index % 4 {
                    0 => {
                        let _ = world.create_sphere_shape(body, &shape_def, &sphere);
                    }
                    1 => {
                        let _ = world.create_capsule_shape(body, &shape_def, &capsule);
                    }
                    2 => {
                        let _ = world.create_hull_shape(body, &shape_def, &box_hull);
                    }
                    _ => {
                        let _ = world.create_hull_shape(body, &shape_def, &rock);
                    }
                }
                index += 1;
            }
        }
    }
    scene
}

pub(crate) fn update_wave_pile(
    world: &World,
    scene: &WavePileScene,
    step_count: &mut i32,
    sleep_step: &mut i32,
    hash: &mut u32,
) -> bool {
    if *hash == 0 && world.awake_body_count() == 0 {
        *hash = crate::base::constants::HASH_INIT;
        for body in &scene.bodies {
            *hash = hash_transform(*hash, &world.body_transform(*body));
        }
        *sleep_step = *step_count;
    }
    *step_count += 1;
    *hash != 0
}

// MARK: - Mesh drop

pub(crate) struct MeshDropScene {
    #[allow(dead_code)]
    pub mesh: TriangleMesh,
    pub bodies: Vec<BodyID>,
}

impl MeshDropScene {
    pub(crate) const GRID_COUNT: usize = 20;
}

pub(crate) fn create_mesh_drop(world: &mut World, origin: Pos) -> MeshDropScene {
    let mesh = create_wave_mesh(40, 40, 1.0, 0.5, 0.1, 0.2).expect("wave mesh");
    let ground = world.create_body(&BodyDef::default());
    let mut ground_shape = ShapeDef::default();
    ground_shape.filter.category_bits = 1;
    let _ = world.create_mesh_shape(ground, &ground_shape, mesh.clone(), Vec3::ONE);

    let box_hull = make_box_hull(0.02, 0.2, 0.04);
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.rolling_resistance = 0.1;
    shape_def.filter.category_bits = 2;
    shape_def.filter.mask_bits = 1;

    let mut random = SceneRandom::new(3_963_634_789);
    let mut scene = MeshDropScene {
        mesh,
        bodies: Vec::new(),
    };
    for i in 0..MeshDropScene::GRID_COUNT {
        for j in 0..MeshDropScene::GRID_COUNT {
            body_def.linear_velocity = random.vec3_uniform(-1.0, 1.0);
            body_def.angular_velocity = random.vec3_uniform(-5.0, 5.0);
            body_def.position = offset_pos(
                origin,
                Vec3::new(
                    0.5 * (i as f32 - 0.5 * MeshDropScene::GRID_COUNT as f32),
                    5.0,
                    0.5 * (j as f32 - 0.5 * MeshDropScene::GRID_COUNT as f32),
                ),
            );
            let body = world.create_body(&body_def);
            scene.bodies.push(body);
            let _ = world.create_hull_shape(body, &shape_def, &box_hull);
        }
    }
    scene
}

pub(crate) fn update_mesh_drop(
    world: &World,
    scene: &MeshDropScene,
    step_count: &mut i32,
    sleep_step: &mut i32,
    hash: &mut u32,
) -> bool {
    if *hash == 0 && world.awake_body_count() == 0 {
        *hash = crate::base::constants::HASH_INIT;
        for body in &scene.bodies {
            *hash = hash_transform(*hash, &world.body_transform(*body));
        }
        *sleep_step = *step_count;
    }
    *step_count += 1;
    *hash != 0
}

// MARK: - Query spawn

pub(crate) struct QuerySpawnScene {
    pub bodies: Vec<BodyID>,
    pub spawn_count: i32,
    pub query_hit_count: i32,
    pub query_hash: u32,
}

impl QuerySpawnScene {
    pub(crate) const COUNT: i32 = 50;
    pub(crate) const CAST_RADIUS: f32 = 0.5;
}

impl Default for QuerySpawnScene {
    fn default() -> Self {
        Self {
            bodies: Vec::new(),
            spawn_count: 0,
            query_hit_count: 0,
            query_hash: 0,
        }
    }
}

pub(crate) fn create_query_spawn(world: &mut World) -> QuerySpawnScene {
    world.gravity = Vec3::ZERO;
    QuerySpawnScene::default()
}

pub(crate) fn query_spawn_once(
    world: &mut World,
    scene: &mut QuerySpawnScene,
    random: &mut SceneRandom,
) {
    let filter = QueryFilter::default_filter();

    let ray_origin = Pos::new(
        f64::from(random.float_range(-12.0, 12.0)),
        f64::from(random.float_range(-12.0, 12.0)),
        f64::from(random.float_range(-12.0, 12.0)),
    );
    let ray_translation = random.unit_vector() * 30.0;

    let spawn_position: Pos;
    let ray = world.cast_ray_closest(ray_origin, ray_translation, filter.clone());
    if ray.hit {
        scene.query_hit_count += 1;
        let fraction = ray.fraction;
        scene.query_hash = hash_bytes(scene.query_hash, &fraction);
        let normal = ray.normal;
        scene.query_hash = hash_bytes(scene.query_hash, &normal);
        spawn_position = offset_pos(ray.point, ray.normal * 1.2);
    } else {
        spawn_position = Pos::new(
            f64::from(random.float_range(-6.0, 6.0)),
            f64::from(random.float_range(-6.0, 6.0)),
            f64::from(random.float_range(-6.0, 6.0)),
        );
    }

    let center = random.vec3_uniform(-10.0, 10.0);
    let extent = random.float_range(1.0, 4.0);
    let aabb = AABB::new(
        Vec3::new(center.x - extent, center.y - extent, center.z - extent),
        Vec3::new(center.x + extent, center.y + extent, center.z + extent),
    );

    let mut overlap_count = 0i32;
    let _ = world.overlap_aabb(aabb, filter.clone(), |id| {
        overlap_count += 1;
        scene.query_hit_count += 1;
        let index1 = id.index1;
        scene.query_hash = hash_bytes(scene.query_hash, &index1);
        true
    });

    let mut fraction: f32 = 1.0;
    let proxy_point = Vec3::ZERO;
    let proxy = ShapeProxy::new(
        Some(&proxy_point as *const Vec3),
        1,
        QuerySpawnScene::CAST_RADIUS,
    );
    let mut cast_input = ShapeCastInput::default();
    cast_input.proxy = proxy;
    cast_input.translation = ray_translation;
    cast_input.max_fraction = 1.0;
    let _ = world.shape_cast(ray_origin, cast_input, filter.clone(), |_, _, _, cast_fraction, _, _, _| {
        fraction = cast_fraction;
        cast_fraction
    });
    if fraction < 1.0 {
        scene.query_hit_count += 1;
        let value = fraction;
        scene.query_hash = hash_bytes(scene.query_hash, &value);
    }

    let size: f32 = 0.3 + 0.2 * fraction;

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = spawn_position;
    body_def.rotation = random.quat();
    body_def.linear_velocity = random.vec3_uniform(-0.2, 0.2);
    body_def.angular_velocity = random.vec3_uniform(-0.5, 0.5);
    body_def.linear_damping = 1.0;
    body_def.angular_damping = 1.0;

    let body = world.create_body(&body_def);

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.rolling_resistance = 0.2;

    match (scene.spawn_count + overlap_count) % 3 {
        0 => {
            let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::ZERO, size));
        }
        1 => {
            let _ = world.create_capsule_shape(
                body,
                &shape_def,
                &Capsule::new(
                    Vec3::new(0.0, -size, 0.0),
                    Vec3::new(0.0, size, 0.0),
                    0.7 * size,
                ),
            );
        }
        _ => {
            let _ = world.create_hull_shape(
                body,
                &shape_def,
                &make_box_hull(size, 0.7 * size, 0.5 * size),
            );
        }
    }

    scene.bodies.push(body);
    scene.spawn_count += 1;
}

pub(crate) fn update_query_spawn(
    world: &mut World,
    scene: &mut QuerySpawnScene,
    random: &mut SceneRandom,
    step_count: &mut i32,
    sleep_step: &mut i32,
    hash: &mut u32,
) -> bool {
    if scene.spawn_count < QuerySpawnScene::COUNT {
        query_spawn_once(world, scene, random);
    } else if *hash == 0 && world.awake_body_count() == 0 {
        *hash = crate::base::constants::HASH_INIT;
        for body in &scene.bodies {
            *hash = hash_transform(*hash, &world.body_transform(*body));
        }
        *sleep_step = *step_count;
    }
    *step_count += 1;
    *hash != 0
}

/// Re-export so sibling modules can hash a single `f32`/`i32`/`Vec3` field exactly like Swift's
/// `withUnsafeBytes(of:)` + `hash(...)` pair.
pub(crate) fn hash_float(hash: u32, value: f32) -> u32 {
    hash_bytes(hash, &value)
}

/// Swift's bare `hash(_:_:_:)`.
pub(crate) fn hash_bytes_raw(hash: u32, data: &[u8]) -> u32 {
    hash(hash, data)
}
