//! Ported from Tests/NatselTests/Simulation/DoublePrecisionWorldTests.swift.

use crate::character::character_virtual::{CharacterDef, CharacterGroundState, CharacterVirtual};
use crate::collision::hull::make_offset_box_hull;
use crate::math::functions::{lerp_position, offset_aabb, offset_pos, sub_pos};
use crate::math::types::{AABB, Pos, Quat, Vec3};
use crate::math::world_aabb::WorldAABB;
use crate::simulation::World;
use crate::soft_body::soft_body_runtime::SoftBodyDef;
use crate::soft_body::soft_body_settings::{
    SoftBodyBendType, SoftBodySettings, SoftBodyVertexAttributes,
};
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::joint_defs::DistanceJointDef;
use crate::types::shape_def::{QueryFilter, ShapeDef};

// "world position boundaries retain small offsets"
#[test]
fn positions() {
    for offset in [1.0e9_f64, -1.0e9_f64, 1.0e12_f64] {
        let p = Pos::new(offset + 0.125, offset - 0.25, offset + 0.5);
        let d = Vec3::new(0.25, -0.125, 0.5);
        assert_eq!(sub_pos(offset_pos(p, d), p), d);
        assert_eq!(
            lerp_position(p, offset_pos(p, d), 0.5),
            offset_pos(p, 0.5 * d)
        );
        let box_ = offset_aabb(
            AABB::new(
                Vec3::new(-0.1, -0.2, -0.3),
                Vec3::new(0.1, 0.2, 0.3),
            ),
            p,
        );
        assert!(f64::from(box_.lower_bound.x) <= p.x - 0.1);
        assert!(f64::from(box_.upper_bound.x) >= p.x + 0.1);
    }
}

// "queries, integration and rollback retain fractional coordinates"
#[test]
fn queries_and_rollback() {
    for offset in [1.0e9_f64, -1.0e9_f64, 1.0e12_f64] {
        let mut world = World::default();
        world.gravity = Vec3::ZERO;
        let mut def = BodyDef::default();
        def.position = Pos::new(offset + 0.125, offset, offset);
        let ground = world.create_body(&def);
        let shape = world.create_sphere_shape(ground, &ShapeDef::default(), Sphere::new(Vec3::ZERO, 0.5));
        let start = Pos::new(offset - 2.0, offset, offset);
        let result = world.cast_ray_closest(start, Vec3::new(4.0, 0.0, 0.0), QueryFilter::default_filter());
        assert!(result.hit);
        assert!((result.point.x - (offset - 0.375)).abs() < 0.001);
        let single = world.shape_ray_cast(shape, start, Vec3::new(4.0, 0.0, 0.0));
        assert!(single.hit);
        assert_eq!(single.point, result.point);
        let closest = world.shape_closest_point(shape, Pos::new(offset + 2.0, offset, offset));
        assert!((closest.x - (offset + 0.625)).abs() < 0.001);
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(offset, offset + 3.0, offset);
        def.linear_velocity = Vec3::new(0.125, 0.0, 0.0);
        def.enable_sleep = false;
        let body = world.create_body(&def);
        let _ = world.create_sphere_shape(body, &ShapeDef::default(), Sphere::new(Vec3::ZERO, 0.1));
        let state = world.save_state();
        for _ in 0..60 {
            world.step(1.0 / 60.0, 4);
        }
        let position = world.body_position(body);
        assert!((position.x - offset - 0.125).abs() < 0.01);
        let hash = world.world_state_hash();
        assert!(world.restore_state(&state));
        for _ in 0..60 {
            world.step(1.0 / 60.0, 4);
        }
        assert_eq!(world.world_state_hash(), hash);
    }
}

// "rigid and soft contact remain stable far from the origin"
#[test]
fn contacts() {
    let mut world = World::default();
    let origin = Pos::new(1.0e9 + 0.125, 1.0e9, -1.0e9);
    let mut def = BodyDef::default();
    def.position = origin;
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(10.0, 1.0, 10.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    def.type_ = BodyType::DynamicBody;
    def.position = offset_pos(origin, Vec3::new(0.0, 3.0, 0.0));
    let body = world.create_body(&def);
    let _ = world.create_sphere_shape(body, &ShapeDef::default(), Sphere::new(Vec3::ZERO, 0.5));
    // Swift defaulted `attributes` to `SoftBodyVertexAttributes()` and `bendType` to `.distance`;
    // both are passed explicitly because they change the constraints that get built.
    let mut soft = SoftBodyDef::with_settings(
        SoftBodySettings::make_cloth(
            3,
            3,
            0.25,
            SoftBodyVertexAttributes::default(),
            SoftBodyBendType::Distance,
            |_, _| false,
        ),
        Pos::ZERO,
        Quat::IDENTITY,
    );
    soft.position = offset_pos(origin, Vec3::new(2.0, 2.0, 0.0));
    let cloth = world.create_soft_body(&soft);
    for _ in 0..180 {
        world.step(1.0 / 60.0, 4);
    }
    assert!((world.body_position(body).y - origin.y - 0.5).abs() < 0.03);
    for p in world.soft_body_vertex_positions(cloth) {
        assert!(p.y - origin.y > -0.05);
        assert!(p.y - origin.y < 0.15);
    }
}

// "precise bounds reject float broad-phase false positives"
#[test]
fn precise_bounds() {
    let mut world = World::default();
    let origin = Pos::new(1.0e12 + 0.125, 1.0e12, -1.0e12);
    let mut def = BodyDef::default();
    def.position = origin;
    let body = world.create_body(&def);
    let shape = world.create_sphere_shape(body, &ShapeDef::default(), Sphere::new(Vec3::ZERO, 0.25));
    let bounds = world.shape_world_aabb(shape);
    assert!(bounds.contains(offset_pos(origin, Vec3::new(0.25, 0.0, 0.0))));
    assert_eq!(bounds, world.body_world_aabb(body));
    let mut hits = 0;
    let _ = world.overlap_world_aabb(
        WorldAABB::from_local_bounds(
            AABB::new(
                Vec3::new(-0.1, -0.1, -0.1),
                Vec3::new(0.1, 0.1, 0.1),
            ),
            origin,
        ),
        QueryFilter::default_filter(),
        |_| {
            hits += 1;
            true
        },
    );
    assert_eq!(hits, 1);
    hits = 0;
    let _ = world.overlap_world_aabb(
        WorldAABB::from_local_bounds(
            AABB::new(Vec3::new(1.0, 1.0, 1.0), Vec3::new(2.0, 2.0, 2.0)),
            origin,
        ),
        QueryFilter::default_filter(),
        |_| {
            hits += 1;
            true
        },
    );
    assert_eq!(hits, 0);
}

// "joint anchors and character ground queries work at distant coordinates"
#[test]
fn distant_systems() {
    let mut world = World::default();
    let origin = Pos::new(1.0e9 + 0.125, -1.0e9, 1.0e9 + 0.375);
    let mut def = BodyDef::default();
    def.position = origin;
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(20.0, 1.0, 20.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    def.type_ = BodyType::DynamicBody;
    def.position = offset_pos(origin, Vec3::new(0.0, 4.0, 0.0));
    let body = world.create_body(&def);
    let _ = world.create_sphere_shape(body, &ShapeDef::default(), Sphere::new(Vec3::ZERO, 0.2));
    let mut joint = DistanceJointDef::default();
    joint.base.body_id_a = ground;
    joint.base.body_id_b = body;
    joint.base.local_frame_a.p = Vec3::new(0.0, 5.0, 0.0);
    joint.length = 1.0;
    let _ = world.create_distance_joint(&joint);
    let mut character_def = CharacterDef::new();
    character_def.position = offset_pos(origin, Vec3::new(3.0, 0.01, 0.0));

    // Swift's `CharacterVirtual` holds the `World` class by reference, so the test could step the
    // world and re-read its gravity between character updates. The Rust character borrows the
    // world mutably for its whole life, so the same interleaving goes through a raw pointer.
    // SAFETY: `world_ptr` comes from `world`, and the two phases never overlap: `step` finishes
    // (releasing its borrow) before `extended_update` starts and vice versa.
    let world_ptr: *mut World = &mut world;
    let mut character = CharacterVirtual::new(unsafe { &mut *world_ptr }, &character_def)
        .expect("character creation failed");
    for _ in 0..120 {
        unsafe { (*world_ptr).step(1.0 / 60.0, 4) };
        character.linear_velocity = Vec3::new(
            0.0,
            character.linear_velocity.y - 10.0 / 60.0,
            0.0,
        );
        let gravity = unsafe { (*world_ptr).gravity };
        character.extended_update(1.0 / 60.0, gravity);
    }
    assert!((world.body_position(body).y - origin.y - 4.0).abs() < 0.01);
    assert_eq!(character.ground_state(), CharacterGroundState::OnGround);
    assert!((character.position().y - origin.y).abs() < 0.05);
}

// "fluid volume keeps its precise surface height far from the origin"
#[test]
fn distant_fluid() {
    let mut world = World::default();
    let origin = Pos::new(1.0e12 + 0.125, 1.0e12, -1.0e12);
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = origin;
    let body = world.create_body(&def);
    let _ = world.create_sphere_shape(body, &ShapeDef::default(), Sphere::new(Vec3::ZERO, 0.5));
    let submerged = world.body_submerged_volume(body, origin, Vec3::AXIS_Y);
    assert!((submerged.submerged_volume / submerged.total_volume - 0.5).abs() < 1.0e-5);
    let bounds = WorldAABB::from_local_bounds(
        AABB::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 0.0, 1.0)),
        origin,
    );
    assert_eq!(
        world.apply_buoyancy(
            bounds,
            Vec3::AXIS_Y,
            2.0,
            0.0,
            0.0,
            Vec3::ZERO,
            1.0 / 60.0,
            QueryFilter::default_filter(),
        ),
        1
    );
    assert!(world.body_linear_velocity(body).y > 0.0);
}
