//! Ported from Tests/NatselTests/SoftBody/RayAccelerationTests.swift.

use crate::math::types::{Pos, Vec3};
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_runtime::SoftBodyDef;
use crate::soft_body::soft_body_settings::{SoftBodyBendType, SoftBodySettings, SoftBodyVertexAttributes};
use crate::types::shape_def::QueryFilter;

fn cloth(columns: i32, rows: i32, spacing: f32) -> SoftBodySettings {
    SoftBodySettings::make_cloth(
        columns,
        rows,
        spacing,
        SoftBodyVertexAttributes::default(),
        SoftBodyBendType::Distance,
        |_, _| false,
    )
}

#[test]
// "BVH rays match a linear scan including shared triangle edges"
fn oracle() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    let mut def = SoftBodyDef::default();
    def.settings = cloth(24, 24, 0.125);
    def.position = Pos::new(1.0e12 + 0.125, 0.0, -1.0e12);
    def.faces_double_sided = true;
    let id = world.create_soft_body(&def);

    for i in 0..160 {
        let origin = Pos::new(
            1.0e12 + f64::from((i % 20) as f32) * 0.125 - 1.0,
            2.0,
            -1.0e12 + f64::from((i / 20) as f32) * 0.125 - 0.5,
        );
        world.enable_soft_body_query_acceleration = false;
        let expected = world.cast_ray_soft_body(id, origin, Vec3::new(0.0, -4.0, 0.0));
        world.enable_soft_body_query_acceleration = true;
        let actual = world.cast_ray_soft_body(id, origin, Vec3::new(0.0, -4.0, 0.0));

        assert_eq!(actual.hit, expected.hit);
        assert_eq!(actual.face_index, expected.face_index);
        assert_eq!(actual.fraction, expected.fraction);
        assert_eq!(actual.normal, expected.normal);
        assert_eq!(actual.point, expected.point);
    }
}

#[test]
// "vertex edits, rollback, and slot reuse invalidate the face cache"
fn invalidation() {
    let mut world = World::default();
    let mut def = SoftBodyDef::default();
    def.settings = cloth(6, 6, 0.25);
    def.faces_double_sided = true;
    let id = world.create_soft_body(&def);

    let origin = Pos::new(0.0, 3.0, 0.0);
    let first = world.cast_ray_soft_body(id, origin, Vec3::new(0.0, -6.0, 0.0));
    assert!(first.hit);

    // The cache is keyed by the soft body's record id. The suite lives inside the crate, so it
    // reads the `pub(crate)` map directly, as Swift's `world.softBodyRayCaches` allowed.
    let cache_revision = world
        .soft_body_ray_caches
        .get(&world.soft_bodies[(id.index1 - 1) as usize].id)
        .map(|cache| cache.revision);
    assert!(cache_revision.is_some(), "a ray must populate the cache");
    let saved = world.save_state();

    let vertex_count = world.soft_body_vertex_count(id);
    for i in 0..vertex_count {
        let p = world.soft_body_vertex_position(id, i).expect("vertex position");
        world.set_soft_body_vertex_position(id, i, Pos::new(p.x, 1.0, p.z));
    }

    let moved = world.cast_ray_closest_soft_body(
        origin,
        Vec3::new(0.0, -6.0, 0.0),
        QueryFilter::default_filter(),
    );
    assert!(moved.hit);
    assert!((moved.point.y - 1.0).abs() < 0.00001);

    // Re-casting after the edits must have rebuilt the cache for the new vertex positions.
    let moved_revision = world
        .soft_body_ray_caches
        .get(&world.soft_bodies[(id.index1 - 1) as usize].id)
        .map(|cache| cache.revision);
    assert!(
        moved_revision != cache_revision,
        "vertex edits must invalidate the face cache"
    );

    assert!(world.restore_state(&saved));
    assert!(world.soft_body_ray_caches.is_empty());
    assert_eq!(
        world
            .cast_ray_soft_body(id, origin, Vec3::new(0.0, -6.0, 0.0))
            .point,
        first.point
    );

    world.destroy_soft_body(id);
    assert!(world.soft_body_ray_caches.is_empty());

    let mut reused_def = def;
    reused_def.position = Pos::new(0.0, -1.0, 0.0);
    let reused = world.create_soft_body(&reused_def);
    assert!(!world.soft_body_is_valid(id));
    assert!(
        (world
            .cast_ray_soft_body(reused, origin, Vec3::new(0.0, -6.0, 0.0))
            .point
            .y
            + 1.0)
            .abs()
            < 0.00001
    );
}
