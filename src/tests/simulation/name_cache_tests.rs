//! Ported from Tests/NatselTests/Simulation/NameCacheTests.swift.

use crate::containers::name_cache::{NameCache, NULL_NAME};
use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::shape_def::ShapeDef;

#[test]
fn cache_unit() {
    let mut cache = NameCache::new();

    // Empty and nil map to the null id, which resolves to no string.
    assert_eq!(cache.add(Some("")), NULL_NAME);
    assert_eq!(cache.add(None), NULL_NAME);
    assert_eq!(cache.find(NULL_NAME), None);

    // Distinct strings get distinct ids and round-trip.
    let crate_id = cache.add(Some("crate"));
    let barrel_id = cache.add(Some("barrel"));
    assert_ne!(crate_id, NULL_NAME);
    assert_ne!(barrel_id, NULL_NAME);
    assert_ne!(crate_id, barrel_id);
    assert_eq!(cache.find(crate_id), Some("crate"));
    assert_eq!(cache.find(barrel_id), Some("barrel"));

    // Re-interning the same bytes returns the same id.
    assert_eq!(cache.add(Some("crate")), crate_id);

    // A name far longer than any inline buffer round-trips exactly.
    let long_name = "abcdefghijklmnopqrstuvwxyz".repeat(150);
    let long_id = cache.add(Some(&long_name));
    assert_eq!(cache.find(long_id), Some(long_name.as_str()));

    // A missing id resolves to nil, and to the caller's default.
    assert_eq!(cache.find(12345), None);
    assert_eq!(cache.find(12345).unwrap_or("fallback"), "fallback");
}

#[test]
fn world_names() {
    let body_names = [
        "crate",
        "barrel",
        "crate",
        "a_very_long_body_name_that_exceeds_any_inline_name_buffer",
    ];
    let shape_names = [
        "box_hull",
        "box_hull",
        "crate",
        "a_very_long_shape_name_that_never_fit_the_old_inline_buffer",
    ];

    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -10.0, 0.0);

    let mut ground_def = BodyDef::default();
    ground_def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&ground_def);
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(20.0, 1.0, 20.0));

    let mut bodies = Vec::new();
    let mut shapes = Vec::new();
    for i in 0..body_names.len() {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, 1.0 + 1.1 * i as f64, 0.0);
        def.name = Some(body_names[i].to_string());
        let body = world.create_body(&def);

        let mut shape_def = ShapeDef::default();
        shape_def.density = 1.0;
        shape_def.name = Some(shape_names[i].to_string());
        let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

        bodies.push(body);
        shapes.push(shape);
    }

    for i in 0..body_names.len() {
        assert_eq!(world.body_name(bodies[i]), Some(body_names[i]));
        assert_eq!(world.shape_name(shapes[i]), Some(shape_names[i]));
    }

    // Shape names intern independently: a shape may share a body's name.
    assert_eq!(world.body_name(bodies[0]), Some("crate"));
    assert_eq!(world.shape_name(shapes[2]), Some("crate"));

    // Setting a name replaces it; clearing removes it.
    world.set_body_name(bodies[0], Some("renamed"));
    assert_eq!(world.body_name(bodies[0]), Some("renamed"));

    world.set_body_name(bodies[0], None);
    assert_eq!(world.body_name(bodies[0]), None);

    // The unnamed ground stays unnamed.
    assert_eq!(world.body_name(ground), None);
}

#[test]
fn names_survive_stepping() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 3.0, 0.0);
    def.name = Some("falling_box".to_string());
    let box_body = world.create_body(&def);

    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    shape_def.name = Some("box_shape".to_string());
    let shape = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    for _ in 0..60 {
        world.step(1.0f32 / 60.0, 4);
    }

    assert_eq!(world.body_name(box_body), Some("falling_box"));
    assert_eq!(world.shape_name(shape), Some("box_shape"));
}
