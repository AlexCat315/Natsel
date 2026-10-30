//! Ported from Tests/NatselTests/Simulation/ExplosionTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;
use crate::types::world_def::ExplosionDef;

fn make_scene() -> (World, Vec<BodyID>) {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut ids: Vec<BodyID> = Vec::new();
    for i in 0..5 {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(f64::from(i as f64 * 2.0 - 4.0), 0.0, 0.0);
        let body = world.create_body(&def);

        let mut shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        if i % 2 == 0 {
            let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::ZERO, 0.5));
        } else {
            let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
        }
        ids.push(body);
    }
    (world, ids)
}

/// Swift's FNV-1a feed over the raw bytes of one scalar/vector value.
fn feed(mut hash: u64, bytes: &[u8]) -> u64 {
    for byte in bytes {
        hash = (hash ^ u64::from(*byte)).wrapping_mul(1_099_511_628_211);
    }
    hash
}

// "radial impulse is repeatable"
#[test]
fn explosion() {
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let (mut world, ids) = make_scene();

        let mut def = ExplosionDef::default();
        def.position = Pos::ZERO;
        def.radius = 3.0;
        def.falloff = 2.0;
        def.impulse_per_area = 50.0;
        world.explode(&def);

        let mut hash: u64 = 14_695_981_039_346_656_037;

        for id in &ids {
            hash = feed(hash, &world.body_linear_velocity(*id).x.to_ne_bytes());
            hash = feed(hash, &world.body_linear_velocity(*id).y.to_ne_bytes());
            hash = feed(hash, &world.body_linear_velocity(*id).z.to_ne_bytes());
            hash = feed(hash, &world.body_angular_velocity(*id).x.to_ne_bytes());
            hash = feed(hash, &world.body_angular_velocity(*id).y.to_ne_bytes());
            hash = feed(hash, &world.body_angular_velocity(*id).z.to_ne_bytes());
        }

        samples.push(hash);

        // Bodies left of the centre are pushed away from it, symmetric about the origin.
        assert!(world.body_linear_velocity(ids[0]).x < 0.0);
        assert!(world.body_linear_velocity(ids[2]).x > 0.0);
        assert!(world.body_linear_velocity(ids[4]).x > 0.0);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

// "shapes beyond radius plus falloff are untouched"
#[test]
fn out_of_range() {
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let (mut world, ids) = make_scene();

        let mut def = ExplosionDef::default();
        def.position = Pos::ZERO;
        def.radius = 0.1;
        def.falloff = 0.1;
        def.impulse_per_area = 50.0;
        world.explode(&def);

        // Only the body at the explosion centre is within range; the others are far outside.
        let velocities: Vec<Vec3> = ids
            .iter()
            .map(|id| world.body_linear_velocity(*id))
            .collect();
        assert_eq!(velocities[0], Vec3::ZERO);
        assert_eq!(velocities[1], Vec3::ZERO);
        samples.push(u64::from(velocities[2].x.to_bits()));
        assert_eq!(velocities[3], Vec3::ZERO);
        assert_eq!(velocities[4], Vec3::ZERO);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

// "zero explosion scale opts a shape out"
#[test]
fn zero_scale() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    shape_def.explosion_scale = 0.0;
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::ZERO, 0.5));

    let mut explosion = ExplosionDef::default();
    explosion.position = Pos::ZERO;
    explosion.radius = 5.0;
    explosion.falloff = 1.0;
    explosion.impulse_per_area = 100.0;
    world.explode(&explosion);

    assert_eq!(world.body_linear_velocity(body), Vec3::ZERO);
}
