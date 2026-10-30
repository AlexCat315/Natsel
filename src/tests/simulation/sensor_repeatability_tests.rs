//! Ported from Tests/NatselTests/Simulation/SensorRepeatabilityTests.swift.

use crate::math::functions::offset_pos;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::tests::simulation::mixed_query_scene::make_test_query_scene;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

/// `Tests/NatselTests/Collision/CollisionDigest.swift` as this file uses it: the same FNV-1a
/// seed and prime, and the same little-endian byte fold over a `UInt32` word.
struct CollisionDigest {
    value: u64,
}

impl CollisionDigest {
    const SEED: u64 = 14_695_981_039_346_656_037;

    fn new() -> Self {
        Self { value: Self::SEED }
    }

    fn add_word(&mut self, word: u32) {
        for byte in 0..4 {
            self.value ^= ((word >> (8 * byte)) & 255) as u64;
            self.value = self.value.wrapping_mul(1_099_511_628_211);
        }
    }
}

// "all six sensor geometry types preserve ordered begin/end events", arguments: 0..<2
fn run_repeatability(scenario: i32) {
    fn snapshot(scenario: i32) -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let (mut world, origin) = make_test_query_scene(scenario, true);

        let mut body_def = BodyDef::default();
        body_def.type_ = BodyType::KinematicBody;
        let visitor = world.create_body(&body_def);

        let mut shape_def = ShapeDef::default();
        shape_def.enable_sensor_events = true;
        let _ = world.create_sphere_shape(visitor, &shape_def, &Sphere::new(Vec3::ZERO, 0.3));

        let mut digest = CollisionDigest::new();
        for i in 0..120 {
            let position = Vec3::new(
                ((i * 7) % 19) as f32 * 0.25 - 1.0,
                (i % 9) as f32 * 0.25 - 0.5,
                ((i * 11) % 17) as f32 * 0.25 - 0.75,
            );
            world.set_body_transform(
                visitor,
                offset_pos(origin, position),
                crate::math::types::Quat::IDENTITY,
            );
            world.step(1.0f32 / 60.0, 4);
            let events = world.sensor_events();
            digest.add_word(events.begin.len() as u32);
            digest.add_word(events.end.len() as u32);
            for event in &events.begin {
                digest.add_word(event.sensor_shape_id.index1 as u32);
                digest.add_word(event.visitor_shape_id.index1 as u32);
            }
            for event in &events.end {
                digest.add_word(event.sensor_shape_id.index1 as u32);
                digest.add_word(event.visitor_shape_id.index1 as u32);
            }
        }
        samples.push(digest.value);
        samples
    }

    let first = snapshot(scenario);
    let replay = snapshot(scenario);
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn repeatability_0() {
    run_repeatability(0);
}

#[test]
fn repeatability_1() {
    run_repeatability(1);
}

// "removing a sensor preserves the moved sensor index and subsequent destruction"
#[test]
fn swap_removal() {
    let mut world = World::default();
    let body = world.create_body(&BodyDef::default());
    let mut definition = ShapeDef::default();
    definition.is_sensor = true;
    let mut ids = Vec::new();
    for _ in 0..4 {
        ids.push(world.create_sphere_shape(
            body,
            &definition,
            &Sphere::new(Vec3::ZERO, 1.0),
        ));
    }
    world.destroy_shape(ids[0], true);
    for (index, sensor) in world.sensors.iter().enumerate() {
        assert_eq!(
            world.shapes[sensor.shape_id as usize].sensor_index,
            index as i32
        );
    }
    world.destroy_shape(ids[2], true);
    world.destroy_shape(ids[3], true);
    assert_eq!(world.sensors.len(), 1);
    assert_eq!(world.shape(ids[1]).sensor_index, 0);
}

// "destroyed sensor publishes its end event on the next step only"
#[test]
fn destruction() {
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    let mut shape_def = ShapeDef::default();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    let sensor = world.create_sphere_shape(ground, &shape_def, &Sphere::new(Vec3::ZERO, 1.0));

    let mut definition = BodyDef::default();
    definition.type_ = BodyType::KinematicBody;
    let visitor_body = world.create_body(&definition);
    shape_def.is_sensor = false;
    let visitor = world.create_sphere_shape(visitor_body, &shape_def, &Sphere::new(Vec3::ZERO, 0.5));

    world.step(1.0f32 / 60.0, 4);
    assert_eq!(world.sensor_events().begin.len(), 1);

    world.destroy_shape(sensor, true);
    assert!(world.sensor_events().end.is_empty());

    world.step(1.0f32 / 60.0, 4);
    let events = world.sensor_events();
    assert!(events.begin.is_empty());
    assert_eq!(events.end.len(), 1);
    assert_eq!(events.end.first().unwrap().sensor_shape_id, sensor);
    assert_eq!(events.end.first().unwrap().visitor_shape_id, visitor);

    world.step(1.0f32 / 60.0, 4);
    assert!(world.sensor_events().end.is_empty());
}

#[allow(dead_code)]
fn _unused_pos() -> Pos {
    Pos::ZERO
}
