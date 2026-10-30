//! Ported from Tests/NatselTests/Simulation/JointEventTests.swift.
//!
//! Joint events and shape-level contact data.

use crate::collision::hull::make_box_hull;
use crate::math::types::Pos;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::BodyID;
use crate::types::joint_defs::DistanceJointDef;
use crate::types::shape_def::ShapeDef;

/// A static anchor at y=5 with a heavy box hanging from y=4 on a unit distance joint.
fn make_loaded_pendulum(world: &mut World) -> crate::types::ids::JointID {
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    let anchor: BodyID = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 4.0, 0.0);
    let bob = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(bob, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    let mut joint_def = DistanceJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.length = 1.0;
    world.create_distance_joint(&joint_def)
}

// "a joint under load reports an event when its threshold is low"
#[test]
fn joint_event_fires() {
    let mut world = World::default();
    world.enable_sleep = false;

    let joint = make_loaded_pendulum(&mut world);

    // With a zero force threshold any awake loaded joint is reported.
    world.set_joint_force_threshold(joint, 0.0);

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }

    let events = world.joint_event_list();
    assert!(!events.is_empty());
    assert!(events.iter().any(|e| e.joint_id.index1 == joint.index1));
}

// "a high threshold suppresses the event"
#[test]
fn high_threshold_suppresses() {
    let mut world = World::default();
    world.enable_sleep = false;

    let joint = make_loaded_pendulum(&mut world);

    world.set_joint_force_threshold(joint, 1.0e9);
    world.set_joint_torque_threshold(joint, 1.0e9);

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }
    assert!(world.joint_event_list().is_empty());
}

// "joint events are cleared each step"
#[test]
fn events_clear_each_step() {
    let mut world = World::default();
    world.enable_sleep = false;

    let joint = make_loaded_pendulum(&mut world);
    world.set_joint_force_threshold(joint, 0.0);

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }
    assert!(!world.joint_event_list().is_empty());

    // Destroying the joint removes it from the next step's events.
    world.destroy_joint(joint.index1 - 1, false);
    world.step(1.0f32 / 60.0, 4);
    assert!(world.joint_event_list().is_empty());
}

// "shape contact data reports only its own touching contacts"
#[test]
fn shape_contact_data() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let ground_shape_def = ShapeDef::default();
    let ground_shape = world.create_hull_shape(ground, &ground_shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.5, 0.0);
    let box_ = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let box_shape = world.create_hull_shape(box_, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }

    let contacts = world.shape_contact_data(box_shape, 32);
    assert_eq!(contacts.len(), 1);
    let first = match contacts.first() {
        Some(first) => first.clone(),
        None => return,
    };

    // The contact lists both shapes.
    let involves_box =
        first.shape_id_a.index1 == box_shape.index1 || first.shape_id_b.index1 == box_shape.index1;
    let involves_ground = first.shape_id_a.index1 == ground_shape.index1
        || first.shape_id_b.index1 == ground_shape.index1;
    assert!(involves_box);
    assert!(involves_ground);

    // The same contact is reported from the ground's side.
    assert_eq!(world.shape_contact_data(ground_shape, 32).len(), 1);
    assert!(world.contact_is_valid(first.contact_id));

    // A detached shape reports nothing.
    world.destroy_shape(box_shape, false);
    assert!(world.shape_contact_data(ground_shape, 32).is_empty());
}

// "a sensor shape reports no contact data"
#[test]
fn sensor_has_no_contacts() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.5, 0.0);
    let probe = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    let sensor = world.create_hull_shape(probe, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }

    // Sensors never surface through contact data.
    assert!(world.shape_contact_data(sensor, 32).is_empty());
}
