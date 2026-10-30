//! Ported from Tests/NatselTests/Simulation/JointApiTests.swift.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::types::Pos;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::BodyID;
use crate::types::joint_defs::{DistanceJointDef, FilterJointDef};
use crate::types::shape_def::ShapeDef;

struct Fixture {
    world: World,
    ground: BodyID,
    body: BodyID,
}

fn make_fixture() -> Fixture {
    let mut fixture = Fixture {
        world: World::default(),
        ground: BodyID::NULL,
        body: BodyID::NULL,
    };

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    fixture.ground = fixture.world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 0.0, 0.0);
    fixture.body = fixture.world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = fixture.world.create_hull_shape(
        fixture.body,
        &shape_def,
        &make_box_hull(0.5, 0.5, 0.5),
    );
    fixture
}

// "distance joint setters round trip"
#[test]
fn distance_joint_api() {
    let mut f = make_fixture();

    let mut def = DistanceJointDef::default();
    def.base.body_id_a = f.ground;
    def.base.body_id_b = f.body;
    def.length = 2.0;
    let joint = f.world.create_distance_joint(&def);

    f.world.set_distance_joint_length(joint, 3.0);
    assert_eq!(f.world.distance_joint_length(joint).unwrap(), 3.0);

    f.world.enable_distance_joint_spring(joint, true);
    assert!(f.world.is_distance_joint_spring_enabled(joint));

    f.world.set_distance_joint_spring_force_range(joint, -50.0, 75.0);
    let range = f.world.distance_joint_spring_force_range(joint).unwrap();
    assert!(range.0 == -50.0 && range.1 == 75.0);

    f.world.set_distance_joint_spring_hertz(joint, 4.0);
    assert_eq!(f.world.distance_joint_spring_hertz(joint).unwrap(), 4.0);

    f.world.set_distance_joint_spring_damping_ratio(joint, 0.6);
    assert_eq!(
        f.world.distance_joint_spring_damping_ratio(joint).unwrap(),
        0.6
    );

    f.world.enable_distance_joint_limit(joint, true);
    assert!(f.world.is_distance_joint_limit_enabled(joint));

    f.world.set_distance_joint_length_range(joint, 1.0, 5.0);
    assert_eq!(f.world.distance_joint_min_length(joint).unwrap(), 1.0);
    assert_eq!(f.world.distance_joint_max_length(joint).unwrap(), 5.0);
    assert!(f.world.distance_joint_current_length(joint).is_some());

    f.world.enable_distance_joint_motor(joint, true);
    assert!(f.world.is_distance_joint_motor_enabled(joint));

    f.world.set_distance_joint_motor_speed(joint, 1.5);
    assert_eq!(f.world.distance_joint_motor_speed(joint).unwrap(), 1.5);

    f.world.set_distance_joint_max_motor_force(joint, 25.0);
    assert_eq!(f.world.distance_joint_max_motor_force(joint).unwrap(), 25.0);

    f.world.destroy_joint(joint.index1 - 1, false);
    assert!(f.world.distance_joint_sim(joint).is_none());
}

// "length clamps to the linear slop"
#[test]
fn length_clamp() {
    let mut f = make_fixture();

    let mut def = DistanceJointDef::default();
    def.base.body_id_a = f.ground;
    def.base.body_id_b = f.body;
    let joint = f.world.create_distance_joint(&def);

    f.world.set_distance_joint_length(joint, -5.0);
    assert_eq!(
        f.world.distance_joint_length(joint).unwrap(),
        b3::linear_slop()
    );

    // The range normalizes so min <= max regardless of the argument order.
    f.world.set_distance_joint_length_range(joint, 5.0, 1.0);
    assert_eq!(f.world.distance_joint_min_length(joint).unwrap(), 1.0);
    assert_eq!(f.world.distance_joint_max_length(joint).unwrap(), 5.0);
}

// "filter joint prevents contact but keeps both bodies"
#[test]
fn filter_joint() {
    let mut f = make_fixture();

    let mut def = FilterJointDef::default();
    def.base.body_id_a = f.ground;
    def.base.body_id_b = f.body;
    let joint = f.world.create_filter_joint(&def);

    // The filter joint connects the bodies, so moving the body must keep it valid.
    f.world.step(1.0f32 / 60.0, 4);

    let mut contacts_between = 0;
    for c in f.world.contacts.iter() {
        if c.contact_id == b3::NULL_INDEX {
            continue;
        }
        let shape_a = &f.world.shapes[c.shape_id_a as usize];
        let shape_b = &f.world.shapes[c.shape_id_b as usize];
        if (shape_a.body_id == f.ground.index1 - 1 && shape_b.body_id == f.body.index1 - 1)
            || (shape_a.body_id == f.body.index1 - 1 && shape_b.body_id == f.ground.index1 - 1)
        {
            contacts_between += 1;
        }
    }
    assert_eq!(contacts_between, 0);

    f.world.destroy_joint(joint.index1 - 1, true);
    assert_eq!(
        f.world.joints[(joint.index1 - 1) as usize].joint_id,
        b3::NULL_INDEX
    );
}
