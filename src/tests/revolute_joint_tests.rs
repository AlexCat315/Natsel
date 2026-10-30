//! Ported from Tests/NatselTests/RevoluteJointTests.swift.
//!
//! Revolute-joint spring/limit/motor sims must be byte-identical across two runs.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::joint_defs::RevoluteJointDef;
use crate::types::shape_def::ShapeDef;

fn fnv(mut h: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(1_099_511_628_211);
    }
    h
}

#[test]
fn spring_revolute() {
    // "spring, limit, and motor are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();
        world.enable_sleep = false;

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        def.position = Pos::new(0.0, 3.0, 0.0);
        let body_a = world.create_body(&def);

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(1.5, 3.0, 0.0);
        def.angular_velocity = Vec3::new(1.0, 0.5, 0.0);
        let body_b = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(body_b, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

        let mut jd = RevoluteJointDef::default();
        jd.base.body_id_a = body_a;
        jd.base.body_id_b = body_b;
        jd.base.local_frame_a.p = Vec3::new(1.0, 0.0, 0.0);
        jd.base.local_frame_b.p = Vec3::new(-0.5, 0.0, 0.0);
        jd.enable_spring = true;
        jd.hertz = 4.0;
        jd.damping_ratio = 0.6;
        jd.enable_motor = true;
        jd.max_motor_torque = 50.0;
        jd.motor_speed = 1.0;
        jd.enable_limit = true;
        jd.lower_angle = -0.5;
        jd.upper_angle = 0.5;
        let joint = world.create_revolute_joint(&jd);

        for _ in 0..120 {
            world.step(1.0 / 60.0, 4);
        }

        let t = world.body_transform(body_b);
        let v = world.body_linear_velocity(body_b);
        let av = world.body_angular_velocity(body_b);
        let sim = world.read_joint_sim(joint.index1 - 1).revolute_joint();

        let mut h: u64 = 14_695_981_039_346_656_037;
        h = fnv(h, &t.p.x.to_bits().to_le_bytes());
        h = fnv(h, &t.p.y.to_bits().to_le_bytes());
        h = fnv(h, &t.p.z.to_bits().to_le_bytes());
        h = fnv(h, &t.q.v.x.to_bits().to_le_bytes());
        h = fnv(h, &t.q.v.y.to_bits().to_le_bytes());
        h = fnv(h, &t.q.v.z.to_bits().to_le_bytes());
        h = fnv(h, &t.q.s.to_bits().to_le_bytes());
        h = fnv(h, &v.x.to_bits().to_le_bytes());
        h = fnv(h, &v.y.to_bits().to_le_bytes());
        h = fnv(h, &v.z.to_bits().to_le_bytes());
        h = fnv(h, &av.x.to_bits().to_le_bytes());
        h = fnv(h, &av.y.to_bits().to_le_bytes());
        h = fnv(h, &av.z.to_bits().to_le_bytes());
        h = fnv(h, &sim.linear_impulse.x.to_bits().to_le_bytes());
        h = fnv(h, &sim.linear_impulse.y.to_bits().to_le_bytes());
        h = fnv(h, &sim.linear_impulse.z.to_bits().to_le_bytes());
        h = fnv(h, &sim.perp_impulse.x.to_bits().to_le_bytes());
        h = fnv(h, &sim.perp_impulse.y.to_bits().to_le_bytes());
        h = fnv(h, &sim.spring_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.motor_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.lower_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.upper_impulse.to_bits().to_le_bytes());

        samples.push(h);
        samples.push(sim.motor_impulse.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn hinged_pendulum() {
    // "hinged pendulum is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();
        world.enable_sleep = false;

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        def.position = Pos::new(0.0, 3.0, 0.0);
        let body_a = world.create_body(&def);

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(1.5, 3.0, 0.0);
        let body_b = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(body_b, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

        let mut jd = RevoluteJointDef::default();
        jd.base.body_id_a = body_a;
        jd.base.body_id_b = body_b;
        jd.base.local_frame_a.p = Vec3::new(1.0, 0.0, 0.0);
        jd.base.local_frame_b.p = Vec3::new(-0.5, 0.0, 0.0);
        let joint = world.create_revolute_joint(&jd);

        for _ in 0..120 {
            world.step(1.0 / 60.0, 4);
        }

        let t = world.body_transform(body_b);
        let v = world.body_linear_velocity(body_b);
        let av = world.body_angular_velocity(body_b);
        let sim = world.read_joint_sim(joint.index1 - 1).revolute_joint();

        let mut h: u64 = 14_695_981_039_346_656_037;
        h = fnv(h, &t.p.x.to_bits().to_le_bytes());
        h = fnv(h, &t.p.y.to_bits().to_le_bytes());
        h = fnv(h, &t.p.z.to_bits().to_le_bytes());
        h = fnv(h, &t.q.v.x.to_bits().to_le_bytes());
        h = fnv(h, &t.q.v.y.to_bits().to_le_bytes());
        h = fnv(h, &t.q.v.z.to_bits().to_le_bytes());
        h = fnv(h, &t.q.s.to_bits().to_le_bytes());
        h = fnv(h, &v.x.to_bits().to_le_bytes());
        h = fnv(h, &v.y.to_bits().to_le_bytes());
        h = fnv(h, &v.z.to_bits().to_le_bytes());
        h = fnv(h, &av.x.to_bits().to_le_bytes());
        h = fnv(h, &av.y.to_bits().to_le_bytes());
        h = fnv(h, &av.z.to_bits().to_le_bytes());
        h = fnv(h, &sim.linear_impulse.x.to_bits().to_le_bytes());
        h = fnv(h, &sim.linear_impulse.y.to_bits().to_le_bytes());
        h = fnv(h, &sim.linear_impulse.z.to_bits().to_le_bytes());
        h = fnv(h, &sim.perp_impulse.x.to_bits().to_le_bytes());
        h = fnv(h, &sim.perp_impulse.y.to_bits().to_le_bytes());
        h = fnv(h, &sim.spring_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.motor_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.lower_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.upper_impulse.to_bits().to_le_bytes());

        samples.push(h);
        samples.push(sim.linear_impulse.x.to_bits() as u64);
        samples.push(sim.linear_impulse.y.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
