//! Ported from Tests/NatselTests/WheelJointTests.swift.
//!
//! Wheel-joint suspension/spin/steering sims must be byte-identical across two runs.

use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::joint_defs::WheelJointDef;
use crate::types::shape_def::ShapeDef;

fn fnv(mut h: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(1_099_511_628_211);
    }
    h
}

#[test]
fn wheel() {
    // "suspension, spin, and steering are repeatable"
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
        def.position = Pos::new(0.5, 3.0, 0.0);
        def.angular_velocity = Vec3::new(0.0, 0.0, 1.0);
        let body_b = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_sphere_shape(body_b, &shape_def, &Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.3));

        let mut jd = WheelJointDef::default();
        jd.base.body_id_a = body_a;
        jd.base.body_id_b = body_b;
        jd.enable_spin_motor = true;
        jd.max_spin_torque = 50.0;
        jd.spin_speed = 2.0;
        jd.enable_suspension_spring = true;
        jd.suspension_hertz = 4.0;
        jd.suspension_damping_ratio = 0.7;
        jd.enable_suspension_limit = true;
        jd.lower_suspension_limit = -0.2;
        jd.upper_suspension_limit = 0.2;
        jd.enable_steering = true;
        jd.steering_hertz = 4.0;
        jd.steering_damping_ratio = 0.7;
        jd.target_steering_angle = 0.3;
        jd.max_steering_torque = 30.0;
        jd.enable_steering_limit = true;
        jd.lower_steering_limit = -0.5;
        jd.upper_steering_limit = 0.5;
        let joint = world.create_wheel_joint(&jd);

        for _ in 0..120 {
            world.step(1.0 / 60.0, 4);
        }

        let t = world.body_transform(body_b);
        let v = world.body_linear_velocity(body_b);
        let av = world.body_angular_velocity(body_b);
        let sim = world.read_joint_sim(joint.index1 - 1).wheel_joint();

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
        h = fnv(h, &sim.angular_impulse.x.to_bits().to_le_bytes());
        h = fnv(h, &sim.angular_impulse.y.to_bits().to_le_bytes());
        h = fnv(h, &sim.spin_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.suspension_spring_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.lower_suspension_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.upper_suspension_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.steering_spring_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.lower_steering_impulse.to_bits().to_le_bytes());
        h = fnv(h, &sim.upper_steering_impulse.to_bits().to_le_bytes());

        samples.push(h);
        samples.push(sim.spin_impulse.to_bits() as u64);
        samples.push(sim.steering_spring_impulse.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
