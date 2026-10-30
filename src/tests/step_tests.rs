//! Ported from Tests/NatselTests/StepTests.swift.
//!
//! A contact-free integration step must be byte-identical across two runs and exercise
//! motion locks, kinematic/static body handling, and applied forces/torques.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

fn fnv(mut h: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(1_099_511_628_211);
    }
    h
}

#[inline]
fn fnv_f32(h: u64, x: f32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[inline]
fn fnv_f64(h: u64, x: f64) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[test]
fn contact_free_step() {
    // "contact-free integration is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        // TODO(migration): add `impl Default for World` in the final build-fix phase.
        let mut world = World::default();
        world.enable_sleep = false;
        world.enable_continuous = false;

        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, 5.0, 0.0);
        def.linear_velocity = Vec3::new(1.0, 2.0, 3.0);
        def.angular_velocity = Vec3::new(0.1, 0.2, 0.3);
        def.linear_damping = 0.1;
        def.angular_damping = 0.2;
        def.gravity_scale = 0.5;
        let b0 = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(b0, &shape_def, &make_box_hull(0.5, 1.0, 1.5));

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(-1.0, 3.0, 0.0);
        def.linear_velocity = Vec3::new(0.5, -0.5, 1.0);
        def.angular_velocity = Vec3::new(1.0, 0.0, 0.0);
        def.motion_locks.linear_x = true;
        def.motion_locks.angular_y = true;
        def.motion_locks.angular_z = true;
        let b1 = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 500.0;
        let _ = world.create_sphere_shape(
            b1,
            &shape_def,
            Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5),
        );

        def = BodyDef::default();
        def.type_ = BodyType::KinematicBody;
        def.position = Pos::new(2.0, 0.0, 0.0);
        def.linear_velocity = Vec3::new(0.0, 0.0, 1.0);
        def.angular_velocity = Vec3::new(0.3, 0.0, 0.0);
        let b2 = world.create_body(&def);
        shape_def = ShapeDef::default();
        let _ = world.create_sphere_shape(
            b2,
            &shape_def,
            Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.25),
        );

        def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        def.position = Pos::new(10.0, 10.0, 10.0);
        let b3s = world.create_body(&def);
        shape_def = ShapeDef::default();
        let _ = world.create_sphere_shape(
            b3s,
            &shape_def,
            Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0),
        );

        world.apply_body_force(b0, Vec3::new(100.0, 0.0, 0.0), Pos::new(0.2, 5.2, 0.0), true);
        world.apply_body_torque(b0, Vec3::new(0.0, 10.0, 0.0), true);

        for _ in 0..5 {
            world.step(1.0 / 60.0, 4);
        }

        let mut h: u64 = 14_695_981_039_346_656_037;
        let ids = [b0, b1, b2, b3s];
        for id in ids {
            let transform = world.body_transform(id);
            let v = world.body_linear_velocity(id);
            let w = world.body_angular_velocity(id);
            let center = world.body_world_center(id);
            h = fnv_f64(h, transform.p.x);
            h = fnv_f64(h, transform.p.y);
            h = fnv_f64(h, transform.p.z);
            h = fnv_f32(h, transform.q.v.x);
            h = fnv_f32(h, transform.q.v.y);
            h = fnv_f32(h, transform.q.v.z);
            h = fnv_f32(h, transform.q.s);
            h = fnv_f32(h, v.x);
            h = fnv_f32(h, v.y);
            h = fnv_f32(h, v.z);
            h = fnv_f32(h, w.x);
            h = fnv_f32(h, w.y);
            h = fnv_f32(h, w.z);
            // `body_world_center` is a `Pos` (double precision), so hash it at full width.
            // Swift's test passed a literal 12-byte length for a 24-byte `Pos`, truncating it;
            // hashing the whole value is stricter and this sample is only compared against
            // itself, so the repeatability contract is unchanged.
            h = fnv_f64(h, center.x);
            h = fnv_f64(h, center.y);
            h = fnv_f64(h, center.z);
            let flags = world.bodies[id.index1 as usize].flags;
            h = fnv(h, &flags.to_ne_bytes());
        }
        samples.push(h);

        let t0 = world.body_transform(b0);
        samples.push((t0.p.x as f32).to_bits() as u64);
        samples.push((t0.p.y as f32).to_bits() as u64);
        samples.push((t0.p.z as f32).to_bits() as u64);
        samples.push(t0.q.v.x.to_bits() as u64);
        samples.push(t0.q.s.to_bits() as u64);

        samples.push(world.body_linear_velocity(b0).x.to_bits() as u64);
        samples.push(world.body_linear_velocity(b0).y.to_bits() as u64);
        samples.push(world.body_linear_velocity(b0).z.to_bits() as u64);

        // Motion locks zero the x translation.
        samples.push((world.body_transform(b1).p.x as f32).to_bits() as u64);
        assert_eq!(world.body_linear_velocity(b1).x.to_bits(), 0);
        assert_eq!(world.bodies[b1.index1 as usize].flags, 0x0000_7031);

        // Kinematic body moves but ignores gravity.
        samples.push((world.body_transform(b2).p.z as f32).to_bits() as u64);
        assert_eq!(world.bodies[b2.index1 as usize].flags, 0x0000_6000);

        // Static body is untouched.
        assert_eq!(world.body_transform(b3s).p, Pos::new(10.0, 10.0, 10.0));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
