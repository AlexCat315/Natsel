//! Ported from Tests/NatselTests/Simulation/ImpulseDeterminismTests.swift.

use crate::base::constants::{HASH_INIT, NULL_INDEX};
use crate::base::hash::hash_bytes;
use crate::collision::hull::make_box_hull;
use crate::collision::mesh::procedural::create_wave_mesh;
use crate::math::types::{Pos, Vec3};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;

/// A plain-old-data value hashed by its raw bytes, matching Swift's
/// `withUnsafeBytes(of: &value) { hash(h, $0.baseAddress!, Int32($0.count)) }`.
fn hash_floats(h: u32, value: &impl Copy) -> u32 {
    hash_bytes(h, value)
}

/// Hash the touching manifolds reachable from one body, following the intrusive contact list.
fn hash_contacts(world: &World, body: BodyID, h: &mut u32, capacity: i32) {
    let mut contact_key = world.bodies[world.body_index(body) as usize].head_contact_key;
    let mut count = 0;
    while contact_key != NULL_INDEX && count < capacity {
        let contact_id = contact_key >> 1;
        let edge_index = (contact_key & 1) as usize;
        let contact = world.contacts[contact_id as usize];
        if (contact.flags & ContactFlags::TOUCHING.0) != 0 {
            let view = contact.manifolds();
            for m in 0..contact.manifold_count as usize {
                let manifold = view.get(m);
                *h = hash_floats(*h, &manifold.rolling_impulse);
                *h = hash_floats(*h, &manifold.friction_impulse);
                *h = hash_floats(*h, &manifold.twist_impulse);
                for p in 0..manifold.point_count as usize {
                    let point = manifold.points[p];
                    *h = hash_floats(*h, &point.normal_impulse);
                    *h = hash_floats(*h, &point.total_normal_impulse);
                }
            }
            count += 1;
        }
        contact_key = contact.edges[edge_index].next_key;
    }
}

// "rolling resistance mix is repeatable"
#[test]
fn rolling_mix() {
    for wide in [false, true] {
        fn snapshot(wide: bool) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let mut world = World::default();
            world.enable_wide_solver = wide;
            let ground = world.create_body(&BodyDef::default());
            let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(40.0, 1.0, 40.0));

            let mut body_ids: Vec<BodyID> = Vec::new();
            let mut seed: u32 = 271_828;
            let box_hull = make_box_hull(0.4, 0.4, 0.4);
            let sphere = Sphere::new(Vec3::ZERO, 0.4);
            let capsule = Capsule::new(
                Vec3::new(-0.3, 0.0, 0.0),
                Vec3::new(0.3, 0.0, 0.0),
                0.25,
            );

            for i in 0..64i32 {
                let row = i / 8;
                let column = i % 8;
                let mut def = BodyDef::default();
                def.type_ = BodyType::DynamicBody;
                def.position = Pos::new(
                    f64::from(2.0 * column as f32 - 7.0),
                    1.45,
                    f64::from(2.0 * row as f32 - 7.0),
                );
                let id = world.create_body(&def);
                body_ids.push(id);

                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let mut shape_def = ShapeDef::default();
                shape_def.density = 1.0;
                shape_def.base_material.rolling_resistance = if (seed >> 28) < 5 { 0.2 } else { 0.0 };

                match i % 3 {
                    0 => {
                        let _ = world.create_sphere_shape(id, &shape_def, sphere);
                    }
                    1 => {
                        let _ = world.create_capsule_shape(id, &shape_def, capsule);
                    }
                    _ => {
                        let _ = world.create_hull_shape(id, &shape_def, &box_hull);
                    }
                }

                let sign: f32 = if (i & 1) != 0 { -1.0 } else { 1.0 };
                world.set_body_linear_velocity(id, Vec3::new(-0.0, -0.5, sign * 0.5));
                world.set_body_angular_velocity(id, Vec3::new(sign * 3.0, -0.0, -2.0 * sign));
            }

            let mut h = HASH_INIT;
            for _ in 0..120 {
                world.step(1.0f32 / 60.0, 4);
                for id in &body_ids {
                    h = super::determinism_scene::hash_transform(h, &world.body_transform(*id));
                    hash_contacts(&world, *id, &mut h, 4);
                }
            }

            samples.push(u64::from(h));
            samples
        }

        let first = snapshot(wide);
        let replay = snapshot(wide);
        assert!(!first.is_empty());
        assert_eq!(first, replay);
    }
}

fn make_mesh_mix(wide: bool) -> (World, Vec<BodyID>) {
    let mut world = World::default();
    world.enable_wide_solver = wide;
    let mesh = create_wave_mesh(40, 40, 0.5, 0.3, 0.2, 0.3).expect("wave mesh");
    let mut ground_def = BodyDef::default();
    ground_def.position = Pos::new(-10.0, 0.0, -10.0);
    let ground = world.create_body(&ground_def);
    let _ = world.create_mesh_shape(ground, &ShapeDef::default(), mesh, Vec3::ONE);

    let mut body_ids: Vec<BodyID> = Vec::new();
    let small_box = make_box_hull(0.2, 0.2, 0.2);
    let large_box = make_box_hull(0.9, 0.3, 0.9);
    let sphere = Sphere::new(Vec3::ZERO, 0.3);
    let capsule = Capsule::new(
        Vec3::new(-0.5, 0.0, 0.0),
        Vec3::new(0.5, 0.0, 0.0),
        0.2,
    );

    let mut seed: u32 = 314_159;
    for i in 0..64i32 {
        let row = i / 8;
        let column = i % 8;
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(
            f64::from(2.2 * column as f32 - 7.7),
            1.2,
            f64::from(2.2 * row as f32 - 7.7),
        );
        let id = world.create_body(&def);
        body_ids.push(id);

        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let mut shape_def = ShapeDef::default();
        shape_def.density = 1.0;
        shape_def.base_material.rolling_resistance = if (seed >> 28) < 5 { 0.2 } else { 0.0 };
        match (seed >> 24) & 3 {
            0 => {
                let _ = world.create_hull_shape(id, &shape_def, &large_box);
            }
            1 => {
                let _ = world.create_hull_shape(id, &shape_def, &small_box);
            }
            2 => {
                let _ = world.create_sphere_shape(id, &shape_def, sphere);
            }
            _ => {
                let _ = world.create_capsule_shape(id, &shape_def, capsule);
            }
        }

        let sign: f32 = if (i & 1) != 0 { -1.0 } else { 1.0 };
        world.set_body_linear_velocity(id, Vec3::new(-0.0, -0.5, -0.0));
        world.set_body_angular_velocity(id, Vec3::new(-0.0, sign * 0.5, -0.0));
    }

    (world, body_ids)
}

// "mesh mix is repeatable"
#[test]
fn mesh_mix() {
    for wide in [false, true] {
        fn snapshot(wide: bool) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let (mut world, body_ids) = make_mesh_mix(wide);

            let mut h = HASH_INIT;
            for _ in 0..180 {
                world.step(1.0f32 / 60.0, 4);
                for id in &body_ids {
                    h = super::determinism_scene::hash_transform(h, &world.body_transform(*id));
                    let v = world.body_linear_velocity(*id);
                    let w = world.body_angular_velocity(*id);
                    h = hash_floats(h, &v);
                    h = hash_floats(h, &w);
                    hash_contacts(&world, *id, &mut h, 8);
                }
            }

            samples.push(u64::from(h));
            samples
        }

        let first = snapshot(wide);
        let replay = snapshot(wide);
        assert!(!first.is_empty());
        assert_eq!(first, replay);
    }
}

// "mesh mix preserves every impulse bit"
#[test]
fn mesh_mix_paths_agree() {
    let (mut scalar, _) = make_mesh_mix(false);
    let (mut wide, _) = make_mesh_mix(true);
    fn bits(v: Vec3) -> [u32; 3] {
        [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
    }
    for step in 0..180 {
        scalar.step(1.0 / 60.0, 4);
        wide.step(1.0 / 60.0, 4);
        assert!(
            scalar.world_state_hash() == wide.world_state_hash(),
            "state at step {step}"
        );
        for id in 0..scalar.contacts.len() {
            let a = scalar.contacts[id];
            let b = wide.contacts[id];
            assert!(
                a.manifold_count == b.manifold_count,
                "manifold count step {step} id {id}"
            );
            // A freed slot has a null payload, so `manifolds()` traps; Swift's loop body never
            // ran for those slots either, because their manifold count is zero.
            if a.manifold_count <= 0 {
                continue;
            }
            let a_view = a.manifolds();
            let b_view = b.manifolds();
            for mi in 0..a.manifold_count as usize {
                let x = a_view.get(mi);
                let y = b_view.get(mi);
                assert!(
                    bits(x.friction_impulse) == bits(y.friction_impulse),
                    "friction step {step} id {id} manifold {mi} {:?} != {:?}",
                    x.friction_impulse,
                    y.friction_impulse
                );
                assert!(
                    bits(x.rolling_impulse) == bits(y.rolling_impulse),
                    "rolling step {step} id {id} manifold {mi} {:?} != {:?}",
                    x.rolling_impulse,
                    y.rolling_impulse
                );
                assert!(
                    x.twist_impulse.to_bits() == y.twist_impulse.to_bits(),
                    "twist step {step} id {id} manifold {mi}"
                );
                for pi in 0..x.point_count as usize {
                    assert!(
                        x.points[pi].normal_impulse.to_bits()
                            == y.points[pi].normal_impulse.to_bits(),
                        "normal step {step} id {id} manifold {mi} point {pi}"
                    );
                    assert!(
                        x.points[pi].total_normal_impulse.to_bits()
                            == y.points[pi].total_normal_impulse.to_bits(),
                        "total step {step} id {id} manifold {mi} point {pi}"
                    );
                }
            }
        }
    }
}
