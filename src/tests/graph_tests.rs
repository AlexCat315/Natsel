//! Ported from Tests/NatselTests/GraphTests.swift.
//!
//! Constraint-graph coloring, per-contact color/local indices, and waking a sleeping body must be
//! identical across two runs.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::contact_store::ContactStore;
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
fn fnv_i32(h: u64, x: i32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[test]
fn graph_and_wake() {
    // "graph coloring and waking are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        let ground = world.create_body(&def);

        let mut shape_def = ShapeDef::default();
        shape_def.density = 0.0;
        let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

        let positions = [
            Vec3::new(0.0, 0.9, 0.0),
            Vec3::new(0.6, 0.9, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ];
        for p in positions {
            def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::from_vec3(p);
            let body = world.create_body(&def);
            shape_def = ShapeDef::default();
            shape_def.density = 1000.0;
            let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::ZERO, 0.5));
        }

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(-0.9, 0.9, 0.0);
        let box_body = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

        world.update_broad_phase_pairs();

        // Swift wrote through `inout ContactStore(&world.contacts)` while calling `updateContact`;
        // the store is rebuilt over the live buffer through a raw pointer, exactly as
        // `collide_pass.rs` does.
        {
            let contact_ptr = world.contacts.as_mut_ptr();
            let contact_count = world.contacts.len();
            let contact_ids = world.solver_sets[2].contact_indices.clone();
            let mut store = ContactStore::from_buffer(
                // SAFETY: `contacts` is not resized by the narrow phase and does not move while
                // the store lives, which is the invariant Swift's
                // `withUnsafeMutableBufferPointer` gave it.
                unsafe { std::slice::from_raw_parts_mut(contact_ptr, contact_count) },
            );
            for cid in contact_ids {
                let contact = store.get(cid);
                let shape_a = &world.shapes[contact.shape_id_a as usize];
                let shape_b = &world.shapes[contact.shape_id_b as usize];
                let sim_a = world.body_sim_for(world.make_body_id(shape_a.body_id));
                let sim_b = world.body_sim_for(world.make_body_id(shape_b.body_id));
                world.update_contact_default_scratch(
                    cid,
                    sim_a.local_center,
                    sim_a.transform,
                    sim_b.local_center,
                    sim_b.transform,
                    false,
                    &mut store,
                );
            }
        }

        let contact_ids = world.solver_sets[2].contact_indices.clone();
        for cid in contact_ids {
            let flags = world.contacts[cid as usize].flags;
            if (flags & ContactFlags::SIM_TOUCHING.0) != 0 {
                world.link_contact(cid);
                world.add_contact_to_graph(cid);
            }
        }

        let mut gh: u64 = 14_695_981_039_346_656_037;
        for c in world.contacts.iter() {
            if c.contact_id == b3::NULL_INDEX {
                continue;
            }
            gh = fnv_i32(gh, c.color_index);
            gh = fnv_i32(gh, c.local_index);
            gh = fnv_i32(gh, c.encoded_body_sim_a);
            gh = fnv_i32(gh, c.encoded_body_sim_b);
        }
        for i in 0..b3::GRAPH_COLOR_COUNT as usize {
            let color = &world.constraint_graph.colors[i];
            gh = fnv_i32(gh, color.convex_contacts.len() as i32);
            gh = fnv_i32(gh, color.contacts.len() as i32);
            gh = fnv_i32(gh, color.joint_sims.len() as i32);
            for &id in color.convex_contacts.iter() {
                gh = fnv_i32(gh, id);
            }
            for spec in color.contacts.iter() {
                gh = fnv_i32(gh, spec.contact_id);
            }
        }
        samples.push(gh);

        // Verify per-contact colors and indices.
        let mut touching_contacts: Vec<(i32, i32, i32, i32, i32, i32)> = Vec::new();
        for c in world.contacts.iter() {
            if c.contact_id == b3::NULL_INDEX {
                continue;
            }
            touching_contacts.push((
                c.contact_id,
                c.color_index,
                c.local_index,
                c.encoded_body_sim_a,
                c.encoded_body_sim_b,
                if (c.flags & ContactFlags::SIM_TOUCHING.0) != 0 {
                    1
                } else {
                    0
                },
            ));
        }
        assert_eq!(touching_contacts.len(), 8);
        assert_eq!(touching_contacts[0], (0, 22, 0, -2, 0, 1));
        assert_eq!(touching_contacts[1], (1, 22, 1, -2, 1, 1));
        assert_eq!(touching_contacts[2], (2, 22, 2, -2, 3, 1));
        assert_eq!(touching_contacts[3], (3, 0, 0, 0, 1, 1));
        assert_eq!(touching_contacts[4], (4, -1, 4, 0, 2, 0));
        assert_eq!(touching_contacts[5], (5, 1, 0, 3, 0, 1));
        assert_eq!(touching_contacts[6], (6, -1, 6, 1, 2, 0));
        assert_eq!(touching_contacts[7], (7, -1, 7, 3, 2, 0));

        let mut nonzero = 0;
        for i in 0..b3::GRAPH_COLOR_COUNT as usize {
            let c = &world.constraint_graph.colors[i];
            if c.convex_contacts.len() + c.contacts.len() + c.joint_sims.len() > 0 {
                nonzero += 1;
            }
        }
        assert_eq!(nonzero, 3);
        assert_eq!(world.constraint_graph.colors[0].convex_contacts.len(), 1);
        assert_eq!(world.constraint_graph.colors[1].convex_contacts.len(), 1);
        assert_eq!(world.constraint_graph.colors[22].convex_contacts.len(), 3);

        // Wake a sleeping body.
        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(20.0, 1.0, 0.0);
        def.is_awake = false;
        let sleeper = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_sphere_shape(sleeper, &shape_def, Sphere::new(Vec3::ZERO, 0.5));

        let mut sb = world.body(sleeper);
        assert_eq!(sb.set_index, 3);
        assert_eq!(sb.local_index, 0);
        assert_eq!(world.solver_sets.len(), 4);
        assert_eq!(world.solver_sets[2].body_sims.len(), 4);

        world.wake_body(world.body_index(sleeper));

        sb = world.body(sleeper);
        assert_eq!(sb.set_index, 2);
        assert_eq!(sb.local_index, 4);
        assert_eq!(world.solver_sets.len(), 4);
        assert_eq!(world.solver_sets[2].body_sims.len(), 5);
        assert_eq!(world.solver_sets[2].body_states.len(), 5);
        assert_eq!(world.solver_sets[2].island_sims.len(), 3);
        assert_eq!(sb.island_id, 3);
        assert_eq!(world.islands[3].set_index, 2);
        assert_eq!(world.solver_sets[3].set_index, b3::NULL_INDEX);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
