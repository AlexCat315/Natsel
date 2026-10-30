//! Ported from Tests/NatselTests/Simulation/SolverStorageTests.swift.

use std::sync::Arc;

use crate::simulation::contact_payload::{ContactManifolds, ContactPayload, ContactPayloadPool};
use crate::simulation::world_runtime::{Contact, JointSim, World};
use crate::types::collision_types::Manifold;

// "joint variants share compact POD storage"
#[test]
fn joint_layout() {
    assert!(std::mem::size_of::<JointSim>() < 768);
    assert!(std::mem::size_of::<Contact>() < 256);
    println!(
        "LAYOUT JointSim={} Contact={} Payload={}",
        std::mem::size_of::<JointSim>(),
        std::mem::size_of::<Contact>(),
        std::mem::size_of::<ContactPayload>(),
    );
}

// "contact slabs preserve addresses and reset reused slots"
#[test]
fn stable_slabs() {
    let mut pool = ContactPayloadPool::new();
    // SAFETY: `initialize` returns a live, aligned slot inside one of the pool's slabs; the pool
    // owns that slot for its whole lifetime and only ever hands out this one at a time, so writing
    // through the pointer is in-bounds and hits initialized memory.
    let first = pool.initialize(0);
    let mut manifold = Manifold::default();
    manifold.points[0].normal_impulse = 7.0;
    unsafe {
        (*first).manifolds = ContactManifolds::from_manifold(manifold);
    }

    let mut contact = Contact::default();
    contact.payload = Some(first);
    let view = contact.manifold_buffer();
    for id in 1..1025 {
        let _ = pool.initialize(id);
    }
    assert_eq!(view[0].points[0].normal_impulse, 7.0);

    // SAFETY: `first` is still slot 0 of `pool`; the intervening `initialize` calls touched other
    // slots only, so this write lands on the same initialized payload the view reads.
    unsafe {
        (*first).manifolds.set(
            0,
            {
                let mut m = (*first).manifolds.get(0);
                m.points[0].normal_impulse = 9.0;
                m
            },
        );
    }
    assert_eq!(view[0].points[0].normal_impulse, 9.0);

    assert_eq!(pool.initialize(0), first);
    // SAFETY: `initialize(0)` just reset slot 0 to a fresh payload.
    unsafe {
        assert_eq!((*first).manifolds.count(), 0);
    }
}

// "inline and multiple manifold snapshots preserve value semantics"
#[test]
fn manifold_snapshots() {
    let mut manifolds = ContactManifolds::from_manifold(Manifold::default());

    let mut single = manifolds.clone();
    single.set(
        0,
        {
            let mut m = single.get(0);
            m.twist_impulse = 1.0;
            m
        },
    );
    assert_eq!(manifolds.get(0).twist_impulse, 0.0);

    manifolds.append(Manifold::default());
    let mut multiple = manifolds.clone();
    multiple.set(
        1,
        {
            let mut m = multiple.get(1);
            m.twist_impulse = 2.0;
            m
        },
    );
    assert_eq!(manifolds.get(1).twist_impulse, 0.0);

    manifolds.remove_all(true);
    manifolds.append(single.get(0));
    assert_eq!(manifolds.count(), 1);
    manifolds.with_unsafe_mutable_buffer_pointer(|buffer| {
        buffer[0].twist_impulse = 3.0;
    });
    assert_eq!(manifolds.get(0).twist_impulse, 3.0);
    assert_eq!(multiple.get(1).twist_impulse, 2.0);
}

// "default material mixers inline while custom callbacks preserve material ids"
#[test]
fn material_mixing() {
    let mut world = World::default();
    assert_eq!(world.mix_friction(0.25, 7, 1.0, 9), 0.5);
    assert_eq!(world.mix_restitution(0.25, 7, 1.0, 9), 1.0);

    world.friction_callback = Some(Arc::new(|a: f32, id_a: u64, b: f32, id_b: u64| {
        assert!(id_a == 7 && id_b == 9);
        a + b
    }));
    world.restitution_callback = Some(Arc::new(|a: f32, id_a: u64, b: f32, id_b: u64| {
        assert!(id_a == 7 && id_b == 9);
        a - b
    }));

    assert_eq!(world.mix_friction(0.25, 7, 1.0, 9), 1.25);
    assert_eq!(world.mix_restitution(0.25, 7, 1.0, 9), -0.75);

    world.friction_callback = None;
    world.restitution_callback = None;
    assert_eq!(world.mix_friction(0.25, 7, 1.0, 9), 0.5);
}
