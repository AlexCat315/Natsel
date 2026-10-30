//! Ported from Tests/NatselTests/BodyTests.swift.
//!
//! Body creation, solver-set placement, forces, impulses, transforms, and mass data must be
//! byte-identical across two runs.
//!
//! TODO(migration): the following `World`/`BodySim` accessors used here are not all `pub` yet —
//! make them `pub(crate)` (or add them) during the final build-fix phase: `World::body_sim_for`,
//! `World::body_state`, `World::apply_body_linear_impulse`, `World::set_body_transform`,
//! `World::body_mass_data`, `World::set_body_mass_data`, `World::body_world_inverse_rotational_inertia`,
//! `World::is_valid_body`. `World::body` is replaced by indexing `world.bodies[id.index1 as usize]`.

use crate::base::constants::NULL_INDEX;
use crate::math::types::{Matrix3, Pos, Quat, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::MassData;

#[test]
fn body_repeatability() {
    // "body creation, forces, and mass are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        // TODO(migration): add `impl Default for World` in the final build-fix phase.
        let mut world = World::default();

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        def.position = Pos::new(1.0, 2.0, 3.0);
        let s1 = world.create_body(&def);

        def = BodyDef::default();
        def.type_ = BodyType::KinematicBody;
        def.position = Pos::new(0.0, 1.0, 0.0);
        let k1 = world.create_body(&def);

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(1.0, 2.0, 3.0);
        def.rotation = Quat::IDENTITY;
        def.linear_velocity = Vec3::new(0.5, -1.25, 2.0);
        def.angular_velocity = Vec3::new(0.1, 0.2, -0.3);
        def.linear_damping = 0.1;
        def.angular_damping = 0.2;
        def.gravity_scale = 0.5;
        let d1 = world.create_body(&def);

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(-2.0, 0.0, 0.0);
        def.is_awake = false;
        let d2 = world.create_body(&def);

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(5.0, 5.0, 5.0);
        def.is_enabled = false;
        let d3 = world.create_body(&def);

        assert_eq!(world.solver_sets.len(), 4);
        assert_eq!(world.islands.len(), 3);

        assert_eq!(world.solver_sets[0].body_sims.len(), 1);
        assert_eq!(world.solver_sets[0].body_states.len(), 0);
        assert_eq!(world.solver_sets[0].island_sims.len(), 0);
        assert_eq!(world.solver_sets[1].body_sims.len(), 1);
        assert_eq!(world.solver_sets[2].body_sims.len(), 2);
        assert_eq!(world.solver_sets[2].body_states.len(), 2);
        assert_eq!(world.solver_sets[2].island_sims.len(), 2);
        assert_eq!(world.solver_sets[3].body_sims.len(), 1);
        assert_eq!(world.solver_sets[3].island_sims.len(), 1);

        // Static
        let body = &world.bodies[s1.index1 as usize];
        assert_eq!(body.set_index, 0);
        assert_eq!(body.local_index, 0);
        assert_eq!(body.generation, 1);
        assert_eq!(body.id.index1, 0);
        assert_eq!(body.island_id, NULL_INDEX);
        assert_eq!(body.flags, 0x0000_6000);
        let sim = world.body_sim_for(s1);
        assert_eq!(sim.transform.p, Pos::new(1.0, 2.0, 3.0));
        samples.push(sim.transform.q.s.to_bits() as u64);
        assert!((sim.center.x - 1.0).abs() < 1e-6);
        assert_eq!(sim.inv_mass, 0.0);
        assert_eq!(sim.min_extent, 100_000.0);
        assert_eq!(sim.max_extent.x, 0.0);
        assert_eq!(sim.flags, 0x0000_6000);

        // Kinematic
        let body = &world.bodies[k1.index1 as usize];
        assert_eq!(body.set_index, 2);
        assert_eq!(body.local_index, 0);
        assert_eq!(body.island_id, 0);
        assert_eq!(body.island_index, 0);
        assert_eq!(body.flags, 0x0000_6000);

        // Dynamic
        let body = &world.bodies[d1.index1 as usize];
        assert_eq!(body.set_index, 2);
        assert_eq!(body.local_index, 1);
        assert_eq!(body.island_id, 1);
        assert_eq!(body.flags, 0x0000_7000);
        let sim = world.body_sim_for(d1);
        samples.push(sim.transform.q.s.to_bits() as u64);
        assert!((world.body_linear_velocity(d1).x - 0.5).abs() < 1e-6);
        samples.push(world.body_angular_velocity(d1).z.to_bits() as u64);
        samples.push(world.body_state(d1).expect("body state").delta_rotation.s.to_bits() as u64);

        // Sleeping
        let body = &world.bodies[d2.index1 as usize];
        assert_eq!(body.set_index, 3);
        assert_eq!(body.local_index, 0);
        assert_eq!(body.island_id, 2);

        // Disabled
        let body = &world.bodies[d3.index1 as usize];
        assert_eq!(body.set_index, 1);
        assert_eq!(body.island_id, NULL_INDEX);

        // Forces
        world.apply_body_force(d1, Vec3::new(10.0, 20.0, 30.0), Pos::new(1.5, 2.5, 3.5), false);
        world.apply_body_torque(d1, Vec3::new(1.0, -2.0, 3.0), false);
        let sim = world.body_sim_for(d1);
        samples.push(sim.force.x.to_bits() as u64);
        samples.push(sim.force.y.to_bits() as u64);
        samples.push(sim.torque.z.to_bits() as u64);

        // Impulse
        world.apply_body_linear_impulse(
            d1,
            Vec3::new(1.0, 2.0, 3.0),
            Pos::new(1.5, 2.5, 3.5),
            false,
        );
        samples.push(world.body_linear_velocity(d1).x.to_bits() as u64);
        samples.push(world.body_linear_velocity(d1).y.to_bits() as u64);
        samples.push(world.body_angular_velocity(d1).x.to_bits() as u64);

        // Transform
        world.set_body_transform(d1, Pos::new(4.0, 5.0, 6.0), Quat::IDENTITY);
        let sim = world.body_sim_for(d1);
        assert_eq!(sim.transform.p, Pos::new(4.0, 5.0, 6.0));
        samples.push(sim.transform.q.s.to_bits() as u64);
        assert_eq!(sim.center, Pos::new(4.0, 5.0, 6.0));

        // Mass data
        let mut mass_data = MassData::default();
        mass_data.mass = 5.0;
        mass_data.center = Vec3::new(0.1, 0.2, 0.3);
        mass_data.inertia = Matrix3::new(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
            Vec3::new(0.0, 0.0, 3.0),
        );
        world.set_body_mass_data(d1, mass_data);

        let got = world.body_mass_data(d1);
        samples.push(got.mass.to_bits() as u64);
        samples.push(got.center.x.to_bits() as u64);
        samples.push(got.inertia.cx.x.to_bits() as u64);
        samples.push(got.inertia.cy.y.to_bits() as u64);

        let center = world.body_world_center(d1);
        samples.push(center.x.to_bits() as u64);
        samples.push(center.y.to_bits() as u64);
        samples.push(center.z.to_bits() as u64);

        let inverse = world.body_world_inverse_rotational_inertia(d1);
        samples.push(inverse.cx.x.to_bits() as u64);
        samples.push(inverse.cy.y.to_bits() as u64);

        // Destroy
        world.destroy_body(k1);
        assert_eq!(world.solver_sets.len(), 4);
        assert_eq!(world.bodies.len(), 5);
        assert!(!world.is_valid_body(k1));
        assert!(world.is_valid_body(d1));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
