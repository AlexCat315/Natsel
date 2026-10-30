//! Ported from Tests/NatselTests/ShapeCreationTests.swift.
//!
//! Shape creation, hull interning, body mass bookkeeping, and shape destruction must be
//! byte-identical across two runs.
//!
//! TODO(migration): the following `World`/`Shape`/`BodySim` accessors used here are not all `pub`
//! yet — make them `pub(crate)` (or add them) during the final build-fix phase so this module
//! compiles: `World::shape`, `World::body_sim_for`, `World::destroy_shape`, `World::body_mass_data`,
//! `World::set_body_mass_data`, `World::body_world_inverse_rotational_inertia`.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::shape_runtime::ShapeGeometry;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::shape_def::ShapeDef;
use crate::types::shape_def::ShapeType;

fn hull_hash(shape: &crate::simulation::shape_runtime::Shape) -> u64 {
    match &shape.geometry {
        ShapeGeometry::Hull(h) => h.hash(),
        _ => 0,
    }
}

#[test]
fn create_shapes() {
    // "shape creation and body mass are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        // TODO(migration): add `impl Default for World` in the final build-fix phase.
        let mut world = World::default();

        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(1.0, 2.0, 3.0);
        let body = world.create_body(&def);

        let mut shape_def = ShapeDef::default();
        shape_def.density = 2.0;
        let sh1 = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5));

        shape_def = ShapeDef::default();
        shape_def.density = 2.0;
        let sh2 = world.create_capsule_shape(
            body,
            &shape_def,
            Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5),
        );

        shape_def = ShapeDef::default();
        shape_def.density = 1.0;
        let box_hull = make_box_hull(1.0, 2.0, 3.0);
        let sh3 = world.create_hull_shape(body, &shape_def, &box_hull);
        let sh4 = world.create_hull_shape(body, &shape_def, &box_hull);

        assert_eq!(world.shapes.len(), 4);
        assert_eq!(world.fat_aabbs.len(), 4);

        // Sphere
        let shape = world.shape(sh1);
        assert_eq!(shape.id, 0);
        assert_eq!(shape.body_id, body.index1);
        assert_eq!(shape.type_, ShapeType::SphereShape);
        samples.push(shape.density.to_bits() as u64);
        samples.push(shape.aabb_margin.to_bits() as u64);
        assert_eq!(shape.proxy_key, 2);
        assert_eq!(shape.prev_shape_id, 1);
        assert_eq!(shape.next_shape_id, -1);
        assert_eq!(shape.materials.len(), 1);
        assert_eq!(shape.generation, 1);
        assert_eq!(shape.local_centroid, Vec3::new(0.0, 0.0, 0.0));
        samples.push(shape.aabb.lower_bound.x.to_bits() as u64);
        samples.push(shape.aabb.lower_bound.y.to_bits() as u64);
        samples.push(shape.aabb.upper_bound.z.to_bits() as u64);
        let fat = world.fat_aabbs[0];
        samples.push(fat.lower_bound.x.to_bits() as u64);
        samples.push(fat.upper_bound.z.to_bits() as u64);

        // Capsule
        let shape = world.shape(sh2);
        assert_eq!(shape.proxy_key, 6);
        assert_eq!(shape.prev_shape_id, 2);
        assert_eq!(shape.next_shape_id, 0);
        samples.push(shape.aabb.lower_bound.y.to_bits() as u64);
        samples.push(shape.aabb.upper_bound.y.to_bits() as u64);
        let fat = world.fat_aabbs[1];
        samples.push(fat.upper_bound.y.to_bits() as u64);

        // Hulls
        let shape = world.shape(sh3);
        assert_eq!(shape.proxy_key, 10);
        assert_eq!(shape.prev_shape_id, 3);
        assert_eq!(shape.next_shape_id, 1);
        samples.push(shape.aabb.lower_bound.x.to_bits() as u64);
        samples.push(shape.aabb.upper_bound.z.to_bits() as u64);
        samples.push(hull_hash(&world.shape(sh3)));

        let shape = world.shape(sh4);
        assert_eq!(shape.proxy_key, 14);
        assert_eq!(shape.prev_shape_id, -1);
        assert_eq!(shape.next_shape_id, 2);
        samples.push(hull_hash(&world.shape(sh4)));

        // Identical hulls are interned
        assert_eq!(hull_hash(&world.shape(sh3)), hull_hash(&world.shape(sh4)));

        // Body mass properties
        let b = &world.bodies[body.index1 as usize];
        samples.push(b.mass.to_bits() as u64);
        assert_eq!(b.head_shape_id, 3);
        assert_eq!(b.shape_count, 4);
        samples.push(b.flags as u64);

        let sim = world.body_sim_for(body);
        samples.push(sim.inv_mass.to_bits() as u64);
        assert_eq!(sim.center, Pos::new(1.0, 2.0, 3.0));
        assert_eq!(sim.local_center, Vec3::new(0.0, 0.0, 0.0));
        samples.push(sim.min_extent.to_bits() as u64);
        assert_eq!(sim.max_extent, Vec3::new(1.0, 2.0, 3.0));
        samples.push(sim.inertia.cx.x.to_bits() as u64);
        samples.push(sim.inertia.cy.y.to_bits() as u64);
        samples.push(sim.inertia.cz.z.to_bits() as u64);

        // Destroy one of the two interned hull shapes.
        world.destroy_shape(sh3, true);
        assert_eq!(world.shapes.len(), 4);
        assert_eq!(world.shapes[2].id, crate::base::constants::NULL_INDEX);
        let updated_body = &world.bodies[body.index1 as usize];
        assert_eq!(updated_body.head_shape_id, 3);
        assert_eq!(updated_body.shape_count, 3);
        samples.push(updated_body.mass.to_bits() as u64);

        let updated_sim = world.body_sim_for(body);
        samples.push(updated_sim.inv_mass.to_bits() as u64);
        samples.push(updated_sim.inertia.cx.x.to_bits() as u64);

        // The remaining shape still references the interned hull.
        samples.push(hull_hash(&world.shape(sh4)));
        assert_eq!(world.shape(sh4).proxy_key, 14);

        world.destroy_shape(sh4, true);
        let updated_body = &world.bodies[body.index1 as usize];
        samples.push(updated_body.mass.to_bits() as u64);
        let updated_sim = world.body_sim_for(body);
        samples.push(updated_sim.inv_mass.to_bits() as u64);
        assert_eq!(updated_body.shape_count, 2);
        assert_eq!(world.shapes[3].id, crate::base::constants::NULL_INDEX);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
