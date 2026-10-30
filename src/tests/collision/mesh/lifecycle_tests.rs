//! Ported from Tests/NatselTests/Collision/Mesh/LifecycleTests.swift.
//!
//! Disjoint, shape removal, and body removal must unlink every mesh constraint and leave no stale
//! graph or island references.

use crate::base::constants as b3;
use crate::collision::mesh::procedural::create_grid_mesh;
use crate::math::types::{Pos, Quat, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

#[test]
fn removal() {
    // "disjoint, shape removal, and body removal unlink mesh constraints"
    // Swift's `arguments: 0..<5` expands to one case per action.
    for action in 0..5i32 {
        let mut world = World::default();
        world.enable_sleep = action >= 3;
        let ground = world.create_body(&BodyDef::default());
        let geometry = create_grid_mesh(4, 4, 1.0, 0, true).expect("mesh creation failed");
        let _ = world.create_mesh_shape(ground, &ShapeDef::default(), geometry, Vec3::ONE);
        let mut body_def = BodyDef::default();
        body_def.type_ = BodyType::DynamicBody;
        body_def.position = Pos::new(0.13, 0.39, 0.17);
        let body = world.create_body(&body_def);
        let shape = world.create_sphere_shape(
            body,
            &ShapeDef::default(),
            Sphere::new(Vec3::ZERO, 0.4),
        );
        world.step(1.0 / 60.0, 4);
        assert!(world
            .contacts
            .iter()
            .any(|c| c.contact_id != b3::NULL_INDEX && c.manifold_count > 0));
        if action >= 3 {
            for _ in 0..180 {
                world.step(1.0 / 60.0, 4);
            }
            assert!(!world.body_is_awake(body));
        }
        match action {
            0 => world.set_body_transform(body, Pos::new(20.0, 20.0, 20.0), Quat::IDENTITY),
            1 | 3 => world.destroy_shape(shape, true),
            _ => world.destroy_body(body),
        }
        world.step(1.0 / 60.0, 4);
        assert!(world
            .contacts
            .iter()
            .all(|c| c.contact_id == b3::NULL_INDEX));
        assert!(world
            .constraint_graph
            .colors
            .iter()
            .all(|c| c.contacts.is_empty() && c.convex_contacts.is_empty()));
        // Reuse the freed IDs and advance again to catch stale graph/island references.
        body_def.position = Pos::new(10.0, 10.0, 10.0);
        let replacement = world.create_body(&body_def);
        let _ = world.create_sphere_shape(
            replacement,
            &ShapeDef::default(),
            Sphere::new(Vec3::ZERO, 0.4),
        );
        world.step(1.0 / 60.0, 4);
        assert!(world
            .contacts
            .iter()
            .all(|c| c.contact_id == b3::NULL_INDEX));
    }
}
