//! Ported from Tests/NatselTests/Collision/Compound/DynamicInstanceTests.swift.
//!
//! Convex compound mass, transforms, and materials must match explicit fixtures, and the instance
//! must survive replacement, destruction, and rejection of mesh children.

use crate::collision::hull::make_offset_box_hull;
use crate::math::functions::transform_point;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::tests::collision::compound::fixture::make_test_compound;
use crate::tests::world_position_assertions::distance;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;

fn body(world: &mut World) -> BodyID {
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 2.0, 0.0);
    world.create_body(&def)
}

#[test]
fn mass_and_queries() {
    // "convex compound mass, transforms, and materials match explicit fixtures"
    let mut world = World::default();
    let id = body(&mut world);
    let compound = make_test_compound(2).unwrap();
    let instance = world
        .create_dynamic_compound(id, &ShapeDef::default(), &compound)
        .unwrap();
    assert!(world.dynamic_compound_is_valid(&instance));
    assert_eq!(instance.child_shape_ids.len(), compound.child_count());
    let mut expected_mass: f32 = 0.0;
    for i in 0..compound.child_count() {
        let child = compound.child(i);
        let hull = match &child.geometry {
            crate::collision::compound::compound::CompoundChildGeometry::Hull(hull) => hull,
            _ => continue,
        };
        expected_mass += hull.volume() * 1000.0;
        let local = world.shape_hull(instance.child_shape_ids[i]).unwrap();
        assert!(
            distance(local.center(), transform_point(child.transform, hull.center())) < 1.0e-5
        );
        let material = world.shape(instance.child_shape_ids[i]).material(0);
        assert_eq!(
            material,
            compound.materials()[child.material_indices[0] as usize]
        );
    }
    assert!((world.body_mass(id) - expected_mass).abs() < expected_mass * 1.0e-5);

    let hit = world.cast_ray_closest(
        Pos::new(-1.0, 5.0, -0.5),
        Vec3::new(0.0, -6.0, 0.0),
        crate::types::shape_def::QueryFilter::default_filter(),
    );
    assert!(hit.hit);
    assert!(instance.child_index_of(hit.shape_id).is_some());
}

#[test]
fn lifecycle() {
    // "compound fixtures collide, replace, and restore across structural changes"
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(10.0, 0.5, 10.0, Vec3::new(0.0, -0.5, 0.0)),
    );
    let id = body(&mut world);
    let original = make_test_compound(2).unwrap();
    let instance = world
        .create_dynamic_compound(id, &ShapeDef::default(), &original)
        .unwrap();
    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
    }
    assert!(world.body_position(id).y > -0.5);
    assert!(world.touching_contact_count() > 0);
    let saved = world.save_state();
    let sphere = make_test_compound(0).unwrap();
    let replacement = world
        .replace_dynamic_compound(&instance, &ShapeDef::default(), &sphere)
        .unwrap();
    assert!(!world.dynamic_compound_is_valid(&instance));
    assert!(world.dynamic_compound_is_valid(&replacement));
    world.destroy_dynamic_compound(&replacement, true);
    assert_eq!(world.body(id).shape_count, 0);
    assert!(world.restore_state(&saved));
    assert!(world.dynamic_compound_is_valid(&instance));
    world.step(1.0 / 60.0, 4);
}

#[test]
fn unsupported_geometry_is_atomic() {
    // "mesh children are rejected without attaching any shapes"
    let mut world = World::default();
    let id = body(&mut world);
    let mixed = make_test_compound(4).unwrap();
    let convex = make_test_compound(0).unwrap();
    assert!(world
        .create_dynamic_compound(id, &ShapeDef::default(), &mixed)
        .is_none());
    assert_eq!(world.body(id).shape_count, 0);
    let instance = world
        .create_dynamic_compound(id, &ShapeDef::default(), &convex)
        .unwrap();
    let mass = world.body_mass(id);
    assert!(world
        .replace_dynamic_compound(&instance, &ShapeDef::default(), &mixed)
        .is_none());
    assert!(world.dynamic_compound_is_valid(&instance));
    assert_eq!(world.body_mass(id), mass);
}
