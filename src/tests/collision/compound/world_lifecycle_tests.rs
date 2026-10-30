//! Ported from Tests/NatselTests/Collision/Compound/WorldLifecycleTests.swift.
//!
//! Compound queries through the world must use body transforms and embedded materials, and
//! destroying a compound must remove every child contact.

use crate::base::constants as b3;
use crate::collision::compound::child_queries::{overlap_compound, ray_cast_compound, shape_cast_compound};
use crate::math::functions::{
    inv_rotate_vector, inv_transform_world_point, make_quat_from_axis_angle, rotate_vector,
    to_relative_transform, transform_world_point,
};
use crate::math::types::{Pos, Transform, Vec3, WorldTransform};
use crate::simulation::shape_runtime::shape_cast_shape;
use crate::simulation::World;
use crate::tests::collision::compound::fixture::make_test_compound;
use crate::tests::world_position_assertions::distance;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::queries::{RayCastInput, ShapeCastInput, ShapeProxy};
use crate::types::shape_def::{QueryFilter, ShapeDef};

#[test]
fn queries() {
    // "compound queries use body transforms and embedded materials"
    let mut world = World::default();
    let mut body_def = BodyDef::default();
    body_def.position = Pos::new(2.0, 3.0, -1.0);
    body_def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Y, 0.3);
    let body = world.create_body(&body_def);
    let compound = make_test_compound(4).unwrap();
    let shape_id = world.create_compound_shape(body, &ShapeDef::default(), &compound);
    assert!(world.shape_is_valid(shape_id));
    assert_eq!(world.compound_shape_count_value(), 1);
    let shape = world.shape(shape_id);
    let transform = WorldTransform::new(body_def.position, body_def.rotation);
    for i in 0..20i32 {
        let origin = Vec3::new(i as f32 * 0.15 - 1.0, 2.0, 0.5);
        let translation = Vec3::new(0.0, -4.0, 0.0);
        let world_origin = transform_world_point(transform, origin);
        let world_translation = rotate_vector(transform.q, translation);
        let ray = RayCastInput::new(
            inv_transform_world_point(transform, world_origin),
            inv_rotate_vector(transform.q, world_translation),
            1.0,
        );
        let expected = ray_cast_compound(&compound, ray);
        let actual = world.cast_ray_closest(world_origin, world_translation, QueryFilter::default_filter());
        assert_eq!(actual.hit, expected.hit);
        if expected.hit {
            assert_eq!(actual.child_index, expected.child_index);
            assert_eq!(actual.triangle_index, expected.triangle_index);
            assert_eq!(
                actual.user_material_id,
                compound.materials()[expected.material_index as usize].user_material_id
            );
            assert!((actual.fraction - expected.fraction).abs() < 0.00001);
        }

        let point = Vec3::new(origin.x, 0.0, origin.z);
        let proxy = ShapeProxy::new(Some(&point), 1, 0.3);
        let expected_overlap = overlap_compound(&compound, Transform::IDENTITY, proxy);
        let world_point = transform_world_point(transform, point);
        {
            let zero = Vec3::ZERO;
            let world_proxy = ShapeProxy::new(Some(&zero), 1, 0.3);
            let mut count = 0;
            let _ = world.overlap_shape(world_point, world_proxy, QueryFilter::default_filter(), |_| {
                count += 1;
                true
            });
            assert_eq!(count == 1, expected_overlap);
        }

        let mut input = ShapeCastInput::default();
        input.proxy = proxy;
        input.translation = translation;
        input.max_fraction = 1.0;
        input.can_encroach = true;
        let cast = shape_cast_shape(&shape, to_relative_transform(transform, Pos::ZERO), &input);
        let local_point = inv_transform_world_point(transform, Pos::from_vec3(point));
        let expected_cast = {
            let mut local = input;
            local.proxy = ShapeProxy::new(Some(&local_point), 1, 0.3);
            local.translation = inv_rotate_vector(transform.q, input.translation);
            shape_cast_compound(&compound, &local)
        };
        assert_eq!(cast.hit, expected_cast.hit);
        assert_eq!(cast.fraction.to_bits(), expected_cast.fraction.to_bits());
        assert_eq!(cast.child_index, expected_cast.child_index);
        assert_eq!(cast.material_index, expected_cast.material_index);
        assert!(
            distance(
                Pos::from_vec3(cast.point),
                transform_world_point(transform, expected_cast.point)
            ) < 1.0e-5
        );
    }
    // `shape` borrowed the world for the query loop above; release it before mutating.
    drop(shape);
    world.destroy_shape(shape_id, true);
    assert_eq!(world.compound_shape_count_value(), 0);
    assert!(!world.shape_is_valid(shape_id));
    body_def.type_ = BodyType::DynamicBody;
    let dynamic = world.create_body(&body_def);
    let dynamic_shape = world.create_compound_shape(dynamic, &ShapeDef::default(), &compound);
    assert!(!world.shape_is_valid(dynamic_shape));
}

#[test]
fn destruction() {
    // "destroying a compound removes every child contact"
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    let compound = make_test_compound(5).unwrap();
    let shape = world.create_compound_shape(ground, &ShapeDef::default(), &compound);
    let mut definition = BodyDef::default();
    definition.type_ = BodyType::DynamicBody;
    definition.position = Pos::new(-0.4, 0.7, 0.4);
    let body = world.create_body(&definition);
    let _ = world.create_sphere_shape(body, &ShapeDef::default(), Sphere::new(Vec3::ZERO, 0.5));
    world.step(1.0 / 60.0, 4);
    assert!(world.contacts.iter().any(|c| c.contact_id != b3::NULL_INDEX));
    world.destroy_shape(shape, true);
    assert!(!world.contacts.iter().any(|c| c.contact_id != b3::NULL_INDEX));
    assert_eq!(world.compound_shape_count_value(), 0);
    world.step(1.0 / 60.0, 4);
}
