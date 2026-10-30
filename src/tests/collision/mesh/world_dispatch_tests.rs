//! Ported from Tests/NatselTests/Collision/Mesh/WorldDispatchTests.swift.
//!
//! Mesh creation, scaled casts, overlap, and destruction must all dispatch through world
//! transforms.

use crate::collision::mesh::overlap::shape_cast_mesh;
use crate::collision::mesh::procedural::create_grid_mesh;
use crate::math::functions::{to_relative_transform, transform_world_point};
use crate::math::types::{Pos, Vec3, WorldTransform};
use crate::simulation::shape_runtime::shape_cast_shape;
use crate::simulation::World;
use crate::tests::world_position_assertions::distance;
use crate::types::body_def::BodyDef;
use crate::types::geometry::Mesh;
use crate::types::queries::{ShapeCastInput, ShapeProxy};
use crate::types::shape_def::{QueryFilter, ShapeDef, SurfaceMaterial};

#[test]
fn queries() {
    // "mesh creation, scaled casts, overlap, and destruction use world transforms"
    let mut world = World::default();
    let mut body_def = BodyDef::default();
    body_def.position = Pos::new(1.0, 2.0, -1.0);
    let body = world.create_body(&body_def);
    let data = create_grid_mesh(5, 4, 0.75, 3, true).expect("mesh creation failed");
    let mut shape_def = ShapeDef::default();
    shape_def.materials = (0..3)
        .map(|i: i32| {
            let mut material = SurfaceMaterial::default();
            material.user_material_id = (10 * (i + 1)) as u64;
            material
        })
        .collect();
    let shape =
        world.create_mesh_shape(body, &shape_def, data.clone(), Vec3::new(-2.0, 0.5, 1.5));
    assert!(world.shape_is_valid(shape));
    let origin = Pos::new(1.125, 4.0, -0.75);
    let translation = Vec3::new(0.0, -4.0, 0.0);
    let ray = world.cast_ray_closest(origin, translation, QueryFilter::default_filter());
    assert!(ray.hit);
    assert_eq!(ray.shape_id, shape);
    assert_eq!(ray.fraction, 0.5);
    assert_eq!(ray.point, Pos::new(1.125, 2.0, -0.75));
    assert_eq!(ray.normal, Vec3::AXIS_Y);
    assert!([10u64, 20, 30].contains(&ray.user_material_id));

    let mut center = Vec3::new(1.125, 2.1, -0.75);
    {
        let mut count = 0;
        let proxy = ShapeProxy::new(Some(&center), 1, 0.25);
        let _ = world.overlap_shape(Pos::ZERO, proxy, QueryFilter::default_filter(), |id| {
            assert_eq!(id, shape);
            count += 1;
            true
        });
        assert_eq!(count, 1);
    }
    let _ = &mut center;

    let mesh_shape = &world.shape(shape);
    let mesh = Mesh::new(Some(data), Vec3::new(-2.0, 0.5, 1.5));
    let transform = WorldTransform::new(body_def.position, crate::math::types::Quat::IDENTITY);
    let mut point = Vec3::new(1.125, 4.0, -0.75);
    {
        let mut input = ShapeCastInput::default();
        input.proxy = ShapeProxy::new(Some(&point), 1, 0.25);
        input.translation = translation;
        input.max_fraction = 1.0;
        let output = shape_cast_shape(mesh_shape, to_relative_transform(transform, Pos::ZERO), &input);
        let local_point = crate::math::functions::sub_pos(Pos::from_vec3(point), body_def.position);
        let mut local_input = input;
        local_input.proxy = ShapeProxy::new(Some(&local_point), 1, 0.25);
        let local = shape_cast_mesh(&mesh, local_input);
        assert!(output.hit);
        assert_eq!(output.fraction.to_bits(), local.fraction.to_bits());
        assert!(
            distance(
                Pos::new(output.point.x as f64, output.point.y as f64, output.point.z as f64),
                transform_world_point(transform, local.point)
            ) < 1.0e-5
        );
        assert_eq!(output.triangle_index, local.triangle_index);
    }
    let _ = &mut point;

    world.destroy_shape(shape, true);
    assert!(!world.shape_is_valid(shape));
    assert!(!world.cast_ray_closest(origin, translation, QueryFilter::default_filter()).hit);
}
