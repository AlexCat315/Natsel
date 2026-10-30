//! Ported from Tests/NatselTests/Simulation/ShapeWorldExtraTests.swift.

use std::ffi::c_void;

use crate::collision::hull::make_box_hull;
use crate::collision::mesh::storage::TriangleMesh;
use crate::math::types::Vec3;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::{MeshBuildOptions, ShapeDef, SurfaceMaterial};

fn make_body(world: &mut World) -> crate::types::ids::BodyID {
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    world.create_body(&def)
}

#[test]
fn sensor_and_user_data() {
    let mut world = World::default();
    let body = make_body(&mut world);

    let mut shape_def = ShapeDef::default();
    shape_def.is_sensor = true;
    let sensor = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5));
    assert!(world.shape_is_sensor(sensor));

    shape_def = ShapeDef::default();
    let solid = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(5.0, 0.0, 0.0), 0.5));
    assert!(!world.shape_is_sensor(solid));

    let mut token: i32 = 42;
    let pointer = (&mut token as *mut i32) as *mut c_void;
    world.set_shape_user_data(solid, Some(pointer));
    assert_eq!(world.shape_user_data(solid), Some(pointer));

    world.set_shape_user_data(solid, None);
    assert_eq!(world.shape_user_data(solid), None);
}

#[test]
fn shape_aabb() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = crate::math::types::Pos::new(3.0, 4.0, 5.0);
    let body = world.create_body(&def);

    let shape_def = ShapeDef::default();
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let aabb = world.shape_aabb(shape);
    assert!((aabb.lower_bound.x - 2.5).abs() < 1e-4);
    assert!((aabb.lower_bound.y - 3.5).abs() < 1e-4);
    assert!((aabb.lower_bound.z - 4.5).abs() < 1e-4);
    assert!((aabb.upper_bound.x - 3.5).abs() < 1e-4);
    assert!((aabb.upper_bound.y - 4.5).abs() < 1e-4);
    assert!((aabb.upper_bound.z - 5.5).abs() < 1e-4);
}

#[test]
fn closest_point() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = crate::math::types::Pos::new(0.0, 0.0, 0.0);
    let body = world.create_body(&def);

    let shape_def = ShapeDef::default();
    let shape = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0));

    let point = world.shape_closest_point(shape, crate::math::types::Pos::new(5.0, 0.0, 0.0));
    assert!((point.x - 1.0).abs() < 1e-4);
    assert!(point.y.abs() < 1e-4);
    assert!(point.z.abs() < 1e-4);

    let inside = world.shape_closest_point(shape, crate::math::types::Pos::new(0.25, 0.0, 0.0));
    let d = (inside.x * inside.x + inside.y * inside.y + inside.z * inside.z).sqrt();
    assert!((d - 1.0).abs() < 1e-3 || d < 1e-3);
}

#[test]
fn mesh_materials() {
    let mut world = World::default();
    world.enable_sleep = false;

    let vertices = vec![
        Vec3::new(-3.0, 0.0, -1.0),
        Vec3::new(-2.0, 0.0, 1.0),
        Vec3::new(-1.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, -1.0),
        Vec3::new(2.0, 0.0, 1.0),
        Vec3::new(3.0, 0.0, -1.0),
    ];
    let indices = vec![0, 1, 2, 3, 4, 5];
    let materials = [0u8, 1u8];
    let mesh = TriangleMesh::new(&vertices, &indices, Some(&materials), MeshBuildOptions::default())
        .expect("mesh creation failed");

    let body = make_body(&mut world);
    let mut shape_def = ShapeDef::default();
    let mut material_a = SurfaceMaterial::default_material();
    material_a.friction = 0.2;
    let mut material_b = SurfaceMaterial::default_material();
    material_b.friction = 0.8;
    shape_def.materials = vec![material_a, material_b];

    let shape = world.create_mesh_shape(body, &shape_def, mesh, Vec3::new(1.0, 1.0, 1.0));

    assert_eq!(world.shape_mesh_material_count(shape), 2);
    assert!((world.shape_mesh_surface_material(shape, 0).unwrap().friction - 0.2).abs() < 1e-6);
    assert!((world.shape_mesh_surface_material(shape, 1).unwrap().friction - 0.8).abs() < 1e-6);

    assert!(world.shape_mesh_surface_material(shape, 2).is_none());

    let mut replacement = SurfaceMaterial::default_material();
    replacement.friction = 0.55;
    world.set_shape_mesh_material(shape, 1, replacement);
    assert!((world.shape_mesh_surface_material(shape, 1).unwrap().friction - 0.55).abs() < 1e-6);
}

#[test]
fn event_toggles() {
    let mut world = World::default();
    let body = make_body(&mut world);

    let shape_def = ShapeDef::default();
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    assert!(!world.shape_are_hit_events_enabled(shape));
    world.enable_shape_hit_events(shape, true);
    assert!(world.shape_are_hit_events_enabled(shape));
    world.enable_shape_hit_events(shape, false);
    assert!(!world.shape_are_hit_events_enabled(shape));

    world.enable_shape_contact_events(shape, true);
    assert!(world.shape_are_contact_events_enabled(shape));

    world.enable_shape_sensor_events(shape, true);
    assert!(world.shape_are_sensor_events_enabled(shape));

    world.enable_shape_pre_solve_events(shape, true);
    assert!(world.shape_are_pre_solve_events_enabled(shape));
}

#[test]
fn world_bounds() {
    let mut world = World::default();
    world.enable_sleep = false;

    assert!(world.world_bounds().is_none());

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = crate::math::types::Pos::new(-5.0, 0.0, 0.0);
    let left = world.create_body(&def);
    let _ = world.create_hull_shape(left, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = crate::math::types::Pos::new(5.0, 0.0, 0.0);
    let right = world.create_body(&def);
    let _ = world.create_hull_shape(right, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    world.step(1.0f32 / 60.0, 1);

    let bounds = world.world_bounds().expect("bounds missing");
    assert!(bounds.lower_bound.x <= -5.5 + 1e-3);
    assert!(bounds.upper_bound.x >= 5.5 - 1e-3);
}

#[test]
fn public_flags() {
    let mut world = World::default();

    assert!(world.enable_speculative);
    world.enable_speculative = false;
    assert!(!world.enable_speculative);

    assert!(!world.enable_restitution_propagation);
    world.enable_restitution_propagation = true;
    assert!(world.enable_restitution_propagation);
}
