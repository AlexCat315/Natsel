//! Ported from Tests/NatselTests/Collision/Compound/Fixture.swift.
//!
//! The shared compound fixtures used by the compound tests and by
//! `src/tests/simulation/mixed_query_scene.rs`.

use crate::collision::compound::compound::Compound;
use crate::collision::hull::make_box_hull;
use crate::collision::mesh::procedural::create_grid_mesh;
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Transform, Vec3};
use crate::types::geometry::{
    Capsule, CompoundCapsuleDef, CompoundDef, CompoundHullDef, CompoundMeshDef, CompoundSphereDef,
    Sphere,
};
use crate::types::shape_def::SurfaceMaterial;

/// Swift's `makeTestCompound(_ scenario:)`.
///
/// The six scenarios cover: a lone sphere, two capsules, two hulls, two meshes, a mixed bag, and
/// everything at once.
pub fn make_test_compound(scenario: i32) -> Option<Compound> {
    let capsule_counts = [0, 2, 0, 0, 2, 6];
    let hull_counts = [0, 0, 2, 0, 2, 6];
    let mesh_counts = [0, 0, 0, 2, 2, 4];
    let sphere_counts = [1, 0, 0, 0, 3, 8];

    let mut a = SurfaceMaterial::default_material();
    a.friction = 0.4;
    a.restitution = 0.1;
    a.rolling_resistance = 0.05;
    a.tangent_velocity = Vec3::new(0.125, 0.0, 0.25);
    a.user_material_id = 5;
    let mut b = a;
    b.friction = 0.7;
    b.user_material_id = 6;
    let materials = [a, b, a];

    let hulls = [make_box_hull(0.5, 0.25, 0.75), make_box_hull(0.4, 0.35, 0.6)];
    let mesh = create_grid_mesh(2, 3, 0.5, 3, true)?;

    let mut definition = CompoundDef::default();
    for i in 0..capsule_counts[scenario as usize] {
        let item = CompoundCapsuleDef {
            capsule: Capsule::new(
                Vec3::new(i as f32 - 1.0, -0.5, 0.25),
                Vec3::new(i as f32 - 1.0, 0.5, 0.25),
                0.2,
            ),
            material: materials[(i % 2) as usize],
        };
        definition.capsules.push(item);
    }
    for i in 0..hull_counts[scenario as usize] {
        let item = CompoundHullDef {
            hull: Some(hulls[if i % 3 == 2 { 1 } else { 0 }].clone()),
            transform: Transform::new(
                Vec3::new(i as f32 * 0.75 - 1.0, 0.25, -0.5),
                make_quat_from_axis_angle(Vec3::AXIS_Y, i as f32 * 0.1),
            ),
            material: materials[(i % 2) as usize],
        };
        definition.hulls.push(item);
    }
    for i in 0..mesh_counts[scenario as usize] {
        let item = CompoundMeshDef {
            mesh_data: Some(mesh.clone()),
            transform: Transform::new(
                Vec3::new(i as f32 * 1.5, -0.5, 0.5),
                make_quat_from_axis_angle(Vec3::AXIS_Y, i as f32 * 0.1),
            ),
            scale: Vec3::new(if i % 2 == 0 { 1.0 } else { -1.0 }, 1.0, 1.25),
            materials: materials.to_vec(),
        };
        definition.meshes.push(item);
    }
    for i in 0..sphere_counts[scenario as usize] {
        let item = CompoundSphereDef {
            sphere: Sphere::new(Vec3::new(i as f32 * 0.5 - 1.0, 0.25, 1.0), 0.3),
            material: materials[(i % 2) as usize],
        };
        definition.spheres.push(item);
    }
    Compound::new(&definition)
}
