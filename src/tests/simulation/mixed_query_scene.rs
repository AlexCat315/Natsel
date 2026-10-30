//! Ported from Tests/NatselTests/Simulation/MixedQueryScene.swift.
//!
//! A scene builder rather than a test module: it lays out one body per shape kind so the query
//! suites can exercise every dispatch path from a single world.

use crate::collision::hull::make_box_hull;
use crate::collision::mesh::procedural::create_grid_mesh;
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::tests::collision::compound::fixture::make_test_compound;
use crate::tests::collision::height_field::encoding_tests::make_test_height_field;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::shape_def::{ShapeDef, SurfaceMaterial};

/// Build the mixed-shape scene at `scenario`'s origin. `sensors` turns every shape into a sensor.
///
/// Swift threw through `#require`; the ported fixtures return `Option`, so a missing fixture is a
/// hard failure here exactly as the Swift `try` propagated one.
pub(crate) fn make_test_query_scene(scenario: i32, sensors: bool) -> (World, Pos) {
    let mut world = World::default();
    let origin = if scenario == 0 {
        Vec3::ZERO
    } else {
        Vec3::new(1000000.0, -1000000.0, 2000000.0)
    };
    let mesh = create_grid_mesh(4, 5, 0.5, 3, true).expect("mesh creation failed");
    let height = make_test_height_field(4).expect("field creation failed");
    let compound = make_test_compound(4).expect("compound creation failed");
    for j in 0..6i32 {
        let mut body_def = BodyDef::default();
        body_def.type_ = if sensors {
            BodyType::StaticBody
        } else if j == 0 {
            BodyType::DynamicBody
        } else if j == 1 {
            BodyType::KinematicBody
        } else {
            BodyType::StaticBody
        };
        body_def.position = Pos::from_vec3(
            origin + Vec3::new((j % 3) as f32 * 0.75 - 1.0, 0.0, 0.0),
        );
        body_def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Y, j as f32 * 0.15);
        let body = world.create_body(&body_def);
        let mut shape_def = ShapeDef::default();
        shape_def.filter.category_bits = 1u64 << (j % 3);
        shape_def.is_sensor = sensors;
        shape_def.enable_sensor_events = true;
        shape_def.materials = vec![SurfaceMaterial::default_material(); 3];
        match j {
            0 => {
                let _ = world.create_sphere_shape(
                    body,
                    &shape_def,
                    Sphere::new(Vec3::new(0.0, 0.25, 0.0), 0.6),
                );
            }
            1 => {
                let _ = world.create_capsule_shape(
                    body,
                    &shape_def,
                    Capsule::new(
                        Vec3::new(0.0, -0.5, 0.0),
                        Vec3::new(0.0, 0.5, 0.0),
                        0.3,
                    ),
                );
            }
            2 => {
                let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.25, 0.75));
            }
            3 => {
                // Swift defaulted `scale` to `Vec3.one`; pass it explicitly.
                // Swift's `TriangleMesh` is a class shared by reference; the Rust value is `Clone`.
                let _ = world.create_mesh_shape(body, &shape_def, mesh.clone(), Vec3::ONE);
            }
            4 => {
                // Swift's `HeightField` is a class shared by reference; the Rust value is `Clone`.
                let _ = world.create_height_field_shape(body, &shape_def, height.clone());
            }
            _ => {
                let _ = world.create_compound_shape(body, &shape_def, &compound);
            }
        }
    }
    (world, Pos::from_vec3(origin))
}
