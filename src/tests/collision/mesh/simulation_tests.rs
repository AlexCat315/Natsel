//! Ported from Tests/NatselTests/Collision/Mesh/SimulationTests.swift.
//!
//! Sphere, capsule, and hull bodies dropped onto every mesh scenario must produce identical
//! trajectories and contact frames across two runs.

use crate::collision::hull::make_box_hull;
use crate::collision::mesh::procedural::{create_grid_mesh, create_platform_mesh, create_wave_mesh};
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::tests::simulation::physics_digest::PhysicsDigest;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::shape_def::{ShapeDef, SurfaceMaterial};

#[test]
fn simulation() {
    // "sphere, capsule, and hull trajectories and all contact frames are repeatable"
    // Swift's `arguments: 0..<27` expands to one case per scenario.
    for scenario in 0..27i32 {
        fn snapshot(scenario: i32) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let mut world = World::default();
            world.enable_sleep = scenario >= 18;
            let mut body_def = BodyDef::default();
            body_def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Y, 0.2);
            let ground = world.create_body(&body_def);
            let geometry = match (scenario % 9) / 3 {
                0 => create_grid_mesh(8, 8, 0.5, 3, true),
                1 => create_wave_mesh(8, 8, 0.5, 0.25, 0.4, 0.3),
                _ => create_platform_mesh(Vec3::new(0.0, -0.25, 0.0), 0.5, 3.0, 4.0),
            };
            let mesh = geometry.expect("mesh creation failed");
            let mut shape_def = ShapeDef::default();
            shape_def.materials = (0..3)
                .map(|i: i32| {
                    let mut material = SurfaceMaterial::default_material();
                    material.friction = 0.2 + 0.2 * i as f32;
                    material.restitution = 0.05 * i as f32;
                    material.tangent_velocity =
                        Vec3::new((i - 1) as f32 * 0.05, 0.0, 0.02 * i as f32);
                    material.user_material_id = (10 + i) as u64;
                    material
                })
                .collect();
            let scale = if (scenario % 9) / 3 == 2 {
                Vec3::new(-1.25, 1.0, 0.75)
            } else {
                Vec3::ONE
            };
            let _ = world.create_mesh_shape(ground, &shape_def, mesh, scale);

            body_def = BodyDef::default();
            body_def.type_ = BodyType::DynamicBody;
            body_def.position = Pos::new(0.13, 2.0, 0.17);
            body_def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Y, 0.3);
            body_def.linear_velocity = Vec3::new(0.35, -0.1, 0.2);
            body_def.angular_velocity = Vec3::new(0.1, 0.2, 0.15);
            if (9..18).contains(&scenario) {
                body_def.linear_velocity.y = -150.0;
                body_def.is_bullet = true;
            }
            let body = world.create_body(&body_def);
            shape_def = ShapeDef::default();
            shape_def.density = 1000.0;
            shape_def.base_material.friction = 0.7;
            shape_def.base_material.restitution = 0.15;
            shape_def.base_material.rolling_resistance = 0.05;
            match scenario % 3 {
                0 => {
                    let _ =
                        world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::ZERO, 0.4));
                }
                1 => {
                    let _ = world.create_capsule_shape(
                        body,
                        &shape_def,
                        Capsule::new(
                            Vec3::new(0.0, -0.35, 0.0),
                            Vec3::new(0.0, 0.35, 0.0),
                            0.3,
                        ),
                    );
                }
                _ => {
                    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.4, 0.4, 0.4));
                }
            }

            let mut bodies = PhysicsDigest::new();
            let mut contacts = PhysicsDigest::new();
            for _ in 0..180 {
                world.step(1.0 / 60.0, 4);
                let transform = world.body_transform(body);
                bodies.add_pos(transform.p);
                bodies.add_vec3(transform.q.v);
                bodies.add_float(transform.q.s);
                bodies.add_vec3(world.body_linear_velocity(body));
                bodies.add_vec3(world.body_angular_velocity(body));
                contacts.add_contacts(&world);
                samples.push(bodies.value);
                samples.push(contacts.value);
            }
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert_eq!(first.len(), 360, "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}
