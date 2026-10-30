//! Ported from Tests/NatselTests/Simulation/BuoyancyTests.swift.

use std::f32::consts::PI;

use crate::collision::hull::make_box_hull;
use crate::math::functions::{make_quat_from_axis_angle, normalize, rotate_vector};
use crate::math::types::{Pos, Quat, Vec3};
use crate::math::world_aabb::WorldAABB;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::{QueryFilter, ShapeDef};

fn dynamic_body(world: &mut World, position: Vec3) -> crate::types::ids::BodyID {
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(
        f64::from(position.x),
        f64::from(position.y),
        f64::from(position.z),
    );
    world.create_body(&def)
}

#[test]
fn sphere() {
    let mut world = World::default();
    let body = dynamic_body(&mut world, Vec3::new(0.0, 0.0, 0.0));
    let _ = world.create_sphere_shape(body, &ShapeDef::default(), Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0));

    let half = world.body_submerged_volume(body, Pos::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    assert!((half.total_volume - (4.0 / 3.0) * PI).abs() < 1.0e-4);
    assert!((half.submerged_volume - (2.0 / 3.0) * PI).abs() < 1.0e-4);
    assert!((half.center_of_buoyancy.y + 3.0 / 8.0).abs() < 1.0e-4);

    let none = world.body_submerged_volume(body, Pos::new(0.0, -2.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    assert_eq!(none.submerged_volume, 0.0);

    let all = world.body_submerged_volume(body, Pos::new(0.0, 2.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    assert!((all.submerged_volume - all.total_volume).abs() < 1.0e-4);
}

#[test]
fn hull() {
    let mut world = World::default();
    let body = dynamic_body(&mut world, Vec3::new(0.0, 0.0, 0.0));
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(1.0, 1.0, 1.0));

    let quarter = world.body_submerged_volume(body, Pos::new(0.0, -0.5, 0.0), Vec3::new(0.0, 1.0, 0.0));
    assert!((quarter.total_volume - 8.0).abs() < 1.0e-4);
    assert!((quarter.submerged_volume - 2.0).abs() < 1.0e-4);
    assert!((quarter.center_of_buoyancy.y + 0.75).abs() < 1.0e-4);

    let tilted = dynamic_body(&mut world, Vec3::new(5.0, 0.0, 0.0));
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(5.0, 0.0, 0.0);
    def.rotation = make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.25 * PI);
    let tilted = world.create_body(&def);
    let _ = world.create_hull_shape(tilted, &ShapeDef::default(), &make_box_hull(1.0, 1.0, 1.0));
    let half = world.body_submerged_volume(tilted, Pos::new(5.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    assert!((half.submerged_volume - 4.0).abs() < 1.0e-3);
    assert!(half.center_of_buoyancy.y < -0.3);
    assert!((half.center_of_buoyancy.x - 5.0).abs() < 2.0e-3);

    let corner = dynamic_body(&mut world, Vec3::new(10.0, 0.0, 0.0));
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(10.0, 0.0, 0.0);
    def.rotation = make_quat_from_axis_angle(normalize(Vec3::new(1.0, 0.0, 1.0)), 0.6);
    let corner = world.create_body(&def);
    let _ = world.create_hull_shape(corner, &ShapeDef::default(), &make_box_hull(1.0, 1.0, 1.0));

    let surface = Vec3::new(10.0, -0.9, 0.0);
    let exact = world.body_submerged_volume(
        corner,
        Pos::new(f64::from(surface.x), f64::from(surface.y), f64::from(surface.z)),
        Vec3::new(0.0, 1.0, 0.0),
    );
    let n: i32 = 60;
    let q = world.body_rotation(corner);
    let mut count = 0;
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let local = Vec3::new(
                    -1.0 + (i as f32 + 0.5) * 2.0 / n as f32,
                    -1.0 + (j as f32 + 0.5) * 2.0 / n as f32,
                    -1.0 + (k as f32 + 0.5) * 2.0 / n as f32,
                );
                if rotate_vector(q, local).y < -0.9 {
                    count += 1;
                }
            }
        }
    }
    let sampled = 8.0 * count as f32 / (n * n * n) as f32;
    assert!((exact.submerged_volume - sampled).abs() < 0.01 + 0.02 * sampled);
}

#[test]
fn capsule() {
    let mut world = World::default();
    let horizontal = dynamic_body(&mut world, Vec3::new(0.0, 0.0, 0.0));
    let _ = world.create_capsule_shape(
        horizontal,
        &ShapeDef::default(),
        crate::types::geometry::Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.5),
    );
    let total = PI * 0.25 * 2.0 + (4.0 / 3.0) * PI * 0.125;
    let half = world.body_submerged_volume(horizontal, Pos::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    assert!((half.total_volume - total).abs() < 1.0e-4);
    assert!((half.submerged_volume - 0.5 * total).abs() < 2.0e-3 * total);

    let vertical = dynamic_body(&mut world, Vec3::new(5.0, 0.0, 0.0));
    let _ = world.create_capsule_shape(
        vertical,
        &ShapeDef::default(),
        crate::types::geometry::Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5),
    );
    // Surface at y = -1: only the lower hemisphere is submerged.
    let hemisphere = world.body_submerged_volume(vertical, Pos::new(5.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    assert!((hemisphere.submerged_volume - (2.0 / 3.0) * PI * 0.125).abs() < 3.0e-3 * total);
    assert!((hemisphere.center_of_buoyancy.y - (-1.0 - 3.0 / 8.0 * 0.5)).abs() < 0.01);
}

fn run_floating(buoyancy: f32) {
    let mut world = World::default();
    let box_body = dynamic_body(&mut world, Vec3::new(0.0, 1.0, 0.0));
    let _ = world.create_hull_shape(box_body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    let mut draft: f32 = 0.0;
    for i in 0..900 {
        world.apply_buoyancy_impulse(
            box_body,
            Pos::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            buoyancy,
            1.0,
            1.0,
            Vec3::ZERO,
            1.0f32 / 60.0,
        );
        world.step(1.0f32 / 60.0, 4);
        if i >= 600 {
            let volume = world.body_submerged_volume(box_body, Pos::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
            draft += (volume.submerged_volume / volume.total_volume) / 300.0;
        }
    }
    assert!((draft - 1.0 / buoyancy).abs() < 0.02);
}

#[test]
fn floating_2() {
    run_floating(2.0);
}

#[test]
fn floating_4() {
    run_floating(4.0);
}

#[test]
fn sink_and_current() {
    let mut world = World::default();
    let heavy = dynamic_body(&mut world, Vec3::new(0.0, 0.0, 0.0));
    let _ = world.create_sphere_shape(heavy, &ShapeDef::default(), Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.3));
    let light = dynamic_body(&mut world, Vec3::new(5.0, 0.0, 0.0));
    let _ = world.create_sphere_shape(light, &ShapeDef::default(), Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.3));

    let water = WorldAABB::new(
        Pos::new(-50.0, -50.0, -50.0),
        Pos::new(50.0, 0.1, 50.0),
    );
    for _ in 0..300 {
        let _ = world.apply_buoyancy(
            water,
            Vec3::new(0.0, 1.0, 0.0),
            0.5,
            0.5,
            0.01,
            Vec3::ZERO,
            1.0f32 / 60.0,
            QueryFilter::default_filter(),
        );
        world.apply_buoyancy_impulse(
            light,
            Pos::new(0.0, 0.1, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            3.0,
            1.0,
            0.01,
            Vec3::new(1.0, 0.0, 0.0),
            1.0f32 / 60.0,
        );
        world.step(1.0f32 / 60.0, 4);
    }
    assert!(world.body_position(heavy).y < -5.0);
    assert!(world.body_position(light).x > 6.0);
    assert!(world.body_position(light).y > -0.3);
}

#[test]
fn righting() {
    let mut world = World::default();
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    def.rotation = make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 1.2);
    let plank = world.create_body(&def);
    let _ = world.create_hull_shape(plank, &ShapeDef::default(), &make_box_hull(1.0, 0.1, 0.5));

    for _ in 0..900 {
        world.apply_buoyancy_impulse(
            plank,
            Pos::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            2.0,
            1.0,
            1.0,
            Vec3::ZERO,
            1.0f32 / 60.0,
        );
        world.step(1.0f32 / 60.0, 4);
    }
    let up = rotate_vector(world.body_rotation(plank), Vec3::new(0.0, 1.0, 0.0));
    assert!(up.y.abs() > 0.95);
}
