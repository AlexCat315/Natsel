//! Soft body edge collision.
//!
//! Ported from `Tests/NatselTests/SoftBody/SoftEdgeCollisionTests.swift`.

use crate::collision::hull::make_box_hull;
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Pos, Quat, Vec3};
use crate::types::body_def::BodyType;
use crate::types::geometry::Capsule;
use crate::types::ids::BodyID;
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_runtime::{SoftBodyDef, SoftBodyID};
use crate::soft_body::soft_body_settings::{
    SoftBodyBendType, SoftBodyFace, SoftBodySettings, SoftBodyVertex, SoftBodyVertexAttributes,
};
use crate::types::body_def::BodyDef;
use crate::types::shape_def::ShapeDef;
use crate::tests::SWIFT_FLOAT_PI;

/// A world with a ground far below, so anything that falls through is obvious.
fn low_ground_world(workers: i32) -> World {
    let mut world = World::default();
    world.worker_count = workers;
    let mut def = BodyDef::default();
    def.position = Pos::new(0.0, -3.0, 0.0);
    let body = world.create_body(&def);
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(20.0, 1.0, 20.0));
    world
}

/// Height of the highest vertex: whether the body was caught at all.
fn max_y(world: &World, id: SoftBodyID) -> f32 {
    world
        .soft_body_vertex_positions(id)
        .iter()
        .map(|p| p.y as f32)
        .fold(f32::MIN, f32::max)
}

fn run(world: &mut World, steps: i32) {
    for _ in 0..steps {
        world.step(1.0 / 60.0, 4);
    }
}

/// A cloth with separate column and row spacing, in the xz plane centered on the origin.
fn make_strip<F>(
    columns: i32,
    column_spacing: f32,
    rows: i32,
    row_spacing: f32,
    pinned: F,
) -> SoftBodySettings
where
    F: Fn(i32, i32) -> bool,
{
    let mut settings = SoftBodySettings::new();
    for z in 0..rows {
        for x in 0..columns {
            let position = Vec3::new(
                ((x as f32) - 0.5 * ((columns - 1) as f32)) * column_spacing,
                0.0,
                ((z as f32) - 0.5 * ((rows - 1) as f32)) * row_spacing,
            );
            settings.vertices.push(SoftBodyVertex::new(
                position,
                Vec3::ZERO,
                if pinned(x, z) { 0.0 } else { 1.0 },
            ));
        }
    }
    let index = |x: i32, z: i32| -> u32 { (x + z * columns) as u32 };
    for z in 0..(rows - 1) {
        for x in 0..(columns - 1) {
            settings.faces.push(SoftBodyFace::new(
                index(x, z),
                index(x, z + 1),
                index(x + 1, z + 1),
            ));
            settings.faces.push(SoftBodyFace::new(
                index(x, z),
                index(x + 1, z + 1),
                index(x + 1, z),
            ));
        }
    }
    settings.create_constraints(
        &[SoftBodyVertexAttributes::default()],
        SoftBodyBendType::Distance,
        0.13962634,
    );
    settings.optimize();
    settings
}

#[test]
// "a narrow strip hangs on a crossing strip only through edge contacts"
fn crossing_strips() {
    fn drop(edges: bool) -> f32 {
        let mut world = low_ground_world(1);
        // Rows 0.1 apart so no lower vertex sits at z = 0; the upper strip's vertices sit at
        // x = +-0.15, beside the lower strip.
        let mut lower = SoftBodyDef::with_settings(
            make_strip(2, 0.06, 20, 0.1, |_x, z| z == 0 || z == 19),
            Pos::new(0.0, 0.5, 0.0),
            Quat::IDENTITY,
        );
        lower.collision_thickness = 0.02;
        lower.enable_edge_collision = edges;
        let _ = world.create_soft_body(&lower);

        let mut upper = SoftBodyDef::with_settings(
            make_strip(2, 0.06, 8, 0.3, |_x, _z| false),
            Pos::new(0.0, 1.2, 0.0),
            make_quat_from_axis_angle(Vec3::AXIS_Y, 0.5 * SWIFT_FLOAT_PI),
        );
        upper.collision_thickness = 0.02;
        upper.enable_edge_collision = edges;
        let id = world.create_soft_body(&upper);

        run(&mut world, 120);
        max_y(&world, id)
    }

    let held = drop(true);
    let fell = drop(false);
    // Hanging over the lower strip at y = 0.5, less its sag and the thickness.
    assert!(held > 0.35, "{held}");
    assert!(fell < -1.5, "{fell}");
}

#[test]
// "crossing ropes catch each other between their vertices"
fn crossing_ropes() {
    fn drop(edges: bool) -> f32 {
        let mut world = low_ground_world(1);
        let mut lower = SoftBodyDef::with_settings(
            SoftBodySettings::make_rope(10, 0.2, 0.0, false),
            Pos::new(-0.9, 0.5, 0.0),
            make_quat_from_axis_angle(Vec3::AXIS_Z, 0.5 * SWIFT_FLOAT_PI),
        );
        lower.collision_thickness = 0.03;
        lower.enable_edge_collision = edges;
        lower.gravity_scale = 0.0;
        let a = world.create_soft_body(&lower);
        for v in 0..10 {
            world.set_soft_body_vertex_inverse_mass(a, v, 0.0);
        }

        let mut upper = SoftBodyDef::with_settings(
            SoftBodySettings::make_rope(10, 0.2, 0.0, false),
            Pos::new(0.0, 1.5, 0.9),
            make_quat_from_axis_angle(Vec3::AXIS_X, 0.5 * SWIFT_FLOAT_PI),
        );
        upper.collision_thickness = 0.03;
        upper.enable_edge_collision = edges;
        let b = world.create_soft_body(&upper);

        run(&mut world, 90);
        // The middle of the falling rope.
        let p = world.soft_body_vertex_positions(b);
        p.iter()
            .min_by(|p, q| {
                p.z.abs()
                    .partial_cmp(&q.z.abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .unwrap()
            .y as f32
    }

    let held = drop(true);
    let fell = drop(false);
    assert!(held > 0.4 && held < 0.6, "{held}");
    assert!(fell < -1.5, "{fell}");
}

#[test]
// "a coarse cloth drapes over a thin rod between its vertices"
fn thin_rod() {
    fn drop(edges: bool, dynamic_rod: bool) -> (f32, World, BodyID) {
        let mut world = low_ground_world(1);
        let mut rod_def = BodyDef::default();
        rod_def.position = Pos::new(0.0, 1.0, 0.0);
        if dynamic_rod {
            rod_def.type_ = BodyType::DynamicBody;
            rod_def.gravity_scale = 0.0;
        }
        let rod = world.create_body(&rod_def);
        let _ = world.create_capsule_shape(
            rod,
            &ShapeDef::default(),
            Capsule::new(
                Vec3::new(0.0, 0.0, -2.0),
                Vec3::new(0.0, 0.0, 2.0),
                0.01,
            ),
        );
        // Vertices at x = +-0.125, +-0.375: the rod at x = 0 passes between them.
        let mut cloth = SoftBodyDef::with_settings(
            SoftBodySettings::make_cloth(
                8,
                8,
                0.25,
                SoftBodyVertexAttributes::default(),
                SoftBodyBendType::Distance,
                |_x, _z| false,
            ),
            Pos::new(0.0, 2.5, 0.0),
            Quat::IDENTITY,
        );
        cloth.enable_edge_collision = edges;
        cloth.friction = 0.6;
        let id = world.create_soft_body(&cloth);
        // The rod is reached after about 33 steps; a balanced cloth later slides off a thin rod.
        run(&mut world, if dynamic_rod { 45 } else { 50 });
        (max_y(&world, id), world, rod)
    }

    let held = drop(true, false).0;
    let fell = drop(false, false).0;
    assert!(held > 0.9, "{held}");
    assert!(fell < 0.5, "{fell}");

    // A free light rod is carried down by the heavy cloth, at about the speed momentum allows:
    // 64 kg of cloth at 5.5 m/s against a rod of 1.3 kg.
    let (_, world, rod) = drop(true, true);
    let v = world.body_linear_velocity(rod).y;
    assert!(v < -4.0 && v > -6.5, "{v}");
}

#[test]
// "edge contacts keep the result independent of the worker count"
fn threads() {
    fn simulate(workers: i32) -> Vec<Pos> {
        let mut world = low_ground_world(workers);
        let mut rod_def = BodyDef::default();
        rod_def.position = Pos::new(0.0, 0.5, 0.0);
        let rod = world.create_body(&rod_def);
        let _ = world.create_capsule_shape(
            rod,
            &ShapeDef::default(),
            Capsule::new(
                Vec3::new(-3.0, 0.0, 0.0),
                Vec3::new(3.0, 0.0, 0.0),
                0.02,
            ),
        );

        let mut ids: Vec<SoftBodyID> = Vec::new();
        for k in 0..4 {
            let mut def = SoftBodyDef::with_settings(
                make_strip(3, 0.08, 12, 0.15, |_x, _z| false),
                Pos::new(
                    (0.4 * (k as f64) - 0.6) as f64,
                    (1.0 + 0.3 * (k as f64)) as f64,
                    0.0,
                ),
                make_quat_from_axis_angle(Vec3::AXIS_Y, 0.7 * (k as f32)),
            );
            def.enable_self_collision = true;
            ids.push(world.create_soft_body(&def));
        }
        run(&mut world, 60);
        ids.iter()
            .flat_map(|id| world.soft_body_vertex_positions(*id))
            .collect()
    }

    let serial = simulate(1);
    let parallel = simulate(6);
    assert!(serial.iter().zip(parallel.iter()).all(|(a, b)| {
        a.x.to_bits() == b.x.to_bits()
            && a.y.to_bits() == b.y.to_bits()
            && a.z.to_bits() == b.z.to_bits()
    }));
}

#[test]
// "one large self-colliding cloth is bit identical with chunked contact detection"
fn single_cloth_workers() {
    fn simulate(workers: i32) -> Vec<Pos> {
        let mut world = low_ground_world(workers);
        let mut def = SoftBodyDef::with_settings(
            make_strip(24, 0.05, 24, 0.05, |_x, row| row == 0),
            Pos::new(0.0, 1.5, 0.0),
            Quat::IDENTITY,
        );
        def.enable_self_collision = true;
        def.enable_sleep = false;
        def.vertex_radius = 0.02;
        let id = world.create_soft_body(&def);
        run(&mut world, 60);
        world.soft_body_vertex_positions(id)
    }

    let serial = simulate(1);
    for workers in [2, 4, 8] {
        let parallel = simulate(workers);
        assert!(serial.iter().zip(parallel.iter()).all(|(a, b)| {
            a.x.to_bits() == b.x.to_bits()
                && a.y.to_bits() == b.y.to_bits()
                && a.z.to_bits() == b.z.to_bits()
        }));
    }
}
