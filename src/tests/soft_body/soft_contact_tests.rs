//! Soft body self contact and soft-soft contact.
//!
//! Ported from `Tests/NatselTests/SoftBody/SoftContactTests.swift`.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::functions::{make_quat_from_axis_angle, min_float};
use crate::math::types::{Pos, Quat, Vec3};
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_contacts::closest_triangle_barycentric;
use crate::soft_body::soft_body_runtime::{SoftBodyDef, SoftBodyID};
use crate::soft_body::soft_body_settings::{
    SoftBodyBendType, SoftBodySettings, SoftBodyVertexAttributes,
};
use crate::tests::world_position_assertions::distance;
use crate::types::body_def::BodyDef;
use crate::types::shape_def::ShapeDef;

fn ground_world(workers: i32) -> World {
    let mut world = World::default();
    world.worker_count = workers;
    let mut def = BodyDef::default();
    def.position = Pos::new(0.0, -1.0, 0.0);
    let body = world.create_body(&def);
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(20.0, 1.0, 20.0));
    world
}

/// Smallest distance from a vertex to a triangle of the same mesh that self collision covers: not
/// its own or an edge neighbor's triangle, and at least two edge lengths away in the rest pose.
fn minimum_self_distance(world: &World, id: SoftBodyID) -> f32 {
    let record = &world.soft_bodies[(id.index1 - 1) as usize];
    let mut copy = record.clone();
    copy.build_neighbors();
    let mut best = f32::MAX;
    let rest: Vec<Vec3> = record.settings.vertices.iter().map(|v| v.position).collect();
    for (i, x) in record.positions.iter().enumerate() {
        for face in record.settings.faces.iter() {
            let ia = face.vertex.x as i32;
            let ib = face.vertex.y as i32;
            let ic = face.vertex.z as i32;
            let v = i as i32;
            if v == ia
                || v == ib
                || v == ic
                || copy.are_neighbors(v, ia)
                || copy.are_neighbors(v, ib)
                || copy.are_neighbors(v, ic)
            {
                continue;
            }
            let (bu, bv, bw, _interior) = closest_triangle_barycentric(
                rest[ia as usize],
                rest[ib as usize],
                rest[ic as usize],
                rest[i],
            );
            let rq = bu * rest[ia as usize] + bv * rest[ib as usize] + bw * rest[ic as usize];
            if distance(rest[i], rq) < 2.0 * record.average_edge_length {
                continue;
            }
            let a = record.positions[ia as usize];
            let b = record.positions[ib as usize];
            let c = record.positions[ic as usize];
            let (u, vv, w, _) = closest_triangle_barycentric(a, b, c, *x);
            best = min_float(best, distance(*x, u * a + vv * b + w * c));
        }
    }
    best
}

/// A long strip hanging above the ground: it folds onto itself as it lands.
fn drop_strip(world: &mut World, self_collision: bool) -> SoftBodyID {
    let settings = SoftBodySettings::make_cloth(
        5,
        36,
        0.05,
        SoftBodyVertexAttributes::default(),
        SoftBodyBendType::Distance,
        |_x, _z| false,
    );
    let mut def = SoftBodyDef::with_settings(
        settings,
        Pos::new(0.0, 1.0, 0.0),
        make_quat_from_axis_angle(Vec3::AXIS_X, 0.5 * b3::PI),
    );
    def.enable_self_collision = self_collision;
    def.iteration_count = 8;
    def.friction = 0.5;
    world.create_soft_body(&def)
}

#[test]
// "closest point on a triangle with barycentric coordinates"
fn barycentric() {
    let a = Vec3::new(0.0, 0.0, 0.0);
    let b = Vec3::new(1.0, 0.0, 0.0);
    let c = Vec3::new(0.0, 1.0, 0.0);

    let inside = closest_triangle_barycentric(a, b, c, Vec3::new(0.25, 0.25, 2.0));
    assert!(inside.3);
    assert!(
        (inside.0 - 0.5).abs() < 1.0e-6
            && (inside.1 - 0.25).abs() < 1.0e-6
            && (inside.2 - 0.25).abs() < 1.0e-6
    );

    let edge = closest_triangle_barycentric(a, b, c, Vec3::new(0.5, -1.0, 0.0));
    assert!(
        edge.3 == false
            && (edge.0 - 0.5).abs() < 1.0e-6
            && (edge.1 - 0.5).abs() < 1.0e-6
    );

    let corner = closest_triangle_barycentric(a, b, c, Vec3::new(2.0, 2.0, 0.0));
    assert!(corner.3 == false);
    let q = corner.0 * a + corner.1 * b + corner.2 * c;
    assert!(distance(q, Vec3::new(0.5, 0.5, 0.0)) < 1.0e-6);
}

#[test]
// "a folding strip keeps its layers apart with self collision"
fn self_collision() {
    let mut free = ground_world(1);
    let free_strip = drop_strip(&mut free, false);
    let mut guarded = ground_world(1);
    let guarded_strip = drop_strip(&mut guarded, true);
    let thickness = guarded.soft_body_collision_thickness(guarded_strip);
    assert!(thickness > 0.0);
    for _ in 0..150 {
        free.step(1.0 / 60.0, 4);
        guarded.step(1.0 / 60.0, 4);
    }
    let free_distance = minimum_self_distance(&free, free_strip);
    let guarded_distance = minimum_self_distance(&guarded, guarded_strip);
    // Without self collision the layers pass through each other.
    assert!(free_distance < 0.2 * thickness);
    assert!(
        guarded_distance > 0.6 * thickness,
        "{guarded_distance} vs {thickness}"
    );
}

#[test]
// "a cloth lands on another cloth instead of passing through it"
fn cloth_on_cloth() {
    fn run(collide: bool) -> (f32, f32) {
        let mut world = ground_world(1);
        let mut bottom = SoftBodyDef::with_settings(
            SoftBodySettings::make_cloth(
                12,
                12,
                0.1,
                SoftBodyVertexAttributes::default(),
                SoftBodyBendType::Distance,
                |_x, _z| false,
            ),
            Pos::new(0.0, 0.05, 0.0),
            Quat::IDENTITY,
        );
        bottom.collide_with_soft_bodies = collide;
        let lower = world.create_soft_body(&bottom);

        let mut top = SoftBodyDef::with_settings(
            SoftBodySettings::make_cloth(
                8,
                8,
                0.1,
                SoftBodyVertexAttributes::default(),
                SoftBodyBendType::Distance,
                |_x, _z| false,
            ),
            Pos::new(0.05, 0.6, 0.05),
            Quat::IDENTITY,
        );
        top.collide_with_soft_bodies = collide;
        let upper = world.create_soft_body(&top);

        for _ in 0..120 {
            world.step(1.0 / 60.0, 4);
        }
        let average = |id: SoftBodyID| -> f32 {
            let p = world.soft_body_vertex_positions(id);
            p.iter().fold(0.0f32, |acc, v| acc + v.y as f32) / p.len() as f32
        };
        (
            average(upper) - average(lower),
            world.soft_body_collision_thickness(upper),
        )
    }

    let (gap, thickness) = run(true);
    let (overlap, _) = run(false);
    assert!(gap > 0.7 * thickness, "{gap} vs {thickness}");
    assert!(overlap.abs() < 0.2 * thickness);
}

#[test]
// "ropes rest on each other through vertex contacts"
fn ropes() {
    let mut world = ground_world(1);

    let mut lower = SoftBodyDef::with_settings(
        SoftBodySettings::make_rope(30, 0.05, 0.0, false),
        Pos::new(0.0, 0.05, 0.7),
        make_quat_from_axis_angle(Vec3::AXIS_X, 0.5 * b3::PI),
    );
    lower.collision_thickness = 0.04;
    let a = world.create_soft_body(&lower);

    let mut upper = SoftBodyDef::with_settings(
        SoftBodySettings::make_rope(30, 0.05, 0.0, false),
        Pos::new(-0.7, 0.4, 0.0),
        make_quat_from_axis_angle(Vec3::AXIS_Z, 0.5 * b3::PI),
    );
    upper.collision_thickness = 0.04;
    let b = world.create_soft_body(&upper);

    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
    }
    // The upper rope crosses over the lower one near the origin.
    let crossing = world
        .soft_body_vertex_positions(b)
        .into_iter()
        .min_by(|p, q| p.x.abs().partial_cmp(&q.x.abs()).unwrap())
        .unwrap();
    let below = world
        .soft_body_vertex_positions(a)
        .into_iter()
        .min_by(|p, q| {
            (p.z - crossing.z)
                .abs()
                .partial_cmp(&(q.z - crossing.z).abs())
                .unwrap()
        })
        .unwrap();
    assert!(
        crossing.y - below.y > 0.03,
        "{} over {}",
        crossing.y,
        below.y
    );
}

#[test]
// "results do not depend on the worker count"
fn threads() {
    fn run(workers: i32) -> Vec<Pos> {
        let mut world = ground_world(workers);
        let mut ids: Vec<SoftBodyID> = Vec::new();
        for k in 0..4 {
            let mut def = SoftBodyDef::with_settings(
                SoftBodySettings::make_cloth(
                    10,
                    10,
                    0.1,
                    SoftBodyVertexAttributes::default(),
                    SoftBodyBendType::Distance,
                    |_x, _z| false,
                ),
                Pos::new(
                    ((k / 2) as f32 * 3.0) as f64,
                    (0.3 + 0.4 * (k % 2) as f32) as f64,
                    0.0,
                ),
                Quat::IDENTITY,
            );
            def.enable_self_collision = k == 0;
            ids.push(world.create_soft_body(&def));
        }
        for _ in 0..60 {
            world.step(1.0 / 60.0, 4);
        }
        ids.iter()
            .flat_map(|id| world.soft_body_vertex_positions(*id))
            .collect()
    }

    let serial = run(1);
    let parallel = run(6);
    assert_eq!(serial.len(), parallel.len());
    assert!(serial.iter().zip(parallel.iter()).all(|(a, b)| {
        a.x.to_bits() == b.x.to_bits()
            && a.y.to_bits() == b.y.to_bits()
            && a.z.to_bits() == b.z.to_bits()
    }));
}

#[test]
// "a falling cloth wakes a sleeping one"
fn waking() {
    let mut world = ground_world(1);
    let resting = world.create_soft_body(&SoftBodyDef::with_settings(
        SoftBodySettings::make_cloth(
            8,
            8,
            0.1,
            SoftBodyVertexAttributes::default(),
            SoftBodyBendType::Distance,
            |_x, _z| false,
        ),
        Pos::new(0.0, 0.02, 0.0),
        Quat::IDENTITY,
    ));
    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
    }
    assert!(world.soft_body_is_awake(resting) == false);

    let falling = world.create_soft_body(&SoftBodyDef::with_settings(
        SoftBodySettings::make_cloth(
            6,
            6,
            0.1,
            SoftBodyVertexAttributes::default(),
            SoftBodyBendType::Distance,
            |_x, _z| false,
        ),
        Pos::new(0.0, 1.0, 0.0),
        Quat::IDENTITY,
    ));
    let mut woke = false;
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
        woke = woke || world.soft_body_is_awake(resting);
    }
    assert!(woke);
    let top = world
        .soft_body_vertex_positions(falling)
        .iter()
        .fold(0.0f64, |acc, v| acc + v.y)
        / 36.0;
    assert!(top > 0.02);
}
