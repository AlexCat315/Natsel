//! Ported from Tests/NatselTests/Simulation/OverlapEquivalenceTests.swift.

use crate::collision::hull::{make_box_hull, make_transformed_box_hull, Hull};
use crate::math::functions::{make_quat_from_axis_angle, normalize, offset_pos, transform_point};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::World;
use crate::collision::compound::compound::Compound;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{CompoundDef, CompoundHullDef};
use crate::types::ids::BodyID;
use crate::types::queries::ShapeProxy;
use crate::types::shape_def::{QueryFilter, ShapeDef, SurfaceMaterial};

fn make_world() -> (World, BodyID) {
    let mut world = World::default();
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(10.0, 0.0, 0.0);
    let body = world.create_body(&def);
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(1.0, 1.0, 1.0));
    world.step(1.0f32 / 60.0, 1);
    (world, body)
}

fn count_hits(world: &World, origin: Pos, points: &[Vec3]) -> i32 {
    let mut hits = 0;
    let proxy = ShapeProxy::new(points.as_ptr(), points.len() as i32, 0.0);
    let _ = world.overlap_shape(origin, proxy, QueryFilter::default_filter(), |_| {
        hits += 1;
        true
    });
    hits
}

fn points_of(hull: &Hull) -> &[Vec3] {
    hull.points()
}

#[test]
fn hull_proxy_equivalence() {
    let (world, _) = make_world();

    // Overlapping target: a 10 wide query box centered on the body.
    let near_offset = Vec3::new(10.0, 0.0, 0.0);
    let baked_near = make_transformed_box_hull(5.0, 5.0, 5.0, Transform::new(near_offset, Quat::IDENTITY));
    let baked_near_hits = count_hits(&world, Pos::new(0.0, 0.0, 0.0), points_of(&baked_near));

    let local_near = make_box_hull(5.0, 5.0, 5.0);
    let local_near_hits = count_hits(
        &world,
        offset_pos(Pos::new(0.0, 0.0, 0.0), near_offset),
        points_of(&local_near),
    );

    assert_eq!(baked_near_hits, 1);
    assert_eq!(local_near_hits, baked_near_hits);

    // Clearing target: same box far from the body, both formulations agree on the miss.
    let far_offset = Vec3::new(100.0, 0.0, 0.0);
    let baked_far = make_transformed_box_hull(5.0, 5.0, 5.0, Transform::new(far_offset, Quat::IDENTITY));
    let baked_far_hits = count_hits(&world, Pos::new(0.0, 0.0, 0.0), points_of(&baked_far));

    let local_far = make_box_hull(5.0, 5.0, 5.0);
    let local_far_hits = count_hits(
        &world,
        offset_pos(Pos::new(0.0, 0.0, 0.0), far_offset),
        points_of(&local_far),
    );

    assert_eq!(baked_far_hits, 0);
    assert_eq!(local_far_hits, baked_far_hits);
}

#[test]
fn hull_proxy_rotation() {
    let (world, _) = make_world();

    let rotation = make_quat_from_axis_angle(normalize(Vec3::new(1.0, 2.0, 3.0)), 0.7);
    let offset = Vec3::new(10.0, 0.0, 0.0);

    let baked = make_transformed_box_hull(5.0, 5.0, 5.0, Transform::new(offset, rotation));
    let baked_hits = count_hits(&world, Pos::new(0.0, 0.0, 0.0), points_of(&baked));

    // The origin-relative form needs the rotation in the query frame as well, so rotate the
    // local box by the same quaternion before shifting the origin.
    let rotated = make_transformed_box_hull(5.0, 5.0, 5.0, Transform::new(Vec3::ZERO, rotation));
    let local_hits = count_hits(
        &world,
        offset_pos(Pos::new(0.0, 0.0, 0.0), offset),
        points_of(&rotated),
    );

    assert_eq!(baked_hits, 1);
    assert_eq!(local_hits, baked_hits);
}

#[test]
fn compound_rotated_body() {
    let mut definition = CompoundDef::default();
    for i in 0..2 {
        let mut item = CompoundHullDef::default();
        item.hull = Some(make_box_hull(0.5, 0.5, 0.5));
        item.transform = Transform::new(Vec3::new(i as f32 * 2.0 - 1.0, 0.0, 0.0), Quat::IDENTITY);
        item.material = SurfaceMaterial::default_material();
        definition.hulls.push(item);
    }
    let compound = Compound::new(&definition).expect("compound creation failed");

    let mut world = World::default();
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Y, 0.3);
    let body = world.create_body(&def);
    let _ = world.create_compound_shape(body, &ShapeDef::default(), &compound);

    world.step(1.0f32 / 60.0, 1);

    let body_transform = world.body_transform(body);
    let t = Transform::new(Vec3::ZERO, body_transform.q);

    // Query the world at each child's world position.
    for i in 0..2 {
        let local = Vec3::new(i as f32 * 2.0 - 1.0, 0.0, 0.0);
        let world_point = transform_point(t, local);

        let proxy = ShapeProxy::new(&world_point as *const Vec3, 1, 0.25);
        let mut hits = 0;
        let _ = world.overlap_shape(world_point, proxy, QueryFilter::default_filter(), |_| {
            hits += 1;
            true
        });
        assert_eq!(hits, 1, "child {i} at {world_point:?}");

        // The body-scoped query must agree.
        let body_hit = world.body_overlap_shape(
            body,
            world_point,
            proxy,
            &QueryFilter::default_filter(),
            body_transform,
        );
        assert!(body_hit);
    }

    // The gap between the two children stays empty.
    let in_gap = transform_point(t, Vec3::ZERO);
    let gap_proxy = ShapeProxy::new(&in_gap as *const Vec3, 1, 0.25);
    let mut gap_hits = 0;
    let _ = world.overlap_shape(in_gap, gap_proxy, QueryFilter::default_filter(), |_| {
        gap_hits += 1;
        true
    });
    assert_eq!(gap_hits, 0);
}
