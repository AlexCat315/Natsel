//! Ported from Tests/NatselTests/Simulation/BodyQueryTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::functions::offset_pos;
use crate::math::types::{Pos, Quat, Vec3, WorldTransform};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::queries::ShapeProxy;
use crate::types::shape_def::{QueryFilter, ShapeDef};

fn make_query_world() -> (World, crate::types::ids::BodyID) {
    let mut world = World::default();
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::StaticBody;
    let body = world.create_body(&body_def);
    (world, body)
}

fn identity_at(x: f32, y: f32, z: f32) -> WorldTransform {
    WorldTransform::new(Pos::new(x as f64, y as f64, z as f64), Quat::IDENTITY)
}

fn with_proxy<R>(point: &mut Vec3, radius: f32, f: impl FnOnce(ShapeProxy) -> R) -> R {
    f(ShapeProxy::new(Some(point as *const Vec3), 1, radius))
}

fn is_normalized(v: Vec3) -> bool {
    (v.x * v.x + v.y * v.y + v.z * v.z - 1.0).abs() < 1e-4
}

#[test]
fn cast_ray_hits_sphere() {
    let (world, body) = make_query_world();
    let mut shape_def = ShapeDef::default();
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0));

    let result = world.body_cast_ray(
        body,
        Pos::new(0.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        1.0,
        identity_at(5.0, 0.0, 0.0),
    );

    assert!(result.hit);
    assert!((result.fraction - 0.4).abs() < 1e-5);
    assert!((result.normal.x + 1.0).abs() < 1e-5);
    assert!(result.normal.y.abs() < 1e-5);
    assert!(result.normal.z.abs() < 1e-5);

    let point = result.point;
    assert!((point.x - 4.0).abs() < 1e-4);
    assert!(point.y.abs() < 1e-4);
    assert!(point.z.abs() < 1e-4);
}

#[test]
fn cast_ray_miss() {
    let (world, body) = make_query_world();
    let mut shape_def = ShapeDef::default();
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0));

    let result = world.body_cast_ray(
        body,
        Pos::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 10.0, 0.0),
        &QueryFilter::default_filter(),
        1.0,
        identity_at(5.0, 0.0, 0.0),
    );
    assert!(!result.hit);
}

#[test]
fn cast_ray_closest_shape() {
    let (world, body) = make_query_world();
    let shape_def = ShapeDef::default();
    let near = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0));
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(4.0, 0.0, 0.0), 1.0));

    let result = world.body_cast_ray(
        body,
        Pos::new(-5.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        1.0,
        identity_at(0.0, 0.0, 0.0),
    );

    assert!(result.hit);
    assert_eq!(result.shape_id.index1, near.index1);
    assert_eq!(result.shape_id.generation, near.generation);
    assert!((result.fraction - 0.4).abs() < 1e-5);
}

#[test]
fn cast_ray_far_from_origin() {
    let (world, body) = make_query_world();
    let mut shape_def = ShapeDef::default();
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0));

    let origin = Pos::new(1.0e6, -2.0e6, 5.0e5);
    let body_transform = WorldTransform::new(offset_pos(origin, Vec3::new(5.0, 0.0, 0.0)), Quat::IDENTITY);
    let result = world.body_cast_ray(
        body,
        origin,
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        1.0,
        body_transform,
    );

    assert!(result.hit);
    assert!((result.fraction - 0.4).abs() < 1e-5);
    assert!((result.normal.x + 1.0).abs() < 1e-5);
}

#[test]
fn cast_shape_hits_box() {
    let (world, body) = make_query_world();
    let shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(1.0, 1.0, 1.0));

    let mut point = Vec3::new(0.0, 0.0, 0.0);
    let result = with_proxy(&mut point, 0.5, |proxy| {
        world.body_cast_shape(
            body,
            Pos::new(0.0, 0.0, 0.0),
            proxy,
            Vec3::new(10.0, 0.0, 0.0),
            &QueryFilter::default_filter(),
            1.0,
            false,
            identity_at(5.0, 0.0, 0.0),
        )
    });

    assert!(result.hit);
    assert!((result.fraction - 0.35).abs() < 1e-2);
    assert!((result.normal.x + 1.0).abs() < 1e-4);
    assert!((result.point.x - 4.0).abs() < 1e-3);
}

#[test]
fn overlap_respects_body_transform() {
    let (world, body) = make_query_world();
    let shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(1.0, 1.0, 1.0));

    let mut point = Vec3::new(0.0, 0.0, 0.0);
    let overlaps_here = with_proxy(&mut point, 0.5, |proxy| {
        world.body_overlap_shape(
            body,
            Pos::new(0.0, 0.0, 0.0),
            proxy,
            &QueryFilter::default_filter(),
            identity_at(0.0, 0.0, 0.0),
        )
    });
    let overlaps_far = with_proxy(&mut point, 0.5, |proxy| {
        world.body_overlap_shape(
            body,
            Pos::new(0.0, 0.0, 0.0),
            proxy,
            &QueryFilter::default_filter(),
            identity_at(20.0, 0.0, 0.0),
        )
    });
    assert!(overlaps_here);
    assert!(!overlaps_far);
}

#[test]
fn overlap_true_and_false() {
    let (world, body) = make_query_world();
    let shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(1.0, 1.0, 1.0));

    let mut point = Vec3::new(0.0, 0.0, 0.0);
    let touches = with_proxy(&mut point, 0.5, |proxy| {
        world.body_overlap_shape(
            body,
            Pos::new(5.0, 0.0, 0.0),
            proxy,
            &QueryFilter::default_filter(),
            identity_at(5.0, 0.0, 0.0),
        )
    });
    let misses = with_proxy(&mut point, 0.5, |proxy| {
        world.body_overlap_shape(
            body,
            Pos::new(20.0, 0.0, 0.0),
            proxy,
            &QueryFilter::default_filter(),
            identity_at(5.0, 0.0, 0.0),
        )
    });
    assert!(touches);
    assert!(!misses);
}

#[test]
fn mover_touches_box() {
    let (world, body) = make_query_world();
    let shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let mover = Capsule::new(Vec3::new(-0.3, 0.6, 0.0), Vec3::new(0.3, 0.6, 0.0), 0.2);
    let planes = world.body_collide_mover(
        body,
        Pos::new(0.0, 0.0, 0.0),
        &mover,
        &QueryFilter::default_filter(),
        identity_at(0.0, 0.0, 0.0),
        4,
    );

    assert_eq!(planes.len(), 1);
    assert!(is_normalized(planes[0].result.plane.normal));
    assert!(planes[0].result.plane.normal.y > 0.99);
    assert!((planes[0].result.plane.offset - 0.1).abs() < 1e-4);
}

#[test]
fn mover_capacity() {
    let (world, body) = make_query_world();
    let shape_def = ShapeDef::default();
    for i in 0..4 {
        let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
        let _ = i;
    }

    let mover = Capsule::new(Vec3::new(-0.3, 0.6, 0.0), Vec3::new(0.3, 0.6, 0.0), 0.2);
    let planes = world.body_collide_mover(
        body,
        Pos::new(0.0, 0.0, 0.0),
        &mover,
        &QueryFilter::default_filter(),
        identity_at(0.0, 0.0, 0.0),
        2,
    );
    assert_eq!(planes.len(), 2);
}

#[test]
fn mover_separated() {
    let (world, body) = make_query_world();
    let shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let mover = Capsule::new(Vec3::new(-0.3, 5.0, 0.0), Vec3::new(0.3, 5.0, 0.0), 0.2);
    let planes = world.body_collide_mover(
        body,
        Pos::new(0.0, 0.0, 0.0),
        &mover,
        &QueryFilter::default_filter(),
        identity_at(0.0, 0.0, 0.0),
        4,
    );
    assert!(planes.is_empty());
}
