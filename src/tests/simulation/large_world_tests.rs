//! Ported from Tests/NatselTests/Simulation/LargeWorldTests.swift.

use crate::collision::convex_manifold::collide_hulls;
use crate::collision::hull::{make_box_hull, make_cube_hull};
use crate::math::functions::{inv_mul_world_transforms, offset_pos, sub_pos};
use crate::math::types::{Pos, Quat, Vec3, WorldTransform};
use crate::simulation::World;
use crate::simulation::shape_runtime::{Shape, ShapeGeometry, compute_fat_shape_aabb};
use crate::tests::world_position_assertions::abs_float;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::collision_types::{LocalManifold, LocalManifoldPoint, SATCache};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::ids::BodyID;
use crate::types::queries::{QueryFilter, ShapeProxy};
use crate::types::shape_def::{ShapeDef, ShapeType};

/// Two cubes overlapping by 0.1 along x produce a four point face manifold at the origin.
// "a two face overlap yields four points at the origin"
#[test]
fn manifold_at_origin() {
    let box_a = make_box_hull(0.5, 0.5, 0.5);
    let box_b = make_box_hull(0.5, 0.5, 0.5);

    let xf_a = WorldTransform::new(Pos::ZERO, Quat::IDENTITY);
    let xf_b = WorldTransform::new(Pos::new(0.9, 0.0, 0.0), Quat::IDENTITY);

    let mut buffer = [LocalManifoldPoint::default(); 8];
    let mut manifold = LocalManifold::default();
    let mut cache = SATCache::default();
    manifold.points = buffer.as_mut_ptr();
    collide_hulls(
        &mut manifold,
        8,
        &box_a,
        &box_b,
        inv_mul_world_transforms(xf_a, xf_b),
        &mut cache,
    );

    // Two cube faces overlap, so the clipped manifold has four points.
    assert_eq!(manifold.point_count, 4);
    for i in 0..manifold.point_count as usize {
        assert!(
            abs_float((buffer[i].separation + 0.1) as f64) < 0.01,
            "point {}",
            i
        );
    }
}

/// The tight AABB of a unit cube is exactly its half extent, with the margin folded in.
// "a unit cube keeps its exact extent in the fat AABB"
#[test]
fn aabb_at_origin() {
    let box_ = make_box_hull(0.5, 0.5, 0.5);
    let mut shape = Shape::default();
    shape.geometry = ShapeGeometry::Hull(box_);
    shape.type_ = ShapeType::HullShape;

    let aabb = compute_fat_shape_aabb(&shape, WorldTransform::new(Pos::ZERO, Quat::IDENTITY), 0.0);
    let eps = f32::EPSILON;

    assert!(abs_float((aabb.lower_bound.x + 0.5) as f64) <= eps as f64);
    assert!(abs_float((aabb.lower_bound.y + 0.5) as f64) <= eps as f64);
    assert!(abs_float((aabb.lower_bound.z + 0.5) as f64) <= eps as f64);
    assert!(abs_float((aabb.upper_bound.x - 0.5) as f64) <= eps as f64);
    assert!(abs_float((aabb.upper_bound.y - 0.5) as f64) <= eps as f64);
    assert!(abs_float((aabb.upper_bound.z - 0.5) as f64) <= eps as f64);
}

/// A shape placed far from the origin must still be found by its own AABB query.
// "a shape far from the origin stays inside its own bounds"
#[test]
fn far_shape_bounds() {
    let mut world = World::default();
    world.enable_sleep = false;

    let far_far: f32 = 1.0e5;
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(far_far as f64, -far_far as f64, far_far as f64);
    let body = world.create_body(&def);
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    world.step(1.0f32 / 60.0, 1);

    // A tiny query point at the body centre must overlap the shape.
    let mut point = Vec3::new(far_far, -far_far, far_far);
    let mut hits = 0;
    let proxy = ShapeProxy::new(Some(&point as *const Vec3), 1, 0.0);
    let _ = world.overlap_shape(Pos::ZERO, proxy, QueryFilter::default_filter(), |_id| {
        hits += 1;
        true
    });
    point = point;
    assert_eq!(hits, 1);

    // A ray straight down must hit the top face at the expected height.
    let result = world.cast_ray_closest(
        Pos::new(far_far as f64, (-far_far + 3.0) as f64, far_far as f64),
        Vec3::new(0.0, -6.0, 0.0),
        QueryFilter::default_filter(),
    );
    assert!(result.hit);
    if result.hit {
        let gap = result.point.y - (-far_far as f64 + 0.5);
        assert!(abs_float(gap) < 0.01);
    }
}

const STACK_COUNT: usize = 6;
const MAX_STEPS: usize = 400;

/// Drop a short stack onto a ground box centered at `baseX`. Returns each body's final
/// position relative to the base and the step on which the stack settles.
fn run_stack(base_x: f32) -> (Vec<Vec3>, i32) {
    let base = Pos::new(base_x as f64, 0.0, 0.0);

    let mut world = World::default();

    let mut def = BodyDef::default();
    def.position = base;
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(10.0, 1.0, 10.0));

    let mut bodies: Vec<BodyID> = Vec::new();
    for i in 0..STACK_COUNT {
        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = offset_pos(base, Vec3::new(0.0, 2.0 + 1.05 * i as f32, 0.0));
        let body = world.create_body(&def);

        let mut shape_def = ShapeDef::default();
        shape_def.density = 1.0;
        let _ = world.create_hull_shape(body, &shape_def, &make_cube_hull(0.5));
        bodies.push(body);
    }

    let mut sleep_step: i32 = -1;
    for step in 0..MAX_STEPS {
        world.step(1.0f32 / 60.0, 4);
        if !world.is_body_awake_id(bodies[STACK_COUNT - 1]) {
            sleep_step = step as i32;
            break;
        }
    }

    let positions = bodies
        .iter()
        .map(|body| sub_pos(world.body_transform(*body).p, base))
        .collect();
    (positions, sleep_step)
}

// "a six box stack settles at the origin"
#[test]
fn stack_settles() {
    let (positions, sleep_step) = run_stack(0.0);
    assert!(sleep_step >= 0);
    // The stack must not sink into the ground or scatter.
    for (i, p) in positions.iter().enumerate() {
        assert!(p.y > 0.4, "body {} y {}", i, p.y);
        assert!(abs_float(p.x as f64) < 0.5, "body {} x {}", i, p.x);
        assert!(abs_float(p.z as f64) < 0.5, "body {} z {}", i, p.z);
    }
}

/// Fire a fast bullet at a thin wall. Returns the bullet's final x. If continuous collision
/// works the bullet stops at the wall instead of tunneling past it.
fn bullet_stops(offset: f64) {
    let base = Pos::new(offset, offset, offset);
    let mut world = World::default();

    // Thin static wall at x = 5, spanning y and z.
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = offset_pos(base, Vec3::new(5.0, 0.0, 0.0));
    let wall = world.create_body(&def);
    let _ = world.create_hull_shape(wall, &ShapeDef::default(), &make_box_hull(0.05, 5.0, 5.0));

    // Small fast bullet aimed at the wall, no gravity.
    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.is_bullet = true;
    def.gravity_scale = 0.0;
    def.position = base;
    def.linear_velocity = Vec3::new(200.0, 0.0, 0.0);
    let bullet = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = world.create_sphere_shape(bullet, &shape_def, Sphere::new(Vec3::ZERO, 0.1));

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }

    let final_x = world.body_transform(bullet).p.x - base.x;
    // Wall front face is at 5 - 0.05; the radius is 0.1, so a caught bullet stays short of 5.
    assert!(final_x < 5.0);
}

// "a fast bullet does not tunnel through a thin wall"
#[test]
fn bullet_stops_0() {
    bullet_stops(0.0);
}

#[test]
fn bullet_stops_1e9() {
    bullet_stops(1.0e9);
}

#[test]
fn bullet_stops_neg1e9() {
    bullet_stops(-1.0e9);
}

#[test]
fn bullet_stops_1e12() {
    bullet_stops(1.0e12);
}

/// The origin-relative queries hit at the origin with the expected geometry.
fn queries_at_origin(offset: f64) {
    let base = Pos::new(offset, offset, offset);
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = base;
    let body = world.create_body(&def);
    let shape_def = ShapeDef::default();
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(1.0, 1.0, 1.0));

    world.step(1.0f32 / 60.0, 1);

    // Sphere proxy swept from the left into the box, hitting the left face at relative x = -1.
    let mut cast_point = Vec3::new(-5.0, 0.0, 0.0);
    let cast_rel_x = std::cell::Cell::new(0.0f32);
    let cast_hit = std::cell::Cell::new(false);
    {
        let proxy = ShapeProxy::new(Some(&cast_point as *const Vec3), 1, 0.25);
        let cast_rel_x = &cast_rel_x;
        let cast_hit = &cast_hit;
        let _ = world.cast_shape(
            base,
            proxy,
            Vec3::new(10.0, 0.0, 0.0),
            &QueryFilter::default_filter(),
            Some(&|_id, point: Pos, _normal, _fraction| {
                cast_hit.set(true);
                cast_rel_x.set((point.x - base.x) as f32);
                0.0
            }),
        );
    }
    cast_point = cast_point;
    assert!(cast_hit.get());
    assert!(abs_float((cast_rel_x.get() + 1.0) as f64) < 0.05);

    // Sphere proxy sitting at the box center.
    let mut overlap_point = Vec3::new(0.0, 0.0, 0.0);
    let overlap_hit = std::cell::Cell::new(false);
    {
        let proxy = ShapeProxy::new(Some(&overlap_point as *const Vec3), 1, 0.5);
        let overlap_hit = &overlap_hit;
        let _ = world.overlap_shape(
            base,
            proxy,
            QueryFilter::default_filter(),
            |_id| {
                overlap_hit.set(true);
                true
            },
        );
    }
    overlap_point = overlap_point;
    assert!(overlap_hit.get());

    // Capsule swept from the left into the box.
    let mover_cast = Capsule::new(
        Vec3::new(-5.0, -0.3, 0.0),
        Vec3::new(-5.0, 0.3, 0.0),
        0.25,
    );
    let mover_fraction = world.cast_mover(
        base,
        &mover_cast,
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        None,
    );
    assert!(mover_fraction < 1.0);

    // Capsule overlapping the left face reports a contact plane.
    let mover_collide = Capsule::new(
        Vec3::new(-1.1, -0.3, 0.0),
        Vec3::new(-1.1, 0.3, 0.0),
        0.3,
    );
    let plane_count = std::cell::Cell::new(0usize);
    {
        let plane_count = &plane_count;
        world.collide_mover(
            base,
            &mover_collide,
            &QueryFilter::default_filter(),
            |_id, planes| {
                plane_count.set(plane_count.get() + planes.len());
                true
            },
        );
    }
    assert!(plane_count.get() > 0);

    // World ray cast from the left into the box, hitting the left face at relative x = -1.
    let ray = world.cast_ray_closest(
        offset_pos(base, Vec3::new(-5.0, 0.0, 0.0)),
        Vec3::new(10.0, 0.0, 0.0),
        QueryFilter::default_filter(),
    );
    assert!(ray.hit);
    if ray.hit {
        assert!(abs_float((ray.point.x - base.x) + 1.0) < 0.05);
    }

    // Direct shape ray cast against the same box.
    let shape_ray = world.shape_ray_cast(shape, offset_pos(base, Vec3::new(-5.0, 0.0, 0.0)), Vec3::new(10.0, 0.0, 0.0));
    assert!(shape_ray.hit);
    if shape_ray.hit {
        assert!(abs_float((shape_ray.point.x - base.x) + 1.0) < 0.05);
    }
}

// "origin relative queries report the expected geometry"
#[test]
fn queries_at_origin_0() {
    queries_at_origin(0.0);
}

#[test]
fn queries_at_origin_1e9() {
    queries_at_origin(1.0e9);
}

#[test]
fn queries_at_origin_neg1e9() {
    queries_at_origin(-1.0e9);
}

#[test]
fn queries_at_origin_1e12() {
    queries_at_origin(1.0e12);
}
