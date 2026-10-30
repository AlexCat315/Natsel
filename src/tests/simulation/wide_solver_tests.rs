//! Ported from Tests/NatselTests/Simulation/WideSolverTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::functions::{
    cross, dot, make_quat_from_axis_angle, max_float, min_float, normalize,
};
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::simulation::contact_solver::{SymMatrix3, inv_rotate_vector_wide};
use crate::simulation::wide_types::{
    FloatW, QuatW, SymMatrix3W, Vec3W, add_w, cross_w, dot_w, max_w, min_w, mul_mv_w, neg_w, sub_w,
};
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::shape_def::ShapeDef;

fn make_stack(world: &mut World) {
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -1.0, 0.0);
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(10.0, 1.0, 10.0));

    for i in 0..6 {
        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, 0.6 + i as f32 * 1.05, 0.0);
        let box_body = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    }
}

// "lane arithmetic matches scalar"
#[test]
fn lane_arithmetic() {
    let a = Vec3W::gather(
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(-1.0, 0.5, 2.0),
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(4.0, 5.0, 6.0),
    );
    let b = Vec3W::gather(
        Vec3::new(0.5, 1.0, 1.5),
        Vec3::new(2.0, -1.0, 0.25),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(-4.0, 0.0, 1.0),
    );

    let sum = add_w(a, b);
    let diff = sub_w(a, b);
    let crs = cross_w(a, b);
    let dt = dot_w(a, b);

    for lane in 0..4usize {
        let sa = a.lane(lane);
        let sb = b.lane(lane);
        assert_eq!(sum.lane(lane), sa + sb);
        assert_eq!(diff.lane(lane), sa - sb);
        assert_eq!(crs.lane(lane), cross(sa, sb));
        assert_eq!(dt[lane], dot(sa, sb));
    }
}

// "symmetric matrix product matches scalar"
#[test]
fn matrix_product() {
    let m = SymMatrix3::with_components(2.0, 0.5, -1.0, 3.0, 0.25, 4.0);
    let wide = SymMatrix3W::gather(m, m, m, m);
    let v = Vec3W::gather(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
    );

    let out = mul_mv_w(wide, v);
    for lane in 0..4usize {
        assert_eq!(out.lane(lane), m.mul_mv(v.lane(lane)));
    }
}

// "inverse rotation matches scalar"
#[test]
fn inverse_rotation() {
    let q = make_quat_from_axis_angle(normalize(Vec3::new(1.0, 2.0, 3.0)), 0.7);
    let wide_quat = QuatW::new(Vec3W::splat(q.v), FloatW::splat(q.s));
    let v = Vec3W::gather(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 2.0, 3.0),
    );

    let world = World::default();
    let out = world.inv_rotate_vec_w(wide_quat, v);
    for lane in 0..4usize {
        assert_eq!(out.lane(lane), inv_rotate_vector_wide(q, v.lane(lane)));
    }
}

// "SIMD negation and selections preserve IEEE operand bits"
#[test]
fn signed_zero_and_nan() {
    let a = FloatW::new(0.0, -0.0, f32::from_bits(0x7fc0_1234), -3.0);
    let b = FloatW::new(-0.0, 0.0, 1.0, f32::from_bits(0x7fc0_5678));
    let negative = neg_w(a);
    let maximum = max_w(a, b);
    let minimum = min_w(a, b);
    for lane in 0..4usize {
        assert_eq!(negative[lane].to_bits(), (-a[lane]).to_bits());
        assert_eq!(maximum[lane].to_bits(), max_float(a[lane], b[lane]).to_bits());
        assert_eq!(minimum[lane].to_bits(), min_float(a[lane], b[lane]).to_bits());
    }
}

// "stack settles deterministically"
#[test]
fn stack_deterministic() {
    let mut hashes: Vec<u64> = Vec::new();
    for _ in 0..2 {
        let mut world = World::default();
        world.enable_sleep = false;
        make_stack(&mut world);
        for _ in 0..90 {
            world.step(1.0 / 60.0, 4);
        }
        hashes.push(world.world_state_hash());
    }
    assert_eq!(hashes[0], hashes[1]);
}
