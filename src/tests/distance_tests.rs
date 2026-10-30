//! Ported from Tests/NatselTests/DistanceTests.swift.
//!
//! GJK distance, shape-cast, and time-of-impact must be byte-identical across two runs.

use crate::collision::distance::{shape_cast, shape_distance};
use crate::collision::time_of_impact::time_of_impact;
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Quat, Transform, Vec3};
use crate::types::queries::{
    DistanceInput, ShapeCastPairInput, ShapeProxy, SimplexCache, Sweep, TOIInput, TOIOutput,
    TOIState,
};

fn make_box(hx: f32, hy: f32, hz: f32, center: Vec3) -> [Vec3; 8] {
    let mut verts = [Vec3::ZERO; 8];
    for i in 0..8 {
        let fx = if (i & 1) != 0 { -hx } else { hx };
        let fy = if (i & 2) != 0 { -hy } else { hy };
        let fz = if (i & 4) != 0 { -hz } else { hz };
        verts[i] = Vec3::new(center.x + fx, center.y + fy, center.z + fz);
    }
    verts
}

#[test]
fn box_box_distance() {
    // "box-box GJK distance is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let a = make_box(1.0, 1.0, 1.0, Vec3::new(0.0, 0.0, 0.0));
        let b = make_box(1.0, 1.0, 1.0, Vec3::new(0.0, 0.0, 0.0));
        let mut cache = SimplexCache::default();
        let mut input = DistanceInput::default();
        input.proxy_a = ShapeProxy::new(Some(a.as_ptr()), 8, 0.0);
        input.proxy_b = ShapeProxy::new(Some(b.as_ptr()), 8, 0.0);
        input.transform = Transform::new(Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY);
        input.use_radii = false;

        let output = shape_distance(&input, &mut cache, None);
        samples.push(output.distance.to_bits() as u64);
        samples.push(output.normal.x.to_bits() as u64);
        assert_eq!(output.normal.y.to_bits(), 0);
        assert_eq!(output.normal.z.to_bits(), 0);
        assert_eq!(output.point_a, Vec3::new(1.0, 1.0, 1.0));
        assert_eq!(output.point_b, Vec3::new(2.0, 1.0, 1.0));
        assert_eq!(output.iterations, 1);
        assert_eq!(cache.count, 1);
        assert_eq!(cache.index_a[0], 0);
        assert_eq!(cache.index_b[0], 1);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn point_radii() {
    // "point-point distance with radii is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let a: [Vec3; 1] = [Vec3::new(0.0, 0.0, 0.0)];
        let b: [Vec3; 1] = [Vec3::new(0.0, 0.0, 0.0)];
        let mut cache = SimplexCache::default();
        let mut input = DistanceInput::default();
        input.proxy_a = ShapeProxy::new(Some(a.as_ptr()), 1, 1.0);
        input.proxy_b = ShapeProxy::new(Some(b.as_ptr()), 1, 1.0);
        input.transform = Transform::new(Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY);
        input.use_radii = true;

        let output = shape_distance(&input, &mut cache, None);
        samples.push(output.distance.to_bits() as u64);
        samples.push(output.normal.x.to_bits() as u64);
        samples.push(output.point_a.x.to_bits() as u64);
        samples.push(output.point_b.x.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn overlap() {
    // "overlapping boxes report zero distance"
    let a = make_box(1.0, 1.0, 1.0, Vec3::new(0.0, 0.0, 0.0));
    let b = make_box(1.0, 1.0, 1.0, Vec3::new(0.0, 0.0, 0.0));
    let mut cache = SimplexCache::default();
    let mut input = DistanceInput::default();
    input.proxy_a = ShapeProxy::new(Some(a.as_ptr()), 8, 0.0);
    input.proxy_b = ShapeProxy::new(Some(b.as_ptr()), 8, 0.0);
    input.transform = Transform::new(Vec3::new(0.5, 0.0, 0.0), Quat::IDENTITY);
    input.use_radii = false;

    let output = shape_distance(&input, &mut cache, None);
    assert_eq!(output.distance, 0.0);
    assert_eq!(output.normal, Vec3::new(0.0, 0.0, 0.0));
}

#[test]
fn box_cast() {
    // "box-box shape cast is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let a = make_box(1.0, 1.0, 1.0, Vec3::new(0.0, 0.0, 0.0));
        let b = a;
        let mut input = ShapeCastPairInput::default();
        input.proxy_a = ShapeProxy::new(Some(a.as_ptr()), 8, 0.0);
        input.proxy_b = ShapeProxy::new(Some(b.as_ptr()), 8, 0.0);
        input.transform = Transform::new(Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY);
        input.translation_b = Vec3::new(-5.0, 0.0, 0.0);
        input.max_fraction = 1.0;
        input.can_encroach = false;

        let output = shape_cast(&input);
        assert!(output.hit);
        samples.push(output.fraction.to_bits() as u64);
        assert_eq!(output.normal, Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(output.point, Vec3::new(1.0, 1.0, 1.0));
        assert_eq!(output.iterations, 2);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn point_cast() {
    // "point-point shape cast is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let a: [Vec3; 1] = [Vec3::new(0.0, 0.0, 0.0)];
        let b = a;
        let mut input = ShapeCastPairInput::default();
        input.proxy_a = ShapeProxy::new(Some(a.as_ptr()), 1, 1.0);
        input.proxy_b = ShapeProxy::new(Some(b.as_ptr()), 1, 1.0);
        input.transform = Transform::new(Vec3::new(5.0, 0.0, 0.0), Quat::IDENTITY);
        input.translation_b = Vec3::new(-10.0, 0.0, 0.0);
        input.max_fraction = 1.0;
        input.can_encroach = false;

        let output = shape_cast(&input);
        assert!(output.hit);
        samples.push(output.fraction.to_bits() as u64);
        samples.push(output.normal.x.to_bits() as u64);
        samples.push(output.point.x.to_bits() as u64);
        assert_eq!(output.iterations, 2);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn receding_cast() {
    // "receding cast misses"
    let a: [Vec3; 1] = [Vec3::new(0.0, 0.0, 0.0)];
    let b: [Vec3; 1] = [Vec3::new(0.0, 0.0, 0.0)];
    let mut input = ShapeCastPairInput::default();
    input.proxy_a = ShapeProxy::new(Some(a.as_ptr()), 1, 1.0);
    input.proxy_b = ShapeProxy::new(Some(b.as_ptr()), 1, 1.0);
    input.transform = Transform::new(Vec3::new(5.0, 0.0, 0.0), Quat::IDENTITY);
    input.translation_b = Vec3::new(10.0, 0.0, 0.0);
    input.max_fraction = 1.0;
    input.can_encroach = false;

    let output = shape_cast(&input);
    assert!(!output.hit);
    assert_eq!(output.fraction, 0.0);
}

fn run_time_of_impact(sweep_b: Sweep) -> TOIOutput {
    let box_pts = make_box(1.0, 1.0, 1.0, Vec3::new(0.0, 0.0, 0.0));
    let mut input = TOIInput::default();
    input.proxy_a = ShapeProxy::new(Some(box_pts.as_ptr()), 8, 0.0);
    input.proxy_b = ShapeProxy::new(Some(box_pts.as_ptr()), 8, 0.0);
    input.sweep_a = Sweep::new(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0),
        Quat::IDENTITY,
        Quat::IDENTITY,
    );
    input.sweep_b = sweep_b;
    input.max_fraction = 1.0;
    time_of_impact(&input)
}

#[test]
fn translate() {
    // "TOI translation is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let output = run_time_of_impact(Sweep::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            Quat::IDENTITY,
            Quat::IDENTITY,
        ));
        assert_eq!(output.state, TOIState::Hit);
        samples.push(output.fraction.to_bits() as u64);
        samples.push(output.point.x.to_bits() as u64);
        samples.push(output.point.y.to_bits() as u64);
        samples.push(output.point.z.to_bits() as u64);
        assert_eq!(output.normal, Vec3::new(1.0, 0.0, 0.0));
        samples.push(output.distance.to_bits() as u64);
        assert_eq!(output.distance_iterations, 2);
        assert_eq!(output.push_back_iterations, 1);
        assert_eq!(output.root_iterations, 2);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn recede() {
    // "TOI receding is separated"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let output = run_time_of_impact(Sweep::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Quat::IDENTITY,
            Quat::IDENTITY,
        ));
        assert_eq!(output.state, TOIState::Separated);
        samples.push(output.fraction.to_bits() as u64);
        samples.push(output.point.x.to_bits() as u64);
        samples.push(output.distance.to_bits() as u64);
        assert_eq!(output.distance_iterations, 1);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn rotate() {
    // "TOI rotation is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let q = make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.785398163);
        let output = run_time_of_impact(Sweep::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(4.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
            Quat::IDENTITY,
            q,
        ));
        assert_eq!(output.state, TOIState::Hit);
        samples.push(output.fraction.to_bits() as u64);
        samples.push(output.point.x.to_bits() as u64);
        samples.push(output.point.y.to_bits() as u64);
        assert_eq!(output.normal, Vec3::new(1.0, 0.0, 0.0));
        samples.push(output.distance.to_bits() as u64);
        assert_eq!(output.distance_iterations, 2);
        assert_eq!(output.root_iterations, 4);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
