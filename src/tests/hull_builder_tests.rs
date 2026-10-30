//! Ported from Tests/NatselTests/HullBuilderTests.swift.
//!
//! Convex hull construction from point clouds and primitive generators must be byte-identical
//! across two runs (determinism contract). The content hash is computed over the same data in the
//! same order on both passes, so only internal consistency is asserted.

use crate::collision::hull::is_valid_hull;
use crate::collision::hull_builder::{
    clone_and_transform_hull, create_complex_hull, create_cone, create_cylinder, create_hull,
    create_rock,
};
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Transform, Vec3};

fn fnv(mut h: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(1_099_511_628_211);
    }
    h
}

#[inline]
fn fnv_f32(h: u64, x: f32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[inline]
fn fnv_i32(h: u64, x: i32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

/// Hash the raw bytes of a slice of `T` deterministically.
unsafe fn slice_bytes<T>(s: &[T]) -> &[u8] {
    std::slice::from_raw_parts(s.as_ptr() as *const u8, s.len() * std::mem::size_of::<T>())
}

fn content_hash(hull: &crate::collision::hull::Hull) -> u64 {
    let mut h: u64 = 14_695_981_039_346_656_037;
    h = fnv_i32(h, hull.vertex_count());
    h = fnv_i32(h, hull.half_edge_count());
    h = fnv_i32(h, hull.face_count());

    for p in hull.points() {
        h = fnv_f32(h, p.x);
        h = fnv_f32(h, p.y);
        h = fnv_f32(h, p.z);
    }

    // SAFETY: `edges`/`planes` are valid slices owned by `hull` for the call.
    unsafe {
        h = fnv(h, slice_bytes(hull.edges()));
        h = fnv(h, slice_bytes(hull.planes()));
    }

    h = fnv_f32(h, hull.volume());
    h = fnv_f32(h, hull.surface_area());
    h = fnv_f32(h, hull.inner_radius());
    let center = hull.center();
    h = fnv_f32(h, center.x);
    h = fnv_f32(h, center.y);
    h = fnv_f32(h, center.z);

    // SAFETY: `central_inertia` is a `Matrix3` value; hash its bytes deterministically.
    let inertia = hull.central_inertia();
    unsafe {
        h = fnv(h, slice_bytes(std::slice::from_ref(&inertia)));
    }
    h
}

#[test]
fn cube() {
    // "cube hull is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let c: f32 = 1.0;
        let pts: [Vec3; 8] = [
            Vec3::new(-c, -c, -c),
            Vec3::new(c, -c, c),
            Vec3::new(c, c, -c),
            Vec3::new(-c, c, c),
            Vec3::new(c, -c, -c),
            Vec3::new(-c, c, -c),
            Vec3::new(-c, -c, c),
            Vec3::new(c, c, c),
        ];

        let hull = create_hull(&pts, 8).expect("hull");
        assert_eq!(hull.vertex_count(), 8);
        assert_eq!(hull.half_edge_count(), 24);
        assert_eq!(hull.face_count(), 6);
        assert!(is_valid_hull(&hull));
        samples.push(content_hash(&hull));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn icosahedron() {
    // "icosahedron hull is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let t = (1.0 + (5.0_f32).sqrt()) / 2.0;
        let pts: [Vec3; 12] = [
            Vec3::new(-1.0, t, 0.0),
            Vec3::new(1.0, t, 0.0),
            Vec3::new(-1.0, -t, 0.0),
            Vec3::new(1.0, -t, 0.0),
            Vec3::new(0.0, -1.0, t),
            Vec3::new(0.0, 1.0, t),
            Vec3::new(0.0, -1.0, -t),
            Vec3::new(0.0, 1.0, -t),
            Vec3::new(t, 0.0, -1.0),
            Vec3::new(t, 0.0, 1.0),
            Vec3::new(-t, 0.0, -1.0),
            Vec3::new(-t, 0.0, 1.0),
        ];

        let hull = create_hull(&pts, 12).expect("hull");
        assert_eq!(hull.vertex_count(), 12);
        assert_eq!(hull.half_edge_count(), 60);
        assert_eq!(hull.face_count(), 20);
        assert!(is_valid_hull(&hull));
        samples.push(content_hash(&hull));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn cloud() {
    // "random cloud hull is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut pts: Vec<Vec3> = Vec::new();
        let mut s: u32 = 7;
        for _ in 0..40 {
            s = s.wrapping_mul(2_147_001_325).wrapping_add(715_136_305);
            let nx = ((s & 0xffff) as f32) / 65535.0 - 0.5;
            s = s.wrapping_mul(2_147_001_325).wrapping_add(715_136_305);
            let ny = ((s & 0xffff) as f32) / 65535.0 - 0.5;
            s = s.wrapping_mul(2_147_001_325).wrapping_add(715_136_305);
            let nz = ((s & 0xffff) as f32) / 65535.0 - 0.5;
            pts.push(Vec3::new(nx, ny, nz));
        }

        let hull = create_hull(&pts, 40).expect("hull");
        assert_eq!(hull.vertex_count(), 19);
        assert_eq!(hull.half_edge_count(), 102);
        assert_eq!(hull.face_count(), 34);
        assert!(is_valid_hull(&hull));
        samples.push(content_hash(&hull));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn primitives() {
    // "generated primitive hulls are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        // TODO(migration): verify `create_cylinder` argument order against the Swift source
        // (height, radius, y_offset, sides) — count assertions depend on it.
        let cylinder = create_cylinder(2.0, 1.0, 0.0, 8).expect("cylinder");
        assert_eq!(cylinder.vertex_count(), 16);
        assert_eq!(cylinder.half_edge_count(), 48);
        assert_eq!(cylinder.face_count(), 10);
        samples.push(content_hash(&cylinder));

        // TODO(migration): verify `create_cone` argument order (height, radius1, radius2, slices).
        let cone = create_cone(2.0, 1.0, 0.5, 8).expect("cone");
        assert_eq!(cone.vertex_count(), 16);
        assert_eq!(cone.half_edge_count(), 48);
        assert_eq!(cone.face_count(), 10);
        samples.push(content_hash(&cone));

        let rock = create_rock(1.0).expect("rock");
        assert_eq!(rock.vertex_count(), 10);
        assert_eq!(rock.half_edge_count(), 48);
        assert_eq!(rock.face_count(), 16);
        samples.push(content_hash(&rock));

        let complex = create_complex_hull(1.0).expect("complex");
        assert_eq!(complex.vertex_count(), 32);
        assert_eq!(complex.half_edge_count(), 178);
        assert_eq!(complex.face_count(), 59);
        samples.push(content_hash(&complex));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn clone_and_transform() {
    // "clone and transform is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let base = create_cylinder(2.0, 1.0, 0.0, 6).expect("base");
        let xf = Transform::new(
            Vec3::new(1.0, 2.0, 3.0),
            make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5),
        );

        let positive = clone_and_transform_hull(&base, xf, Vec3::new(1.0, 2.0, 0.5)).expect("positive");
        assert_eq!(positive.vertex_count(), 12);
        assert_eq!(positive.half_edge_count(), 36);
        assert_eq!(positive.face_count(), 8);
        assert!(is_valid_hull(&positive));
        samples.push(content_hash(&positive));

        let negative = clone_and_transform_hull(&base, xf, Vec3::new(1.0, 2.0, -0.5)).expect("negative");
        assert_eq!(negative.vertex_count(), 12);
        assert_eq!(negative.half_edge_count(), 36);
        assert_eq!(negative.face_count(), 8);
        assert!(is_valid_hull(&negative));
        samples.push(content_hash(&negative));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
