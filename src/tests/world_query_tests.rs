//! Ported from Tests/NatselTests/WorldQueryTests.swift.

use crate::collision::dynamic_tree::TreeStats;
use crate::collision::hull::make_box_hull;
use crate::math::types::{AABB, Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::queries::{RayResult, ShapeCastInput, ShapeProxy};
use crate::types::shape_def::{QueryFilter, ShapeDef};

fn fnv(mut h: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(1_099_511_628_211);
    }
    h
}

fn fnv_pos(h: u64, p: Pos) -> u64 {
    let mut h = h;
    for c in [p.x, p.y, p.z] {
        h = fnv(h, &c.to_ne_bytes());
    }
    h
}

fn fnv_vec(h: u64, v: Vec3) -> u64 {
    let mut h = h;
    for c in [v.x, v.y, v.z] {
        h = fnv(h, &c.to_ne_bytes());
    }
    h
}

#[test]
fn queries() {
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();
        world.enable_sleep = false;

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        let wall = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        let _ = world.create_hull_shape(wall, &shape_def, &make_box_hull(1.0, 1.0, 1.0));

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(3.0, 0.0, 0.0);
        let sphere = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_sphere_shape(sphere, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5));

        let mut h: u64 = 14_695_981_039_346_656_037;
        let filter = QueryFilter::default_filter();

        // overlap AABB around the origin
        let mut overlap_count = 0i32;
        let ts: TreeStats = world.overlap_aabb(
            AABB::new(Vec3::new(-0.2, -0.2, -0.2), Vec3::new(0.2, 0.2, 0.2)),
            filter,
            |_id| {
                overlap_count += 1;
                true
            },
        );
        assert_eq!(overlap_count, 1);
        assert_eq!(ts.node_visits, 2);
        assert_eq!(ts.leaf_visits, 1);
        h = fnv(h, &overlap_count.to_ne_bytes());
        h = fnv(h, &ts.node_visits.to_ne_bytes());
        h = fnv(h, &ts.leaf_visits.to_ne_bytes());

        // overlap shape at the origin with a sphere proxy of radius 0.1
        let center = Vec3::new(0.0, 0.0, 0.0);
        let proxy = ShapeProxy::new(Some(&center as *const Vec3), 1, 0.1);
        let mut shape_count = 0i32;
        let overlap_stats: TreeStats = world.overlap_shape(Pos::ZERO, proxy, filter, |_id| {
            shape_count += 1;
            true
        });
        assert_eq!(shape_count, 1);
        assert_eq!(overlap_stats.node_visits, 2);
        assert_eq!(overlap_stats.leaf_visits, 1);
        h = fnv(h, &shape_count.to_ne_bytes());
        h = fnv(h, &overlap_stats.node_visits.to_ne_bytes());
        h = fnv(h, &overlap_stats.leaf_visits.to_ne_bytes());

        // ray closest
        let ray: RayResult = world.cast_ray_closest(
            Pos::new(5.0, 0.0, 0.0),
            Vec3::new(-10.0, 0.0, 0.0),
            filter,
        );
        assert!(ray.hit);
        samples.push(ray.fraction.to_bits() as u64);
        assert!(ray.point == Pos::new(3.5, 0.0, 0.0));
        assert!(ray.normal == Vec3::new(1.0, 0.0, 0.0));
        let hit = if ray.hit { 1i32 } else { 0i32 };
        h = fnv(h, &hit.to_ne_bytes());
        h = fnv(h, &ray.fraction.to_ne_bytes());
        h = fnv_pos(h, ray.point);
        h = fnv_vec(h, ray.normal);

        // shape cast
        let mut cast_fraction: f32 = 0.0;
        let mut cast_point = Pos::ZERO;
        let mut cast_normal = Vec3::ZERO;
        let mut cast_hit = false;
        let mut input = ShapeCastInput::default();
        input.proxy = proxy;
        input.translation = Vec3::new(-10.0, 0.0, 0.0);
        input.max_fraction = 1.0;
        let cast_stats: TreeStats = world.shape_cast(
            Pos::new(5.0, 0.0, 0.0),
            input,
            filter,
            |_id, p, n, f, _mat, _tri, _child| {
                if (0.0..=1.0).contains(&f) {
                    if !cast_hit || f < cast_fraction {
                        cast_hit = true;
                        cast_fraction = f;
                        cast_point = p;
                        cast_normal = n;
                    }
                }
                f
            },
        );
        assert!(cast_hit);
        samples.push(cast_fraction.to_bits() as u64);
        assert!(cast_point == Pos::new(3.5, 0.0, 0.0));
        assert!(cast_normal == Vec3::new(1.0, 0.0, 0.0));
        let cast_hit_int = if cast_hit { 1i32 } else { 0i32 };
        h = fnv(h, &cast_hit_int.to_ne_bytes());
        h = fnv(h, &cast_fraction.to_ne_bytes());
        h = fnv_pos(h, cast_point);
        h = fnv_vec(h, cast_normal);
        h = fnv(h, &cast_stats.node_visits.to_ne_bytes());
        h = fnv(h, &cast_stats.leaf_visits.to_ne_bytes());

        samples.push(h);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
