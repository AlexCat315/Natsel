//! Ported from Tests/NatselTests/Collision/Compound/ChildTraversalTests.swift.
//!
//! Compound shape casts, ray casts, overlaps, ordered child queries, and mover planes must be
//! repeatable across all six fixture scenarios.

use crate::collision::compound::child_mover::collide_mover_and_compound;
use crate::collision::compound::child_queries::{
    overlap_compound, query_compound, ray_cast_compound, shape_cast_compound,
};
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Transform, Vec3};
use crate::tests::collision::collision_digest::CollisionDigest;
use crate::tests::collision::compound::fixture::make_test_compound;
use crate::types::geometry::Capsule;
use crate::types::queries::{RayCastInput, ShapeCastInput, ShapeProxy};

#[test]
fn queries() {
    // "Casts, overlaps, and ordered queries are repeatable"
    // Swift's `arguments: 0..<6` expands to one case per scenario.
    for scenario in 0..6 {
        fn snapshot(scenario: i32) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let field = make_test_compound(scenario).unwrap();
            let mut rays = CollisionDigest::new();
            let mut casts = CollisionDigest::new();
            let mut overlaps = CollisionDigest::new();
            let mut queries = CollisionDigest::new();
            let mut movers = CollisionDigest::new();
            let transform = Transform::new(
                Vec3::new(0.125, -0.25, 0.5),
                make_quat_from_axis_angle(Vec3::AXIS_Y, 0.35),
            );
            for i in 0..120i32 {
                let x = ((i * 7) % 19) as f32 * 0.375 - 1.0;
                let z = ((i * 11) % 17) as f32 * 0.25 - 0.75;
                let y: f32 = if i < 60 { 2.0 } else { -2.0 };
                let mut points = [Vec3::new(x, y, z), Vec3::new(x + 0.1, y + 0.5, z - 0.15)];
                let mut input = ShapeCastInput::default();
                input.translation = Vec3::new(
                    (i % 4 - 2) as f32 * 1.25,
                    if i < 60 { -4.0 } else { 4.0 },
                    (i % 5 - 2) as f32 * 0.75,
                );
                input.max_fraction = if i % 3 == 0 { 0.35 } else { 1.0 };
                input.can_encroach = i % 2 == 0;
                input.proxy = ShapeProxy::new(Some(points.as_ptr()), 2, (i % 3) as f32 * 0.15);
                casts.add_cast_output(&shape_cast_compound(&field, &input));
                let ray = RayCastInput::new(points[0], input.translation, input.max_fraction);
                rays.add_cast_output(&ray_cast_compound(&field, ray));

                points[0].y = (i % 7) as f32 * 0.3 - 1.0;
                points[1].y = points[0].y + 0.5;
                let proxy = ShapeProxy::new(Some(points.as_ptr()), 2, (i % 3) as f32 * 0.15);
                overlaps.add_u32(if overlap_compound(&field, transform, proxy) {
                    1
                } else {
                    0
                });

                let center = Vec3::new(x, (i % 3) as f32 * 0.3 - 0.3, z);
                let extent = Vec3::new(0.45, 0.3, 0.375);
                let mut count = 0;
                let limit = if i % 4 == 0 { 1 } else { 100000 };
                query_compound(
                    &field,
                    crate::math::types::AABB::new(center - extent, center + extent),
                    |index| {
                        queries.add_u32(index as u32);
                        count += 1;
                        count < limit
                    },
                );
                queries.add_u32(count as u32);

                let capsule = Capsule::new(points[0], points[1], 0.4);
                let planes =
                    collide_mover_and_compound(&field, &capsule, (1 + i % 4) as usize);
                movers.add_u32(planes.len() as u32);
                for plane in planes.iter() {
                    movers.add_plane_result(plane);
                }
            }
            samples.push(movers.value);
            samples.push(rays.value);
            samples.push(casts.value);
            samples.push(overlaps.value);
            samples.push(queries.value);
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}
