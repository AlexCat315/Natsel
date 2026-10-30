//! Ported from Tests/NatselTests/Collision/Mover/WorldMoverTests.swift.
//!
//! Mixed-shape mover planes, casts, filters, and large world origins must be repeatable.

use crate::math::types::Vec3;
use crate::tests::collision::collision_digest::CollisionDigest;
use crate::tests::simulation::mixed_query_scene::make_test_query_scene;
use crate::types::geometry::Capsule;
use crate::types::ids::ShapeID;
use crate::types::shape_def::QueryFilter;

#[test]
fn repeatability() {
    // "mixed shape planes, casts, filters, and large origins are repeatable"
    // Swift's `arguments: 0..<2` expands to one case per origin scenario.
    for scenario in 0..2i32 {
        fn snapshot(scenario: i32) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let (world, origin) = make_test_query_scene(scenario, false);
            let mut planes = CollisionDigest::new();
            let mut casts = CollisionDigest::new();
            let mut filters = CollisionDigest::new();
            for i in 0..120i32 {
                let x = ((i * 7) % 19) as f32 * 0.25 - 1.0;
                let z = ((i * 11) % 17) as f32 * 0.25 - 0.75;
                let y = (i % 9) as f32 * 0.25 - 0.5;
                let mut mover = Capsule::new(
                    Vec3::new(x, y, z),
                    Vec3::new(x + 0.1, y + 0.5, z - 0.15),
                    (i % 3) as f32 * 0.2,
                );
                let mut filter = QueryFilter::default_filter();
                filter.mask_bits = if i % 3 == 0 { 1 } else { u64::MAX };
                let mut count = 0;
                let limit = if i % 4 == 0 { 1 } else { 100000 };
                world.collide_mover(origin, &mover, &filter, |id, results| {
                    planes.add_u32(id.index1 as u32);
                    planes.add_u32(results.len() as u32);
                    for plane in results.iter() {
                        planes.add_plane_result(plane);
                    }
                    count += 1;
                    count < limit
                });
                planes.add_u32(count as u32);

                mover.center1.y = 2.0;
                mover.center2.y = 2.5;
                let translation = Vec3::new(
                    (i % 4 - 2) as f32 * 1.25,
                    -4.0,
                    (i % 5 - 2) as f32 * 0.75,
                );
                // Swift passed a closure that recorded each candidate id and rejected odd ids.
                // `cast_mover` takes a `&dyn Fn`, so candidate ids are collected through a
                // `RefCell` and folded into the digest in call order afterwards.
                let recorded: std::cell::RefCell<Vec<i32>> = std::cell::RefCell::new(Vec::new());
                let callback: Option<&dyn Fn(ShapeID) -> bool> = if i % 2 == 0 {
                    Some(&|id: ShapeID| {
                        recorded.borrow_mut().push(id.index1);
                        id.index1 % 2 == 0
                    })
                } else {
                    None
                };
                let fraction = world.cast_mover(origin, &mover, translation, &filter, callback);
                for id in recorded.borrow().iter() {
                    filters.add_u32(*id as u32);
                }
                casts.add_f32(fraction);
            }
            samples.push(planes.value);
            samples.push(casts.value);
            samples.push(filters.value);
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}
