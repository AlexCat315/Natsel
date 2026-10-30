//! Ported from Tests/NatselTests/Collision/Mover/PlaneSolverTests.swift.
//!
//! Ordered plane pushes and clipped vectors must be repeatable, and the parallel/game plane
//! configurations must cover the solver's convergence cases.

use crate::collision::mover::plane_solver::{clip_vector, solve_planes};
use crate::math::functions::{dot, length_squared, normalize};
use crate::math::types::{Plane, Vec3};
use crate::tests::collision::collision_digest::CollisionDigest;
use crate::types::queries::CollisionPlane;

#[test]
fn repeatability() {
    // "ordered plane pushes and clipped vectors are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut digest = CollisionDigest::new();
        for i in 0..1000i32 {
            let mut planes: Vec<CollisionPlane> = (0..(i % 8))
                .map(|j: i32| {
                    let mut normal = Vec3::new(
                        ((i + j * 3) % 9 - 4) as f32,
                        ((i * 3 + j * 5) % 11 - 5) as f32,
                        ((i * 7 + j * 2) % 13 - 6) as f32,
                    );
                    if length_squared(normal) == 0.0 {
                        normal.y = 1.0;
                    }
                    CollisionPlane::new(
                        Plane::new(normalize(normal), ((i + j) % 17 - 8) as f32 * 0.2),
                        if j % 3 == 0 {
                            0.15
                        } else if j % 3 == 1 {
                            f32::MAX
                        } else {
                            0.0
                        },
                        3.0,
                        (i + j) % 2 == 0,
                    )
                })
                .collect();
            let target = Vec3::new(
                ((i % 7 - 3) as f32) * 0.75,
                ((i % 11 - 5) as f32) * 0.25,
                ((i % 13 - 6) as f32) * 0.5,
            );
            let result = solve_planes(target, &mut planes);
            digest.add_vec3(result.delta);
            digest.add_u32(result.iteration_count as u32);
            for plane in planes.iter() {
                digest.add_f32(plane.push);
            }
            digest.add_vec3(clip_vector(target, &planes));
        }
        samples.push(digest.value);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn convergence() {
    // "parallel and game planes cover convergence cases"
    let mut planes = [
        CollisionPlane::new(Plane::new(Vec3::AXIS_Z, 0.5), f32::MAX, 0.0, false),
        CollisionPlane::new(Plane::new(Vec3::AXIS_Z, 1.0), f32::MAX, 0.0, false),
    ];
    let parallel = solve_planes(Vec3::ZERO, &mut planes);
    assert_eq!(parallel.iteration_count, 2);
    assert!((parallel.delta.z - 1.0).abs() < 0.0055);

    let target = Vec3::new(-2.5390625, 0.0, -73.6880798);
    planes[0].plane = Plane::new(Vec3::new(0.0, -0.23941046, 0.970918416), 0.390724182);
    planes[1].plane = Plane::new(Vec3::AXIS_Z, 1.49998093);
    for i in 0..planes.len() {
        planes[i].plane.offset -= dot(planes[i].plane.normal, target);
    }
    assert_eq!(solve_planes(Vec3::ZERO, &mut planes).iteration_count, 20);
}
