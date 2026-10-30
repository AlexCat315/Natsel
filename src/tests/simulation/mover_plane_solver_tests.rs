//! Ported from Tests/NatselTests/Simulation/MoverPlaneSolverTests.swift.

use crate::collision::mover::plane_solver::solve_planes;
use crate::math::functions::{abs_float, dot};
use crate::math::types::Vec3;
use crate::types::queries::CollisionPlane;

// "parallel planes resolve in one pass per plane"
#[test]
fn parallel_planes() {
    let mut planes = vec![CollisionPlane::default(); 3];
    planes[0].plane.normal = Vec3::new(0.0, 0.0, 1.0);
    planes[0].plane.offset = 0.5;
    planes[0].push_limit = f32::MAX;
    planes[1].plane.normal = Vec3::new(0.0, 0.0, 1.0);
    planes[1].plane.offset = 1.0;
    planes[1].push_limit = f32::MAX;

    let result = solve_planes(Vec3::ZERO, &mut planes);

    assert_eq!(result.iteration_count, 2);
    assert!(abs_float(result.delta.z - 1.0) < 0.0055);
}

// "a deep target takes the full iteration budget"
#[test]
fn game_planes() {
    // This scenario takes many iterations because the target is deep into the plane.
    let mut planes = vec![CollisionPlane::default(); 3];
    planes[0].plane.normal = Vec3::new(0.0, -0.23941046, 0.970918416);
    planes[0].plane.offset = 0.390724182;
    planes[0].push_limit = f32::MAX;
    planes[1].plane.normal = Vec3::new(0.0, 0.0, 1.0);
    planes[1].plane.offset = 1.49998093;
    planes[1].push_limit = f32::MAX;

    let mut target = Vec3::new(-2.5390625, 0.0, -73.6880798);
    planes[0].plane.offset -= dot(planes[0].plane.normal, target);
    planes[1].plane.offset -= dot(planes[1].plane.normal, target);
    target = Vec3::ZERO;

    let result = solve_planes(target, &mut planes);

    assert_eq!(result.iteration_count, 20);
}
