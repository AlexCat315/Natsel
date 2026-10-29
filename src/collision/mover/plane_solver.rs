//! Mover plane helpers.

use crate::base::constants as b3;
use crate::collision::internal_math::plane_separation;
use crate::math::functions::*;
use crate::math::types::Vec3;
use crate::types::queries::{CollisionPlane, PlaneSolverResult};

pub fn solve_planes(target_delta: Vec3, planes: &mut [CollisionPlane]) -> PlaneSolverResult {
    for plane in planes.iter_mut() {
        plane.push = 0.0;
    }

    let mut delta = target_delta;
    let mut iteration: i32 = 0;
    while iteration < 20 {
        let mut total_push: f32 = 0.0;
        for plane in planes.iter_mut() {
            let separation = plane_separation(plane.plane, delta) + b3::linear_slop();
            let mut push = -separation;
            let accumulated_push = plane.push;
            plane.push = clamp_float(plane.push + push, 0.0, plane.push_limit);
            push = plane.push - accumulated_push;
            delta = mul_add(delta, push, plane.plane.normal);
            total_push += abs_float(push);
        }

        if total_push < b3::linear_slop() {
            break;
        }
        iteration += 1;
    }

    PlaneSolverResult {
        delta,
        iteration_count: iteration,
    }
}

pub fn clip_vector(vector: Vec3, planes: &[CollisionPlane]) -> Vec3 {
    let mut result = vector;
    for plane in planes {
        if plane.push == 0.0 || !plane.clip_velocity {
            continue;
        }
        result = mul_sub(result, min_float(0.0, dot(result, plane.plane.normal)), plane.plane.normal);
    }
    result
}
