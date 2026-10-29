//! Time of impact using swept separating axes and a root finder.

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::collision::distance::{
    get_point_support, get_sweep_transform, get_final_sweep_transform, shape_distance,
};
use crate::math::functions::*;
use crate::math::types::{Transform, Vec3};
use crate::types::queries::*;

/// The kind of feature the separation function is based on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SeparationType {
    #[default]
    Unknown = 0,
    Vertices,
    Edges,
    FaceA,
    FaceB,
}

/// A separation function evaluated over two sweeps.
#[derive(Clone, Copy, Debug, Default)]
pub struct SeparationFunction {
    pub proxy_a: ShapeProxy,
    pub proxy_b: ShapeProxy,
    pub sweep_a: Sweep,
    pub sweep_b: Sweep,
    pub witness1: Vec3,
    pub witness2: Vec3,
    pub kind: SeparationType,
}

/// Count the number of distinct indices.
pub fn unique_count(vertex_count: i32, vertices: &mut [i32; 3]) -> i32 {
    b3_assert(1 <= vertex_count && vertex_count <= 3);
    match vertex_count {
        1 => 1,
        2 => {
            if vertices[0] != vertices[1] {
                2
            } else {
                1
            }
        }
        3 => {
            if vertices[0] != vertices[1] && vertices[0] != vertices[2] && vertices[1] != vertices[2] {
                3
            } else if vertices[0] == vertices[1]
                && vertices[0] == vertices[2]
                && vertices[1] == vertices[2]
            {
                1
            } else {
                2
            }
        }
        _ => {
            b3_assert(false);
            0
        }
    }
}

/// Check if the cross product of two edges switches direction.
pub fn check_fast_edges(
    xf_a: Transform,
    local_edge_a: Vec3,
    xf_b: Transform,
    local_edge_b: Vec3,
    axis0: Vec3,
) -> bool {
    let edge_a = rotate_vector(xf_a.q, local_edge_a);
    let edge_b = rotate_vector(xf_b.q, local_edge_b);
    let axis = cross(edge_a, edge_b);
    dot(axis, axis0) < 0.0
}

/// Build a separation function from a GJK simplex cache.
pub fn make_separation_function(
    cache: &SimplexCache,
    proxy_a: ShapeProxy,
    sweep_a: Sweep,
    proxy_b: ShapeProxy,
    sweep_b: Sweep,
    world_normal: Vec3,
    t1: f32,
) -> SeparationFunction {
    b3_assert(1 <= cache.count && cache.count <= 3);
    b3_validate(is_normalized(world_normal));

    let mut fcn = SeparationFunction {
        proxy_a,
        proxy_b,
        sweep_a,
        sweep_b,
        witness1: Vec3::default(),
        witness2: Vec3::default(),
        kind: SeparationType::Unknown,
    };

    let mut index_a = [
        cache.index_a[0] as i32,
        cache.index_a[1] as i32,
        cache.index_a[2] as i32,
    ];
    let mut index_b = [
        cache.index_b[0] as i32,
        cache.index_b[1] as i32,
        cache.index_b[2] as i32,
    ];

    let unique_count_a = unique_count(cache.count as i32, &mut index_a);
    let unique_count_b = unique_count(cache.count as i32, &mut index_b);

    let xf_a1 = get_sweep_transform(&sweep_a, t1);
    let xf_b1 = get_sweep_transform(&sweep_b, t1);
    let q_a = xf_a1.q;
    let q_b = xf_b1.q;
    let delta_p = xf_b1.p - xf_a1.p;

    let points_a = proxy_a.points();
    let points_b = proxy_b.points();

    match cache.count {
        1 => {
            fcn.kind = SeparationType::Vertices;
            fcn.witness1 = world_normal;
        }
        2 => {
            if unique_count_a == 2 && unique_count_b == 2 {
                let va1 = points_a[index_a[0] as usize];
                let local_edge_a = normalize(points_a[index_a[1] as usize] - va1);
                let edge_a = rotate_vector(q_a, local_edge_a);

                let vb1 = points_b[index_b[0] as usize];
                let mut local_edge_b = normalize(points_b[index_b[1] as usize] - vb1);
                let edge_b = rotate_vector(q_b, local_edge_b);

                let mut axis = cross(edge_a, edge_b);
                let length_sq = length_squared(axis);
                let tolerance_sq: f32 = 0.05 * 0.05;
                if length_sq < tolerance_sq {
                    fcn.kind = SeparationType::Vertices;
                    fcn.witness1 = world_normal;
                } else {
                    let delta =
                        (rotate_vector(q_b, vb1) - rotate_vector(q_a, va1)) + delta_p;
                    if dot(delta, axis) < 0.0 {
                        axis = -axis;
                        local_edge_b = -local_edge_b;
                    }
                    let xf_a2 = get_final_sweep_transform(&sweep_a);
                    let xf_b2 = get_final_sweep_transform(&sweep_b);
                    let fast_edges = check_fast_edges(xf_a2, local_edge_a, xf_b2, local_edge_b, axis);
                    if fast_edges {
                        fcn.kind = SeparationType::Vertices;
                        fcn.witness1 = normalize(axis);
                    } else {
                        fcn.kind = SeparationType::Edges;
                        fcn.witness1 = local_edge_a;
                        fcn.witness2 = local_edge_b;
                    }
                }
            } else {
                b3_validate(is_normalized(world_normal));
                fcn.kind = SeparationType::Vertices;
                fcn.witness1 = world_normal;
            }
        }
        3 => {
            if unique_count_a == 3 {
                let va1 = points_a[index_a[0] as usize];
                let va2 = points_a[index_a[1] as usize];
                let va3 = points_a[index_a[2] as usize];
                let mut local_axis_a = normalize(cross(va2 - va1, va3 - va1));
                let axis_a = rotate_vector(q_a, local_axis_a);

                let local_point_a = (1.0 / 3.0) * ((va1 + va2) + va3);
                let local_point_b = points_b[index_b[0] as usize];
                let delta =
                    (rotate_vector(q_b, local_point_b) - rotate_vector(q_a, local_point_a)) + delta_p;

                if dot(delta, axis_a) < 0.0 {
                    local_axis_a = -local_axis_a;
                }

                fcn.kind = SeparationType::FaceA;
                fcn.witness1 = local_axis_a;
                fcn.witness2 = local_point_a;
            } else if unique_count_b == 3 {
                let vb1 = points_b[index_b[0] as usize];
                let vb2 = points_b[index_b[1] as usize];
                let vb3 = points_b[index_b[2] as usize];
                let mut local_axis_b = normalize(cross(vb2 - vb1, vb3 - vb1));
                let axis_b = rotate_vector(q_b, local_axis_b);

                let local_point_a = points_a[index_a[0] as usize];
                let local_point_b = (1.0 / 3.0) * ((vb1 + vb2) + vb3);
                let delta =
                    (rotate_vector(q_a, local_point_a) - rotate_vector(q_b, local_point_b)) - delta_p;

                if dot(delta, axis_b) < 0.0 {
                    local_axis_b = -local_axis_b;
                }

                fcn.kind = SeparationType::FaceB;
                fcn.witness1 = local_axis_b;
                fcn.witness2 = local_point_b;
            } else {
                b3_assert(unique_count_a == 2 && unique_count_b == 2);

                if index_a[0] == index_a[1] {
                    index_a[1] = index_a[2];
                    b3_assert(index_a[0] != index_a[1]);
                }

                let va1 = points_a[index_a[0] as usize];
                let va2 = points_a[index_a[1] as usize];
                let local_edge_a = normalize(va2 - va1);
                let edge_a = rotate_vector(q_a, local_edge_a);

                if index_b[0] == index_b[1] {
                    index_b[1] = index_b[2];
                    b3_assert(index_b[0] != index_b[1]);
                }

                let vb1 = points_b[index_b[0] as usize];
                let vb2 = points_b[index_b[1] as usize];
                let mut local_edge_b = normalize(vb2 - vb1);
                let edge_b = rotate_vector(q_b, local_edge_b);

                let mut axis = cross(edge_a, edge_b);
                let length_sq = length_squared(axis);
                let tolerance_sq: f32 = 0.005 * 0.005;
                if length_sq < tolerance_sq {
                    fcn.kind = SeparationType::Vertices;
                    fcn.witness1 = world_normal;
                } else {
                    let delta = (rotate_vector(q_b, vb1) - rotate_vector(q_a, va1)) + delta_p;
                    if dot(delta, axis) < 0.0 {
                        axis = -axis;
                        local_edge_b = -local_edge_b;
                    }
                    let xf_a2 = get_final_sweep_transform(&sweep_a);
                    let xf_b2 = get_final_sweep_transform(&sweep_b);
                    let fast_edges = check_fast_edges(xf_a2, local_edge_a, xf_b2, local_edge_b, axis);
                    if fast_edges {
                        fcn.kind = SeparationType::Vertices;
                        fcn.witness1 = normalize(axis);
                    } else {
                        fcn.kind = SeparationType::Edges;
                        fcn.witness1 = local_edge_a;
                        fcn.witness2 = local_edge_b;
                    }
                }
            }
        }
        _ => b3_assert(false),
    }

    fcn
}

/// Find the minimum separation along the separating axis at time `t`.
pub fn find_min_separation(
    fcn: &SeparationFunction,
    index_a: &mut i32,
    index_b: &mut i32,
    t: f32,
) -> f32 {
    let xf_a = get_sweep_transform(&fcn.sweep_a, t);
    let xf_b = get_sweep_transform(&fcn.sweep_b, t);
    let points_a = fcn.proxy_a.points();
    let points_b = fcn.proxy_b.points();

    match fcn.kind {
        SeparationType::Vertices => {
            let axis = fcn.witness1;
            let local_axis_a = inv_rotate_vector(xf_a.q, axis);
            let local_axis_b = inv_rotate_vector(xf_b.q, -axis);

            *index_a = get_point_support(points_a, local_axis_a);
            *index_b = get_point_support(points_b, local_axis_b);

            let delta_p = xf_b.p - xf_a.p;
            let local_point_a = points_a[*index_a as usize];
            let local_point_b = points_b[*index_b as usize];
            let delta =
                (rotate_vector(xf_b.q, local_point_b) - rotate_vector(xf_a.q, local_point_a)) + delta_p;
            dot(delta, axis)
        }
        SeparationType::Edges => {
            let edge_a = rotate_vector(xf_a.q, fcn.witness1);
            let edge_b = rotate_vector(xf_b.q, fcn.witness2);
            let mut axis = cross(edge_a, edge_b);
            b3_assert(axis.x != 0.0 || axis.y != 0.0 || axis.z != 0.0);
            axis = normalize(axis);

            let axis_a = inv_rotate_vector(xf_a.q, axis);
            *index_a = get_point_support(points_a, axis_a);

            let axis_b = inv_rotate_vector(xf_b.q, axis);
            *index_b = get_point_support(points_b, -axis_b);

            let delta_p = xf_b.p - xf_a.p;
            let local_point_a = points_a[*index_a as usize];
            let local_point_b = points_b[*index_b as usize];
            let delta =
                (rotate_vector(xf_b.q, local_point_b) - rotate_vector(xf_a.q, local_point_a)) + delta_p;
            dot(delta, axis)
        }
        SeparationType::FaceA => {
            let normal = rotate_vector(xf_a.q, fcn.witness1);
            *index_a = -1;
            let point_a = transform_point(xf_a, fcn.witness2);

            let axis_b = inv_rotate_vector(xf_b.q, normal);
            *index_b = get_point_support(points_b, -axis_b);
            let point_b = transform_point(xf_b, points_b[*index_b as usize]);

            dot(point_b - point_a, normal)
        }
        SeparationType::FaceB => {
            let normal = rotate_vector(xf_b.q, fcn.witness1);

            let axis_a = inv_rotate_vector(xf_a.q, normal);
            *index_a = get_point_support(points_a, -axis_a);
            let point_a = transform_point(xf_a, points_a[*index_a as usize]);

            *index_b = -1;
            let point_b = transform_point(xf_b, fcn.witness2);

            dot(point_a - point_b, normal)
        }
        SeparationType::Unknown => {
            b3_assert(false);
            0.0
        }
    }
}

/// Evaluate the separation at time `beta` for the given witness indices.
pub fn evaluate_separation(fcn: &SeparationFunction, index1: i32, index2: i32, beta: f32) -> f32 {
    let transform1 = get_sweep_transform(&fcn.sweep_a, beta);
    let transform2 = get_sweep_transform(&fcn.sweep_b, beta);
    let points_a = fcn.proxy_a.points();
    let points_b = fcn.proxy_b.points();

    match fcn.kind {
        SeparationType::Vertices => {
            let axis = fcn.witness1;
            let point1 = transform_point(transform1, points_a[index1 as usize]);
            let point2 = transform_point(transform2, points_b[index2 as usize]);
            dot(point2 - point1, axis)
        }
        SeparationType::Edges => {
            let edge1 = rotate_vector(transform1.q, fcn.witness1);
            let edge2 = rotate_vector(transform2.q, fcn.witness2);
            let axis = normalize(cross(edge1, edge2));
            let point1 = transform_point(transform1, points_a[index1 as usize]);
            let point2 = transform_point(transform2, points_b[index2 as usize]);
            dot(point2 - point1, axis)
        }
        SeparationType::FaceA => {
            let axis = rotate_vector(transform1.q, fcn.witness1);
            let point1 = transform_point(transform1, fcn.witness2);
            let point2 = transform_point(transform2, points_b[index2 as usize]);
            dot(point2 - point1, axis)
        }
        SeparationType::FaceB => {
            let axis = rotate_vector(transform2.q, fcn.witness1);
            let point1 = transform_point(transform1, points_a[index1 as usize]);
            let point2 = transform_point(transform2, fcn.witness2);
            dot(point1 - point2, axis)
        }
        SeparationType::Unknown => {
            b3_assert(false);
            0.0
        }
    }
}

/// Fall back to a fixed world axis when the edge/edge root finder fails.
pub fn force_fixed_axis(fcn: &mut SeparationFunction, beta: f32) {
    b3_assert(fcn.kind == SeparationType::Edges);

    let transform1 = get_sweep_transform(&fcn.sweep_a, beta);
    let transform2 = get_sweep_transform(&fcn.sweep_b, beta);
    let edge1 = rotate_vector(transform1.q, fcn.witness1);
    let edge2 = rotate_vector(transform2.q, fcn.witness2);
    let axis = normalize(cross(edge1, edge2));

    fcn.kind = SeparationType::Vertices;
    fcn.witness1 = axis;
    fcn.witness2 = Vec3::default();
}

/// Compute the upper bound on time before two shapes penetrate.
pub fn time_of_impact(input: &TOIInput) -> TOIOutput {
    let mut output = TOIOutput {
        state: TOIState::Unknown,
        fraction: -1.0,
        ..Default::default()
    };

    let mut sweep_a = input.sweep_a;
    let mut sweep_b = input.sweep_b;

    let origin = sweep_a.c1;
    sweep_a.c1 = Vec3::default();
    sweep_a.c2 = sweep_a.c2 - origin;
    sweep_b.c1 = sweep_b.c1 - origin;
    sweep_b.c2 = sweep_b.c2 - origin;

    let proxy_a = input.proxy_a;
    let proxy_b = input.proxy_b;

    let max_push_back_iterations = proxy_a.count + proxy_b.count;
    let t_max = input.max_fraction;

    let linear_slop = b3::linear_slop();
    let total_radius = proxy_a.radius + proxy_b.radius;
    let target = max_float(linear_slop, total_radius - linear_slop);
    let tolerance = 0.25 * linear_slop;
    b3_assert(target > tolerance);

    let mut t1: f32 = 0.0;
    let max_iterations = 25;
    let mut distance_iterations: i32 = 0;

    let mut cache = SimplexCache::default();
    let mut distance_input = DistanceInput {
        proxy_a,
        proxy_b,
        use_radii: false,
        ..Default::default()
    };

    loop {
        let xf_a = get_sweep_transform(&sweep_a, t1);
        let xf_b = get_sweep_transform(&sweep_b, t1);
        distance_input.transform = inv_mul_transforms(xf_a, xf_b);
        let distance_output = shape_distance(&distance_input, &mut cache, None);
        output.distance = distance_output.distance;

        let world_normal = rotate_vector(xf_a.q, distance_output.normal);
        let world_point_a = transform_point(xf_a, distance_output.point_a);
        let world_point_b = transform_point(xf_a, distance_output.point_b);

        output.distance_iterations += 1;
        distance_iterations += 1;

        if distance_output.distance <= 0.0 {
            output.state = TOIState::Overlapped;
            output.fraction = 0.0;
            let p_a = mul_add(world_point_a, proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5) + origin;
            output.normal = world_normal;
            break;
        }

        if distance_output.distance <= target + tolerance {
            output.state = TOIState::Hit;
            let p_a = mul_add(world_point_a, proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5) + origin;
            output.normal = world_normal;
            output.fraction = t1;
            break;
        }

        if distance_iterations == max_iterations {
            output.state = TOIState::Failed;
            output.fraction = t1;
            let p_a = mul_add(world_point_a, input.proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -input.proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5) + origin;
            output.normal = world_normal;
            break;
        }

        let mut fcn = make_separation_function(
            &cache,
            proxy_a,
            sweep_a,
            proxy_b,
            sweep_b,
            world_normal,
            t1,
        );

        let mut done = false;
        let mut t2 = t_max;
        let mut push_back_iterations: i32 = 0;

        loop {
            let mut index_a: i32 = 0;
            let mut index_b: i32 = 0;
            let mut s2 = find_min_separation(&fcn, &mut index_a, &mut index_b, t2);

            if s2 - target > tolerance {
                output.state = TOIState::Separated;
                output.fraction = input.max_fraction;
                done = true;
                break;
            }

            if s2 >= target - tolerance {
                t1 = t2;
                break;
            }

            let mut s1 = evaluate_separation(&fcn, index_a, index_b, t1);

            if s1 < target - tolerance {
                b3_validate(false);
                output.state = TOIState::Failed;
                output.fraction = t1;
                done = true;
                break;
            }

            if s1 <= target + tolerance {
                output.state = TOIState::Hit;
                output.fraction = t1;
                done = true;
                break;
            }

            let mut root_iteration_count: i32 = 0;
            let max_root_iterations: i32 = 50;
            let mut a1 = t1;
            let mut a2 = t2;
            loop {
                let t = if root_iteration_count & 1 != 0 {
                    a1 + (target - s1) * (a2 - a1) / (s2 - s1)
                } else {
                    0.5 * (a1 + a2)
                };

                output.root_iterations += 1;
                root_iteration_count += 1;

                let s = evaluate_separation(&fcn, index_a, index_b, t);

                if abs_float(s - target) <= tolerance {
                    t2 = t;
                    break;
                }

                if s > target {
                    a1 = t;
                    s1 = s;
                } else {
                    a2 = t;
                    s2 = s;
                }

                if root_iteration_count == max_root_iterations {
                    b3_validate(false);
                    break;
                }
            }

            if root_iteration_count == max_root_iterations - 1
                && fcn.kind == SeparationType::Edges
            {
                b3_validate(false);
                root_iteration_count = 0;
                t2 = input.max_fraction;
                force_fixed_axis(&mut fcn, t1);
                b3_assert(fcn.kind != SeparationType::Edges);
            }

            output.push_back_iterations += 1;
            push_back_iterations += 1;

            if push_back_iterations == max_push_back_iterations {
                break;
            }
        }

        if done {
            let p_a = mul_add(world_point_a, input.proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -input.proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5) + origin;
            output.normal = world_normal;
            break;
        }
    }

    b3_assert(output.state != TOIState::Unknown);
    b3_assert(output.fraction >= 0.0);
    output
}
