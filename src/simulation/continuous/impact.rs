//! Triangle-query TOI, including its centroid fallback and static-surface assumption.

use crate::base::constants as b3;
use crate::collision::time_of_impact::time_of_impact;
use crate::math::functions::{
    aabb_transform, cross, dot, inv_transform_point, invert_transform, max_float, normalize,
    rotate_vector, transform_point,
};
use crate::math::types::{AABB, Transform, Vec3};
use crate::simulation::shape_runtime::{
    compute_shape_min_extent, compute_swept_shape_aabb, get_shape_centroid, make_shape_proxy, Shape,
};
use crate::simulation::triangle_surface::TriangleSurface;
use crate::types::queries::{ShapeProxy, Sweep, TOIInput, TOIOutput};

/// Triangle-query TOI, including its centroid fallback and static-surface assumption.
pub(crate) fn triangle_surface_time_of_impact(
    surface: &TriangleSurface,
    shape_b: &Shape,
    sweep_a: Sweep,
    sweep_b: Sweep,
    max_fraction: f32,
    is_sensor: bool,
) -> TOIOutput {
    let centroid = get_shape_centroid(shape_b);
    let transform_a = Transform::new(
        sweep_a.c1 - rotate_vector(sweep_a.q1, sweep_a.local_center),
        sweep_a.q1,
    );
    let min_extent = compute_shape_min_extent(shape_b, centroid);
    let fallback_radius = max_float(0.5 * min_extent, b3::linear_slop());
    let bounds = compute_swept_shape_aabb(shape_b, sweep_b, max_fraction);
    let local_bounds = aabb_transform(invert_transform(transform_a), bounds);
    triangle_surface_impact(
        surface,
        shape_b,
        sweep_a,
        sweep_b,
        max_fraction,
        is_sensor,
        transform_a,
        local_bounds,
        fallback_radius,
    )
}

/// TOI against the triangles of a surface already limited to `local_bounds`.
pub(crate) fn triangle_surface_impact(
    surface: &TriangleSurface,
    shape_b: &Shape,
    sweep_a: Sweep,
    sweep_b: Sweep,
    max_fraction: f32,
    is_sensor: bool,
    transform_a: Transform,
    local_bounds: AABB,
    fallback_radius: f32,
) -> TOIOutput {
    let centroid = get_shape_centroid(shape_b);
    let transform_b1 = Transform::new(
        sweep_b.c1 - rotate_vector(sweep_b.q1, sweep_b.local_center),
        sweep_b.q1,
    );
    let transform_b2 = Transform::new(
        sweep_b.c2 - rotate_vector(sweep_b.q2, sweep_b.local_center),
        sweep_b.q2,
    );
    let center1 = inv_transform_point(transform_a, transform_point(transform_b1, centroid));
    let center2 = inv_transform_point(transform_a, transform_point(transform_b2, centroid));
    let buffer = make_shape_proxy(shape_b);
    // `buffer` owns the point storage and outlives every TOI call made below, which is the
    // invariant Swift's `withProxy` provided.
    let proxy = buffer.proxy();
    let mut input = TOIInput::default();
    input.proxy_b = proxy;
    input.sweep_a = sweep_a;
    input.sweep_b = sweep_b;
    input.max_fraction = max_fraction;
    let mut best = TOIOutput::default();
    surface.query(local_bounds, |a, b, c, _| {
        let normal = normalize(cross(b - a, c - a));
        let offset1 = dot(normal, center1 - a);
        let offset2 = dot(normal, center2 - a);
        if offset1 < 0.0 {
            return true;
        }
        if !is_sensor && offset1 - offset2 < fallback_radius && offset2 > fallback_radius {
            return true;
        }
        let triangle: [Vec3; 3] = [a, b, c];
        // SAFETY: `triangle` is a local that stays alive, unmutated and unmoved for the whole
        // `time_of_impact` call, matching Swift's `withInlineReadBuffer`. The proxy borrows the
        // storage for the duration of that call only.
        input.proxy_a = ShapeProxy::new(Some(triangle.as_ptr()), 3, 0.0);
        let mut output = time_of_impact(&input);
        if 0.0 < output.fraction && output.fraction < input.max_fraction {
            best = output;
            input.max_fraction = output.fraction;
        } else if output.fraction == 0.0 {
            // SAFETY: `centroid` is an immutable local that outlives the fallback TOI call,
            // mirroring Swift's `withUnsafePointer(to: centroid)`.
            let mut fallback = input;
            fallback.proxy_b = ShapeProxy::new(
                Some(&centroid as *const Vec3),
                1,
                fallback_radius + b3::linear_slop(),
            );
            output = time_of_impact(&fallback);
            if 0.0 < output.fraction && output.fraction < input.max_fraction {
                best = output;
                input.max_fraction = output.fraction;
                best.used_fallback = true;
            }
        }
        true
    });
    best
}
