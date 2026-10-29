//! Time of impact between a compound and a convex shape, child by child.

use crate::base::constants as b3;
use crate::collision::compound::child_queries::query_compound;
use crate::collision::compound::compound::{Compound, CompoundChildGeometry};
use crate::collision::time_of_impact::time_of_impact;
use crate::math::functions::{
    aabb_transform, invert_transform, max_float, mul_transforms,
};
use crate::math::types::{Transform, Vec3};
use crate::simulation::continuous::impact::triangle_surface_impact;
use crate::simulation::shape_runtime::{
    compute_shape_min_extent, compute_swept_shape_aabb, get_shape_centroid, make_shape_proxy, Shape,
};
use crate::simulation::triangle_surface::TriangleSurface;
use crate::types::queries::{ShapeProxy, Sweep, TOIInput, TOIOutput};

/// Time of impact between a compound and a convex shape, child by child.
pub(crate) fn compound_time_of_impact(
    compound: &Compound,
    shape_b: &Shape,
    sweep_a: Sweep,
    sweep_b: Sweep,
    max_fraction: f32,
) -> TOIOutput {
    let transform = Transform::new(sweep_a.c1, sweep_a.q1);
    let centroid = get_shape_centroid(shape_b);
    let fallback_radius = max_float(
        0.75 * compute_shape_min_extent(shape_b, centroid),
        b3::speculative_distance(),
    );
    let bounds = compute_swept_shape_aabb(shape_b, sweep_b, max_fraction);
    let local_bounds = aabb_transform(invert_transform(transform), bounds);
    let buffer = make_shape_proxy(shape_b);
    // `buffer` owns the point storage and outlives every TOI call made below, which is the
    // invariant Swift's `withProxy` provided.
    let proxy = buffer.proxy();
    let mut input = TOIInput::default();
    input.proxy_b = proxy;
    input.sweep_b = sweep_b;
    input.max_fraction = max_fraction;
    let mut best = TOIOutput::default();
    query_compound(compound, local_bounds, |index| {
        let child = compound.child(index);
        let child_transform = mul_transforms(transform, child.transform);
        // A compound child is static for the TOI window: it only moves with `sweep_a`.
        let mut sweep = Sweep::default();
        sweep.local_center = Vec3::ZERO;
        sweep.c1 = child_transform.p;
        sweep.c2 = child_transform.p;
        sweep.q1 = child_transform.q;
        sweep.q2 = child_transform.q;
        input.sweep_a = sweep;
        let output: TOIOutput = match &child.geometry {
            // Swift wrote `TriangleSurface(.mesh(mesh))!`; the enum construction cannot fail, so
            // the force unwrap has no Rust counterpart.
            CompoundChildGeometry::Mesh(mesh) => triangle_surface_impact(
                &TriangleSurface::Mesh(mesh.clone()),
                shape_b,
                sweep,
                sweep_b,
                input.max_fraction,
                false,
                child_transform,
                aabb_transform(invert_transform(child.transform), local_bounds),
                fallback_radius,
            ),
            CompoundChildGeometry::Capsule(capsule) => {
                let points = [capsule.center1, capsule.center2];
                // SAFETY: `points` is a local that stays alive, unmutated and unmoved for the
                // whole `time_of_impact` call, matching Swift's `withUnsafeBufferPointer`.
                input.proxy_a = ShapeProxy::new(Some(points.as_ptr()), 2, capsule.radius);
                time_of_impact(&input)
            }
            CompoundChildGeometry::Hull(hull) => {
                // SAFETY: the point slice is owned by `child`, which lives until the end of this
                // closure body, exactly like the pointer Swift read from `hull.points`.
                input.proxy_a =
                    ShapeProxy::new(Some(hull.points().as_ptr()), hull.vertex_count(), 0.0);
                time_of_impact(&input)
            }
            CompoundChildGeometry::Sphere(sphere) => {
                // SAFETY: `sphere.center` is read through an immutable borrow of `child`, which
                // outlives the fallback-free TOI call, mirroring `withUnsafePointer(to:)`.
                input.proxy_a =
                    ShapeProxy::new(Some(&sphere.center as *const Vec3), 1, sphere.radius);
                time_of_impact(&input)
            }
        };
        if 0.0 < output.fraction && output.fraction < input.max_fraction {
            best = output;
            input.max_fraction = output.fraction;
        }
        input.proxy_a = ShapeProxy::default();
        true
    });
    best
}
