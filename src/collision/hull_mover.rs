//! Collision between a mover capsule and a hull.

use crate::collision::distance::shape_distance;
use crate::collision::hull::Hull;
use crate::math::types::{Plane, Transform};
use crate::types::geometry::Capsule;
use crate::types::queries::{DistanceInput, PlaneResult, ShapeProxy, SimplexCache};

pub fn collide_mover_and_hull(result: &mut PlaneResult, shape: &Hull, mover: &Capsule) -> i32 {
    let points = [mover.center1, mover.center2];
    let input = DistanceInput {
        proxy_a: ShapeProxy::new(Some(shape.points().as_ptr()), shape.vertex_count(), 0.0),
        proxy_b: ShapeProxy::new(Some(points.as_ptr()), 2, mover.radius),
        transform: Transform::IDENTITY,
        use_radii: false,
    };
    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    if output.distance == 0.0 {
        return 0;
    }
    if output.distance <= mover.radius {
        *result = PlaneResult::default();
        result.plane = Plane::new(output.normal, mover.radius - output.distance);
        result.point = output.point_a;
        return 1;
    }
    0
}
