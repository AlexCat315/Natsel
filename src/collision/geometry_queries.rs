//! Triangle queries and inertia tensors.

use crate::collision::internal_math::{make_diagonal_matrix, TrianglePoint};
use crate::math::functions::*;
use crate::math::types::{Matrix3, Vec3};
use crate::types::collision_types::TriangleFeature;

/// Find the closest point on triangle `a-b-c` to point `q`.
pub fn closest_point_on_triangle(a: Vec3, b: Vec3, c: Vec3, q: Vec3) -> TrianglePoint {
    let ab = b - a;
    let ac = c - a;
    let aq = q - a;

    let d1 = dot(ab, aq);
    let d2 = dot(ac, aq);
    if d1 <= 0.0 && d2 <= 0.0 {
        return TrianglePoint {
            point: a,
            feature: TriangleFeature::FeatureVertex1,
        };
    }

    let bq = q - b;
    let d3 = dot(ab, bq);
    let d4 = dot(ac, bq);
    if d3 > 0.0 && d4 <= d3 {
        return TrianglePoint {
            point: b,
            feature: TriangleFeature::FeatureVertex2,
        };
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        return TrianglePoint {
            point: mul_add(a, t, ab),
            feature: TriangleFeature::FeatureEdge1,
        };
    }

    let cq = q - c;
    let d5 = dot(ab, cq);
    let d6 = dot(ac, cq);
    if d6 >= 0.0 && d5 <= d6 {
        return TrianglePoint {
            point: c,
            feature: TriangleFeature::FeatureVertex3,
        };
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        return TrianglePoint {
            point: mul_add(a, t, ac),
            feature: TriangleFeature::FeatureEdge3,
        };
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 >= d3 && d5 >= d6 {
        let bc = c - b;
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return TrianglePoint {
            point: mul_add(b, t, bc),
            feature: TriangleFeature::FeatureEdge2,
        };
    }

    let t1 = vb / (va + vb + vc);
    let t2 = vc / (va + vb + vc);
    let p = mul_add(mul_add(a, t1, ab), t2, ac);
    TrianglePoint {
        point: p,
        feature: TriangleFeature::FeatureTriangleFace,
    }
}

/// The inertia tensor of a solid box about its center.
pub fn box_inertia(mass: f32, min: Vec3, max: Vec3) -> Matrix3 {
    let delta = max - min;
    let ixx = mass * (delta.y * delta.y + delta.z * delta.z) / 12.0;
    let iyy = mass * (delta.x * delta.x + delta.z * delta.z) / 12.0;
    let izz = mass * (delta.x * delta.x + delta.y * delta.y) / 12.0;
    make_diagonal_matrix(ixx, iyy, izz)
}
