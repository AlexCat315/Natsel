//! Shape proxy helpers: point buffers, local transforms, and proxy bounds.

use crate::collision::hull::Hull;
use crate::math::functions::*;
use crate::math::types::{AABB, Transform, Vec3};
use crate::types::queries::ShapeProxy;

/// Owns point storage for a shape proxy so the proxy's pointer stays valid.
#[derive(Clone, Debug)]
pub struct ShapeProxyBuffer {
    points: Vec<Vec3>,
    radius: f32,
    hull: Option<Hull>,
}

impl ShapeProxyBuffer {
    pub fn segment(a: Vec3, b: Vec3, radius: f32) -> Self {
        Self {
            points: vec![a, b],
            radius,
            hull: None,
        }
    }

    pub fn point(a: Vec3, radius: f32) -> Self {
        Self {
            points: vec![a],
            radius,
            hull: None,
        }
    }

    pub fn from_hull(hull: Hull) -> Self {
        Self {
            points: Vec::new(),
            radius: 0.0,
            hull: Some(hull),
        }
    }

    pub fn count(&self) -> i32 {
        match &self.hull {
            Some(h) => h.vertex_count(),
            None => self.points.len() as i32,
        }
    }

    /// Build a proxy that borrows this buffer. The proxy must not outlive `self`.
    pub fn proxy(&self) -> ShapeProxy {
        match &self.hull {
            Some(h) => ShapeProxy::new(Some(h.points().as_ptr()), h.vertex_count(), 0.0),
            None => ShapeProxy::new(Some(self.points.as_ptr()), self.points.len() as i32, self.radius),
        }
    }
}

/// Transform a proxy into a local frame using a caller-provided buffer.
pub fn make_local_proxy(proxy: ShapeProxy, transform: Transform, buffer: &mut Vec<Vec3>) -> ShapeProxy {
    let inv_transform = invert_transform(transform);
    let rotation = make_matrix_from_quat(inv_transform.q);

    let count = min_int(proxy.count, crate::base::constants::MAX_SHAPE_CAST_POINTS) as usize;
    buffer.clear();
    buffer.reserve(count);
    for i in 0..count {
        buffer.push(mul_mv(rotation, proxy.points()[i]) + inv_transform.p);
    }
    ShapeProxy::new(Some(buffer.as_ptr()), count as i32, proxy.radius)
}

/// Compute the AABB of a proxy.
pub fn compute_proxy_aabb(proxy: ShapeProxy) -> AABB {
    let points = proxy.points();
    let mut aabb = AABB::new(points[0], points[0]);
    for p in &points[1..] {
        aabb.lower_bound = min_vec3(aabb.lower_bound, *p);
        aabb.upper_bound = max_vec3(aabb.upper_bound, *p);
    }
    let r = Vec3::new(proxy.radius, proxy.radius, proxy.radius);
    aabb.lower_bound = aabb.lower_bound - r;
    aabb.upper_bound = aabb.upper_bound + r;
    aabb
}
