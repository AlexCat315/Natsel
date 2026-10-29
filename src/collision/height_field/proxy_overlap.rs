//! Height field versus proxy overlap.

use crate::base::constants as b3;
use crate::collision::aabb::bounds_triangle_overlap;
use crate::collision::distance::shape_distance;
use crate::collision::proxy::{compute_proxy_aabb, make_local_proxy};
use crate::math::types::Transform;
use crate::types::queries::{DistanceInput, ShapeProxy, SimplexCache};

use super::quantization::HeightField;
use super::regions::HeightFieldRegion;

/// Do a height field and a proxy overlap?
pub fn overlap_height_field(field: &HeightField, transform: Transform, proxy: ShapeProxy) -> bool {
    let mut local_points = Vec::new();
    let local = make_local_proxy(proxy, transform, &mut local_points);
    let bounds = compute_proxy_aabb(local);
    let center = 0.5 * (bounds.lower_bound + bounds.upper_bound);
    let extent = bounds.upper_bound - center;
    let region = HeightFieldRegion::new(field, bounds);

    let mut input = DistanceInput {
        proxy_a: ShapeProxy::default(),
        proxy_b: local,
        transform: Transform::IDENTITY,
        use_radii: true,
    };
    let mut cache = SimplexCache::default();

    for row in region.rows {
        for column in region.columns {
            let cell = row * (field.column_count() - 1) + column;
            if field.material_indices()[cell] == 0xff {
                continue;
            }
            let v = field.corners(row, column);

            for second in [false, true] {
                let sat = if second {
                    [v[2], v[3], v[1]]
                } else {
                    [v[0], v[2], v[1]]
                };
                if !bounds_triangle_overlap(center, extent, sat[0], sat[1], sat[2]) {
                    continue;
                }

                let points = if second {
                    [v[3], v[1], v[2]]
                } else {
                    [v[0], v[2], v[1]]
                };
                input.proxy_a = ShapeProxy::new(Some(points.as_ptr()), 3, 0.0);
                cache.count = 0;
                if shape_distance(&input, &mut cache, None).distance < 0.1 * b3::linear_slop() {
                    return true;
                }
            }
        }
    }

    false
}
