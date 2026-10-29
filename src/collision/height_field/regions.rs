//! Height-field cell region iteration.

use crate::math::functions::{max_vec3, min_vec3};
use crate::math::types::{AABB, Vec3};

use super::quantization::HeightField;

/// The cell ranges overlapped by a query.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HeightFieldRegion {
    pub rows: core::ops::Range<usize>,
    pub columns: core::ops::Range<usize>,
}

impl HeightFieldRegion {
    pub fn new(field: &HeightField, bounds: AABB) -> Self {
        let scale = field.scale();
        let min_row = (bounds.lower_bound.z / scale.z).floor() as i64;
        let max_row = (bounds.upper_bound.z / scale.z).floor() as i64;
        let min_column = (bounds.lower_bound.x / scale.x).floor() as i64;
        let max_column = (bounds.upper_bound.x / scale.x).floor() as i64;

        let row_start = 0.max(min_row);
        let row_end = ((field.row_count() as i64) - 1).min(max_row + 1);
        let column_start = 0.max(min_column);
        let column_end = ((field.column_count() as i64) - 1).min(max_column + 1);

        Self {
            rows: row_start as usize..row_start.max(row_end) as usize,
            columns: column_start as usize..column_start.max(column_end) as usize,
        }
    }
}

/// The AABB of a cell's four corners.
pub fn height_cell_bounds(corners: &[Vec3; 4]) -> AABB {
    AABB::new(
        min_vec3(min_vec3(corners[0], corners[1]), min_vec3(corners[2], corners[3])),
        max_vec3(max_vec3(corners[0], corners[1]), max_vec3(corners[2], corners[3])),
    )
}

/// Visit height-field triangles overlapping the bounds. Return values are ignored.
pub fn query_height_field(
    field: &HeightField,
    bounds: AABB,
    mut callback: impl FnMut(Vec3, Vec3, Vec3, i32) -> bool,
) {
    let region = HeightFieldRegion::new(field, bounds);
    for row in region.rows {
        for column in region.columns {
            let cell = row * (field.column_count() - 1) + column;
            if field.material_indices()[cell] == 0xff {
                continue;
            }
            let v = field.corners(row, column);
            if crate::math::functions::aabb_overlaps(bounds, height_cell_bounds(&v)) {
                let index = (2 * cell) as i32;
                if field.header().clockwise != 0 {
                    let _ = callback(v[0], v[1], v[2], index);
                    let _ = callback(v[3], v[2], v[1], index + 1);
                } else {
                    let _ = callback(v[0], v[2], v[1], index);
                    let _ = callback(v[3], v[1], v[2], index + 1);
                }
            }
        }
    }
}
