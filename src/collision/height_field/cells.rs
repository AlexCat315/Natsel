//! Height-field cell corners and triangles.

use crate::collision::internal_math::Triangle;
use crate::math::functions::aabb_transform;
use crate::math::types::{AABB, Transform, Vec3};

use super::quantization::HeightField;

impl HeightField {
    /// The four grid corners of a cell.
    pub fn corners(&self, row: usize, column: usize) -> [Vec3; 4] {
        let columns = self.column_count();
        let i11 = row * columns + column;
        let i12 = i11 + 1;
        let i21 = (row + 1) * columns + column;
        let i22 = i21 + 1;

        let header = self.header();
        let heights = self.compressed_heights();
        let h11 = header.min_height + header.height_scale * heights[i11] as f32;
        let h12 = header.min_height + header.height_scale * heights[i12] as f32;
        let h21 = header.min_height + header.height_scale * heights[i21] as f32;
        let h22 = header.min_height + header.height_scale * heights[i22] as f32;

        let x1 = column as f32;
        let x2 = (column + 1) as f32;
        let z1 = row as f32;
        let z2 = (row + 1) as f32;
        let scale = header.scale;

        [
            scale * Vec3::new(x1, h11, z1),
            scale * Vec3::new(x2, h12, z1),
            scale * Vec3::new(x1, h21, z2),
            scale * Vec3::new(x2, h22, z2),
        ]
    }

    /// A height-field triangle by index.
    pub fn triangle(&self, index: i32) -> Triangle {
        let columns = self.column_count();
        let cell = index as usize >> 1;
        let row = cell / (columns - 1);
        let column = cell - row * (columns - 1);
        let v = self.corners(row, column);

        let i11 = (row * columns + column) as i32;
        let i12 = i11 + 1;
        let i21 = ((row + 1) * columns + column) as i32;
        let i22 = i21 + 1;

        let mut triangle = Triangle::default();
        triangle.flags = self.edge_flags()[index as usize] as i32;
        if index & 1 == 0 {
            triangle.vertices = [v[0], v[2], v[1]];
            triangle.i1 = i11;
            triangle.i2 = i21;
            triangle.i3 = i12;
        } else {
            triangle.vertices = [v[3], v[1], v[2]];
            triangle.i1 = i22;
            triangle.i2 = i12;
            triangle.i3 = i21;
        }

        if self.header().clockwise != 0 {
            triangle.vertices.swap(1, 2);
            std::mem::swap(&mut triangle.i2, &mut triangle.i3);
            let flags = triangle.flags;
            let edge1 = flags & 0x11;
            let edge3 = flags & 0x44;
            triangle.flags = (flags & !0x55) | (edge1 << 2) | (edge3 >> 2);
        }

        triangle
    }

    /// The material index of a height-field triangle.
    pub fn material(&self, triangle: i32) -> i32 {
        self.material_indices()[triangle as usize >> 1] as i32
    }
}

/// Compute the world AABB of a height field.
pub fn compute_height_field_aabb(field: &HeightField, transform: Transform) -> AABB {
    aabb_transform(transform, field.bounds())
}
