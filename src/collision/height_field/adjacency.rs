//! Height-field edge adjacency flags.

use crate::collision::internal_math::{make_plane_from_points, plane_separation};
use crate::math::functions::*;
use crate::math::types::Vec3;

/// Compute the per-triangle edge flags of a height field.
pub fn height_field_adjacency(
    heights: &[f32],
    materials: &[u8],
    columns: usize,
    rows: usize,
    scale: Vec3,
) -> Vec<u8> {
    let vertices = |row: usize, column: usize, second: bool| -> [Vec3; 3] {
        let i11 = row * columns + column;
        let i12 = i11 + 1;
        let i21 = (row + 1) * columns + column;
        let i22 = i21 + 1;
        let x1 = column as f32;
        let x2 = (column + 1) as f32;
        let z1 = row as f32;
        let z2 = (row + 1) as f32;
        if second {
            [
                scale * Vec3::new(x2, heights[i22], z2),
                scale * Vec3::new(x2, heights[i12], z1),
                scale * Vec3::new(x1, heights[i21], z2),
            ]
        } else {
            [
                scale * Vec3::new(x1, heights[i11], z1),
                scale * Vec3::new(x1, heights[i21], z2),
                scale * Vec3::new(x2, heights[i12], z1),
            ]
        }
    };

    let edge_flags = |separation: f32, cosine: f32, edge: u8| -> u8 {
        let mut flags: u8 = 0;
        if separation > 0.0 || cosine > 0.9962 {
            flags |= 1 << edge;
        }
        if separation < 0.0 || cosine > 0.9962 {
            flags |= 0x10 << edge;
        }
        flags
    };

    let mut flags = vec![0u8; 2 * (rows - 1) * (columns - 1)];
    for row in 0..rows - 1 {
        for column in 0..columns - 1 {
            let cell = row * (columns - 1) + column;
            if materials[cell] == 0xff {
                continue;
            }

            let first = vertices(row, column, false);
            let second = vertices(row, column, true);
            let plane1 = make_plane_from_points(first[0], first[1], first[2]);
            let plane2 = make_plane_from_points(second[0], second[1], second[2]);

            let diagonal = edge_flags(
                plane_separation(plane1, second[0]),
                dot(plane1.normal, plane2.normal),
                1,
            );
            let mut flags1 = diagonal;
            let mut flags2 = diagonal;

            if row > 0 && materials[(row - 1) * (columns - 1) + column] != 0xff {
                let v = vertices(row - 1, column, true);
                let n = normalize(cross(v[1] - v[0], v[2] - v[0]));
                flags1 |= edge_flags(plane_separation(plane1, v[1]), dot(plane1.normal, n), 2);
            }
            if row + 1 < rows - 1 && materials[(row + 1) * (columns - 1) + column] != 0xff {
                let v = vertices(row + 1, column, false);
                let n = normalize(cross(v[1] - v[0], v[2] - v[0]));
                flags2 |= edge_flags(plane_separation(plane2, v[1]), dot(plane2.normal, n), 2);
            }
            if column > 0 && materials[row * (columns - 1) + column - 1] != 0xff {
                let v = vertices(row, column - 1, true);
                let n = normalize(cross(v[1] - v[0], v[2] - v[0]));
                flags1 |= edge_flags(plane_separation(plane1, v[2]), dot(plane1.normal, n), 0);
            }
            if column + 1 < columns - 1 && materials[row * (columns - 1) + column + 1] != 0xff {
                let v = vertices(row, column + 1, false);
                let n = normalize(cross(v[1] - v[0], v[2] - v[0]));
                flags2 |= edge_flags(plane_separation(plane2, v[2]), dot(plane2.normal, n), 0);
            }

            flags[2 * cell] = flags1;
            flags[2 * cell + 1] = flags2;
        }
    }

    flags
}
