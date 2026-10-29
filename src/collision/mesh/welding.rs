//! Spatial vertex welding for mesh construction.

use std::collections::HashMap;

use crate::math::types::Vec3;

struct SpatialHash {
    cell_size: f32,
    tolerance: f32,
    cells: HashMap<u64, Vec<usize>>,
}

fn key(x: i32, y: i32, z: i32) -> u64 {
    let mut k: u64 = 0;
    for coordinate in [x, y, z] {
        k ^= (coordinate as i64 as u64).wrapping_add(0x9e37_79b9).wrapping_add(k << 6).wrapping_add(k >> 2);
    }
    k
}

impl SpatialHash {
    fn new(tolerance: f32) -> Self {
        Self {
            cell_size: 2.0 * tolerance,
            tolerance,
            cells: HashMap::new(),
        }
    }

    fn find_duplicate(&mut self, vertices: &[Vec3], current: usize) -> Option<usize> {
        let vertex = vertices[current];
        let x = (vertex.x / self.cell_size).floor() as i32;
        let y = (vertex.y / self.cell_size).floor() as i32;
        let z = (vertex.z / self.cell_size).floor() as i32;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(indices) = self.cells.get(&key(x + dx, y + dy, z + dz)) {
                        for &index in indices {
                            let other = vertices[index];
                            if abs_diff(vertex.x, other.x) <= self.tolerance
                                && abs_diff(vertex.y, other.y) <= self.tolerance
                                && abs_diff(vertex.z, other.z) <= self.tolerance
                            {
                                return Some(index);
                            }
                        }
                    }
                }
            }
        }
        self.cells.entry(key(x, y, z)).or_default().push(current);
        None
    }
}

fn abs_diff(a: f32, b: f32) -> f32 {
    (a - b).abs()
}

/// Weld nearby vertices and remap indices.
pub fn weld_mesh_vertices(vertices: &[Vec3], indices: &[i32], tolerance: f32) -> (Vec<Vec3>, Vec<i32>) {
    assert!(tolerance > 0.0);
    let mut hash = SpatialHash::new(tolerance);
    let mut mapping = vec![0i32; vertices.len()];
    let mut unique: Vec<Vec3> = Vec::with_capacity(vertices.len());

    for i in 0..vertices.len() {
        match hash.find_duplicate(vertices, i) {
            Some(duplicate) => mapping[i] = mapping[duplicate],
            None => {
                mapping[i] = unique.len() as i32;
                unique.push(vertices[i]);
            }
        }
    }

    let remapped = indices.iter().map(|&i| mapping[i as usize]).collect();
    (unique, remapped)
}
