//! Mesh triangle access and candidate traversal.

use crate::collision::internal_math::Triangle;
use crate::math::types::Vec3;
use crate::types::geometry::{Mesh, MeshTriangle};

/// Build a scaled, winding-corrected triangle.
pub fn mesh_triangle(mesh: &Mesh, index: i32) -> Triangle {
    let data = mesh.data.as_ref().expect("mesh has data");
    let source = data.triangles()[index as usize];
    let flags = data.edge_flags()[index as usize] as i32;
    let vertices = data.vertices();

    let mut triangle = Triangle::default();
    triangle.vertices[0] = mesh.scale * vertices[source.index1 as usize];
    triangle.i1 = source.index1;
    if mesh.scale.x * mesh.scale.y * mesh.scale.z < 0.0 {
        triangle.i2 = source.index3;
        triangle.i3 = source.index2;
        triangle.flags = (flags >> 4) & 7;
    } else {
        triangle.i2 = source.index2;
        triangle.i3 = source.index3;
        triangle.flags = flags;
    }
    triangle.vertices[1] = mesh.scale * vertices[triangle.i2 as usize];
    triangle.vertices[2] = mesh.scale * vertices[triangle.i3 as usize];
    triangle
}

/// Mesh query bounds in local mesh space.
#[derive(Clone, Copy, Debug)]
pub struct MeshQueryBounds {
    pub lower: Vec3,
    pub upper: Vec3,
}

impl MeshQueryBounds {
    pub fn new(bounds: crate::math::types::AABB, scale: Vec3) -> Self {
        let inverse = Vec3::new(1.0 / scale.x, 1.0 / scale.y, 1.0 / scale.z);
        let a = inverse * bounds.lower_bound;
        let b = inverse * bounds.upper_bound;
        Self {
            lower: crate::math::functions::min_vec3(a, b),
            upper: crate::math::functions::max_vec3(a, b),
        }
    }

    pub fn center(&self) -> Vec3 {
        0.5 * (self.lower + self.upper)
    }

    pub fn extent(&self) -> Vec3 {
        self.upper - self.center()
    }
}

/// Visit candidate triangles overlapping the query bounds. The callback returns false to stop.
pub fn visit_mesh_candidates<F>(data: &super::storage::TriangleMesh, bounds: MeshQueryBounds, mut callback: F)
where
    F: FnMut(usize, &MeshTriangle) -> bool,
{
    let nodes = data.nodes();
    let triangles = data.triangles();
    let mut stack: Vec<usize> = vec![0];
    while let Some(index) = stack.pop() {
        let node = nodes[index];
        if !crate::collision::aabb::bounds_overlap(
            node.lower_bound,
            node.upper_bound,
            bounds.lower,
            bounds.upper,
        ) {
            continue;
        }
        if node.is_leaf() {
            let offset = node.triangle_offset as usize;
            for triangle_index in offset..offset + node.leaf_triangle_count() {
                if !callback(triangle_index, &triangles[triangle_index]) {
                    return;
                }
            }
        } else {
            stack.push(index + node.child_offset());
            stack.push(index + 1);
        }
    }
}
