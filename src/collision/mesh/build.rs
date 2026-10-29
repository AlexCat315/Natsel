//! Mesh construction options, validation, and preparation.

use crate::base::constants as b3;
use crate::collision::internal_math::{bounds3_empty, get_by_index, major_axis};
use crate::math::functions::*;
use crate::math::types::{AABB, Vec3};
use crate::types::geometry::{MeshTriangle, MeshNode};

use super::edges::identify_mesh_edges;
use super::tree_builder::{MeshPrimitive, MeshTreeBuild};
use super::welding::weld_mesh_vertices;

/// Mesh construction choices. A zero welding tolerance preserves all input vertices.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MeshBuildOptions {
    pub weld_tolerance: f32,
    pub use_median_split: bool,
    pub identify_edges: bool,
    pub clockwise_winding: bool,
}

pub struct PreparedMesh {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<MeshTriangle>,
    pub nodes: Vec<MeshNode>,
    pub materials: Vec<u8>,
    pub flags: Vec<u8>,
    pub bounds: AABB,
    pub surface_area: f32,
    pub tree_height: usize,
    pub degenerate_count: usize,
    pub material_count: usize,
}

pub fn prepare_mesh(
    source: &[Vec3],
    source_indices: &[i32],
    materials: Option<&[u8]>,
    options: MeshBuildOptions,
) -> Option<PreparedMesh> {
    if source.len() < 3 || source_indices.is_empty() || source_indices.len() % 3 != 0 {
        return None;
    }
    if let Some(m) = materials {
        if m.len() != source_indices.len() / 3 {
            return None;
        }
    }
    if source_indices.iter().any(|&i| i < 0 || i as usize >= source.len()) {
        return None;
    }

    let (vertices, indices) = if options.weld_tolerance > 0.0 {
        weld_mesh_vertices(source, source_indices, options.weld_tolerance)
    } else {
        (source.to_vec(), source_indices.to_vec())
    };

    let mut bounds = bounds3_empty();
    let mut primitives: Vec<MeshPrimitive> = Vec::new();
    let mut degenerate_count = 0usize;
    let mut surface_area: f32 = 0.0;
    let mut material_count = 1usize;

    let min_area = 0.01 * b3::linear_slop() * b3::linear_slop();

    for index in 0..indices.len() / 3 {
        let i1 = indices[3 * index];
        let i2 = indices[3 * index + 1];
        let i3 = indices[3 * index + 2];
        let v1 = vertices[i1 as usize];
        let v2 = vertices[i2 as usize];
        let v3 = vertices[i3 as usize];
        let normal = cross(v2 - v1, v3 - v1);
        let area = 0.5 * length(normal);
        if area < min_area {
            if i1 != i2 && i1 != i3 && i2 != i3 {
                degenerate_count += 1;
            }
            continue;
        }
        surface_area += area;
        let box_ = AABB::new(min_vec3(v1, min_vec3(v2, v3)), max_vec3(v1, max_vec3(v2, v3)));
        primitives.push(MeshPrimitive {
            bounds: box_,
            center: aabb_center(box_),
            triangle_index: index,
        });
        if let Some(m) = materials {
            material_count = material_count.max(m[index] as usize + 1);
        }
        bounds = aabb_union(bounds, box_);
    }

    if primitives.is_empty() || !is_sane_aabb(bounds) {
        return None;
    }

    let mut tree = MeshTreeBuild::new(&mut primitives);
    if !tree.fits_traversal_stack() {
        return None;
    }

    let mut triangles = Vec::with_capacity(primitives.len());
    let mut sorted_materials = Vec::with_capacity(primitives.len());
    for primitive in &primitives {
        let offset = 3 * primitive.triangle_index;
        triangles.push(MeshTriangle {
            index1: indices[offset],
            index2: indices[offset + if options.clockwise_winding { 2 } else { 1 }],
            index3: indices[offset + if options.clockwise_winding { 1 } else { 2 }],
        });
        sorted_materials.push(materials.map(|m| m[primitive.triangle_index]).unwrap_or(0));
    }

    let flags = if options.identify_edges {
        identify_mesh_edges(&vertices, &triangles)
    } else {
        vec![0u8; triangles.len()]
    };

    Some(PreparedMesh {
        vertices,
        triangles,
        nodes: tree.nodes,
        materials: sorted_materials,
        flags,
        bounds,
        surface_area,
        tree_height: tree.height,
        degenerate_count,
        material_count,
    })
}

/// Deterministic serialization of the mesh content for hashing.
pub fn mesh_hash_inputs(
    vertices: &[Vec3],
    triangles: &[MeshTriangle],
    nodes: &[MeshNode],
    materials: &[u8],
    flags: &[u8],
) -> Vec<u8> {
    let mut out = Vec::with_capacity(
        vertices.len() * 12 + triangles.len() * 12 + nodes.len() * 28 + materials.len() + flags.len(),
    );
    for v in vertices {
        out.extend_from_slice(&v.x.to_bits().to_le_bytes());
        out.extend_from_slice(&v.y.to_bits().to_le_bytes());
        out.extend_from_slice(&v.z.to_bits().to_le_bytes());
    }
    for t in triangles {
        out.extend_from_slice(&t.index1.to_le_bytes());
        out.extend_from_slice(&t.index2.to_le_bytes());
        out.extend_from_slice(&t.index3.to_le_bytes());
    }
    for n in nodes {
        out.extend_from_slice(&n.lower_bound.x.to_bits().to_le_bytes());
        out.extend_from_slice(&n.lower_bound.y.to_bits().to_le_bytes());
        out.extend_from_slice(&n.lower_bound.z.to_bits().to_le_bytes());
        out.extend_from_slice(&n.data.to_le_bytes());
        out.extend_from_slice(&n.upper_bound.x.to_bits().to_le_bytes());
        out.extend_from_slice(&n.upper_bound.y.to_bits().to_le_bytes());
        out.extend_from_slice(&n.upper_bound.z.to_bits().to_le_bytes());
        out.extend_from_slice(&n.triangle_offset.to_le_bytes());
    }
    out.extend_from_slice(materials);
    out.extend_from_slice(flags);
    out
}

/// Deterministic median split helper shared by the tree builder.
pub fn split_index(primitives: &mut [MeshPrimitive], range_start: usize, range_end: usize) -> (usize, usize) {
    let count = range_end - range_start;
    let mut lower = primitives[range_start].center;
    let mut upper = lower;
    for p in &primitives[range_start + 1..range_end] {
        lower = min_vec3(lower, p.center);
        upper = max_vec3(upper, p.center);
    }
    let d = upper - lower;
    let axis = if d.x >= d.y && d.x >= d.z {
        0
    } else if d.y >= d.z {
        1
    } else {
        2
    };
    primitives[range_start..range_end].sort_by(|a, b| {
        let ca = get_by_index(a.center, axis);
        let cb = get_by_index(b.center, axis);
        if ca < cb {
            core::cmp::Ordering::Less
        } else if ca > cb {
            core::cmp::Ordering::Greater
        } else {
            a.triangle_index.cmp(&b.triangle_index)
        }
    });
    let _ = major_axis(d);
    let index = count / 2;
    (axis, index.max(1).min(count - 1))
}
