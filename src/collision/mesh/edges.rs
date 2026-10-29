//! Mesh edge adjacency flags.

use std::collections::HashMap;

use crate::collision::internal_math::signed_volume;
use crate::math::functions::*;
use crate::math::types::Vec3;
use crate::types::geometry::{MeshEdgeFlags, MeshTriangle};

struct AdjacencyEdge {
    triangle1: usize,
    triangle2: Option<usize>,
    count: u16,
    edge1: usize,
    edge2: usize,
}

/// Identify concave and coplanar edges.
pub fn identify_mesh_edges(vertices: &[Vec3], triangles: &[MeshTriangle]) -> Vec<u8> {
    let mut edges: Vec<AdjacencyEdge> = Vec::with_capacity(3 * triangles.len());
    let mut normals: Vec<Vec3> = Vec::with_capacity(triangles.len());

    for (i, triangle) in triangles.iter().enumerate() {
        let indices = [triangle.index1, triangle.index2, triangle.index3];
        for j in 0..3 {
            let a = indices[j];
            let b = indices[(j + 1) % 3];
            edges.push(AdjacencyEdge {
                triangle1: i,
                triangle2: None,
                count: 1,
                edge1: j,
                edge2: 0,
            });
            let _ = (a, b);
        }
        let v1 = vertices[triangle.index1 as usize];
        let v2 = vertices[triangle.index2 as usize];
        let v3 = vertices[triangle.index3 as usize];
        normals.push(normalize(cross(v2 - v1, v3 - v1)));
    }

    // Map undirected edge to the first occurrence.
    let mut map: HashMap<u64, usize> = HashMap::with_capacity(edges.len());
    for i in 0..triangles.len() {
        let triangle = triangles[i];
        let indices = [triangle.index1, triangle.index2, triangle.index3];
        for j in 0..3 {
            let a = indices[j] as u32;
            let b = indices[(j + 1) % 3] as u32;
            let k = (a.min(b) as u64) << 32 | a.max(b) as u64;
            if let Some(&other) = map.get(&k) {
                if edges[other].count == 1 {
                    edges[other].triangle2 = Some(i);
                    edges[other].edge2 = j;
                }
                edges[other].count = edges[other].count.saturating_add(1);
            } else {
                map.insert(k, 3 * i + j);
                edges[3 * i + j].triangle1 = i;
                edges[3 * i + j].edge1 = j;
            }
        }
    }

    let mut flags = vec![0u8; triangles.len()];
    for edge in &edges {
        let Some(triangle2) = edge.triangle2 else { continue };
        if edge.count != 2 {
            continue;
        }
        let a = triangles[edge.triangle1];
        let b = triangles[triangle2];
        let opposite = match edge.edge2 {
            0 => b.index3,
            1 => b.index1,
            _ => b.index2,
        };
        let volume = signed_volume(
            vertices[a.index1 as usize],
            vertices[a.index2 as usize],
            vertices[a.index3 as usize],
            vertices[opposite as usize],
        );
        let cos_angle = dot(normals[edge.triangle1], normals[triangle2]);
        let bit1 = 1u8 << edge.edge1;
        let bit2 = 1u8 << edge.edge2;
        if volume > 0.0 || cos_angle > 0.9962 {
            flags[edge.triangle1] |= bit1;
            flags[triangle2] |= bit2;
        }
        if volume < 0.0 || cos_angle > 0.9962 {
            flags[edge.triangle1] |= 0x10u8 << edge.edge1;
            flags[triangle2] |= 0x10u8 << edge.edge2;
        }
    }
    let _ = MeshEdgeFlags::CONCAVE_EDGE1;
    flags
}
