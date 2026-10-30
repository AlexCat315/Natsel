//! Manifold clipping helpers.

use crate::base::core::{b3_assert, b3_validate};
use crate::collision::hull::Hull;
use crate::collision::internal_math::plane_separation;
use crate::math::functions::*;
use crate::math::types::{Plane, Vec3};
use crate::types::collision_types::{FeaturePair, SeparatingFeature};

pub const MAX_CLIP_POINTS: i32 = 64;
/// The single point feature pair.
pub const FEATURE_PAIR_SINGLE: FeaturePair = FeaturePair {
    owner1: 0,
    index1: 0,
    owner2: 0,
    index2: 0,
};

/// A separating axis candidate.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SeparatingAxis {
    pub normal: Vec3,
    pub separation: f32,
    pub index_a: i32,
    pub index_b: i32,
    pub kind: SeparatingFeature,
}

/// The result of the separating axis queries.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct AxisQuery {
    pub face_a: SeparatingAxis,
    pub face_b: SeparatingAxis,
    pub edge: SeparatingAxis,
    pub separated_feature: SeparatingFeature,
}

/// A vertex used during clipping.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ClipVertex {
    pub position: Vec3,
    pub separation: f32,
    pub pair: FeaturePair,
}

/// Which shape a feature belongs to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FeatureOwner {
    #[default]
    ShapeA = 0,
    ShapeB = 1,
}

/// Pack a feature pair into a 32-bit contact point id.
pub fn make_feature_id(pair: &FeaturePair) -> u32 {
    ((pair.owner1 as u32) << 24)
        | ((pair.index1 as u32) << 16)
        | ((pair.owner2 as u32) << 8)
        | pair.index2 as u32
}

/// Make a feature pair from an incoming and an outgoing edge.
pub fn make_feature_pair(
    owner1: FeatureOwner,
    index1: i32,
    owner2: FeatureOwner,
    index2: i32,
) -> FeaturePair {
    b3_assert(0 <= index1 && index1 <= u8::MAX as i32);
    b3_assert(0 <= index2 && index2 <= u8::MAX as i32);

    FeaturePair {
        index1: index1 as u8,
        owner1: owner1 as u8,
        index2: index2 as u8,
        owner2: owner2 as u8,
    }
}

/// Flip a feature pair.
pub fn flip_pair(pair: &FeaturePair) -> FeaturePair {
    b3_assert(pair.owner1 == 0 || pair.owner1 == 1);
    b3_assert(pair.owner2 == 0 || pair.owner2 == 1);

    FeaturePair {
        owner1: 1 - pair.owner2,
        owner2: 1 - pair.owner1,
        index1: pair.index2,
        index2: pair.index1,
    }
}

/// Pick the separating axis with the largest separation.
pub fn get_best_axis(query: &AxisQuery) -> SeparatingAxis {
    b3_validate(query.face_a.kind == SeparatingFeature::FaceAxisA);
    b3_validate(query.edge.kind == SeparatingFeature::EdgePairAxis);
    b3_validate(query.face_b.kind == SeparatingFeature::FaceAxisB);

    if query.face_a.separation > query.face_b.separation {
        if query.edge.separation > query.face_a.separation {
            return query.edge;
        }
        return query.face_a;
    }

    if query.edge.separation > query.face_b.separation {
        return query.edge;
    }
    query.face_b
}

/// Find the face on the other hull most anti-parallel to the reference normal, starting at the
/// supplied vertex.
pub fn find_incident_face(hull: &Hull, ref_normal: Vec3, vertex_index: i32) -> i32 {
    b3_assert(0 <= vertex_index && vertex_index < hull.vertex_count());

    let vertices = hull.vertices();
    let edges = hull.edges();
    let planes = hull.planes();
    let points = hull.points();

    let mut min_edge_index: i32 = -1;
    let mut min_edge_projection = f32::MAX;

    let vertex = vertices[vertex_index as usize];

    let mut edge_index = vertex.edge as i32;
    let start_edge_index = vertex.edge as i32;
    let edge_origin = points[edges[edge_index as usize].origin as usize];
    b3_assert(edges[edge_index as usize].origin as i32 == vertex_index);

    loop {
        let edge = edges[edge_index as usize];
        let twin = edges[edge.twin as usize];
        let twin_origin = points[twin.origin as usize];

        let axis = normalize(twin_origin - edge_origin);
        let edge_projection = abs_float(dot(axis, ref_normal));
        if edge_projection < min_edge_projection {
            min_edge_index = edge_index;
            min_edge_projection = edge_projection;
        }

        edge_index = twin.next as i32;
        b3_assert(edges[edge_index as usize].origin as i32 == vertex_index);

        if edge_index == start_edge_index {
            break;
        }
    }

    b3_assert(min_edge_index >= 0);

    let min_edge = edges[min_edge_index as usize];
    let min_face_index1 = min_edge.face;
    let min_plane1 = planes[min_face_index1 as usize];

    let min_twin = edges[min_edge.twin as usize];
    let min_face_index2 = min_twin.face;
    let min_plane2 = planes[min_face_index2 as usize];

    if dot(min_plane1.normal, ref_normal) < dot(min_plane2.normal, ref_normal) {
        min_face_index1 as i32
    } else {
        min_face_index2 as i32
    }
}

/// Sutherland-Hodgman style clipping of a polygon against a single plane.
pub fn clip_polygon(
    out: &mut [ClipVertex],
    polygon: &[ClipVertex],
    count: i32,
    clip_plane: Plane,
    edge: i32,
    ref_plane: Plane,
) -> i32 {
    b3_assert(count >= 3);

    let mut vertex1 = polygon[(count - 1) as usize];
    let mut distance1 = plane_separation(clip_plane, vertex1.position);
    let mut out_count: i32 = 0;

    for index in 0..count as usize {
        let vertex2 = polygon[index];
        let distance2 = plane_separation(clip_plane, vertex2.position);

        if distance1 <= 0.0 && distance2 <= 0.0 {
            out[out_count as usize] = vertex2;
            out_count += 1;
        } else if distance1 <= 0.0 && distance2 > 0.0 {
            let fraction = distance1 / (distance1 - distance2);
            let position = mul_add(vertex1.position, fraction, vertex2.position - vertex1.position);

            let mut vertex = ClipVertex {
                position,
                separation: plane_separation(ref_plane, position),
                pair: vertex2.pair,
            };
            vertex.pair.owner2 = FeatureOwner::ShapeA as u8;
            vertex.pair.index2 = edge as u8;
            out[out_count as usize] = vertex;
            out_count += 1;
        } else if distance2 <= 0.0 && distance1 > 0.0 {
            let fraction = distance1 / (distance1 - distance2);
            let position = mul_add(vertex1.position, fraction, vertex2.position - vertex1.position);

            let mut vertex = ClipVertex {
                position,
                separation: plane_separation(ref_plane, position),
                pair: vertex1.pair,
            };
            vertex.pair.owner1 = FeatureOwner::ShapeA as u8;
            vertex.pair.index1 = edge as u8;
            out[out_count as usize] = vertex;
            out_count += 1;

            out[out_count as usize] = vertex2;
            out_count += 1;
        }

        vertex1 = vertex2;
        distance1 = distance2;
    }

    b3_validate(validate_polygon(&out[..out_count as usize]));
    out_count
}

/// Verify that incoming and outgoing edges match.
pub fn validate_polygon(polygon: &[ClipVertex]) -> bool {
    if polygon.is_empty() {
        return true;
    }

    let mut vertex1 = polygon[polygon.len() - 1];
    for vertex2 in polygon.iter() {
        if vertex1.pair.owner2 != vertex2.pair.owner1 {
            return false;
        }
        if vertex1.pair.index2 != vertex2.pair.index1 {
            return false;
        }
        vertex1 = *vertex2;
    }
    true
}
