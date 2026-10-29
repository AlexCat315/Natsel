//! Manifold, separating axis, and local contact types.

use crate::math::types::Vec3;

/// A manifold point is a contact point belonging to a contact manifold.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ManifoldPoint {
    /// Location of the contact point relative to the bodyA center of mass in world space.
    pub anchor_a: Vec3,
    /// Location of the contact point relative to the bodyB center of mass in world space.
    pub anchor_b: Vec3,
    /// The separation of the contact point, negative if penetrating.
    pub separation: f32,
    /// The impulse along the manifold normal vector.
    pub normal_impulse: f32,
    /// The total normal impulse applied across sub-stepping and restitution.
    pub total_normal_impulse: f32,
    /// Relative normal velocity pre-solve. Negative when approaching.
    pub normal_velocity: f32,
    /// Cached separation used for contact recycling.
    pub base_separation: f32,
    /// Local point for matching. Uniquely identifies a contact point between two shapes.
    pub feature_id: u32,
    /// Triangle index if one of the shapes is a mesh or height field.
    pub triangle_index: i32,
    /// Did this contact point exist in the previous step?
    pub persisted: bool,
}

/// A contact manifold describes the contact points between colliding shapes.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Manifold {
    /// The manifold points. There may be 1 to 4 valid points.
    pub points: [ManifoldPoint; 4],
    /// The unit normal vector in world space, points from shape A to shape B.
    pub normal: Vec3,
    /// Central friction angular impulse (applied about the normal).
    pub twist_impulse: f32,
    /// Central friction linear impulse.
    pub friction_impulse: Vec3,
    /// Rolling resistance angular impulse.
    pub rolling_impulse: Vec3,
    /// The number of contact points, 0 to 4.
    pub point_count: i32,
}

impl Default for Manifold {
    fn default() -> Self {
        Self {
            points: [ManifoldPoint::default(); 4],
            normal: Vec3::ZERO,
            twist_impulse: 0.0,
            friction_impulse: Vec3::ZERO,
            rolling_impulse: Vec3::ZERO,
            point_count: 0,
        }
    }
}

/// Cached separating axis feature.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum SeparatingFeature {
    #[default]
    InvalidAxis = 0,
    BacksideAxis,
    FaceAxisA,
    FaceAxisB,
    EdgePairAxis,
    ClosestPointsAxis,
    ManualFaceAxisA,
    ManualFaceAxisB,
    ManualEdgePairAxis,
}

/// Cached triangle feature.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum TriangleFeature {
    #[default]
    FeatureNone = 0,
    FeatureTriangleFace,
    FeatureHullFace,
    FeatureEdge1,
    FeatureEdge2,
    FeatureEdge3,
    FeatureVertex1,
    FeatureVertex2,
    FeatureVertex3,
}

/// Separating axis test cache. Provides temporal acceleration of collision routines.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SATCache {
    /// The separation when the cache is populated. Negative for overlap.
    pub separation: f32,
    pub type_: SeparatingFeature,
    /// Index of the feature on shape A.
    pub index_a: u8,
    /// Index of the feature on shape B.
    pub index_b: u8,
    /// Was the cache re-used?
    pub hit: u8,
}

/// Contact points are always the result of two edges intersecting.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct FeaturePair {
    /// Incoming type (either edge on shape A or shape B).
    pub owner1: u8,
    /// Incoming edge index.
    pub index1: u8,
    /// Outgoing type.
    pub owner2: u8,
    /// Outgoing edge index.
    pub index2: u8,
}

/// A local manifold point and normal in frame A.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct LocalManifoldPoint {
    /// Local point in frame A.
    pub point: Vec3,
    /// The contact point separation. Negative for overlap.
    pub separation: f32,
    /// The feature pair for this point.
    pub pair: FeaturePair,
    /// The triangle index when colliding with a mesh or height field.
    pub triangle_index: i32,
}

/// A local manifold with no dynamic information. Used by collide functions.
#[derive(Clone, Copy, Debug)]
pub struct LocalManifold {
    /// Local normal in frame A.
    pub normal: Vec3,
    /// The triangle normal.
    pub triangle_normal: Vec3,
    /// The manifold points. From a caller-owned buffer.
    pub points: *mut LocalManifoldPoint,
    /// The number of manifold points.
    pub point_count: i32,
    /// The index of the triangle.
    pub triangle_index: i32,
    pub i1: i32,
    pub i2: i32,
    pub i3: i32,
    /// The squared distance of a sphere from a triangle.
    pub squared_distance: f32,
    /// The triangle feature involved.
    pub feature: TriangleFeature,
    /// Mesh edge flags.
    pub triangle_flags: i32,
}

unsafe impl Send for LocalManifold {}

impl Default for LocalManifold {
    fn default() -> Self {
        Self {
            normal: Vec3::ZERO,
            triangle_normal: Vec3::ZERO,
            points: std::ptr::null_mut(),
            point_count: 0,
            triangle_index: 0,
            i1: 0,
            i2: 0,
            i3: 0,
            squared_distance: 0.0,
            feature: TriangleFeature::FeatureNone,
            triangle_flags: 0,
        }
    }
}

impl LocalManifold {
    pub fn points(&self) -> &[LocalManifoldPoint] {
        if self.points.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(self.points, self.point_count as usize) }
        }
    }

    pub fn points_mut(&mut self) -> &mut [LocalManifoldPoint] {
        if self.points.is_null() {
            &mut []
        } else {
            unsafe { std::slice::from_raw_parts_mut(self.points, self.point_count as usize) }
        }
    }
}
