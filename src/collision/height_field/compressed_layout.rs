//! Internal header for an owned height-field allocation.

use crate::math::types::{AABB, Vec3};

/// The height-field header. The version is always first.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct HeightFieldData {
    pub version: u64,
    pub hash: u64,
    pub byte_count: i32,
    pub aabb: AABB,
    pub min_height: f32,
    pub max_height: f32,
    pub height_scale: f32,
    pub scale: Vec3,
    pub column_count: i32,
    pub row_count: i32,
    pub heights_offset: i32,
    pub material_offset: i32,
    pub flags_offset: i32,
    pub clockwise: u8,
    pub padding: [u8; 7],
}

/// Align a byte count up to 8 bytes.
#[inline]
pub fn align_height_field_bytes(count: usize) -> usize {
    (count + 7) & !7
}
