//! Immutable, quantized terrain shared by height-field shapes.

use crate::base::constants as b3;
use crate::base::hash::hash64_non_zero;
use crate::collision::geometry_storage::GeometryStorage;
use crate::math::functions::*;
use crate::math::types::{AABB, Vec3};
use crate::types::geometry::HeightFieldDef;

use super::adjacency::height_field_adjacency;
use super::compressed_layout::{align_height_field_bytes, HeightFieldData};

/// Immutable, quantized terrain shared by height-field shapes.
#[derive(Clone)]
pub struct HeightField {
    storage: GeometryStorage,
}

impl HeightField {
    pub fn new(definition: &HeightFieldDef) -> Option<Self> {
        let columns = definition.count_x as usize;
        let rows = definition.count_z as usize;
        if columns < 2
            || rows < 2
            || definition.heights.len() != columns * rows
            || (!definition.material_indices.is_empty()
                && definition.material_indices.len() != (columns - 1) * (rows - 1))
            || definition.scale.x <= 0.0
            || definition.scale.y <= 0.0
            || definition.scale.z <= 0.0
            || !is_valid_vec3(definition.scale)
            || !definition.global_minimum_height.is_finite()
            || !definition.global_maximum_height.is_finite()
            || definition.global_minimum_height > definition.global_maximum_height
            || definition.heights.iter().any(|h| !h.is_finite())
        {
            return None;
        }

        let height_count = columns * rows;
        let cell_count = (columns - 1) * (rows - 1);
        let heights_offset = align_height_field_bytes(std::mem::size_of::<HeightFieldData>());
        let material_offset = heights_offset + align_height_field_bytes(2 * height_count);
        let flags_offset = material_offset + align_height_field_bytes(cell_count);
        let size = flags_offset + align_height_field_bytes(2 * cell_count);
        let byte_count = size as i32;

        let storage = GeometryStorage::new(size)?;
        let base = storage.base();

        let height =
            max_float(definition.global_maximum_height - definition.global_minimum_height, b3::linear_slop());
        let height_scale = height / u16::MAX as f32;

        let compressed = unsafe { base.add(heights_offset) as *mut u16 };
        let mut lower = definition.global_maximum_height;
        let mut upper = definition.global_minimum_height;
        let inverse: f32 = 1.0 / height_scale;
        for i in 0..height_count {
            let clamped = clamp_float(
                definition.heights[i],
                definition.global_minimum_height,
                definition.global_maximum_height,
            );
            let scaled = (clamped - definition.global_minimum_height) * inverse;
            unsafe {
                compressed.add(i).write(min_float(scaled, u16::MAX as f32) as u16);
            }
            lower = min_float(lower, clamped);
            upper = max_float(upper, clamped);
        }

        let mut decompressed = Vec::with_capacity(height_count);
        for i in 0..height_count {
            let value = unsafe { compressed.add(i).read() };
            decompressed.push(definition.global_minimum_height + height_scale * value as f32);
        }

        let materials: Vec<u8> = if definition.material_indices.is_empty() {
            vec![0u8; cell_count]
        } else {
            definition.material_indices.clone()
        };
        if cell_count > 0 {
            unsafe {
                std::ptr::copy_nonoverlapping(
                    materials.as_ptr(),
                    base.add(material_offset),
                    cell_count,
                );
            }
        }

        let aabb = AABB::new(
            Vec3::new(0.0, definition.scale.y * lower, 0.0),
            Vec3::new(
                definition.scale.x * (columns - 1) as f32,
                definition.scale.y * upper,
                definition.scale.z * (rows - 1) as f32,
            ),
        );

        let flags = height_field_adjacency(
            &decompressed,
            &materials,
            columns,
            rows,
            definition.scale,
        );
        if !flags.is_empty() {
            unsafe {
                std::ptr::copy_nonoverlapping(flags.as_ptr(), base.add(flags_offset), flags.len());
            }
        }

        let header = HeightFieldData {
            version: b3::HEIGHT_FIELD_VERSION,
            hash: 0,
            byte_count,
            aabb,
            min_height: definition.global_minimum_height,
            max_height: definition.global_maximum_height,
            height_scale,
            scale: definition.scale,
            column_count: definition.count_x,
            row_count: definition.count_z,
            heights_offset: heights_offset as i32,
            material_offset: material_offset as i32,
            flags_offset: flags_offset as i32,
            clockwise: definition.clockwise_winding as u8,
            padding: [0; 7],
        };
        unsafe {
            (base as *mut HeightFieldData).write(header);
        }

        let hash_input = unsafe { std::slice::from_raw_parts(base as *const u8, size) };
        let hash = hash64_non_zero(hash_input);
        unsafe {
            (*base.cast::<HeightFieldData>()).hash = hash;
        }

        Some(Self { storage })
    }

    pub fn header(&self) -> HeightFieldData {
        unsafe { std::ptr::read_unaligned(self.storage.base() as *const HeightFieldData) }
    }

    pub fn byte_count(&self) -> i32 {
        self.storage.byte_count() as i32
    }

    pub fn compressed_heights(&self) -> &[u16] {
        let header = self.header();
        unsafe {
            self.storage.slice_ref::<u16>(
                header.heights_offset as usize,
                (self.column_count() * self.row_count()) as usize,
            )
        }
    }

    pub fn material_indices(&self) -> &[u8] {
        let header = self.header();
        unsafe {
            self.storage.slice_ref::<u8>(
                header.material_offset as usize,
                ((self.column_count() - 1) * (self.row_count() - 1)) as usize,
            )
        }
    }

    pub fn edge_flags(&self) -> &[u8] {
        let header = self.header();
        unsafe {
            self.storage.slice_ref::<u8>(
                header.flags_offset as usize,
                2 * (self.column_count() - 1) * (self.row_count() - 1),
            )
        }
    }

    pub fn column_count(&self) -> usize {
        self.header().column_count as usize
    }

    pub fn row_count(&self) -> usize {
        self.header().row_count as usize
    }

    pub fn bounds(&self) -> AABB {
        self.header().aabb
    }

    pub fn scale(&self) -> Vec3 {
        self.header().scale
    }

    pub fn hash(&self) -> u64 {
        self.header().hash
    }
}

impl core::fmt::Debug for HeightField {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("HeightField")
            .field("column_count", &self.column_count())
            .field("row_count", &self.row_count())
            .field("hash", &self.hash())
            .finish()
    }
}
