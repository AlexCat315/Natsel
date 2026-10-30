//! Ported from Tests/NatselTests/Collision/CollisionDigest.swift.
//!
//! FNV-style digest over the collision query results. Shared by the compound, height-field, mesh
//! and mover repeatability suites, so the mix order is part of the determinism contract.

use crate::math::types::{Pos, Vec3};
use crate::types::queries::{CastOutput, PlaneResult};

pub struct CollisionDigest {
    pub value: u64,
}

impl Default for CollisionDigest {
    fn default() -> Self {
        Self {
            value: 14695981039346656037,
        }
    }
}

impl CollisionDigest {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_pos(&mut self, value: Pos) {
        self.add_f32(value.x as f32);
        self.add_f32(value.y as f32);
        self.add_f32(value.z as f32);
    }

    pub fn add_u32(&mut self, word: u32) {
        for byte in 0..4 {
            self.value ^= ((word >> (8 * byte)) & 255) as u64;
            self.value = self.value.wrapping_mul(1099511628211);
        }
    }

    pub fn add_f32(&mut self, scalar: f32) {
        self.add_u32(scalar.to_bits());
    }

    pub fn add_vec3(&mut self, vector: Vec3) {
        self.add_f32(vector.x);
        self.add_f32(vector.y);
        self.add_f32(vector.z);
    }

    pub fn add_cast_output(&mut self, output: &CastOutput) {
        self.add_vec3(output.normal);
        self.add_vec3(output.point);
        self.add_f32(output.fraction);
        self.add_u32(output.iterations as u32);
        self.add_u32(output.triangle_index as u32);
        self.add_u32(output.child_index as u32);
        self.add_u32(output.material_index as u32);
        self.add_u32(if output.hit { 1 } else { 0 });
    }

    pub fn add_plane_result(&mut self, plane: &PlaneResult) {
        self.add_vec3(plane.plane.normal);
        self.add_f32(plane.plane.offset);
        self.add_vec3(plane.point);
        self.add_u32(plane.triangle_index as u32);
        self.add_u32(plane.child_index as u32);
        self.add_u32(plane.material_index as u32);
    }
}
