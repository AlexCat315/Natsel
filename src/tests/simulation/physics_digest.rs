//! Ported from Tests/NatselTests/Simulation/PhysicsDigest.swift.
//!
//! FNV-1a digest over body transforms and contact impulses. Shared by the collision simulation
//! tests, which compare digests between the scalar and wide paths.

use crate::base::constants as b3;
use crate::simulation::World;
use crate::math::types::{Pos, Vec3};

pub struct PhysicsDigest {
    pub value: u64,
}

impl Default for PhysicsDigest {
    fn default() -> Self {
        Self {
            value: 14_695_981_039_346_656_037,
        }
    }
}

impl PhysicsDigest {
    pub fn new() -> Self {
        Self::default()
    }

    /// `add(_ value: Pos)`
    pub fn add_pos(&mut self, value: Pos) {
        for scalar in [value.x, value.y, value.z] {
            let bits = scalar.to_bits();
            self.add_u32(bits as u32);
            self.add_u32((bits >> 32) as u32);
        }
    }

    /// `add(_ word: UInt32)`
    pub fn add_u32(&mut self, word: u32) {
        for byte in 0..4 {
            self.value ^= u64::from((word >> (8 * byte)) & 255);
            self.value = self.value.wrapping_mul(1_099_511_628_211);
        }
    }

    /// `add(_ value: Float)`
    pub fn add_float(&mut self, value: f32) {
        self.add_u32(value.to_bits());
    }

    /// `add(_ value: Vec3)`
    pub fn add_vec3(&mut self, value: Vec3) {
        self.add_float(value.x);
        self.add_float(value.y);
        self.add_float(value.z);
    }

    pub fn add_contacts(&mut self, world: &World) {
        let mut count: u32 = 0;
        for contact in world.contacts.iter() {
            if contact.contact_id == b3::NULL_INDEX {
                continue;
            }
            count += 1;
            self.add_u32(contact.flags);
            self.add_u32(contact.manifold_count as u32);
            self.add_float(contact.friction);
            self.add_float(contact.restitution);
            self.add_float(contact.rolling_resistance);
            self.add_vec3(contact.tangent_velocity);
            let manifolds = contact.manifolds();
            for manifold_index in 0..manifolds.count() {
                let manifold = manifolds.get(manifold_index);
                self.add_vec3(manifold.normal);
                self.add_u32(manifold.point_count as u32);
                self.add_vec3(manifold.friction_impulse);
                self.add_vec3(manifold.rolling_impulse);
                self.add_float(manifold.twist_impulse);
                for i in 0..(manifold.point_count as usize) {
                    let point = manifold.points[i];
                    self.add_vec3(point.anchor_a);
                    self.add_vec3(point.anchor_b);
                    self.add_float(point.separation);
                    self.add_float(point.normal_impulse);
                    self.add_float(point.total_normal_impulse);
                    self.add_u32(point.feature_id);
                    self.add_u32(point.triangle_index as u32);
                    self.add_u32(if point.persisted { 1 } else { 0 });
                }
            }
        }
        self.add_u32(count);
    }
}
