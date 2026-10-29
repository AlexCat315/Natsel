//! The broad-phase computes proxy pairs. This does not persist pairs; it reports potentially new
//! pairs for the client to consume.
//!
//! Translated from `Sources/Natsel/Simulation/BroadPhase.swift`.

use crate::base::core::{b3_assert, internal_assert};
use crate::collision::dynamic_tree::DynamicTree;
use crate::containers::hash_set::HashSet;
use crate::math::functions::{aabb_overlaps, max_int};
use crate::math::types::AABB;
use crate::types::body_def::BodyType;
use crate::types::world_def::Capacity;

/// Proxy type is stored in the low two bits of the proxy key.
#[inline(always)]
pub fn proxy_type_of(key: i32) -> BodyType {
    // Swift's `BodyType(rawValue: key & 3)!`. `key & 3` keeps the two low bits of the two's
    // complement word, so a negative key yields the same value as in Swift; only 3 is invalid.
    match key & 3 {
        0 => BodyType::StaticBody,
        1 => BodyType::KinematicBody,
        2 => BodyType::DynamicBody,
        _ => internal_assert("invalid proxy body type", file!(), line!()),
    }
}

#[inline(always)]
pub fn proxy_id_of_key(key: i32) -> i32 {
    // Swift's `>>` on Int32 is an arithmetic shift; Rust's `>>` on i32 is arithmetic too.
    key >> 2
}

#[inline(always)]
pub fn proxy_key(id: i32, proxy_body_type: BodyType) -> i32 {
    (id << 2) | proxy_body_type as i32
}

/// A broad-phase with one dynamic tree per body type.
///
/// Swift's `final class BroadPhase`; only `World` holds the reference, so it is an owned struct.
pub struct BroadPhase {
    /// One tree for each body type.
    pub trees: Vec<DynamicTree>,

    /// The moved siblings gathered from the dynamic body tree.
    pub moved_siblings: Vec<i32>,

    /// Tracks shape pairs that have a contact.
    pub pair_set: HashSet,
}

impl BroadPhase {
    /// `init(capacity:)`
    pub fn new(capacity: Capacity) -> Self {
        let pair_set = HashSet::with_capacity(max_int(32, 2 * capacity.contact_count));

        let static_capacity = max_int(16, capacity.static_shape_count);
        let kinematic_capacity: i32 = 16;
        let dynamic_capacity = max_int(16, capacity.dynamic_shape_count);

        let trees = vec![
            DynamicTree::new(static_capacity),
            DynamicTree::new(kinematic_capacity),
            DynamicTree::new(dynamic_capacity),
        ];

        Self {
            trees,
            moved_siblings: Vec::new(),
            pair_set,
        }
    }

    /// Create a proxy and return its key. Swift's `@discardableResult`, so no `#[must_use]`.
    pub fn create_proxy(
        &mut self,
        aabb: AABB,
        proxy_body_type: BodyType,
        category_bits: u64,
        shape_index: i32,
        force_pair_creation: bool,
    ) -> i32 {
        b3_assert(proxy_body_type as i32 >= 0 && proxy_body_type as i32 < BodyType::COUNT);

        let mark = proxy_body_type != BodyType::StaticBody || force_pair_creation;

        let proxy_id = self.trees[proxy_body_type as usize].create_proxy_internal(
            aabb,
            category_bits,
            // Swift's `UInt64(bitPattern: Int64(shapeIndex))` sign-extends; `as u64` does the same.
            shape_index as u64,
            mark,
        );
        proxy_key(proxy_id, proxy_body_type)
    }

    /// Destroy a proxy.
    pub fn destroy_proxy(&mut self, key: i32) {
        let type_ = proxy_type_of(key);
        let proxy_id = proxy_id_of_key(key);

        b3_assert(type_ as i32 >= 0 && type_ as i32 < BodyType::COUNT);
        self.trees[type_ as usize].destroy_proxy(proxy_id);
    }

    /// Move a proxy.
    pub fn move_proxy(&mut self, key: i32, aabb: AABB) {
        let type_ = proxy_type_of(key);
        let proxy_id = proxy_id_of_key(key);

        self.trees[type_ as usize].move_proxy_internal(proxy_id, aabb, true);
    }

    /// Get the shape index stored in a proxy.
    #[inline(always)]
    pub fn get_shape_index(&self, key: i32) -> i32 {
        let type_ = proxy_type_of(key);
        let proxy_id = proxy_id_of_key(key);

        // Swift's `Int32(truncatingIfNeeded:)` keeps the low 32 bits, like `as i32`.
        self.trees[type_ as usize].get_user_data(proxy_id) as i32
    }

    /// Do two proxies overlap?
    pub fn test_overlap(&self, key_a: i32, key_b: i32) -> bool {
        let type_a = proxy_type_of(key_a) as usize;
        let proxy_a = proxy_id_of_key(key_a);
        let type_b = proxy_type_of(key_b) as usize;
        let proxy_b = proxy_id_of_key(key_b);

        let aabb_a = self.trees[type_a].get_aabb(proxy_a);
        let aabb_b = self.trees[type_b].get_aabb(proxy_b);
        aabb_overlaps(aabb_a, aabb_b)
    }

    /// Mark a proxy as moved. Serial use case.
    pub fn mark_proxy_moved_serial(&mut self, key: i32) {
        let type_ = proxy_type_of(key);
        let proxy_id = proxy_id_of_key(key);
        self.trees[type_ as usize].mark_proxy_moved_serial(proxy_id);
    }

    /// Mark a proxy as moved with a new AABB.
    pub fn mark_proxy_moved(&mut self, key: i32, aabb: AABB) {
        let type_ = proxy_type_of(key);
        let proxy_id = proxy_id_of_key(key);
        self.trees[type_ as usize].mark_proxy_moved(proxy_id, aabb);
    }

    /// Validate the dynamic and kinematic trees.
    pub fn validate(&self) {
        self.trees[BodyType::DynamicBody as usize].validate();
        self.trees[BodyType::KinematicBody as usize].validate();
    }

    /// Validate that no proxy has a moved flag.
    pub fn validate_no_moved(&self) {
        for j in 0..BodyType::COUNT as usize {
            self.trees[j].validate_no_moved();
        }
    }

    /// Does any tree need a rebuild? Swift's computed `var needsRebuild`.
    pub fn needs_rebuild(&self) -> bool {
        self.trees[BodyType::StaticBody as usize].needs_rebuild()
            || self.trees[BodyType::KinematicBody as usize].needs_rebuild()
            || self.trees[BodyType::DynamicBody as usize].needs_rebuild()
    }
}
