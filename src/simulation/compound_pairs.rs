//! Compound pair emission for the broad-phase gauntlet.
//!
//! Translated from `Sources/Natsel/Simulation/CompoundPairs.swift`.

use crate::collision::compound::Compound;
use crate::collision::compound::child_queries::query_compound;
use crate::containers::hash_set::shape_pair_key;
use crate::math::functions::{aabb_transform, invert_transform, to_relative_transform};
use crate::math::types::Pos;

// Foreign paths assumed until `broad_phase.rs` (`Simulation/BroadPhase.swift`) lands: the
// file-level `proxyTypeOf` / `proxyIdOfKey` helpers.
use crate::simulation::broad_phase::{proxy_id_of_key, proxy_type_of};
use crate::simulation::world_runtime::World;

impl World {
    pub(crate) fn emit_compound_pairs(
        &self,
        compound_id: i32,
        other_id: i32,
        compound: &Compound,
        keys: &mut Vec<u64>,
    ) {
        // Swift copied `shapes[Int(otherId)]` by value; only the proxy key is read, so a
        // borrowed shape is equivalent.
        let other = &self.shapes[other_id as usize];
        let proxy = other.proxy_key;
        let bounds = self.broad_phase.trees[proxy_type_of(proxy) as usize]
            .get_aabb(proxy_id_of_key(proxy));
        let transform = to_relative_transform(
            self.body_transform_quick(self.shapes[compound_id as usize].body_id),
            Pos::ZERO,
        );
        let local = aabb_transform(invert_transform(transform), bounds);
        let start = keys.len();
        query_compound(compound, local, |child| {
            let key = shape_pair_key(compound_id, other_id, child as i32);
            if !self.broad_phase.pair_set.contains_key(key) {
                keys.push(key);
            }
            true
        });
        if keys.len() > start && !self.should_create_pair(compound_id, other_id) {
            keys.truncate(start);
        }
    }
}
