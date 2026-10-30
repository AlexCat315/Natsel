//! Narrow-phase collide pass.
//!
//! Translated from `Sources/Natsel/Simulation/CollidePass.swift`.
//!
//! The pass runs over the awake contacts of one step, either serially or split across workers.
//! Each worker collides a disjoint slice of contact ids, writes only contacts inside that slice
//! and only its own state bit set and diagnostics slot, so the worker count cannot change the
//! result: the merged diagnostics and the state transitions are both taken in index order.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::contact_update`: the `World` method
//!   `update_contact(contact_id, local_center_a, xf_a, local_center_b, xf_b, is_fast, scratch,
//!   store) -> bool`, Swift's overload that takes the mesh scratch pointer plus the
//!   `ContactStore` (the scratch is passed as `&mut MeshContactScratch`, the Rust form of
//!   `UnsafeMutablePointer<MeshContactScratch>` used by `mesh_contact/*`).
//! - `crate::simulation::contact_creation`: `World::destroy_contact(contact_id, wake_bodies)`
//!   and `World::add_contact_to_graph` / `World::remove_contact_from_graph`.
//! - `crate::simulation::island`: `World::link_contact(contact_id)` and
//!   `World::unlink_contact(contact_id)`.

use std::thread;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::base::platform::b3_prefetch;
use crate::collision::internal_math::modified_cross;
use crate::containers::bit_set::BitSet;
use crate::math::functions::{
    abs_vec3, aabb_overlaps, conjugate, distance_squared, dot, dot_quat, inv_mul_quat,
    inv_mul_world_transforms, length_squared, make_matrix_from_quat, max_int, max_vec3, min_float,
    min_int, mul_mv, mul_quat, sub_pos,
};
use crate::math::types::{AABB, Pos, Vec3, WorldTransform};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::contact_payload::MeshContactScratch;
use crate::simulation::contact_store::ContactStore;
use crate::simulation::solver_integration::StepContext;
use crate::simulation::world_runtime::{BodyFlags, BodySim, Contact, SetType, World, remove_swap_array};
use crate::types::events::{ContactBeginTouchEvent, ContactEndTouchEvent};
use crate::types::ids::{ContactID, ShapeID};

/// Compact narrow-phase view of a body sim. Reading a full `BodySim` by value here copied several
/// hundred bytes twice per contact and showed up as the largest `memmove` source in the profile.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ContactBodySnapshot {
    pub transform: WorldTransform,
    pub local_center: Vec3,
    pub center: Pos,
    pub max_extent: Vec3,
    pub flags: u32,
}

/// Raw mirrors of every buffer the parallel collide workers touch.
///
/// Swift captured these with nested `withUnsafeBufferPointer` calls plus
/// `nonisolated(unsafe) let` bindings, because the workers call `&mut World` methods while the
/// buffers stay borrowed; Rust cannot express that aliasing through the borrow checker, so the
/// buffers cross the thread boundary as raw pointers and are rebuilt as slices per worker.
///
/// SAFETY: the world is locked for the whole step and only this stage runs concurrently, exactly
/// the situation Swift's `nonisolated(unsafe)` captured. Every field points at storage that outlives
/// the thread scope: `solver_sets`, `fat_aabbs` and `contacts` are not resized or moved during the
/// pass, `contact_ids` is the `contact_index_scratch` buffer that is only refilled at the start of
/// the next pass, and the per-worker `bits`/`recycled`/`sat_calls`/`sat_hits` arrays own their
/// elements until after the scope joins. Workers write only their own element of the per-worker
/// arrays, only their own `bits` slot, and only contacts inside their own disjoint id slice, so no
/// two threads ever touch the same memory — the same contract the Swift block relied on.
#[derive(Clone, Copy)]
struct CollideBuffers {
    world: *mut World,
    awake_sims: *const BodySim,
    awake_sims_count: usize,
    static_sims: *const BodySim,
    static_sims_count: usize,
    fat_aabbs: *const AABB,
    fat_aabbs_count: usize,
    contacts: *mut Contact,
    contacts_count: usize,
    contact_ids: *const i32,
    contact_ids_count: usize,
    bits: *mut BitSet,
    recycled: *mut i32,
    sat_calls: *mut i32,
    sat_hits: *mut i32,
}

// SAFETY: see the type-level note; `CollideBuffers` only ever crosses the thread boundary of the
// collide worker scope, whose Send contract is the one Swift already satisfied.
unsafe impl Send for CollideBuffers {}
unsafe impl Sync for CollideBuffers {}

impl World {
    /// Swift's `private static let parallelCollideMinCount = 512`.
    ///
    /// Below this contact count the dispatch cost dominates and the pass stays serial.
    const PARALLEL_COLLIDE_MIN_COUNT: usize = 512;

    /// Resolve the body sim referenced by an encoded contact body sim index.
    /// Resolve a cached contact body-sim encoding to the solver set and local index. Returning the
    /// location instead of the sim itself avoids copying the whole `BodySim` per contact, which the
    /// profile showed as the dominant cost in the collide pass.
    #[inline(always)]
    pub(crate) fn resolve_contact_body_sim_index(
        &self,
        encoded_index: i32,
        body_id: i32,
    ) -> (i32, i32) {
        if encoded_index >= 0 {
            return (SetType::AwakeSet.raw_value(), encoded_index);
        }

        if encoded_index < b3::NULL_INDEX {
            return (SetType::StaticSet.raw_value(), -encoded_index - 2);
        }

        let body = &self.bodies[body_id as usize];
        (body.set_index, body.local_index)
    }

    /// Read the handful of body-sim fields the narrow phase needs without copying the whole struct.
    #[inline(always)]
    pub(crate) fn contact_body_snapshot_at(
        &self,
        set: i32,
        local: i32,
        awake_sims: &[BodySim],
        static_sims: &[BodySim],
    ) -> ContactBodySnapshot {
        let index = local as usize;
        let sims: &[BodySim];
        if set == SetType::AwakeSet.raw_value() {
            sims = awake_sims;
        } else if set == SetType::StaticSet.raw_value() {
            sims = static_sims;
        } else {
            let sim = &self.solver_sets[set as usize].body_sims[index];
            return ContactBodySnapshot {
                transform: sim.transform,
                local_center: sim.local_center,
                center: sim.center,
                max_extent: sim.max_extent,
                flags: sim.flags,
            };
        }
        let sim = &sims[index];
        ContactBodySnapshot {
            transform: sim.transform,
            local_center: sim.local_center,
            center: sim.center,
            max_extent: sim.max_extent,
            flags: sim.flags,
        }
    }

    /// True if the encoded index refers to a static body.
    #[inline(always)]
    pub(crate) fn is_static_sim_index(&self, encoded_index: i32) -> bool {
        encoded_index < b3::NULL_INDEX
    }

    /// Update awake contacts and apply state transitions.
    pub(crate) fn collide(&mut self, context: &mut StepContext) {
        // Swift's `collide(_ context: inout StepContext)` does not read the step context; the
        // parameter is kept because `solver_integration.rs` calls `self.collide(&mut context)`.
        let _ = context;

        // Gather contacts from all graph colors, then the non-touching contacts. Reuse the list
        // across steps; rebuilding it allocated a new array every frame.
        self.contact_index_scratch.clear();
        for i in 0..b3::GRAPH_COLOR_COUNT as usize {
            let color = &self.constraint_graph.colors[i];
            self.contact_index_scratch
                .extend_from_slice(&color.convex_contacts);
            for spec in &color.contacts {
                self.contact_index_scratch.push(spec.contact_id);
            }
        }

        let awake_set_index = SetType::AwakeSet.raw_value() as usize;
        self.contact_index_scratch
            .extend_from_slice(&self.solver_sets[awake_set_index].contact_indices);

        // The early-out stays where Swift had it, ahead of the mesh scratch counter reset.
        if self.contact_index_scratch.is_empty() {
            return;
        }
        // Swift bound a copy-on-write *view* of the scratch (`let contactIndices =
        // contactIndexScratch`), so the pass reads a stable buffer while still calling `&mut self`
        // methods. `contact_index_scratch` is not written again until the next step, so a slice
        // over the same storage is that view.
        //
        // SAFETY: the buffer is only cleared and refilled at the top of the next `collide` call,
        // and nothing in this pass resizes or writes `contact_index_scratch`, so the elements stay
        // alive and unchanged for the whole pass.
        let contact_indices: &[i32] = unsafe {
            std::slice::from_raw_parts(
                self.contact_index_scratch.as_ptr(),
                self.contact_index_scratch.len(),
            )
        };

        self.mesh_contact_scratch.recycled_contacts = 0;
        self.mesh_contact_scratch.sat_calls = 0;
        self.mesh_contact_scratch.sat_cache_hits = 0;

        let contact_capacity = max_int(self.contacts.len() as i32, 8) as u32;
        let workers = max_int(1, min_int(self.worker_count, 8));
        let worker_count = workers as usize;

        // Each worker collides its own slice and writes only contacts inside that slice, so the
        // passes never touch the same element.
        if workers > 1 && contact_indices.len() >= Self::PARALLEL_COLLIDE_MIN_COUNT {
            let mut bits: Vec<BitSet> = vec![BitSet::new(); worker_count];
            for i in 0..worker_count {
                bits[i].set_bit_count_and_clear(contact_capacity);
            }

            let chunk = (contact_indices.len() + worker_count - 1) / worker_count;
            let mut recycled = vec![0i32; worker_count];
            let mut worker_sat_calls = vec![0i32; worker_count];
            let mut worker_sat_hits = vec![0i32; worker_count];

            let static_set_index = SetType::StaticSet.raw_value() as usize;

            let world_ptr = std::ptr::from_mut(&mut *self);
            let awake_sims_ptr = self.solver_sets[awake_set_index].body_sims.as_ptr();
            let awake_sims_len = self.solver_sets[awake_set_index].body_sims.len();
            let static_sims_ptr = self.solver_sets[static_set_index].body_sims.as_ptr();
            let static_sims_len = self.solver_sets[static_set_index].body_sims.len();
            let fat_ptr = self.fat_aabbs.as_ptr();
            let fat_len = self.fat_aabbs.len();
            let contact_ptr = self.contacts.as_mut_ptr();
            // Read the count here: inside the workers `contacts` is still borrowed for
            // writing by this closure.
            let contact_count = self.contacts.len();
            let index_ptr = contact_indices.as_ptr();
            let index_count = contact_indices.len();

            let buffers = CollideBuffers {
                world: world_ptr,
                awake_sims: awake_sims_ptr,
                awake_sims_count: awake_sims_len,
                static_sims: static_sims_ptr,
                static_sims_count: static_sims_len,
                fat_aabbs: fat_ptr,
                fat_aabbs_count: fat_len,
                contacts: contact_ptr,
                contacts_count: contact_count,
                contact_ids: index_ptr,
                contact_ids_count: index_count,
                bits: bits.as_mut_ptr(),
                recycled: recycled.as_mut_ptr(),
                sat_calls: worker_sat_calls.as_mut_ptr(),
                sat_hits: worker_sat_hits.as_mut_ptr(),
            };

            // Swift's `DispatchQueue.concurrentPerform(iterations: Int(workers))`. A worker panic
            // propagates when the scope joins, like a trap inside the Swift block would.
            thread::scope(|scope| {
                for w in 0..worker_count {
                    scope.spawn(move || {
                        // Capture the whole `CollideBuffers` rather than its individual fields:
                        // Rust's precise closure capture would otherwise move the raw pointers
                        // themselves into the closure, and those are not `Send` on their own even
                        // though `CollideBuffers` is.
                        let buffers = buffers;
                        let start = w * chunk;
                        let end = buffers.contact_ids_count.min(start + chunk);
                        if start >= end {
                            return;
                        }
                        let mut recycled_count: i32 = 0;

                        // SAFETY: every range below is live, initialized storage for the whole
                        // scope (see `CollideBuffers`), and this worker writes only `bits[w]`,
                        // `recycled[w]`, `sat_calls[w]`, `sat_hits[w]` and contacts whose ids fall
                        // in `start..end`. The early-out above leaves `bits[w]` pre-cleared, so the
                        // merged block counts stay equal.
                        let world: &mut World = unsafe { &mut *buffers.world };
                        let awake_sims: &[BodySim] = unsafe {
                            std::slice::from_raw_parts(buffers.awake_sims, buffers.awake_sims_count)
                        };
                        let static_sims: &[BodySim] = unsafe {
                            std::slice::from_raw_parts(
                                buffers.static_sims,
                                buffers.static_sims_count,
                            )
                        };
                        let fat: &[AABB] = unsafe {
                            std::slice::from_raw_parts(buffers.fat_aabbs, buffers.fat_aabbs_count)
                        };
                        let state_bits: &mut BitSet =
                            unsafe { &mut *buffers.bits.add(w) };
                        let mut local_store = ContactStore::from_buffer(unsafe {
                            std::slice::from_raw_parts_mut(buffers.contacts, buffers.contacts_count)
                        });
                        let mut local_scratch = MeshContactScratch::default();

                        for i in start..end {
                            let contact_id = unsafe { *buffers.contact_ids.add(i) };
                            world.collide_one_contact(
                                contact_id,
                                &mut *state_bits,
                                &mut local_store,
                                Some(&mut recycled_count),
                                &mut local_scratch,
                                awake_sims,
                                static_sims,
                                fat,
                            );
                        }

                        unsafe {
                            *buffers.recycled.add(w) = recycled_count;
                            *buffers.sat_calls.add(w) = local_scratch.sat_calls;
                            *buffers.sat_hits.add(w) = local_scratch.sat_cache_hits;
                        }
                    });
                }
            });

            let mut state_bits = BitSet::new();
            state_bits.set_bit_count_and_clear(contact_capacity);
            let mut total_recycled: i32 = 0;
            for i in 0..worker_count {
                state_bits.in_place_union(&bits[i]);
                total_recycled += recycled[i];
                self.mesh_contact_scratch.sat_calls += worker_sat_calls[i];
                self.mesh_contact_scratch.sat_cache_hits += worker_sat_hits[i];
            }
            self.task_contexts[0].recycled_contact_count = total_recycled;
            self.sat_call_count += self.mesh_contact_scratch.sat_calls;
            self.sat_cache_hit_count += self.mesh_contact_scratch.sat_cache_hits;
            self.collide_state_pass(&state_bits);
            state_bits.destroy();
            for i in 0..worker_count {
                bits[i].destroy();
            }
            return;
        }

        let mut state_bits = BitSet::with_bit_capacity(contact_capacity);
        state_bits.set_bit_count_and_clear(contact_capacity);

        // Borrow the body-sim and AABB buffers once for the whole pass. Indexing `solverSets[...]`
        // per contact copied the SolverSet and retained its five arrays twice per contact.
        //
        // SAFETY: `solver_sets` and `fat_aabbs` are not resized during the pass and the collide
        // pass only reads these buffers, which is the same guarantee Swift's
        // `withUnsafeBufferPointer` captures gave.
        let static_set_index = SetType::StaticSet.raw_value() as usize;
        let awake_sims: &[BodySim] = unsafe {
            std::slice::from_raw_parts(
                self.solver_sets[awake_set_index].body_sims.as_ptr(),
                self.solver_sets[awake_set_index].body_sims.len(),
            )
        };
        let static_sims: &[BodySim] = unsafe {
            std::slice::from_raw_parts(
                self.solver_sets[static_set_index].body_sims.as_ptr(),
                self.solver_sets[static_set_index].body_sims.len(),
            )
        };
        let fat: &[AABB] = unsafe {
            std::slice::from_raw_parts(self.fat_aabbs.as_ptr(), self.fat_aabbs.len())
        };
        self.collide_each_contact(
            contact_indices,
            &mut state_bits,
            awake_sims,
            static_sims,
            fat,
        );

        self.task_contexts[0].recycled_contact_count = self.mesh_contact_scratch.recycled_contacts;
        self.sat_call_count += self.mesh_contact_scratch.sat_calls;
        self.sat_cache_hit_count += self.mesh_contact_scratch.sat_cache_hits;
        self.collide_state_pass(&state_bits);
        state_bits.destroy();
    }

    /// Apply contact state transitions in contact id order. This phase is serial because it moves
    /// contacts between the graph and the awake set.
    fn collide_state_pass(&mut self, state_bits: &BitSet) {
        // Process contact state changes in contact id order. Iterating set bits avoids scanning
        // the entire contact table (tens of thousands of entries) every step.
        state_bits.for_each_set_bit(|contact_id| {
            // Read the flag word first: most slots are untouched on a given step, and copying the
            // whole contact for them was pure overhead.
            let flags = self.contacts[contact_id as usize].flags;
            if flags == 0 {
                return;
            }
            let mut contact = self.contacts[contact_id as usize];

            if (flags & ContactFlags::SIM_DISJOINT.0) != 0 {
                // Bounding boxes no longer overlap
                self.destroy_contact(contact_id, false);
            } else if (flags & ContactFlags::SIM_STARTED_TOUCHING.0) != 0 {
                b3_assert(contact.island_id == b3::NULL_INDEX);

                if (flags & ContactFlags::ENABLE_CONTACT_EVENTS.0) != 0 {
                    let shape_a = &self.shapes[contact.shape_id_a as usize];
                    let shape_b = &self.shapes[contact.shape_id_b as usize];
                    let mut event = ContactBeginTouchEvent::default();
                    event.shape_id_a =
                        ShapeID::new(shape_a.id + 1, self.world_id, shape_a.generation);
                    event.shape_id_b =
                        ShapeID::new(shape_b.id + 1, self.world_id, shape_b.generation);
                    event.contact_id =
                        ContactID::new(contact_id + 1, self.world_id, contact.generation);
                    self.contact_begin_events.push(event);
                }

                b3_assert(contact.manifold_count > 0);
                b3_assert(contact.set_index == SetType::AwakeSet.raw_value());

                // Link first because this wakes colliding bodies and ensures the body sims are in
                // the correct place.
                contact.flags &= !ContactFlags::SIM_STARTED_TOUCHING.0;
                contact.flags |= ContactFlags::TOUCHING.0;
                self.contacts[contact_id as usize] = contact;
                self.link_contact(contact_id);

                contact = self.contacts[contact_id as usize];

                // Make sure these didn't change
                b3_assert(contact.color_index == b3::NULL_INDEX);

                let old_local_index = contact.local_index;

                self.add_contact_to_graph(contact_id);
                self.remove_non_touching_contact(SetType::AwakeSet.raw_value(), old_local_index);
            } else if (flags & ContactFlags::SIM_STOPPED_TOUCHING.0) != 0 {
                if (flags & ContactFlags::ENABLE_CONTACT_EVENTS.0) != 0 {
                    let shape_a = &self.shapes[contact.shape_id_a as usize];
                    let shape_b = &self.shapes[contact.shape_id_b as usize];
                    let mut event = ContactEndTouchEvent::default();
                    event.shape_id_a =
                        ShapeID::new(shape_a.id + 1, self.world_id, shape_a.generation);
                    event.shape_id_b =
                        ShapeID::new(shape_b.id + 1, self.world_id, shape_b.generation);
                    event.contact_id =
                        ContactID::new(contact_id + 1, self.world_id, contact.generation);
                    self.contact_end_events.pending.push(event);
                }

                contact.flags &= !ContactFlags::SIM_STOPPED_TOUCHING.0;
                contact.flags &= !ContactFlags::TOUCHING.0;
                self.contacts[contact_id as usize] = contact;

                b3_assert(contact.manifold_count == 0);

                // Cache these here for the remove below
                let color_index = contact.color_index;
                let local_index = contact.local_index;

                self.unlink_contact(contact_id);

                contact = self.contacts[contact_id as usize];
                let body_id_a = contact.edges[0].body_id;
                let body_id_b = contact.edges[1].body_id;

                self.add_non_touching_contact(contact_id);

                let is_mesh_contact = (contact.flags & ContactFlags::SIM_MESH_CONTACT.0) != 0;
                self.remove_contact_from_graph(
                    body_id_a,
                    body_id_b,
                    color_index,
                    local_index,
                    is_mesh_contact,
                );
            }
        });
    }

    /// Update the awake contacts one by one, sharing one store across the whole pass.
    ///
    /// `ContactStore` holds a write reference to the world's contact array. Taking it per contact
    /// made the array multiply referenced for every contact, so each write copied the entire array;
    /// with tens of thousands of contacts that copy dominated the collide phase.
    fn collide_each_contact(
        &mut self,
        contact_ids: &[i32],
        state_bits: &mut BitSet,
        awake_sims: &[BodySim],
        static_sims: &[BodySim],
        fat_aabbs: &[AABB],
    ) {
        // Collide never resizes the contact array, so one stable buffer serves the whole pass and
        // each contact access skips the array uniqueness check.
        let contact_ptr = self.contacts.as_mut_ptr();
        let contact_count = self.contacts.len();
        let scratch_ptr = std::ptr::from_mut(&mut self.mesh_contact_scratch);

        let mut store = ContactStore::from_buffer(
            // SAFETY: `contacts` is uniquely writable for the whole pass and never resized by the
            // narrow phase, which is the invariant Swift's `withUnsafeMutableBufferPointer` gave
            // `ContactStore(buffer)`. Writing through the store while calling `&mut self` methods
            // is the unchecked-exclusivity aliasing MIGRATION.md allows raw pointers for; the
            // referenced storage does not move while the store lives.
            unsafe { std::slice::from_raw_parts_mut(contact_ptr, contact_count) },
        );

        // Prefetch the contact four iterations ahead.
        let prefetch_distance = 4usize;
        let base = contact_ptr.cast::<u8>();
        let stride = std::mem::size_of::<Contact>();
        for i in 0..contact_ids.len() {
            if i + prefetch_distance < contact_ids.len() {
                let p = unsafe { base.add(contact_ids[i + prefetch_distance] as usize * stride) };
                // SAFETY: `p` is inside the live `contacts` allocation and the three following
                // cache lines are prefetches of possibly-past-the-end addresses, which is exactly
                // what `b3Prefetch` on a raw pointer does: a prefetch never dereferences.
                unsafe {
                    b3_prefetch(p);
                    b3_prefetch(p.add(64));
                    b3_prefetch(p.add(128));
                    b3_prefetch(p.add(192));
                }
            }
            // SAFETY: the scratch is owned by `self` and not otherwise borrowed while this slice
            // of the pass runs; Swift passed the same `withUnsafeMutablePointer(to:)` result.
            let scratch: &mut MeshContactScratch = unsafe { &mut *scratch_ptr };
            self.collide_one_contact(
                contact_ids[i],
                &mut *state_bits,
                &mut store,
                None,
                scratch,
                awake_sims,
                static_sims,
                fat_aabbs,
            );
        }
    }

    fn collide_one_contact(
        &mut self,
        contact_id: i32,
        state_bits: &mut BitSet,
        store: &mut ContactStore<'_>,
        mut recycled_count: Option<&mut i32>,
        scratch: &mut MeshContactScratch,
        awake_sims: &[BodySim],
        static_sims: &[BodySim],
        fat_aabbs: &[AABB],
    ) {
        // Borrow the stored contact so recycling does not retain and then copy its manifolds.
        let update = store.with_contact(
            contact_id,
            |contact| -> Option<(ContactBodySnapshot, ContactBodySnapshot, bool)> {
                b3_assert(contact.contact_id == contact_id);

                let shape_id_a = contact.shape_id_a;
                let shape_id_b = contact.shape_id_b;

                // Do proxies still overlap?
                let overlap =
                    aabb_overlaps(fat_aabbs[shape_id_a as usize], fat_aabbs[shape_id_b as usize]);
                if overlap == false {
                    // This contact will be destroyed
                    contact.flags |= ContactFlags::SIM_DISJOINT.0;
                    contact.flags &= !ContactFlags::SIM_TOUCHING.0;
                    state_bits.set_bit(contact_id as u32);
                    return None;
                }

                // Update contact respecting shape/body order (A,B)
                let encoded_a = contact.encoded_body_sim_a;
                let encoded_b = contact.encoded_body_sim_b;
                let loc_a = self.resolve_contact_body_sim_index(
                    encoded_a,
                    contact.edges[0].body_id,
                );
                let loc_b = self.resolve_contact_body_sim_index(
                    encoded_b,
                    contact.edges[1].body_id,
                );
                let body_sim_a =
                    self.contact_body_snapshot_at(loc_a.0, loc_a.1, awake_sims, static_sims);
                let body_sim_b =
                    self.contact_body_snapshot_at(loc_b.0, loc_b.1, awake_sims, static_sims);

                let was_touching = (contact.flags & ContactFlags::SIM_TOUCHING.0) != 0;

                let transform_a = body_sim_a.transform;
                let transform_b = body_sim_b.transform;

                let is_fast =
                    ((body_sim_a.flags | body_sim_b.flags) & BodyFlags::IS_FAST.0) != 0;

                let recycle_distance = self.contact_recycle_distance;
                let speculative_distance = b3::speculative_distance();
                let recycle_distance_non_touching =
                    min_float(recycle_distance, speculative_distance);
                let recycle_tolerance = if was_touching {
                    recycle_distance
                } else {
                    recycle_distance_non_touching
                };

                // Contact recycling optimization.
                if is_fast == false
                    && recycle_distance > 0.0
                    && (contact.flags & ContactFlags::RELATIVE_TRANSFORM_VALID.0) != 0
                    && (contact.flags & ContactFlags::RECYCLE.0) != 0
                {
                    // The scalar part of inverse quaternion multiplication is the quaternion dot
                    // product.
                    let angle_a = dot_quat(transform_a.q, contact.cached_rotation_a);
                    let angle_b = dot_quat(transform_b.q, contact.cached_rotation_b);
                    let angular_distance = min_float(angle_a * angle_a, angle_b * angle_b);

                    let xf = inv_mul_world_transforms(transform_a, transform_b);
                    let xfc = contact.cached_relative_pose;
                    let max_extent_a = if self.is_static_sim_index(encoded_a) {
                        Vec3::ZERO
                    } else {
                        body_sim_a.max_extent
                    };
                    let max_extent_b = if self.is_static_sim_index(encoded_b) {
                        Vec3::ZERO
                    } else {
                        body_sim_b.max_extent
                    };
                    let max_extent = max_vec3(max_extent_a, max_extent_b);

                    let dist_squared = distance_squared(xf.p, xfc.p);

                    if angular_distance > b3::CONTACT_RECYCLE_ANGULAR_DISTANCE
                        && dist_squared < recycle_tolerance * recycle_tolerance
                    {
                        let distance = dist_squared.sqrt();
                        let slack = recycle_tolerance - distance;

                        let qr = inv_mul_quat(xfc.q, xf.q);
                        let arc = modified_cross(abs_vec3(qr.v), max_extent);

                        let arc_sq = 4.0 * length_squared(arc);
                        if arc_sq < slack * slack {
                            let dq_a = mul_quat(transform_a.q, conjugate(contact.cached_rotation_a));
                            let dq_b = mul_quat(transform_b.q, conjugate(contact.cached_rotation_b));
                            let matrix_a = make_matrix_from_quat(dq_a);
                            let matrix_b = make_matrix_from_quat(dq_b);

                            // Minimize round-off
                            let dc = sub_pos(body_sim_b.center, body_sim_a.center);

                            contact.manifolds().with_unsafe_mutable_buffer_pointer(|manifolds| {
                                for manifold_index in 0..manifolds.len() {
                                    let manifold = &mut manifolds[manifold_index];
                                    let normal = manifold.normal;
                                    for point_index in 0..manifold.point_count as usize {
                                        // Keep anchors but update separation, same as sub-stepping.
                                        let r_a = mul_mv(
                                            matrix_a,
                                            manifold.points[point_index].anchor_a,
                                        );
                                        let r_b = mul_mv(
                                            matrix_b,
                                            manifold.points[point_index].anchor_b,
                                        );
                                        let dp = dc + (r_b - r_a);
                                        manifold.points[point_index].separation =
                                            manifold.points[point_index].base_separation
                                                + dot(dp, normal);
                                        manifold.points[point_index].normal_velocity = 0.0;
                                        manifold.points[point_index].persisted = true;
                                    }
                                }
                            });

                            // Contact is recycled. This also skips updating other aspects of the
                            // contact such as material parameters.
                            if let Some(recycled_count) = recycled_count.as_mut() {
                                **recycled_count += 1;
                            } else {
                                scratch.recycled_contacts += 1;
                            }
                            return None;
                        }
                    }
                }

                // Caching for contact recycling.
                contact.cached_rotation_a = transform_a.q;
                contact.cached_rotation_b = transform_b.q;
                contact.cached_relative_pose = inv_mul_world_transforms(transform_a, transform_b);
                contact.flags |= ContactFlags::RELATIVE_TRANSFORM_VALID.0;

                Some((body_sim_a, body_sim_b, was_touching))
            },
        );
        let Some((body_sim_a, body_sim_b, was_touching)) = update else {
            return;
        };

        // This updates solid contacts
        let touching = self.update_contact(
            contact_id,
            body_sim_a.local_center,
            body_sim_a.transform,
            body_sim_b.local_center,
            body_sim_b.transform,
            ((body_sim_a.flags | body_sim_b.flags) & BodyFlags::IS_FAST.0) != 0,
            scratch,
            &mut *store,
        );

        store.with_contact(contact_id, |contact| {
            if touching && was_touching && (contact.flags & ContactFlags::SIM_MESH_CONTACT.0) != 0 {
                self.constraint_graph.colors[contact.color_index as usize]
                    .contacts[contact.local_index as usize]
                    .manifold_count = contact.manifold_count as u16;
            }

            if touching && was_touching == false {
                contact.flags |= ContactFlags::SIM_STARTED_TOUCHING.0;
                state_bits.set_bit(contact_id as u32);
            } else if touching == false && was_touching {
                contact.flags |= ContactFlags::SIM_STOPPED_TOUCHING.0;
                state_bits.set_bit(contact_id as u32);
            }

            if touching {
                contact.manifolds().with_unsafe_mutable_buffer_pointer(|manifolds| {
                    for manifold_index in 0..manifolds.len() {
                        let manifold = &mut manifolds[manifold_index];
                        for point_index in 0..manifold.point_count as usize {
                            // Cache separation
                            manifold.points[point_index].base_separation =
                                manifold.points[point_index].separation;
                        }
                    }
                });
            }
        });
    }

    /// Add a non-touching contact to the awake solver set.
    pub(crate) fn add_non_touching_contact(&mut self, contact_id: i32) {
        let mut contact = self.contacts[contact_id as usize];
        b3_assert(contact.set_index == SetType::AwakeSet.raw_value());
        let awake_set_index = SetType::AwakeSet.raw_value() as usize;
        contact.color_index = b3::NULL_INDEX;
        contact.local_index = self.solver_sets[awake_set_index].contact_indices.len() as i32;
        self.solver_sets[awake_set_index]
            .contact_indices
            .push(contact.contact_id);
        self.contacts[contact_id as usize] = contact;
    }

    /// Remove a non-touching contact from a solver set.
    pub(crate) fn remove_non_touching_contact(&mut self, set_index: i32, local_index: i32) {
        let moved_index = remove_swap_array(
            &mut self.solver_sets[set_index as usize].contact_indices,
            local_index as usize,
        );
        if moved_index != b3::NULL_INDEX {
            let moved_contact_index =
                self.solver_sets[set_index as usize].contact_indices[local_index as usize];
            let mut moved_contact = self.contacts[moved_contact_index as usize];
            b3_assert(moved_contact.set_index == set_index);
            b3_assert(moved_contact.color_index == b3::NULL_INDEX);
            b3_assert(moved_contact.local_index == moved_index);
            moved_contact.local_index = local_index;
            self.contacts[moved_contact_index as usize] = moved_contact;
        }
    }
}
