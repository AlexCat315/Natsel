//! Soft body simulation with small step XPBD. Each world step is split into `iteration_count`
//! substeps; every substep integrates, projects the constraints once, resolves collisions, and
//! derives velocities from the position change.
//!
//! Soft bodies run after the rigid bodies are finalized. Collision planes therefore sit at the rigid
//! bodies' end-of-step poses, and the impulses exchanged with dynamic bodies change their velocities
//! for the next step. Each soft body is independent, so they are simulated in parallel when the
//! world has workers; rigid body velocity changes are merged afterwards in soft body order.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodySolver.swift`.
//!
//! Foreign vocabulary from the sibling modules, verified against the files that have landed
//! (`soft_body_runtime.rs`, `soft_body_settings.rs`, `soft_body_batches.rs`,
//! `soft_body_contacts.rs`, `soft_body_collision.rs`); only `soft_body_edge_collision.rs` is still
//! pending, so its path alone is still an assumption:
//! - `crate::soft_body::soft_body_runtime::SoftBodyRecord` (`pub(crate)`): fields `id: i32`,
//!   `origin: Pos`, `bounds: AABB`, `positions: Vec<Vec3>`, `previous_positions: Vec<Vec3>`,
//!   `velocities: Vec<Vec3>`, `inv_masses: Vec<f32>`, `has_contact: Vec<bool>`,
//!   `colliders: Vec<SoftBodyCollider>`, `vertex_planes: Vec<SoftVertexPlane>`,
//!   `vertex_plane_counts: Vec<u8>`, `settings: SoftBodySettings`,
//!   `skinned_positions: Vec<Vec3>`, `skinned_normals: Vec<Vec3>`, `has_skin_pose: bool`,
//!   `iteration_count: i32`, `linear_damping: f32`, `max_linear_velocity: f32`,
//!   `pressure: f32`, `gravity_scale: f32`, `filter: Filter`, `awake: bool`, `sleep_time: f32`,
//!   `enable_sleep: bool`, `sleep_threshold: f32`, `enable_self_collision: bool`,
//!   `collide_with_soft_bodies: bool`, `two_way_coupling: bool`, `face_sides: HashMap<u64, f32>`;
//!   the associated const `SoftBodyRecord::PLANES_PER_VERTEX: usize` (Swift's `static let
//!   planesPerVertex`); the methods `vertex_count() -> usize`, `rebase(reference: Pos)`,
//!   `update_skin()`, `update_bounds()`.
//! - `crate::soft_body::soft_body_runtime::SoftVertexPlane`: `Copy` with `normal: Vec3`,
//!   `offset: f32`, `collider: i32`.
//! - `crate::soft_body::soft_body_runtime::SoftBodyCollider`: `Copy` with `body_index: i32`,
//!   `awake_index: i32`, `is_dynamic: bool`, `inv_mass: f32`, `inv_inertia: Matrix3`,
//!   `center: Pos`, `linear_velocity`/`angular_velocity`/`original_linear_velocity`/
//!   `original_angular_velocity: Vec3`, `friction: f32`, `restitution: f32`, `updated: bool`,
//!   `contact_count: i32`, `displacement`/`rotation: Vec3`, and
//!   `point_velocity(r: Vec3) -> Vec3`.
//! - `crate::soft_body::soft_body_runtime::World::wake_soft_body(index: usize)` and the record
//!   helper `World::soft_body_records()` (`softBodies` is `Vec<SoftBodyRecord>`,
//!   `soft_body_id_pool: IDPool`, `worker_count: i32`, `gravity: Vec3`, `contact_speed: f32`).
//! - `crate::soft_body::soft_body_settings::SoftBodySettings`: `faces: Vec<SoftBodyFace>`,
//!   `skinned: Vec<SoftBodySkinned>`, `long_range_attachments: Vec<SoftBodyLRA>`; `SoftBodyFace`
//!   with `vertex: SoftBodyU32x3` read as `.x`/`.y`/`.z` (`u32`), `SoftBodyLRA` with
//!   `anchor`/`vertex: u32`, `max_distance: f32`, `SoftBodySkinned` with `vertex: u32`,
//!   `max_distance`/`back_stop_distance`/`back_stop_radius: f32`.
//! - `crate::soft_body::soft_body_batches`: the record methods `solve_bend_batches`,
//!   `solve_volume_batches`, `solve_edge_batches`, all `(&self, x: &mut [Vec3], w: &[f32],
//!   inv_h2: f32)`.
//! - `crate::soft_body::soft_body_edge_collision` (pending): the record method
//!   `apply_edge_collisions(&self, x: &mut [Vec3], x0: &mut [Vec3], w: &[f32],
//!   contact: &mut [bool], colliders: &mut [SoftBodyCollider], h: f32, last: bool)`.
//! - `crate::soft_body::soft_body_collision::World::detect_soft_body_collisions(&self, record:
//!   &mut SoftBodyRecord, dt: f32)`.
//! - `crate::soft_body::soft_body_contacts`: `SoftContact`, `SoftEdgeContact`, and `SoftContactBody`
//!   (`Copy`, the three Swift buffer pointers as `x: *mut Vec3`, `x0: *mut Vec3`,
//!   `w: *const f32`), the record method `contact_thickness(&self) -> f32`, the `World` method
//!   `detect_soft_contacts(&self, records: *mut SoftBodyRecord, group: &[usize], dt: f32,
//!   contacts: &mut Vec<SoftContact>, edge_contacts: &mut Vec<SoftEdgeContact>, parallel: bool,
//!   detection_workers: usize)`, and the free functions `solve_soft_contacts` /
//!   `solve_soft_edge_contacts(contacts: &[..Contact], bodies: &[SoftContactBody],
//!   max_recovery: f32)`.
//! - `max_speed` is a computed property of the record in Swift (`SoftBodySolver.swift`), so it is
//!   defined as the method `SoftBodyRecord::max_speed()` here; `soft_body_contacts.rs` reads it as
//!   `record.max_speed` (a field). Reconcile at the build phase - the method is the faithful
//!   translation, the field does not exist in Swift.
//!
//! Adaptations (behaviour identical):
//! - Swift read `let first = records[group[0]]` and `let settings = self.settings` as copy-on-write
//!   struct copies; only the fields those expressions actually feed (`bounds`, `origin`, and the
//!   three settings arrays) are read here, so no copy of the whole record is made.
//! - Swift's nested `forEachBody` closure captures `group` and `parallel`; at file scope those
//!   become parameters, and the two captured pointers (`records`, `self`) travel in the `Copy`
//!   `GroupContext` the body closures capture, exactly like `CollideBuffers` does for the parallel
//!   collide pass.
//! - Swift's `withUnsafeMutableBufferPointer` nests over the local arrays it swapped out of the
//!   record are plain indexed slices after the same swap; the swap-back that Swift's `defer` did at
//!   scope exit is written out at the end of the two functions, which have no other exit path.
//! - `Int(maxInt(1, minInt(workerCount, 8)))`, group indices and vertex indices are `usize`; the
//!   iteration counts and indices Swift kept as `Int32` stay `i32`.

use std::collections::HashMap;
use std::thread;

use crate::base::constants as b3;
use crate::math::functions::{
    aabb_center, aabb_inflate, atan2_approx, cross, dot, length, length_squared, max_float,
    max_int, min_float, min_int, mul_add, mul_mv, mul_sub, offset_aabb, offset_pos, sub_pos,
};
use crate::math::spatial_pairs::overlapping_bounds_pairs;
use crate::math::types::Vec3;
use crate::math::world_aabb::WorldAABB;
use crate::simulation::shape_runtime::should_shapes_collide;
use crate::simulation::world_runtime::{SetType, World};
use crate::soft_body::soft_body_contacts::{
    SoftContact, SoftContactBody, SoftEdgeContact, solve_soft_contacts, solve_soft_edge_contacts,
};
use crate::soft_body::soft_body_runtime::{SoftBodyCollider, SoftBodyRecord};
use crate::soft_body::soft_body_settings::{SoftBodyLRA, SoftBodySkinned};
use crate::types::body_def::BodyType;

/// The two pointers Swift captured with `nonisolated(unsafe)` inside the soft body step: the
/// world's record storage and the world itself. Groups are simulated concurrently and each body
/// only ever touches its own record, so no two threads write the same element.
///
/// SAFETY: `records` addresses the world's `soft_bodies` storage, which is not resized or moved for
/// the whole step (`step_soft_bodies` only mutates records and solver-set body states), and `world`
/// is the same world those records belong to. Per-body work reads world state and writes only the
/// record at its own index, which is the contract Swift's captured `self` plus
/// `nonisolated(unsafe)` record pointer relied on.
#[derive(Clone, Copy)]
struct GroupContext {
    records: *mut SoftBodyRecord,
    world: *mut World,
}

// SAFETY: see the type-level note; a `GroupContext` only crosses the worker scope of one soft body
// group, whose members are disjoint records.
unsafe impl Send for GroupContext {}
unsafe impl Sync for GroupContext {}

/// Run per body work, on the workers when allowed. Bodies only touch their own records.
///
/// Swift's nested `forEachBody` inside `simulateSoftBodyGroup`; the captures become parameters.
fn for_each_body<F>(parallel: bool, group: &[usize], body: F)
where
    F: Fn(usize, usize) + Sync,
{
    if parallel {
        // Swift's `DispatchQueue.concurrentPerform(iterations: list.count) { k in body(k, list[k]) }`.
        thread::scope(|scope| {
            for k in 0..group.len() {
                let index = group[k];
                scope.spawn(move || body(k, index));
            }
        });
    } else {
        for (k, &index) in group.iter().enumerate() {
            body(k, index);
        }
    }
}

/// Connected-component root search with path halving, in the caller's `parent` array.
///
/// Swift's nested `find(_:)`; the captured `parent` becomes a parameter.
fn union_find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

impl World {
    pub(crate) fn step_soft_bodies(&mut self, dt: f32) {
        if !(self.soft_body_id_pool.id_count() > 0 && dt > 0.0) {
            return;
        }

        self.wake_touched_soft_bodies();
        self.wake_soft_bodies_touched_by_soft_bodies(dt);

        let mut active: Vec<usize> = Vec::new();
        for i in 0..self.soft_bodies.len() {
            if self.soft_bodies[i].id != b3::NULL_INDEX && self.soft_bodies[i].awake {
                active.push(i);
            }
        }
        if active.is_empty() {
            return;
        }

        let groups = self.soft_contact_groups(&active, dt);
        let gravity = self.gravity;
        let workers = max_int(1, min_int(self.worker_count, 8)) as usize;
        let parallel_work: usize =
            active
                .iter()
                .fold(0usize, |acc, &i| acc + self.soft_bodies[i].vertex_count());

        // Lone bodies run in parallel; bodies in contact run together, each substep's per body work
        // spread over the workers.
        let singles: Vec<&[usize]> = groups
            .iter()
            .filter(|group| group.len() == 1)
            .map(|group| group.as_slice())
            .collect();
        let clusters: Vec<&[usize]> = groups
            .iter()
            .filter(|group| group.len() > 1)
            .map(|group| group.as_slice())
            .collect();

        let records = self.soft_bodies.as_mut_ptr();
        let context = GroupContext {
            records,
            world: std::ptr::from_mut(&mut *self),
        };
        if workers > 1 && singles.len() > 1 && parallel_work >= 512 {
            thread::scope(|scope| {
                for k in 0..singles.len() {
                    let group = singles[k];
                    scope.spawn(move || {
                        // SAFETY: one single-body group per iteration, so this thread is the only
                        // writer of the record at `group[0]`; the world is only read apart from
                        // that record (see `GroupContext`).
                        let world = unsafe { &mut *context.world };
                        world.simulate_soft_body_group(
                            context.records, group, dt, gravity, false, 1,
                        );
                    });
                }
            });
        } else {
            for &group in &singles {
                let detection_workers = if singles.len() == 1 {
                    // SAFETY: `group[0]` is a live record index and this serial branch runs before
                    // any worker touches the record storage; only the vertex count is read.
                    if unsafe { &*records.add(group[0]) }.vertex_count() >= 512 {
                        workers
                    } else {
                        1
                    }
                } else {
                    1
                };
                // SAFETY: `group[0]` is a live record index; this is the serial branch, so no other
                // thread holds a reference into the record storage.
                self.simulate_soft_body_group(
                    records,
                    group,
                    dt,
                    gravity,
                    false,
                    detection_workers,
                );
            }
        }
        for &group in &clusters {
            let work = group.iter().fold(0usize, |acc, &i| {
                // SAFETY: live record storage, read-only here; the group's work is dispatched
                // inside `simulate_soft_body_group` below.
                acc + unsafe { &*records.add(i) }.vertex_count()
            });
            self.simulate_soft_body_group(
                records,
                group,
                dt,
                gravity,
                workers > 1 && work >= 512,
                1,
            );
        }

        // Merge rigid body velocity changes in a fixed order.
        for &i in &active {
            self.apply_soft_body_impulses(i);
            self.soft_bodies[i].update_sleep(dt);
        }
    }

    /// Simulate soft bodies that may touch each other substep by substep, projecting their contacts
    /// between the constraints and the rigid body collisions. Runs on any thread; touches only the
    /// group's records.
    pub(crate) fn simulate_soft_body_group(
        &mut self,
        records: *mut SoftBodyRecord,
        group: &[usize],
        dt: f32,
        gravity: Vec3,
        parallel: bool,
        detection_workers: usize,
    ) {
        let context = GroupContext {
            records,
            world: std::ptr::from_mut(&mut *self),
        };

        // SAFETY: `group[0]` addresses a live record of this step's storage; only `bounds` and
        // `origin` are read, which Swift's record copy did as well.
        let first_bounds = unsafe { &*records.add(group[0]) }.bounds;
        // SAFETY: as above, for the record's origin.
        let first_origin = unsafe { &*records.add(group[0]) }.origin;
        let local_center = aabb_center(first_bounds);
        let rebase_distance = 1024.0 * b3::length_units_per_meter();
        let reference = if length_squared(local_center) > rebase_distance * rebase_distance {
            offset_pos(first_origin, local_center)
        } else {
            first_origin
        };
        for &index in group {
            // SAFETY: live record storage, and this loop is sequential.
            unsafe { &mut *records.add(index) }.rebase(reference);
        }
        for_each_body(parallel, group, |_k, index| {
            // SAFETY: every body writes only its own record; the world reference is used for the
            // read-only broad-phase and shape lookups inside the detection pass, as in Swift.
            let world = unsafe { &mut *context.world };
            let record = unsafe { &mut *context.records.add(index) };
            world.detect_soft_body_collisions(record, dt);
        });

        let mut contacts: Vec<SoftContact> = Vec::new();
        let mut edge_contacts: Vec<SoftEdgeContact> = Vec::new();
        // SAFETY: live record storage; the parallel dispatch below has not started yet.
        let needs_contacts =
            group.len() > 1 || unsafe { &*records.add(group[0]) }.enable_self_collision;
        if needs_contacts {
            self.detect_soft_contacts(
                records,
                group,
                dt,
                &mut contacts,
                &mut edge_contacts,
                parallel,
                detection_workers,
            );
        } else {
            // SAFETY: live record storage, sequential here.
            unsafe { &mut *records.add(group[0]) }
                .face_sides
                .clear();
        }

        if contacts.is_empty() && edge_contacts.is_empty() {
            for_each_body(parallel, group, |_k, index| {
                // SAFETY: this body's record is only written by this iteration.
                let record = unsafe { &mut *context.records.add(index) };
                record.simulate(dt, gravity);
                record.finish_step();
            });
            return;
        }

        // Bodies in contact substep together, at the largest iteration count among them.
        let iterations = group.iter().fold(1i32, |acc, &i| {
            // SAFETY: live record storage at every `group` index for the whole step, read-only here.
            max_int(acc, unsafe { &*records.add(i) }.iteration_count)
        });
        let mut substeps: Vec<SoftBodySubstep> = Vec::new();
        for &index in group {
            // SAFETY: live record storage, sequential here; `begin_step` mutates only this record.
            let substep = unsafe { &mut *records.add(index) }.begin_step(dt, gravity, iterations);
            substeps.push(substep);
        }

        // Arrays keep their storage for the whole step.
        let mut views: Vec<SoftContactBody> = Vec::new();
        for &index in group {
            // SAFETY: live record storage, and the views are only used for the group's own records;
            // Swift built the same three buffer pointers off `records + index`.
            let record = unsafe { &mut *records.add(index) };
            views.push(SoftContactBody {
                x: record.positions.as_mut_ptr(),
                x0: record.previous_positions.as_mut_ptr(),
                w: record.inv_masses.as_ptr(),
            });
        }

        let count = substeps[0].count;
        for iteration in 0..count {
            let last = iteration == count - 1;
            for_each_body(parallel, group, |k, index| {
                // SAFETY: this body's record is only written by this iteration; `substeps` is
                // read-only here and lives for the whole loop.
                let record = unsafe { &mut *context.records.add(index) };
                record.solve_substep(substeps[k]);
            });
            let max_recovery = self.contact_speed * substeps[0].h;
            solve_soft_contacts(&contacts, &views, max_recovery);
            solve_soft_edge_contacts(&edge_contacts, &views, max_recovery);
            for_each_body(parallel, group, |k, index| {
                // SAFETY: as for the constraint pass above.
                let record = unsafe { &mut *context.records.add(index) };
                record.collide_substep(substeps[k], last);
            });
        }
        for_each_body(parallel, group, |_k, index| {
            // SAFETY: as above; `finish_step` only rewrites this record's bounds.
            let record = unsafe { &mut *context.records.add(index) };
            record.finish_step();
        });
    }

    /// Connected components of soft bodies whose swept bounds overlap, in index order.
    fn soft_contact_groups(&self, active: &[usize], dt: f32) -> Vec<Vec<usize>> {
        let mut parent: Vec<usize> = (0..active.len()).collect();
        let boxes: Vec<WorldAABB> = active
            .iter()
            .map(|&i| {
                let record = &self.soft_bodies[i];
                WorldAABB::from_local_bounds(
                    aabb_inflate(
                        record.bounds,
                        record.max_speed() * dt + record.contact_thickness(),
                    ),
                    record.origin,
                )
            })
            .collect();
        for pair in overlapping_bounds_pairs(&boxes).pairs {
            let a = pair.a;
            let b = pair.b;
            if !(self.soft_bodies[active[a]].collide_with_soft_bodies
                && self.soft_bodies[active[b]].collide_with_soft_bodies
                && should_shapes_collide(
                    &self.soft_bodies[active[a]].filter,
                    &self.soft_bodies[active[b]].filter,
                ))
            {
                continue;
            }
            let ra = union_find(&mut parent, a);
            let rb = union_find(&mut parent, b);
            if ra != rb {
                parent[ra.max(rb)] = ra.min(rb);
            }
        }
        let mut groups: Vec<Vec<usize>> = Vec::new();
        let mut group_of_root: HashMap<usize, usize> = HashMap::new();
        for k in 0..active.len() {
            let root = union_find(&mut parent, k);
            if let Some(&g) = group_of_root.get(&root) {
                groups[g].push(active[k]);
            } else {
                group_of_root.insert(root, groups.len());
                groups.push(vec![active[k]]);
            }
        }
        groups
    }

    /// A sleeping soft body wakes when a moving soft body reaches it.
    fn wake_soft_bodies_touched_by_soft_bodies(&mut self, dt: f32) {
        let ids: Vec<usize> = (0..self.soft_bodies.len())
            .filter(|&i| {
                self.soft_bodies[i].id != b3::NULL_INDEX
                    && self.soft_bodies[i].collide_with_soft_bodies
            })
            .collect();
        let boxes: Vec<WorldAABB> = ids
            .iter()
            .map(|&i| {
                let record = &self.soft_bodies[i];
                let margin =
                    record.contact_thickness() + if record.awake { record.max_speed() * dt } else { 0.0 };
                WorldAABB::from_local_bounds(aabb_inflate(record.bounds, margin), record.origin)
            })
            .collect();
        for pair in overlapping_bounds_pairs(&boxes).pairs {
            for &(i, j) in &[(ids[pair.a], ids[pair.b]), (ids[pair.b], ids[pair.a])] {
                if !(self.soft_bodies[i].awake == false
                    && self.soft_bodies[j].awake
                    && self.soft_bodies[j].max_speed() > self.soft_bodies[j].sleep_threshold
                    && should_shapes_collide(
                        &self.soft_bodies[i].filter,
                        &self.soft_bodies[j].filter,
                    ))
                {
                    continue;
                }
                self.wake_soft_body(i);
            }
        }
    }

    /// Sleeping soft bodies wake when a moving rigid body enters their bounds.
    fn wake_touched_soft_bodies(&mut self) {
        for i in 0..self.soft_bodies.len() {
            if !(self.soft_bodies[i].id != b3::NULL_INDEX && self.soft_bodies[i].awake == false) {
                continue;
            }
            let bounds = offset_aabb(
                aabb_inflate(self.soft_bodies[i].bounds, b3::speculative_distance()),
                self.soft_bodies[i].origin,
            );
            let mut wake = false;
            for tree in self.broad_phase.trees[1..].iter() {
                let _ = tree.query(
                    bounds,
                    self.soft_bodies[i].filter.mask_bits,
                    false,
                    |_proxy_id, data| {
                        // Swift read the shape and body structs by value (copy-on-write structs);
                        // only the fields the test needs are read here.
                        let shape_index = data as usize;
                        let body_index = self.shapes[shape_index].body_id as usize;
                        let set_index = self.bodies[body_index].set_index;
                        let sleep_velocity = self.bodies[body_index].sleep_velocity;
                        let sleep_threshold = self.bodies[body_index].sleep_threshold;
                        if set_index == SetType::AwakeSet.raw_value()
                            && sleep_velocity > sleep_threshold
                            && should_shapes_collide(
                                &self.soft_bodies[i].filter,
                                &self.shapes[shape_index].filter,
                            )
                        {
                            wake = true;
                            return false;
                        }
                        true
                    },
                );
                if wake {
                    break;
                }
            }
            if wake {
                self.wake_soft_body(i);
            }
        }
    }

    /// Apply the velocity changes accumulated on dynamic colliders to the rigid bodies.
    fn apply_soft_body_impulses(&mut self, index: usize) {
        let awake = SetType::AwakeSet.raw_value() as usize;
        let collider_count = self.soft_bodies[index].colliders.len();
        for c in 0..collider_count {
            let collider = self.soft_bodies[index].colliders[c];
            let body_index = collider.body_index as usize;
            // Swift copied the whole `Body` struct; only these fields are read from it.
            let set_index = self.bodies[body_index].set_index;
            let local = self.bodies[body_index].local_index as usize;
            let body_type = self.bodies[body_index].type_;
            let body_sleep_threshold = self.bodies[body_index].sleep_threshold;
            if collider.updated
                && collider.awake_index != b3::NULL_INDEX
                && set_index == SetType::AwakeSet.raw_value()
            {
                let mut state = self.solver_sets[awake].body_states[local];
                state.linear_velocity += collider.linear_velocity - collider.original_linear_velocity;
                state.angular_velocity +=
                    collider.angular_velocity - collider.original_angular_velocity;
                self.solver_sets[awake].body_states[local] = state;
            } else if collider.contact_count > 0
                && body_type == BodyType::DynamicBody
                && set_index >= SetType::FirstSleepingSet.raw_value()
                && self.soft_bodies[index].max_speed() > body_sleep_threshold
                && self.soft_bodies[index].two_way_coupling
            {
                // A moving soft body landing on a sleeping body wakes it; it responds next step.
                let _ = self.wake_body(collider.body_index);
            }
        }
    }
}

// MARK: - XPBD

/// Constants of one substep.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SoftBodySubstep {
    pub count: usize,
    pub h: f32,
    pub inv_h2: f32,
    pub damping: f32,
    pub max_speed_squared: f32,
    pub g: Vec3,
    pub restitution_threshold: f32,
}

impl SoftBodySubstep {
    #[inline]
    pub const fn new() -> Self {
        Self {
            count: 1,
            h: 0.0,
            inv_h2: 0.0,
            damping: 1.0,
            max_speed_squared: 0.0,
            g: Vec3::ZERO,
            restitution_threshold: 0.0,
        }
    }
}

impl Default for SoftBodySubstep {
    /// Swift's member defaults: `count = 1` and `damping = 1`, not the all-zeroes derive.
    #[inline(always)]
    fn default() -> Self {
        SoftBodySubstep::new()
    }
}

impl SoftBodyRecord {
    /// Largest vertex speed after the last step.
    pub(crate) fn max_speed(&self) -> f32 {
        let mut m: f32 = 0.0;
        for i in 0..self.velocities.len() {
            m = max_float(m, length_squared(self.velocities[i]));
        }
        m.sqrt()
    }

    pub(crate) fn simulate(&mut self, dt: f32, gravity: Vec3) {
        let substep = self.begin_step(dt, gravity, self.iteration_count);
        for iteration in 0..substep.count {
            self.solve_substep(substep);
            self.collide_substep(substep, iteration == substep.count - 1);
        }
    }

    /// Per step constants and the skinning update. `iterations` may exceed the body's own count
    /// when it substeps together with other soft bodies.
    pub(crate) fn begin_step(
        &mut self,
        dt: f32,
        gravity: Vec3,
        iterations: i32,
    ) -> SoftBodySubstep {
        // Kinematic skinned vertices follow the skin.
        if self.has_skin_pose && self.settings.skinned.is_empty() == false {
            self.update_skin();
            for (k, s) in self.settings.skinned.iter().enumerate() {
                if self.inv_masses[s.vertex as usize] != 0.0 {
                    continue;
                }
                self.positions[s.vertex as usize] = self.skinned_positions[k];
            }
        }
        for i in 0..self.colliders.len() {
            self.colliders[i].contact_count = 0;
            self.colliders[i].displacement = Vec3::default();
            self.colliders[i].rotation = Vec3::default();
        }

        let mut substep = SoftBodySubstep::new();
        substep.count = max_int(1, iterations) as usize;
        substep.h = dt / substep.count as f32;
        substep.inv_h2 = 1.0 / (substep.h * substep.h);
        substep.damping = max_float(0.0, 1.0 - self.linear_damping * substep.h);
        substep.max_speed_squared = self.max_linear_velocity * self.max_linear_velocity;
        substep.g = self.gravity_scale * gravity;
        substep.restitution_threshold = (-2.0 * length(gravity)) * substep.h;
        substep
    }

    /// Pressure, integration, and the constraints of one substep.
    pub(crate) fn solve_substep(&mut self, substep: SoftBodySubstep) {
        let h = substep.h;
        let damping = substep.damping;
        let max_speed_squared = substep.max_speed_squared;
        let g = substep.g;

        // Move the mutable state out so the buffers are uniquely referenced and the helpers below
        // can read the rest of the record.
        let mut xs: Vec<Vec3> = Vec::new();
        let mut x0s: Vec<Vec3> = Vec::new();
        let mut vs: Vec<Vec3> = Vec::new();
        core::mem::swap(&mut xs, &mut self.positions);
        core::mem::swap(&mut x0s, &mut self.previous_positions);
        core::mem::swap(&mut vs, &mut self.velocities);

        let n = xs.len();
        if self.pressure > 0.0 && self.settings.faces.is_empty() == false {
            self.apply_pressure(&mut xs, &mut vs, &self.inv_masses, h);
        }

        // Integrate.
        for i in 0..n {
            x0s[i] = xs[i];
            if !(self.inv_masses[i] > 0.0) {
                continue;
            }
            let mut vi = damping * mul_add(vs[i], h, g);
            let s2 = length_squared(vi);
            if s2 > max_speed_squared {
                vi = (self.max_linear_velocity / s2.sqrt()) * vi;
            }
            vs[i] = vi;
            xs[i] = mul_add(xs[i], h, vi);
        }

        self.solve_bend_batches(&mut xs, &self.inv_masses, substep.inv_h2);
        self.solve_volume_batches(&mut xs, &self.inv_masses, substep.inv_h2);
        self.apply_skin(&self.settings.skinned, &mut xs, &self.inv_masses);
        self.solve_edge_batches(&mut xs, &self.inv_masses, substep.inv_h2);
        self.apply_long_range_attachments(
            &self.settings.long_range_attachments,
            &mut xs,
            &self.inv_masses,
        );

        core::mem::swap(&mut xs, &mut self.positions);
        core::mem::swap(&mut x0s, &mut self.previous_positions);
        core::mem::swap(&mut vs, &mut self.velocities);
    }

    /// Rigid body collisions of one substep, and velocities from the position change.
    pub(crate) fn collide_substep(&mut self, substep: SoftBodySubstep, last: bool) {
        let mut xs: Vec<Vec3> = Vec::new();
        let mut x0s: Vec<Vec3> = Vec::new();
        let mut vs: Vec<Vec3> = Vec::new();
        let mut contacts: Vec<bool> = Vec::new();
        let mut collider_state: Vec<SoftBodyCollider> = Vec::new();
        core::mem::swap(&mut xs, &mut self.positions);
        core::mem::swap(&mut x0s, &mut self.previous_positions);
        core::mem::swap(&mut vs, &mut self.velocities);
        core::mem::swap(&mut contacts, &mut self.has_contact);
        core::mem::swap(&mut collider_state, &mut self.colliders);

        self.apply_edge_collisions(
            &mut xs,
            &mut x0s,
            &self.inv_masses,
            &mut contacts,
            &mut collider_state,
            substep.h,
            last,
        );
        self.apply_collisions(
            &mut xs,
            &mut x0s,
            &mut vs,
            &self.inv_masses,
            &mut contacts,
            &mut collider_state,
            substep.h,
            substep.restitution_threshold,
            last,
        );

        core::mem::swap(&mut xs, &mut self.positions);
        core::mem::swap(&mut x0s, &mut self.previous_positions);
        core::mem::swap(&mut vs, &mut self.velocities);
        core::mem::swap(&mut contacts, &mut self.has_contact);
        core::mem::swap(&mut collider_state, &mut self.colliders);
    }

    /// Called after `simulate`; bounds use the final positions.
    pub(crate) fn finish_step(&mut self) {
        self.update_bounds();
    }

    // MARK: Constraints

    fn apply_long_range_attachments(
        &self,
        attachments: &[SoftBodyLRA],
        x: &mut [Vec3],
        w: &[f32],
    ) {
        if attachments.is_empty() {
            return;
        }
        for a in attachments.iter() {
            if !(w[a.vertex as usize] > 0.0) {
                continue;
            }
            let anchor = x[a.anchor as usize];
            let d = x[a.vertex as usize] - anchor;
            let len2 = length_squared(d);
            if len2 > a.max_distance * a.max_distance {
                x[a.vertex as usize] = mul_add(anchor, a.max_distance / len2.sqrt(), d);
            }
        }
    }

    fn apply_skin(&self, skinned: &[SoftBodySkinned], x: &mut [Vec3], w: &[f32]) {
        if !(self.has_skin_pose && skinned.is_empty() == false) {
            return;
        }
        for (k, s) in skinned.iter().enumerate() {
            if !(w[s.vertex as usize] > 0.0) {
                continue;
            }
            let v = s.vertex as usize;
            let target = self.skinned_positions[k];
            let mut p = x[v];

            // Stay within the max distance of the skin.
            let d = p - target;
            let len2 = length_squared(d);
            if len2 > s.max_distance * s.max_distance {
                p = if len2 > 0.0 {
                    mul_add(target, s.max_distance / len2.sqrt(), d)
                } else {
                    target
                };
            }

            // Stay outside the back stop sphere.
            if s.back_stop_radius > 0.0 {
                let center = mul_sub(
                    target,
                    s.back_stop_distance + s.back_stop_radius,
                    self.skinned_normals[k],
                );
                let e = p - center;
                let len = length(e);
                if len < s.back_stop_radius {
                    p = if len > 1.0e-9 {
                        mul_add(center, s.back_stop_radius / len, e)
                    } else {
                        mul_add(center, s.back_stop_radius, self.skinned_normals[k])
                    };
                }
            }
            x[v] = p;
        }
    }

    /// Inflate a closed surface: p = pressure / V on each face, shared by its three vertices.
    fn apply_pressure(&self, x: &mut [Vec3], v: &mut [Vec3], w: &[f32], h: f32) {
        let faces = &self.settings.faces;
        let origin = x[0];
        let mut six_volume: f32 = 0.0;
        for f in faces.iter() {
            let a = x[f.vertex.x as usize] - origin;
            let b = x[f.vertex.y as usize] - origin;
            let c = x[f.vertex.z as usize] - origin;
            six_volume += dot(a, cross(b, c));
        }
        if !(six_volume > 1.0e-9) {
            return;
        }
        // Face force = p * area normal = p * cross / 2, split over three vertices.
        let coefficient = self.pressure / (six_volume / 6.0) * h / 6.0;
        for f in faces.iter() {
            let i0 = f.vertex.x as usize;
            let i1 = f.vertex.y as usize;
            let i2 = f.vertex.z as usize;
            let impulse = coefficient * cross(x[i1] - x[i0], x[i2] - x[i0]);
            v[i0] = mul_add(v[i0], w[i0], impulse);
            v[i1] = mul_add(v[i1], w[i1], impulse);
            v[i2] = mul_add(v[i2], w[i2], impulse);
        }
    }

    // MARK: Collision

    /// Project colliding vertices out of their planes, then update velocities with friction,
    /// restitution, and the impulse exchange with dynamic bodies (Müller et al., Detailed Rigid
    /// Body Simulation with Extended Position Based Dynamics, section 3.6).
    fn apply_collisions(
        &self,
        x: &mut [Vec3],
        x0: &mut [Vec3],
        v: &mut [Vec3],
        w: &[f32],
        contact: &mut [bool],
        colliders: &mut [SoftBodyCollider],
        h: f32,
        restitution_threshold: f32,
        last: bool,
    ) {
        let inv_h = 1.0 / h;
        let n = x.len();
        let planes = &self.vertex_planes;
        let counts = &self.vertex_plane_counts;
        for i in 0..n {
            if !(w[i] > 0.0) {
                continue;
            }
            let previous_velocity = v[i];
            let mut vi = inv_h * (x[i] - x0[i]);

            for k in 0..counts[i] as usize {
                let plane = planes[i * SoftBodyRecord::PLANES_PER_VERTEX + k];
                let ci = plane.collider;
                let normal = plane.normal;
                let penetration = plane.offset - dot(normal, x[i]);
                if penetration > 0.0 {
                    x[i] = mul_add(x[i], penetration, normal);
                    if last {
                        contact[i] = true;
                    }

                    let collider = ci as usize;
                    let r = x[i] - sub_pos(colliders[collider].center, self.origin);
                    let body_velocity = colliders[collider].point_velocity(r);
                    let relative = vi - body_velocity;
                    let vn = dot(normal, relative);
                    let normal_velocity = min_float(vn, 0.0) * normal;
                    let tangent_velocity = relative - vn * normal;
                    let tangent_length = length(tangent_velocity);

                    // Friction bounded by the normal correction: |dv_t| <= mu * d / h.
                    let mut dv = Vec3::default();
                    if tangent_length > 0.0 {
                        dv = min_float(
                            colliders[collider].friction * penetration / (tangent_length * h),
                            1.0,
                        ) * tangent_velocity;
                    }
                    // Remove the approach velocity, then bounce off fast impacts.
                    dv += normal_velocity;
                    let previous_normal = dot(previous_velocity - body_velocity, normal);
                    if previous_normal < restitution_threshold {
                        dv = mul_add(
                            dv,
                            colliders[collider].restitution * previous_normal,
                            normal,
                        );
                    }

                    if colliders[collider].is_dynamic {
                        let rn = cross(r, normal);
                        let w2 = colliders[collider].inv_mass
                            + dot(rn, mul_mv(colliders[collider].inv_inertia, rn));
                        let total = w[i] + w2;
                        if total > 0.0 {
                            let p = (1.0 / total) * dv;
                            vi = mul_sub(vi, w[i], p);
                            colliders[collider].linear_velocity = mul_add(
                                colliders[collider].linear_velocity,
                                colliders[collider].inv_mass,
                                p,
                            );
                            colliders[collider].angular_velocity =
                                colliders[collider].angular_velocity
                                    + mul_mv(
                                        colliders[collider].inv_inertia,
                                        cross(r, p),
                                    );
                            colliders[collider].updated = true;
                        }
                    } else {
                        vi = vi - dv;
                    }
                    if last {
                        colliders[collider].contact_count += 1;
                    }
                }
            }
            v[i] = vi;
        }
    }

    // MARK: Sleep

    pub(crate) fn update_sleep(&mut self, dt: f32) {
        if !self.enable_sleep {
            self.sleep_time = 0.0;
            return;
        }
        if self.max_speed() < self.sleep_threshold {
            self.sleep_time += dt;
            if self.sleep_time >= b3::TIME_TO_SLEEP {
                self.awake = false;
                self.velocities.fill(Vec3::default());
            }
        } else {
            self.sleep_time = 0.0;
        }
    }
}

// MARK: - Dihedral gradients

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct DihedralGradients {
    pub angle: f32,
    pub g0: Vec3,
    pub g1: Vec3,
    pub g2: Vec3,
    pub g3: Vec3,
    pub valid: bool,
}

/// Angle and its gradients for the dihedral between triangles (x0, x1, x2) and (x1, x0, x3)
/// (Bridson et al. 2003, Simulation of Clothing with Folds and Wrinkles).
#[inline(always)]
pub fn dihedral_gradients(x0: Vec3, x1: Vec3, x2: Vec3, x3: Vec3) -> DihedralGradients {
    let mut result = DihedralGradients::default();
    let e = x1 - x0;
    let len = length(e);
    let n1 = cross(e, x2 - x0);
    let n2 = cross(x3 - x0, e);
    let n1_squared = length_squared(n1);
    let n2_squared = length_squared(n2);
    if !(len > 1.0e-9 && n1_squared > 1.0e-18 && n2_squared > 1.0e-18) {
        return result;
    }

    result.angle = atan2_approx(dot(cross(n1, n2), e) / len, dot(n1, n2));

    let inv_len = 1.0 / len;
    let m1 = (1.0 / n1_squared) * n1;
    let m2 = (1.0 / n2_squared) * n2;
    // Gradients of the angle; they sum to zero.
    result.g2 = (-len) * m1;
    result.g3 = (-len) * m2;
    result.g0 = -(dot(x2 - x1, e) * inv_len * m1 + dot(x3 - x1, e) * inv_len * m2);
    result.g1 = dot(x2 - x0, e) * inv_len * m1 + dot(x3 - x0, e) * inv_len * m2;
    result.valid = true;
    result
}
