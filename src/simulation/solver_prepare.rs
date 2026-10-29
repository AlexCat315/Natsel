//! Constraint preparation in two passes: contacts and joints.
//!
//! Translated from `Sources/Natsel/Simulation/SolverPrepare.swift`.
//!
//! Planning runs on one thread: it fixes the solver order of the contacts, gives every scalar
//! contact and every four-wide batch its constraint slot and its run in the manifold pools, and sizes
//! the arrays. Filling then writes only those slots, so it runs on any number of threads and lays the
//! data out exactly as appending in order did.

use std::ops::Range;

use crate::base::constants as b3;
use crate::math::functions::min_float;

use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::contact_solver::{ContactConstraint, ManifoldConstraint};
use crate::simulation::solver_integration::{
    buffer_view, make_soft, Softness, StepContext,
};
use crate::simulation::solver_stages::SolverStageKind;
use crate::simulation::wide_contact_solver::{ContactConstraintW, ManifoldConstraintW};
use crate::simulation::wide_joint_solver::SphericalConstraintW;
use crate::simulation::world_runtime::{
    Body, BodySim, BodyState, Contact, JointSim, SetType, World,
};

/// One unit of contact preparation: a scalar contact or a batch of up to four contacts.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ContactPrepareItem {
    pub color: i32,
    pub wide: bool,

    /// Constraint index in the color's scalar or wide list.
    pub slot: i32,

    /// First contact id in `World.prepare_ids` and how many (one for scalar).
    pub id_start: i32,
    pub id_count: i32,

    /// Run in the scalar manifold pool or the wide manifold pool.
    pub manifold_start: i32,
}

impl ContactPrepareItem {
    #[inline]
    pub const fn new() -> Self {
        Self {
            color: 0,
            wide: false,
            slot: 0,
            id_start: 0,
            id_count: 0,
            manifold_start: 0,
        }
    }
}

/// One planned four-wide spherical joint batch.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct WideJointPrepareItem {
    pub color: i32,
    pub first: i32,
    pub lane_count: i32,
    pub batch: i32,
}

impl WideJointPrepareItem {
    #[inline]
    pub const fn new() -> Self {
        Self {
            color: 0,
            first: 0,
            lane_count: 0,
            batch: 0,
        }
    }
}

/// Pointers and parameters for filling, taken after the arrays are sized.
#[derive(Clone)]
pub(crate) struct ContactPrepareViews {
    pub items: *const ContactPrepareItem,
    pub ids: *const i32,
    pub contacts: *const Contact,
    pub sims: *const BodySim,
    pub sims_count: usize,
    pub states: *mut BodyState,
    pub states_count: usize,
    pub manifold_pool: *mut ManifoldConstraint,
    pub wide_manifolds: *mut ManifoldConstraintW,
    pub constraints: Vec<*mut ContactConstraint>,
    pub wide_constraints: Vec<*mut ContactConstraintW>,
    pub softness: Softness,
    pub static_softness: Softness,
    pub warm_start_scale: f32,

    /// Joint lists of every color, with the index of each color's first joint.
    pub joints: Vec<*mut JointSim>,
    pub joint_starts: Vec<usize>,

    /// Body table and the sims of the two sets joint bodies can be in, for joint preparation.
    pub bodies: *const Body,
    pub static_sims: *const BodySim,
    pub awake_sims: *const BodySim,
    pub wide_joint_items: *const WideJointPrepareItem,
    pub wide_joint_batches: Vec<*mut SphericalConstraintW>,
}

impl Default for ContactPrepareViews {
    #[inline]
    fn default() -> Self {
        Self {
            items: std::ptr::null(),
            ids: std::ptr::null(),
            contacts: std::ptr::null(),
            sims: std::ptr::null(),
            sims_count: 0,
            states: std::ptr::null_mut(),
            states_count: 0,
            manifold_pool: std::ptr::null_mut(),
            wide_manifolds: std::ptr::null_mut(),
            constraints: Vec::new(),
            wide_constraints: Vec::new(),
            softness: Softness::new(),
            static_softness: Softness::new(),
            warm_start_scale: 0.0,
            joints: Vec::new(),
            joint_starts: Vec::new(),
            bodies: std::ptr::null(),
            static_sims: std::ptr::null(),
            awake_sims: std::ptr::null(),
            wide_joint_items: std::ptr::null(),
            wide_joint_batches: Vec::new(),
        }
    }
}

impl World {
    // MARK: Session

    /// Wake the worker pool for preparation and solving. Without one everything runs inline.
    pub(crate) fn begin_solver_session(&mut self) {
        let pool = match self.solver_pool() {
            Some(pool) => pool,
            None => return,
        };
        let mut pool = pool;

        // Swift captured `unowned self` in the handler, so the workers reached the world through the
        // class reference. Rust moves the raw address into the `'static` handler instead.
        // SAFETY: the world that begins a session outlives it. `World` is not moved or dropped while
        // a session is active: `end_solver_session` runs `WorkerPool::end`, which waits until no
        // worker is spinning, before the world can be mutated any further or returned to the pool.
        // Each stage handler writes only the array slots its own block owns, exactly as in Swift, so
        // no two threads alias a write.
        let world = self as *mut World;
        pool.begin(move |stage, block, thread| {
            // SAFETY: as above; the handler only lives between `begin` and `end`.
            unsafe { (*world).execute_solver_block(stage, block, thread) };
        });
        self.active_pool = Some(pool);
    }

    pub(crate) fn end_solver_session(&mut self) {
        if let Some(mut pool) = self.active_pool.take() {
            pool.end();
            // Swift's `activePool = nil` released one reference to the same class instance; Rust
            // returns the single owned pool to `workerPool` so it survives to the next step.
            self.worker_pool = Some(pool);
        }
    }

    // MARK: Joints

    /// Prepare every joint (all colors including overflow), then pack the wide batches.
    pub(crate) fn prepare_all_joints(&mut self, context: &StepContext) {
        // Swift copied `prepareViews` into a local, filled it, and assigned it back before running
        // the stage; nothing reads the field in between, so the in-place writes are equivalent and
        // avoid cloning the pointer lists.
        self.stage_context = context.clone();
        self.prepare_views.joints.clear();
        self.prepare_views.joint_starts.clear();
        let mut total = 0usize;
        for ci in 0..b3::GRAPH_COLOR_COUNT as usize {
            self.prepare_views
                .joints
                .push(self.constraint_graph.colors[ci].joint_sims.as_mut_ptr());
            self.prepare_views.joint_starts.push(total);
            total += self.constraint_graph.colors[ci].joint_sims.len();
        }
        self.prepare_views.joint_starts.push(total);
        self.prepare_views.bodies = self.bodies.as_ptr();
        self.prepare_views.static_sims =
            self.solver_sets[SetType::StaticSet.raw_value() as usize]
                .body_sims
                .as_ptr();
        self.prepare_views.awake_sims = self.solver_sets[SetType::AwakeSet.raw_value() as usize]
            .body_sims
            .as_ptr();
        self.run_stage(
            SolverStageKind::PrepareJoints,
            0,
            total,
            Self::PARALLEL_PREPARE_ITEMS,
        );
        self.prepare_wide_joints();
        self.prepare_views.bodies = std::ptr::null();
        self.prepare_views.static_sims = std::ptr::null();
        self.prepare_views.awake_sims = std::ptr::null();
    }

    /// A joint body during preparation. Reads through raw pointers: going through the Swift arrays
    /// retains their buffers, and several threads retaining one buffer serialize on its count.
    #[inline(always)]
    pub(crate) fn joint_prepare_body(&self, body_id: i32) -> Body {
        let table = self.prepare_views.bodies;
        if !table.is_null() {
            // SAFETY: `prepare_all_joints` installs the world's body table for the whole joint
            // stage and joint bodies are only read here.
            return unsafe { (*table.add(body_id as usize)).clone() };
        }
        self.bodies[body_id as usize].clone()
    }

    /// The sim of a joint body during preparation. Joint bodies are awake or static.
    #[inline(always)]
    pub(crate) fn joint_prepare_sim(&self, body: &Body) -> BodySim {
        if body.set_index == SetType::AwakeSet.raw_value() && !self.prepare_views.awake_sims.is_null()
        {
            // SAFETY: the awake set's sim buffer is installed for the whole joint stage.
            return unsafe { *self.prepare_views.awake_sims.add(body.local_index as usize) };
        }
        if body.set_index == SetType::StaticSet.raw_value() && !self.prepare_views.static_sims.is_null()
        {
            // SAFETY: the static set's sim buffer is installed for the whole joint stage.
            return unsafe { *self.prepare_views.static_sims.add(body.local_index as usize) };
        }
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
    }

    /// Prepare joints `range` of the flattened joint list.
    pub(crate) fn prepare_joint_range(&self, range: Range<usize>) {
        let starts = &self.prepare_views.joint_starts;
        let lists = &self.prepare_views.joints;
        let context = &self.stage_context;
        let mut ci = 0usize;
        for index in range {
            while starts[ci + 1] <= index {
                ci += 1;
            }
            let list = lists[ci];
            if list.is_null() {
                continue;
            }
            // SAFETY: `prepare_all_joints` installed one pointer per graph color, the color's joint
            // list is not resized during the stage, and `index - starts[ci]` is inside it because
            // `starts` bounds the color's slice. Each joint is written only by its own index.
            unsafe {
                self.prepare_joint(list.add(index - starts[ci]), context);
            }
        }
    }

    // MARK: Contacts

    /// Prepare all constraint data for this step.
    pub(crate) fn prepare_all_contacts(&mut self, context: &StepContext) {
        for i in 0..self.solver_contacts.constraints.len() {
            self.solver_contacts.constraints[i].clear();
        }
        for i in 0..self.solver_contacts.convex_counts.len() {
            self.solver_contacts.convex_counts[i] = 0;
            self.solver_contacts.wide_convex[i] = false;
            self.solver_contacts.wide_mesh[i] = false;
            self.solver_contacts.wide_convex_counts[i] = 0;
            self.solver_contacts.wide_constraints[i].clear();
        }
        // The manifold pools keep their high-water size: planning assigns every run, filling writes
        // it, and the unused tail is never read. Clearing them would zero megabytes every step.
        self.solve_any_restitution = false;

        self.prepare_all_joints(context);

        let contact_hertz = min_float(self.contact_hertz, 0.125 * context.inv_h);
        let contact_softness = make_soft(contact_hertz, self.contact_damping_ratio, context.h);
        let static_softness = make_soft(
            2.0 * contact_hertz,
            0.5 * self.contact_damping_ratio,
            context.h,
        );

        // Overflow first, then colors in order.
        self.prepare_overflow_contacts(contact_softness, static_softness);
        let count = self.plan_contact_preparation();
        if count == 0 {
            return;
        }

        let awake = SetType::AwakeSet.raw_value() as usize;
        self.prepare_views.items = self.prepare_items.as_ptr();
        self.prepare_views.ids = self.prepare_ids.as_ptr();
        self.prepare_views.contacts = self.contacts.as_ptr();
        self.prepare_views.sims = self.solver_sets[awake].body_sims.as_ptr();
        self.prepare_views.sims_count = self.solver_sets[awake].body_sims.len();
        self.prepare_views.states = self.solver_sets[awake].body_states.as_mut_ptr();
        self.prepare_views.states_count = self.solver_sets[awake].body_states.len();
        self.prepare_views.manifold_pool = self.solver_contacts.manifold_pool.as_mut_ptr();
        self.prepare_views.wide_manifolds = self.solver_contacts.wide_manifolds.as_mut_ptr();
        self.prepare_views.constraints.clear();
        self.prepare_views.wide_constraints.clear();
        for ci in 0..OVERFLOW_INDEX as usize {
            self.prepare_views
                .constraints
                .push(self.solver_contacts.constraints[ci].as_mut_ptr());
            self.prepare_views
                .wide_constraints
                .push(self.solver_contacts.wide_constraints[ci].as_mut_ptr());
        }
        self.prepare_views.softness = contact_softness;
        self.prepare_views.static_softness = static_softness;
        self.prepare_views.warm_start_scale = if self.enable_warm_starting { 1.0 } else { 0.0 };

        self.run_stage(
            SolverStageKind::PrepareContacts,
            0,
            count,
            Self::PARALLEL_PREPARE_ITEMS,
        );
    }

    /// Fix the order, slots, and manifold runs of every colored contact and size the arrays.
    /// Returns the number of preparation items.
    fn plan_contact_preparation(&mut self) -> usize {
        self.prepare_items.clear();
        self.prepare_ids.clear();
        let mut contact_total = 0usize;
        for ci in 0..OVERFLOW_INDEX as usize {
            contact_total += self.constraint_graph.colors[ci].convex_contacts.len()
                + self.constraint_graph.colors[ci].contacts.len();
        }
        if contact_total == 0 {
            return 0;
        }
        self.prepare_items.reserve(contact_total);
        self.prepare_ids.reserve(contact_total);

        let mut manifold_total: i32 = 0;
        let mut wide_manifold_total: i32 = 0;
        let mut any_restitution = false;

        for ci in 0..OVERFLOW_INDEX as usize {
            let convex_count = self.constraint_graph.colors[ci].convex_contacts.len();
            // Swift borrowed the color's id array; reading it by index keeps the same order and
            // avoids holding a borrow of `constraintGraph` across the writes to `solverContacts`.
            let mesh_ids: Vec<i32> = if self.constraint_graph.colors[ci].contacts.is_empty() {
                Vec::new()
            } else {
                self.solver_mesh_contact_ids(ci as i32)
            };
            if convex_count == 0 && mesh_ids.is_empty() {
                continue;
            }

            let wide_convex = self.enable_wide_solver && convex_count >= 4;
            let wide_mesh = self.enable_wide_solver && mesh_ids.len() >= 4;
            self.solver_contacts.wide_convex[ci] = wide_convex;
            self.solver_contacts.wide_mesh[ci] = wide_mesh;
            let scalar_convex = !wide_convex || self.prepare_scalar_for_wide_colors;
            let scalar_mesh = !wide_mesh || self.prepare_scalar_for_wide_colors;
            self.solver_contacts.convex_counts[ci] = if scalar_convex {
                convex_count as i32
            } else {
                0
            };

            // Scalar constraints: convex then mesh.
            let mut slot: i32 = 0;
            for pass in 0..2 {
                let enabled = if pass == 0 { scalar_convex } else { scalar_mesh };
                if !enabled {
                    continue;
                }
                let count = if pass == 0 { convex_count } else { mesh_ids.len() };
                for k in 0..count {
                    let id = if pass == 0 {
                        self.constraint_graph.colors[ci].convex_contacts[k]
                    } else {
                        mesh_ids[k]
                    };
                    let contact = self.contacts[id as usize];
                    let mut item = ContactPrepareItem::new();
                    item.color = ci as i32;
                    item.slot = slot;
                    item.id_start = self.prepare_ids.len() as i32;
                    item.id_count = 1;
                    item.manifold_start = manifold_total;
                    self.prepare_items.push(item);
                    self.prepare_ids.push(id);
                    manifold_total += contact.manifold_count;
                    any_restitution = any_restitution || contact.restitution > 0.0;
                    slot += 1;
                }
            }
            let length = self.solver_contacts.constraints[ci].len();
            self.solver_contacts.constraints[ci]
                .resize(length + slot as usize, ContactConstraint::default());

            // Wide batches: convex then mesh.
            let mut batch: i32 = 0;
            for pass in 0..2 {
                if pass == 1 {
                    self.solver_contacts.wide_convex_counts[ci] = batch;
                }
                let enabled = if pass == 0 { wide_convex } else { wide_mesh };
                if !enabled {
                    continue;
                }
                let count = if pass == 0 { convex_count } else { mesh_ids.len() };
                // Swift's `stride(from: 0, to: count, by: 4)`.
                let mut first = 0usize;
                while first < count {
                    let lanes = std::cmp::min(4, count - first);
                    let mut item = ContactPrepareItem::new();
                    item.color = ci as i32;
                    item.wide = true;
                    item.slot = batch;
                    item.id_start = self.prepare_ids.len() as i32;
                    item.id_count = lanes as i32;
                    item.manifold_start = wide_manifold_total;
                    self.prepare_items.push(item);
                    let mut max_manifolds: i32 = 0;
                    for lane in 0..lanes {
                        let id = if pass == 0 {
                            self.constraint_graph.colors[ci].convex_contacts[first + lane]
                        } else {
                            mesh_ids[first + lane]
                        };
                        let contact = self.contacts[id as usize];
                        self.prepare_ids.push(id);
                        max_manifolds = std::cmp::max(max_manifolds, contact.manifold_count);
                        any_restitution = any_restitution || contact.restitution > 0.0;
                    }
                    wide_manifold_total += max_manifolds;
                    batch += 1;
                    first += 4;
                }
            }
            let length = self.solver_contacts.wide_constraints[ci].len();
            self.solver_contacts.wide_constraints[ci]
                .resize(length + batch as usize, ContactConstraintW::default());
        }
        if any_restitution {
            self.solve_any_restitution = true;
        }

        let pool = manifold_total as isize - self.solver_contacts.manifold_pool.len() as isize;
        if pool > 0 {
            let length = self.solver_contacts.manifold_pool.len();
            self.solver_contacts
                .manifold_pool
                .resize(length + pool as usize, ManifoldConstraint::default());
        }
        let wide_pool = wide_manifold_total as isize
            - self.solver_contacts.wide_manifolds.len() as isize;
        if wide_pool > 0 {
            let length = self.solver_contacts.wide_manifolds.len();
            self.solver_contacts
                .wide_manifolds
                .resize(length + wide_pool as usize, ManifoldConstraintW::default());
        }
        self.prepare_items.len()
    }

    /// Fill items `range` of the plan. Each writes only its own slots.
    pub(crate) fn prepare_contact_range(&self, range: Range<usize>) {
        let views = &self.prepare_views;
        if views.items.is_null() || views.ids.is_null() || views.contacts.is_null() {
            return;
        }
        for i in range {
            // SAFETY: `plan_contact_preparation` sized `prepareItems` to the returned count and the
            // ids and contacts arrays are not resized during the stage.
            let item = unsafe { *views.items.add(i) };
            let ci = item.color as usize;
            if item.wide {
                let out = views.wide_constraints[ci];
                if out.is_null()
                    || views.wide_manifolds.is_null()
                    || views.sims.is_null()
                    || views.states.is_null()
                {
                    continue;
                }
                // SAFETY: each item owns its constraint slot, its id run, and its manifold run.
                unsafe {
                    let ids = views.ids.add(item.id_start as usize);
                    let states =
                        std::slice::from_raw_parts_mut(views.states, views.states_count);
                    self.prepare_wide_batch(
                        ids,
                        item.id_count as usize,
                        out.add(item.slot as usize),
                        views.wide_manifolds.add(item.manifold_start as usize),
                        item.manifold_start as usize,
                        views.contacts,
                        views.sims,
                        states,
                        views.softness,
                        views.static_softness,
                        views.warm_start_scale,
                    );
                }
            } else {
                let out = views.constraints[ci];
                if out.is_null() || views.manifold_pool.is_null() {
                    continue;
                }
                // SAFETY: as above; the scalar branch writes only this item's constraint slot and
                // its manifold run in the scalar pool.
                unsafe {
                    let sims = buffer_view(views.sims, views.sims_count);
                    let states = std::slice::from_raw_parts_mut(views.states, views.states_count);
                    self.prepare_contact_constraint(
                        *views.ids.add(item.id_start as usize),
                        out.add(item.slot as usize),
                        views.manifold_pool.add(item.manifold_start as usize),
                        item.manifold_start,
                        views.contacts,
                        sims,
                        states,
                        views.softness,
                        views.static_softness,
                        views.warm_start_scale,
                    );
                }
            }
        }
    }
}
