//! Staged constraint solver.
//!
//! Translated from `Sources/Natsel/Simulation/SolverStages.swift`.
//!
//! Each substep runs body integration and, per graph color, warm starting, the biased solve, and
//! relaxation; then restitution and impulse storage. A graph color holds joints and contacts that
//! share no dynamic body, so all of a color's work can run on any number of threads and give the
//! same bits as running it in order. The overflow constraints and vehicles can touch any body; they
//! run on the calling thread between colors, in the same order as the serial solver.

use std::ops::Range;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::base::platform::{get_milliseconds_and_reset, get_ticks, performance_core_count};
use crate::base::worker_pool::{WorkerPool, WorkerStage};
use crate::math::functions::{max_int, min_int};

use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::contact_solver::{ContactConstraint, ManifoldConstraint};
use crate::simulation::distance_joint_solver::JointColorView;
use crate::simulation::solver_integration::{buffer_view, mutable_buffer_view, StepContext};
use crate::simulation::wide_contact_solver::{ContactConstraintW, ManifoldConstraintW};
use crate::simulation::world_runtime::{BodySim, BodyState, Contact, SetType, World};

/// The stage a worker block runs. Swift's `enum SolverStageKind: Int32`; the raw values are the
/// numbers carried through [`WorkerStage`], so they must stay in this order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum SolverStageKind {
    PrepareJoints = 0,
    PrepareContacts = 1,
    PrepareWideJoints = 2,
    IntegrateVelocities = 3,
    IntegratePositions = 4,
    WarmStart = 5,
    Push = 6,
    Relax = 7,
    Restitution = 8,
    Store = 9,
    FinalizeBodies = 10,
}

impl SolverStageKind {
    /// Swift's `.rawValue`.
    #[inline(always)]
    pub fn raw_value(self) -> i32 {
        self as i32
    }

    /// Swift's `SolverStageKind(rawValue:)`.
    #[inline(always)]
    pub fn from_raw(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::PrepareJoints),
            1 => Some(Self::PrepareContacts),
            2 => Some(Self::PrepareWideJoints),
            3 => Some(Self::IntegrateVelocities),
            4 => Some(Self::IntegratePositions),
            5 => Some(Self::WarmStart),
            6 => Some(Self::Push),
            7 => Some(Self::Relax),
            8 => Some(Self::Restitution),
            9 => Some(Self::Store),
            10 => Some(Self::FinalizeBodies),
            _ => None,
        }
    }
}

/// One color's solver work as raw pointers.
#[derive(Clone, Copy)]
pub(crate) struct SolverColorView {
    pub joints: JointColorView,
    pub constraints: *const ContactConstraint,
    pub scalar_convex: usize,
    pub scalar_count: usize,
    pub wide: *const ContactConstraintW,
    pub wide_convex: usize,
    pub wide_count: usize,
    pub use_wide_convex: bool,
    pub use_wide_mesh: bool,
}

impl Default for SolverColorView {
    #[inline]
    fn default() -> Self {
        Self {
            joints: JointColorView::default(),
            constraints: std::ptr::null(),
            scalar_convex: 0,
            scalar_count: 0,
            wide: std::ptr::null(),
            wide_convex: 0,
            wide_count: 0,
            use_wide_convex: false,
            use_wide_mesh: false,
        }
    }
}

impl SolverColorView {
    #[inline(always)]
    pub fn convex_items(&self) -> usize {
        if self.use_wide_convex {
            self.wide_convex
        } else {
            self.scalar_convex
        }
    }

    #[inline(always)]
    pub fn mesh_items(&self) -> usize {
        if self.use_wide_mesh {
            self.wide_count - self.wide_convex
        } else {
            self.scalar_count - self.scalar_convex
        }
    }

    /// Joint units, then convex contacts, then mesh contacts.
    #[inline(always)]
    pub fn item_count(&self) -> usize {
        self.joints.unit_count + self.convex_items() + self.mesh_items()
    }
}

/// Pointers into the solver arrays, taken once per step after preparation. The arrays are not
/// resized until the impulses are stored.
#[derive(Clone, Copy)]
pub(crate) struct SolverViews {
    pub sims: *const BodySim,
    pub sims_count: usize,
    pub states: *mut BodyState,
    pub states_count: usize,
    pub contacts: *mut Contact,
    pub manifold_pool: *mut ManifoldConstraint,
    pub manifold_pool_count: usize,
    pub wide_manifolds: *mut ManifoldConstraintW,
    pub colors: Vec<SolverColorView>,
}

impl Default for SolverViews {
    #[inline]
    fn default() -> Self {
        Self {
            sims: std::ptr::null(),
            sims_count: 0,
            states: std::ptr::null_mut(),
            states_count: 0,
            contacts: std::ptr::null_mut(),
            manifold_pool: std::ptr::null_mut(),
            manifold_pool_count: 0,
            wide_manifolds: std::ptr::null_mut(),
            colors: Vec::new(),
        }
    }
}

/// Swift's `Range<Int>.clamped(to:)`. Both ranges are signed so a range shifted below zero by the
/// color offsets clamps to the empty range exactly as Swift's does.
#[inline(always)]
fn clamp_range(range: Range<isize>, bounds: Range<isize>) -> Range<isize> {
    std::cmp::max(range.start, bounds.start)..std::cmp::min(range.end, bounds.end)
}

/// Swift's `Range<Int>` subscript offsets. Only called on a clamped, non-empty range.
#[inline(always)]
fn index_range(range: Range<isize>) -> Range<usize> {
    range.start as usize..range.end as usize
}

impl World {
    /// Minimum items before a color stage is spread over the workers.
    pub(crate) const PARALLEL_COLOR_ITEMS: usize = 32;
    /// Minimum bodies before an integration stage is spread over the workers.
    pub(crate) const PARALLEL_BODY_ITEMS: usize = 512;
    /// Minimum joints or contact items before preparation is spread over the workers.
    pub(crate) const PARALLEL_PREPARE_ITEMS: usize = 64;

    pub(crate) fn build_solver_views(&mut self) {
        let mut views = SolverViews::default();
        let awake = SetType::AwakeSet.raw_value() as usize;
        views.sims = self.solver_sets[awake].body_sims.as_ptr();
        views.sims_count = self.solver_sets[awake].body_sims.len();
        views.states = self.solver_sets[awake].body_states.as_mut_ptr();
        views.states_count = self.solver_sets[awake].body_states.len();
        views.contacts = self.contacts.as_mut_ptr();
        views.manifold_pool = self.solver_contacts.manifold_pool.as_mut_ptr();
        views.manifold_pool_count = self.solver_contacts.manifold_pool.len();
        views.wide_manifolds = self.solver_contacts.wide_manifolds.as_mut_ptr();

        views.colors.reserve(OVERFLOW_INDEX as usize);
        for ci in 0..OVERFLOW_INDEX as usize {
            let mut color = SolverColorView::default();
            let wide = self.enable_wide_solver && !self.solver_joints.starts[ci].is_empty();
            color.joints.joints = self.constraint_graph.colors[ci].joint_sims.as_mut_ptr();
            color.joints.joint_count = self.constraint_graph.colors[ci].joint_sims.len();
            if wide {
                color.joints.starts = self.solver_joints.starts[ci].as_ptr();
                color.joints.batches = self.solver_joints.batches[ci].as_mut_ptr();
                color.joints.units = self.solver_joints.units[ci].as_ptr();
                color.joints.unit_count = self.solver_joints.units[ci].len();
            } else {
                color.joints.unit_count = color.joints.joint_count;
            }

            color.constraints = self.solver_contacts.constraints[ci].as_ptr();
            color.scalar_count = self.solver_contacts.constraints[ci].len();
            color.scalar_convex = self.solver_contacts.convex_counts[ci] as usize;
            color.wide = self.solver_contacts.wide_constraints[ci].as_ptr();
            color.wide_count = self.solver_contacts.wide_constraints[ci].len();
            color.wide_convex = self.solver_contacts.wide_convex_counts[ci] as usize;
            color.use_wide_convex = self.uses_wide_contacts(ci as i32, false);
            color.use_wide_mesh = self.uses_wide_contacts(ci as i32, true);
            views.colors.push(color);
        }
        self.solver_views = views;
    }

    /// The worker pool for `workerCount` threads, or nil when solving on one thread.
    /// The solver uses at most `performanceCoreCount` threads: its stages are separated by
    /// barriers, so a thread on a slower core holds every stage back.
    ///
    /// The pool is taken out of `workerPool` and returned to it by `endSolverSession`; Swift shared
    /// one reference between the two fields, Rust moves the single owned value.
    pub(crate) fn solver_pool(&mut self) -> Option<WorkerPool> {
        let threads = std::cmp::min(
            max_int(1, min_int(self.worker_count, b3::MAX_WORKERS)) as usize,
            performance_core_count(),
        );
        if threads <= 1 {
            self.worker_pool = None;
            return None;
        }
        let existing = self.worker_pool.take();
        let pool = match existing {
            Some(pool) if pool.thread_count() == threads => pool,
            // A pool with the wrong thread count is dropped, which joins its threads, exactly as
            // Swift releasing the old reference did.
            _ => WorkerPool::new(threads),
        };
        self.worker_pool = Some(pool);
        self.worker_pool.take()
    }

    // MARK: Execution

    /// Run one block of a stage. Called on any thread.
    pub(crate) fn execute_solver_block(&self, stage: WorkerStage, block: usize, thread: usize) {
        let start = block * stage.block_size as usize;
        let range = start
            ..std::cmp::min(
                start + stage.block_size as usize,
                stage.item_count as usize,
            );
        let kind = match SolverStageKind::from_raw(stage.kind) {
            Some(kind) => kind,
            // Swift force-unwrapped the raw value, which traps on an unknown stage.
            None => {
                b3_assert(false);
                panic!("invalid solver stage kind {}", stage.kind);
            }
        };
        self.execute_solver_range(kind, stage.color as usize, range, thread);
    }

    /// `thread` indexes per-thread scratch data; zero is the calling thread.
    pub(crate) fn execute_solver_range(
        &self,
        kind: SolverStageKind,
        ci: usize,
        range: Range<usize>,
        thread: usize,
    ) {
        let context_h = self.stage_context.h;
        let context_inv_dt = self.stage_context.inv_dt;
        match kind {
            SolverStageKind::PrepareJoints => self.prepare_joint_range(range),
            SolverStageKind::PrepareContacts => self.prepare_contact_range(range),
            SolverStageKind::PrepareWideJoints => self.prepare_wide_joint_range(range),
            SolverStageKind::IntegrateVelocities => {
                // SAFETY: `solver_views` was built from the awake set, which lives for the whole
                // solver session, and every worker writes only the body indices of its own block.
                let sims = unsafe { buffer_view(self.solver_views.sims, self.solver_views.sims_count) };
                let states = unsafe {
                    mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                };
                self.integrate_velocities_range(sims, states, range, context_h);
            }
            SolverStageKind::IntegratePositions => {
                // SAFETY: as above; only this block's states are written.
                let states = unsafe {
                    mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                };
                self.integrate_positions_range(states, range, context_h, context_inv_dt);
            }
            SolverStageKind::FinalizeBodies => self.finalize_body_range(range, thread),
            _ => self.execute_color_range(kind, ci, range),
        }
    }

    fn execute_color_range(&self, kind: SolverStageKind, ci: usize, range: Range<usize>) {
        let color = self.solver_views.colors[ci];
        let context = &self.stage_context;

        // Joint units.
        let joints = color.joints.unit_count;
        let joint_range = clamp_range(
            range.start as isize..range.end as isize,
            0..joints as isize,
        );
        if !joint_range.is_empty() {
            // SAFETY: `solver_views.states` is the awake set's state buffer for the whole session,
            // and a color's units touch only the bodies of that color.
            let states = unsafe {
                mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
            };
            let joint_range = index_range(joint_range);
            match kind {
                SolverStageKind::WarmStart => {
                    self.warm_start_joints_range(color.joints, joint_range, context, states)
                }
                SolverStageKind::Push => {
                    self.solve_joints_range(color.joints, joint_range, context, true, states)
                }
                SolverStageKind::Relax => {
                    self.solve_joints_range(color.joints, joint_range, context, false, states)
                }
                _ => {}
            }
        }

        // Convex then mesh contacts.
        let convex = color.convex_items();
        let convex_range = clamp_range(
            range.start as isize - joints as isize..range.end as isize - joints as isize,
            0..convex as isize,
        );
        if !convex_range.is_empty() {
            self.execute_contact_range(kind, color, false, index_range(convex_range));
        }
        let mesh_start = joints + convex;
        let mesh_range = clamp_range(
            range.start as isize - mesh_start as isize..range.end as isize - mesh_start as isize,
            0..color.mesh_items() as isize,
        );
        if !mesh_range.is_empty() {
            self.execute_contact_range(kind, color, true, index_range(mesh_range));
        }
    }

    fn execute_contact_range(
        &self,
        kind: SolverStageKind,
        color: SolverColorView,
        mesh: bool,
        items: Range<usize>,
    ) {
        let context = &self.stage_context;
        if if mesh { color.use_wide_mesh } else { color.use_wide_convex } {
            if color.wide.is_null() || self.solver_views.wide_manifolds.is_null() {
                return;
            }
            let list = color.wide;
            let pool = self.solver_views.wide_manifolds;
            let offset = if mesh { color.wide_convex } else { 0 };
            let range = (items.start + offset)..(items.end + offset);
            match kind {
                SolverStageKind::WarmStart => {
                    // SAFETY: the wide manifold pool and the color's wide constraint list are owned
                    // by `solverContacts` for the whole session and each worker writes only the
                    // constraints of its own color and block.
                    let states = unsafe {
                        mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                    };
                    self.warm_start_contacts_wide_range(list, range, states, pool)
                }
                SolverStageKind::Push => {
                    // SAFETY: as above.
                    let states = unsafe {
                        mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                    };
                    self.push_contacts_wide_range(list, range, context, states, pool)
                }
                SolverStageKind::Relax => {
                    // SAFETY: as above.
                    let states = unsafe {
                        mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                    };
                    self.solve_contacts_wide_range(list, range, context, states, pool)
                }
                SolverStageKind::Restitution => {
                    // SAFETY: as above.
                    let states = unsafe {
                        mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                    };
                    self.apply_restitution_wide_range(list, range, context, states, pool)
                }
                SolverStageKind::Store => {
                    if !self.solver_views.contacts.is_null() {
                        self.store_impulses_wide_range(
                            list,
                            range,
                            pool as *const ManifoldConstraintW,
                            self.solver_views.contacts,
                        );
                    }
                }
                _ => {}
            }
        } else {
            if color.constraints.is_null() {
                return;
            }
            let list = color.constraints;
            let offset = if mesh { color.scalar_convex } else { 0 };
            let range = (items.start + offset)..(items.end + offset);
            match kind {
                SolverStageKind::WarmStart => {
                    // SAFETY: awake states and the manifold pool are stable for the session.
                    let states = unsafe {
                        mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                    };
                    let pool = unsafe {
                        mutable_buffer_view(
                            self.solver_views.manifold_pool,
                            self.solver_views.manifold_pool_count,
                        )
                    };
                    self.warm_start_contacts_body(range, list, states, pool)
                }
                SolverStageKind::Push => {
                    // SAFETY: as above.
                    let states = unsafe {
                        mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                    };
                    let pool = unsafe {
                        mutable_buffer_view(
                            self.solver_views.manifold_pool,
                            self.solver_views.manifold_pool_count,
                        )
                    };
                    self.push_contacts_body(range, list, context, states, pool)
                }
                SolverStageKind::Relax => {
                    // SAFETY: as above.
                    let states = unsafe {
                        mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                    };
                    let pool = unsafe {
                        mutable_buffer_view(
                            self.solver_views.manifold_pool,
                            self.solver_views.manifold_pool_count,
                        )
                    };
                    self.solve_contacts_body(range, list, context, states, pool)
                }
                SolverStageKind::Restitution => {
                    // SAFETY: as above.
                    let states = unsafe {
                        mutable_buffer_view(self.solver_views.states, self.solver_views.states_count)
                    };
                    let pool = unsafe {
                        mutable_buffer_view(
                            self.solver_views.manifold_pool,
                            self.solver_views.manifold_pool_count,
                        )
                    };
                    self.apply_restitution_body(range, list, context, states, pool)
                }
                SolverStageKind::Store => {
                    if !self.solver_views.contacts.is_null()
                        && !self.solver_views.manifold_pool.is_null()
                    {
                        self.store_impulses_body(
                            range,
                            list,
                            self.solver_views.manifold_pool as *const ManifoldConstraint,
                            self.solver_views.contacts,
                        );
                    }
                }
                _ => {}
            }
        }
    }

    /// Run a stage over `itemCount` items, in parallel when it is large enough.
    pub(crate) fn run_stage(
        &mut self,
        kind: SolverStageKind,
        ci: usize,
        item_count: usize,
        minimum: usize,
    ) {
        if item_count == 0 {
            return;
        }
        // The pool is moved out so no borrow of `World` is held while its handler runs stages.
        let pool = self.active_pool.take();
        match pool {
            None => self.execute_solver_range(kind, ci, 0..item_count, 0),
            Some(mut pool) => {
                if item_count < minimum {
                    self.execute_solver_range(kind, ci, 0..item_count, 0);
                } else {
                    // Several blocks per thread so faster threads take up the slack.
                    let target_blocks = 4 * pool.thread_count();
                    let block_size = std::cmp::max(
                        minimum / 4,
                        (item_count + target_blocks - 1) / target_blocks,
                    );
                    let mut stage = WorkerStage::default();
                    stage.kind = kind.raw_value();
                    stage.color = ci as i32;
                    stage.item_count = item_count as i32;
                    stage.block_size = block_size as i32;
                    stage.block_count = ((item_count + block_size - 1) / block_size) as i32;
                    pool.run(stage);
                }
                self.active_pool = Some(pool);
            }
        }
    }

    fn run_colors(&mut self, kind: SolverStageKind) {
        for ci in 0..OVERFLOW_INDEX as usize {
            let item_count = self.solver_views.colors[ci].item_count();
            self.run_stage(kind, ci, item_count, Self::PARALLEL_COLOR_ITEMS);
        }
    }

    fn run_bodies(&mut self, kind: SolverStageKind) {
        let item_count = self.solver_views.states_count;
        self.run_stage(kind, 0, item_count, Self::PARALLEL_BODY_ITEMS);
    }

    // MARK: Driver

    /// Substeps, restitution, and impulse storage. Prepared constraints are required. Runs on the
    /// session's worker pool when `beginSolverSession` started one.
    pub(crate) fn solve_constraint_stages(&mut self, context: &StepContext) {
        let mut ticks = get_ticks();
        self.stage_context = context.clone();
        self.build_solver_views();

        let overflow = OVERFLOW_INDEX;
        for _ in 0..context.sub_step_count as usize {
            self.run_bodies(SolverStageKind::IntegrateVelocities);
            self.profile.integrate_velocities += get_milliseconds_and_reset(&mut ticks);

            self.warm_start_joints(overflow, context);
            self.warm_start_contacts(overflow, true);
            self.run_colors(SolverStageKind::WarmStart);
            self.warm_start_vehicles(context);
            self.profile.warm_start += get_milliseconds_and_reset(&mut ticks);

            self.solve_joints(overflow, context, true);
            self.push_contacts(overflow, context, true);
            self.run_colors(SolverStageKind::Push);
            self.solve_vehicles(context, true);
            self.profile.solve_impulses += get_milliseconds_and_reset(&mut ticks);

            self.run_bodies(SolverStageKind::IntegratePositions);
            self.profile.integrate_positions += get_milliseconds_and_reset(&mut ticks);

            self.solve_joints(overflow, context, false);
            self.solve_contacts(overflow, context, true);
            self.run_colors(SolverStageKind::Relax);
            self.solve_vehicles(context, false);
            self.profile.relax_impulses += get_milliseconds_and_reset(&mut ticks);
        }

        if self.solve_any_restitution && self.restitution_iterations > 0 {
            for _ in 0..self.restitution_iterations as usize {
                self.apply_restitution(overflow, context, true);
                self.run_colors(SolverStageKind::Restitution);
            }
        }
        self.profile.restitution = get_milliseconds_and_reset(&mut ticks);

        self.store_wide_joint_impulses();
        self.store_impulses(overflow, true);
        self.run_colors(SolverStageKind::Store);
        self.profile.store_impulses = get_milliseconds_and_reset(&mut ticks);

        self.solver_views = SolverViews::default();
    }
}
