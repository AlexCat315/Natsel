//! Four independent point-to-point joints. Angular limits, springs and motors retain
//! their scalar kernels; their disabled-state cached impulses still participate in warm start.
//!
//! Translated from `Sources/Natsel/Simulation/WideJointSolver.swift`.
//!
//! Determinism: every lane operation goes through the `wide_*` helpers in `wide_types.rs` in
//! lane order, and the kernels below keep the scalar `solvePointConstraint` multiplication and
//! addition order exactly.
//!
//! Deviation from Swift: `SphericalConstraintW.invIA`/`invIB` are four-lane *symmetric* matrices
//! (`wide_types::SymMatrix3W`, written with the helper already provided for this consumer,
//! `wide_contact_solver::SymMatrix3W::set_lane`) instead of Swift's local full `Matrix3W`. The
//! values are bit-identical: both are filled from `JointSim.invIA`, the world inverse inertia,
//! which is symmetric on both stored triangles (the same reduction the landed wide contact
//! solver makes through `gather_inv_inertia_w`), and every kernel below reads the same operand
//! per multiply as Swift's `Matrix3W` columns.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::world_runtime`: `World` fields `enable_wide_solver`, `solver_joints`,
//!   `wide_joint_items`, `prepare_views`.
//! - `crate::simulation::solver_prepare`: `WideJointPrepareItem` and `ContactPrepareViews`.
//!   `prepare_views.joints` / `prepare_views.wide_joint_batches` are installed here;
//!   `prepare_all_joints` calls `World::prepare_wide_joints` after the scalar joint pass.
//! - `crate::simulation::solver_stages`: `SolverStageKind::PrepareWideJoints` dispatches to
//!   `World::prepare_wide_joint_range`, and `World::run_stage` / `PARALLEL_PREPARE_ITEMS`.
//! - `crate::simulation::wide_contact_solver`: `World::gather_states_w` / `World::scatter_states_w`
//!   and `SymMatrix3W::set_lane`.
//! - `crate::simulation::distance_joint_solver` and `crate::simulation::contact_solver`: call
//!   `warm_start_spherical_w` (by value), `solve_spherical_w` (inout + `use_bias`) and
//!   `store_wide_joint_impulses`.

use std::ops::Range;

use crate::base::constants as b3;
use crate::math::functions::mul_add;
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::contact_solver::SymMatrix3;
use crate::simulation::solver_prepare::WideJointPrepareItem;
use crate::simulation::solver_stages::SolverStageKind;
use crate::simulation::wide_types::{
    FloatW, QuatW, SymMatrix3W, Vec3W, add_w, cross_w, mul_add_w, mul_w, neg_w, select_vec3_w,
    select_w, sub_w, wide_gt, wide_lt,
};
use crate::simulation::world_runtime::{BodyState, JointSim, World};
use crate::types::joint_defs::JointType;

/// Swift's `mulMatrixW`: column-major product, each component summed left to right. Read through
/// the symmetric six-component lanes (`cx = (xx, xy, xz)`, `cy = (xy, yy, yz)`, `cz = (xz, yz, zz)`),
/// preserving the operand order of `m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z` and its rows.
#[inline(always)]
pub(crate) fn mul_matrix_w(m: SymMatrix3W, a: Vec3W) -> Vec3W {
    Vec3W::new(
        m.xx * a.x + m.xy * a.y + m.xz * a.z,
        m.xy * a.x + m.yy * a.y + m.yz * a.z,
        m.xz * a.x + m.yz * a.y + m.zz * a.z,
    )
}

#[inline(always)]
pub(crate) fn rotate_w(q: QuatW, v: Vec3W) -> Vec3W {
    let t1 = cross_w(q.v, v);
    let t2 = mul_add_w(t1, q.s, v);
    let t3 = cross_w(q.v, t2);
    mul_add_w(v, FloatW::splat(2.0), t3)
}

/// Keep every multiplication and addition in the scalar solvePointConstraint order.
/// The inverse-inertia columns of the symmetric lanes are `cx = (xx, xy, xz)`,
/// `cy = (xy, yy, yz)`, `cz = (xz, yz, zz)`; the column vectors below are Swift's `iA.cx` and
/// friends read lane by lane, so each multiply and subtract is the same pair of operands.
#[inline(always)]
pub(crate) fn solve_point_w(
    m_a: FloatW,
    m_b: FloatW,
    i_a: SymMatrix3W,
    i_b: SymMatrix3W,
    r_a: Vec3W,
    r_b: Vec3W,
    rhs: Vec3W,
) -> Vec3W {
    let axy = r_a.z * i_a.yy - r_a.y * i_a.yz;
    let axz = r_a.z * i_a.yz - r_a.y * i_a.zz;
    let a_cx = Vec3W::new(i_a.xx, i_a.xy, i_a.xz);
    let a_cy = Vec3W::new(i_a.xy, i_a.yy, i_a.yz);
    let a_cz = Vec3W::new(i_a.xz, i_a.yz, i_a.zz);
    let ay = sub_w(mul_w(r_a.x, a_cz), mul_w(r_a.z, a_cx));
    let az = sub_w(mul_w(r_a.y, a_cx), mul_w(r_a.x, a_cy));
    let bxy = r_b.z * i_b.yy - r_b.y * i_b.yz;
    let bxz = r_b.z * i_b.yz - r_b.y * i_b.zz;
    let b_cx = Vec3W::new(i_b.xx, i_b.xy, i_b.xz);
    let b_cy = Vec3W::new(i_b.xy, i_b.yy, i_b.yz);
    let b_cz = Vec3W::new(i_b.xz, i_b.yz, i_b.zz);
    let by = sub_w(mul_w(r_b.x, b_cz), mul_w(r_b.z, b_cx));
    let bz = sub_w(mul_w(r_b.y, b_cx), mul_w(r_b.x, b_cy));
    let mass = m_a + m_b;
    let kxx = (axy * r_a.z - axz * r_a.y) + (bxy * r_b.z - bxz * r_b.y) + mass;
    let kyy = (ay.z * r_a.x - ay.x * r_a.z) + (by.z * r_b.x - by.x * r_b.z) + mass;
    let kzz = (az.x * r_a.y - az.y * r_a.x) + (bz.x * r_b.y - bz.y * r_b.x) + mass;
    let kxy = (ay.y * r_a.z - ay.z * r_a.y) + (by.y * r_b.z - by.z * r_b.y);
    let kxz = (az.y * r_a.z - az.z * r_a.y) + (bz.y * r_b.z - bz.z * r_b.y);
    let kyz = (az.z * r_a.x - az.x * r_a.z) + (bz.z * r_b.x - bz.x * r_b.z);
    let cxx = kyy * kzz - kyz * kyz;
    let cyy = kxx * kzz - kxz * kxz;
    let czz = kxx * kyy - kxy * kxy;
    let cxy = kxz * kyz - kxy * kzz;
    let cxz = kxy * kyz - kyy * kxz;
    let cyz = kxy * kxz - kxx * kyz;
    let determinant = kxx * cxx + kxy * cxy + kxz * cxz;
    let abs_determinant = select_w(
        wide_lt(determinant, FloatW::ZERO),
        neg_w(determinant),
        determinant,
    );
    let valid = wide_gt(
        abs_determinant,
        FloatW::splat(1000.0 * f32::MIN_POSITIVE),
    );
    let inv = FloatW::ONE / determinant;
    let value = Vec3W::new(
        inv * (cxx * rhs.x + cxy * rhs.y + cxz * rhs.z),
        inv * (cxy * rhs.x + cyy * rhs.y + cyz * rhs.z),
        inv * (cxz * rhs.x + cyz * rhs.y + czz * rhs.z),
    );
    select_vec3_w(valid, value, Vec3W::zero())
}

/// One four-wide batch of plain spherical joints. The index lanes default to `-1`, the Swift
/// `SIMD4<Int32>(repeating: -1)` sentinel for an unused lane, so `Default` cannot be derived.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub(crate) struct SphericalConstraintW {
    pub(crate) joint_indices: [i32; 4],
    pub(crate) index_a: [i32; 4],
    pub(crate) index_b: [i32; 4],
    pub(crate) lane_count: usize,
    pub(crate) inv_mass_a: FloatW,
    pub(crate) inv_mass_b: FloatW,
    pub(crate) inv_ia: SymMatrix3W,
    pub(crate) inv_ib: SymMatrix3W,
    pub(crate) anchor_a: Vec3W,
    pub(crate) anchor_b: Vec3W,
    pub(crate) delta_center: Vec3W,
    pub(crate) bias_rate: FloatW,
    pub(crate) mass_scale: FloatW,
    pub(crate) impulse_scale: FloatW,
    pub(crate) linear_impulse: Vec3W,
    pub(crate) angular_impulse: Vec3W,
}

impl Default for SphericalConstraintW {
    #[inline]
    fn default() -> Self {
        Self {
            joint_indices: [-1; 4],
            index_a: [-1; 4],
            index_b: [-1; 4],
            lane_count: 0,
            inv_mass_a: FloatW::ZERO,
            inv_mass_b: FloatW::ZERO,
            inv_ia: SymMatrix3W::default(),
            inv_ib: SymMatrix3W::default(),
            anchor_a: Vec3W::default(),
            anchor_b: Vec3W::default(),
            delta_center: Vec3W::default(),
            bias_rate: FloatW::ZERO,
            mass_scale: FloatW::ZERO,
            impulse_scale: FloatW::ZERO,
            linear_impulse: Vec3W::default(),
            angular_impulse: Vec3W::default(),
        }
    }
}

/// Per-color wide joint batches and the unit plans that order them in the solver.
/// `units` holds the first joint of each solver unit (one scalar joint or one wide batch) when
/// the color has wide batches; empty means every joint is its own unit.
pub(crate) struct SolverJointData {
    pub batches: Vec<Vec<SphericalConstraintW>>,
    pub starts: Vec<Vec<isize>>,
    pub units: Vec<Vec<i32>>,
}

impl Default for SolverJointData {
    /// Swift's defaults: one empty list per graph color, so the derived default cannot be used.
    #[inline]
    fn default() -> Self {
        let colors = b3::GRAPH_COLOR_COUNT as usize;
        Self {
            batches: (0..colors).map(|_| Vec::new()).collect(),
            starts: (0..colors).map(|_| Vec::new()).collect(),
            units: (0..colors).map(|_| Vec::new()).collect(),
        }
    }
}

impl World {
    #[inline(always)]
    fn is_wide_spherical(base: *const JointSim) -> bool {
        // SAFETY: every caller passes a pointer into a graph color's `joint_sims` list, taken from
        // `as_ptr()` of an array that is not resized while the wide plan runs; the pointer is
        // aligned, non-null, and points to one live `JointSim`, exactly Swift's `UnsafePointer`.
        let sim = unsafe { &*base };
        if sim.type_ != JointType::SphericalJoint {
            return false;
        }
        let j = sim.spherical_joint();
        !j.enable_spring && !j.enable_motor && !j.enable_twist_limit && !j.enable_cone_limit
    }

    /// Pack runs of plain spherical joints into four-wide batches. The batch boundaries are planned
    /// on this thread; the batches are then filled in parallel. Requires `prepareViews.joints` from
    /// `prepareAllJoints`.
    pub(crate) fn prepare_wide_joints(&mut self) {
        self.wide_joint_items.clear();
        for ci in 0..b3::GRAPH_COLOR_COUNT as usize {
            self.solver_joints.batches[ci].clear();
            self.solver_joints.starts[ci].clear();
            self.solver_joints.units[ci].clear();
            if !self.enable_wide_solver || ci == OVERFLOW_INDEX as usize {
                continue;
            }
            let count = self.constraint_graph.colors[ci].joint_sims.len();
            if count < 4 {
                continue;
            }
            self.solver_joints.starts[ci].extend(std::iter::repeat(-1isize).take(count));

            let list = self.constraint_graph.colors[ci].joint_sims.as_ptr();
            let mut batches = 0usize;
            let mut i = 0usize;
            while i < count {
                // SAFETY: `list` addresses the color's `joint_sims` array of `count` live joints,
                // which the plan never resizes; `i < count` in every check below, and reading a
                // joint here mutates nothing, exactly Swift's `withUnsafeBufferPointer` scan.
                if !Self::is_wide_spherical(unsafe { list.add(i) }) {
                    i += 1;
                    continue;
                }
                let start = i;
                while i < count && Self::is_wide_spherical(unsafe { list.add(i) }) {
                    i += 1;
                }
                if i - start < 4 {
                    continue;
                }
                let mut first = start;
                while first < i {
                    let mut item = WideJointPrepareItem::new();
                    item.color = ci as i32;
                    item.first = first as i32;
                    item.lane_count = std::cmp::min(4, i - first) as i32;
                    item.batch = batches as i32;
                    self.wide_joint_items.push(item);
                    self.solver_joints.starts[ci][first] = batches as isize;
                    batches += 1;
                    first += 4;
                }
            }

            let mut placeholder = SphericalConstraintW::default();
            self.solver_joints.batches[ci].reserve(batches);
            // Swift iterated `wideJointItems.suffix(batches)` - the items just appended for this
            // color, since `batches` counts exactly this color's items and they are last.
            let suffix_start = self.wide_joint_items.len() - batches;
            for k in suffix_start..self.wide_joint_items.len() {
                placeholder.lane_count = self.wide_joint_items[k].lane_count as usize;
                self.solver_joints.batches[ci].push(placeholder);
            }
            self.build_joint_units(ci);
        }

        if self.wide_joint_items.is_empty() {
            return;
        }
        // Swift copied `prepareViews` into a local, filled the two wide fields, and assigned it
        // back; nothing reads the field in between, so the in-place writes are equivalent.
        self.prepare_views.wide_joint_items = self.wide_joint_items.as_ptr();
        self.prepare_views.wide_joint_batches.clear();
        for ci in 0..b3::GRAPH_COLOR_COUNT as usize {
            // Colors without batches yield an empty vector's base pointer; the fill stage only
            // dereferences an entry whose color produced at least one item, whose batch list is
            // non-empty by construction.
            let base = self.solver_joints.batches[ci].as_mut_ptr();
            self.prepare_views.wide_joint_batches.push(base);
        }
        let item_count = self.wide_joint_items.len();
        self.run_stage(
            SolverStageKind::PrepareWideJoints,
            0,
            item_count,
            Self::PARALLEL_PREPARE_ITEMS,
        );
    }

    /// Fill planned wide joint batches `range`.
    pub(crate) fn prepare_wide_joint_range(&self, range: Range<usize>) {
        let views = &self.prepare_views;
        let items = views.wide_joint_items;
        if items.is_null() {
            return;
        }
        for index in range {
            // SAFETY: `wide_joint_items` is the base of `World::wide_joint_items` installed by
            // `prepare_wide_joints` for the whole stage; the plan is not resized while the fill
            // runs and `index` is a block index below the installed item count.
            let item = unsafe { *items.add(index) };
            let ci = item.color as usize;
            let list = views.joints[ci];
            if list.is_null() {
                continue;
            }
            let out = views.wide_joint_batches[ci];
            if out.is_null() {
                continue;
            }
            let mut c = SphericalConstraintW::default();
            c.lane_count = item.lane_count as usize;
            for lane in 0..c.lane_count {
                // SAFETY: `joints[ci]` is the color's `joint_sims` base installed by
                // `prepare_all_joints`, and `item.first + lane` stays inside the run the planner
                // scanned (runs are `min(4, end - first)` lanes from a joint index below the
                // count). The list is not resized while the fill runs; each item writes only its
                // own batch slot, so no two threads alias this `JointSim` except through disjoint
                // lane reads.
                let base = unsafe { list.add(item.first as usize + lane) };
                // SAFETY: as above; one live `JointSim` read through the color's list pointer.
                let sim = unsafe { &*base };
                let joint = sim.spherical_joint();
                c.joint_indices[lane] = item.first + lane as i32;
                c.index_a[lane] = joint.index_a;
                c.index_b[lane] = joint.index_b;
                c.inv_mass_a[lane] = sim.inv_mass_a;
                c.inv_mass_b[lane] = sim.inv_mass_b;
                c.inv_ia.set_lane(lane, SymMatrix3::from_matrix(sim.inv_ia));
                c.inv_ib.set_lane(lane, SymMatrix3::from_matrix(sim.inv_ib));
                c.anchor_a.set_lane(lane, joint.frame_a.p);
                c.anchor_b.set_lane(lane, joint.frame_b.p);
                c.delta_center.set_lane(lane, joint.delta_center);
                c.bias_rate[lane] = sim.constraint_softness.bias_rate;
                c.mass_scale[lane] = sim.constraint_softness.mass_scale;
                c.impulse_scale[lane] = sim.constraint_softness.impulse_scale;
                c.linear_impulse.set_lane(lane, joint.linear_impulse);
                let mut angular = joint.spring_impulse + joint.motor_impulse;
                angular = angular - joint.swing_impulse * joint.swing_axis;
                angular = mul_add(
                    angular,
                    joint.lower_twist_impulse - joint.upper_twist_impulse,
                    joint.twist_jacobian,
                );
                c.angular_impulse.set_lane(lane, angular);
            }
            // SAFETY: `wide_joint_batches[ci]` is the base of `solver_joints.batches[ci]`, which
            // `prepare_wide_joints` reserved and filled with one placeholder per planned batch of
            // this color; `item.batch` is that plan's own index, so the slot is live and owned by
            // exactly this item.
            unsafe { *out.add(item.batch as usize) = c };
        }
    }

    /// Record the first joint of each solver unit for one color: a wide batch consumes its lane
    /// count, a scalar joint consumes one.
    fn build_joint_units(&mut self, ci: usize) {
        let starts_count = self.solver_joints.starts[ci].len();
        if starts_count == 0 {
            return;
        }
        let mut i = 0usize;
        while i < starts_count {
            let start = self.solver_joints.starts[ci][i];
            let step = if start >= 0 {
                self.solver_joints.batches[ci][start as usize].lane_count
            } else {
                1
            };
            self.solver_joints.units[ci].push(i as i32);
            i += step;
        }
    }

    pub(crate) fn warm_start_spherical_w(&self, c: SphericalConstraintW, states: &mut [BodyState]) {
        let mut a = World::gather_states_w(states, c.index_a);
        let mut b = World::gather_states_w(states, c.index_b);
        let r_a = rotate_w(a.dq, c.anchor_a);
        let r_b = rotate_w(b.dq, c.anchor_b);
        a.v = sub_w(a.v, mul_w(c.inv_mass_a, c.linear_impulse));
        a.w = sub_w(
            a.w,
            mul_matrix_w(c.inv_ia, add_w(cross_w(r_a, c.linear_impulse), c.angular_impulse)),
        );
        b.v = add_w(b.v, mul_w(c.inv_mass_b, c.linear_impulse));
        b.w = add_w(
            b.w,
            mul_matrix_w(c.inv_ib, add_w(cross_w(r_b, c.linear_impulse), c.angular_impulse)),
        );
        World::scatter_states_w(states, c.index_a, a);
        World::scatter_states_w(states, c.index_b, b);
    }

    pub(crate) fn solve_spherical_w(
        &self,
        c: &mut SphericalConstraintW,
        states: &mut [BodyState],
        use_bias: bool,
    ) {
        let mut a = World::gather_states_w(states, c.index_a);
        let mut b = World::gather_states_w(states, c.index_b);
        let r_a = rotate_w(a.dq, c.anchor_a);
        let r_b = rotate_w(b.dq, c.anchor_b);
        let cdot = sub_w(
            sub_w(add_w(b.v, cross_w(b.w, r_b)), a.v),
            cross_w(a.w, r_a),
        );
        let mut bias = Vec3W::default();
        let mut mass = FloatW::splat(1.0);
        let mut scale = FloatW::ZERO;
        if use_bias {
            let separation =
                add_w(add_w(sub_w(b.dp, a.dp), sub_w(r_b, r_a)), c.delta_center);
            bias = mul_w(c.bias_rate, separation);
            mass = c.mass_scale;
            scale = c.impulse_scale;
        }
        let solution = solve_point_w(
            c.inv_mass_a,
            c.inv_mass_b,
            c.inv_ia,
            c.inv_ib,
            r_a,
            r_b,
            add_w(cdot, bias),
        );
        let impulse = sub_w(mul_w(neg_w(mass), solution), mul_w(scale, c.linear_impulse));
        c.linear_impulse = add_w(c.linear_impulse, impulse);
        a.v = sub_w(a.v, mul_w(c.inv_mass_a, impulse));
        a.w = sub_w(a.w, mul_matrix_w(c.inv_ia, cross_w(r_a, impulse)));
        b.v = add_w(b.v, mul_w(c.inv_mass_b, impulse));
        b.w = add_w(b.w, mul_matrix_w(c.inv_ib, cross_w(r_b, impulse)));
        World::scatter_states_w(states, c.index_a, a);
        World::scatter_states_w(states, c.index_b, b);
    }

    pub(crate) fn store_wide_joint_impulses(&mut self) {
        if !self.enable_wide_solver {
            return;
        }
        for ci in 0..OVERFLOW_INDEX as usize {
            let batches = &self.solver_joints.batches[ci];
            if batches.is_empty() {
                continue;
            }
            let list = self.constraint_graph.colors[ci].joint_sims.as_mut_ptr();
            for c in batches {
                for lane in 0..c.lane_count {
                    // SAFETY: `joint_indices[lane]` was written during the fill from `item.first`
                    // of this color's plan, so it indexes a live `JointSim` in the color's list,
                    // which is not resized while impulses are stored. Swift wrote the field in
                    // place; the payload accessor read/modify/write below lands the same bits.
                    unsafe {
                        let sim = &mut *list.add(c.joint_indices[lane] as usize);
                        let mut joint = sim.spherical_joint();
                        joint.linear_impulse = c.linear_impulse.lane(lane);
                        sim.set_spherical_joint(joint);
                    }
                }
            }
        }
    }
}
