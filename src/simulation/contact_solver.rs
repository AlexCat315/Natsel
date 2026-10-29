//! Convex and mesh contact solver.
//!
//! Translated from `Sources/Natsel/Simulation/ContactSolver.swift`.
//!
//! The solver batches four independent constraints per SIMD group. Graph coloring guarantees the
//! bodies in a color are disjoint, so the lanes never interact. Each lane evaluates the same
//! per-lane arithmetic in the same color/contact order. `wide_contact_solver.rs` packs disjoint
//! contacts into four SIMD lanes; this file also supplies the sequential overflow and fallback paths.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::wide_contact_solver`: `ContactConstraintW`, `ManifoldConstraintW`, and the
//!   World methods `uses_wide_contacts`, `warm_start_contacts_wide`, `push_contacts_wide`,
//!   `solve_contacts_wide`, `apply_restitution_wide`, `store_impulses_wide` (same argument order as
//!   the Swift wide entry points they replace).
//! - `crate::simulation::overflow_contact_solver`: `warm_start_overflow_contacts`,
//!   `push_overflow_contacts`, `solve_overflow_contacts`, `apply_overflow_restitution`,
//!   `store_overflow_impulses`.
//! - `crate::simulation::distance_joint_solver` / `wide_joint_solver`: `warm_start_joints`,
//!   `solve_joints`, `store_wide_joint_impulses`.

use std::ops::Range;

use crate::base::constants as b3;
use crate::collision::internal_math::{Matrix2, mul_mv2};
use crate::math::functions::{
    abs_float, clamp_float, cross, dot, max_float, min_float, min_int, mul_add, perp,
};
use crate::math::types::{Matrix3, Quat, Vec2, Vec3};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::solver_integration::{Softness, StepContext, mutable_buffer_view};
use crate::simulation::wide_contact_solver::{ContactConstraintW, ManifoldConstraintW};
use crate::simulation::world_runtime::{
    BodyFlags, BodySim, BodyState, Contact, SetType, World, decode_awake_index,
};

// MARK: - Symmetric 3x3

/// Symmetric 3x3 matrix in six-component form for the wide solver.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SymMatrix3 {
    pub cxx: f32,
    pub cxy: f32,
    pub cxz: f32,
    pub cyy: f32,
    pub cyz: f32,
    pub czz: f32,
}

impl SymMatrix3 {
    /// `init()`
    #[inline]
    pub const fn new() -> Self {
        Self {
            cxx: 0.0,
            cxy: 0.0,
            cxz: 0.0,
            cyy: 0.0,
            cyz: 0.0,
            czz: 0.0,
        }
    }

    /// `init(cxx:cxy:cxz:cyy:cyz:czz:)`
    #[inline]
    pub const fn with_components(
        cxx: f32,
        cxy: f32,
        cxz: f32,
        cyy: f32,
        cyz: f32,
        czz: f32,
    ) -> Self {
        Self {
            cxx,
            cxy,
            cxz,
            cyy,
            cyz,
            czz,
        }
    }

    /// `init(_ m: Matrix3)` - the wide solver gathers the upper triangle of the matrix.
    #[inline]
    pub const fn from_matrix(m: Matrix3) -> Self {
        Self {
            cxx: m.cx.x,
            cxy: m.cx.y,
            cxz: m.cx.z,
            cyy: m.cy.y,
            cyz: m.cy.z,
            czz: m.cz.z,
        }
    }

    /// `m * a`, preserving the wide solver association order.
    #[inline(always)]
    pub fn mul_mv(self, a: Vec3) -> Vec3 {
        Vec3::new(
            self.cxx * a.x + (self.cxy * a.y + self.cxz * a.z),
            self.cxy * a.x + (self.cyy * a.y + self.cyz * a.z),
            self.cxz * a.x + (self.cyz * a.y + self.czz * a.z),
        )
    }
}

impl From<Matrix3> for SymMatrix3 {
    /// `init(_ m: Matrix3)`
    #[inline]
    fn from(m: Matrix3) -> Self {
        SymMatrix3::from_matrix(m)
    }
}

#[inline(always)]
pub fn add_sym(a: SymMatrix3, b: SymMatrix3) -> SymMatrix3 {
    SymMatrix3::with_components(
        a.cxx + b.cxx,
        a.cxy + b.cxy,
        a.cxz + b.cxz,
        a.cyy + b.cyy,
        a.cyz + b.cyz,
        a.czz + b.czz,
    )
}

/// Invert a symmetric matrix for the wide solver.
pub fn invert_sym(m: SymMatrix3) -> SymMatrix3 {
    let cxx = m.cyy * m.czz - m.cyz * m.cyz;
    let cxy = m.cxz * m.cyz - m.cxy * m.czz;
    let cxz = m.cxy * m.cyz - m.cxz * m.cyy;
    let cyy = m.cxx * m.czz - m.cxz * m.cxz;
    let cyz = m.cxy * m.cxz - m.cxx * m.cyz;
    let czz = m.cxx * m.cyy - m.cxy * m.cxy;

    let det = m.cxx * cxx + (m.cxy * cxy + m.cxz * cxz);
    let valid = abs_float(det) > 1000.0 * f32::MIN_POSITIVE;
    let inv_det: f32 = if valid { 1.0 / det } else { 0.0 };

    SymMatrix3::with_components(
        inv_det * cxx,
        inv_det * cxy,
        inv_det * cxz,
        inv_det * cyy,
        inv_det * cyz,
        inv_det * czz,
    )
}

// MARK: - Constraints

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ConstraintPoint {
    pub r_a: Vec3,
    pub r_b: Vec3,
    pub base_separation: f32,
    pub relative_velocity: f32,
    pub normal_impulse: f32,
    pub total_normal_impulse: f32,
    pub restitution_impulse: f32,
    pub normal_mass: f32,
    pub lever_arm: f32,
}

impl ConstraintPoint {
    /// `init()`
    #[inline]
    pub const fn new() -> Self {
        Self {
            r_a: Vec3::new(0.0, 0.0, 0.0),
            r_b: Vec3::new(0.0, 0.0, 0.0),
            base_separation: 0.0,
            relative_velocity: 0.0,
            normal_impulse: 0.0,
            total_normal_impulse: 0.0,
            restitution_impulse: 0.0,
            normal_mass: 0.0,
            lever_arm: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ManifoldConstraint {
    pub points: [ConstraintPoint; 4],
    pub point_count: i32,
    pub normal: Vec3,
    pub tangent1: Vec3,
    pub tangent2: Vec3,

    /// Friction centers
    pub center_a: Vec3,
    pub center_b: Vec3,
    pub twist_mass: f32,
    pub twist_impulse: f32,
    pub tangent_mass: Matrix2,
    pub friction_impulse: Vec2,
    pub rolling_impulse: Vec3,
    pub tangent_velocity1: f32,
    pub tangent_velocity2: f32,
}

impl ManifoldConstraint {
    /// `init()`
    #[inline]
    pub fn new() -> Self {
        Self {
            points: [ConstraintPoint::new(); 4],
            point_count: 0,
            normal: Vec3::new(0.0, 0.0, 0.0),
            tangent1: Vec3::new(0.0, 0.0, 0.0),
            tangent2: Vec3::new(0.0, 0.0, 0.0),
            center_a: Vec3::new(0.0, 0.0, 0.0),
            center_b: Vec3::new(0.0, 0.0, 0.0),
            twist_mass: 0.0,
            twist_impulse: 0.0,
            tangent_mass: Matrix2::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0)),
            friction_impulse: Vec2::new(0.0, 0.0),
            rolling_impulse: Vec3::new(0.0, 0.0, 0.0),
            tangent_velocity1: 0.0,
            tangent_velocity2: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct ContactConstraint {
    pub contact_id: i32,
    pub index_a: i32,
    pub index_b: i32,
    pub inv_mass_a: f32,
    pub inv_mass_b: f32,
    pub inv_ia: SymMatrix3,
    pub inv_ib: SymMatrix3,
    pub rolling_mass: SymMatrix3,
    pub softness: Softness,
    pub friction: f32,
    pub restitution: f32,
    pub rolling_resistance: f32,
    pub is_static: bool,
    /// Index of this constraint's first manifold in the solver's flat manifold pool. Storing an
    /// index instead of an array keeps the constraint a plain value type: copying it no longer
    /// retains a heap buffer, which dominated the solver profile.
    pub manifold_start: i32,
    /// Number of manifolds this constraint owns in the flat pool.
    pub manifold_count: i32,
}

impl ContactConstraint {
    /// `init()`
    #[inline]
    pub const fn new() -> Self {
        Self {
            contact_id: b3::NULL_INDEX,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            inv_mass_a: 0.0,
            inv_mass_b: 0.0,
            inv_ia: SymMatrix3::new(),
            inv_ib: SymMatrix3::new(),
            rolling_mass: SymMatrix3::new(),
            softness: Softness::new(),
            friction: 0.0,
            restitution: 0.0,
            rolling_resistance: 0.0,
            is_static: false,
            manifold_start: 0,
            manifold_count: 0,
        }
    }
}

impl Default for ContactConstraint {
    #[inline(always)]
    fn default() -> Self {
        ContactConstraint::new()
    }
}

// MARK: - Solver

/// Inverse rotate used by the wide solver. This is a different formulation from the scalar
/// `inv_rotate_vector` and is not bit-equivalent to it.
#[inline(always)]
pub fn inv_rotate_vector_wide(q: Quat, a: Vec3) -> Vec3 {
    let t0 = cross(q.v, a);
    let t = Vec3::new(t0.x + t0.x, t0.y + t0.y, t0.z + t0.z);
    let u = cross(q.v, t);
    Vec3::new(
        (a.x - q.s * t.x) + u.x,
        (a.y - q.s * t.y) + u.y,
        (a.z - q.s * t.z) + u.z,
    )
}

/// The solver constraint data for each graph color, rebuilt each step.
pub struct SolverContactData {
    pub(crate) constraints: Vec<Vec<ContactConstraint>>,

    /// Flat pool of manifold constraints. Each `ContactConstraint` owns a contiguous run
    /// starting at its `manifold_start`.
    pub(crate) manifold_pool: Vec<ManifoldConstraint>,

    /// Number of convex constraints at the front of each color's constraint list. The remaining
    /// constraints are mesh contacts.
    pub(crate) convex_counts: Vec<i32>,

    // Packed once per step and reused by every substep. Overflow remains sequential.
    pub(crate) wide_constraints: Vec<Vec<ContactConstraintW>>,
    pub(crate) wide_manifolds: Vec<ManifoldConstraintW>,
    pub(crate) wide_convex_counts: Vec<i32>,
    pub(crate) wide_convex: Vec<bool>,
    pub(crate) wide_mesh: Vec<bool>,
}

impl Default for SolverContactData {
    #[inline]
    fn default() -> Self {
        let color_count = b3::GRAPH_COLOR_COUNT as usize;
        Self {
            constraints: vec![Vec::new(); color_count],
            manifold_pool: Vec::new(),
            convex_counts: vec![0; color_count],
            wide_constraints: vec![Vec::new(); color_count],
            wide_manifolds: Vec::new(),
            wide_convex_counts: vec![0; color_count],
            wide_convex: vec![false; color_count],
            wide_mesh: vec![false; color_count],
        }
    }
}

impl SolverContactData {
    /// The index range of the convex or mesh constraints for a color.
    pub fn range(&self, color_index: i32, mesh: bool) -> Range<usize> {
        let count = self.constraints[color_index as usize].len();
        let convex = self.convex_counts[color_index as usize] as usize;
        if mesh { convex..count } else { 0..convex }
    }
}

impl World {
    /// Read one awake body state.
    ///
    /// The element is addressed through one array rather than through `solverSets[...]`, because
    /// subscripting the set copies the whole `SolverSet` and retains all four of its arrays.
    #[inline(always)]
    pub(crate) fn solver_state(&self, index: i32) -> BodyState {
        if index < 0 {
            return BodyState::IDENTITY;
        }
        self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states[index as usize]
    }

    #[inline(always)]
    pub(crate) fn store_solver_state(&mut self, index: i32, state: BodyState) {
        if index < 0 || (state.flags & BodyFlags::DYNAMIC_FLAG.0) == 0 {
            return;
        }
        self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states[index as usize] = state;
    }

    /// Read a state from a borrowed state array. `index < 0` is a static body, which contributes
    /// no velocity, so identity is the correct stand-in.
    #[inline(always)]
    pub(crate) fn state_at(states: *mut BodyState, index: i32) -> BodyState {
        if index >= 0 {
            // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
            // awake set's state array, lent for one solver phase, and a constraint's awake index is
            // always inside it; the phase neither resizes nor frees the array.
            unsafe { *states.add(index as usize) }
        } else {
            BodyState::IDENTITY
        }
    }

    /// Write a state back into a borrowed state array. Static bodies are never written.
    #[inline(always)]
    pub(crate) fn set_state_at(states: *mut BodyState, index: i32, state: BodyState) {
        if index >= 0 && (state.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            // SAFETY: as in [`World::state_at`]; only the constraint's own two body slots are
            // written, and the graph color keeps those bodies disjoint from every other lane.
            unsafe { states.add(index as usize).write(state) };
        }
    }

    /// Lend the awake set's body state array for one solver phase.
    ///
    /// Indexing `solverSets[...]` copies the whole `SolverSet`, which retains four arrays; doing that
    /// twice per constraint (read plus store) was the dominant solver cost. The closure receives the
    /// states in place, and nothing inside a contact phase may resize the awake set.
    #[inline(always)]
    pub(crate) fn with_solver_states<R, F>(&mut self, body: F) -> R
    where
        F: FnOnce(&mut [BodyState]) -> R,
    {
        body(self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states.as_mut_slice())
    }

    /// Convex contact ids for a color, in solver order.
    pub(crate) fn solver_convex_contact_ids(&self, color_index: i32) -> Vec<i32> {
        self.constraint_graph.colors[color_index as usize].convex_contacts.clone()
    }

    /// Mesh contact ids for a color, in solver order. The solver sorts these by manifold count so
    /// contacts with more manifolds are grouped into the same SIMD lanes.
    pub(crate) fn solver_mesh_contact_ids(&self, color_index: i32) -> Vec<i32> {
        let color = &self.constraint_graph.colors[color_index as usize];
        if color_index == OVERFLOW_INDEX {
            return color.contacts.iter().map(|spec| spec.contact_id).collect();
        }
        let maximum = color
            .contacts
            .iter()
            .map(|spec| spec.manifold_count as usize)
            .max()
            .unwrap_or(0);
        let mut buckets: Vec<Vec<i32>> = vec![Vec::new(); maximum + 1];
        for spec in &color.contacts {
            buckets[spec.manifold_count as usize].push(spec.contact_id);
        }
        buckets
            .iter()
            .rev()
            .flat_map(|bucket| bucket.iter().copied())
            .collect()
    }

    /// All contact ids for a color in solver order: convex first, then mesh.
    pub(crate) fn solver_contact_ids(&self, color_index: i32) -> Vec<i32> {
        let mut ids = self.solver_convex_contact_ids(color_index);
        ids.extend(self.solver_mesh_contact_ids(color_index));
        ids
    }

    /// Prepare one scalar contact constraint into its preassigned slots. Its manifolds go to
    /// `manifold_out`, which is uninitialized memory with room for the contact's manifold count.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn prepare_contact_constraint(
        &self,
        contact_id: i32,
        out: *mut ContactConstraint,
        manifold_out: *mut ManifoldConstraint,
        manifold_start: i32,
        contact_store: *const Contact,
        body_sims: &[BodySim],
        body_states: &mut [BodyState],
        softness: Softness,
        static_softness: Softness,
        warm_start_scale: f32,
    ) {
        let inv_tau = 1.0 / b3::speculative_distance();
        let min_friction_weight = b3::MIN_FRICTION_WEIGHT;
        // SAFETY: `contact_id` indexes the world's contact store, which `prepare_all_contacts`
        // sized before this stage and does not resize while any constraint is prepared, exactly as
        // Swift's `contactStore + Int(contactId)` assumed.
        let contact = unsafe { &*contact_store.add(contact_id as usize) };
        let tangent_velocity = contact.tangent_velocity;

        let mut constraint = ContactConstraint::new();
        constraint.contact_id = contact_id;
        constraint.manifold_start = manifold_start;

        let index_a = decode_awake_index(contact.encoded_body_sim_a);
        let index_b = decode_awake_index(contact.encoded_body_sim_b);
        constraint.index_a = index_a;
        constraint.index_b = index_b;

        // Read the few scalar fields directly from the borrowed body-sim buffer. Taking a
        // BodySim by value here copied several hundred bytes for every contact.
        if index_a >= 0 {
            let sim = &body_sims[index_a as usize];
            constraint.inv_mass_a = sim.inv_mass;
            constraint.inv_ia = SymMatrix3::from_matrix(sim.inv_inertia_world);
        } else {
            constraint.inv_mass_a = 0.0;
            constraint.inv_ia = SymMatrix3::new();
        }
        if index_b >= 0 {
            let sim = &body_sims[index_b as usize];
            constraint.inv_mass_b = sim.inv_mass;
            constraint.inv_ib = SymMatrix3::from_matrix(sim.inv_inertia_world);
        } else {
            constraint.inv_mass_b = 0.0;
            constraint.inv_ib = SymMatrix3::new();
        }
        constraint.friction = contact.friction;
        constraint.restitution = contact.restitution;
        constraint.rolling_resistance = contact.rolling_resistance;
        constraint.is_static = (contact.flags & ContactFlags::STATIC_FLAG.0) != 0;
        constraint.softness = if constraint.is_static {
            static_softness
        } else {
            softness
        };

        let inv_iab = add_sym(constraint.inv_ia, constraint.inv_ib);
        if constraint.rolling_resistance > 0.0 {
            constraint.rolling_mass = invert_sym(inv_iab);
        } else {
            constraint.rolling_mass = SymMatrix3::new();
        }

        let manifolds = contact.manifolds();
        let manifold_end = manifolds.count();
        let mut manifold_index = 0usize;
        while manifold_index < manifold_end {
            let manifold = *manifolds.get(manifold_index);
            manifold_index += 1;

            let mut manifold_constraint = ManifoldConstraint::new();
            let point_count = manifold.point_count;
            manifold_constraint.point_count = point_count;
            manifold_constraint.normal = manifold.normal;
            manifold_constraint.tangent1 = perp(manifold.normal);
            manifold_constraint.tangent2 = cross(manifold_constraint.tangent1, manifold.normal);

            manifold_constraint.tangent_velocity1 = dot(tangent_velocity, manifold_constraint.tangent1);
            manifold_constraint.tangent_velocity2 = dot(tangent_velocity, manifold_constraint.tangent2);

            manifold_constraint.twist_impulse = warm_start_scale * manifold.twist_impulse;
            if constraint.rolling_resistance > 0.0 {
                manifold_constraint.rolling_impulse = warm_start_scale * manifold.rolling_impulse;
            } else {
                manifold_constraint.rolling_impulse = Vec3::new(0.0, 0.0, 0.0);
            }

            manifold_constraint.friction_impulse = Vec2::new(
                warm_start_scale * dot(manifold.friction_impulse, manifold_constraint.tangent1),
                warm_start_scale * dot(manifold.friction_impulse, manifold_constraint.tangent2),
            );

            let normal = manifold_constraint.normal;
            let tangent1 = manifold_constraint.tangent1;
            let tangent2 = manifold_constraint.tangent2;

            let mut center_a = Vec3::new(0.0, 0.0, 0.0);
            let mut center_b = Vec3::new(0.0, 0.0, 0.0);
            let mut total_weight: f32 = 0.0;

            for point_index in 0..point_count as usize {
                let mp = manifold.points[point_index];
                let mut cp = ConstraintPoint::new();

                cp.r_a = mp.anchor_a;
                cp.r_b = mp.anchor_b;
                cp.base_separation = mp.separation - dot(cp.r_b - cp.r_a, normal);
                cp.normal_impulse = warm_start_scale * mp.normal_impulse;
                cp.total_normal_impulse = 0.0;
                cp.relative_velocity = 0.0;
                cp.restitution_impulse = 0.0;

                let weight = clamp_float(2.0 - mp.separation * inv_tau, min_friction_weight, 1.0);
                center_a = center_a + weight * cp.r_a;
                center_b = center_b + weight * cp.r_b;
                total_weight += weight;

                let rn_a = cross(cp.r_a, normal);
                let rn_b = cross(cp.r_b, normal);

                let mut k_normal = constraint.inv_mass_a + constraint.inv_mass_b;
                k_normal += dot(rn_a, constraint.inv_ia.mul_mv(rn_a));
                k_normal += dot(rn_b, constraint.inv_ib.mul_mv(rn_b));
                cp.normal_mass = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

                manifold_constraint.points[point_index] = cp;
            }

            if total_weight > 0.0 {
                center_a = (1.0 / total_weight) * center_a;
                center_b = (1.0 / total_weight) * center_b;
                manifold_constraint.center_a = center_a;
                manifold_constraint.center_b = center_b;
            } else {
                manifold_constraint.center_a = Vec3::new(0.0, 0.0, 0.0);
                manifold_constraint.center_b = Vec3::new(0.0, 0.0, 0.0);
            }

            // Lever arms use the friction center.
            for point_index in 0..point_count as usize {
                let mut cp = manifold_constraint.points[point_index];
                let d = cp.r_a - manifold_constraint.center_a;
                cp.lever_arm = dot(d, d).sqrt();
                manifold_constraint.points[point_index] = cp;
            }

            // Compute effective mass for the tangential friction. The wide solver evaluates this
            // once per manifold, not per point.
            {
                let rt_a1 = cross(manifold_constraint.center_a, tangent1);
                let rt_a2 = cross(manifold_constraint.center_a, tangent2);
                let rt_b1 = cross(manifold_constraint.center_b, tangent1);
                let rt_b2 = cross(manifold_constraint.center_b, tangent2);

                let i_a_rt_a1 = constraint.inv_ia.mul_mv(rt_a1);
                let i_a_rt_a2 = constraint.inv_ia.mul_mv(rt_a2);
                let i_b_rt_b1 = constraint.inv_ib.mul_mv(rt_b1);
                let i_b_rt_b2 = constraint.inv_ib.mul_mv(rt_b2);

                let m_sum = constraint.inv_mass_a + constraint.inv_mass_b;
                let kxx = (m_sum + dot(rt_a1, i_a_rt_a1)) + dot(rt_b1, i_b_rt_b1);
                let kyy = (m_sum + dot(rt_a2, i_a_rt_a2)) + dot(rt_b2, i_b_rt_b2);
                let kxy = dot(rt_a1, i_a_rt_a2) + dot(rt_b1, i_b_rt_b2);

                let det = kxx * kyy - kxy * kxy;
                let valid =
                    abs_float(det) > 1000.0 * f32::MIN_POSITIVE && total_weight > 0.0;
                let inv_det: f32 = if valid { 1.0 / det } else { 0.0 };

                let mut tangent_mass = Matrix2::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
                tangent_mass.cx = Vec2::new(inv_det * kyy, -(inv_det * kxy));
                tangent_mass.cy = Vec2::new(-(inv_det * kxy), inv_det * kxx);
                manifold_constraint.tangent_mass = tangent_mass;
            }

            let k_twist = dot(normal, inv_iab.mul_mv(normal));
            manifold_constraint.twist_mass = if k_twist > 0.0 { 1.0 / k_twist } else { 0.0 };

            // Sample relative velocities if needed for restitution or hit events.
            let have_restitution = contact.restitution > 0.0;
            let hit_events = (contact.flags & ContactFlags::SIM_ENABLE_HIT_EVENT.0) != 0;
            if have_restitution || hit_events {
                let state_a = if index_a >= 0 {
                    body_states[index_a as usize]
                } else {
                    BodyState::IDENTITY
                };
                let state_b = if index_b >= 0 {
                    body_states[index_b as usize]
                } else {
                    BodyState::IDENTITY
                };

                for point_index in 0..point_count as usize {
                    let mut cp = manifold_constraint.points[point_index];
                    let vr_a = state_a.linear_velocity + cross(state_a.angular_velocity, cp.r_a);
                    let vr_b = state_b.linear_velocity + cross(state_b.angular_velocity, cp.r_b);
                    cp.relative_velocity = dot(normal, vr_b - vr_a);
                    manifold_constraint.points[point_index] = cp;
                }
            }

            // SAFETY: `manifold_out` addresses this contact's preassigned run in the scalar manifold
            // pool, sized to the contact's manifold count by `plan_contact_preparation`, and every
            // slot is written exactly once in order.
            unsafe {
                manifold_out
                    .add(constraint.manifold_count as usize)
                    .write(manifold_constraint)
            };
            constraint.manifold_count += 1;
        }
        // SAFETY: `out` is this item's uninitialized constraint slot, assigned by
        // `plan_contact_preparation` and written by exactly this one prepare item.
        unsafe { out.write(constraint) };
    }

    // MARK: Warm start

    pub(crate) fn warm_start_contacts(&mut self, color_index: i32, mesh: bool) {
        if color_index == OVERFLOW_INDEX {
            if mesh {
                self.warm_start_overflow_contacts();
            }
            return;
        }
        if self.uses_wide_contacts(color_index, mesh) {
            self.warm_start_contacts_wide(color_index, mesh);
            return;
        }
        let range = self.solver_contacts.range(color_index, mesh);
        if range.is_empty() {
            return;
        }
        let list = self.solver_contacts.constraints[color_index as usize].as_ptr();
        let awake = SetType::AwakeSet.raw_value() as usize;
        let states_ptr = self.solver_sets[awake].body_states.as_mut_ptr();
        let states_count = self.solver_sets[awake].body_states.len();
        let pool_ptr = self.solver_contacts.manifold_pool.as_mut_ptr();
        let pool_count = self.solver_contacts.manifold_pool.len();
        // SAFETY: mirrors Swift's nested `withUnsafeMutableBufferPointer` pair. The awake set's state
        // array and the scalar manifold pool are not resized during a contact phase, they are two
        // disjoint arrays, and this phase is the only live borrow of either.
        unsafe {
            let states = mutable_buffer_view(states_ptr, states_count);
            let pool = mutable_buffer_view(pool_ptr, pool_count);
            self.warm_start_contacts_body(range, list, states, pool);
        }
    }

    pub(crate) fn warm_start_contacts_body(
        &self,
        range: Range<usize>,
        list: *const ContactConstraint,
        states: &mut [BodyState],
        pool: &mut [ManifoldConstraint],
    ) {
        // SAFETY: Swift handed the same borrowed state buffer to `World.stateAt` / `setStateAt`.
        // `states` is the awake set's state array for this phase only, is not resized while any
        // contact phase runs, and every index used below comes from a decoded awake index.
        let states_base = states.as_mut_ptr();
        for constraint_index in range {
            // SAFETY: `list` addresses the color's constraint array and the range comes from that
            // array's own length, exactly as Swift's `UnsafePointer` subscript did.
            let c = unsafe { *list.add(constraint_index) };

            let mut linear_impulse = Vec3::new(0.0, 0.0, 0.0);
            let mut angular_impulse_a = Vec3::new(0.0, 0.0, 0.0);
            let mut angular_impulse_b = Vec3::new(0.0, 0.0, 0.0);

            for manifold_offset in 0..c.manifold_count as usize {
                let cm = &mut pool[c.manifold_start as usize + manifold_offset];
                let mut total_normal_impulse: f32 = 0.0;
                let mut moment_a = Vec3::new(0.0, 0.0, 0.0);
                let mut moment_b = Vec3::new(0.0, 0.0, 0.0);

                for point_index in 0..cm.point_count as usize {
                    let cp = cm.points[point_index];
                    total_normal_impulse += cp.normal_impulse;
                    moment_a = mul_add(moment_a, cp.normal_impulse, cp.r_a);
                    moment_b = mul_add(moment_b, cp.normal_impulse, cp.r_b);
                }

                let normal = cm.normal;
                let mut friction_impulse = cm.friction_impulse.x * cm.tangent1;
                friction_impulse = mul_add(friction_impulse, cm.friction_impulse.y, cm.tangent2);

                let linear = mul_add(friction_impulse, total_normal_impulse, normal);

                let twist_impulse = cm.twist_impulse * normal;
                let mut angular_a =
                    cross(moment_a, normal) + cross(cm.center_a, friction_impulse) + twist_impulse;
                let mut angular_b =
                    cross(moment_b, normal) + cross(cm.center_b, friction_impulse) + twist_impulse;

                if c.rolling_resistance > 0.0 {
                    angular_a = angular_a + cm.rolling_impulse;
                    angular_b = angular_b + cm.rolling_impulse;
                }

                linear_impulse = linear_impulse + linear;
                angular_impulse_a = angular_impulse_a + angular_a;
                angular_impulse_b = angular_impulse_b + angular_b;
            }

            let mut b_a = World::state_at(states_base, c.index_a);
            let mut b_b = World::state_at(states_base, c.index_b);

            b_a.angular_velocity = b_a.angular_velocity - c.inv_ia.mul_mv(angular_impulse_a);
            b_a.linear_velocity = b_a.linear_velocity - c.inv_mass_a * linear_impulse;
            b_b.angular_velocity = b_b.angular_velocity + c.inv_ib.mul_mv(angular_impulse_b);
            b_b.linear_velocity = b_b.linear_velocity + c.inv_mass_b * linear_impulse;

            World::set_state_at(states_base, c.index_a, b_a);
            World::set_state_at(states_base, c.index_b, b_b);
        }
    }

    // MARK: Push (non-penetration with soft bias)

    pub(crate) fn push_contacts(&mut self, color_index: i32, context: &StepContext, mesh: bool) {
        if color_index == OVERFLOW_INDEX {
            if mesh {
                self.push_overflow_contacts(context);
            }
            return;
        }
        if self.uses_wide_contacts(color_index, mesh) {
            self.push_contacts_wide(color_index, context, mesh);
            return;
        }
        let range = self.solver_contacts.range(color_index, mesh);
        if range.is_empty() {
            return;
        }
        let list = self.solver_contacts.constraints[color_index as usize].as_ptr();
        let awake = SetType::AwakeSet.raw_value() as usize;
        let states_ptr = self.solver_sets[awake].body_states.as_mut_ptr();
        let states_count = self.solver_sets[awake].body_states.len();
        let pool_ptr = self.solver_contacts.manifold_pool.as_mut_ptr();
        let pool_count = self.solver_contacts.manifold_pool.len();
        // SAFETY: as in `warm_start_contacts`; the two arrays are disjoint and stable for the phase.
        unsafe {
            let states = mutable_buffer_view(states_ptr, states_count);
            let pool = mutable_buffer_view(pool_ptr, pool_count);
            self.push_contacts_body(range, list, context, states, pool);
        }
    }

    pub(crate) fn push_contacts_body(
        &self,
        range: Range<usize>,
        list: *const ContactConstraint,
        context: &StepContext,
        states: &mut [BodyState],
        pool: &mut [ManifoldConstraint],
    ) {
        // SAFETY: as in `warm_start_contacts_body`.
        let states_base = states.as_mut_ptr();
        let inv_h = context.inv_h;
        let contact_speed = -self.contact_speed;
        let dynamic_bias_rate =
            context.contact_softness.mass_scale * context.contact_softness.bias_rate;
        let dynamic_mass_scale = context.contact_softness.mass_scale;
        let dynamic_impulse_scale = context.contact_softness.impulse_scale;
        let static_bias_rate =
            context.static_softness.mass_scale * context.static_softness.bias_rate;
        let static_mass_scale = context.static_softness.mass_scale;
        let static_impulse_scale = context.static_softness.impulse_scale;

        for constraint_index in range {
            // SAFETY: see `warm_start_contacts_body`.
            let c = unsafe { *list.add(constraint_index) };

            let mut b_a = World::state_at(states_base, c.index_a);
            let mut b_b = World::state_at(states_base, c.index_b);

            let is_static = c.is_static;
            let bias_rate = if is_static { static_bias_rate } else { dynamic_bias_rate };
            let mass_scale = if is_static { static_mass_scale } else { dynamic_mass_scale };
            let impulse_scale = if is_static {
                static_impulse_scale
            } else {
                dynamic_impulse_scale
            };

            let dp = b_b.delta_position - b_a.delta_position;
            let inv_mass = c.inv_mass_a + c.inv_mass_b;

            for manifold_offset in 0..c.manifold_count as usize {
                let manifold_index = c.manifold_start as usize + manifold_offset;
                let cm = &mut pool[manifold_index];
                let normal = cm.normal;
                let normal_separation = dot(normal, dp);
                let normal_a = inv_rotate_vector_wide(b_a.delta_rotation, normal);
                let normal_b = inv_rotate_vector_wide(b_b.delta_rotation, normal);
                let mut linear_normal_velocity =
                    dot(normal, b_b.linear_velocity - b_a.linear_velocity);

                let mut manifold_impulse: f32 = 0.0;

                for point_index in 0..cm.point_count as usize {
                    let mut cp = cm.points[point_index];

                    let s = (normal_separation + (dot(normal_b, cp.r_b) - dot(normal_a, cp.r_a)))
                        + cp.base_separation;
                    let separated = s > 0.0;

                    // Speculative bias - positive
                    let spec_bias = s * inv_h;

                    // Overlap bias - negative
                    let overlap_bias = max_float(bias_rate * s, contact_speed);
                    let velocity_bias = if separated { spec_bias } else { overlap_bias };

                    let point_mass_scale = if separated { 1.0 } else { mass_scale };
                    let point_impulse_scale = if separated { 0.0 } else { impulse_scale };

                    let u_a = cross(cp.r_a, normal);
                    let u_b = cross(cp.r_b, normal);
                    let g_a = c.inv_ia.mul_mv(u_a);
                    let g_b = c.inv_ib.mul_mv(u_b);

                    let v_n = linear_normal_velocity
                        + (dot(u_b, b_b.angular_velocity) - dot(u_a, b_a.angular_velocity));

                    let neg_impulse = cp.normal_mass * (point_mass_scale * v_n + velocity_bias)
                        + point_impulse_scale * cp.normal_impulse;
                    let impulse = max_float(-neg_impulse, -cp.normal_impulse);
                    cp.normal_impulse = cp.normal_impulse + impulse;

                    cm.points[point_index] = cp;

                    linear_normal_velocity += inv_mass * impulse;
                    manifold_impulse += impulse;
                    b_a.angular_velocity = b_a.angular_velocity - impulse * g_a;
                    b_b.angular_velocity = b_b.angular_velocity + impulse * g_b;
                }

                let p = manifold_impulse * normal;
                b_a.linear_velocity = b_a.linear_velocity - c.inv_mass_a * p;
                b_b.linear_velocity = b_b.linear_velocity + c.inv_mass_b * p;
            }

            World::set_state_at(states_base, c.index_a, b_a);
            World::set_state_at(states_base, c.index_b, b_b);
        }
    }

    // MARK: Solve (relax)

    pub(crate) fn solve_contacts(&mut self, color_index: i32, context: &StepContext, mesh: bool) {
        if color_index == OVERFLOW_INDEX {
            if mesh {
                self.solve_overflow_contacts(context);
            }
            return;
        }
        if self.uses_wide_contacts(color_index, mesh) {
            self.solve_contacts_wide(color_index, context, mesh);
            return;
        }
        let range = self.solver_contacts.range(color_index, mesh);
        if range.is_empty() {
            return;
        }
        let list = self.solver_contacts.constraints[color_index as usize].as_ptr();
        let awake = SetType::AwakeSet.raw_value() as usize;
        let states_ptr = self.solver_sets[awake].body_states.as_mut_ptr();
        let states_count = self.solver_sets[awake].body_states.len();
        let pool_ptr = self.solver_contacts.manifold_pool.as_mut_ptr();
        let pool_count = self.solver_contacts.manifold_pool.len();
        // SAFETY: as in `warm_start_contacts`; the two arrays are disjoint and stable for the phase.
        unsafe {
            let states = mutable_buffer_view(states_ptr, states_count);
            let pool = mutable_buffer_view(pool_ptr, pool_count);
            self.solve_contacts_body(range, list, context, states, pool);
        }
    }

    pub(crate) fn solve_contacts_body(
        &self,
        range: Range<usize>,
        list: *const ContactConstraint,
        context: &StepContext,
        states: &mut [BodyState],
        pool: &mut [ManifoldConstraint],
    ) {
        // SAFETY: as in `warm_start_contacts_body`.
        let states_base = states.as_mut_ptr();
        let inv_h = context.inv_h;
        let epsilon = f32::EPSILON;

        for constraint_index in range {
            // SAFETY: see `warm_start_contacts_body`.
            let c = unsafe { *list.add(constraint_index) };

            let mut b_a = World::state_at(states_base, c.index_a);
            let mut b_b = World::state_at(states_base, c.index_b);

            let any_rolling = c.rolling_resistance > 0.0;
            let dp = b_b.delta_position - b_a.delta_position;
            let inv_mass = c.inv_mass_a + c.inv_mass_b;

            for manifold_offset in 0..c.manifold_count as usize {
                let manifold_index = c.manifold_start as usize + manifold_offset;
                let cm = &mut pool[manifold_index];
                let normal = cm.normal;
                let normal_separation = dot(normal, dp);
                let normal_a = inv_rotate_vector_wide(b_a.delta_rotation, normal);
                let normal_b = inv_rotate_vector_wide(b_b.delta_rotation, normal);

                let dv = b_b.linear_velocity - b_a.linear_velocity;
                let mut linear_normal_velocity = dot(normal, dv);
                let linear_tangent_velocity1 = dot(cm.tangent1, dv) - cm.tangent_velocity1;
                let linear_tangent_velocity2 = dot(cm.tangent2, dv) - cm.tangent_velocity2;
                let mut manifold_impulse: f32 = 0.0;
                let mut total_normal_impulse: f32 = 0.0;
                let mut total_twist_limit: f32 = 0.0;

                // Normal constraints
                for point_index in 0..cm.point_count as usize {
                    let mut cp = cm.points[point_index];

                    let s = (normal_separation + (dot(normal_b, cp.r_b) - dot(normal_a, cp.r_a)))
                        + cp.base_separation;

                    // Speculative bias, positive if separated and zero if overlapped
                    let velocity_bias = max_float(0.0, s * inv_h);

                    let u_a = cross(cp.r_a, normal);
                    let u_b = cross(cp.r_b, normal);
                    let g_a = c.inv_ia.mul_mv(u_a);
                    let g_b = c.inv_ib.mul_mv(u_b);

                    let v_n = linear_normal_velocity
                        + (dot(u_b, b_b.angular_velocity) - dot(u_a, b_a.angular_velocity));

                    let neg_impulse = cp.normal_mass * (v_n + velocity_bias);
                    let impulse = max_float(-neg_impulse, -cp.normal_impulse);
                    let new_impulse = cp.normal_impulse + impulse;
                    cp.normal_impulse = new_impulse;
                    cp.total_normal_impulse = cp.total_normal_impulse + new_impulse;
                    total_normal_impulse += new_impulse;
                    total_twist_limit += cp.lever_arm * new_impulse;

                    linear_normal_velocity += inv_mass * impulse;
                    manifold_impulse += impulse;
                    b_a.angular_velocity = b_a.angular_velocity - impulse * g_a;
                    b_b.angular_velocity = b_b.angular_velocity + impulse * g_b;

                    cm.points[point_index] = cp;
                }

                // Rolling resistance. After normal constraints because it uses the normal impulse.
                if any_rolling {
                    let mut delta_impulse =
                        c.rolling_mass.mul_mv(b_a.angular_velocity - b_b.angular_velocity);
                    let old_impulse = cm.rolling_impulse;
                    cm.rolling_impulse = old_impulse + delta_impulse;

                    let max_impulse = c.rolling_resistance * total_normal_impulse;
                    let length_squared = dot(cm.rolling_impulse, cm.rolling_impulse);

                    let mask = length_squared > epsilon + max_impulse * max_impulse;

                    // No approximate rsqrt here to maintain cross-platform determinism.
                    let normalize = max_impulse / (length_squared.sqrt() + epsilon);
                    let scale = if mask { normalize } else { 1.0 };

                    let mut scaled_impulse = scale * cm.rolling_impulse;
                    if c.rolling_resistance == 0.0 {
                        scaled_impulse = Vec3::new(0.0, 0.0, 0.0);
                    }
                    cm.rolling_impulse = scaled_impulse;

                    delta_impulse = cm.rolling_impulse - old_impulse;

                    b_a.angular_velocity = b_a.angular_velocity - c.inv_ia.mul_mv(delta_impulse);
                    b_b.angular_velocity = b_b.angular_velocity + c.inv_ib.mul_mv(delta_impulse);
                }

                // Central twist friction.
                {
                    let twist_speed = dot(normal, b_b.angular_velocity - b_a.angular_velocity);
                    let max_lambda = c.friction * total_twist_limit;
                    let old_impulse = cm.twist_impulse;
                    let mut impulse = -(cm.twist_mass * twist_speed);
                    impulse = max_float(
                        -max_lambda - old_impulse,
                        min_float(impulse, max_lambda - old_impulse),
                    );
                    cm.twist_impulse = old_impulse + impulse;

                    b_a.angular_velocity = b_a.angular_velocity - impulse * c.inv_ia.mul_mv(normal);
                    b_b.angular_velocity = b_b.angular_velocity + impulse * c.inv_ib.mul_mv(normal);
                }

                // Central friction.
                {
                    let tangent1 = cm.tangent1;
                    let tangent2 = cm.tangent2;

                    let r_a = cm.center_a;
                    let r_b = cm.center_b;

                    let angular_relative_velocity =
                        cross(b_b.angular_velocity, r_b) - cross(b_a.angular_velocity, r_a);

                    let v_t = Vec2::new(
                        linear_tangent_velocity1 + dot(angular_relative_velocity, tangent1),
                        linear_tangent_velocity2 + dot(angular_relative_velocity, tangent2),
                    );

                    let mut delta_impulse = mul_mv2(cm.tangent_mass, v_t);
                    delta_impulse = Vec2::new(-delta_impulse.x, -delta_impulse.y);
                    let mut new_impulse = Vec2::new(
                        cm.friction_impulse.x + delta_impulse.x,
                        cm.friction_impulse.y + delta_impulse.y,
                    );

                    let max_impulse = c.friction * total_normal_impulse;
                    let length_squared =
                        new_impulse.x * new_impulse.x + new_impulse.y * new_impulse.y;
                    let mask = length_squared > max_impulse * max_impulse;
                    let normalize = max_impulse / (length_squared.sqrt() + epsilon);
                    let scale = if mask { normalize } else { 1.0 };
                    new_impulse = Vec2::new(scale * new_impulse.x, scale * new_impulse.y);

                    delta_impulse = Vec2::new(
                        new_impulse.x - cm.friction_impulse.x,
                        new_impulse.y - cm.friction_impulse.y,
                    );

                    cm.friction_impulse = new_impulse;

                    let mut p = delta_impulse.x * tangent1;
                    p = p + delta_impulse.y * tangent2;
                    b_a.angular_velocity = b_a.angular_velocity - c.inv_ia.mul_mv(cross(r_a, p));
                    b_b.angular_velocity = b_b.angular_velocity + c.inv_ib.mul_mv(cross(r_b, p));

                    p = p + manifold_impulse * normal;
                    b_a.linear_velocity = b_a.linear_velocity - c.inv_mass_a * p;
                    b_b.linear_velocity = b_b.linear_velocity + c.inv_mass_b * p;
                }
            }

            World::set_state_at(states_base, c.index_a, b_a);
            World::set_state_at(states_base, c.index_b, b_b);
        }
    }

    // MARK: Restitution

    pub(crate) fn apply_restitution(
        &mut self,
        color_index: i32,
        context: &StepContext,
        mesh: bool,
    ) {
        if color_index == OVERFLOW_INDEX {
            if mesh {
                self.apply_overflow_restitution(context);
            }
            return;
        }
        if self.uses_wide_contacts(color_index, mesh) {
            self.apply_restitution_wide(color_index, context, mesh);
            return;
        }
        let range = self.solver_contacts.range(color_index, mesh);
        if range.is_empty() {
            return;
        }
        let list = self.solver_contacts.constraints[color_index as usize].as_ptr();
        let awake = SetType::AwakeSet.raw_value() as usize;
        let states_ptr = self.solver_sets[awake].body_states.as_mut_ptr();
        let states_count = self.solver_sets[awake].body_states.len();
        let pool_ptr = self.solver_contacts.manifold_pool.as_mut_ptr();
        let pool_count = self.solver_contacts.manifold_pool.len();
        // SAFETY: as in `warm_start_contacts`; the two arrays are disjoint and stable for the phase.
        unsafe {
            let states = mutable_buffer_view(states_ptr, states_count);
            let pool = mutable_buffer_view(pool_ptr, pool_count);
            self.apply_restitution_body(range, list, context, states, pool);
        }
    }

    pub(crate) fn apply_restitution_body(
        &self,
        range: Range<usize>,
        list: *const ContactConstraint,
        context: &StepContext,
        states: &mut [BodyState],
        pool: &mut [ManifoldConstraint],
    ) {
        // SAFETY: as in `warm_start_contacts_body`.
        let states_base = states.as_mut_ptr();
        let inv_h = context.inv_h;
        let neg_restitution_threshold = -self.restitution_threshold;
        let propagate = self.enable_restitution_propagation;

        for constraint_index in range {
            // SAFETY: see `warm_start_contacts_body`.
            let c = unsafe { *list.add(constraint_index) };

            if !propagate && c.restitution == 0.0 {
                continue;
            }

            let mut b_a = World::state_at(states_base, c.index_a);
            let mut b_b = World::state_at(states_base, c.index_b);

            let restitution_mask = c.restitution > 0.0;
            let dp = b_b.delta_position - b_a.delta_position;

            for manifold_offset in 0..c.manifold_count as usize {
                let manifold_index = c.manifold_start as usize + manifold_offset;
                let cm = &mut pool[manifold_index];
                let normal = cm.normal;
                let normal_separation = dot(normal, dp);
                let normal_a = inv_rotate_vector_wide(b_a.delta_rotation, normal);
                let normal_b = inv_rotate_vector_wide(b_b.delta_rotation, normal);

                for point_index in 0..cm.point_count as usize {
                    let mut cp = cm.points[point_index];

                    let s = (normal_separation + (dot(normal_b, cp.r_b) - dot(normal_a, cp.r_a)))
                        + cp.base_separation;

                    let normal_mass = if propagate {
                        cp.normal_mass
                    } else if restitution_mask {
                        cp.normal_mass
                    } else {
                        0.0
                    };

                    let compression_impulse = cp.total_normal_impulse - cp.restitution_impulse;
                    let armed = restitution_mask
                        && cp.relative_velocity < neg_restitution_threshold
                        && compression_impulse > 0.0;

                    let spec_bias = max_float(0.0, s * inv_h);
                    let velocity_bias =
                        if armed { c.restitution * cp.relative_velocity } else { spec_bias };

                    let vr_a = b_a.linear_velocity + cross(b_a.angular_velocity, cp.r_a);
                    let vr_b = b_b.linear_velocity + cross(b_b.angular_velocity, cp.r_b);
                    let v_n = dot(vr_b - vr_a, normal);

                    let neg_impulse = normal_mass * (v_n + velocity_bias);

                    let new_impulse = max_float(cp.normal_impulse - neg_impulse, 0.0);
                    let mut impulse = new_impulse - cp.normal_impulse;

                    let approach_impulse =
                        min_float(max_float(-normal_mass * v_n, 0.0), max_float(impulse, 0.0));
                    let allowance = c.restitution * (compression_impulse + approach_impulse)
                        - cp.restitution_impulse;
                    let max_impulse = approach_impulse + max_float(allowance, 0.0);
                    impulse = if armed { min_float(impulse, max_impulse) } else { impulse };

                    cp.normal_impulse = cp.normal_impulse + impulse;
                    cp.restitution_impulse = cp.restitution_impulse + (impulse - approach_impulse);
                    cp.total_normal_impulse = cp.total_normal_impulse + impulse;

                    let p = impulse * normal;
                    b_a.angular_velocity =
                        b_a.angular_velocity - c.inv_ia.mul_mv(cross(cp.r_a, p));
                    b_a.linear_velocity = b_a.linear_velocity - c.inv_mass_a * p;
                    b_b.angular_velocity =
                        b_b.angular_velocity + c.inv_ib.mul_mv(cross(cp.r_b, p));
                    b_b.linear_velocity = b_b.linear_velocity + c.inv_mass_b * p;

                    cm.points[point_index] = cp;
                }
            }

            World::set_state_at(states_base, c.index_a, b_a);
            World::set_state_at(states_base, c.index_b, b_b);
        }
    }

    // MARK: Store

    pub(crate) fn store_impulses(&mut self, color_index: i32, mesh: bool) {
        if color_index == OVERFLOW_INDEX {
            if mesh {
                self.store_overflow_impulses();
            }
            return;
        }
        if self.uses_wide_contacts(color_index, mesh) {
            self.store_impulses_wide(color_index, mesh);
            return;
        }
        let list = self.solver_contacts.constraints[color_index as usize].as_ptr();
        let range = self.solver_contacts.range(color_index, mesh);
        if range.is_empty() {
            return;
        }
        let pool = self.solver_contacts.manifold_pool.as_ptr();
        let store = self.contacts.as_mut_ptr();
        // SAFETY: the manifold pool is only read here and the contact store is written only at this
        // constraint's own contact id, mirroring Swift's nested read-only pool / mutable store buffer
        // pointers. Neither array is resized during the store phase.
        self.store_impulses_body(range, list, pool, store);
    }

    /// Persist the impulses of scalar constraints into their contacts.
    pub(crate) fn store_impulses_body(
        &self,
        range: Range<usize>,
        list: *const ContactConstraint,
        pool: *const ManifoldConstraint,
        store: *mut Contact,
    ) {
        for constraint_index in range {
            // SAFETY: see `warm_start_contacts_body`.
            let c = unsafe { *list.add(constraint_index) };
            let id = c.contact_id as usize;
            // SAFETY: `store` addresses the world's contact array and `contact_id` indexes it; the
            // pool manifolds this contact owns live in the payload slab for the whole step.
            let contact = unsafe { &*store.add(id) };
            if contact.manifold_count <= 0 {
                continue;
            }
            let hit_events = (contact.flags & ContactFlags::SIM_ENABLE_HIT_EVENT.0) != 0;
            let manifolds = contact.manifolds();
            manifolds.with_unsafe_mutable_buffer_pointer(|manifolds| {
                for offset in 0..c.manifold_count as usize {
                    // SAFETY: `pool` plus this constraint's manifold run addresses initialized
                    // constraints owned by the constraint, which preparation wrote for exactly
                    // `manifold_count` manifolds.
                    let cm = unsafe { &*pool.add(c.manifold_start as usize + offset) };
                    let out = &mut manifolds[offset];
                    let count = min_int(out.point_count, cm.point_count) as usize;
                    for pi in 0..count {
                        out.points[pi].normal_impulse = cm.points[pi].normal_impulse;
                        out.points[pi].total_normal_impulse = cm.points[pi].total_normal_impulse;
                        if hit_events {
                            out.points[pi].normal_velocity = cm.points[pi].relative_velocity;
                        }
                    }
                    out.twist_impulse = cm.twist_impulse;
                    out.friction_impulse = cm.friction_impulse.x * cm.tangent1
                        + cm.friction_impulse.y * cm.tangent2;
                    out.rolling_impulse = cm.rolling_impulse;
                }
            });
        }
    }

    /// Store impulses for the whole graph.
    pub(crate) fn store_all_impulses(&mut self) {
        self.store_wide_joint_impulses();
        for mesh in [false, true] {
            self.store_impulses(OVERFLOW_INDEX, mesh);
            for i in 0..OVERFLOW_INDEX as usize {
                self.store_impulses(i as i32, mesh);
            }
        }
    }

    /// Warm start every active constraint in solver order.
    /// Warm start the overflow constraints, then each color in order. Within a color the joints
    /// are solved before the contacts.
    pub(crate) fn warm_start_all(&mut self, context: &StepContext) {
        self.warm_start_joints(OVERFLOW_INDEX, context);
        self.warm_start_contacts(OVERFLOW_INDEX, false);
        self.warm_start_contacts(OVERFLOW_INDEX, true);
        for i in 0..OVERFLOW_INDEX as usize {
            self.warm_start_joints(i as i32, context);
            self.warm_start_contacts(i as i32, false);
            self.warm_start_contacts(i as i32, true);
        }
    }

    pub(crate) fn push_all(&mut self, context: &StepContext) {
        self.solve_joints(OVERFLOW_INDEX, context, true);
        self.push_contacts(OVERFLOW_INDEX, context, false);
        self.push_contacts(OVERFLOW_INDEX, context, true);
        for i in 0..OVERFLOW_INDEX as usize {
            self.solve_joints(i as i32, context, true);
            self.push_contacts(i as i32, context, false);
            self.push_contacts(i as i32, context, true);
        }
    }

    pub(crate) fn solve_all(&mut self, context: &StepContext) {
        self.solve_joints(OVERFLOW_INDEX, context, false);
        self.solve_contacts(OVERFLOW_INDEX, context, false);
        self.solve_contacts(OVERFLOW_INDEX, context, true);
        for i in 0..OVERFLOW_INDEX as usize {
            self.solve_joints(i as i32, context, false);
            self.solve_contacts(i as i32, context, false);
            self.solve_contacts(i as i32, context, true);
        }
    }

    pub(crate) fn apply_restitution_all(&mut self, context: &StepContext) {
        self.apply_restitution(OVERFLOW_INDEX, context, false);
        self.apply_restitution(OVERFLOW_INDEX, context, true);
        for i in 0..OVERFLOW_INDEX as usize {
            self.apply_restitution(i as i32, context, false);
            self.apply_restitution(i as i32, context, true);
        }
    }
}
