//! The overflow color runs the scalar contact solver in sequential order.
//!
//! Translated from `Sources/Natsel/Simulation/OverflowContactSolver.swift`.
//!
//! Overflow solving is deliberately sequential in Swift: overflow constraints touch any body,
//! so they stay on the scalar path and are never parallelised or reordered. That ordering is
//! preserved here verbatim — accumulation order, early-outs, and per-point sequence all follow
//! the Swift source.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::contact_solver`: `ConstraintPoint`, `ManifoldConstraint`, and the
//!   `impl World` buffer helpers `state_at(states: &[BodyState], index)` (and `set_state_at`);
//!   `with_solver_states(body: FnOnce(&mut [BodyState]) -> R)` lends the awake set's state array.
//!   `contact_solver.rs` already dispatches to the entry points defined in this file:
//!   `warm_start_overflow_contacts`, `push_overflow_contacts`, `solve_overflow_contacts`,
//!   `apply_overflow_restitution`, `store_overflow_impulses`, and `solver_prepare.rs` calls
//!   `prepare_overflow_contacts`.
//!
//! Swift's `withSolverStates` is invoked from the private `with_overflow_contacts` helper below
//! (identical body), keeping the phases' access to the awake state array, the constraint list,
//! and the manifold pool on three disjoint world fields exactly as Swift's nested pointer borrows
//! did.

use crate::base::constants as b3;
use crate::collision::internal_math::{dot2, invert2, mul_mv2, sub2, Matrix2};
use crate::math::functions::{
    add_mm, blend2, clamp_float, cross, distance, dot, invert_matrix, max_float, min_float,
    mul_add, mul_mv, mul_sub, perp, rotate_vector,
};
use crate::math::types::{Matrix3, Vec2, Vec3};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::contact_solver::{ConstraintPoint, ManifoldConstraint};
use crate::simulation::solver_integration::{Softness, StepContext};
use crate::simulation::world_runtime::{
    decode_awake_index, BodyFlags, BodyState, SetType, World,
};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct OverflowContactConstraint {
    pub contact_id: i32,
    pub index_a: i32,
    pub index_b: i32,
    pub inv_mass_a: f32,
    pub inv_mass_b: f32,
    pub inv_ia: Matrix3,
    pub inv_ib: Matrix3,
    pub rolling_mass: Matrix3,
    pub softness: Softness,
    pub friction: f32,
    pub restitution: f32,
    pub rolling_resistance: f32,
    /// Swift's `manifoldStart`/`manifoldCount` are `Int`; the pool indices they encode are
    /// created from `i32` contact manifold counts and stay far inside `i32` range.
    pub manifold_start: i32,
    pub manifold_count: i32,
}

impl OverflowContactConstraint {
    /// `init()` — `contactId`/`indexA`/`indexB` default to `B3.nullIndex`.
    #[inline]
    pub const fn new() -> Self {
        Self {
            contact_id: b3::NULL_INDEX,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            inv_mass_a: 0.0,
            inv_mass_b: 0.0,
            inv_ia: Matrix3::ZERO,
            inv_ib: Matrix3::ZERO,
            rolling_mass: Matrix3::ZERO,
            softness: Softness::new(),
            friction: 0.0,
            restitution: 0.0,
            rolling_resistance: 0.0,
            manifold_start: 0,
            manifold_count: 0,
        }
    }
}

impl Default for OverflowContactConstraint {
    #[inline(always)]
    fn default() -> Self {
        OverflowContactConstraint::new()
    }
}

impl World {
    pub(crate) fn prepare_overflow_contacts(
        &mut self,
        softness: Softness,
        static_softness: Softness,
    ) {
        self.overflow_contacts.clear();
        self.overflow_manifolds.clear();
        let specs = &self.constraint_graph.colors[OVERFLOW_INDEX as usize].contacts;
        if specs.is_empty() {
            return;
        }

        let warm_start_scale: f32 = if self.enable_warm_starting { 1.0 } else { 0.0 };
        let inv_tau = 1.0 / b3::speculative_distance();
        let static_flag = ContactFlags::STATIC_FLAG.0;
        let hit_flag = ContactFlags::SIM_ENABLE_HIT_EVENT.0;
        let awake = SetType::AwakeSet.raw_value() as usize;
        let mut any_restitution = false;

        // The three shared borrows (sims, states, contacts) and the two owned vectors the phase
        // appends to are five disjoint fields of the world; Swift reached them through nested
        // buffer pointers on the same struct without an exclusivity conflict. Reading and writing
        // them by direct field paths keeps that separation visible to the borrow checker.
        let sims = &self.solver_sets[awake].body_sims;
        let states = &self.solver_sets[awake].body_states;
        let contact_buffer = &self.contacts;
        let overflow_contacts = &mut self.overflow_contacts;
        let overflow_manifolds = &mut self.overflow_manifolds;

        for spec in specs {
            let contact = &contact_buffer[spec.contact_id as usize];
            let index_a = decode_awake_index(contact.encoded_body_sim_a);
            let index_b = decode_awake_index(contact.encoded_body_sim_b);

            let mut c = OverflowContactConstraint::new();
            c.contact_id = spec.contact_id;
            c.index_a = index_a;
            c.index_b = index_b;
            if index_a >= 0 {
                c.inv_mass_a = sims[index_a as usize].inv_mass;
                c.inv_ia = sims[index_a as usize].inv_inertia_world;
            }
            if index_b >= 0 {
                c.inv_mass_b = sims[index_b as usize].inv_mass;
                c.inv_ib = sims[index_b as usize].inv_inertia_world;
            }
            let mA = c.inv_mass_a;
            let mB = c.inv_mass_b;
            let i_a = c.inv_ia;
            let i_b = c.inv_ib;
            c.rolling_mass = invert_matrix(add_mm(i_a, i_b));
            c.softness = if (contact.flags & static_flag) != 0 { static_softness } else { softness };
            c.friction = contact.friction;
            c.restitution = contact.restitution;
            c.rolling_resistance = contact.rolling_resistance;

            // Only sample contact point normal velocity if needed.
            let have_restitution = contact.restitution > 0.0;
            let hit_events = (contact.flags & hit_flag) != 0;
            let sample_velocity = have_restitution || hit_events;
            any_restitution = any_restitution || have_restitution;

            let mut v_a = Vec3::default();
            let mut w_a = Vec3::default();
            let mut v_b = Vec3::default();
            let mut w_b = Vec3::default();
            if sample_velocity {
                if index_a >= 0 {
                    v_a = states[index_a as usize].linear_velocity;
                    w_a = states[index_a as usize].angular_velocity;
                }
                if index_b >= 0 {
                    v_b = states[index_b as usize].linear_velocity;
                    w_b = states[index_b as usize].angular_velocity;
                }
            }

            let manifold_count = contact.manifold_count as usize;
            c.manifold_start = overflow_manifolds.len() as i32;
            c.manifold_count = manifold_count as i32;
            if manifold_count > 0 {
                // SAFETY: the manifold view writes through the payload slab pointers, which are
                // disjoint from the contact array borrow above; Swift took
                // `UnsafeMutablePointer(mutating:)` of the same buffer to do exactly this.
                let mview = contact.manifolds();
                for manifold_index in 0..manifold_count {
                    let manifold = mview.get_mut(manifold_index);
                    let mut constraint = ManifoldConstraint::new();
                    let point_count = manifold.point_count as usize;
                    let normal = manifold.normal;
                    let tangent1 = perp(normal);
                    let tangent2 = cross(tangent1, normal);

                    constraint.point_count = point_count as i32;
                    constraint.normal = normal;
                    constraint.tangent1 = tangent1;
                    constraint.tangent2 = tangent2;
                    constraint.tangent_velocity1 = dot(contact.tangent_velocity, tangent1);
                    constraint.tangent_velocity2 = dot(contact.tangent_velocity, tangent2);

                    let mut center_a = Vec3::default();
                    let mut center_b = Vec3::default();
                    let mut total_friction_weight: f32 = 0.0;

                    for point_index in 0..point_count {
                        let mut cp = ConstraintPoint::new();
                        let mp = manifold.points[point_index];
                        cp.r_a = mp.anchor_a;
                        cp.r_b = mp.anchor_b;

                        let s = mp.separation;
                        cp.base_separation = s - dot(cp.r_b - cp.r_a, normal);
                        cp.normal_impulse = warm_start_scale * mp.normal_impulse;
                        cp.total_normal_impulse = 0.0;
                        cp.restitution_impulse = 0.0;

                        let r_a = cp.r_a;
                        let r_b = cp.r_b;
                        let rn_a = cross(r_a, normal);
                        let rn_b = cross(r_b, normal);
                        let k_normal = mA + mB + dot(rn_a, mul_mv(i_a, rn_a))
                            + dot(rn_b, mul_mv(i_b, rn_b));
                        cp.normal_mass = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

                        if sample_velocity {
                            let vr_a = v_a + cross(w_a, r_a);
                            let vr_b = v_b + cross(w_b, r_b);
                            let vn = dot(normal, vr_b - vr_a);
                            cp.relative_velocity = vn;
                            manifold.points[point_index].normal_velocity =
                                if hit_events { vn } else { 0.0 };
                        } else {
                            cp.relative_velocity = 0.0;
                            manifold.points[point_index].normal_velocity = 0.0;
                        }

                        let weight =
                            clamp_float(2.0 - s * inv_tau, b3::MIN_FRICTION_WEIGHT, 1.0);
                        center_a = mul_add(center_a, weight, r_a);
                        center_b = mul_add(center_b, weight, r_b);
                        total_friction_weight += weight;
                        constraint.points[point_index] = cp;
                    }

                    let inv_weight = 1.0 / total_friction_weight;
                    center_a = inv_weight * center_a;
                    center_b = inv_weight * center_b;
                    constraint.center_a = center_a;
                    constraint.center_b = center_b;

                    for point_index in 0..point_count {
                        constraint.points[point_index].lever_arm =
                            distance(constraint.points[point_index].r_a, center_a);
                    }

                    let rt_a1 = cross(center_a, tangent1);
                    let rt_a2 = cross(center_a, tangent2);
                    let rt_b1 = cross(center_b, tangent1);
                    let rt_b2 = cross(center_b, tangent2);

                    let mut k = Matrix2::default();
                    k.cx.x = mA + mB + dot(rt_a1, mul_mv(i_a, rt_a1))
                        + dot(rt_b1, mul_mv(i_b, rt_b1));
                    k.cy.y = mA + mB + dot(rt_a2, mul_mv(i_a, rt_a2))
                        + dot(rt_b2, mul_mv(i_b, rt_b2));
                    k.cx.y = dot(rt_a1, mul_mv(i_a, rt_a2)) + dot(rt_b1, mul_mv(i_b, rt_b2));
                    k.cy.x = k.cx.y;
                    constraint.tangent_mass = invert2(k);
                    constraint.friction_impulse = Vec2::new(
                        warm_start_scale * dot(manifold.friction_impulse, tangent1),
                        warm_start_scale * dot(manifold.friction_impulse, tangent2),
                    );

                    let k_twist = dot(normal, mul_mv(add_mm(i_a, i_b), normal));
                    constraint.twist_mass = if k_twist > 0.0 { 1.0 / k_twist } else { 0.0 };
                    constraint.twist_impulse = warm_start_scale * manifold.twist_impulse;
                    constraint.rolling_impulse = warm_start_scale * manifold.rolling_impulse;

                    overflow_manifolds.push(constraint);
                }
            }
            overflow_contacts.push(c);
        }

        if any_restitution {
            self.solve_any_restitution = true;
        }
    }

    #[inline(always)]
    fn with_overflow_contacts<F>(&mut self, body: F)
    where
        F: FnOnce(
            *mut OverflowContactConstraint,
            usize,
            *mut ManifoldConstraint,
            &mut [BodyState],
        ),
    {
        if self.overflow_contacts.is_empty() {
            return;
        }
        let count = self.overflow_contacts.len();
        let list = self.overflow_contacts.as_mut_ptr();
        // Swift substituted the sentinel pointer `bitPattern: 16` for a null pool base. An empty
        // `Vec::as_mut_ptr` is a non-null, correctly aligned dangling pointer with the same
        // property: it is never dereferenced because every phase only walks
        // `0..manifold_count` per constraint, which is zero when the pool is empty.
        let pool = self.overflow_manifolds.as_mut_ptr();
        // SAFETY: `with_solver_states` (contact_solver) lends the awake set's body-state array —
        // a field disjoint from the two vectors the raw pointers above address, exactly the three
        // independent `withUnsafeMutableBufferPointer` nests Swift relied on without an
        // exclusivity conflict. None of the arrays is resized inside a solver phase.
        self.with_solver_states(move |states| body(list, count, pool, states));
    }

    pub(crate) fn warm_start_overflow_contacts(&mut self) {
        self.with_overflow_contacts(|list, count, pool, states| {
            let dynamic_flag = BodyFlags::DYNAMIC_FLAG.0;
            for i in 0..count {
                // SAFETY: `list` is the base of `overflow_contacts` with `count` initialized
                // elements; `i < count`, mirroring Swift's `list + i` subscript. The constraint is
                // read-only in this phase and never written while `c` is live.
                let c = unsafe { *list.add(i) };
                let mut linear_impulse = Vec3::default();
                let mut angular_impulse_a = Vec3::default();
                let mut angular_impulse_b = Vec3::default();

                for manifold_index in 0..c.manifold_count as usize {
                    // SAFETY: the manifold run starts at `c.manifold_start` in `pool` with
                    // `c.manifold_count` initialized entries, installed by
                    // `prepare_overflow_contacts` in append order; the read-only borrow does not
                    // alias `list` (a different array).
                    let constraint =
                        unsafe { &*pool.add(c.manifold_start as usize + manifold_index) };
                    let normal = constraint.normal;
                    let mut total_normal_impulse: f32 = 0.0;
                    let mut moment_a = Vec3::default();
                    let mut moment_b = Vec3::default();
                    for j in 0..constraint.point_count as usize {
                        let cp = constraint.points[j];
                        total_normal_impulse += cp.normal_impulse;
                        moment_a = mul_add(moment_a, cp.normal_impulse, cp.r_a);
                        moment_b = mul_add(moment_b, cp.normal_impulse, cp.r_b);
                    }

                    let mut friction_impulse = constraint.friction_impulse.x * constraint.tangent1;
                    friction_impulse = mul_add(
                        friction_impulse,
                        constraint.friction_impulse.y,
                        constraint.tangent2,
                    );

                    linear_impulse = linear_impulse + mul_add(friction_impulse, total_normal_impulse, normal);

                    let twist_impulse = constraint.twist_impulse * normal;
                    let angular_a = (cross(moment_a, normal)
                        + cross(constraint.center_a, friction_impulse))
                        + twist_impulse;
                    let angular_b = (cross(moment_b, normal)
                        + cross(constraint.center_b, friction_impulse))
                        + twist_impulse;

                    angular_impulse_a = angular_impulse_a + (angular_a + constraint.rolling_impulse);
                    angular_impulse_b = angular_impulse_b + (angular_b + constraint.rolling_impulse);
                }

                let index_a = c.index_a;
                let index_b = c.index_b;
                if index_a >= 0 && (states[index_a as usize].flags & dynamic_flag) != 0 {
                    states[index_a as usize].linear_velocity =
                        mul_sub(states[index_a as usize].linear_velocity, c.inv_mass_a, linear_impulse);
                    states[index_a as usize].angular_velocity = states[index_a as usize].angular_velocity
                        - mul_mv(c.inv_ia, angular_impulse_a);
                }
                if index_b >= 0 && (states[index_b as usize].flags & dynamic_flag) != 0 {
                    states[index_b as usize].linear_velocity =
                        mul_add(states[index_b as usize].linear_velocity, c.inv_mass_b, linear_impulse);
                    states[index_b as usize].angular_velocity = states[index_b as usize].angular_velocity
                        + mul_mv(c.inv_ib, angular_impulse_b);
                }
            }
        });
    }

    pub(crate) fn push_overflow_contacts(&mut self, context: &StepContext) {
        let inv_h = context.inv_h;
        let contact_speed = self.contact_speed;
        self.with_overflow_contacts(|list, count, pool, states| {
            for i in 0..count {
                // SAFETY: see `warm_start_overflow_contacts`; same list/pool discipline.
                let c = unsafe { *list.add(i) };
                let mA = c.inv_mass_a;
                let i_a = c.inv_ia;
                let mB = c.inv_mass_b;
                let i_b = c.inv_ib;
                let state_a = World::state_at(states, c.index_a);
                let state_b = World::state_at(states, c.index_b);
                let mut v_a = state_a.linear_velocity;
                let mut w_a = state_a.angular_velocity;
                let dq_a = state_a.delta_rotation;
                let mut v_b = state_b.linear_velocity;
                let mut w_b = state_b.angular_velocity;
                let dq_b = state_b.delta_rotation;
                let dp = state_b.delta_position - state_a.delta_position;
                let softness = c.softness;

                for j in 0..c.manifold_count as usize {
                    // SAFETY: mutable borrow of this constraint's pool slot for the manifold loop;
                    // the slot is owned by exactly one overflow constraint (append-order run).
                    let constraint = unsafe { &mut *pool.add(c.manifold_start as usize + j) };
                    let normal = constraint.normal;
                    for point_index in 0..constraint.point_count as usize {
                        let r_a = constraint.points[point_index].r_a;
                        let r_b = constraint.points[point_index].r_b;

                        let ds = dp
                            + (rotate_vector(dq_b, r_b) - rotate_vector(dq_a, r_a));
                        let s = dot(ds, normal) + constraint.points[point_index].base_separation;

                        let velocity_bias: f32;
                        let mass_scale: f32;
                        let impulse_scale: f32;
                        if s > 0.0 {
                            velocity_bias = s * inv_h;
                            mass_scale = 1.0;
                            impulse_scale = 0.0;
                        } else {
                            velocity_bias = max_float(
                                softness.mass_scale * softness.bias_rate * s,
                                -contact_speed,
                            );
                            mass_scale = softness.mass_scale;
                            impulse_scale = softness.impulse_scale;
                        }

                        let vr_a = v_a + cross(w_a, r_a);
                        let vr_b = v_b + cross(w_b, r_b);
                        let vn = dot(vr_b - vr_a, normal);

                        let normal_impulse = constraint.points[point_index].normal_impulse;
                        let mut delta_impulse = -constraint.points[point_index].normal_mass
                            * (mass_scale * vn + velocity_bias)
                            - impulse_scale * normal_impulse;
                        let new_impulse = max_float(normal_impulse + delta_impulse, 0.0);
                        delta_impulse = new_impulse - normal_impulse;
                        constraint.points[point_index].normal_impulse = new_impulse;

                        let p = delta_impulse * normal;
                        v_a = mul_sub(v_a, mA, p);
                        w_a = w_a - mul_mv(i_a, cross(r_a, p));
                        v_b = mul_add(v_b, mB, p);
                        w_b = w_b + mul_mv(i_b, cross(r_b, p));
                    }
                }

                World::store_overflow_velocity(states, c.index_a, state_a.flags, v_a, w_a);
                World::store_overflow_velocity(states, c.index_b, state_b.flags, v_b, w_b);
            }
        });
    }

    pub(crate) fn solve_overflow_contacts(&mut self, context: &StepContext) {
        let inv_h = context.inv_h;
        self.with_overflow_contacts(|list, count, pool, states| {
            for i in 0..count {
                // SAFETY: see `warm_start_overflow_contacts`; same list/pool discipline.
                let c = unsafe { *list.add(i) };
                let mA = c.inv_mass_a;
                let i_a = c.inv_ia;
                let mB = c.inv_mass_b;
                let i_b = c.inv_ib;
                let state_a = World::state_at(states, c.index_a);
                let state_b = World::state_at(states, c.index_b);
                let mut v_a = state_a.linear_velocity;
                let mut w_a = state_a.angular_velocity;
                let dq_a = state_a.delta_rotation;
                let mut v_b = state_b.linear_velocity;
                let mut w_b = state_b.angular_velocity;
                let dq_b = state_b.delta_rotation;
                let dp = state_b.delta_position - state_a.delta_position;
                let friction = c.friction;
                let rolling_resistance = c.rolling_resistance;

                for j in 0..c.manifold_count as usize {
                    // SAFETY: see `push_overflow_contacts`.
                    let constraint = unsafe { &mut *pool.add(c.manifold_start as usize + j) };
                    let normal = constraint.normal;
                    let mut total_normal_impulse: f32 = 0.0;
                    let mut total_twist_limit: f32 = 0.0;

                    for point_index in 0..constraint.point_count as usize {
                        let r_a = constraint.points[point_index].r_a;
                        let r_b = constraint.points[point_index].r_b;

                        let ds = dp
                            + (rotate_vector(dq_b, r_b) - rotate_vector(dq_a, r_a));
                        let s = dot(ds, normal) + constraint.points[point_index].base_separation;

                        // speculative bias, zero when overlapped
                        let velocity_bias: f32 = if s > 0.0 { s * inv_h } else { 0.0 };

                        let vr_a = v_a + cross(w_a, r_a);
                        let vr_b = v_b + cross(w_b, r_b);
                        let vn = dot(vr_b - vr_a, normal);

                        let normal_impulse = constraint.points[point_index].normal_impulse;
                        let mut delta_impulse = -constraint.points[point_index].normal_mass
                            * (vn + velocity_bias);
                        let new_impulse = max_float(normal_impulse + delta_impulse, 0.0);
                        delta_impulse = new_impulse - normal_impulse;
                        constraint.points[point_index].normal_impulse = new_impulse;
                        constraint.points[point_index].total_normal_impulse += new_impulse;
                        total_normal_impulse += new_impulse;
                        total_twist_limit += constraint.points[point_index].lever_arm * new_impulse;

                        let p = delta_impulse * normal;
                        v_a = mul_sub(v_a, mA, p);
                        w_a = w_a - mul_mv(i_a, cross(r_a, p));
                        v_b = mul_add(v_b, mB, p);
                        w_b = w_b + mul_mv(i_b, cross(r_b, p));
                    }

                    // Central twist friction
                    {
                        let twist_speed = dot(normal, w_b - w_a);
                        let max_impulse = friction * total_twist_limit;
                        let mut delta_impulse = -constraint.twist_mass * twist_speed;
                        let old_impulse = constraint.twist_impulse;
                        constraint.twist_impulse =
                            clamp_float(old_impulse + delta_impulse, -max_impulse, max_impulse);
                        delta_impulse = constraint.twist_impulse - old_impulse;

                        w_a = w_a - mul_mv(i_a, delta_impulse * normal);
                        w_b = w_b + mul_mv(i_b, delta_impulse * normal);
                    }

                    // Rolling resistance
                    if rolling_resistance > 0.0 {
                        let mut delta_impulse = -mul_mv(c.rolling_mass, w_b - w_a);
                        let old_impulse = constraint.rolling_impulse;
                        constraint.rolling_impulse = old_impulse + delta_impulse;

                        let max_impulse = rolling_resistance * total_normal_impulse;
                        let mag_sqr = dot(constraint.rolling_impulse, constraint.rolling_impulse);
                        // Swift's `Float.ulpOfOne` equals `f32::EPSILON` (both are 2^-23).
                        if mag_sqr > max_impulse * max_impulse + f32::EPSILON {
                            constraint.rolling_impulse =
                                (max_impulse / mag_sqr.sqrt()) * constraint.rolling_impulse;
                        }

                        delta_impulse = constraint.rolling_impulse - old_impulse;
                        w_a = w_a - mul_mv(i_a, delta_impulse);
                        w_b = w_b + mul_mv(i_b, delta_impulse);
                    }

                    // Central friction
                    {
                        let tangent1 = constraint.tangent1;
                        let tangent2 = constraint.tangent2;
                        let r_a = constraint.center_a;
                        let r_b = constraint.center_b;

                        let vr_a = v_a + cross(w_a, r_a);
                        let vr_b = v_b + cross(w_b, r_b);
                        let vr = vr_b - vr_a;
                        let vt = Vec2::new(
                            dot(vr, tangent1) - constraint.tangent_velocity1,
                            dot(vr, tangent2) - constraint.tangent_velocity2,
                        );

                        let tm = mul_mv2(constraint.tangent_mass, vt);
                        let mut delta_impulse = Vec2::new(-tm.x, -tm.y);
                        let mut new_impulse = Vec2::new(
                            constraint.friction_impulse.x + delta_impulse.x,
                            constraint.friction_impulse.y + delta_impulse.y,
                        );

                        let max_impulse = friction * total_normal_impulse;
                        let length_squared = dot2(new_impulse, new_impulse);
                        if length_squared > max_impulse * max_impulse {
                            let scale = max_impulse / length_squared.sqrt();
                            new_impulse.x *= scale;
                            new_impulse.y *= scale;
                        }
                        delta_impulse = sub2(new_impulse, constraint.friction_impulse);
                        constraint.friction_impulse = new_impulse;

                        let p = blend2(delta_impulse.x, tangent1, delta_impulse.y, tangent2);
                        v_a = mul_sub(v_a, mA, p);
                        w_a = w_a - mul_mv(i_a, cross(r_a, p));
                        v_b = mul_add(v_b, mB, p);
                        w_b = w_b + mul_mv(i_b, cross(r_b, p));
                    }
                }

                World::store_overflow_velocity(states, c.index_a, state_a.flags, v_a, w_a);
                World::store_overflow_velocity(states, c.index_b, state_b.flags, v_b, w_b);
            }
        });
    }

    pub(crate) fn apply_overflow_restitution(&mut self, context: &StepContext) {
        let threshold = self.restitution_threshold;
        let inv_h = context.inv_h;
        let propagate = self.enable_restitution_propagation;
        self.with_overflow_contacts(|list, count, pool, states| {
            for i in 0..count {
                // SAFETY: see `warm_start_overflow_contacts`; same list/pool discipline.
                let c = unsafe { *list.add(i) };
                let restitution = c.restitution;
                if !propagate && restitution == 0.0 {
                    continue;
                }

                let mA = c.inv_mass_a;
                let i_a = c.inv_ia;
                let mB = c.inv_mass_b;
                let i_b = c.inv_ib;
                let state_a = World::state_at(states, c.index_a);
                let state_b = World::state_at(states, c.index_b);
                let mut v_a = state_a.linear_velocity;
                let mut w_a = state_a.angular_velocity;
                let dq_a = state_a.delta_rotation;
                let mut v_b = state_b.linear_velocity;
                let mut w_b = state_b.angular_velocity;
                let dq_b = state_b.delta_rotation;
                let dp = state_b.delta_position - state_a.delta_position;

                for j in 0..c.manifold_count as usize {
                    // SAFETY: see `push_overflow_contacts`.
                    let constraint = unsafe { &mut *pool.add(c.manifold_start as usize + j) };
                    let normal = constraint.normal;
                    for point_index in 0..constraint.point_count as usize {
                        let cp = constraint.points[point_index];
                        let r_a = cp.r_a;
                        let r_b = cp.r_b;

                        // The total normal impulse is 0 for speculative points.
                        let compression_impulse = cp.total_normal_impulse - cp.restitution_impulse;
                        let armed = restitution > 0.0
                            && cp.relative_velocity < -threshold
                            && compression_impulse > 0.0;

                        let velocity_bias: f32;
                        if armed {
                            velocity_bias = restitution * cp.relative_velocity;
                        } else {
                            let ds = dp
                                + (rotate_vector(dq_b, r_b) - rotate_vector(dq_a, r_a));
                            let s = dot(ds, normal) + cp.base_separation;
                            velocity_bias = if s > 0.0 { s * inv_h } else { 0.0 };
                        }

                        let vr_a = v_a + cross(w_a, r_a);
                        let vr_b = v_b + cross(w_b, r_b);
                        let vn = dot(vr_b - vr_a, normal);

                        let mut impulse = -cp.normal_mass * (vn + velocity_bias);
                        let new_impulse = max_float(cp.normal_impulse + impulse, 0.0);
                        impulse = new_impulse - cp.normal_impulse;

                        let approach_impulse = min_float(
                            max_float(-cp.normal_mass * vn, 0.0),
                            max_float(impulse, 0.0),
                        );

                        if armed {
                            // Poisson kinetic restitution guarantees no energy gain.
                            let allowance =
                                restitution * (compression_impulse + approach_impulse)
                                    - cp.restitution_impulse;
                            impulse = min_float(
                                impulse,
                                approach_impulse + max_float(allowance, 0.0),
                            );
                        }

                        constraint.points[point_index].normal_impulse += impulse;
                        constraint.points[point_index].restitution_impulse += impulse - approach_impulse;
                        constraint.points[point_index].total_normal_impulse += impulse;

                        let p = impulse * normal;
                        v_a = mul_sub(v_a, mA, p);
                        w_a = w_a - mul_mv(i_a, cross(r_a, p));
                        v_b = mul_add(v_b, mB, p);
                        w_b = w_b + mul_mv(i_b, cross(r_b, p));
                    }
                }

                World::store_overflow_velocity(states, c.index_a, state_a.flags, v_a, w_a);
                World::store_overflow_velocity(states, c.index_b, state_b.flags, v_b, w_b);
            }
        });
    }

    pub(crate) fn store_overflow_impulses(&mut self) {
        if self.overflow_contacts.is_empty() {
            return;
        }
        // The pool slice and the contact array are shared borrows; the manifold writes go through
        // the payload slab, disjoint from both, exactly like Swift's
        // `UnsafeMutablePointer(mutating: contact.manifolds.buffer.baseAddress!)`.
        let pool = &self.overflow_manifolds;
        let store = &self.contacts;
        for c in &self.overflow_contacts {
            if c.manifold_count == 0 {
                continue;
            }
            let contact = &store[c.contact_id as usize];
            let mview = contact.manifolds();
            for manifold_index in 0..c.manifold_count as usize {
                let manifold = mview.get_mut(manifold_index);
                let constraint = pool[c.manifold_start as usize + manifold_index];
                manifold.twist_impulse = constraint.twist_impulse;
                manifold.friction_impulse = blend2(
                    constraint.friction_impulse.x,
                    constraint.tangent1,
                    constraint.friction_impulse.y,
                    constraint.tangent2,
                );
                manifold.rolling_impulse = constraint.rolling_impulse;
                for point_index in 0..constraint.point_count as usize {
                    manifold.points[point_index].normal_impulse =
                        constraint.points[point_index].normal_impulse;
                    manifold.points[point_index].total_normal_impulse =
                        constraint.points[point_index].total_normal_impulse;
                }
            }
        }
    }

    #[inline(always)]
    pub(crate) fn store_overflow_velocity(
        states: &mut [BodyState],
        index: i32,
        flags: u32,
        v: Vec3,
        w: Vec3,
    ) {
        if index >= 0 && (flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            states[index as usize].linear_velocity = v;
            states[index as usize].angular_velocity = w;
        }
    }
}

