//! Persistent four-contact SoA batches. Only graph colors with disjoint dynamic bodies
//! are packed; the overflow color always uses the sequential solver.
//!
//! Translated from `Sources/Natsel/Simulation/WideContactSolver.swift`.
//!
//! Determinism: the kernels run without lane masks - preparation zeroes padded points and
//! absent lanes, so their updates are exact per-lane no-ops. Lane order, operand order,
//! association and signed-zero handling match the scalar solver exactly; every lane operation
//! goes through the `wide_*` helpers in `wide_types.rs`.

use std::ops::Range;

use crate::math::types::Vec3;
use crate::simulation::body_api::ContactFlags;
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::contact_solver::SymMatrix3;
use crate::simulation::solver_integration::{mutable_buffer_view, StepContext};
use crate::simulation::wide_types::{
    BodyStateW, FloatW, MaskW, QuatW, SymMatrix2W, SymMatrix3W, Vec2W, Vec3W, add_w, cross_w,
    dot_w, max_w, min_w, mul_add_w, mul_mv2_w, mul_mv_w, mul_w, neg_w, select_vec3_w, select_w,
    sub_w, transpose4, wide_gt, wide_lt,
};
use crate::simulation::world_runtime::{BodyFlags, BodyState, Contact, SetType, World};

/// One constraint point, four lanes. Mirrors the scalar `contact_solver::ConstraintPoint`
/// with every field widened.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct ConstraintPointW {
    pub r_a: Vec3W,
    pub r_b: Vec3W,
    pub base_separation: FloatW,
    pub relative_velocity: FloatW,
    pub normal_impulse: FloatW,
    pub total_normal_impulse: FloatW,
    pub restitution_impulse: FloatW,
    pub normal_mass: FloatW,
    pub lever_arm: FloatW,
}

impl ConstraintPointW {
    /// `init()`
    #[inline]
    pub const fn new() -> Self {
        Self {
            r_a: Vec3W::zero(),
            r_b: Vec3W::zero(),
            base_separation: FloatW::ZERO,
            relative_velocity: FloatW::ZERO,
            normal_impulse: FloatW::ZERO,
            total_normal_impulse: FloatW::ZERO,
            restitution_impulse: FloatW::ZERO,
            normal_mass: FloatW::ZERO,
            lever_arm: FloatW::ZERO,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct ManifoldConstraintW {
    pub points: [ConstraintPointW; 4],

    /// Distinguish an absent lane from a real manifold containing zero active points.
    pub exists: [i32; 4],
    pub point_counts: [i32; 4],
    pub point_count: usize,
    pub normal: Vec3W,
    pub tangent1: Vec3W,
    pub tangent2: Vec3W,
    pub center_a: Vec3W,
    pub center_b: Vec3W,
    pub twist_mass: FloatW,
    pub twist_impulse: FloatW,
    pub tangent_mass: SymMatrix2W,
    pub friction_impulse: Vec2W,
    pub rolling_impulse: Vec3W,
    pub tangent_velocity1: FloatW,
    pub tangent_velocity2: FloatW,
}

impl ManifoldConstraintW {
    /// `init()`
    #[inline]
    pub const fn new() -> Self {
        Self {
            points: [ConstraintPointW::new(); 4],
            exists: [0; 4],
            point_counts: [0; 4],
            point_count: 0,
            normal: Vec3W::zero(),
            tangent1: Vec3W::zero(),
            tangent2: Vec3W::zero(),
            center_a: Vec3W::zero(),
            center_b: Vec3W::zero(),
            twist_mass: FloatW::ZERO,
            twist_impulse: FloatW::ZERO,
            tangent_mass: SymMatrix2W {
                xx: FloatW::ZERO,
                yy: FloatW::ZERO,
                xy: FloatW::ZERO,
            },
            friction_impulse: Vec2W::zero(),
            rolling_impulse: Vec3W::zero(),
            tangent_velocity1: FloatW::ZERO,
            tangent_velocity2: FloatW::ZERO,
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct ContactConstraintW {
    pub index_a: [i32; 4],
    pub index_b: [i32; 4],
    pub contact_ids: [i32; 4],
    pub lane_manifold_counts: [i32; 4],
    pub inv_mass_a: FloatW,
    pub inv_mass_b: FloatW,
    pub inv_ia: SymMatrix3W,
    pub inv_ib: SymMatrix3W,
    pub rolling_mass: SymMatrix3W,
    pub bias_rate: FloatW,
    pub mass_scale: FloatW,
    pub impulse_scale: FloatW,
    pub friction: FloatW,
    pub restitution: FloatW,
    pub rolling_resistance: FloatW,
    pub any_rolling: bool,
    pub any_restitution: bool,
    pub manifold_start: usize,
    pub manifold_count: usize,
}

impl ContactConstraintW {
    /// `init()`
    #[inline]
    pub const fn new() -> Self {
        Self {
            index_a: [-1; 4],
            index_b: [-1; 4],
            contact_ids: [-1; 4],
            lane_manifold_counts: [0; 4],
            inv_mass_a: FloatW::ZERO,
            inv_mass_b: FloatW::ZERO,
            inv_ia: SymMatrix3W {
                xx: FloatW::ZERO,
                yy: FloatW::ZERO,
                zz: FloatW::ZERO,
                xy: FloatW::ZERO,
                xz: FloatW::ZERO,
                yz: FloatW::ZERO,
            },
            inv_ib: SymMatrix3W {
                xx: FloatW::ZERO,
                yy: FloatW::ZERO,
                zz: FloatW::ZERO,
                xy: FloatW::ZERO,
                xz: FloatW::ZERO,
                yz: FloatW::ZERO,
            },
            rolling_mass: SymMatrix3W {
                xx: FloatW::ZERO,
                yy: FloatW::ZERO,
                zz: FloatW::ZERO,
                xy: FloatW::ZERO,
                xz: FloatW::ZERO,
                yz: FloatW::ZERO,
            },
            bias_rate: FloatW::ZERO,
            mass_scale: FloatW::ZERO,
            impulse_scale: FloatW::ZERO,
            friction: FloatW::ZERO,
            restitution: FloatW::ZERO,
            rolling_resistance: FloatW::ZERO,
            any_rolling: false,
            any_restitution: false,
            manifold_start: 0,
            manifold_count: 0,
        }
    }
}

/// Swift derives the id lane defaults from `SIMD4<Int32>(repeating: -1)`, so `Default` cannot
/// be derived; it must reproduce `init()` exactly.
impl Default for ContactConstraintW {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl SymMatrix3W {
    /// Write one lane from a scalar symmetric matrix. Consumed by the wide joint solver.
    #[inline(always)]
    pub(crate) fn set_lane(&mut self, lane: usize, m: SymMatrix3) {
        self.xx[lane] = m.cxx;
        self.yy[lane] = m.cyy;
        self.zz[lane] = m.czz;
        self.xy[lane] = m.cxy;
        self.xz[lane] = m.cxz;
        self.yz[lane] = m.cyz;
    }
}

// MARK: - Integer lane masks

/// Swift's `SIMD4<Int32> .> SIMD4<Int32>` operator-syntax comparison, producing the same
/// all-bits-set / no-bits-set lane pattern that [`wide_gt`] produces for floats.
#[inline(always)]
pub(crate) fn wide_gt_i32(a: [i32; 4], b: [i32; 4]) -> MaskW {
    MaskW::from_bits([
        if a[0] > b[0] { u32::MAX } else { 0 },
        if a[1] > b[1] { u32::MAX } else { 0 },
        if a[2] > b[2] { u32::MAX } else { 0 },
        if a[3] > b[3] { u32::MAX } else { 0 },
    ])
}

/// Swift's `SIMD4<Int32> .> SIMD4<Int32>(repeating: value)`.
#[inline(always)]
pub(crate) fn wide_gt_i32_splat(a: [i32; 4], b: i32) -> MaskW {
    wide_gt_i32(a, [b; 4])
}

/// Swift's SIMDMask `.&`.
#[inline(always)]
pub(crate) fn mask_and(a: MaskW, b: MaskW) -> MaskW {
    MaskW::from_bits([
        a.bits[0] & b.bits[0],
        a.bits[1] & b.bits[1],
        a.bits[2] & b.bits[2],
        a.bits[3] & b.bits[3],
    ])
}

/// Swift's SIMDMask `.|`.
#[inline(always)]
pub(crate) fn mask_or(a: MaskW, b: MaskW) -> MaskW {
    MaskW::from_bits([
        a.bits[0] | b.bits[0],
        a.bits[1] | b.bits[1],
        a.bits[2] | b.bits[2],
        a.bits[3] | b.bits[3],
    ])
}

/// Swift's `any(mask)` - true when at least one lane is set.
#[inline(always)]
pub(crate) fn mask_any(a: MaskW) -> bool {
    a.bits[0] != 0 || a.bits[1] != 0 || a.bits[2] != 0 || a.bits[3] != 0
}

// MARK: - Lane storage helpers

/// The raw lane addressing below assumes the 64-byte solver state. The compile-time check
/// replaces nothing at runtime; it only pins the layout Swift relied on.
const _: () = assert!(core::mem::size_of::<BodyState>() == 64);

/// One 16-byte row of four `f32` lanes read from raw memory. `FloatW` is 16-byte aligned but
/// the solver addresses rows at arbitrary 4-byte offsets, hence the unaligned scalar loads.
#[inline(always)]
fn load_row(base: *const u8, offset: usize) -> FloatW {
    // SAFETY: callers pass a row offset that stays inside the 64-byte state object, and each
    // f32 is read unaligned from live, initialized memory.
    unsafe {
        let p = base.add(offset) as *const f32;
        FloatW::new(
            p.read_unaligned(),
            p.add(1).read_unaligned(),
            p.add(2).read_unaligned(),
            p.add(3).read_unaligned(),
        )
    }
}

/// One 16-byte row of four `f32` lanes written to raw memory. Writes the whole row including
/// the padding lane, exactly as Swift's `storeBytes(of: FloatW)` did.
#[inline(always)]
fn store_row(base: *mut u8, offset: usize, value: FloatW) {
    // SAFETY: as for `load_row`; the row lies inside a live `BodyState`.
    unsafe {
        let p = base.add(offset) as *mut f32;
        p.write_unaligned(value.storage[0]);
        p.add(1).write_unaligned(value.storage[1]);
        p.add(2).write_unaligned(value.storage[2]);
        p.add(3).write_unaligned(value.storage[3]);
    }
}

/// `UnsafeMutablePointer<ManifoldConstraintW>.point(_:)` - one constraint point in place.
/// Going through `points[i]` would copy or outline the access.
#[inline(always)]
fn point_w(m: *mut ManifoldConstraintW, index: usize) -> *mut ConstraintPointW {
    // SAFETY: `index` is below `point_count` at every call site and `points` is a fixed inline
    // array, so the offset lands inside the pointee for the whole loop.
    unsafe {
        (m as *mut u8)
            .add(core::mem::offset_of!(ManifoldConstraintW, points))
            .cast::<ConstraintPointW>()
            .add(index)
    }
}

/// The four pointers and the range one serial wide entry lends to its kernel, exactly the set
/// Swift's `withWideContacts` closure received.
struct WideContactPointers {
    list: *const ContactConstraintW,
    range: Range<usize>,
    pool: *mut ManifoldConstraintW,
    states: *mut BodyState,
    states_count: usize,
}

impl World {
    #[inline(always)]
    pub(crate) fn uses_wide_contacts(&self, color: i32, mesh: bool) -> bool {
        self.enable_wide_solver
            && color != OVERFLOW_INDEX
            && if mesh {
                self.solver_contacts.wide_mesh[color as usize]
            } else {
                self.solver_contacts.wide_convex[color as usize]
            }
    }

    /// Lend the wide constraints of one color and kind with fresh pointers (the serial path).
    ///
    /// Swift wrapped this in one closure-taking helper (`withWideContacts`); Rust's borrow rules
    /// forbid a `&mut self` method from calling a closure that re-borrows `self`, so each serial
    /// entry below takes the same pointers inline. The invariants are identical: the color's wide
    /// constraint array, the convex/mesh split, the wide manifold pool, and - via the
    /// `withSolverStates` equivalent - the awake set's body states, none of which are resized
    /// while a contact phase runs.
    #[inline(always)]
    fn wide_contacts_pointers(&mut self, color: i32, mesh: bool) -> Option<WideContactPointers> {
        let ci = color as usize;
        let list = &self.solver_contacts.wide_constraints[ci];
        let split = self.solver_contacts.wide_convex_counts[ci] as usize;
        let range = if mesh {
            split..list.len()
        } else {
            0..split
        };
        if range.is_empty() {
            return None;
        }
        let list_base = list.as_ptr();
        let pool_base = self.solver_contacts.wide_manifolds.as_mut_ptr();
        let awake = SetType::AwakeSet.raw_value() as usize;
        let states_base = self.solver_sets[awake].body_states.as_mut_ptr();
        let states_count = self.solver_sets[awake].body_states.len();
        Some(WideContactPointers {
            list: list_base,
            range,
            pool: pool_base,
            states: states_base,
            states_count,
        })
    }

    #[inline(always)]
    pub(crate) fn warm_start_contacts_wide(&mut self, color: i32, mesh: bool) {
        let Some(pointers) = self.wide_contacts_pointers(color, mesh) else {
            return;
        };
        // SAFETY: mirrors Swift's `withSolverStates` inside `withWideContacts`. While a contact
        // phase runs the awake set's state array is the only solver state storage and is not
        // resized; `mutable_buffer_view` gives the slice a lifetime independent of `self`, the
        // same pattern `integrate_velocities` uses.
        let states = unsafe { mutable_buffer_view(pointers.states, pointers.states_count) };
        self.warm_start_contacts_wide_range(pointers.list, pointers.range, states, pointers.pool);
    }

    // The kernels below run without lane masks: prepare zeroes padded
    // points and absent lanes, so their updates are exact no-ops.

    pub(crate) fn warm_start_contacts_wide_range(
        &self,
        list: *const ContactConstraintW,
        range: Range<usize>,
        states: &mut [BodyState],
        pool: *mut ManifoldConstraintW,
    ) {
        for i in range {
            // SAFETY: `i` is inside the color's wide constraint list for this stage.
            let c = unsafe { &*list.add(i) };
            let any_rolling = c.any_rolling;
            let mut linear = Vec3W::zero();
            let mut angular_a = Vec3W::zero();
            let mut angular_b = Vec3W::zero();
            for offset in 0..c.manifold_count {
                // SAFETY: `pool + manifold_start + offset` addresses a manifold slot that
                // `prepare_wide_batch` initialized for this constraint.
                let m = unsafe { pool.add(c.manifold_start + offset) };
                let mut total = FloatW::ZERO;
                let mut moment_a = Vec3W::zero();
                let mut moment_b = Vec3W::zero();
                for pi in 0..unsafe { (*m).point_count } {
                    let p = point_w(m, pi);
                    // SAFETY: `pi` is below the manifold's point count; `point_w` stays inside it.
                    unsafe {
                        total += (*p).normal_impulse;
                        moment_a = mul_add_w(moment_a, (*p).normal_impulse, (*p).r_a);
                        moment_b = mul_add_w(moment_b, (*p).normal_impulse, (*p).r_b);
                    }
                }
                // SAFETY: `m` addresses a live wide manifold.
                let normal = unsafe { (*m).normal };
                let mut friction = mul_w(unsafe { (*m).friction_impulse.x }, unsafe { (*m).tangent1 });
                friction = mul_add_w(
                    friction,
                    unsafe { (*m).friction_impulse.y },
                    unsafe { (*m).tangent2 },
                );
                let manifold_linear = mul_add_w(friction, total, normal);
                let twist = mul_w(unsafe { (*m).twist_impulse }, normal);
                let mut a = add_w(
                    add_w(
                        cross_w(moment_a, normal),
                        cross_w(unsafe { (*m).center_a }, friction),
                    ),
                    twist,
                );
                let mut b = add_w(
                    add_w(
                        cross_w(moment_b, normal),
                        cross_w(unsafe { (*m).center_b }, friction),
                    ),
                    twist,
                );
                if any_rolling {
                    a = add_w(a, unsafe { (*m).rolling_impulse });
                    b = add_w(b, unsafe { (*m).rolling_impulse });
                }
                linear = add_w(linear, manifold_linear);
                angular_a = add_w(angular_a, a);
                angular_b = add_w(angular_b, b);
            }
            let mut a = World::gather_states_w(states, c.index_a);
            let mut b = World::gather_states_w(states, c.index_b);
            a.w = sub_w(a.w, mul_mv_w(c.inv_ia, angular_a));
            a.v = sub_w(a.v, mul_w(c.inv_mass_a, linear));
            b.w = add_w(b.w, mul_mv_w(c.inv_ib, angular_b));
            b.v = add_w(b.v, mul_w(c.inv_mass_b, linear));
            World::scatter_states_w(states, c.index_a, a);
            World::scatter_states_w(states, c.index_b, b);
        }
    }

    pub(crate) fn push_contacts_wide(&mut self, color: i32, context: &StepContext, mesh: bool) {
        let Some(pointers) = self.wide_contacts_pointers(color, mesh) else {
            return;
        };
        // SAFETY: as for `warm_start_contacts_wide`.
        let states = unsafe { mutable_buffer_view(pointers.states, pointers.states_count) };
        self.push_contacts_wide_range(
            pointers.list,
            pointers.range,
            context,
            states,
            pointers.pool,
        );
    }

    pub(crate) fn push_contacts_wide_range(
        &self,
        list: *const ContactConstraintW,
        range: Range<usize>,
        context: &StepContext,
        states: &mut [BodyState],
        pool: *mut ManifoldConstraintW,
    ) {
        let inv_h = FloatW::splat(context.inv_h);
        let min_speed = FloatW::splat(-self.contact_speed);
        for i in range {
            // SAFETY: `i` is inside the color's wide constraint list for this stage.
            let c = unsafe { &*list.add(i) };
            let index_a = c.index_a;
            let index_b = c.index_b;
            let mut a = World::gather_states_w(states, index_a);
            let mut b = World::gather_states_w(states, index_b);
            let bias_rate = c.bias_rate;
            let mass_scale = c.mass_scale;
            let impulse_scale = c.impulse_scale;
            let dp = sub_w(b.dp, a.dp);
            let inv_mass = c.inv_mass_a + c.inv_mass_b;
            for offset in 0..c.manifold_count {
                // SAFETY: the manifold slot was initialized by `prepare_wide_batch`.
                let m = unsafe { pool.add(c.manifold_start + offset) };
                // SAFETY: `m` addresses a live wide manifold.
                let normal = unsafe { (*m).normal };
                let separation = dot_w(normal, dp);
                let normal_a = self.inv_rotate_vec_w(a.dq, normal);
                let normal_b = self.inv_rotate_vec_w(b.dq, normal);
                let mut vn_linear = dot_w(normal, sub_w(b.v, a.v));
                let mut total = FloatW::ZERO;
                for pi in 0..unsafe { (*m).point_count } {
                    let p = point_w(m, pi);
                    // SAFETY: `pi` is below the manifold's point count.
                    unsafe {
                        let r_a = (*p).r_a;
                        let r_b = (*p).r_b;
                        let s = (separation + (dot_w(normal_b, r_b) - dot_w(normal_a, r_a)))
                            + (*p).base_separation;
                        let separated = wide_gt(s, FloatW::ZERO);
                        let bias =
                            select_w(separated, s * inv_h, max_w(bias_rate * s, min_speed));
                        let point_mass_scale = select_w(separated, FloatW::splat(1.0), mass_scale);
                        let point_impulse_scale = select_w(separated, FloatW::ZERO, impulse_scale);
                        let u_a = cross_w(r_a, normal);
                        let u_b = cross_w(r_b, normal);
                        let g_a = mul_mv_w(c.inv_ia, u_a);
                        let g_b = mul_mv_w(c.inv_ib, u_b);
                        let vn = vn_linear + (dot_w(u_b, b.w) - dot_w(u_a, a.w));
                        let neg = (*p).normal_mass * (point_mass_scale * vn + bias)
                            + point_impulse_scale * (*p).normal_impulse;
                        let impulse = max_w(neg_w(neg), neg_w((*p).normal_impulse));
                        (*p).normal_impulse += impulse;
                        vn_linear += inv_mass * impulse;
                        total += impulse;
                        a.w = sub_w(a.w, mul_w(impulse, g_a));
                        b.w = add_w(b.w, mul_w(impulse, g_b));
                    }
                }
                let impulse = mul_w(total, normal);
                a.v = sub_w(a.v, mul_w(c.inv_mass_a, impulse));
                b.v = add_w(b.v, mul_w(c.inv_mass_b, impulse));
            }
            World::scatter_states_w(states, index_a, a);
            World::scatter_states_w(states, index_b, b);
        }
    }

    pub(crate) fn solve_contacts_wide(&mut self, color: i32, context: &StepContext, mesh: bool) {
        let Some(pointers) = self.wide_contacts_pointers(color, mesh) else {
            return;
        };
        // SAFETY: as for `warm_start_contacts_wide`.
        let states = unsafe { mutable_buffer_view(pointers.states, pointers.states_count) };
        self.solve_contacts_wide_range(
            pointers.list,
            pointers.range,
            context,
            states,
            pointers.pool,
        );
    }

    pub(crate) fn solve_contacts_wide_range(
        &self,
        list: *const ContactConstraintW,
        range: Range<usize>,
        context: &StepContext,
        states: &mut [BodyState],
        pool: *mut ManifoldConstraintW,
    ) {
        let inv_h = FloatW::splat(context.inv_h);
        let epsilon = FloatW::splat(f32::EPSILON);
        for i in range {
            // SAFETY: `i` is inside the color's wide constraint list for this stage.
            let c = unsafe { &*list.add(i) };
            let index_a = c.index_a;
            let index_b = c.index_b;
            let mut a = World::gather_states_w(states, index_a);
            let mut b = World::gather_states_w(states, index_b);
            let any_rolling = c.any_rolling;
            let dp = sub_w(b.dp, a.dp);
            let inv_mass = c.inv_mass_a + c.inv_mass_b;
            for offset in 0..c.manifold_count {
                // SAFETY: the manifold slot was initialized by `prepare_wide_batch`.
                let m = unsafe { pool.add(c.manifold_start + offset) };
                // SAFETY: `m` addresses a live wide manifold.
                let normal = unsafe { (*m).normal };
                let separation = dot_w(normal, dp);
                let normal_a = self.inv_rotate_vec_w(a.dq, normal);
                let normal_b = self.inv_rotate_vec_w(b.dq, normal);
                let dv = sub_w(b.v, a.v);
                let mut vn_linear = dot_w(normal, dv);
                let vt1 = dot_w(unsafe { (*m).tangent1 }, dv) - unsafe { (*m).tangent_velocity1 };
                let vt2 = dot_w(unsafe { (*m).tangent2 }, dv) - unsafe { (*m).tangent_velocity2 };
                let mut total = FloatW::ZERO;
                let mut total_normal = FloatW::ZERO;
                let mut twist_limit = FloatW::ZERO;
                for pi in 0..unsafe { (*m).point_count } {
                    let p = point_w(m, pi);
                    // SAFETY: `pi` is below the manifold's point count.
                    unsafe {
                        let r_a = (*p).r_a;
                        let r_b = (*p).r_b;
                        let s = (separation + (dot_w(normal_b, r_b) - dot_w(normal_a, r_a)))
                            + (*p).base_separation;
                        let bias = max_w(FloatW::ZERO, s * inv_h);
                        let u_a = cross_w(r_a, normal);
                        let u_b = cross_w(r_b, normal);
                        let g_a = mul_mv_w(c.inv_ia, u_a);
                        let g_b = mul_mv_w(c.inv_ib, u_b);
                        let vn = vn_linear + (dot_w(u_b, b.w) - dot_w(u_a, a.w));
                        let neg = (*p).normal_mass * (vn + bias);
                        let impulse = max_w(neg_w(neg), neg_w((*p).normal_impulse));
                        let new_impulse = (*p).normal_impulse + impulse;
                        (*p).normal_impulse = new_impulse;
                        (*p).total_normal_impulse += new_impulse;
                        total_normal += new_impulse;
                        twist_limit += (*p).lever_arm * new_impulse;
                        vn_linear += inv_mass * impulse;
                        total += impulse;
                        a.w = sub_w(a.w, mul_w(impulse, g_a));
                        b.w = add_w(b.w, mul_w(impulse, g_b));
                    }
                }
                if any_rolling {
                    // Flip A/B order to negate
                    let old = unsafe { (*m).rolling_impulse };
                    let new = add_w(old, mul_mv_w(c.rolling_mass, sub_w(a.w, b.w)));
                    let limit = c.rolling_resistance * total_normal;
                    let length_squared = dot_w(new, new);
                    let scale = select_w(
                        wide_gt(length_squared, epsilon + limit * limit),
                        limit / (length_squared.square_root() + epsilon),
                        FloatW::splat(1.0),
                    );
                    let rolling_mask = mask_and(
                        wide_gt(c.rolling_resistance, FloatW::ZERO),
                        wide_gt_i32_splat(unsafe { (*m).exists }, 0),
                    );
                    let clamped = select_vec3_w(rolling_mask, mul_w(scale, new), Vec3W::zero());
                    unsafe { (*m).rolling_impulse = clamped };
                    let delta = sub_w(clamped, old);
                    a.w = sub_w(a.w, mul_mv_w(c.inv_ia, delta));
                    b.w = add_w(b.w, mul_mv_w(c.inv_ib, delta));
                }
                // Central twist friction
                {
                    let normal = unsafe { (*m).normal };
                    let twist_speed = dot_w(normal, sub_w(b.w, a.w));
                    let max_lambda = c.friction * twist_limit;
                    let old_twist = unsafe { (*m).twist_impulse };
                    let mut twist = neg_w(unsafe { (*m).twist_mass } * twist_speed);
                    twist = max_w(
                        neg_w(max_lambda) - old_twist,
                        min_w(twist, max_lambda - old_twist),
                    );
                    unsafe { (*m).twist_impulse = old_twist + twist };
                    a.w = sub_w(a.w, mul_w(twist, mul_mv_w(c.inv_ia, normal)));
                    b.w = add_w(b.w, mul_w(twist, mul_mv_w(c.inv_ib, normal)));
                }
                // Central friction
                let t1 = unsafe { (*m).tangent1 };
                let t2 = unsafe { (*m).tangent2 };
                let r_a = unsafe { (*m).center_a };
                let r_b = unsafe { (*m).center_b };
                let angular = sub_w(cross_w(b.w, r_b), cross_w(a.w, r_a));
                let tangent_velocity =
                    Vec2W::new(vt1 + dot_w(angular, t1), vt2 + dot_w(angular, t2));
                let mut delta = mul_mv2_w(unsafe { (*m).tangent_mass }, tangent_velocity);
                delta = Vec2W::new(neg_w(delta.x), neg_w(delta.y));
                let old_friction = unsafe { (*m).friction_impulse };
                let mut new_friction =
                    Vec2W::new(old_friction.x + delta.x, old_friction.y + delta.y);
                let limit = c.friction * total_normal;
                let length_squared =
                    new_friction.x * new_friction.x + new_friction.y * new_friction.y;
                let scale = select_w(
                    wide_gt(length_squared, limit * limit),
                    limit / (length_squared.square_root() + epsilon),
                    FloatW::splat(1.0),
                );
                new_friction = Vec2W::new(scale * new_friction.x, scale * new_friction.y);
                delta = Vec2W::new(
                    new_friction.x - old_friction.x,
                    new_friction.y - old_friction.y,
                );
                unsafe { (*m).friction_impulse = new_friction };
                let mut impulse = add_w(mul_w(delta.x, t1), mul_w(delta.y, t2));
                a.w = sub_w(a.w, mul_mv_w(c.inv_ia, cross_w(r_a, impulse)));
                b.w = add_w(b.w, mul_mv_w(c.inv_ib, cross_w(r_b, impulse)));
                impulse = add_w(impulse, mul_w(total, unsafe { (*m).normal }));
                a.v = sub_w(a.v, mul_w(c.inv_mass_a, impulse));
                b.v = add_w(b.v, mul_w(c.inv_mass_b, impulse));
            }
            World::scatter_states_w(states, index_a, a);
            World::scatter_states_w(states, index_b, b);
        }
    }

    pub(crate) fn apply_restitution_wide(&mut self, color: i32, context: &StepContext, mesh: bool) {
        let Some(pointers) = self.wide_contacts_pointers(color, mesh) else {
            return;
        };
        // SAFETY: as for `warm_start_contacts_wide`.
        let states = unsafe { mutable_buffer_view(pointers.states, pointers.states_count) };
        self.apply_restitution_wide_range(
            pointers.list,
            pointers.range,
            context,
            states,
            pointers.pool,
        );
    }

    pub(crate) fn apply_restitution_wide_range(
        &self,
        list: *const ContactConstraintW,
        range: Range<usize>,
        context: &StepContext,
        states: &mut [BodyState],
        pool: *mut ManifoldConstraintW,
    ) {
        let inv_h = FloatW::splat(context.inv_h);
        let threshold = FloatW::splat(-self.restitution_threshold);
        let propagate = self.enable_restitution_propagation;
        for i in range {
            // SAFETY: `i` is inside the color's wide constraint list for this stage.
            let c = unsafe { &*list.add(i) };
            if !propagate && !c.any_restitution {
                continue;
            }
            let index_a = c.index_a;
            let index_b = c.index_b;
            let inv_ia = c.inv_ia;
            let inv_ib = c.inv_ib;
            let inv_mass_a = c.inv_mass_a;
            let inv_mass_b = c.inv_mass_b;
            let mut a = World::gather_states_w(states, index_a);
            let mut b = World::gather_states_w(states, index_b);
            let original_a = a;
            let original_b = b;
            let restitution_mask = wide_gt(c.restitution, FloatW::ZERO);
            let dp = sub_w(b.dp, a.dp);
            for offset in 0..c.manifold_count {
                // SAFETY: the manifold slot was initialized by `prepare_wide_batch`.
                let m = unsafe { pool.add(c.manifold_start + offset) };
                // SAFETY: `m` addresses a live wide manifold.
                let normal = unsafe { (*m).normal };
                let separation = dot_w(normal, dp);
                let normal_a = self.inv_rotate_vec_w(a.dq, normal);
                let normal_b = self.inv_rotate_vec_w(b.dq, normal);
                for pi in 0..unsafe { (*m).point_count } {
                    let p = point_w(m, pi);
                    // SAFETY: `pi` is below the manifold's point count.
                    unsafe {
                        let mut active = wide_gt_i32_splat((*m).point_counts, pi as i32);
                        if !propagate {
                            active = mask_and(active, restitution_mask);
                        }
                        let s = (separation
                            + (dot_w(normal_b, (*p).r_b) - dot_w(normal_a, (*p).r_a)))
                            + (*p).base_separation;
                        let mass = if propagate {
                            (*p).normal_mass
                        } else {
                            select_w(restitution_mask, (*p).normal_mass, FloatW::ZERO)
                        };
                        let compression = (*p).total_normal_impulse - (*p).restitution_impulse;
                        let armed = mask_and(
                            mask_and(
                                restitution_mask,
                                wide_lt((*p).relative_velocity, threshold),
                            ),
                            wide_gt(compression, FloatW::ZERO),
                        );
                        let bias = select_w(
                            armed,
                            c.restitution * (*p).relative_velocity,
                            max_w(FloatW::ZERO, s * inv_h),
                        );
                        let vr_a = add_w(a.v, cross_w(a.w, (*p).r_a));
                        let vr_b = add_w(b.v, cross_w(b.w, (*p).r_b));
                        let vn = dot_w(sub_w(vr_b, vr_a), normal);
                        let neg = mass * (vn + bias);
                        let mut impulse = max_w((*p).normal_impulse - neg, FloatW::ZERO)
                            - (*p).normal_impulse;
                        let approach = min_w(
                            max_w(neg_w(mass) * vn, FloatW::ZERO),
                            max_w(impulse, FloatW::ZERO),
                        );
                        let allowance =
                            c.restitution * (compression + approach) - (*p).restitution_impulse;
                        impulse = select_w(
                            active,
                            select_w(
                                armed,
                                min_w(impulse, approach + max_w(allowance, FloatW::ZERO)),
                                impulse,
                            ),
                            FloatW::ZERO,
                        );
                        (*p).normal_impulse += impulse;
                        (*p).restitution_impulse += impulse - approach;
                        (*p).total_normal_impulse += impulse;
                        let linear = mul_w(impulse, normal);
                        a.w = select_vec3_w(
                            active,
                            sub_w(a.w, mul_mv_w(inv_ia, cross_w((*p).r_a, linear))),
                            a.w,
                        );
                        a.v = select_vec3_w(active, sub_w(a.v, mul_w(inv_mass_a, linear)), a.v);
                        b.w = select_vec3_w(
                            active,
                            add_w(b.w, mul_mv_w(inv_ib, cross_w((*p).r_b, linear))),
                            b.w,
                        );
                        b.v = select_vec3_w(active, add_w(b.v, mul_w(inv_mass_b, linear)), b.v);
                    }
                }
            }
            if !propagate {
                a.v = select_vec3_w(restitution_mask, a.v, original_a.v);
                a.w = select_vec3_w(restitution_mask, a.w, original_a.w);
                b.v = select_vec3_w(restitution_mask, b.v, original_b.v);
                b.w = select_vec3_w(restitution_mask, b.w, original_b.w);
            }
            World::scatter_states_w(states, index_a, a);
            World::scatter_states_w(states, index_b, b);
        }
    }

    /// Persist directly from SoA lanes without first copying back into scalar constraints.
    pub(crate) fn store_impulses_wide(&mut self, color: i32, mesh: bool) {
        let ci = color as usize;
        let split = self.solver_contacts.wide_convex_counts[ci] as usize;
        let wide = &self.solver_contacts.wide_constraints[ci];
        let range = if mesh { split..wide.len() } else { 0..split };
        if range.is_empty() {
            return;
        }
        let wide_base = wide.as_ptr();
        let pool_base = self.solver_contacts.wide_manifolds.as_ptr();
        let store_base = self.contacts.as_mut_ptr();
        self.store_impulses_wide_range(wide_base, range, pool_base, store_base);
    }

    pub(crate) fn store_impulses_wide_range(
        &self,
        wide_base: *const ContactConstraintW,
        range: Range<usize>,
        pool_base: *const ManifoldConstraintW,
        store_base: *mut Contact,
    ) {
        for i in range {
            // SAFETY: `i` is inside the wide constraint run for this stage.
            let c = unsafe { &*wide_base.add(i) };
            for lane in 0..4 {
                // SAFETY: `id` and `lane_count` come from a prepared constraint; negative ids
                // mark absent lanes and are skipped exactly as Swift's guards did.
                let id = unsafe { c.contact_ids[lane] };
                if id < 0 {
                    continue;
                }
                let id = id as usize;
                let lane_count = unsafe { c.lane_manifold_counts[lane] };
                if lane_count <= 0 {
                    continue;
                }
                let lane_count = lane_count as usize;
                let hit_events = unsafe {
                    ((*store_base.add(id)).flags & ContactFlags::SIM_ENABLE_HIT_EVENT.0) != 0
                };
                // SAFETY: the contact at `id` is alive for the store stage; Swift force-unwrapped
                // the same manifold buffer base. Casting away the const like Swift's
                // `UnsafeMutablePointer(mutating:)` keeps writes flowing to the payload slab.
                let manifold_base = unsafe {
                    (&*store_base.add(id))
                        .manifolds()
                        .buffer()
                        .as_ptr()
                        .cast_mut()
                };
                for offset in 0..lane_count {
                    // SAFETY: `pool_base + manifold_start + offset` was prepared for this lane's
                    // contact, and `manifold_base + offset` is its scalar twin.
                    unsafe {
                        let m = &*pool_base.add(c.manifold_start + offset);
                        let out = manifold_base.add(offset);
                        for pi in 0..(*out).point_count as usize {
                            (*out).points[pi].normal_impulse =
                                m.points[pi].normal_impulse.lane(lane);
                            (*out).points[pi].total_normal_impulse =
                                m.points[pi].total_normal_impulse.lane(lane);
                            if hit_events {
                                (*out).points[pi].normal_velocity =
                                    m.points[pi].relative_velocity.lane(lane);
                            }
                        }
                        (*out).twist_impulse = m.twist_impulse.lane(lane);
                        let fx = m.friction_impulse.x.lane(lane);
                        let fy = m.friction_impulse.y.lane(lane);
                        let t1 = m.tangent1.lane(lane);
                        let t2 = m.tangent2.lane(lane);
                        (*out).friction_impulse = Vec3::new(
                            fx * t1.x + fy * t2.x,
                            fx * t1.y + fy * t2.y,
                            fx * t1.z + fy * t2.z,
                        );
                        (*out).rolling_impulse = m.rolling_impulse.lane(lane);
                    }
                }
            }
        }
    }

    /// Scatter accumulated impulses into the scalar constraints. Test support: requires
    /// `prepare_scalar_for_wide_colors` so every wide lane has a scalar twin.
    pub(crate) fn unpack_wide_impulses(&mut self) {
        if !self.enable_wide_solver {
            return;
        }
        for ci in 0..OVERFLOW_INDEX as usize {
            let list = &self.solver_contacts.constraints[ci];
            let wide = &self.solver_contacts.wide_constraints[ci];
            let manifolds = &self.solver_contacts.wide_manifolds;
            let pool_base = self.solver_contacts.manifold_pool.as_mut_ptr();
            // SAFETY: the scalar manifold pool is not resized while impulses are unpacked, so
            // `pool_base` stays valid for every slot below; mirrors Swift's
            // `manifoldPool.withUnsafeMutableBufferPointer`.
            for c in wide {
                for lane in 0..4 {
                    let id = c.contact_ids[lane];
                    if id < 0 {
                        continue;
                    }
                    let index = match (0..list.len()).find(|&k| list[k].contact_id == id) {
                        Some(k) => k,
                        None => continue,
                    };
                    let s = list[index];
                    for offset in 0..s.manifold_count as usize {
                        let m = &manifolds[c.manifold_start + offset];
                        let out = unsafe { pool_base.add(s.manifold_start as usize + offset) };
                        // SAFETY: `prepare_scalar_for_wide_colors` gave the scalar twin the same
                        // manifold run; the slot is inside the pool for this stage.
                        unsafe {
                            (*out).twist_impulse = m.twist_impulse.lane(lane);
                            (*out).friction_impulse = m.friction_impulse.lane(lane);
                            (*out).rolling_impulse = m.rolling_impulse.lane(lane);
                            for pi in 0..(*out).point_count as usize {
                                (*out).points[pi].normal_impulse =
                                    m.points[pi].normal_impulse.lane(lane);
                                (*out).points[pi].total_normal_impulse =
                                    m.points[pi].total_normal_impulse.lane(lane);
                            }
                        }
                    }
                }
            }
        }
    }

    /// Load four body states as rows and transpose them into lanes. A null index
    /// reads the identity state: zero velocity and delta position, identity delta rotation.
    #[inline(always)]
    pub(crate) fn gather_states_w(states: &mut [BodyState], indices: [i32; 4]) -> BodyStateW {
        let base = states.as_ptr() as *const u8;
        #[inline(always)]
        fn rows(base: *const u8, index: i32) -> (FloatW, FloatW, FloatW, FloatW) {
            if index < 0 {
                return (
                    FloatW::ZERO,
                    FloatW::ZERO,
                    FloatW::ZERO,
                    FloatW::new(0.0, 0.0, 0.0, 1.0),
                );
            }
            // SAFETY: a non-negative index addresses a live `BodyState` in the awake set; the
            // four rows are its 64 bytes read in Swift's order.
            unsafe {
                let p = base.add(index as usize * 64);
                (
                    load_row(p, 0),
                    load_row(p, 16),
                    load_row(p, 32),
                    load_row(p, 48),
                )
            }
        }
        let a = rows(base, indices[0]);
        let b = rows(base, indices[1]);
        let c = rows(base, indices[2]);
        let d = rows(base, indices[3]);
        let v = transpose4(a.0, b.0, c.0, d.0);
        let w = transpose4(a.1, b.1, c.1, d.1);
        let dp = transpose4(a.2, b.2, c.2, d.2);
        let dq = transpose4(a.3, b.3, c.3, d.3);
        BodyStateW::new(
            Vec3W::new(v.0, v.1, v.2),
            Vec3W::new(w.0, w.1, w.2),
            Vec3W::new(dp.0, dp.1, dp.2),
            QuatW::new(Vec3W::new(dq.0, dq.1, dq.2), dq.3),
        )
    }

    /// Write only the velocities back, as two 16-byte rows per dynamic body.
    #[inline(always)]
    pub(crate) fn scatter_states_w(states: &mut [BodyState], indices: [i32; 4], value: BodyStateW) {
        let base = states.as_mut_ptr() as *mut u8;
        let dynamic_flag = BodyFlags::DYNAMIC_FLAG.0;
        let v = transpose4(value.v.x, value.v.y, value.v.z, FloatW::ZERO);
        let w = transpose4(value.w.x, value.w.y, value.w.z, FloatW::ZERO);
        #[inline(always)]
        fn store(base: *mut u8, index: i32, v: FloatW, w: FloatW, dynamic_flag: u32) {
            if index < 0 {
                return;
            }
            // SAFETY: a non-negative index addresses a live awake-set state. The flags word sits
            // in the padding lane after `delta_position` at byte 44, exactly as Swift read it.
            unsafe {
                let p = base.add(index as usize * 64);
                let flags = (p.add(44) as *const u32).read_unaligned();
                if flags & dynamic_flag == 0 {
                    return;
                }
                store_row(p, 0, v);
                store_row(p, 16, w);
            }
        }
        store(base, indices[0], v.0, w.0, dynamic_flag);
        store(base, indices[1], v.1, w.1, dynamic_flag);
        store(base, indices[2], v.2, w.2, dynamic_flag);
        store(base, indices[3], v.3, w.3, dynamic_flag);
    }
}

