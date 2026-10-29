//! Direct SoA preparation for independent colors. Lanes are filled straight from the contacts,
//! so wide colors no longer build scalar constraints only to repack them.
//!
//! Every lane operation keeps the scalar operand order, association, and select semantics of the
//! Swift original, so each lane is bit-identical to the scalar preparation path.

use crate::base::constants as b3;
use crate::simulation::body_api::ContactFlags;
use crate::simulation::solver_integration::Softness;
use crate::simulation::wide_contact_solver::{
    ContactConstraintW, ConstraintPointW, ManifoldConstraintW, mask_and, mask_any, mask_or,
    wide_gt_i32_splat,
};
use crate::simulation::wide_types::{
    BodyStateW, FloatW, MaskW, SymMatrix3W, Vec2W, Vec3W, add_w, cross_w, dot_w, max_w, min_w,
    mul_add_w, mul_mv_w, mul_w, neg_w, select_vec3_w, select_w, sub_w, transpose4, wide_gt,
    wide_lt,
};
use crate::simulation::world_runtime::{BodySim, BodyState, Contact, World, decode_awake_index};
use crate::types::collision_types::{Manifold, ManifoldPoint};

impl World {
    /// Prepare one batch of up to four contacts into its preassigned slots. Its manifolds go to
    /// `manifold_out`, uninitialized memory with room for the largest lane manifold count.
    pub(crate) fn prepare_wide_batch(
        &self,
        ids: *const i32,
        lane_count: usize,
        out: *mut ContactConstraintW,
        manifold_out: *mut ManifoldConstraintW,
        manifold_start: usize,
        contact_base: *const Contact,
        sims: *const BodySim,
        states: &mut [BodyState],
        softness: Softness,
        static_softness: Softness,
        warm_start_scale: f32,
    ) {
        let inv_tau = FloatW::splat(1.0 / b3::speculative_distance());
        let min_friction_weight = FloatW::splat(b3::MIN_FRICTION_WEIGHT);
        let one = FloatW::splat(1.0);
        let two = FloatW::splat(2.0);
        let min_det = FloatW::splat(1000.0 * f32::MIN_POSITIVE);
        let ws = FloatW::splat(warm_start_scale);
        let dynamic_bias = softness.mass_scale * softness.bias_rate;
        let static_bias = static_softness.mass_scale * static_softness.bias_rate;
        let hit_flag = ContactFlags::SIM_ENABLE_HIT_EVENT.0;
        let static_flag = ContactFlags::STATIC_FLAG.0;

        let mut c = ContactConstraintW::new();
        let mut manifolds: [*const Manifold; 4] = [std::ptr::null(); 4];
        let mut manifold_counts: [i32; 4] = [0; 4];
        let mut need_velocity = MaskW::default();
        let mut tangent_velocity = Vec3W::zero();
        let mut sim_lanes_a: [*const u8; 4] = [std::ptr::null(); 4];
        let mut sim_lanes_b: [*const u8; 4] = [std::ptr::null(); 4];
        // Constant lane indices let the compiler insert each scalar directly
        // into its vector instead of spilling through a dynamic SIMD subscript.
        // Swift used an `@inline(__always)` nested function; the closure below is the same shape,
        // called once per lane with a literal index.
        let mut gather_lane = |lane: usize| {
            if lane >= lane_count {
                return;
            }
            // SAFETY: `plan_contact_preparation` sized the id run to `lane_count` and every id is
            // a live contact index; the contact array is not resized during the stage.
            let id = unsafe { *ids.add(lane) };
            // SAFETY: as above; `id` indexes `contact_base` for the whole batch.
            let contact = unsafe { &*contact_base.add(id as usize) };
            let index_a = decode_awake_index(contact.encoded_body_sim_a);
            let index_b = decode_awake_index(contact.encoded_body_sim_b);
            c.contact_ids[lane] = id;
            c.index_a[lane] = index_a;
            c.index_b[lane] = index_b;
            if index_a >= 0 {
                // SAFETY: a decoded awake index addresses a live sim; the sim array is fixed for
                // the stage.
                let sim = unsafe { sims.add(index_a as usize) };
                c.inv_mass_a[lane] = unsafe { (*sim).inv_mass };
                sim_lanes_a[lane] = sim.cast::<u8>();
            }
            if index_b >= 0 {
                // SAFETY: as for lane A.
                let sim = unsafe { sims.add(index_b as usize) };
                c.inv_mass_b[lane] = unsafe { (*sim).inv_mass };
                sim_lanes_b[lane] = sim.cast::<u8>();
            }
            let is_static = (contact.flags & static_flag) != 0;
            let soft = if is_static { static_softness } else { softness };
            c.bias_rate[lane] = if is_static { static_bias } else { dynamic_bias };
            c.mass_scale[lane] = soft.mass_scale;
            c.impulse_scale[lane] = soft.impulse_scale;
            let restitution = contact.restitution;
            let rolling = contact.rolling_resistance;
            c.friction[lane] = contact.friction;
            c.restitution[lane] = restitution;
            c.rolling_resistance[lane] = rolling;
            c.any_rolling = c.any_rolling || rolling > 0.0;
            c.any_restitution = c.any_restitution || restitution > 0.0;
            need_velocity.bits[lane] = if restitution > 0.0 || (contact.flags & hit_flag) != 0 {
                u32::MAX
            } else {
                0
            };
            tangent_velocity.set_lane(lane, contact.tangent_velocity);
            let mc = contact.manifold_count;
            manifold_counts[lane] = mc;
            c.manifold_count = std::cmp::max(c.manifold_count, mc as usize);
            if mc > 0 {
                // SAFETY: a contact with manifolds has a live payload, the same invariant Swift's
                // force-unwrapped `baseAddress!` relied on; the pointer addresses the payload's
                // manifold run which outlives the view borrow.
                manifolds[lane] = contact.manifolds().buffer().as_ptr();
            }
        };
        gather_lane(0);
        gather_lane(1);
        gather_lane(2);
        gather_lane(3);
        c.inv_ia = gather_inv_inertia_w(sim_lanes_a);
        c.inv_ib = gather_inv_inertia_w(sim_lanes_b);
        c.lane_manifold_counts = manifold_counts;

        let m_a = c.inv_mass_a;
        let m_b = c.inv_mass_b;
        let i_a = c.inv_ia;
        let i_b = c.inv_ib;
        let inv_iab = add_sym_w(i_a, i_b);
        let rolling_mask = wide_gt(c.rolling_resistance, FloatW::ZERO);
        if c.any_rolling {
            let inv = invert_sym_w(inv_iab);
            c.rolling_mass.xx = select_w(rolling_mask, inv.xx, FloatW::ZERO);
            c.rolling_mass.yy = select_w(rolling_mask, inv.yy, FloatW::ZERO);
            c.rolling_mass.zz = select_w(rolling_mask, inv.zz, FloatW::ZERO);
            c.rolling_mass.xy = select_w(rolling_mask, inv.xy, FloatW::ZERO);
            c.rolling_mass.xz = select_w(rolling_mask, inv.xz, FloatW::ZERO);
            c.rolling_mass.yz = select_w(rolling_mask, inv.yz, FloatW::ZERO);
        }

        let mut state_a = BodyStateW::default();
        let mut state_b = BodyStateW::default();
        let any_velocity = mask_any(need_velocity);
        if any_velocity {
            state_a = World::gather_states_w(states, c.index_a);
            state_b = World::gather_states_w(states, c.index_b);
        }

        c.manifold_start = manifold_start;
        for offset in 0..c.manifold_count {
            let mut m = ManifoldConstraintW::new();
            let mut lanes: [*const Manifold; 4] = [std::ptr::null(); 4];
            for lane in 0..4 {
                if offset < manifold_counts[lane] as usize {
                    // SAFETY: a lane with a manifold count above `offset` had its base pointer set
                    // above, and the run holds `manifold_counts[lane]` initialized manifolds.
                    let sm = unsafe { manifolds[lane].add(offset) };
                    lanes[lane] = sm;
                    m.exists[lane] = 1;
                    // SAFETY: `sm` addresses an initialized manifold.
                    let pc = unsafe { (*sm).point_count };
                    m.point_counts[lane] = pc;
                    m.point_count = std::cmp::max(m.point_count, pc as usize);
                }
            }
            let header = gather_manifold_header_w(lanes);
            let normal = header.normal;

            let tangent1 = perp_w(normal);
            let tangent2 = cross_w(tangent1, normal);
            m.normal = normal;
            m.tangent1 = tangent1;
            m.tangent2 = tangent2;
            m.tangent_velocity1 = dot_w(tangent_velocity, tangent1);
            m.tangent_velocity2 = dot_w(tangent_velocity, tangent2);
            m.twist_impulse = ws * header.twist_impulse;
            let rolling_keep = mask_and(rolling_mask, wide_gt_i32_splat(m.exists, 0));
            m.rolling_impulse = select_vec3_w(
                rolling_keep,
                mul_w(ws, header.rolling_impulse),
                Vec3W::zero(),
            );
            m.friction_impulse = Vec2W::new(
                ws * dot_w(header.friction_impulse, tangent1),
                ws * dot_w(header.friction_impulse, tangent2),
            );

            let mut center_a = Vec3W::zero();
            let mut center_b = Vec3W::zero();
            let mut total_weight = FloatW::ZERO;
            for pi in 0..m.point_count {
                let point = gather_manifold_point_w(lanes, pi);
                let r_a = point.r_a;
                let r_b = point.r_b;
                let separation = point.separation;
                let normal_impulse = point.normal_impulse;
                let point_mask = wide_gt_i32_splat(m.point_counts, pi as i32);
                let mut weight = min_w(max_w(two - separation * inv_tau, min_friction_weight), one);
                weight = select_w(point_mask, weight, FloatW::ZERO);
                center_a = mul_add_w(center_a, weight, r_a);
                center_b = mul_add_w(center_b, weight, r_b);
                total_weight += weight;

                let mut cp = ConstraintPointW::new();
                cp.r_a = r_a;
                cp.r_b = r_b;
                cp.base_separation = separation - dot_w(sub_w(r_b, r_a), normal);
                cp.normal_impulse = ws * normal_impulse;

                let rn_a = cross_w(r_a, normal);
                let rn_b = cross_w(r_b, normal);
                let mut k_normal = m_a + m_b;
                k_normal += dot_w(rn_a, mul_mv_w(i_a, rn_a));
                k_normal += dot_w(rn_b, mul_mv_w(i_b, rn_b));
                cp.normal_mass = select_w(
                    mask_and(wide_gt(k_normal, FloatW::ZERO), point_mask),
                    one / k_normal,
                    FloatW::ZERO,
                );
                m.points[pi] = cp;
            }

            let friction_valid = wide_gt(total_weight, FloatW::ZERO);
            let inv_weight = select_w(friction_valid, one / total_weight, FloatW::ZERO);
            center_a = mul_w(inv_weight, center_a);
            center_b = mul_w(inv_weight, center_b);
            m.center_a = center_a;
            m.center_b = center_b;

            for pi in 0..m.point_count {
                let point_mask = wide_gt_i32_splat(m.point_counts, pi as i32);
                let d = sub_w(m.points[pi].r_a, center_a);
                m.points[pi].lever_arm =
                    select_w(point_mask, dot_w(d, d).square_root(), FloatW::ZERO);
            }

            {
                let rt_a1 = cross_w(center_a, tangent1);
                let rt_a2 = cross_w(center_a, tangent2);
                let rt_b1 = cross_w(center_b, tangent1);
                let rt_b2 = cross_w(center_b, tangent2);
                let i_art_a1 = mul_mv_w(i_a, rt_a1);
                let i_art_a2 = mul_mv_w(i_a, rt_a2);
                let i_brt_b1 = mul_mv_w(i_b, rt_b1);
                let i_brt_b2 = mul_mv_w(i_b, rt_b2);
                let m_sum = m_a + m_b;
                let kxx = (m_sum + dot_w(rt_a1, i_art_a1)) + dot_w(rt_b1, i_brt_b1);
                let kyy = (m_sum + dot_w(rt_a2, i_art_a2)) + dot_w(rt_b2, i_brt_b2);
                let kxy = dot_w(rt_a1, i_art_a2) + dot_w(rt_b1, i_brt_b2);
                let det = kxx * kyy - kxy * kxy;
                let valid = mask_and(wide_gt(abs_w(det), min_det), friction_valid);
                let inv_det = select_w(valid, one / det, FloatW::ZERO);
                m.tangent_mass.xx = inv_det * kyy;
                m.tangent_mass.xy = neg_w(inv_det * kxy);
                m.tangent_mass.yy = inv_det * kxx;
            }

            let k_twist = dot_w(normal, mul_mv_w(inv_iab, normal));
            m.twist_mass = select_w(wide_gt(k_twist, FloatW::ZERO), one / k_twist, FloatW::ZERO);

            if any_velocity {
                for pi in 0..m.point_count {
                    let r_a = m.points[pi].r_a;
                    let r_b = m.points[pi].r_b;
                    let vr_a = add_w(state_a.v, cross_w(state_a.w, r_a));
                    let vr_b = add_w(state_b.v, cross_w(state_b.w, r_b));
                    let vn = dot_w(normal, sub_w(vr_b, vr_a));
                    let point_mask = wide_gt_i32_splat(m.point_counts, pi as i32);
                    m.points[pi].relative_velocity =
                        select_w(mask_and(need_velocity, point_mask), vn, FloatW::ZERO);
                }
            }
            // SAFETY: `manifold_out` is uninitialized memory reserved for this batch's largest
            // lane manifold count, and each offset is written exactly once.
            unsafe { manifold_out.add(offset).write(m) };
        }
        // SAFETY: `out` is this batch's single uninitialized constraint slot.
        unsafe { out.write(c) };
    }
}

// MARK: - Lane-wise scalar helpers

/// Swift's `absW`: clear the sign bit per lane, leaving NaN payloads and signed zeros untouched.
#[inline(always)]
pub(crate) fn abs_w(a: FloatW) -> FloatW {
    let bits = a.to_bits();
    FloatW::from_bits([
        bits[0] & 0x7FFF_FFFF,
        bits[1] & 0x7FFF_FFFF,
        bits[2] & 0x7FFF_FFFF,
        bits[3] & 0x7FFF_FFFF,
    ])
}

#[inline(always)]
pub(crate) fn add_sym_w(a: SymMatrix3W, b: SymMatrix3W) -> SymMatrix3W {
    SymMatrix3W {
        xx: a.xx + b.xx,
        xy: a.xy + b.xy,
        xz: a.xz + b.xz,
        yy: a.yy + b.yy,
        yz: a.yz + b.yz,
        zz: a.zz + b.zz,
    }
}

/// Lane-wise `invert_sym`.
#[inline(always)]
pub(crate) fn invert_sym_w(m: SymMatrix3W) -> SymMatrix3W {
    let cxx = m.yy * m.zz - m.yz * m.yz;
    let cxy = m.xz * m.yz - m.xy * m.zz;
    let cxz = m.xy * m.yz - m.xz * m.yy;
    let cyy = m.xx * m.zz - m.xz * m.xz;
    let cyz = m.xy * m.xz - m.xx * m.yz;
    let czz = m.xx * m.yy - m.xy * m.xy;
    let det = m.xx * cxx + (m.xy * cxy + m.xz * cxz);
    let valid = wide_gt(abs_w(det), FloatW::splat(1000.0 * f32::MIN_POSITIVE));
    let inv_det = select_w(valid, FloatW::splat(1.0) / det, FloatW::ZERO);
    SymMatrix3W {
        xx: inv_det * cxx,
        xy: inv_det * cxy,
        xz: inv_det * cxz,
        yy: inv_det * cyy,
        yz: inv_det * cyz,
        zz: inv_det * czz,
    }
}

/// Lane-wise `perp`.
#[inline(always)]
pub(crate) fn perp_w(a: Vec3W) -> Vec3W {
    let half = FloatW::splat(0.5);
    let mask = mask_or(wide_lt(a.x, neg_w(half)), wide_gt(a.x, half));
    let p = Vec3W::new(
        select_w(mask, a.y, FloatW::ZERO),
        select_w(mask, neg_w(a.x), a.z),
        select_w(mask, FloatW::ZERO, neg_w(a.y)),
    );
    let length_squared = dot_w(p, p);
    let valid = wide_gt(length_squared, FloatW::splat(1000.0 * f32::MIN_POSITIVE));
    let s = select_w(valid, FloatW::splat(1.0) / length_squared.square_root(), FloatW::ZERO);
    mul_w(s, p)
}

// MARK: - Row gathers

/// One 16-byte row of four `f32` lanes from raw memory; a null lane pointer is all zero.
/// The rows lie inside live, initialized objects but are addressed at arbitrary 4-byte offsets,
/// hence the unaligned loads.
#[inline(always)]
fn row(p: *const u8, offset: usize) -> FloatW {
    if p.is_null() {
        return FloatW::ZERO;
    }
    // SAFETY: callers pass an offset that stays inside the pointee of a live, initialized object.
    unsafe {
        let q = p.add(offset) as *const f32;
        FloatW::new(
            q.read_unaligned(),
            q.add(1).read_unaligned(),
            q.add(2).read_unaligned(),
            q.add(3).read_unaligned(),
        )
    }
}

/// One `f32` from raw memory; a null lane pointer is zero.
#[inline(always)]
fn scalar(p: *const u8, offset: usize) -> f32 {
    if p.is_null() {
        return 0.0;
    }
    // SAFETY: as for `row`; the scalar lies inside the pointee.
    unsafe { (p.add(offset) as *const f32).read_unaligned() }
}

/// `BodySim.invInertiaWorld` is a column-major `Matrix3` whose columns overlap the symmetric
/// entries: bytes 0..11 are (xx, xy, xz), 12..23 are (xy, yy, yz), and 24..35 are (xz, yz, zz),
/// so row 0 yields (xx, xy, xz), row 1 starts at the `yy`, and the `zz` is a scalar at +32.
const INV_INERTIA_OFFSET: usize = core::mem::offset_of!(BodySim, inv_inertia_world);

/// The upper triangle of each lane's world inverse inertia. Absent lanes are zero.
#[inline(always)]
pub(crate) fn gather_inv_inertia_w(sims: [*const u8; 4]) -> SymMatrix3W {
    let o = INV_INERTIA_OFFSET;
    let r0 = transpose4(row(sims[0], o), row(sims[1], o), row(sims[2], o), row(sims[3], o));
    let r1 = transpose4(
        row(sims[0], o + 16),
        row(sims[1], o + 16),
        row(sims[2], o + 16),
        row(sims[3], o + 16),
    );
    SymMatrix3W {
        xx: r0.0,
        xy: r0.1,
        xz: r0.2,
        yy: r1.0,
        yz: r1.1,
        zz: FloatW::new(
            scalar(sims[0], o + 32),
            scalar(sims[1], o + 32),
            scalar(sims[2], o + 32),
            scalar(sims[3], o + 32),
        ),
    }
}

/// Normal, twist, friction, and rolling impulses are contiguous after the points, so the first
/// row is (nx, ny, nz, twist), the second (fx, fy, fz, rx), and the last two rolling scalars sit
/// at +32 and +36.
const MANIFOLD_NORMAL_OFFSET: usize = core::mem::offset_of!(Manifold, normal);
const MANIFOLD_POINT_STRIDE: usize = core::mem::size_of::<ManifoldPoint>();

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub(crate) struct ManifoldHeaderW {
    pub(crate) normal: Vec3W,
    pub(crate) twist_impulse: FloatW,
    pub(crate) friction_impulse: Vec3W,
    pub(crate) rolling_impulse: Vec3W,
}

/// normal, twist, friction, and rolling impulses are contiguous after the points. Absent lanes
/// are zero.
#[inline(always)]
pub(crate) fn gather_manifold_header_w(lanes: [*const Manifold; 4]) -> ManifoldHeaderW {
    let p0 = lanes[0].cast::<u8>();
    let p1 = lanes[1].cast::<u8>();
    let p2 = lanes[2].cast::<u8>();
    let p3 = lanes[3].cast::<u8>();
    let o = MANIFOLD_NORMAL_OFFSET;
    let a = transpose4(row(p0, o), row(p1, o), row(p2, o), row(p3, o));
    let b = transpose4(
        row(p0, o + 16),
        row(p1, o + 16),
        row(p2, o + 16),
        row(p3, o + 16),
    );
    ManifoldHeaderW {
        normal: Vec3W::new(a.0, a.1, a.2),
        twist_impulse: a.3,
        friction_impulse: Vec3W::new(b.0, b.1, b.2),
        rolling_impulse: Vec3W::new(
            b.3,
            FloatW::new(
                scalar(p0, o + 32),
                scalar(p1, o + 32),
                scalar(p2, o + 32),
                scalar(p3, o + 32),
            ),
            FloatW::new(
                scalar(p0, o + 36),
                scalar(p1, o + 36),
                scalar(p2, o + 36),
                scalar(p3, o + 36),
            ),
        ),
    }
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub(crate) struct ManifoldPointW {
    pub(crate) r_a: Vec3W,
    pub(crate) r_b: Vec3W,
    pub(crate) separation: FloatW,
    pub(crate) normal_impulse: FloatW,
}

/// anchorA, anchorB, separation, and normal impulse of point `index`. Lanes without the point
/// are zero. The two rows are (ax, ay, az, bx) and (by, bz, separation, normalImpulse):
/// `ManifoldPoint` is all `f32` fields in that order, so the second anchor begins mid-row.
#[inline(always)]
pub(crate) fn gather_manifold_point_w(
    lanes: [*const Manifold; 4],
    index: usize,
) -> ManifoldPointW {
    #[inline(always)]
    fn point(m: *const Manifold, index: usize) -> *const u8 {
        if m.is_null() {
            return std::ptr::null();
        }
        // SAFETY: a non-null lane pointer addresses an initialized manifold for the whole batch.
        let pc = unsafe { (*m).point_count };
        if index >= pc as usize {
            return std::ptr::null();
        }
        // SAFETY: `index` is below the point count, so the slot lies inside the manifold's
        // four-point inline array.
        unsafe { m.cast::<u8>().add(index * MANIFOLD_POINT_STRIDE) }
    }
    let p0 = point(lanes[0], index);
    let p1 = point(lanes[1], index);
    let p2 = point(lanes[2], index);
    let p3 = point(lanes[3], index);
    let a = transpose4(row(p0, 0), row(p1, 0), row(p2, 0), row(p3, 0));
    let b = transpose4(row(p0, 16), row(p1, 16), row(p2, 16), row(p3, 16));
    ManifoldPointW {
        r_a: Vec3W::new(a.0, a.1, a.2),
        r_b: Vec3W::new(a.3, b.0, b.1),
        separation: b.2,
        normal_impulse: b.3,
    }
}
