//! Supplemental moving-pair continuous collision for dynamic and kinematic bodies.
//!
//! Translated from `Sources/Natsel/Simulation/Continuous/DynamicImpact.swift`.

use std::collections::HashMap;

use crate::base::constants as b3;
use crate::math::functions::{
    aabb_inflate, aabb_union, abs_vec3, cross, dot, inv_mul_quat, length, length_squared,
    make_matrix_from_quat, max_vec3, min_vec3, mul_mm, mul_mv, nlerp, offset_pos, rotate_vector,
    sub_pos, transpose,
};
use crate::math::spatial_pairs::overlapping_bounds_pairs;
use crate::math::types::{AABB, Matrix3, Pos, Quat, Transform, Vec3};
use crate::math::world_aabb::WorldAABB;
use crate::simulation::shape_runtime::{compute_shape_aabb, is_convex, should_shapes_collide, ShapeFlags};
use crate::simulation::solver_integration::FinalizeFlag;
use crate::simulation::world_runtime::{BodyFlags, BodySim, BodyState, SetType, World};
use crate::types::body_def::BodyType;
use crate::types::events::ContactHitEvent;
use crate::types::ids::ShapeID;
use crate::types::queries::Sweep;
use crate::types::queries::TOIOutput;

/// Captured before finalization resets slow bodies' previous poses. Every pair uses the same
/// interval, regardless of body creation order or worker scheduling.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ContinuousStart {
    pub center: Pos,
    pub rotation: Quat,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct DynamicContinuousImpulse {
    pub body: i32,
    pub point: Pos,
    pub impulse: Vec3,
}

#[derive(Clone, Copy)]
struct ContinuousCandidate {
    shape: i32,
    body: i32,
    sim: isize,
    start: ContinuousStart,
    end: ContinuousStart,
    local_center: Vec3,
    bounds: WorldAABB,
    dynamic: bool,
    fast: bool,
    inv_mass: f32,
    inv_inertia_local: Matrix3,
}

impl ContinuousCandidate {
    fn inverse_inertia(&self, fraction: f32) -> Matrix3 {
        if !self.dynamic {
            return Matrix3::default();
        }
        let rotation = make_matrix_from_quat(nlerp(self.start.rotation, self.end.rotation, fraction));
        mul_mm(mul_mm(rotation, self.inv_inertia_local), transpose(rotation))
    }

    fn sweep(&self, relative_to: Pos) -> Sweep {
        let mut result = Sweep::default();
        result.c1 = sub_pos(self.start.center, relative_to);
        result.c2 = sub_pos(self.end.center, relative_to);
        result.q1 = self.start.rotation;
        result.q2 = self.end.rotation;
        result.local_center = self.local_center;
        result
    }
}

/// One accepted moving-pair TOI. Swift stored this as the tuple `(a: Int, b: Int, output: TOIOutput)`.
struct ContinuousHit {
    a: usize,
    b: usize,
    output: TOIOutput,
}

impl World {
    /// Supplemental moving-pair CCD. Broad phase compares swept bounds in double precision;
    /// endpoint-only proxies would miss bodies crossing between their final positions.
    pub(crate) fn solve_dynamic_continuous(
        &mut self,
        starts: &[ContinuousStart],
        sims: &mut [BodySim],
        states: &mut [BodyState],
    ) {
        self.dynamic_continuous_impulses.clear();
        self.dynamic_continuous_active_bodies.clear();
        if !sims.iter().any(|sim| (sim.flags & BodyFlags::IS_FAST.0) != 0) {
            return;
        }
        let mut candidates: Vec<ContinuousCandidate> = Vec::new();
        for body in self.bodies.iter() {
            if body.id == b3::NULL_INDEX
                || body.type_ == BodyType::StaticBody
                || body.set_index == SetType::DisabledSet.raw_value()
            {
                continue;
            }
            let awake = body.set_index == SetType::AwakeSet.raw_value();
            let index = body.local_index as usize;
            let sim = if awake {
                sims[index]
            } else {
                self.solver_sets[body.set_index as usize].body_sims[index]
            };
            let start = if awake {
                starts[index]
            } else {
                ContinuousStart { center: sim.center, rotation: sim.transform.q }
            };
            let end = ContinuousStart { center: sim.center, rotation: sim.transform.q };
            let mut shape_id = body.head_shape_id;
            while shape_id != b3::NULL_INDEX {
                // Swift advanced `shapeId` in a `defer` at each loop-body exit, including the
                // `continue` guards below; reading the link up front reproduces that.
                let next_shape_id = self.shapes[shape_id as usize].next_shape_id;
                let shape = &self.shapes[shape_id as usize];
                if !is_convex(shape.type_) {
                    shape_id = next_shape_id;
                    continue;
                }
                if shape.sensor_index != b3::NULL_INDEX {
                    shape_id = next_shape_id;
                    continue;
                }
                let xf1 = Transform::new(
                    -rotate_vector(start.rotation, sim.local_center),
                    start.rotation,
                );
                let xf2 = Transform::new(
                    sub_pos(end.center, start.center) - rotate_vector(end.rotation, sim.local_center),
                    end.rotation,
                );
                let mut bounds = aabb_union(
                    compute_shape_aabb(shape, xf1),
                    compute_shape_aabb(shape, xf2),
                );
                // The COM-centered bounding sphere contains the complete rotational arc.
                if length_squared(inv_mul_quat(start.rotation, end.rotation).v) > 1.0e-12 {
                    let box_ = compute_shape_aabb(shape, Transform::IDENTITY);
                    let radius = length(max_vec3(
                        abs_vec3(box_.lower_bound - sim.local_center),
                        abs_vec3(box_.upper_bound - sim.local_center),
                    ));
                    let delta = sub_pos(end.center, start.center);
                    bounds = aabb_union(
                        bounds,
                        aabb_inflate(
                            AABB::new(min_vec3(Vec3::ZERO, delta), max_vec3(Vec3::ZERO, delta)),
                            radius,
                        ),
                    );
                }
                candidates.push(ContinuousCandidate {
                    shape: shape_id,
                    body: body.id,
                    sim: if awake { index as isize } else { -1 },
                    start,
                    end,
                    local_center: sim.local_center,
                    bounds: WorldAABB::from_local_bounds(bounds, start.center),
                    dynamic: body.type_ == BodyType::DynamicBody,
                    fast: awake && (sim.flags & BodyFlags::IS_FAST.0) != 0,
                    inv_mass: if body.type_ == BodyType::DynamicBody { sim.inv_mass } else { 0.0 },
                    inv_inertia_local: sim.inv_inertia_local,
                });
                shape_id = next_shape_id;
            }
        }
        let mut hits: Vec<ContinuousHit> = Vec::new();
        let bounds: Vec<WorldAABB> = candidates.iter().map(|candidate| candidate.bounds).collect();
        for pair in overlapping_bounds_pairs(&bounds).pairs {
            let i = pair.a;
            let b_index = pair.b;
            let a = candidates[i];
            let b = candidates[b_index];
            if a.body == b.body {
                continue;
            }
            if !(a.fast || b.fast) {
                continue;
            }
            if !(a.dynamic || b.dynamic) {
                continue;
            }
            if !a.bounds.overlaps(&b.bounds) {
                continue;
            }
            if !self.should_bodies_collide(&self.bodies[a.body as usize], &self.bodies[b.body as usize])
            {
                continue;
            }
            let shape_a = &self.shapes[a.shape as usize];
            let shape_b = &self.shapes[b.shape as usize];
            if !should_shapes_collide(&shape_a.filter, &shape_b.filter) {
                continue;
            }
            let id_a = ShapeID::new(a.shape + 1, self.world_id, shape_a.generation);
            let id_b = ShapeID::new(b.shape + 1, self.world_id, shape_b.generation);
            if (shape_a.flags | shape_b.flags) & ShapeFlags::ENABLE_CUSTOM_FILTERING.0 != 0 {
                if let Some(filter) = self.custom_filter.as_ref() {
                    if !filter(id_a, id_b) {
                        continue;
                    }
                }
            }
            let result = self.shape_time_of_impact(
                shape_a,
                shape_b,
                a.sweep(a.start.center),
                b.sweep(a.start.center),
                1.0,
            );
            if !(result.fraction > 0.0) {
                continue;
            }
            if !(result.fraction < 1.0) {
                continue;
            }
            if (shape_a.flags | shape_b.flags) & ShapeFlags::ENABLE_PRE_SOLVE_EVENTS.0 != 0 {
                if let Some(callback) = self.pre_solve.as_ref() {
                    if !callback(id_a, id_b, offset_pos(a.start.center, result.point), result.normal)
                    {
                        continue;
                    }
                }
            }
            hits.push(ContinuousHit { a: i, b: b_index, output: result });
        }
        hits.sort_by(|hit0, hit1| {
            if hit0.output.fraction != hit1.output.fraction {
                if hit0.output.fraction < hit1.output.fraction {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                }
            } else if hit0.a != hit1.a {
                if hit0.a < hit1.a {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                }
            } else if hit0.b != hit1.b {
                if hit0.b < hit1.b {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                }
            } else {
                std::cmp::Ordering::Equal
            }
        });
        let mut fractions = vec![1.0f32; sims.len()];
        let mut carries = vec![Vec3::ZERO; sims.len()];
        let mut sleeping_velocities: HashMap<i32, (Vec3, Vec3)> = HashMap::new();
        for hit in &hits {
            let a = candidates[hit.a];
            let b = candidates[hit.b];
            let result = hit.output;
            if !(a.sim < 0 || fractions[a.sim as usize] >= result.fraction) {
                continue;
            }
            if !(b.sim < 0 || fractions[b.sim as usize] >= result.fraction) {
                continue;
            }
            // Resolve the impact velocity now. Waiting for next step's soft contact solver can let
            // fast pairs cross again while its position correction is still converging.
            let normal = result.normal;
            let point = result.point;
            let ca = result.fraction * sub_pos(a.end.center, a.start.center);
            let cb = sub_pos(b.start.center, a.start.center)
                + result.fraction * sub_pos(b.end.center, b.start.center);
            let ra = point - ca;
            let rb = point - cb;
            let ma = a.inv_mass;
            let mb = b.inv_mass;
            let ia = a.inverse_inertia(result.fraction);
            let ib = b.inverse_inertia(result.fraction);
            let sleep_a = sleeping_velocities
                .get(&a.body)
                .copied()
                .unwrap_or((Vec3::ZERO, Vec3::ZERO));
            let sleep_b = sleeping_velocities
                .get(&b.body)
                .copied()
                .unwrap_or((Vec3::ZERO, Vec3::ZERO));
            let va = if a.sim >= 0 {
                let state = states[a.sim as usize];
                state.linear_velocity + cross(state.angular_velocity, ra)
            } else {
                sleep_a.0 + cross(sleep_a.1, ra)
            };
            let vb = if b.sim >= 0 {
                let state = states[b.sim as usize];
                state.linear_velocity + cross(state.angular_velocity, rb)
            } else {
                sleep_b.0 + cross(sleep_b.1, rb)
            };
            let approach = dot(vb - va, normal);
            let ran = cross(ra, normal);
            let rbn = cross(rb, normal);
            let mass = ma + mb + dot(ran, mul_mv(ia, ran)) + dot(rbn, mul_mv(ib, rbn));
            if approach < 0.0 && mass > 0.0 {
                let material_a = self.shapes[a.shape as usize].material(0);
                let material_b = self.shapes[b.shape as usize].material(0);
                let bounce = if -approach > self.restitution_threshold {
                    self.mix_restitution(
                        material_a.restitution,
                        material_a.user_material_id,
                        material_b.restitution,
                        material_b.user_material_id,
                    )
                } else {
                    0.0
                };
                let impulse = (-(1.0 + bounce) * approach / mass) * normal;
                if ma > 0.0 {
                    self.dynamic_continuous_active_bodies.push(a.body);
                    if a.sim >= 0 {
                        states[a.sim as usize].linear_velocity -= ma * impulse;
                        states[a.sim as usize].angular_velocity -= mul_mv(ia, cross(ra, impulse));
                    } else {
                        sleeping_velocities.insert(
                            a.body,
                            (
                                sleep_a.0 - ma * impulse,
                                sleep_a.1 - mul_mv(ia, cross(ra, impulse)),
                            ),
                        );
                        self.dynamic_continuous_impulses.push(DynamicContinuousImpulse {
                            body: a.body,
                            point: offset_pos(a.start.center, result.point),
                            impulse: -impulse,
                        });
                    }
                }
                if mb > 0.0 {
                    self.dynamic_continuous_active_bodies.push(b.body);
                    if b.sim >= 0 {
                        states[b.sim as usize].linear_velocity += mb * impulse;
                        states[b.sim as usize].angular_velocity += mul_mv(ib, cross(rb, impulse));
                    } else {
                        sleeping_velocities.insert(
                            b.body,
                            (
                                sleep_b.0 + mb * impulse,
                                sleep_b.1 + mul_mv(ib, cross(rb, impulse)),
                            ),
                        );
                        self.dynamic_continuous_impulses.push(DynamicContinuousImpulse {
                            body: b.body,
                            point: offset_pos(a.start.center, result.point),
                            impulse,
                        });
                    }
                }
                let shape_a = &self.shapes[a.shape as usize];
                let shape_b = &self.shapes[b.shape as usize];
                let flags = shape_a.flags | shape_b.flags;
                if flags
                    & (ShapeFlags::ENABLE_CONTACT_EVENTS.0 | ShapeFlags::ENABLE_HIT_EVENTS.0)
                    != 0
                {
                    let mut event = ContactHitEvent::default();
                    event.shape_id_a = self.make_shape_id(a.shape);
                    event.shape_id_b = self.make_shape_id(b.shape);
                    event.point = offset_pos(a.start.center, result.point);
                    event.normal = normal;
                    event.approach_speed = -approach;
                    event.user_material_id_a = material_a.user_material_id;
                    event.user_material_id_b = material_b.user_material_id;
                    self.continuous_impact_events.push(event);
                    if -approach > self.hit_event_threshold
                        && flags & ShapeFlags::ENABLE_HIT_EVENTS.0 != 0
                    {
                        self.contact_hit_events.push(event);
                    }
                }
            }
            for (mover, other, n) in [(a, b, normal), (b, a, -normal)] {
                if mover.dynamic
                    && mover.sim >= 0
                    && result.fraction < fractions[mover.sim as usize]
                {
                    fractions[mover.sim as usize] = result.fraction;
                    carries[mover.sim as usize] = if other.dynamic {
                        Vec3::ZERO
                    } else {
                        ((1.0 - result.fraction)
                            * dot(sub_pos(other.end.center, other.start.center), n))
                            * n
                    };
                }
            }
        }
        for i in 0..sims.len() {
            if !(fractions[i] < 1.0) {
                continue;
            }
            let t = fractions[i];
            let mut sim = sims[i];
            sim.center = offset_pos(
                starts[i].center,
                t * sub_pos(sim.center, starts[i].center) + carries[i],
            );
            sim.transform.q = nlerp(starts[i].rotation, sim.transform.q, t);
            sim.transform.p = offset_pos(sim.center, -rotate_vector(sim.transform.q, sim.local_center));
            sim.center0 = starts[i].center;
            sim.rotation0 = starts[i].rotation;
            sim.flags |= BodyFlags::IS_FAST.0 | BodyFlags::HAD_TIME_OF_IMPACT.0;
            sims[i] = sim;
            self.body_move_events[i].transform = sim.transform;
            self.finalize_flags[i] = if (sim.flags & BodyFlags::IS_BULLET.0) != 0 {
                FinalizeFlag::BULLET
            } else {
                FinalizeFlag::CONTINUOUS
            };
        }
    }
}
