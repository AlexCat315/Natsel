//! Continuous collision.
//!
//! Translated from `Sources/Natsel/Simulation/Continuous.swift`.
//!
//! Swift keeps `Continuous.swift` beside a `Continuous/` directory. Rust cannot have both
//! `continuous.rs` and `continuous/mod.rs`, so the directory's children are declared here.

pub mod compound_impact;
pub mod dynamic_impact;
pub mod impact;

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::collision::internal_math::farthest_point_on_aabb;
use crate::collision::time_of_impact::time_of_impact;
use crate::math::functions::{
    aabb_contains, aabb_inflate, aabb_union, cross, distance, dot, inv_mul_quat, length, lerp,
    max_float, max_int, max_vec3, min_vec3, nlerp, offset_aabb, offset_pos, rotate_vector, sub_pos,
    transform_point,
};
use crate::math::types::{AABB, Pos, Transform, Vec3, WorldTransform};
use crate::simulation::continuous::compound_impact::compound_time_of_impact;
use crate::simulation::continuous::impact::triangle_surface_time_of_impact;
use crate::simulation::shape_runtime::{
    compute_fat_shape_aabb, compute_shape_aabb, compute_shape_min_extent, make_shape_proxy,
    should_shapes_collide, Shape, ShapeFlags, ShapeGeometry,
};
use crate::simulation::solver_integration::SensorHit;
use crate::simulation::triangle_surface::TriangleSurface;
use crate::simulation::world_runtime::{
    make_relative_sweep, BodyFlags, BodySim, BodyState, SetType, SolverSet, World,
};
use crate::types::body_def::BodyType;
use crate::types::events::ContactHitEvent;
use crate::types::queries::{Sweep, TOIInput, TOIOutput};
use crate::types::shape_def::ShapeType;
use crate::types::world_def::DEFAULT_MASK_BITS;

/// Mutable state shared by the swept queries of one fast body.
///
/// Swift's `final class ContinuousContext` is reused across steps to avoid allocating a class and
/// a sensor-hit array per fast body; here it is an owned field of `World`, moved out for the
/// duration of one pass and written back. The Swift `world: World!` back-reference has no Rust
/// counterpart: `World` owns the context, so the routines that need the world receive it directly.
#[derive(Clone, Debug, PartialEq)]
pub struct ContinuousContext {
    pub fast_body_id: i32,
    pub fast_shape_id: i32,
    pub base: Pos,
    pub sweep: Sweep,
    pub fraction: f32,
    pub impact: Option<ContinuousImpact>,

    pub sensor_hits: Vec<ContinuousSensorHit>,
    pub distance_iterations: i32,
    pub push_back_iterations: i32,
    pub root_iterations: i32,
}

impl Default for ContinuousContext {
    /// Swift's property initializers, not the all-zeroes derive: the sweep starts at fraction 1
    /// and the ids start at `B3.nullIndex`.
    #[inline]
    fn default() -> Self {
        Self {
            fast_body_id: b3::NULL_INDEX,
            fast_shape_id: b3::NULL_INDEX,
            base: Pos::default(),
            sweep: Sweep::default(),
            fraction: 1.0,
            impact: None,
            sensor_hits: Vec::new(),
            distance_iterations: 0,
            push_back_iterations: 0,
            root_iterations: 0,
        }
    }
}

impl ContinuousContext {
    /// Swift resets these fields at the top of every `solveContinuous` pass.
    #[inline(always)]
    fn begin_pass(&mut self, fast_body_id: i32, base: Pos, sweep: Sweep) {
        self.fast_body_id = fast_body_id;
        self.base = base;
        self.sweep = sweep;
        self.fraction = 1.0;
        self.impact = None;
        // Swift `removeAll(keepingCapacity: true)`; `Vec::clear` keeps the allocation.
        self.sensor_hits.clear();
        self.distance_iterations = 0;
        self.push_back_iterations = 0;
        self.root_iterations = 0;
    }
}

/// The earliest solid hit found by a swept query: `(surface:shape, mover:shape, output)`.
/// Swift stored this as the named tuple `(surface: Int32, mover: Int32, output: TOIOutput)?`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ContinuousImpact {
    pub surface: i32,
    pub mover: i32,
    pub output: TOIOutput,
}

/// A sensor hit recorded during a swept query, with the fraction needed to compare it against
/// the solid hit. Swift stored this as `(sensorId: Int32, visitorId: Int32, fraction: Float)`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ContinuousSensorHit {
    pub sensor_id: i32,
    pub visitor_id: i32,
    pub fraction: f32,
}

/// Offset an AABB by a position, rounding outward.
#[inline(always)]
pub fn aabb_offset(local_box: AABB, origin: Pos) -> AABB {
    offset_aabb(local_box, origin)
}

impl World {
    /// Is this shape moving fast enough to need continuous collision?
    pub(crate) fn is_shape_fast(
        &self,
        shape: &Shape,
        centroid1: Vec3,
        centroid2: Vec3,
        rotation_chord: f32,
        safety_factor: f32,
    ) -> bool {
        let radius: f32;
        match &shape.geometry {
            ShapeGeometry::Sphere(sphere) => {
                radius = sphere.radius;
            }
            ShapeGeometry::Capsule(capsule) => {
                radius = 0.5 * distance(capsule.center1, capsule.center2) + capsule.radius;
            }
            ShapeGeometry::Hull(hull) => {
                let farthest_point = farthest_point_on_aabb(hull.aabb(), shape.local_centroid);
                radius = distance(farthest_point, shape.local_centroid);
            }
            _ => {
                return true;
            }
        }

        let min_extent = compute_shape_min_extent(shape, shape.local_centroid);
        let max_motion = distance(centroid1, centroid2) + rotation_chord * radius;
        max_motion > safety_factor * min_extent
    }

    /// Time of impact between convex shapes or a mesh and a convex shape.
    pub(crate) fn shape_time_of_impact(
        &self,
        shape_a: &Shape,
        shape_b: &Shape,
        sweep_a: Sweep,
        sweep_b: Sweep,
        max_fraction: f32,
    ) -> TOIOutput {
        if let ShapeGeometry::Compound(compound) = &shape_a.geometry {
            return compound_time_of_impact(compound, shape_b, sweep_a, sweep_b, max_fraction);
        }

        if let Some(surface) = TriangleSurface::from_geometry(&shape_a.geometry) {
            return triangle_surface_time_of_impact(
                &surface,
                shape_b,
                sweep_a,
                sweep_b,
                max_fraction,
                shape_a.sensor_index != b3::NULL_INDEX,
            );
        }

        b3_assert(
            shape_b.type_ != ShapeType::CompoundShape
                && shape_b.type_ != ShapeType::MeshShape
                && shape_b.type_ != ShapeType::HeightShape,
        );

        let buffer_a = make_shape_proxy(shape_a);
        let buffer_b = make_shape_proxy(shape_b);
        // `buffer_a` and `buffer_b` own the point storage and outlive the TOI call below, which is
        // the invariant Swift's nested `withProxy` closures provided.
        let proxy_a = buffer_a.proxy();
        let proxy_b = buffer_b.proxy();

        let mut input = TOIInput::default();
        input.proxy_a = proxy_a;
        input.proxy_b = proxy_b;
        input.sweep_a = sweep_a;
        input.sweep_b = sweep_b;
        input.max_fraction = max_fraction;
        time_of_impact(&input)
    }

    /// Process one proxy during a swept query.
    fn continuous_query_callback(
        &self,
        shape_id: i32,
        context: &mut ContinuousContext,
    ) -> bool {
        let shapes = &self.shapes;
        self.continuous_query_callback_shapes(
            shape_id,
            context,
            &shapes[context.fast_shape_id as usize],
            &shapes[shape_id as usize],
        )
    }

    /// The callback on borrowed shapes; copying a `Shape` retains its geometry and materials.
    fn continuous_query_callback_shapes(
        &self,
        shape_id: i32,
        context: &mut ContinuousContext,
        fast_shape: &Shape,
        shape: &Shape,
    ) -> bool {
        let fast_body_id = context.fast_body_id;
        let fast_shape_id_value = fast_shape.id;

        // Skip same shape
        if shape_id == fast_shape_id_value {
            return true;
        }

        let body_id = shape.body_id;

        // Skip same body
        if body_id == fast_shape.body_id {
            return true;
        }

        // Skip sensors unless both shapes want sensor events
        let is_sensor = shape.sensor_index != b3::NULL_INDEX;
        if is_sensor
            && ((shape.flags & ShapeFlags::ENABLE_SENSOR_EVENTS.0 == 0)
                || (fast_shape.flags & ShapeFlags::ENABLE_SENSOR_EVENTS.0 == 0))
        {
            return true;
        }

        // Skip filtered shapes
        if !should_shapes_collide(&fast_shape.filter, &shape.filter) {
            return true;
        }

        // Skip bullets
        if (self.bodies[body_id as usize].flags & BodyFlags::IS_BULLET.0) != 0 {
            return true;
        }

        // Skip filtered bodies
        if !self.should_bodies_collide(&self.bodies[fast_body_id as usize], &self.bodies[body_id as usize])
        {
            return true;
        }

        // Time of impact versus shape. Supports all convex shape types.
        let set_index = self.bodies[body_id as usize].set_index as usize;
        let local_index = self.bodies[body_id as usize].local_index as usize;
        let sweep_a = make_relative_sweep(
            &self.solver_sets[set_index].body_sims[local_index],
            context.base,
        );
        let output = self.shape_time_of_impact(
            shape,
            fast_shape,
            sweep_a,
            context.sweep,
            context.fraction,
        );

        if is_sensor {
            // Only accept a sensor hit that is sooner than the current solid hit.
            if output.fraction <= context.fraction {
                context.sensor_hits.push(ContinuousSensorHit {
                    sensor_id: shape_id,
                    visitor_id: fast_shape_id_value,
                    fraction: output.fraction,
                });
            }
        } else if 0.0 < output.fraction && output.fraction < context.fraction {
            context.fraction = output.fraction;
            context.impact = Some(ContinuousImpact {
                surface: shape_id,
                mover: fast_shape_id_value,
                output,
            });
            context.distance_iterations =
                max_int(context.distance_iterations, output.distance_iterations);
            context.push_back_iterations =
                max_int(context.push_back_iterations, output.push_back_iterations);
            context.root_iterations = max_int(context.root_iterations, output.root_iterations);
        }

        // Continue query
        true
    }

    /// Continuous collision of a dynamic body against static bodies (and against dynamic bodies
    /// for bullets).
    pub(crate) fn solve_continuous_body(&mut self, body_sim_index: i32, dt: f32) {
        // Swift's `var awakeSet = solverSets[awakeSet]` shares storage until the mutable buffer
        // pointers force a copy of `bodySims`/`bodyStates`; `solverSets` keeps the pre-pass
        // contents readable by the swept-query callback until the write-back below. Cloning the
        // set reproduces exactly that, so the clone (not a swap) is required.
        let awake = SetType::AwakeSet.raw_value() as usize;
        let mut awake_set = self.solver_sets[awake].clone();
        self.solve_continuous_with_set(body_sim_index, dt, &mut awake_set);
        self.solver_sets[awake] = awake_set;
    }

    /// Continuous pass against an awake set the caller already owns. The caller keeps the set so
    /// the per-body loop does not copy it in and out of `solverSets` for every fast body.
    pub(crate) fn solve_continuous_with_set(
        &mut self,
        body_sim_index: i32,
        dt: f32,
        awake_set: &mut SolverSet,
    ) {
        self.solve_continuous(
            body_sim_index,
            dt,
            &mut awake_set.body_sims,
            &mut awake_set.body_states,
        )
    }

    /// The finalize pass already owns these buffers. Borrow them once for the whole pass so
    /// each fast body does not repeat Array uniqueness checks when its pose is written back.
    pub(crate) fn solve_continuous(
        &mut self,
        body_sim_index: i32,
        dt: f32,
        sims: &mut [BodySim],
        states: &mut [BodyState],
    ) {
        let mut fast_body_sim = sims[body_sim_index as usize];
        b3_validate((fast_body_sim.flags & BodyFlags::IS_FAST.0) != 0);

        // Re-center the sweep on the fast body so the TOI and the swept query stay in float precision
        let base = fast_body_sim.center0;

        let sweep = make_relative_sweep(&fast_body_sim, base);

        let mut xf1 = Transform::default();
        xf1.q = sweep.q1;
        xf1.p = sweep.c1 - rotate_vector(sweep.q1, sweep.local_center);

        let mut xf2 = Transform::default();
        xf2.q = sweep.q2;
        xf2.p = sweep.c2 - rotate_vector(sweep.q2, sweep.local_center);

        let static_tree = &self.broad_phase.trees[BodyType::StaticBody as usize];
        let kinematic_tree = &self.broad_phase.trees[BodyType::KinematicBody as usize];
        let dynamic_tree = &self.broad_phase.trees[BodyType::DynamicBody as usize];
        let fast_head_shape_id = self.bodies[fast_body_sim.body_id as usize].head_shape_id;

        // Swift reused the `ContinuousContext` class in place; moving the owned value out of
        // `self` for the pass and writing it back keeps the sensor-hit capacity alive across
        // steps, exactly like the reused class.
        let mut context = std::mem::take(&mut self.continuous_context);
        context.begin_pass(fast_body_sim.body_id, base, sweep);

        let is_bullet = (fast_body_sim.flags & BodyFlags::IS_BULLET.0) != 0;
        let safety_factor = self.bodies[fast_body_sim.body_id as usize].safety_factor;
        let rotation_chord = 2.0 * length(inv_mul_quat(sweep.q1, sweep.q2).v);

        let mut shape_id = fast_head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let fast_index = shape_id as usize;
            shape_id = self.shapes[fast_index].next_shape_id;

            context.fast_shape_id = fast_index as i32;
            let local_centroid = self.shapes[fast_index].local_centroid;
            let centroid1 = transform_point(xf1, local_centroid);
            let centroid2 = transform_point(xf2, local_centroid);

            let box1 = self.shapes[fast_index].aabb;
            // xf2 is relative to the base, so translate the box back to world space, rounding outward
            let box2 = aabb_offset(compute_shape_aabb(&self.shapes[fast_index], xf2), base);

            // Store this to avoid double computation in the case there is no impact event
            self.shapes[fast_index].aabb = box2;

            // No continuous collision for meshes
            let shape_type = self.shapes[fast_index].type_;
            if shape_type == ShapeType::MeshShape || shape_type == ShapeType::HeightShape {
                continue;
            }

            // No continuous collision for sensors
            if self.shapes[fast_index].sensor_index != b3::NULL_INDEX {
                continue;
            }

            if !is_bullet {
                let is_shape_fast = self.is_shape_fast(
                    &self.shapes[fast_index],
                    centroid1,
                    centroid2,
                    rotation_chord,
                    safety_factor,
                );
                if !is_shape_fast {
                    continue;
                }
            }

            let mut swept_box = aabb_union(box1, box2);
            if self.enable_dynamic_continuous && rotation_chord > 0.0 {
                let radius = length(fast_body_sim.max_extent);
                let arc_bounds = aabb_inflate(
                    AABB::new(min_vec3(sweep.c1, sweep.c2), max_vec3(sweep.c1, sweep.c2)),
                    radius,
                );
                swept_box = aabb_union(swept_box, aabb_offset(arc_bounds, base));
            }
            let _ = static_tree.query(swept_box, DEFAULT_MASK_BITS, false, |_proxy_id, user_data| {
                self.continuous_query_callback(user_data as i64 as i32, &mut context)
            });

            if is_bullet {
                let _ = kinematic_tree.query(
                    swept_box,
                    DEFAULT_MASK_BITS,
                    false,
                    |_proxy_id, user_data| {
                        self.continuous_query_callback(user_data as i64 as i32, &mut context)
                    },
                );
                let _ = dynamic_tree.query(
                    swept_box,
                    DEFAULT_MASK_BITS,
                    false,
                    |_proxy_id, user_data| {
                        self.continuous_query_callback(user_data as i64 as i32, &mut context)
                    },
                );
            }
        }

        let speculative_scalar = b3::speculative_distance();

        if context.fraction < 1.0 {
            // Handle time of impact event. The sweep is relative to the base, so re-add the base
            // to return the advanced pose to world space.
            let q = nlerp(sweep.q1, sweep.q2, context.fraction);
            let c = lerp(sweep.c1, sweep.c2, context.fraction);
            let origin = c - rotate_vector(q, sweep.local_center);

            // Advance body
            let transform = WorldTransform::new(offset_pos(base, origin), q);
            let center = offset_pos(base, c);
            fast_body_sim.transform = transform;
            fast_body_sim.center = center;
            fast_body_sim.rotation0 = q;
            fast_body_sim.center0 = center;
            fast_body_sim.flags |= BodyFlags::HAD_TIME_OF_IMPACT.0;

            // Timeloss means there is a lost gravity contribution. Other forces and torques are
            // ignored for now.
            let mut fast_body_state = states[body_sim_index as usize];
            let v = fast_body_state.linear_velocity;
            let time_loss = (1.0 - context.fraction) * dt;
            let mut dv = (-time_loss * fast_body_sim.gravity_scale) * self.gravity;
            dv.x = if (fast_body_state.flags & BodyFlags::LOCK_LINEAR_X.0) != 0 {
                0.0
            } else {
                dv.x
            };
            dv.y = if (fast_body_state.flags & BodyFlags::LOCK_LINEAR_Y.0) != 0 {
                0.0
            } else {
                dv.y
            };
            dv.z = if (fast_body_state.flags & BodyFlags::LOCK_LINEAR_Z.0) != 0 {
                0.0
            } else {
                dv.z
            };
            fast_body_state.linear_velocity = v + dv;
            states[body_sim_index as usize] = fast_body_state;

            if let Some(impact) = context.impact {
                let a = &self.shapes[impact.surface as usize];
                let b = &self.shapes[impact.mover as usize];
                let flags = a.flags | b.flags;
                if flags
                    & (ShapeFlags::ENABLE_CONTACT_EVENTS.0 | ShapeFlags::ENABLE_HIT_EVENTS.0)
                    != 0
                {
                    let point = offset_pos(base, impact.output.point);
                    let other = &self.bodies[a.body_id as usize];
                    let other_sim = if other.set_index == SetType::AwakeSet.raw_value() {
                        sims[other.local_index as usize]
                    } else {
                        self.solver_sets[other.set_index as usize].body_sims
                            [other.local_index as usize]
                    };
                    let other_state = if other.set_index == SetType::AwakeSet.raw_value() {
                        states[other.local_index as usize]
                    } else {
                        BodyState::default()
                    };
                    let va = other_state.linear_velocity
                        + cross(other_state.angular_velocity, sub_pos(point, other_sim.center));
                    let vb = fast_body_state.linear_velocity
                        + cross(fast_body_state.angular_velocity, sub_pos(point, center));
                    let mut event = ContactHitEvent::default();
                    event.shape_id_a = self.make_shape_id(impact.surface);
                    event.shape_id_b = self.make_shape_id(impact.mover);
                    event.point = point;
                    event.normal = impact.output.normal;
                    event.approach_speed = max_float(0.0, -dot(vb - va, event.normal));
                    event.user_material_id_a = a.material(0).user_material_id;
                    event.user_material_id_b = b.material(0).user_material_id;
                    self.continuous_impact_events.push(event);
                    if event.approach_speed > self.hit_event_threshold
                        && flags & ShapeFlags::ENABLE_HIT_EVENTS.0 != 0
                    {
                        self.contact_hit_events.push(event);
                    }
                }
            }

            // The move event was written before CCD, so correct it with the impact pose
            if (body_sim_index as usize) < self.body_move_events.len() {
                // Swift's value-semantics array read copies the element; `clone` matches that,
                // and the corrected copy is written back below.
                let mut event = self.body_move_events[body_sim_index as usize].clone();
                event.transform = fast_body_sim.transform;
                self.body_move_events[body_sim_index as usize] = event;
            }

            // Prepare AABBs for broad-phase.
            // Even though a body is fast, it may not move much. So the AABB may not need enlargement.
            shape_id = fast_head_shape_id;
            while shape_id != b3::NULL_INDEX {
                let index = shape_id as usize;

                // Must recompute aabb at the interpolated transform
                let aabb =
                    compute_fat_shape_aabb(&self.shapes[index], transform, speculative_scalar);
                self.shapes[index].aabb = aabb;

                let fat_aabb = self.fat_aabbs[shape_id as usize];
                if !aabb_contains(fat_aabb, aabb) {
                    let inflated = aabb_inflate(aabb, self.shapes[index].aabb_margin);
                    self.fat_aabbs[shape_id as usize] = inflated;

                    // Regular bodies mark the hierarchy as moved using atomic operations.
                    // Bullets are handled separately at a later stage.
                    if !is_bullet {
                        let proxy_key = self.shapes[index].proxy_key;
                        self.broad_phase.mark_proxy_moved(proxy_key, inflated);
                    } else {
                        fast_body_sim.flags |= BodyFlags::ENLARGE_BULLET_BOUNDS.0;
                    }
                }

                shape_id = self.shapes[index].next_shape_id;
            }
        } else {
            // No time of impact event

            // Advance body
            fast_body_sim.rotation0 = fast_body_sim.transform.q;
            fast_body_sim.center0 = fast_body_sim.center;

            // Prepare AABBs for broad-phase
            shape_id = fast_head_shape_id;
            while shape_id != b3::NULL_INDEX {
                let index = shape_id as usize;

                // shape.aabb is still valid from above
                let fat_aabb = self.fat_aabbs[index];
                if !aabb_contains(fat_aabb, self.shapes[index].aabb) {
                    let inflated = aabb_inflate(self.shapes[index].aabb, self.shapes[index].aabb_margin);
                    self.fat_aabbs[shape_id as usize] = inflated;

                    if !is_bullet {
                        let proxy_key = self.shapes[index].proxy_key;
                        self.broad_phase.mark_proxy_moved(proxy_key, inflated);
                    } else {
                        fast_body_sim.flags |= BodyFlags::ENLARGE_BULLET_BOUNDS.0;
                    }
                }

                shape_id = self.shapes[index].next_shape_id;
            }
        }

        // Push sensor hits on the task context for serial processing.
        for hit in &context.sensor_hits {
            // Skip any sensor hits that occurred after a solid hit
            if hit.fraction < context.fraction {
                self.task_contexts[0].sensor_hits.push(SensorHit {
                    sensor_id: hit.sensor_id,
                    visitor_id: hit.visitor_id,
                });
            }
        }

        self.task_contexts[0].distance_iterations =
            max_int(self.task_contexts[0].distance_iterations, context.distance_iterations);
        self.task_contexts[0].push_back_iterations = max_int(
            self.task_contexts[0].push_back_iterations,
            context.push_back_iterations,
        );
        self.task_contexts[0].root_iterations =
            max_int(self.task_contexts[0].root_iterations, context.root_iterations);

        sims[body_sim_index as usize] = fast_body_sim;

        self.continuous_context = context;
    }
}
