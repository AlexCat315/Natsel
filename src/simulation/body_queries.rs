//! Body-level queries and the remaining body/shape accessors.
//!
//! Translated from `Sources/Natsel/Simulation/BodyQueries.swift`.
//!
//! These run against a caller supplied body transform so they can be used for speculative
//! movement without stepping the simulation.

use std::ffi::c_void;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::compound::compound::Compound;
use crate::collision::distance::shape_distance;
use crate::collision::height_field::quantization::HeightField;
use crate::collision::hull::Hull;
use crate::collision::time_of_impact::time_of_impact;
use crate::math::functions::*;
use crate::math::types::{AABB, Pos, Quat, Transform, Vec3, WorldTransform};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::shape_planes::collide_mover;
use crate::simulation::shape_runtime::{
    ShapeFlags, ShapeGeometry, compute_shape_aabb, destroy_shape_proxy,
    get_shape_centroid, get_shape_projected_area, make_shape_proxy, overlap_shape, ray_cast_shape,
    shape_cast_shape, should_query_collide,
};
use crate::simulation::world_runtime::{BodyFlags, SetType, World};
use crate::types::body_def::{BodyType, MotionLocks};
use crate::types::events::ContactData;
use crate::types::geometry::{Capsule, Mesh, Sphere};
use crate::types::ids::{BodyID, ContactID, JointID, ShapeID};
use crate::types::queries::{
    BodyCastResult, BodyPlaneResult, BodyTOIResult, DistanceInput, DistanceOutput, RayCastInput,
    ShapeCastInput, ShapeProxy, SimplexCache, TOIInput, TOIOutput, WorldCastOutput,
};
use crate::types::shape_def::{Filter, QueryFilter, ShapeType, SurfaceMaterial};
use crate::types::world_def::ExplosionDef;

impl World {
    /// Cast a ray against the shapes of a single body.
    pub fn body_cast_ray(
        &self,
        body_id: BodyID,
        origin: Pos,
        translation: Vec3,
        filter: &QueryFilter,
        max_fraction: f32,
        body_transform: WorldTransform,
    ) -> BodyCastResult {
        let mut result = BodyCastResult::default();

        let body = self.body(body_id);
        if body.id == b3::NULL_INDEX {
            return result;
        }

        // The consistent framing is to center on the ray origin.
        let mut shape_input = RayCastInput::default();
        shape_input.origin = Vec3::ZERO;
        shape_input.translation = translation;
        shape_input.max_fraction = max_fraction;

        let transform = to_relative_transform(body_transform, origin);

        let mut shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = self.shapes[shape_id as usize].clone();
            shape_id = shape.next_shape_id;

            if !should_query_collide(&shape.filter, filter) {
                continue;
            }

            let shape_output = ray_cast_shape(&shape, transform, &shape_input);

            if shape_output.hit == false {
                continue;
            }

            if shape_output.fraction > shape_input.max_fraction {
                continue;
            }

            // Careful with id, shapeId is the next shape.
            let id = ShapeID::new(shape.id + 1, body_id.world0, shape.generation);
            let material_index =
                clamp_int(shape_output.material_index, 0, shape.material_count() - 1);
            let user_material_id = shape.material(material_index).user_material_id;

            result.shape_id = id;
            result.point = offset_pos(origin, shape_output.point);
            result.normal = shape_output.normal;
            result.fraction = shape_output.fraction;
            result.triangle_index = shape_output.triangle_index;
            result.user_material_id = user_material_id;
            result.iterations = shape_output.iterations;
            result.hit = true;

            shape_input.max_fraction = shape_output.fraction;
        }

        result
    }

    /// Cast a shape against the shapes of a single body.
    pub fn body_cast_shape(
        &self,
        body_id: BodyID,
        origin: Pos,
        proxy: ShapeProxy,
        translation: Vec3,
        filter: &QueryFilter,
        max_fraction: f32,
        can_encroach: bool,
        body_transform: WorldTransform,
    ) -> BodyCastResult {
        let mut result = BodyCastResult::default();

        let body = self.body(body_id);
        if body.id == b3::NULL_INDEX {
            return result;
        }

        let transform = to_relative_transform(body_transform, origin);

        let mut shape_input = ShapeCastInput::default();
        shape_input.proxy = proxy;
        shape_input.translation = translation;
        shape_input.max_fraction = max_fraction;
        shape_input.can_encroach = can_encroach;

        let mut shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = self.shapes[shape_id as usize].clone();
            shape_id = shape.next_shape_id;

            if !should_query_collide(&shape.filter, filter) {
                continue;
            }

            let shape_output = shape_cast_shape(&shape, transform, &shape_input);

            if shape_output.hit == false {
                continue;
            }

            if shape_output.fraction > shape_input.max_fraction {
                continue;
            }

            // Careful with id, shapeId is the next shape.
            let id = ShapeID::new(shape.id + 1, body_id.world0, shape.generation);
            let material_index =
                clamp_int(shape_output.material_index, 0, shape.material_count() - 1);
            let user_material_id = shape.material(material_index).user_material_id;

            result.shape_id = id;
            result.point = offset_pos(origin, shape_output.point);
            result.normal = shape_output.normal;
            result.fraction = shape_output.fraction;
            result.triangle_index = shape_output.triangle_index;
            result.user_material_id = user_material_id;
            result.iterations = shape_output.iterations;
            result.hit = true;

            shape_input.max_fraction = shape_output.fraction;
        }

        result
    }

    /// Test a shape against the shapes of a single body.
    pub fn body_overlap_shape(
        &self,
        body_id: BodyID,
        origin: Pos,
        proxy: ShapeProxy,
        filter: &QueryFilter,
        body_transform: WorldTransform,
    ) -> bool {
        let body = self.body(body_id);
        if body.id == b3::NULL_INDEX {
            return false;
        }

        let transform = to_relative_transform(body_transform, origin);

        let mut shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = self.shapes[shape_id as usize].clone();
            shape_id = shape.next_shape_id;

            if !should_query_collide(&shape.filter, filter) {
                continue;
            }

            if overlap_shape(&shape, transform, &proxy) {
                return true;
            }
        }

        false
    }

    /// Collide a capsule mover against the convex shapes of a single body.
    pub fn body_collide_mover(
        &self,
        body_id: BodyID,
        origin: Pos,
        mover: &Capsule,
        filter: &QueryFilter,
        body_transform: WorldTransform,
        capacity: usize,
    ) -> Vec<BodyPlaneResult> {
        if capacity == 0 {
            return Vec::new();
        }

        let body = self.body(body_id);
        if body.id == b3::NULL_INDEX {
            return Vec::new();
        }

        let transform = to_relative_transform(body_transform, origin);

        let mut results: Vec<BodyPlaneResult> = Vec::new();
        let mut shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = self.shapes[shape_id as usize].clone();
            shape_id = shape.next_shape_id;

            if !should_query_collide(&shape.filter, filter) {
                continue;
            }

            match shape.type_ {
                ShapeType::SphereShape | ShapeType::CapsuleShape | ShapeType::HullShape => {}
                _ => continue,
            }

            let planes = collide_mover(&shape, transform, mover, 1);
            if let Some(plane) = planes.first() {
                let id = ShapeID::new(shape.id + 1, body_id.world0, shape.generation);
                let mut entry = BodyPlaneResult::default();
                entry.shape_id = id;
                entry.result = *plane;
                results.push(entry);

                if results.len() == capacity {
                    return results;
                }
            }
        }

        results
    }

    /// Compute the time of impact of a capsule mover against a single body.
    pub fn body_time_of_impact_mover(
        &self,
        body_id: BodyID,
        origin: Pos,
        mover: &Capsule,
        mover_translation: Vec3,
        filter: &QueryFilter,
        body_transform_1: WorldTransform,
        body_transform_2: WorldTransform,
    ) -> BodyTOIResult {
        let mut result = BodyTOIResult::default();
        result.fraction = 1.0;

        let body = self.body(body_id);
        if body.id == b3::NULL_INDEX {
            return result;
        }

        let xf1 = to_relative_transform(body_transform_1, origin);
        let xf2 = to_relative_transform(body_transform_2, origin);

        let sim = self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];
        let local_center = sim.local_center;

        let mut input = TOIInput::default();
        let capsule_points = [mover.center1, mover.center2];
        input.sweep_a.c1 = transform_point(xf1, local_center);
        input.sweep_a.c2 = transform_point(xf2, local_center);
        input.sweep_a.q1 = body_transform_1.q;
        input.sweep_a.q2 = body_transform_2.q;
        input.sweep_a.local_center = local_center;

        input.sweep_b.c1 = Vec3::ZERO;
        input.sweep_b.c2 = mover_translation;
        input.sweep_b.q1 = Quat::IDENTITY;
        input.sweep_b.q2 = Quat::IDENTITY;
        input.sweep_b.local_center = Vec3::ZERO;

        input.max_fraction = 1.0;

        let mut shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = self.shapes[shape_id as usize].clone();
            shape_id = shape.next_shape_id;

            if !should_query_collide(&shape.filter, filter) {
                continue;
            }

            match shape.type_ {
                ShapeType::SphereShape | ShapeType::CapsuleShape | ShapeType::HullShape => {}
                _ => continue,
            }

            let buffer = make_shape_proxy(&shape);
            let output: TOIOutput = {
                // SAFETY: `capsule_points` is a two element array owned by this function and
                // `buffer` owns the proxy point storage, so both pointers stay valid for the whole
                // TOI call. This is the invariant Swift's
                // `withUnsafeBufferPointer { buffer.withProxy { ... } }` pair gave, and it is kept
                // rather than redesigned so the narrow phase sees the same address.
                input.proxy_a = buffer.proxy();
                input.proxy_b = ShapeProxy::new(Some(capsule_points.as_ptr()), 2, mover.radius);
                time_of_impact(&input)
            };

            // Mimic the continuous query: ignore shapes that initially overlap.
            if 0.0 < output.fraction && output.fraction < result.fraction {
                input.max_fraction = output.fraction;

                result.point = offset_pos(origin, output.point);
                result.normal = output.normal;
                result.fraction = output.fraction;
                result.shape_id =
                    ShapeID::new(shape.id + 1, self.world_id, shape.generation);
            }
        }

        result
    }
}

impl World {
    /// Contact manifold digests for divergence debugging.
    pub fn contact_dump(&self) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for i in 0..self.contacts.len() {
            if self.contacts[i].contact_id != i as i32 {
                continue;
            }
            let c = self.contacts[i];
            if c.payload.is_none() || c.manifold_count <= 0 {
                continue;
            }
            let view = c.manifolds();
            for m in 0..view.count() {
                let manifold = view.get(m);
                let mut parts: Vec<f32> = vec![
                    manifold.normal.x,
                    manifold.normal.y,
                    manifold.normal.z,
                    manifold.twist_impulse,
                    manifold.friction_impulse.x,
                    manifold.friction_impulse.y,
                    manifold.friction_impulse.z,
                    manifold.rolling_impulse.x,
                    manifold.rolling_impulse.y,
                    manifold.rolling_impulse.z,
                ];
                for pi in 0..manifold.point_count as usize {
                    let p = manifold.points[pi];
                    parts.push(p.anchor_a.x);
                    parts.push(p.anchor_a.y);
                    parts.push(p.anchor_a.z);
                    parts.push(p.anchor_b.x);
                    parts.push(p.anchor_b.y);
                    parts.push(p.anchor_b.z);
                    parts.push(p.separation);
                    parts.push(p.normal_impulse);
                    parts.push(p.total_normal_impulse);
                }
                let hex = parts
                    .iter()
                    .map(|value| format!("{:x}", value.to_bits()))
                    .collect::<Vec<_>>()
                    .join(" ");
                out.push(format!("C {} {} {}", c.shape_id_a, c.shape_id_b, hex));
            }
        }
        out
    }

    /// Per-body position, rotation, and awake velocity bits for divergence debugging.
    pub fn body_state_dump(&self) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for i in 0..self.bodies.len() {
            if self.bodies[i].id != i as i32 {
                continue;
            }
            let body = &self.bodies[i];
            let sim = self.solver_sets[body.set_index as usize].body_sims
                [body.local_index as usize];
            let position_bits = [sim.transform.p.x, sim.transform.p.y, sim.transform.p.z]
                .iter()
                .map(|value| format!("{:x}", value.to_bits()))
                .collect::<Vec<_>>()
                .join(" ");
            let mut parts: Vec<f32> = vec![
                sim.transform.q.v.x,
                sim.transform.q.v.y,
                sim.transform.q.v.z,
                sim.transform.q.s,
            ];
            if body.set_index == SetType::AwakeSet.raw_value() {
                let s = self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
                    [body.local_index as usize];
                parts.push(s.linear_velocity.x);
                parts.push(s.linear_velocity.y);
                parts.push(s.linear_velocity.z);
                parts.push(s.angular_velocity.x);
                parts.push(s.angular_velocity.y);
                parts.push(s.angular_velocity.z);
            }
            let hex = parts
                .iter()
                .map(|value| format!("{:x}", value.to_bits()))
                .collect::<Vec<_>>()
                .join(" ");
            out.push(format!("{} {} {}", i, position_bits, hex));
        }
        out
    }

    /// FNV-1a hash over every live body's transform and, when awake, its velocities.
    pub fn world_state_hash(&self) -> u64 {
        let hash_init: u64 = 14_695_981_039_346_656_037;
        let prime: u64 = 1_099_511_628_211;

        let mut hash = hash_init;

        #[inline(always)]
        fn mix_bits(value: f32, hash: &mut u64, prime: u64) {
            *hash = (*hash ^ value.to_bits() as u64).wrapping_mul(prime);
        }

        #[inline(always)]
        fn mix_position(p: Pos, hash: &mut u64, prime: u64) {
            *hash = (*hash ^ p.x.to_bits() as u64).wrapping_mul(prime);
            *hash = (*hash ^ p.y.to_bits() as u64).wrapping_mul(prime);
            *hash = (*hash ^ p.z.to_bits() as u64).wrapping_mul(prime);
        }

        for i in 0..self.bodies.len() {
            let body = &self.bodies[i];

            // Free or never-used slot
            if body.id != i as i32 {
                continue;
            }

            let sim =
                self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];

            mix_position(sim.transform.p, &mut hash, prime);
            mix_bits(sim.transform.q.v.x, &mut hash, prime);
            mix_bits(sim.transform.q.v.y, &mut hash, prime);
            mix_bits(sim.transform.q.v.z, &mut hash, prime);
            mix_bits(sim.transform.q.s, &mut hash, prime);

            // Only awake bodies have a state.
            if body.set_index == SetType::AwakeSet.raw_value() {
                let state = self.solver_sets[SetType::AwakeSet.raw_value() as usize]
                    .body_states[body.local_index as usize];
                mix_bits(state.linear_velocity.x, &mut hash, prime);
                mix_bits(state.linear_velocity.y, &mut hash, prime);
                mix_bits(state.linear_velocity.z, &mut hash, prime);
                mix_bits(state.angular_velocity.x, &mut hash, prime);
                mix_bits(state.angular_velocity.y, &mut hash, prime);
                mix_bits(state.angular_velocity.z, &mut hash, prime);
            }
        }

        hash
    }
}

// MARK: - Explosion

impl World {
    /// Apply a radial impulse field to every dynamic shape within range. The impulse scales with
    /// each shape's projected area and its explosion scale, and falls off between `radius` and
    /// `radius + falloff`.
    pub fn explode(&mut self, def: &ExplosionDef) {
        let radius = def.radius;
        let falloff = def.falloff;
        let impulse_per_area = def.impulse_per_area;
        let position = def.position;

        b3_assert(is_valid_position(position));
        b3_assert(radius >= 0.0);
        b3_assert(falloff >= 0.0);

        // The broad-phase tree is float, so translate a local query box out to world with outward rounding
        let extent = radius + falloff;
        let local_box = AABB::new(
            Vec3::new(-extent, -extent, -extent),
            Vec3::new(extent, extent, extent),
        );
        let aabb = offset_aabb(local_box, position);

        // Swift called `applyExplosion` from inside the tree traversal because both `self` and the
        // tree were reference-aliasable objects. Rust cannot express that aliasing, so the
        // traversal first collects the shape indices in exactly the order the query visits them and
        // only then applies the impulses. `apply_explosion` and `wake_body` never touch the
        // broad-phase trees, so the visited set and its order are unchanged.
        let mut hit_shape_ids: Vec<i32> = Vec::new();
        {
            let tree = &self.broad_phase.trees[BodyType::DynamicBody as usize];
            tree.query(aabb, def.mask_bits, false, |_proxy_id, user_data| {
                let shape_id = user_data as i64 as i32;
                hit_shape_ids.push(shape_id);
                true
            });
        }

        for shape_id in hit_shape_ids {
            self.apply_explosion(shape_id, position, radius, falloff, impulse_per_area);
        }
    }

    fn apply_explosion(
        &mut self,
        shape_id: i32,
        position: Pos,
        radius: f32,
        falloff: f32,
        impulse_per_area: f32,
    ) {
        let mut shape = self.shapes[shape_id as usize].clone();
        if shape.explosion_scale == 0.0 {
            return;
        }

        let body_id = shape.body_id;
        b3_assert(self.bodies[body_id as usize].type_ == BodyType::DynamicBody);

        let xf = self.body_transform_quick(body_id);

        // Re-center the explosion into the shape local frame so distance and direction stay precise
        // far from the origin. Everything below runs in that near-origin frame.
        let local_position = inv_transform_world_point(xf, position);

        let buffer = make_shape_proxy(&shape);
        let point_storage = [local_position];
        let mut output = DistanceOutput::default();
        let mut cache = SimplexCache::default();

        {
            // SAFETY: `point_storage` is a one element array owned by this scope and `buffer` owns
            // the proxy storage, so both pointers stay valid for the whole distance call. This is
            // what Swift's `withUnsafePointer(to:)` + `withProxy` pair guaranteed.
            let mut input = DistanceInput::default();
            input.proxy_a = buffer.proxy();
            input.proxy_b = ShapeProxy::new(Some(point_storage.as_ptr()), 1, 0.0);
            input.transform = Transform::IDENTITY;
            input.use_radii = true;
            output = shape_distance(&input, &mut cache, None);
        }

        if output.distance > radius + falloff {
            return;
        }

        self.wake_body(body_id);

        let mut body = self.bodies[body_id as usize].clone();
        if body.set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        // Witness point is already in the body local query frame
        let mut closest_point = output.point_a;
        if output.distance == 0.0 {
            closest_point = get_shape_centroid(&shape);
        }

        let mut direction = closest_point - local_position;
        if length_squared(direction) > 100.0 * f32::EPSILON * f32::EPSILON {
            direction = normalize(direction);
        } else {
            direction = Vec3::new(1.0, 0.0, 0.0);
        }

        let area = get_shape_projected_area(&shape, direction);
        let mut scale: f32 = 1.0;
        if output.distance > radius && falloff > 0.0 {
            scale = clamp_float((radius + falloff - output.distance) / falloff, 0.0, 1.0);
        }

        let magnitude = impulse_per_area * area * scale * shape.explosion_scale;
        let impulse = magnitude * rotate_vector(xf.q, direction);

        let local_index = body.local_index as usize;
        let awake_set_index = SetType::AwakeSet.raw_value() as usize;
        let mut state = self.solver_sets[awake_set_index].body_states[local_index];
        let sim = self.solver_sets[awake_set_index].body_sims[local_index];
        state.linear_velocity = mul_add(state.linear_velocity, sim.inv_mass, impulse);

        // Lever arm from the center of mass to the closest point, rotated to world
        let r = rotate_vector(xf.q, closest_point - sim.local_center);
        state.angular_velocity =
            state.angular_velocity + mul_mv(sim.inv_inertia_world, cross(r, impulse));

        self.solver_sets[awake_set_index].body_states[local_index] = state;

        // Keep the local shape copy in sync if the wake path touched it.
        shape = self.shapes[shape_id as usize].clone();
        self.shapes[shape_id as usize] = shape;
        body = self.bodies[body_id as usize].clone();
        self.bodies[body_id as usize] = body;
    }
}

// MARK: - Body flags

impl World {
    /// Enable or disable contact recycling for a body. When either body of a pair disables it the
    /// pair is rebuilt each step instead of reusing cached separations.
    pub fn enable_body_contact_recycling(&mut self, body_id: BodyID, enable: bool) {
        let index = self.body_index(body_id);
        let mut body = self.bodies[index as usize].clone();

        let new_flag = if enable { BodyFlags::ENABLE_CONTACT_RECYCLING.0 } else { 0 };
        if (body.flags & BodyFlags::ENABLE_CONTACT_RECYCLING.0) == new_flag {
            return;
        }

        body.flags &= !BodyFlags::ENABLE_CONTACT_RECYCLING.0;
        body.flags |= new_flag;
        self.bodies[index as usize] = body;

        self.sync_body_flags(index);
    }

    /// Is contact recycling enabled on this body?
    pub fn is_body_contact_recycling_enabled(&self, body_id: BodyID) -> bool {
        let body = self.body(body_id);
        (body.flags & BodyFlags::ENABLE_CONTACT_RECYCLING.0) != 0
    }
}

// MARK: - Bullet and motion locks

impl World {
    /// Enable or disable continuous collision for this body. The flag is written to the body as
    /// well as the sim so a later `syncBodyFlags` cannot revert it.
    pub fn set_body_bullet(&mut self, body_id: BodyID, bullet: bool) {
        let index = self.body_index(body_id);
        let mut body = self.bodies[index as usize].clone();

        let new_flag = if bullet { BodyFlags::IS_BULLET.0 } else { 0 };
        if (body.flags & BodyFlags::IS_BULLET.0) == new_flag {
            return;
        }

        body.flags &= !BodyFlags::IS_BULLET.0;
        body.flags |= new_flag;
        self.bodies[index as usize] = body.clone();

        // Write the sim directly: the bullet flag is not a transient flag and must survive
        // the sync path unchanged.
        let mut sim = self.body_sim(&body);
        sim.flags &= !BodyFlags::IS_BULLET.0;
        sim.flags |= new_flag;
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize] = sim;
    }

    /// Is continuous collision enabled for this body?
    pub fn is_body_bullet(&self, body_id: BodyID) -> bool {
        (self.body(body_id).flags & BodyFlags::IS_BULLET.0) != 0
    }

    /// Replace the motion locks on a body. Locks that are reverting from locked to free wake the
    /// body so the newly allowed motion starts immediately.
    pub fn set_body_motion_locks(&mut self, body_id: BodyID, locks: &MotionLocks) {
        let index = self.body_index(body_id);
        let mut body = self.bodies[index as usize].clone();

        let lock_mask = BodyFlags::LOCK_LINEAR_X.0
            | BodyFlags::LOCK_LINEAR_Y.0
            | BodyFlags::LOCK_LINEAR_Z.0
            | BodyFlags::LOCK_ANGULAR_X.0
            | BodyFlags::LOCK_ANGULAR_Y.0
            | BodyFlags::LOCK_ANGULAR_Z.0;

        let mut new_flags = body.flags & !lock_mask;
        new_flags |= if locks.linear_x { BodyFlags::LOCK_LINEAR_X.0 } else { 0 };
        new_flags |= if locks.linear_y { BodyFlags::LOCK_LINEAR_Y.0 } else { 0 };
        new_flags |= if locks.linear_z { BodyFlags::LOCK_LINEAR_Z.0 } else { 0 };
        new_flags |= if locks.angular_x { BodyFlags::LOCK_ANGULAR_X.0 } else { 0 };
        new_flags |= if locks.angular_y { BodyFlags::LOCK_ANGULAR_Y.0 } else { 0 };
        new_flags |= if locks.angular_z { BodyFlags::LOCK_ANGULAR_Z.0 } else { 0 };

        if new_flags == body.flags {
            return;
        }

        // Unlocking a direction is a state change the user can observe, so wake the body.
        let unlocked = (body.flags & !new_flags) & lock_mask;
        body.flags = new_flags;
        self.bodies[index as usize] = body;

        self.sync_body_flags(index);

        if unlocked != 0 {
            self.wake_body(index);
        }
    }

    /// The current motion locks for a body.
    pub fn body_motion_locks(&self, body_id: BodyID) -> MotionLocks {
        let flags = self.body(body_id).flags;
        let mut locks = MotionLocks::default();
        locks.linear_x = (flags & BodyFlags::LOCK_LINEAR_X.0) != 0;
        locks.linear_y = (flags & BodyFlags::LOCK_LINEAR_Y.0) != 0;
        locks.linear_z = (flags & BodyFlags::LOCK_LINEAR_Z.0) != 0;
        locks.angular_x = (flags & BodyFlags::LOCK_ANGULAR_X.0) != 0;
        locks.angular_y = (flags & BodyFlags::LOCK_ANGULAR_Y.0) != 0;
        locks.angular_z = (flags & BodyFlags::LOCK_ANGULAR_Z.0) != 0;
        locks
    }
}

// MARK: - Enable and disable

impl World {
    /// Disable a body: destroy its contacts, park its joints and sim in the disabled set, and drop
    /// its broad-phase proxies. A disabled body is skipped by the simulation.
    pub fn disable_body(&mut self, body_id: BodyID) {
        let index = self.body_index(body_id);
        let mut body = self.bodies[index as usize].clone();
        if body.set_index == SetType::DisabledSet.raw_value() {
            return;
        }

        // Destroy contacts and wake bodies touching this body. This avoids floating bodies.
        // This is necessary even for static bodies.
        self.destroy_body_contacts(index, true);

        body = self.bodies[index as usize].clone();
        let current_set_index = body.set_index;

        let disabled_set_index = SetType::DisabledSet.raw_value();

        // Unlink joints and transfer them to the disabled set
        let mut joint_key = body.head_joint_key;
        while joint_key != b3::NULL_INDEX {
            let joint_id = joint_key >> 1;
            let edge_index = (joint_key & 1) as usize;

            let joint = self.joints[joint_id as usize].clone();
            joint_key = joint.edges[edge_index].next_key;

            // The joint may already be disabled by the other body.
            if joint.set_index == disabled_set_index {
                continue;
            }

            b3_assert(
                joint.set_index == current_set_index
                    || current_set_index == SetType::StaticSet.raw_value()
                    || joint.set_index == SetType::StaticSet.raw_value(),
            );

            // Remove the joint from its island
            self.unlink_joint(joint_id);

            let refreshed = self.joints[joint_id as usize].clone();
            self.transfer_joint(disabled_set_index, refreshed.set_index, joint_id);
        }

        // Remove shapes from the broad-phase
        body = self.bodies[index as usize].clone();
        let mut shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let mut shape = self.shapes[shape_id as usize].clone();
            shape_id = shape.next_shape_id;
            let write_index = shape.id;
            destroy_shape_proxy(&mut shape, &mut self.broad_phase);
            self.shapes[write_index as usize] = shape;
        }

        // Disabled bodies are not in an island. If the island becomes empty it is destroyed.
        self.remove_body_from_island(index);

        // The contact teardown above may have woken this body or its neighbors, moving the sim.
        // Re-read the set before transferring or the source index is stale.
        let final_set_index = self.bodies[index as usize].set_index;
        self.transfer_body(disabled_set_index, final_set_index, index);
    }

    /// Re-enable a previously disabled body.
    pub fn enable_body(&mut self, body_id: BodyID) {
        let index = self.body_index(body_id);
        let mut body = self.bodies[index as usize].clone();
        if body.set_index != SetType::DisabledSet.raw_value() {
            return;
        }

        let disabled_set_index = SetType::DisabledSet.raw_value();
        let set_id = if body.type_ == BodyType::StaticBody {
            SetType::StaticSet.raw_value()
        } else {
            SetType::AwakeSet.raw_value()
        };

        self.transfer_body(set_id, disabled_set_index, index);

        // The sim moved sets, so re-read the body before indexing the new set.
        body = self.bodies[index as usize].clone();
        let transform = self.solver_sets[body.set_index as usize].body_sims
            [body.local_index as usize]
            .transform;

        // Add shapes to the broad-phase
        let mut shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let mut shape = self.shapes[shape_id as usize].clone();
            let next_shape_id = shape.next_shape_id;
            let write_index = shape.id;
            self.create_shape_proxy(&mut shape, body.type_, transform, true);
            self.shapes[write_index as usize] = shape;
            shape_id = next_shape_id;
        }

        if set_id != SetType::StaticSet.raw_value() {
            self.create_island_for_body(set_id, index);
        }

        // Transfer joints. If the other body is still disabled, leave the joint alone.
        body = self.bodies[index as usize].clone();
        let mut joint_key = body.head_joint_key;
        while joint_key != b3::NULL_INDEX {
            let joint_id = joint_key >> 1;
            let edge_index = (joint_key & 1) as usize;

            let joint = self.joints[joint_id as usize].clone();
            b3_assert(joint.set_index == disabled_set_index);
            b3_assert(joint.island_id == b3::NULL_INDEX);

            joint_key = joint.edges[edge_index].next_key;

            let body_a = self.bodies[joint.edges[0].body_id as usize].clone();
            let body_b = self.bodies[joint.edges[1].body_id as usize].clone();

            if body_a.set_index == disabled_set_index || body_b.set_index == disabled_set_index {
                // One body is still disabled.
                continue;
            }

            let joint_set_id: i32;
            if body_a.type_ != BodyType::DynamicBody && body_b.type_ != BodyType::DynamicBody {
                joint_set_id = SetType::StaticSet.raw_value();
            } else if body_a.set_index == SetType::StaticSet.raw_value() {
                joint_set_id = body_b.set_index;
            } else {
                joint_set_id = body_a.set_index;
            }

            self.transfer_joint(joint_set_id, disabled_set_index, joint_id);

            // Now that the joint is in the correct set, link it into the island.
            if joint_set_id != SetType::StaticSet.raw_value() {
                self.link_joint(joint_id);
            }
        }
    }

    /// True while the body is enabled (not parked in the disabled set).
    pub fn is_body_enabled(&self, body_id: BodyID) -> bool {
        self.body(body_id).set_index != SetType::DisabledSet.raw_value()
    }
}

// MARK: - Body type

impl World {
    /// Change a body's type, moving it between solver sets and rebuilding its contacts, joints,
    /// and broad-phase proxies. Not supported for bodies with compound or height-field shapes.
    /// The body type: static, kinematic, or dynamic.
    pub fn body_type(&self, body_id: BodyID) -> BodyType {
        self.body(body_id).type_
    }

    pub fn set_body_type(&mut self, body_id: BodyID, type_: BodyType) {
        let index = self.body_index(body_id);
        let mut body = self.bodies[index as usize].clone();

        let original_type = body.type_;
        if original_type == type_ {
            return;
        }

        if type_ != BodyType::StaticBody {
            let mut shape_id = body.head_shape_id;
            while shape_id != b3::NULL_INDEX {
                let shape = self.shapes[shape_id as usize].clone();
                if shape.type_ == ShapeType::CompoundShape || shape.type_ == ShapeType::HeightShape
                {
                    // Setting the body type is not supported for bodies with compound shapes.
                    return;
                }
                shape_id = shape.next_shape_id;
            }
        }

        let disabled_set_index = SetType::DisabledSet.raw_value();
        let static_set_index = SetType::StaticSet.raw_value();
        let awake_set_index = SetType::AwakeSet.raw_value();

        // Stage 1: disabled bodies do not change solver sets or islands when they change type.
        if body.set_index == disabled_set_index {
            body.type_ = type_;
            if type_ == BodyType::DynamicBody {
                body.flags |= BodyFlags::DYNAMIC_FLAG.0;
            } else {
                body.flags &= !BodyFlags::DYNAMIC_FLAG.0;
            }
            self.bodies[index as usize] = body;
            self.sync_body_flags(index);
            self.update_body_mass_data(index);
            return;
        }

        // Stage 2: destroy all contacts but don't wake bodies (we don't need to).
        self.destroy_body_contacts(index, false);

        // Stage 3: wake this body (does nothing if static).
        self.wake_body(index);

        // Stage 4: move joints to temporary storage in the static set.
        body = self.bodies[index as usize].clone();
        let mut joint_key = body.head_joint_key;
        while joint_key != b3::NULL_INDEX {
            let joint_id = joint_key >> 1;
            let edge_index = (joint_key & 1) as usize;

            let joint = self.joints[joint_id as usize].clone();
            joint_key = joint.edges[edge_index].next_key;

            // The joint may be disabled by the other body.
            if joint.set_index == disabled_set_index {
                continue;
            }

            // Wake attached bodies: the wake above does not wake bodies attached to a static body.
            self.wake_body(joint.edges[0].body_id);
            self.wake_body(joint.edges[1].body_id);

            // Remove the joint from its island
            self.unlink_joint(joint_id);

            // Transfer every joint to the static set so it can acquire consistent colors.
            let refreshed = self.joints[joint_id as usize].clone();
            self.transfer_joint(static_set_index, refreshed.set_index, joint_id);
        }

        // Stage 5: change the body type and transfer the body.
        body = self.bodies[index as usize].clone();
        body.type_ = type_;
        if type_ == BodyType::DynamicBody {
            body.flags |= BodyFlags::DYNAMIC_FLAG.0;
        } else {
            body.flags &= !BodyFlags::DYNAMIC_FLAG.0;
        }
        self.bodies[index as usize] = body.clone();

        body = self.bodies[index as usize].clone();
        let source_set_index = self.bodies[index as usize].set_index;
        let target_set_index =
            if type_ == BodyType::StaticBody { static_set_index } else { awake_set_index };
        self.transfer_body(target_set_index, source_set_index, index);

        // Stage 6: update island participation.
        if original_type == BodyType::StaticBody {
            self.create_island_for_body(awake_set_index, index);
        } else if type_ == BodyType::StaticBody {
            self.remove_body_from_island(index);
        }

        // Stage 7: transfer joints to the target set.
        body = self.bodies[index as usize].clone();
        joint_key = body.head_joint_key;
        while joint_key != b3::NULL_INDEX {
            let joint_id = joint_key >> 1;
            let edge_index = (joint_key & 1) as usize;

            let joint = self.joints[joint_id as usize].clone();
            joint_key = joint.edges[edge_index].next_key;

            if joint.set_index == disabled_set_index {
                continue;
            }

            let body_a = self.bodies[joint.edges[0].body_id as usize].clone();
            let body_b = self.bodies[joint.edges[1].body_id as usize].clone();

            if body_a.type_ == BodyType::DynamicBody || body_b.type_ == BodyType::DynamicBody {
                self.transfer_joint(awake_set_index, static_set_index, joint_id);
            }
        }

        // Recreate shape proxies in the broad-phase.
        body = self.bodies[index as usize].clone();
        let transform = self.solver_sets[body.set_index as usize].body_sims
            [body.local_index as usize]
            .transform;
        let mut shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let mut shape = self.shapes[shape_id as usize].clone();
            let next_shape_id = shape.next_shape_id;
            let write_index = shape.id;
            destroy_shape_proxy(&mut shape, &mut self.broad_phase);
            self.create_shape_proxy(&mut shape, type_, transform, true);
            self.shapes[write_index as usize] = shape;
            shape_id = next_shape_id;
        }

        // Relink all joints.
        body = self.bodies[index as usize].clone();
        joint_key = body.head_joint_key;
        while joint_key != b3::NULL_INDEX {
            let joint_id = joint_key >> 1;
            let edge_index = (joint_key & 1) as usize;

            let joint = self.joints[joint_id as usize].clone();
            joint_key = joint.edges[edge_index].next_key;

            let other_edge_index = edge_index ^ 1;
            let other_body = self.bodies[joint.edges[other_edge_index].body_id as usize].clone();

            if other_body.set_index == disabled_set_index {
                continue;
            }

            if body.type_ != BodyType::DynamicBody && other_body.type_ != BodyType::DynamicBody {
                continue;
            }

            self.link_joint(joint_id);
        }

        self.sync_body_flags(index);

        // A body type change flips the mass properties.
        self.update_body_mass_data(index);
    }
}

// MARK: - Awake state and fast flag

impl World {
    /// Awake or sleep a body. Sleeping splits the island first when it has pending removals.
    pub fn set_body_awake(&mut self, body_id: BodyID, awake: bool) {
        let index = self.body_index(body_id);
        let body = self.bodies[index as usize].clone();

        if awake && body.set_index >= SetType::FirstSleepingSet.raw_value() {
            self.wake_body(index);
        } else if awake == false && body.set_index == SetType::AwakeSet.raw_value() {
            let island = self.islands[body.island_id as usize].clone();
            if island.constraint_remove_count > 0 {
                // Must split the island before sleeping. This is expensive.
                self.split_island(body.island_id);
            }
            self.try_sleep_island(self.bodies[index as usize].island_id);
        }
    }

    /// True while the body is in the awake solver set.
    ///
    /// Swift overloaded this with `World.isBodyAwake(_ body: Body)` from `WorldRuntime.swift`;
    /// Rust has no overloading, so the by-id form carries the `_id` suffix.
    pub fn is_body_awake_id(&self, body_id: BodyID) -> bool {
        self.body(body_id).set_index == SetType::AwakeSet.raw_value()
    }

    /// True while the body sim carries the fast flag. The flag is set during body finalization and
    /// gates contact recycling, so it only lives on the sim.
    pub fn is_body_fast(&self, body_id: BodyID) -> bool {
        let body = self.body(body_id);
        if body.set_index == b3::NULL_INDEX {
            return false;
        }
        let sim =
            self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];
        (sim.flags & BodyFlags::IS_FAST.0) != 0
    }
}

// MARK: - Names

impl World {
    /// Set a body's name. The string is interned in the world cache, so repeated names share an id.
    pub fn set_body_name(&mut self, body_id: BodyID, name: Option<&str>) {
        let index = self.body_index(body_id);
        let mut body = self.bodies[index as usize].clone();
        body.name_id = self.names.add(name);
        self.bodies[index as usize] = body;
    }

    /// The body's name, or nil when it has none.
    pub fn body_name(&self, body_id: BodyID) -> Option<&str> {
        let name_id = self.body(body_id).name_id;
        self.names.find(name_id)
    }

    /// Set a shape's name.
    pub fn set_shape_name(&mut self, shape_id: ShapeID, name: Option<&str>) {
        let index = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[index].clone();
        shape.name_id = self.names.add(name);
        self.shapes[index] = shape;
    }

    /// The shape's name, or nil when it has none.
    pub fn shape_name(&self, shape_id: ShapeID) -> Option<&str> {
        let name_id = self.shapes[(shape_id.index1 - 1) as usize].name_id;
        self.names.find(name_id)
    }
}

impl World {
    /// Ray cast a single shape from `origin`. The cast is re-centered on the origin so it stays
    /// precise far from the world origin; the returned point is a world position.
    pub fn shape_ray_cast(
        &self,
        shape_id: ShapeID,
        origin: Pos,
        translation: Vec3,
    ) -> WorldCastOutput {
        let shape = self.shapes[(shape_id.index1 - 1) as usize].clone();
        let transform = to_relative_transform(self.body_transform_quick(shape.body_id), origin);

        // The ray starts at the origin, so its origin in the re-centered frame is zero.
        let mut input = RayCastInput::default();
        input.origin = Vec3::ZERO;
        input.translation = translation;
        input.max_fraction = 1.0;

        WorldCastOutput::from_local(&ray_cast_shape(&shape, transform, &input), origin)
    }
}

// MARK: - Forces and impulses

impl World {
    /// Apply a force at a world point. The force is cleared after the next step.
    ///
    /// Swift overloaded these six entry points with the positional-`wake` forms in
    /// `BodyAPI.swift`; Rust cannot overload, so the labeled-`wake:` forms keep the same behaviour
    /// under a `_wake` suffix.
    pub fn apply_body_force_wake(&mut self, body_id: BodyID, force: Vec3, point: Pos, wake: bool) {
        b3_assert(is_valid_vec3(force));
        let index = self.body_index(body_id);

        if wake {
            self.wake_body(index);
        }

        if self.bodies[index as usize].set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        let mut sim = self.body_sim_for(body_id);
        sim.force = sim.force + force;
        sim.torque = sim.torque + cross(sub_pos(point, sim.center), force);
        self.set_body_sim(body_id, sim);
    }

    /// Apply a force through the center of mass.
    pub fn apply_body_force_to_center_wake(&mut self, body_id: BodyID, force: Vec3, wake: bool) {
        b3_assert(is_valid_vec3(force));
        let index = self.body_index(body_id);

        if wake {
            self.wake_body(index);
        }

        if self.bodies[index as usize].set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        let mut sim = self.body_sim_for(body_id);
        sim.force = sim.force + force;
        self.set_body_sim(body_id, sim);
    }

    /// Apply a pure torque. The torque is cleared after the next step.
    pub fn apply_body_torque_wake(&mut self, body_id: BodyID, torque: Vec3, wake: bool) {
        b3_assert(is_valid_vec3(torque));
        let index = self.body_index(body_id);

        if wake {
            self.wake_body(index);
        }

        if self.bodies[index as usize].set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        let mut sim = self.body_sim_for(body_id);
        sim.torque = sim.torque + torque;
        self.set_body_sim(body_id, sim);
    }

    /// Apply an impulse at a world point, changing velocity immediately.
    pub fn apply_body_linear_impulse_wake(
        &mut self,
        body_id: BodyID,
        impulse: Vec3,
        point: Pos,
        wake: bool,
    ) {
        b3_assert(is_valid_vec3(impulse));
        let index = self.body_index(body_id);

        if wake {
            self.wake_body(index);
        }

        let record = self.bodies[index as usize].clone();
        if record.set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        let mut state = self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
            [record.local_index as usize];
        let sim = self.body_sim_for(body_id);

        state.linear_velocity = mul_add(state.linear_velocity, sim.inv_mass, impulse);

        // Clamp to the maximum linear speed.
        let max_speed = self.max_linear_speed;
        if length_squared(state.linear_velocity) > max_speed * max_speed {
            state.linear_velocity = max_speed * normalize(state.linear_velocity);
        }

        let delta = mul_mv(sim.inv_inertia_world, cross(sub_pos(point, sim.center), impulse));
        state.angular_velocity = state.angular_velocity + delta;
        self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
            [record.local_index as usize] = state;
    }

    /// Apply an impulse through the center of mass.
    pub fn apply_body_linear_impulse_to_center_wake(
        &mut self,
        body_id: BodyID,
        impulse: Vec3,
        wake: bool,
    ) {
        b3_assert(is_valid_vec3(impulse));
        let index = self.body_index(body_id);

        if wake {
            self.wake_body(index);
        }

        let record = self.bodies[index as usize].clone();
        if record.set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        let mut state = self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
            [record.local_index as usize];
        let sim = self.body_sim_for(body_id);

        state.linear_velocity = mul_add(state.linear_velocity, sim.inv_mass, impulse);

        let max_speed = self.max_linear_speed;
        if length_squared(state.linear_velocity) > max_speed * max_speed {
            state.linear_velocity = max_speed * normalize(state.linear_velocity);
        }

        self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
            [record.local_index as usize] = state;
    }

    /// Apply an angular impulse, changing angular velocity immediately.
    ///
    /// Unlike `BodyAPI.swift`'s positional-`wake` form, which rotates the impulse into the local
    /// inertia tensor, this one applies it straight through `invInertiaWorld`.
    pub fn apply_body_angular_impulse_wake(
        &mut self,
        body_id: BodyID,
        impulse: Vec3,
        wake: bool,
    ) {
        b3_assert(is_valid_vec3(impulse));
        let index = self.body_index(body_id);

        if wake {
            self.wake_body(index);
        }

        let record = self.bodies[index as usize].clone();
        if record.set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        let mut state = self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
            [record.local_index as usize];
        let sim = self.body_sim_for(body_id);
        state.angular_velocity = state.angular_velocity + mul_mv(sim.inv_inertia_world, impulse);
        self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
            [record.local_index as usize] = state;
    }

    // MARK: Body queries

    /// Number of shapes attached to the body.
    pub fn body_shape_count(&self, body_id: BodyID) -> i32 {
        self.body(body_id).shape_count
    }

    /// The body's joints, in list order.
    pub fn body_joints(&self, body_id: BodyID) -> Vec<JointID> {
        let mut ids: Vec<JointID> = Vec::new();
        let mut key = self.body(body_id).head_joint_key;
        while key != b3::NULL_INDEX {
            let joint_id = key >> 1;
            let edge_index = (key & 1) as usize;
            let joint = self.joints[joint_id as usize].clone();
            key = joint.edges[edge_index].next_key;
            ids.push(JointID::new(joint_id + 1, self.world_id, joint.generation));
        }
        ids
    }
}

// MARK: - Shape accessors

impl World {
    /// The live shape for `id`, or `None` when the id is stale or was never created.
    ///
    /// Several shape queries go through this so a rejected or destroyed id returns a sensible
    /// default instead of panicking.
    pub(crate) fn live_shape(&self, id: ShapeID) -> Option<&crate::simulation::shape_runtime::Shape> {
        if !self.shape_is_valid(id) {
            return None;
        }
        Some(&self.shapes[(id.index1 - 1) as usize])
    }

    /// The shape's geometry type, or `None` when the id is not a live shape.
    pub fn shape_type(&self, shape_id: ShapeID) -> Option<ShapeType> {
        self.live_shape(shape_id).map(|shape| shape.type_)
    }

    /// The body that owns this shape, or a null `BodyID` when the id is not a live shape.
    pub fn shape_body(&self, shape_id: ShapeID) -> BodyID {
        let Some(shape) = self.live_shape(shape_id) else {
            return BodyID::default();
        };
        let body = self.bodies[shape.body_id as usize].clone();
        BodyID::new(body.id + 1, self.world_id, body.generation)
    }

    // MARK: Material

    /// The shape's density, or zero when the id is not a live shape.
    pub fn shape_density(&self, shape_id: ShapeID) -> f32 {
        self.live_shape(shape_id).map_or(0.0, |shape| shape.density)
    }

    /// Set the shape density. Updating the body mass requires a body pass.
    pub fn set_shape_density(&mut self, shape_id: ShapeID, density: f32, update_body_mass: bool) {
        let index = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[index].clone();
        shape.density = density;
        self.shapes[index] = shape.clone();

        // The setter recomputes the mass inline rather than deferring it, so the dirty flag
        // never survives into a step.
        if update_body_mass {
            self.update_body_mass_data(shape.body_id);
        }
    }

    /// The base surface material's friction.
    pub fn shape_friction(&self, shape_id: ShapeID) -> f32 {
        self.shapes[(shape_id.index1 - 1) as usize].material(0).friction
    }

    /// Set the base surface material's friction.
    pub fn set_shape_friction(&mut self, shape_id: ShapeID, friction: f32) {
        let index = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[index].clone();
        if shape.materials.is_empty() {
            return;
        }
        shape.materials[0].friction = friction;
        self.shapes[index] = shape;
    }

    /// The base surface material's restitution.
    pub fn shape_restitution(&self, shape_id: ShapeID) -> f32 {
        self.shapes[(shape_id.index1 - 1) as usize].material(0).restitution
    }

    /// Set the base surface material's restitution.
    pub fn set_shape_restitution(&mut self, shape_id: ShapeID, restitution: f32) {
        let index = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[index].clone();
        if shape.materials.is_empty() {
            return;
        }
        shape.materials[0].restitution = restitution;
        self.shapes[index] = shape;
    }

    /// The base surface material.
    pub fn shape_surface_material(&self, shape_id: ShapeID) -> SurfaceMaterial {
        self.shapes[(shape_id.index1 - 1) as usize].material(0)
    }

    /// Replace the base surface material. Per-triangle mesh materials are untouched.
    pub fn set_shape_surface_material(&mut self, shape_id: ShapeID, material: SurfaceMaterial) {
        let index = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[index].clone();
        if shape.materials.is_empty() {
            return;
        }
        shape.materials[0] = material;
        self.shapes[index] = shape;
    }

    /// The shape's collision filter.
    pub fn shape_filter(&self, shape_id: ShapeID) -> Filter {
        self.shapes[(shape_id.index1 - 1) as usize].filter
    }

    // MARK: Event flags

    pub fn shape_are_sensor_events_enabled(&self, shape_id: ShapeID) -> bool {
        (self.shapes[(shape_id.index1 - 1) as usize].flags & ShapeFlags::ENABLE_SENSOR_EVENTS.0)
            != 0
    }

    pub fn shape_are_contact_events_enabled(&self, shape_id: ShapeID) -> bool {
        (self.shapes[(shape_id.index1 - 1) as usize].flags & ShapeFlags::ENABLE_CONTACT_EVENTS.0)
            != 0
    }

    pub fn shape_are_hit_events_enabled(&self, shape_id: ShapeID) -> bool {
        (self.shapes[(shape_id.index1 - 1) as usize].flags & ShapeFlags::ENABLE_HIT_EVENTS.0) != 0
    }

    pub fn shape_are_pre_solve_events_enabled(&self, shape_id: ShapeID) -> bool {
        (self.shapes[(shape_id.index1 - 1) as usize].flags
            & ShapeFlags::ENABLE_PRE_SOLVE_EVENTS.0)
            != 0
    }

    // MARK: Geometry access

    /// The shape's sphere, or nil when it is another type.
    pub fn shape_sphere(&self, shape_id: ShapeID) -> Option<Sphere> {
        match &self.shapes[(shape_id.index1 - 1) as usize].geometry {
            ShapeGeometry::Sphere(sphere) => Some(*sphere),
            _ => None,
        }
    }

    /// The shape's capsule, or nil when it is another type.
    pub fn shape_capsule(&self, shape_id: ShapeID) -> Option<Capsule> {
        match &self.shapes[(shape_id.index1 - 1) as usize].geometry {
            ShapeGeometry::Capsule(capsule) => Some(*capsule),
            _ => None,
        }
    }

    /// The shape's hull, or nil when it is another type.
    pub fn shape_hull(&self, shape_id: ShapeID) -> Option<Hull> {
        match &self.shapes[(shape_id.index1 - 1) as usize].geometry {
            ShapeGeometry::Hull(hull) => Some(hull.clone()),
            _ => None,
        }
    }

    /// The shape's mesh, or nil when it is another type.
    pub fn shape_mesh(&self, shape_id: ShapeID) -> Option<Mesh> {
        match &self.shapes[(shape_id.index1 - 1) as usize].geometry {
            ShapeGeometry::Mesh(mesh) => Some(mesh.clone()),
            _ => None,
        }
    }

    /// The shape's height field, or nil when it is another type.
    pub fn shape_height_field(&self, shape_id: ShapeID) -> Option<HeightField> {
        match &self.shapes[(shape_id.index1 - 1) as usize].geometry {
            ShapeGeometry::HeightField(field) => Some(field.clone()),
            _ => None,
        }
    }

    /// The shape's compound, or nil when it is another type.
    pub fn shape_compound(&self, shape_id: ShapeID) -> Option<Compound> {
        match &self.shapes[(shape_id.index1 - 1) as usize].geometry {
            ShapeGeometry::Compound(compound) => Some(compound.clone()),
            _ => None,
        }
    }
}

// MARK: - Additional shape API

impl World {
    /// True when the shape is a sensor.
    pub fn shape_is_sensor(&self, shape_id: ShapeID) -> bool {
        self.shapes[(shape_id.index1 - 1) as usize].sensor_index != b3::NULL_INDEX
    }

    /// The shape's user data pointer.
    pub fn shape_user_data(&self, shape_id: ShapeID) -> Option<*mut c_void> {
        self.shapes[(shape_id.index1 - 1) as usize].user_data
    }

    /// Replace the shape's user data pointer.
    pub fn set_shape_user_data(&mut self, shape_id: ShapeID, user_data: Option<*mut c_void>) {
        let index = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[index].clone();
        shape.user_data = user_data;
        self.shapes[index] = shape;
    }

    /// The shape's AABB in world space.
    pub fn shape_aabb(&self, shape_id: ShapeID) -> AABB {
        let shape = self.shapes[(shape_id.index1 - 1) as usize].clone();
        let transform = self.body_transform_quick(shape.body_id);
        offset_aabb(
            compute_shape_aabb(&shape, Transform::new(Vec3::ZERO, transform.q)),
            transform.p,
        )
    }

    /// The closest point on the shape to a target, in world space.
    pub fn shape_closest_point(&self, shape_id: ShapeID, target: Pos) -> Pos {
        let shape = self.shapes[(shape_id.index1 - 1) as usize].clone();
        let transform = self.body_transform_quick(shape.body_id);

        // The low level query runs in the body frame so it stays precise far from the origin.
        let local = inv_transform_world_point(transform, target);
        let mut output = DistanceOutput::default();
        let mut cache = SimplexCache::default();

        {
            // SAFETY: `point_storage` is a one element array owned by this scope and the proxy
            // buffer owns the shape points, matching Swift's
            // `withUnsafePointer(to:) { makeShapeProxy(shape).withProxy { ... } }`.
            let point_storage = [local];
            let buffer = make_shape_proxy(&shape);
            let mut input = DistanceInput::default();
            input.proxy_a = buffer.proxy();
            input.proxy_b = ShapeProxy::new(Some(point_storage.as_ptr()), 1, 0.0);
            input.transform = Transform::IDENTITY;
            input.use_radii = true;
            output = shape_distance(&input, &mut cache, None);
        }

        transform_world_point(transform, output.point_a)
    }

    /// The number of materials a mesh shape carries.
    pub fn shape_mesh_material_count(&self, shape_id: ShapeID) -> i32 {
        self.shapes[(shape_id.index1 - 1) as usize].material_count()
    }

    /// A mesh shape's per-triangle surface material.
    pub fn shape_mesh_surface_material(
        &self,
        shape_id: ShapeID,
        index: i32,
    ) -> Option<SurfaceMaterial> {
        let shape = self.shapes[(shape_id.index1 - 1) as usize].clone();
        if index < 0 || index >= shape.material_count() {
            return None;
        }
        Some(shape.material(index))
    }

    /// Replace a mesh shape's per-triangle surface material.
    pub fn set_shape_mesh_material(
        &mut self,
        shape_id: ShapeID,
        index: i32,
        material: SurfaceMaterial,
    ) {
        let position = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[position].clone();
        if index < 0 || index >= shape.materials.len() as i32 {
            return;
        }
        shape.materials[index as usize] = material;
        self.shapes[position] = shape;
    }

    /// Enable or disable sensor events for a shape.
    pub fn enable_shape_sensor_events(&mut self, shape_id: ShapeID, enable: bool) {
        self.set_shape_flag(shape_id, ShapeFlags::ENABLE_SENSOR_EVENTS, enable);
    }

    /// Enable or disable contact events for a shape.
    pub fn enable_shape_contact_events(&mut self, shape_id: ShapeID, enable: bool) {
        self.set_shape_flag(shape_id, ShapeFlags::ENABLE_CONTACT_EVENTS, enable);
    }

    /// Enable or disable hit events for a shape.
    pub fn enable_shape_hit_events(&mut self, shape_id: ShapeID, enable: bool) {
        self.set_shape_flag(shape_id, ShapeFlags::ENABLE_HIT_EVENTS, enable);
    }

    /// Enable or disable pre-solve events for a shape.
    pub fn enable_shape_pre_solve_events(&mut self, shape_id: ShapeID, enable: bool) {
        self.set_shape_flag(shape_id, ShapeFlags::ENABLE_PRE_SOLVE_EVENTS, enable);
    }

    fn set_shape_flag(&mut self, shape_id: ShapeID, flag: ShapeFlags, enable: bool) {
        let index = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[index].clone();
        if enable {
            shape.flags |= flag.0;
        } else {
            shape.flags &= !flag.0;
        }
        self.shapes[index] = shape;
    }

    /// Replace the shape's collision filter. This is almost as expensive as recreating the shape.
    pub fn set_shape_filter(&mut self, shape_id: ShapeID, filter: Filter) {
        let index = (shape_id.index1 - 1) as usize;
        let mut shape = self.shapes[index].clone();
        shape.filter = filter;
        self.shapes[index] = shape;
    }

    /// The world's axis-aligned bounds, or nil when it has no shapes.
    pub fn world_bounds(&self) -> Option<AABB> {
        let mut bounds = AABB::default();
        let mut have_bounds = false;

        for tree in self.broad_phase.trees.iter() {
            if tree.proxy_count <= 0 {
                continue;
            }
            let root = tree.root_bounds();
            bounds = if have_bounds { aabb_union(bounds, root) } else { root };
            have_bounds = true;
        }
        if have_bounds { Some(bounds) } else { None }
    }
}

// MARK: - Body extras

impl World {
    /// The touching contacts of a body. Only touching contacts are reported.
    pub fn body_contact_data(&self, body_id: BodyID, capacity: usize) -> Vec<ContactData> {
        let mut result: Vec<ContactData> = Vec::new();
        let mut key = self.body(body_id).head_contact_key;

        while key != b3::NULL_INDEX && result.len() < capacity {
            let contact_id = key >> 1;
            let edge_index = (key & 1) as usize;
            let contact = self.contacts[contact_id as usize];
            key = contact.edges[edge_index].next_key;

            if (contact.flags & ContactFlags::TOUCHING.0) == 0 {
                continue;
            }

            let shape_a = self.shapes[contact.shape_id_a as usize].clone();
            let shape_b = self.shapes[contact.shape_id_b as usize].clone();

            let mut data = ContactData::default();
            data.contact_id = ContactID::new(
                contact.contact_id + 1,
                self.world_id,
                contact.generation,
            );
            data.shape_id_a =
                ShapeID::new(shape_a.id + 1, self.world_id, shape_a.generation);
            data.shape_id_b =
                ShapeID::new(shape_b.id + 1, self.world_id, shape_b.generation);
            data.manifolds = contact.manifold_buffer().to_vec();
            result.push(data);
        }

        result
    }

    /// The world AABB containing all of the body's shapes. Empty and centered on the body origin
    /// when the body has no shapes.
    pub fn body_compute_aabb(&self, body_id: BodyID) -> AABB {
        let record = self.body(body_id);
        if record.head_shape_id == b3::NULL_INDEX {
            let p = self.body_transform_quick(self.body_index(body_id)).p;
            return offset_aabb(AABB::new(Vec3::ZERO, Vec3::ZERO), p);
        }

        let first = self.shapes[record.head_shape_id as usize].clone();
        let mut aabb = first.aabb;
        let mut shape_id = first.next_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = self.shapes[shape_id as usize].clone();
            aabb = aabb_union(aabb, shape.aabb);
            shape_id = shape.next_shape_id;
        }
        aabb
    }

    /// Recompute the body's mass properties from its shapes.
    pub fn apply_body_mass_from_shapes(&mut self, body_id: BodyID) {
        let index = self.body_index(body_id);
        self.update_body_mass_data(index);
    }

    /// The closest point on the body to a world target, and its distance.
    pub fn body_closest_point(&self, body_id: BodyID, target: Vec3) -> (Vec3, f32) {
        let transform = to_relative_transform(self.body_transform_quick(self.body_index(body_id)), Pos::ZERO);

        let mut closest_distance = f32::MAX;
        let mut closest_point = transform.p;

        let local_target = target;
        let record = self.body(body_id);
        let mut shape_id = record.head_shape_id;

        while shape_id != b3::NULL_INDEX {
            let shape = self.shapes[shape_id as usize].clone();
            let mut cache = SimplexCache::default();

            let output = {
                // SAFETY: `point_storage` is a one element array owned by this scope and `buffer`
                // owns the shape points, matching Swift's
                // `withUnsafePointer(to:) { makeShapeProxy(shape).withProxy { ... } }`.
                let point_storage = [local_target];
                let buffer = make_shape_proxy(&shape);
                let mut input = DistanceInput::default();
                input.proxy_a = ShapeProxy::new(Some(point_storage.as_ptr()), 1, 0.0);
                input.proxy_b = buffer.proxy();
                input.transform = transform;
                input.use_radii = false;
                shape_distance(&input, &mut cache, None)
            };

            if output.distance < closest_distance {
                closest_distance = output.distance;
                closest_point = output.point_b;
            }
            shape_id = shape.next_shape_id;
        }

        (closest_point, closest_distance)
    }

    /// Allow or forbid fast rotation for this body.
    pub fn allow_body_fast_rotation(&mut self, body_id: BodyID, allow: bool) {
        let index = self.body_index(body_id);
        let mut record = self.bodies[index as usize].clone();
        if allow {
            record.flags |= BodyFlags::ALLOW_FAST_ROTATION.0;
        } else {
            record.flags &= !BodyFlags::ALLOW_FAST_ROTATION.0;
        }
        self.bodies[index as usize] = record;
        self.sync_body_flags(index);
    }

    /// True when this body may rotate faster than the angular speed limit.
    pub fn is_body_fast_rotation_allowed(&self, body_id: BodyID) -> bool {
        (self.body(body_id).flags & BodyFlags::ALLOW_FAST_ROTATION.0) != 0
    }

    /// The body's user data pointer.
    pub fn body_user_data(&self, body_id: BodyID) -> Option<*mut c_void> {
        self.body(body_id).user_data
    }

    /// Replace the body's user data pointer.
    pub fn set_body_user_data(&mut self, body_id: BodyID, user_data: Option<*mut c_void>) {
        let index = self.body_index(body_id);
        let mut record = self.bodies[index as usize].clone();
        record.user_data = user_data;
        self.bodies[index as usize] = record;
    }

    /// Drive a body toward a target transform by setting its velocity. Used by character movers.
    pub fn set_body_target_transform(
        &mut self,
        body_id: BodyID,
        target: WorldTransform,
        time_step: f32,
        wake: bool,
    ) {
        let index = self.body_index(body_id);
        let record = self.bodies[index as usize].clone();

        if record.set_index == SetType::DisabledSet.raw_value() {
            return;
        }
        if record.type_ == BodyType::StaticBody || time_step <= 0.0 {
            return;
        }
        if record.set_index != SetType::AwakeSet.raw_value() && wake == false {
            return;
        }

        if wake {
            self.wake_body(index);
        }

        let sim = self.body_sim_for(body_id);
        let current = self.body(body_id);
        if current.set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        // Linear velocity from the center difference, taken in world precision.
        let center1 = sim.center;
        let center2 = transform_world_point(target, sim.local_center);
        let inv_time_step = 1.0 / time_step;
        let linear_velocity = inv_time_step * sub_pos(center2, center1);

        // w = 2 * (q2 - q1) * conjugate(q1) / dt, along the shortest arc.
        let mut q1 = sim.transform.q;
        let mut q2 = target.q;
        if dot_quat(q1, q2) < 0.0 {
            q2 = Quat::new(
                Vec3::new(-q2.v.x, -q2.v.y, -q2.v.z),
                -q2.s,
            );
        }
        q1 = sim.transform.q;
        let dq = Quat::new(
            Vec3::new(q2.v.x - q1.v.x, q2.v.y - q1.v.y, q2.v.z - q1.v.z),
            q2.s - q1.s,
        );
        let omega = mul_quat(dq, conjugate(q1));
        let angular_velocity = (2.0 * inv_time_step) * omega.v;

        let local_index = current.local_index as usize;
        let awake_set_index = SetType::AwakeSet.raw_value() as usize;
        let mut state = self.solver_sets[awake_set_index].body_states[local_index];
        state.linear_velocity = linear_velocity;
        state.angular_velocity = angular_velocity;
        self.solver_sets[awake_set_index].body_states[local_index] = state;
    }
}

// MARK: - Shape contacts and contact validity

impl World {
    /// The touching contacts that involve this shape. Sensors report nothing.
    pub fn shape_contact_data(&self, shape_id: ShapeID, capacity: usize) -> Vec<ContactData> {
        let shape_index = shape_id.index1 - 1;
        let shape = self.shapes[shape_index as usize].clone();
        if shape.sensor_index != b3::NULL_INDEX {
            return Vec::new();
        }

        let mut result: Vec<ContactData> = Vec::new();
        let mut key = self.bodies[shape.body_id as usize].head_contact_key;

        while key != b3::NULL_INDEX && result.len() < capacity {
            let contact_id = key >> 1;
            let edge_index = (key & 1) as usize;
            let contact = self.contacts[contact_id as usize];
            key = contact.edges[edge_index].next_key;

            let involves_shape =
                contact.shape_id_a == shape_index || contact.shape_id_b == shape_index;
            if !involves_shape || (contact.flags & ContactFlags::TOUCHING.0) == 0 {
                continue;
            }

            let shape_a = self.shapes[contact.shape_id_a as usize].clone();
            let shape_b = self.shapes[contact.shape_id_b as usize].clone();

            let mut data = ContactData::default();
            data.contact_id = ContactID::new(
                contact.contact_id + 1,
                self.world_id,
                contact.generation,
            );
            data.shape_id_a =
                ShapeID::new(shape_a.id + 1, self.world_id, shape_a.generation);
            data.shape_id_b =
                ShapeID::new(shape_b.id + 1, self.world_id, shape_b.generation);

            data.manifolds = contact.manifold_buffer().to_vec();
            result.push(data);
        }

        result
    }

    /// True while the contact id refers to a live contact.
    pub fn contact_is_valid(&self, contact_id: ContactID) -> bool {
        let index = contact_id.index1 - 1;
        if index < 0 || index as usize >= self.contacts.len() {
            return false;
        }
        let contact = self.contacts[index as usize];
        contact.contact_id != b3::NULL_INDEX && contact.generation == contact_id.generation
    }
}

impl World {
    /// True when continuous collision handled this body during the most recent step. The flag is
    /// transient and is cleared at the start of the next step.
    pub fn is_body_time_of_impact(&self, body_id: BodyID) -> bool {
        let record = self.body(body_id);
        let sim =
            self.solver_sets[record.set_index as usize].body_sims[record.local_index as usize];
        (sim.flags & BodyFlags::HAD_TIME_OF_IMPACT.0) != 0
    }

    /// The body's minimum extent, the smallest half dimension over its shapes.
    pub fn body_min_extent_value(&self, body_id: BodyID) -> f32 {
        self.body_min_extent(body_id)
    }
}
