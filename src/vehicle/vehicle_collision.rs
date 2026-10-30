//! Wheel collision: each step every wheel of an awake vehicle is cast along its suspension. The
//! casts only read the world, so with workers they run in parallel across vehicles.
//!
//! Translated from `Sources/Natsel/Vehicle/VehicleCollision.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md):
//! - `crate::vehicle::vehicle_runtime` (`VehicleRuntime.swift`): `VehicleRecord` (fields `id: i32`,
//!   `generation: u16`, `body_id: BodyID`, `active: bool`, `wheels: Vec<VehicleWheel>`, `tester:
//!   VehicleCollisionTester`, `filter: QueryFilter` (`Clone`), `cos_max_slope_angle: f32`,
//!   `forward_input: f32`, `right_input: f32`, `drivetrain: VehicleDrivetrain`),
//!   `VehicleRecord::empty()` (Swift's `static let empty`), `VehicleWheel` (`Copy`, fields
//!   `settings: WheelParameters`, `steer_angle`, `has_contact`, `contact_body: i32`,
//!   `contact_body_id: BodyID`, `contact_shape_id: ShapeID`, `contact_position: Pos`,
//!   `contact_normal`, `longitudinal`, `lateral`, `suspension_length`, `surface_friction`,
//!   `user_material_id: u64`), and `WheelParameters` (`Copy`, fields `position`,
//!   `suspension_direction`, `steering_axis`, `wheel_up`, `wheel_forward`,
//!   `suspension_max_length`, `radius`, `width`, `max_steer_angle`). The World method
//!   `vehicle_forward_speed(&self, VehicleID) -> f32`.
//! - `crate::vehicle::vehicle_types`: `VehicleID::new(index1, world0, generation)` and
//!   `VehicleCollisionTester::{Ray, Sphere, Cylinder(f32)}` (tuple payload; `Clone`).
//! - `crate::vehicle::vehicle_drivetrain` (this batch): `VehicleControllerKind`.
//! - `crate::simulation::filtered_queries::{QueryHit, World::closest_ray_hit,
//!   World::closest_shape_cast_hit}`; Swift defaulted `canEncroach` to `false`, passed explicitly
//!   per the established precedent.
//! - the shared `World` anchor: `vehicles`, `vehicle_id_pool`, `bodies`, `shapes`, `worker_count`,
//!   `gravity`, `world_id`, `body_index`, `is_valid_body`, `make_body_id`, `wake_body`,
//!   `body_transform_quick`.

use std::thread;

use crate::base::constants as b3;
use crate::math::functions::{
    atan2_approx, clamp_float, compute_cos_sin, cross, dot, length, length_squared,
    make_quat_from_axis_angle, max_float, max_int, min_float, min_int, mul_add, mul_quat,
    normalize, pos_mul_add, pos_mul_sub, rotate_vector, sub_pos, transform_world_point,
};
use crate::math::types::{Quat, Vec3, WorldTransform};
use crate::simulation::filtered_queries::QueryHit;
use crate::simulation::world_runtime::{SetType, World};
use crate::types::body_def::BodyType;
use crate::types::ids::ShapeID;
use crate::types::queries::ShapeProxy;
use crate::vehicle::vehicle_drivetrain::VehicleControllerKind;
use crate::vehicle::vehicle_runtime::{VehicleRecord, WheelParameters};
use crate::vehicle::vehicle_types::{VehicleCollisionTester, VehicleID};

/// Rim points per side of the cylinder wheel proxy.
const CYLINDER_SEGMENTS: usize = 12;

/// The world and vehicle storage handed to the parallel wheel-cast workers.
///
/// SAFETY: this struct only crosses the thread boundary of the cast scope inside
/// [`World::update_vehicle_collisions`]. `records` addresses the stable heap buffer of
/// `vehicles` (no reallocation happens while the scope runs because the vehicle count and the
/// `active` list are fixed before it starts); `indices` addresses the read-only `active` list;
/// `world` is only ever used through shared references. Swift formed the same aliasing
/// (`withUnsafeMutableBufferPointer` plus `nonisolated(unsafe) let world = self`) and relied on
/// Swift's unchecked exclusivity: each worker writes only the wheel array of its own vehicle
/// record, and the wheel casts only read bodies, shapes, and the broad phase, which no worker
/// mutates. The shared-`World` view therefore never observes a concurrent write through it.
#[derive(Clone, Copy)]
struct WheelCastBuffers {
    world: *const World,
    records: *mut VehicleRecord,
    indices: *const usize,
    count: usize,
}

// SAFETY: see the type-level note; the Send/Sync contract is the one the Swift
// `DispatchQueue.concurrentPerform` closure already satisfied.
unsafe impl Send for WheelCastBuffers {}
unsafe impl Sync for WheelCastBuffers {}

impl World {
    /// Cast all wheels and wake whatever the vehicles stand on. Runs after the narrow phase and
    /// before the solver, while waking solver sets is still allowed.
    pub(crate) fn update_vehicle_collisions(&mut self) {
        if !(self.vehicle_id_pool.id_count() > 0) {
            return;
        }

        // Validate, wake, and steer serially; these change world state.
        let mut active: Vec<usize> = Vec::new();
        for i in 0..self.vehicles.len() {
            if self.vehicles[i].id == b3::NULL_INDEX {
                continue;
            }
            self.vehicles[i].active = false;

            let chassis_id = self.vehicles[i].body_id;
            if !self.is_valid_body(chassis_id) {
                // The chassis was destroyed; the vehicle goes with it.
                let generation = self.vehicles[i].generation;
                self.vehicles[i] = VehicleRecord::empty();
                self.vehicles[i].generation = generation;
                self.vehicle_id_pool.free(i as i32);
                continue;
            }

            let chassis_index = self.body_index(chassis_id);
            if self.bodies[chassis_index as usize].set_index != SetType::AwakeSet.raw_value() {
                if !self.should_wake_sleeping_vehicle(i) {
                    continue;
                }
                self.wake_body(chassis_index);
                if self.bodies[chassis_index as usize].set_index != SetType::AwakeSet.raw_value() {
                    continue;
                }
            }

            self.vehicles[i].active = true;
            self.apply_vehicle_steering(i);
            active.push(i);
        }

        // Cast the wheels.
        let workers = max_int(1, min_int(self.worker_count, 8));
        let buffers = {
            let records: *mut VehicleRecord = self.vehicles.as_mut_ptr();
            let world: *const World = std::ptr::from_ref(&*self);
            WheelCastBuffers {
                world,
                records,
                indices: active.as_ptr(),
                count: active.len(),
            }
        };
        if workers > 1 && active.len() >= 8 {
            let worker_count = workers as usize;
            // Every iteration is independent (see `WheelCastBuffers`), so partitioning the index
            // list into contiguous chunks — the established port of
            // `DispatchQueue.concurrentPerform` (see `collide_pass.rs`) — yields the same final
            // state as Swift's unordered iteration; accumulation order is derived from the item
            // indices, not from completion order.
            let chunk = (active.len() + worker_count - 1) / worker_count;
            thread::scope(|scope| {
                for wi in 0..worker_count {
                    scope.spawn(move || {
                        // Bind the whole buffer struct so the closure captures it (and its
                        // `Send`/`Sync` impls) instead of the individual raw-pointer fields.
                        let buffers = buffers;
                        // SAFETY: the scope outlives every worker; `world`, `records`, and
                        // `indices` address live, initialized storage for the whole scope
                        // (`WheelCastBuffers`). Worker `wi` reads indices `[wi * chunk,
                        // min(len, (wi + 1) * chunk))` — a disjoint slice of the `active` list —
                        // so each vehicle record is touched by exactly one thread and the shared
                        // `World` view is only ever read.
                        let world = unsafe { &*buffers.world };
                        let start = wi * chunk;
                        let end = buffers.count.min(start + chunk);
                        for k in start..end {
                            let i = unsafe { *buffers.indices.add(k) };
                            // SAFETY: `records` addresses the vehicles buffer and `i < count` is
                            // an index of a live record; the pointer is the exclusive handle for
                            // this vehicle during the scope.
                            let record = unsafe { buffers.records.add(i) };
                            world.cast_all(record);
                        }
                    });
                }
            });
        } else {
            for i in active.iter().copied() {
                // SAFETY: one record at a time, written only through this pointer while the
                // surrounding `&self` borrows of `cast_all` are read-only into other storage
                // (bodies, shapes, broad phase); the `&mut` borrow ends when the raw pointer is
                // formed, matching Swift's mutable-buffer-pointer + `self` capture.
                let record: *mut VehicleRecord = std::ptr::from_mut(&mut self.vehicles[i]);
                self.cast_all(record);
            }
        }

        for i in active.iter().copied() {
            let chassis_index = self.body_index(self.vehicles[i].body_id);

            // Bodies under the wheels must be awake to receive the tire and suspension impulses.
            for w in 0..self.vehicles[i].wheels.len() {
                if self.vehicles[i].wheels[w].has_contact {
                    let ground = self.vehicles[i].wheels[w].contact_body;
                    if self.bodies[ground as usize].type_ == BodyType::DynamicBody
                        && self.bodies[ground as usize].set_index
                            >= SetType::FirstSleepingSet.raw_value()
                    {
                        self.wake_body(ground);
                    }
                }
            }

            // Throttle keeps the vehicle awake even when it is stuck.
            if self.vehicles[i].forward_input != 0.0 {
                self.bodies[chassis_index as usize].sleep_time = 0.0;
            }
        }
    }

    /// Swift's nested `castAll(_:)`: cast every wheel of one vehicle record.
    fn cast_all(&self, record: *mut VehicleRecord) {
        // SAFETY: `record` addresses a live, initialized `VehicleRecord` for the duration of the
        // call (the vehicles buffer base plus a validated index); reads only.
        let body_id = unsafe { (*record).body_id };
        let chassis_index = self.body_index(body_id);
        let transform = self.body_transform_quick(chassis_index);
        // SAFETY: as above; reads only.
        let wheel_count = unsafe { (*record).wheels.len() };
        for w in 0..wheel_count {
            self.cast_vehicle_wheel(record, w, transform, chassis_index);
        }
    }

    /// A sleeping vehicle wakes when its ground moves or disappears.
    fn should_wake_sleeping_vehicle(&self, index: usize) -> bool {
        for wheel in self.vehicles[index].wheels.iter() {
            if !wheel.has_contact {
                continue;
            }
            if !self.is_valid_body(wheel.contact_body_id) {
                return true;
            }
            let ground = &self.bodies[self.body_index(wheel.contact_body_id) as usize];
            if ground.set_index == SetType::AwakeSet.raw_value()
                && ground.type_ != BodyType::StaticBody
            {
                return true;
            }
        }
        false
    }

    /// Steering from driver input. Motorcycles limit steering by the lean they can sustain.
    fn apply_vehicle_steering(&mut self, index: usize) {
        let speed = self.vehicle_forward_speed(VehicleID::new(
            self.vehicles[index].id + 1,
            self.world_id,
            self.vehicles[index].generation,
        ));
        let mut record = VehicleRecord::empty();
        std::mem::swap(&mut record, &mut self.vehicles[index]);
        // Swift's `defer { swap(&record, &vehicles[index]) }`: the swap-back runs on every exit
        // path, including the early return below.

        if record.drivetrain.kind == VehicleControllerKind::Tracked {
            for w in 0..record.wheels.len() {
                record.wheels[w].steer_angle = 0.0;
            }
            std::mem::swap(&mut record, &mut self.vehicles[index]);
            return;
        }

        let mut limit: f32 = f32::MAX;
        if record.drivetrain.kind == VehicleControllerKind::Motorcycle
            && record.drivetrain.lean.settings.enable_lean_steering_limit
        {
            let g = length(self.gravity);
            let wheel_base = record.drivetrain.lean.wheel_base;
            if speed * speed > 1.0e-4 && wheel_base > 0.0 && g > 0.0 {
                let cs = compute_cos_sin(record.drivetrain.lean.settings.max_lean_angle);
                let tan_lean = if cs.cosine > 0.0 {
                    cs.sine / cs.cosine
                } else {
                    f32::MAX
                };
                limit = atan2_approx(wheel_base * g * tan_lean, speed * speed);
            }
        }

        for w in 0..record.wheels.len() {
            let max_steer = min_float(record.wheels[w].settings.max_steer_angle, limit);
            // Positive right input turns right: a negative rotation about the steering (up) axis.
            record.wheels[w].steer_angle = -record.right_input * max_steer;
        }

        std::mem::swap(&mut record, &mut self.vehicles[index]);
    }

    fn cast_vehicle_wheel(
        &self,
        record_pointer: *mut VehicleRecord,
        w: usize,
        chassis: WorldTransform,
        chassis_index: i32,
    ) {
        // Read fields, not the whole record: a copy would retain the wheel array and make the
        // write back below copy it.
        //
        // SAFETY: `record_pointer` addresses a live, initialized `VehicleRecord`; these reads
        // happen while no write to the record is in flight (the write-back is at the end).
        let tester = unsafe { (*record_pointer).tester.clone() };
        let filter = unsafe { (*record_pointer).filter.clone() };
        let cos_max_slope = unsafe { (*record_pointer).cos_max_slope_angle };
        // SAFETY: `VehicleWheel` is `Copy` (Swift's `BitwiseCopyable`), so this is the Swift
        // value copy of the wheel out of the record.
        let mut wheel = unsafe { (&(*record_pointer)).wheels[w] };
        let s = wheel.settings;

        let origin = transform_world_point(chassis, s.position);
        let direction = rotate_vector(chassis.q, s.suspension_direction);
        let steer = make_quat_from_axis_angle(s.steering_axis, wheel.steer_angle);
        let forward = rotate_vector(chassis.q, rotate_vector(steer, s.wheel_forward));
        let accept = |normal: Vec3| dot(normal, -direction) >= cos_max_slope;

        let mut hit = QueryHit::default();
        let mut length: f32 = 0.0;
        match tester {
            VehicleCollisionTester::Ray => {
                let cast_length = s.suspension_max_length + s.radius;
                hit = self.closest_ray_hit(
                    origin,
                    cast_length * direction,
                    &filter,
                    chassis_index,
                    accept,
                );
                length = hit.fraction * cast_length - s.radius;
            }
            VehicleCollisionTester::Sphere => {
                let point = Vec3::default();
                // SAFETY: `point` is live and never written again until `origin`'s scope ends,
                // which is after the cast returns; the proxy borrows it for exactly the Swift
                // `withUnsafePointer(to:)` lifetime.
                let proxy = ShapeProxy::new(Some(std::ptr::from_ref(&point)), 1, s.radius);
                hit = self.closest_shape_cast_hit(
                    origin,
                    proxy,
                    s.suspension_max_length * direction,
                    &filter,
                    chassis_index,
                    false,
                    accept,
                );
                length = hit.fraction * s.suspension_max_length;
            }
            VehicleCollisionTester::Cylinder(fraction) => {
                // Swift's `withUnsafeTemporaryAllocation(of: Vec3.self, capacity: 2 *
                // cylinderSegments)`: a fixed stack buffer, all 2 * CYLINDER_SEGMENTS entries
                // written before the cast reads them.
                let mut buffer = [Vec3::default(); 2 * CYLINDER_SEGMENTS];
                let radius =
                    self.make_wheel_cylinder_points(s, chassis.q, steer, fraction, &mut buffer);
                // SAFETY: `buffer` is live, initialized, and never reallocated or written again
                // until the cast returns; the pointer covers exactly `buffer.len()` elements,
                // matching the Swift temporary allocation's lifetime.
                let proxy = ShapeProxy::new(
                    Some(buffer.as_ptr()),
                    buffer.len() as i32,
                    radius,
                );
                hit = self.closest_shape_cast_hit(
                    origin,
                    proxy,
                    s.suspension_max_length * direction,
                    &filter,
                    chassis_index,
                    false,
                    accept,
                );
                length = hit.fraction * s.suspension_max_length;

                // A flat cylinder touches along a line; GJK reports one end of it. Move the
                // contact to the wheel's mid plane so the tire force acts in the middle of the
                // tread.
                if hit.hit {
                    let axle = rotate_vector(
                        chassis.q,
                        rotate_vector(steer, cross(s.wheel_up, s.wheel_forward)),
                    );
                    let center = pos_mul_add(origin, length, direction);
                    hit.point =
                        pos_mul_sub(hit.point, dot(sub_pos(hit.point, center), axle), axle);
                }
            }
        }

        let lateral = if hit.hit {
            cross(hit.normal, forward)
        } else {
            Vec3::default()
        };
        if hit.hit && length_squared(lateral) > 1.0e-6 && length <= s.suspension_max_length {
            let shape = &self.shapes[hit.shape_index as usize];
            let material = shape.material(hit.material_index);
            wheel.has_contact = true;
            wheel.contact_body = shape.body_id;
            wheel.contact_body_id = self.make_body_id(shape.body_id);
            wheel.contact_shape_id =
                ShapeID::new(shape.id + 1, self.world_id, shape.generation);
            wheel.contact_position = hit.point;
            wheel.contact_normal = hit.normal;
            wheel.lateral = normalize(lateral);
            wheel.longitudinal = cross(wheel.lateral, hit.normal);
            wheel.suspension_length = length;
            wheel.surface_friction = material.friction;
            wheel.user_material_id = material.user_material_id;
        } else {
            wheel.has_contact = false;
            wheel.contact_body = b3::NULL_INDEX;
            wheel.suspension_length = s.suspension_max_length;
            wheel.longitudinal = forward;
            wheel.lateral = Vec3::default();
            wheel.contact_normal = -direction;
            wheel.contact_position =
                pos_mul_add(origin, s.suspension_max_length + s.radius, direction);
        }

        // SAFETY: `record_pointer` addresses the live record slot; `w` is the exclusive wheel
        // index for this call, and this is the write-back of the local copy, exactly as in
        // Swift. The field reads above completed before this point.
        unsafe {
            (&mut (*record_pointer).wheels)[w] = wheel;
        }
    }

    /// A rounded cylinder around the axle, relative to the suspension attachment. Fills `points`
    /// with `2 * cylinderSegments` vertices and returns the rounding radius.
    fn make_wheel_cylinder_points(
        &self,
        s: WheelParameters,
        chassis: Quat,
        steer: Quat,
        convex_radius_fraction: f32,
        points: &mut [Vec3],
    ) -> f32 {
        let half_width = 0.5 * s.width;
        let convex_radius =
            clamp_float(convex_radius_fraction, 0.0, 1.0) * min_float(s.radius, half_width);
        let rim = s.radius - convex_radius;
        let side = max_float(half_width - convex_radius, 0.0);

        let rotation = mul_quat(chassis, steer);
        let up = rotate_vector(rotation, s.wheel_up);
        let forward = rotate_vector(rotation, s.wheel_forward);
        let axle = normalize(cross(up, forward));

        for k in 0..CYLINDER_SEGMENTS {
            let cs = compute_cos_sin(b3::TWO_PI * (k as f32) / (CYLINDER_SEGMENTS as f32));
            let radial = cs.cosine * up + cs.sine * forward;
            let center = rim * radial;
            points[2 * k] = mul_add(center, side, axle);
            points[2 * k + 1] = mul_add(center, -side, axle);
        }
        convex_radius
    }
}
