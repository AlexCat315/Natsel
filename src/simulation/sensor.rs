//! Sensors and their touch events.
//!
//! Translated from `Sources/Natsel/Simulation/Sensor.swift`.

use crate::base::constants as b3;
use crate::collision::capsule::overlap_capsule;
use crate::collision::compound::child_queries::overlap_compound;
use crate::collision::height_field::proxy_overlap::overlap_height_field;
use crate::collision::hull_queries::overlap_hull;
use crate::collision::mesh::overlap::overlap_mesh;
use crate::collision::sphere::overlap_sphere;
use crate::containers::bit_set::BitSet;
use crate::math::functions::{
    inv_mul_transforms, max_int, min_int, to_relative_transform, transform_point,
};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::types::body_def::BodyType;
use crate::types::events::{SensorBeginTouchEvent, SensorEndTouchEvent};
use crate::types::ids::ShapeID;
use crate::types::queries::ShapeProxy;

use crate::simulation::shape_runtime::{
    Shape, ShapeFlags, ShapeGeometry, is_convex, make_shape_proxy, should_shapes_collide,
};
use crate::simulation::world_runtime::{SetType, World};

/// A shape that overlaps a sensor.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct SensorVisitor {
    pub shape_id: i32,
    pub generation: u16,
}

impl Default for SensorVisitor {
    /// Swift's `init() {}` applied the property defaults, and `shapeId` starts at `B3.nullIndex`,
    /// so the derived all-zeroes default is not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            shape_id: b3::NULL_INDEX,
            generation: 0,
        }
    }
}

impl SensorVisitor {
    /// Swift's `init()`.
    #[inline(always)]
    pub fn new() -> Self {
        Self::default()
    }
}

/// A sensor records the shapes overlapping it across two frames.
///
/// Swift's `struct Sensor` had value semantics with copy-on-write arrays; `Vec` fields reproduce
/// that, and the whole record is owned by `World::sensors`.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Sensor {
    pub shape_id: i32,

    /// Hits recorded during the current step (e.g. by CCD).
    pub hits: Vec<SensorVisitor>,

    /// Overlaps from the previous frame.
    pub overlaps1: Vec<SensorVisitor>,

    /// Overlaps from the current frame.
    pub overlaps2: Vec<SensorVisitor>,
}

impl Default for Sensor {
    /// Swift's `init() {}` left `shapeId` at `B3.nullIndex`, so the derived all-zeroes default is
    /// not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            shape_id: b3::NULL_INDEX,
            hits: Vec::new(),
            overlaps1: Vec::new(),
            overlaps2: Vec::new(),
        }
    }
}

impl Sensor {
    /// Swift's `init()`.
    #[inline(always)]
    pub fn new() -> Self {
        Self::default()
    }
}

impl World {
    /// True when a sensor shape overlaps a visitor shape.
    pub(crate) fn overlap_sensor(
        &self,
        sensor_shape: &Shape,
        sensor_transform: Transform,
        visitor_shape: &Shape,
        visitor_transform: Transform,
    ) -> bool {
        let proxy_buffer = make_shape_proxy(visitor_shape);

        // Get the visitor shape in the frame of the sensor
        let relative_transform = inv_mul_transforms(sensor_transform, visitor_transform);

        let proxy = proxy_buffer.proxy();

        let mut local_points = vec![Vec3::ZERO; b3::MAX_SHAPE_CAST_POINTS as usize];
        let count = min_int(proxy.count, b3::MAX_SHAPE_CAST_POINTS);
        for i in 0..count as usize {
            local_points[i] = transform_point(relative_transform, proxy.points()[i]);
        }

        // SAFETY: `local_points` is fixed-size storage owned by this scope and never resized after
        // the pointer is taken, so it outlives every proxy dereference below. This is the guarantee
        // Swift's `withUnsafeMutableBufferPointer` closure gave `buffer.baseAddress`.
        let local_proxy = ShapeProxy::new(Some(local_points.as_ptr()), count, proxy.radius);
        let identity = Transform::new(Vec3::ZERO, Quat::IDENTITY);

        match &sensor_shape.geometry {
            ShapeGeometry::Sphere(sphere) => overlap_sphere(sphere, identity, &local_proxy),
            ShapeGeometry::Capsule(capsule) => overlap_capsule(capsule, identity, &local_proxy),
            ShapeGeometry::Hull(hull) => overlap_hull(hull, identity, &local_proxy),
            ShapeGeometry::Compound(compound) => overlap_compound(compound, identity, local_proxy),
            ShapeGeometry::HeightField(field) => {
                overlap_height_field(field, identity, local_proxy)
            }
            ShapeGeometry::Mesh(mesh) => overlap_mesh(mesh, identity, local_proxy),
        }
    }

    /// Query callback for one sensor.
    ///
    /// Swift took `sensor` as `inout` and `sensorShape` by value; the reference parameters below are
    /// the equivalent of that borrow.
    fn sensor_query_callback(
        &self,
        shape_id: i32,
        sensor: &mut Sensor,
        sensor_shape: &Shape,
        transform: Transform,
    ) -> bool {
        if shape_id == sensor_shape.id {
            return true;
        }

        let other_shape = &self.shapes[shape_id as usize];

        // Visitors must be convex.
        if !is_convex(other_shape.type_) {
            return true;
        }

        // Are sensor events enabled on the other shape?
        if (other_shape.flags & ShapeFlags::ENABLE_SENSOR_EVENTS.0) == 0 {
            return true;
        }

        // Skip shapes on the same body
        if other_shape.body_id == sensor_shape.body_id {
            return true;
        }

        // Check filter
        if !should_shapes_collide(&sensor_shape.filter, &other_shape.filter) {
            return true;
        }

        let other_transform =
            to_relative_transform(self.body_transform_quick(other_shape.body_id), Pos::ZERO);

        if !self.overlap_sensor(sensor_shape, transform, other_shape, other_transform) {
            return true;
        }

        // Record the overlap
        let mut visitor = SensorVisitor::new();
        visitor.shape_id = shape_id;
        visitor.generation = other_shape.generation;
        sensor.overlaps2.push(visitor);

        true
    }

    /// Update all sensors and publish begin/end touch events.
    pub(crate) fn overlap_sensors(&mut self) {
        let sensor_count = self.sensors.len();
        if sensor_count == 0 {
            return;
        }

        let bit_capacity = max_int(sensor_count as i32, 8) as u32;
        let mut event_bits = BitSet::with_bit_capacity(bit_capacity);
        event_bits.set_bit_count_and_clear(bit_capacity);

        for sensor_index in 0..sensor_count {
            // Swift's `var sensor = sensors[i]` took a CoW snapshot and wrote it back at the end of
            // each path; the clone plus the matching stores below keep that.
            let mut sensor = self.sensors[sensor_index].clone();
            // Swift copied the shape value out of `shapes`, which was a free CoW read. Nothing here
            // writes to `shapes`, so the shape is borrowed instead of deep-copied.
            let sensor_shape = &self.shapes[sensor.shape_id as usize];

            // Swap overlap arrays
            std::mem::swap(&mut sensor.overlaps1, &mut sensor.overlaps2);
            sensor.overlaps2.clear();

            // Append sensor hits
            sensor.overlaps2.extend_from_slice(&sensor.hits);

            // Clear the hits
            sensor.hits.clear();

            let body = self.bodies[sensor_shape.body_id as usize].clone();
            if body.set_index == SetType::DisabledSet.raw_value()
                || (sensor_shape.flags & ShapeFlags::ENABLE_SENSOR_EVENTS.0) == 0
            {
                if !sensor.overlaps1.is_empty() {
                    // This sensor is dropping all overlaps because it has been disabled.
                    event_bits.set_bit(sensor_index as u32);
                }
                self.sensors[sensor_index] = sensor;
                continue;
            }

            let transform =
                to_relative_transform(self.body_transform_quick(sensor_shape.body_id), Pos::ZERO);

            let query_bounds = sensor_shape.aabb;
            for tree_index in 0..BodyType::COUNT as usize {
                // Swift ran the callback from inside the tree traversal because both `self` and the
                // tree were reference-aliasable objects. Rust cannot express that aliasing, so each
                // tree first collects the visited shape indices in exactly the order the query
                // visits them, and only then runs the callback. `sensor_query_callback` never
                // touches the broad-phase trees, so the visited set and its order are unchanged,
                // and `overlaps2` is appended in the same interleaving as Swift because the whole
                // traversal and callback pass stays inside one tree iteration.
                let mut candidate_shape_ids: Vec<i32> = Vec::new();
                {
                    let tree = &self.broad_phase.trees[tree_index];
                    tree.query(
                        query_bounds,
                        sensor_shape.filter.mask_bits,
                        false,
                        |_proxy_id, user_data| {
                            let other_shape_id = user_data as i64 as i32;
                            candidate_shape_ids.push(other_shape_id);
                            // Swift's callback always returned true, so the traversal never stops.
                            true
                        },
                    );
                }

                for other_shape_id in candidate_shape_ids {
                    // The callback's result drove the traversal, which has already finished here.
                    self.sensor_query_callback(other_shape_id, &mut sensor, sensor_shape, transform);
                }
            }

            // Sort the overlaps to enable finding begin and end events.
            // Swift's `sort` was unstable; the stable sort keeps ties (one shape id reached by both a
            // CCD hit and a query overlap, with different generations) resolved by insertion order,
            // which is reproducible on every platform.
            sensor.overlaps2.sort_by(|a, b| a.shape_id.cmp(&b.shape_id));

            // Remove duplicates from overlaps2.
            let mut unique_count = 0usize;
            for i in 0..sensor.overlaps2.len() {
                if unique_count == 0
                    || sensor.overlaps2[i].shape_id != sensor.overlaps2[unique_count - 1].shape_id
                {
                    let kept = sensor.overlaps2[i];
                    sensor.overlaps2[unique_count] = kept;
                    unique_count += 1;
                }
            }
            // Swift's `removeLast(count - uniqueCount)` truncated to `uniqueCount` entries.
            sensor.overlaps2.truncate(unique_count);

            let count1 = sensor.overlaps1.len();
            let count2 = sensor.overlaps2.len();
            if count1 != count2 {
                event_bits.set_bit(sensor_index as u32);
            } else {
                for i in 0..count1 {
                    if sensor.overlaps1[i].shape_id != sensor.overlaps2[i].shape_id
                        || sensor.overlaps1[i].generation != sensor.overlaps2[i].generation
                    {
                        event_bits.set_bit(sensor_index as u32);
                        break;
                    }
                }
            }

            self.sensors[sensor_index] = sensor;
        }

        // Iterate sensors bits and publish events
        for sensor_index in 0..sensor_count {
            // Swift's `for ... where eventBits.getBit(...)` guard.
            if !event_bits.get_bit(sensor_index as u32) {
                continue;
            }

            let sensor = &self.sensors[sensor_index];
            // Swift's CoW shape copy; only `generation` is read, so borrow instead of clone.
            let sensor_shape = &self.shapes[sensor.shape_id as usize];
            let sensor_id = ShapeID::new(
                sensor.shape_id + 1,
                self.world_id,
                sensor_shape.generation,
            );

            let count1 = sensor.overlaps1.len();
            let count2 = sensor.overlaps2.len();
            let refs1 = &sensor.overlaps1;
            let refs2 = &sensor.overlaps2;

            // overlaps1 can have overlaps that end
            // overlaps2 can have overlaps that begin
            let mut index1 = 0usize;
            let mut index2 = 0usize;
            while index1 < count1 && index2 < count2 {
                let r1 = refs1[index1];
                let r2 = refs2[index2];
                if r1.shape_id == r2.shape_id {
                    if r1.generation < r2.generation {
                        // end
                        let mut event = SensorEndTouchEvent::default();
                        event.sensor_shape_id = sensor_id;
                        event.visitor_shape_id =
                            ShapeID::new(r1.shape_id + 1, self.world_id, r1.generation);
                        self.sensor_end_events.pending.push(event);
                        index1 += 1;
                    } else if r1.generation > r2.generation {
                        // begin
                        let mut event = SensorBeginTouchEvent::default();
                        event.sensor_shape_id = sensor_id;
                        event.visitor_shape_id =
                            ShapeID::new(r2.shape_id + 1, self.world_id, r2.generation);
                        self.sensor_begin_events.push(event);
                        index2 += 1;
                    } else {
                        // persisted
                        index1 += 1;
                        index2 += 1;
                    }
                } else if r1.shape_id < r2.shape_id {
                    // end
                    let mut event = SensorEndTouchEvent::default();
                    event.sensor_shape_id = sensor_id;
                    event.visitor_shape_id =
                        ShapeID::new(r1.shape_id + 1, self.world_id, r1.generation);
                    self.sensor_end_events.pending.push(event);
                    index1 += 1;
                } else {
                    // begin
                    let mut event = SensorBeginTouchEvent::default();
                    event.sensor_shape_id = sensor_id;
                    event.visitor_shape_id =
                        ShapeID::new(r2.shape_id + 1, self.world_id, r2.generation);
                    self.sensor_begin_events.push(event);
                    index2 += 1;
                }
            }

            while index1 < count1 {
                let r1 = refs1[index1];
                let mut event = SensorEndTouchEvent::default();
                event.sensor_shape_id = sensor_id;
                event.visitor_shape_id =
                    ShapeID::new(r1.shape_id + 1, self.world_id, r1.generation);
                self.sensor_end_events.pending.push(event);
                index1 += 1;
            }

            while index2 < count2 {
                let r2 = refs2[index2];
                let mut event = SensorBeginTouchEvent::default();
                event.sensor_shape_id = sensor_id;
                event.visitor_shape_id =
                    ShapeID::new(r2.shape_id + 1, self.world_id, r2.generation);
                self.sensor_begin_events.push(event);
                index2 += 1;
            }
        }

        // Swift's `defer { eventBits.destroy() }` ran at scope exit; this is the only exit after the
        // bit set was created.
        event_bits.destroy();
    }

    /// Create sensor storage for a sensor shape.
    pub(crate) fn create_sensor(&mut self, shape_id: i32) {
        let mut sensor = Sensor::new();
        sensor.shape_id = shape_id;
        self.sensors.push(sensor);

        let mut shape = self.shapes[shape_id as usize].clone();
        shape.sensor_index = self.sensors.len() as i32 - 1;
        self.shapes[shape_id as usize] = shape;
    }

    /// Destroy the sensor for a shape, emitting end touch events for its overlaps.
    pub(crate) fn destroy_sensor(&mut self, shape_id: i32) {
        let mut shape = self.shapes[shape_id as usize].clone();
        if shape.sensor_index == b3::NULL_INDEX {
            return;
        }

        let sensor_index = shape.sensor_index as usize;
        let sensor = &self.sensors[sensor_index];

        for reference in &sensor.overlaps2 {
            let mut event = SensorEndTouchEvent::default();
            event.sensor_shape_id = ShapeID::new(shape.id + 1, self.world_id, shape.generation);
            event.visitor_shape_id = ShapeID::new(
                reference.shape_id + 1,
                self.world_id,
                reference.generation,
            );
            self.sensor_end_events.pending.push(event);
        }

        // Swift popped the last sensor and stored it in the vacated slot, skipping the self-assign
        // when the destroyed sensor was already last.
        if let Some(last) = self.sensors.pop() {
            if sensor_index < self.sensors.len() {
                self.sensors[sensor_index] = last;
            }
        }

        // Fixup moved sensor
        if sensor_index < self.sensors.len() {
            let moved_shape_id = self.sensors[sensor_index].shape_id;
            let mut other_sensor_shape = self.shapes[moved_shape_id as usize].clone();
            other_sensor_shape.sensor_index = sensor_index as i32;
            self.shapes[moved_shape_id as usize] = other_sensor_shape;
        }

        shape.sensor_index = b3::NULL_INDEX;
        self.shapes[shape_id as usize] = shape;
    }
}

impl World {
    /// Move CCD sensor hits into their sensors before the overlap pass.
    pub(crate) fn accept_sensor_hits(&mut self) {
        for hit in &self.task_contexts[0].sensor_hits {
            // Swift's CoW shape reads; nothing below writes to `shapes`, so the shapes are borrowed.
            let sensor_shape = &self.shapes[hit.sensor_id as usize];
            if sensor_shape.sensor_index == b3::NULL_INDEX {
                continue;
            }

            let visitor_shape = &self.shapes[hit.visitor_id as usize];
            let mut visitor = SensorVisitor::new();
            visitor.shape_id = hit.visitor_id;
            visitor.generation = visitor_shape.generation;
            self.sensors[sensor_shape.sensor_index as usize]
                .hits
                .push(visitor);
        }

        self.task_contexts[0].sensor_hits.clear();
    }

    /// Buffered sensor begin/end touch events from the last step.
    ///
    /// Swift returned a labelled tuple; the labels become the positional order `(begin, end)`.
    pub fn sensor_events(&self) -> (Vec<SensorBeginTouchEvent>, Vec<SensorEndTouchEvent>) {
        (
            self.sensor_begin_events.clone(),
            self.sensor_end_events.published().to_vec(),
        )
    }
}
