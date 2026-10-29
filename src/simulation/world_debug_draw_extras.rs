//! Debug draw for vehicles and soft bodies.
//!
//! Translated from `Sources/Natsel/Simulation/WorldDebugDrawExtras.swift`.
//!
//! Emission order mirrors the Swift source exactly: per vehicle, chassis attachment, rim
//! segments, spoke, then the contact scales; per soft body, edges first, then pinned/contact
//! vertices.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::vehicle::vehicle_runtime::VehicleRecord`: fields `id: i32`, `generation: u16`,
//!   `body_id: BodyID`, `wheels: Vec<VehicleWheel>`, where each wheel carries `settings`
//!   (`WheelParameters` with `position: Vec3`, `radius: f32`), `has_contact: bool`,
//!   `contact_position: Pos`, `contact_normal: Vec3`, `longitudinal: Vec3`.
//! - `crate::vehicle::vehicle_types::VehicleID`: `VehicleID::new(index1, world0, generation)`.
//! - `World::vehicle_wheel_transform(&self, vehicle_id: VehicleID, wheel: i32, wheel_right:
//!   Vec3, wheel_up: Vec3) -> Option<WorldTransform>` (from `VehicleRuntime.swift`). Swift
//!   defaulted `wheelRight`/`wheelUp` to `(1, 0, 0)` / `(0, 1, 0)`; following the
//!   `world_mover.rs` precedent for defaulted Swift arguments, this pass passes those default
//!   axes explicitly.
//! - `crate::soft_body::soft_body_runtime::SoftBodyRecord`: fields `id: i32`, `origin: Pos`,
//!   `bounds: AABB`, `awake: bool`, `positions: Vec<Vec3>`, `inv_masses: Vec<f32>`,
//!   `has_contact: Vec<bool>`, `settings.edges: Vec<SoftBodyEdge>` with `edge.vertex.x` /
//!   `edge.vertex.y` carrying the Swift `SIMD2<UInt32>` pair.
//!
//! Swift wrapped the soft-body vertex loop in `positions.withUnsafeBufferPointer { x in ... }`;
//! the Rust record stores a plain owned buffer, so the same `x` view is bound once before the
//! loops and iterated in the same index order.

use crate::base::constants as b3;
use crate::math::functions::{
    aabb_overlaps, compute_cos_sin, offset_aabb, offset_pos, pos_mul_add, rotate_vector,
    transform_world_point,
};
use crate::math::types::{Pos, Vec3};
use crate::simulation::world_runtime::World;
use crate::types::debug_draw::{DebugDraw, HexColor, debug_colors};
use crate::vehicle::vehicle_types::VehicleID;

impl World {
    pub(crate) fn draw_vehicles_pass(&self, options: &DebugDraw) {
        let draw_segment: &dyn Fn(Pos, Pos, HexColor) = match options.draw_segment.as_ref() {
            Some(f) => &**f,
            None => return,
        };
        if !options.draw_vehicles || self.vehicle_id_pool.id_count() <= 0 {
            return;
        }
        let draw_point: Option<&dyn Fn(Pos, f32, HexColor)> =
            options.draw_point.as_ref().map(|f| &**f);

        for record in &self.vehicles {
            if record.id == b3::NULL_INDEX || !self.is_valid_body(record.body_id) {
                continue;
            }
            let id = VehicleID::new(record.id + 1, self.world_id, record.generation);
            let chassis = self.body_transform(record.body_id);
            for (w, wheel) in record.wheels.iter().enumerate() {
                let s = &wheel.settings;
                let attachment = transform_world_point(chassis, s.position);
                let Some(transform) =
                    self.vehicle_wheel_transform(id, w as i32, Vec3::AXIS_X, Vec3::AXIS_Y)
                else {
                    continue;
                };
                let center = transform.p;
                draw_segment(attachment, center, debug_colors::YELLOW);

                // Rim in the wheel plane. The local x axis of the wheel transform is the axle.
                let up = rotate_vector(transform.q, Vec3::new(0.0, 1.0, 0.0));
                let forward = rotate_vector(transform.q, Vec3::new(0.0, 0.0, 1.0));
                let segments = 16usize;
                let mut previous = pos_mul_add(center, s.radius, up);
                for k in 1..=segments {
                    let cs =
                        compute_cos_sin(b3::TWO_PI * k as f32 / segments as f32);
                    let point = pos_mul_add(
                        pos_mul_add(center, s.radius * cs.cosine, up),
                        s.radius * cs.sine,
                        forward,
                    );
                    draw_segment(
                        previous,
                        point,
                        if wheel.has_contact {
                            debug_colors::GREEN
                        } else {
                            debug_colors::RED
                        },
                    );
                    previous = point;
                }
                // Spoke shows the rotation.
                draw_segment(
                    center,
                    pos_mul_add(center, s.radius, up),
                    debug_colors::WHITE,
                );

                if wheel.has_contact {
                    let scale = 0.3 * options.joint_scale;
                    draw_segment(
                        wheel.contact_position,
                        pos_mul_add(wheel.contact_position, scale, wheel.contact_normal),
                        debug_colors::CYAN,
                    );
                    draw_segment(
                        wheel.contact_position,
                        pos_mul_add(wheel.contact_position, scale, wheel.longitudinal),
                        debug_colors::ORANGE,
                    );
                    if let Some(draw_point) = draw_point {
                        draw_point(wheel.contact_position, 5.0, debug_colors::CYAN);
                    }
                }
            }
        }
    }

    pub(crate) fn draw_soft_bodies_pass(&self, options: &DebugDraw) {
        let draw_segment: &dyn Fn(Pos, Pos, HexColor) = match options.draw_segment.as_ref() {
            Some(f) => &**f,
            None => return,
        };
        if !options.draw_soft_bodies || self.soft_body_id_pool.id_count() <= 0 {
            return;
        }
        for record in &self.soft_bodies {
            if record.id == b3::NULL_INDEX {
                continue;
            }
            if !aabb_overlaps(
                offset_aabb(record.bounds, record.origin),
                options.drawing_bounds,
            ) {
                continue;
            }
            let color = if record.awake {
                debug_colors::LIGHT_STEEL_BLUE
            } else {
                debug_colors::GRAY
            };
            let x = record.positions.as_slice();
            for edge in &record.settings.edges {
                draw_segment(
                    offset_pos(record.origin, x[edge.vertex.x as usize]),
                    offset_pos(record.origin, x[edge.vertex.y as usize]),
                    color,
                );
            }
            if let Some(draw_point) = options.draw_point.as_ref() {
                for i in 0..x.len() {
                    if record.inv_masses[i] == 0.0 {
                        draw_point(offset_pos(record.origin, x[i]), 5.0, debug_colors::RED);
                    } else if record.has_contact[i] {
                        draw_point(
                            offset_pos(record.origin, x[i]),
                            3.0,
                            debug_colors::ORANGE,
                        );
                    }
                }
            }
        }
    }
}
