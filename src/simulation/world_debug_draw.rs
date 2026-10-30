//! Debug draw.
//!
//! Translated from `Sources/Natsel/Simulation/WorldDebugDraw.swift`.
//!
//! Emission order is load-bearing: debug output is compared against the Swift build, so the
//! draw order, colour selection, per-flag gating and the exact vertex/line/shape emission
//! sequence mirror the Swift source line by line.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::world_debug_draw_extras`: `World::draw_vehicles_pass` and
//!   `World::draw_soft_bodies_pass` (Swift's `WorldDebugDrawExtras.swift`).
//! - `crate::simulation::coupling_joints`: `World::draw_coupling_joint` (already landed; it
//!   dispatches to `path_joint::draw_path_joint` and the six-DOF/pulley cases exactly as
//!   Swift's `drawCouplingJoint` did).
//!
//! Deviations forced by Rust's borrow model (emission order is unchanged):
//! - Swift drew each shape inside the broad-phase query callback. A callback that mutates
//!   `self.shapes` cannot coexist with the immutable borrow of `self.broad_phase` the query
//!   holds, so each tree's proxy hits are collected during the traversal and replayed in the
//!   same hit order immediately after that tree's query returns. A shape lives in exactly one
//!   tree and is visited at most once per query, so the `user_shape == nil` factory guard sees
//!   the same state it saw in Swift, and every callback fires in the Swift sequence.
//! - Swift's `Set<Int32>` of seen bodies becomes a `Vec<i32>` that is sorted and deduplicated,
//!   matching `Set.sorted()` iteration.

use crate::base::constants as b3;
use crate::math::functions::{
    clamp_int, compute_cos_sin, dot_quat, get_twist_angle, inv_mul_quat, lerp_float,
    make_matrix_from_quat, max_float, mul_world_transforms, normalize, offset_pos, rotate_vector,
    sub_pos, transform_world_point,
};
use crate::math::types::{Pos, Quat, Vec3, WorldTransform};
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::shape_runtime::{Shape, ShapeGeometry};
use crate::simulation::world_runtime::{Body, BodyFlags, JointSim, SetType, World};
use crate::types::body_def::BodyType;
use crate::types::debug_draw::{
    DebugDraw, DebugMaterial, DebugShape, DebugShapeGeometry, HexColor, debug_colors,
    make_debug_color,
};
use crate::types::geometry::{Capsule, HullData, Mesh, Sphere};
use crate::types::ids::ShapeID;
use crate::types::joint_defs::JointType;

/// Draw-callback views of the optional `DebugDraw` entries, bound once per pass so every call
/// site reproduces Swift's `if options.drawX, let drawY = options.drawY` fall-through exactly.
type SegmentFn<'a> = &'a dyn Fn(Pos, Pos, HexColor);
type PointFn<'a> = &'a dyn Fn(Pos, f32, HexColor);
type StringFn<'a> = &'a dyn Fn(Pos, &str, HexColor);

impl World {
    /// Walk the world and emit debug draw callbacks for everything the caller asked to see.
    ///
    /// Swift defaulted `maskBits` to `.max`; ported callers pass it explicitly.
    pub fn draw(&mut self, options: &DebugDraw, mask_bits: u64) {
        // Culling query over all three trees. Each hit records its body for the per-body pass.
        let mut seen_bodies: Vec<i32> = Vec::new();

        let tree_count = self.broad_phase.trees.len();
        for tree_index in 0..tree_count {
            let mut hits: Vec<u64> = Vec::new();
            self.broad_phase.trees[tree_index].query(
                options.drawing_bounds,
                mask_bits,
                false,
                |_, data| {
                    hits.push(data);
                    true
                },
            );
            for data in hits {
                let shape = self.shapes[data as usize].clone();
                seen_bodies.push(shape.body_id);
                self.draw_shape_entry(shape, options);
            }
        }

        // Per body extras: names, mass, sleep, joints, contacts, islands.
        seen_bodies.sort_unstable();
        seen_bodies.dedup();
        for body_id in seen_bodies {
            self.draw_body_extras(body_id, options);
        }

        self.draw_joint_extras_pass(options);
        self.draw_vehicles_pass(options);
        self.draw_soft_bodies_pass(options);
    }

    // MARK: Shape

    fn draw_shape_entry(&mut self, shape: Shape, options: &DebugDraw) {
        if options.draw_shapes {
            let color = self.debug_color_for(&shape);

            if shape.user_shape.is_none() {
                if let Some(factory) = options.create_debug_shape.as_ref() {
                    let debug_shape = DebugShape::new(
                        ShapeID::new(shape.id + 1, self.world_id, shape.generation),
                        shape.type_,
                        debug_geometry_for(&shape),
                    );
                    let mut mutable = shape.clone();
                    mutable.user_shape = factory(&debug_shape);
                    self.shapes[shape.id as usize] = mutable;
                }
            }

            // Swift re-reads the (possibly updated) shape from the store before drawing.
            let cached = &self.shapes[shape.id as usize];
            let user_shape = cached.user_shape;
            let cached_body_id = cached.body_id;
            if let Some(user_shape) = user_shape {
                if let Some(draw_shape) = options.draw_shape.as_ref() {
                    let transform = self.body_transform_quick(cached_body_id);
                    draw_shape(user_shape, transform, color);
                }
            }
        }

        if options.draw_bounds_option {
            if let Some(draw_bounds) = options.draw_bounds.as_ref() {
                let bounds = self.fat_aabbs[shape.id as usize];
                draw_bounds(bounds, debug_colors::GOLD);
            }
        }
    }

    // MARK: Per body

    fn draw_body_extras(&self, body_id: i32, options: &DebugDraw) {
        let body = &self.bodies[body_id as usize];
        if body.set_index == SetType::DisabledSet.raw_value() {
            return;
        }

        let sim = self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];

        let draw_string: Option<StringFn<'_>> = options.draw_string.as_ref().map(|f| &**f);

        if options.draw_body_names {
            let name = self.names.find(body.name_id);
            if let (Some(name), Some(draw_string)) = (name, draw_string) {
                let transform = WorldTransform::new(sim.center, sim.transform.q);
                let p = transform_world_point(transform, Vec3::new(0.05, 0.05, 0.05));
                draw_string(p, name, debug_colors::WHITE);
            }
        }

        if options.draw_mass {
            let transform = WorldTransform::new(sim.center, sim.transform.q);
            if let Some(draw_transform) = options.draw_transform.as_ref() {
                draw_transform(transform);
            }
            if body.type_ == BodyType::DynamicBody {
                if let Some(draw_string) = draw_string {
                    let p = transform_world_point(transform, Vec3::new(0.05, 0.05, 0.05));
                    draw_string(p, &format!("{:.2}", body.mass), debug_colors::WHITE);
                }
            }
        }

        if options.draw_sleep {
            if let Some(draw_point) = options.draw_point.as_ref() {
                let colors = [
                    debug_colors::BLUE,
                    debug_colors::SKY_BLUE,
                    debug_colors::ORANGE,
                    debug_colors::RED,
                ];
                let mut color = debug_colors::BLACK;
                if body.sleep_threshold > 0.0 {
                    let ratio = body.sleep_velocity / body.sleep_threshold;
                    color = colors[clamp_int(ratio as i32, 0, 3) as usize];
                }
                draw_point(sim.center, 10.0, color);
                if let Some(draw_string) = draw_string {
                    let p = offset_pos(sim.center, Vec3::new(0.1, 0.1, 0.1));
                    draw_string(
                        p,
                        &format!("  {:.3}", body.sleep_velocity),
                        color,
                    );
                }
            }
        }

        if options.draw_contacts
            && body.type_ == BodyType::DynamicBody
            && body.set_index == SetType::AwakeSet.raw_value()
        {
            self.draw_body_contacts(body_id, body, options);
        }
    }

    fn draw_body_contacts(&self, _body_id: i32, body: &Body, options: &DebugDraw) {
        let axis_scale = 0.3 * b3::get_length_units_per_meter();
        let mut key = body.head_contact_key;

        let draw_segment: Option<SegmentFn<'_>> = options.draw_segment.as_ref().map(|f| &**f);
        let draw_point: Option<PointFn<'_>> = options.draw_point.as_ref().map(|f| &**f);
        let draw_string: Option<StringFn<'_>> = options.draw_string.as_ref().map(|f| &**f);

        while key != b3::NULL_INDEX {
            let contact_id = key >> 1;
            let edge_index = (key & 1) as usize;
            let contact = self.contacts[contact_id as usize];
            key = contact.edges[edge_index].next_key;

            if !(contact.set_index == SetType::AwakeSet.raw_value()
                && contact.color_index != b3::NULL_INDEX)
            {
                continue;
            }

            let sim_a = self.solver_sets
                [self.bodies[contact.edges[0].body_id as usize].set_index as usize]
                .body_sims
                [self.bodies[contact.edges[0].body_id as usize].local_index as usize];
            let sim_b = self.solver_sets
                [self.bodies[contact.edges[1].body_id as usize].set_index as usize]
                .body_sims
                [self.bodies[contact.edges[1].body_id as usize].local_index as usize];

            let manifold_view = contact.manifolds();
            for manifold_index in 0..contact.manifold_count as usize {
                let manifold = manifold_view.get(manifold_index);
                let normal = manifold.normal;

                // Average the anchors rather than the world points so the friction center stays
                // exact far from the origin.
                let contact_center = if options.draw_anchor_a {
                    sim_a.center
                } else {
                    sim_b.center
                };

                for point_index in 0..manifold.point_count as usize {
                    let mp = manifold.points[point_index];
                    let anchor = if options.draw_anchor_a {
                        mp.anchor_a
                    } else {
                        mp.anchor_b
                    };
                    let p = offset_pos(contact_center, anchor);

                    // Swift's `if a, let x = ... else if b, let y = ...` chain falls through to
                    // the next branch when the flag is set but the callback is nil; reproduce
                    // that by gating each branch on the callback's presence.
                    if options.draw_contact_normals && draw_segment.is_some() {
                        if let Some(draw_segment) = draw_segment {
                            let p2 = offset_pos(p, axis_scale * normal);
                            draw_segment(p, p2, debug_colors::LIGHT_GRAY);
                            if let Some(draw_string) = draw_string {
                                draw_string(
                                    p,
                                    &format!("   {:.2}", 100.0 * mp.separation),
                                    debug_colors::WHITE,
                                );
                            }
                        }
                    } else if options.draw_contact_forces && draw_segment.is_some() {
                        if let Some(draw_segment) = draw_segment {
                            let inv_dt = if self.inv_dt > 0.0 { self.inv_dt } else { 60.0 };
                            let force = mp.total_normal_impulse * inv_dt;
                            let p2 =
                                offset_pos(p, (options.force_scale * force) * normal);
                            draw_segment(p, p2, debug_colors::MAGENTA);
                            if let Some(draw_string) = draw_string {
                                draw_string(
                                    p,
                                    &format!("  {:.1}", force),
                                    debug_colors::WHITE,
                                );
                            }
                        }
                    } else if options.draw_contact_features && draw_string.is_some() {
                        if let Some(draw_string) = draw_string {
                            draw_string(
                                p,
                                &format!("   {:#X}", mp.feature_id),
                                debug_colors::WHITE,
                            );
                        }
                    }

                    if options.draw_graph_colors {
                        if let Some(draw_point) = draw_point {
                            let size: f32 = if contact.color_index == OVERFLOW_INDEX {
                                15.0
                            } else {
                                10.0
                            };
                            let idx = contact.color_index as usize;
                            let palette = &debug_colors::GRAPH;
                            draw_point(p, size, palette[idx % palette.len()]);
                        }
                    }
                }
            }
        }
    }

    // MARK: Joints

    fn draw_joint_extras_pass(&self, options: &DebugDraw) {
        if !options.draw_joints {
            return;
        }
        let Some(draw_segment_arc) = options.draw_segment.as_ref() else {
            return;
        };
        let draw_segment: SegmentFn<'_> = &**draw_segment_arc;
        let draw_point: Option<PointFn<'_>> = options.draw_point.as_ref().map(|f| &**f);
        let scale = max_float(0.0001, options.joint_scale);

        for index in 0..self.joints.len() as i32 {
            let joint = &self.joints[index as usize];
            if joint.joint_id == b3::NULL_INDEX {
                continue;
            }

            let body_a = &self.bodies[joint.edges[0].body_id as usize];
            let body_b = &self.bodies[joint.edges[1].body_id as usize];
            if body_a.set_index == SetType::DisabledSet.raw_value()
                || body_b.set_index == SetType::DisabledSet.raw_value()
            {
                continue;
            }

            let sim = self.read_joint_sim(index);
            let xf_a = self.body_transform_quick(joint.edges[0].body_id);
            let xf_b = self.body_transform_quick(joint.edges[1].body_id);
            let p_a = transform_world_point(xf_a, sim.local_frame_a.p);
            let p_b = transform_world_point(xf_b, sim.local_frame_b.p);

            match joint.type_ {
                JointType::DistanceJoint => {
                    self.draw_distance_joint(sim, xf_a, xf_b, p_a, p_b, draw_segment, draw_point);
                }

                JointType::RevoluteJoint => self.draw_revolute_joint(
                    sim,
                    xf_a,
                    xf_b,
                    scale,
                    draw_segment,
                    draw_point,
                ),

                JointType::PrismaticJoint => self.draw_prismatic_joint(
                    sim,
                    xf_a,
                    xf_b,
                    scale,
                    draw_segment,
                    draw_point,
                ),

                JointType::SphericalJoint => self.draw_spherical_joint(
                    sim,
                    xf_a,
                    xf_b,
                    scale,
                    draw_segment,
                    draw_point,
                ),

                JointType::WheelJoint => {
                    self.draw_wheel_joint(sim, xf_a, xf_b, scale, draw_segment, draw_point)
                }

                JointType::WeldJoint => {
                    let frame_a = mul_world_transforms(xf_a, sim.local_frame_a);
                    let frame_b = mul_world_transforms(xf_b, sim.local_frame_b);
                    let extents = Vec3::new(0.1 * scale, 0.05 * scale, 0.025 * scale);
                    if let Some(draw_box) = options.draw_box.as_ref() {
                        draw_box(extents, frame_a, debug_colors::DARK_ORANGE);
                    }
                    if let Some(draw_box) = options.draw_box.as_ref() {
                        draw_box(extents, frame_b, debug_colors::DARK_CYAN);
                    }
                }

                JointType::ParallelJoint => {
                    let length = 0.1 * scale;
                    let frame_a = mul_world_transforms(xf_a, sim.local_frame_a);
                    draw_segment(
                        frame_a.p,
                        offset_pos(
                            frame_a.p,
                            length * rotate_vector(frame_a.q, Vec3::AXIS_Z),
                        ),
                        debug_colors::GREEN,
                    );
                    let frame_b = mul_world_transforms(xf_b, sim.local_frame_b);
                    draw_segment(
                        frame_b.p,
                        offset_pos(
                            frame_b.p,
                            length * rotate_vector(frame_b.q, Vec3::AXIS_Z),
                        ),
                        debug_colors::BLUE,
                    );
                }

                JointType::MotorJoint => {
                    draw_segment(p_a, p_b, debug_colors::PLUM);
                    if let Some(draw_point) = draw_point {
                        draw_point(p_a, 8.0, debug_colors::LIME);
                    }
                    if let Some(draw_point) = draw_point {
                        draw_point(p_b, 8.0, debug_colors::PLUM);
                    }
                }

                JointType::FilterJoint => {
                    draw_segment(p_a, p_b, debug_colors::GOLD);
                }

                JointType::GearJoint
                | JointType::RackAndPinionJoint
                | JointType::PulleyJoint
                | JointType::SixDOFJoint
                | JointType::PathJoint => {
                    self.draw_coupling_joint(
                        sim,
                        xf_a,
                        xf_b,
                        p_a,
                        p_b,
                        scale,
                        draw_segment,
                        draw_point,
                    );
                }
            }

            if options.draw_graph_colors {
                if let Some(draw_point) = draw_point {
                    let idx = if joint.color_index >= 0 {
                        joint.color_index
                    } else {
                        OVERFLOW_INDEX
                    };
                    let palette = &debug_colors::GRAPH;
                    draw_point(p_a, 6.0, palette[idx as usize % palette.len()]);
                }
            }
        }
    }

    // MARK: Per type

    fn draw_distance_joint(
        &self,
        sim: JointSim,
        _xf_a: WorldTransform,
        _xf_b: WorldTransform,
        p_a: Pos,
        p_b: Pos,
        draw_segment: SegmentFn<'_>,
        draw_point: Option<PointFn<'_>>,
    ) {
        let joint = sim.distance();
        let axis = normalize(sub_pos(p_b, p_a));

        if joint.min_length < joint.max_length && joint.enable_limit {
            let p_min = offset_pos(p_a, joint.min_length * axis);
            let p_max = offset_pos(p_a, joint.max_length * axis);

            if joint.min_length > b3::linear_slop() {
                if let Some(draw_point) = draw_point {
                    draw_point(p_min, 6.0, debug_colors::LIGHT_GREEN);
                }
            }
            if joint.max_length < b3::huge() {
                if let Some(draw_point) = draw_point {
                    draw_point(p_max, 6.0, debug_colors::RED);
                }
            }
            if joint.min_length > b3::linear_slop() && joint.max_length < b3::huge() {
                draw_segment(p_min, p_max, debug_colors::GRAY);
            }
        }

        draw_segment(p_a, p_b, debug_colors::WHITE);
        if let Some(draw_point) = draw_point {
            draw_point(p_a, 4.0, debug_colors::WHITE);
        }
        if let Some(draw_point) = draw_point {
            draw_point(p_b, 4.0, debug_colors::WHITE);
        }

        if joint.hertz > 0.0 && joint.enable_spring {
            let p_rest = offset_pos(p_a, joint.length * axis);
            if let Some(draw_point) = draw_point {
                draw_point(p_rest, 4.0, debug_colors::BLUE);
            }
        }
    }

    /// Draw the local frame axes of a joint at world `frame`.
    fn draw_frame_axes(&self, frame: WorldTransform, length: f32, draw_segment: SegmentFn<'_>) {
        draw_segment(
            frame.p,
            offset_pos(frame.p, length * rotate_vector(frame.q, Vec3::AXIS_X)),
            debug_colors::RED,
        );
        draw_segment(
            frame.p,
            offset_pos(frame.p, length * rotate_vector(frame.q, Vec3::AXIS_Y)),
            debug_colors::GREEN,
        );
        draw_segment(
            frame.p,
            offset_pos(frame.p, length * rotate_vector(frame.q, Vec3::AXIS_Z)),
            debug_colors::BLUE,
        );
    }

    /// Draw an angular wedge from lower to upper about the frame's z axis, plus a marker at the
    /// current twist.
    fn draw_angular_wedge(
        &self,
        frame_a: WorldTransform,
        frame_b: WorldTransform,
        lower: f32,
        upper: f32,
        radius: f32,
        draw_segment: SegmentFn<'_>,
    ) {
        let slice_count = 16usize;

        let quat_a = frame_a.q;
        let mut quat_b = frame_b.q;
        if dot_quat(quat_a, quat_b) < 0.0 {
            // This keeps the twist angle in the range [-pi, pi].
            quat_b = Quat::new(-quat_b.v, -quat_b.s);
        }
        let rel_q = inv_mul_quat(quat_a, quat_b);

        let mut previous = Vec3::default();
        for index in 0..=slice_count {
            let t = index as f32 / slice_count as f32;
            let alpha = lerp_float(lower, upper, t);
            let cs = compute_cos_sin(alpha);
            let vertex = Vec3::new(radius * cs.cosine, radius * cs.sine, 0.0);

            if index == 0 {
                draw_segment(
                    frame_a.p,
                    transform_world_point(frame_a, vertex),
                    debug_colors::CYAN,
                );
            }
            if index == slice_count {
                draw_segment(
                    transform_world_point(frame_a, vertex),
                    frame_a.p,
                    debug_colors::CYAN,
                );
            }
            if index > 0 {
                draw_segment(
                    transform_world_point(frame_a, previous),
                    transform_world_point(frame_a, vertex),
                    debug_colors::CYAN,
                );
            }
            previous = vertex;
        }

        // Marker at the current twist.
        let twist = get_twist_angle(rel_q);
        let cs = compute_cos_sin(twist);
        let mark = Vec3::new(radius * cs.cosine, radius * cs.sine, 0.0);
        draw_segment(
            frame_a.p,
            transform_world_point(frame_a, mark),
            debug_colors::YELLOW,
        );
    }

    fn draw_revolute_joint(
        &self,
        sim: JointSim,
        xf_a: WorldTransform,
        xf_b: WorldTransform,
        scale: f32,
        draw_segment: SegmentFn<'_>,
        _draw_point: Option<PointFn<'_>>,
    ) {
        let joint = sim.revolute();
        let frame_a = mul_world_transforms(xf_a, sim.local_frame_a);
        let frame_b = mul_world_transforms(xf_b, sim.local_frame_b);

        self.draw_frame_axes(frame_a, 0.1 * scale, draw_segment);

        if joint.enable_limit {
            self.draw_angular_wedge(
                frame_a,
                frame_b,
                joint.lower_angle,
                joint.upper_angle,
                0.2 * scale,
                draw_segment,
            );
        }
    }

    fn draw_prismatic_joint(
        &self,
        sim: JointSim,
        xf_a: WorldTransform,
        xf_b: WorldTransform,
        scale: f32,
        draw_segment: SegmentFn<'_>,
        draw_point: Option<PointFn<'_>>,
    ) {
        let joint = sim.prismatic();
        let frame_a = mul_world_transforms(xf_a, sim.local_frame_a);
        let frame_b = mul_world_transforms(xf_b, sim.local_frame_b);

        let matrix = make_matrix_from_quat(frame_a.q);
        let axis = matrix.cx;
        let s = 0.2 * scale;

        draw_segment(
            frame_a.p,
            offset_pos(frame_a.p, s * matrix.cy),
            debug_colors::GREEN,
        );
        draw_segment(
            frame_a.p,
            offset_pos(frame_a.p, s * matrix.cz),
            debug_colors::BLUE,
        );

        if joint.enable_limit {
            let p1 = offset_pos(frame_a.p, joint.lower_translation * axis);
            let p2 = offset_pos(frame_a.p, joint.upper_translation * axis);
            draw_segment(p1, p2, debug_colors::ORANGE);
            if let Some(draw_point) = draw_point {
                draw_point(p1, 10.0, debug_colors::GREEN);
            }
            if let Some(draw_point) = draw_point {
                draw_point(p2, 10.0, debug_colors::RED);
            }
        } else {
            let p1 = offset_pos(frame_a.p, -0.5 * scale * axis);
            let p2 = offset_pos(frame_a.p, 0.5 * scale * axis);
            draw_segment(p1, p2, debug_colors::ORANGE);
        }

        if let Some(draw_point) = draw_point {
            draw_point(frame_b.p, 8.0, debug_colors::VIOLET);
        }
    }

    fn draw_spherical_joint(
        &self,
        sim: JointSim,
        xf_a: WorldTransform,
        xf_b: WorldTransform,
        scale: f32,
        draw_segment: SegmentFn<'_>,
        _draw_point: Option<PointFn<'_>>,
    ) {
        let joint = sim.spherical();
        let frame_a = mul_world_transforms(xf_a, sim.local_frame_a);
        let frame_b = mul_world_transforms(xf_b, sim.local_frame_b);

        self.draw_frame_axes(frame_a, 0.1 * scale, draw_segment);

        // The cone axis of body B.
        draw_segment(
            frame_b.p,
            offset_pos(
                frame_b.p,
                0.2 * scale * rotate_vector(frame_b.q, Vec3::AXIS_Z),
            ),
            debug_colors::ORANGE,
        );

        let slice_count = 16usize;

        if joint.enable_twist_limit {
            self.draw_angular_wedge(
                frame_a,
                frame_b,
                joint.lower_twist_angle,
                joint.upper_twist_angle,
                0.1 * scale,
                draw_segment,
            );
        }

        if joint.enable_cone_limit {
            let radius = 0.1 * scale;
            let cone = compute_cos_sin(joint.cone_angle);
            let cone_radius = radius * cone.sine;
            let cone_height = radius * cone.cosine;

            let mut previous = Vec3::default();
            for index in 0..=slice_count {
                let phi =
                    2.0 * index as f32 / slice_count as f32 * b3::PI;
                let pcs = compute_cos_sin(phi);
                let vertex = Vec3::new(cone_radius * pcs.cosine, cone_radius * pcs.sine, cone_height);

                draw_segment(
                    frame_a.p,
                    transform_world_point(frame_a, vertex),
                    debug_colors::CYAN,
                );
                if index > 0 {
                    draw_segment(
                        transform_world_point(frame_a, previous),
                        transform_world_point(frame_a, vertex),
                        debug_colors::CYAN,
                    );
                }
                previous = vertex;
            }
        }
    }

    fn draw_wheel_joint(
        &self,
        sim: JointSim,
        xf_a: WorldTransform,
        xf_b: WorldTransform,
        scale: f32,
        draw_segment: SegmentFn<'_>,
        draw_point: Option<PointFn<'_>>,
    ) {
        let joint = sim.wheel();
        let frame_a = mul_world_transforms(xf_a, sim.local_frame_a);
        let frame_b = mul_world_transforms(xf_b, sim.local_frame_b);

        let matrix_a = make_matrix_from_quat(frame_a.q);
        let matrix_b = make_matrix_from_quat(frame_b.q);

        draw_segment(frame_a.p, frame_b.p, debug_colors::BLUE);

        if joint.enable_suspension_limit {
            let lower = offset_pos(frame_a.p, joint.lower_suspension_limit * matrix_a.cx);
            let upper = offset_pos(frame_a.p, joint.upper_suspension_limit * matrix_a.cx);
            let perp = matrix_a.cy;
            draw_segment(lower, upper, debug_colors::GRAY);
            draw_segment(
                offset_pos(lower, -0.1 * scale * perp),
                offset_pos(lower, 0.1 * scale * perp),
                debug_colors::GREEN,
            );
            draw_segment(
                offset_pos(upper, -0.1 * scale * perp),
                offset_pos(upper, 0.1 * scale * perp),
                debug_colors::RED,
            );
        } else {
            draw_segment(
                offset_pos(frame_a.p, -1.0 * scale * matrix_a.cx),
                offset_pos(frame_a.p, 1.0 * scale * matrix_a.cx),
                debug_colors::GRAY,
            );
        }

        if joint.enable_steering && joint.enable_steering_limit {
            let frame = WorldTransform::new(frame_b.p, frame_a.q);
            let radius = 0.5 * scale;
            let slice_count = 16usize;
            let lower = joint.lower_steering_limit;
            let upper = joint.upper_steering_limit;

            let mut cs = compute_cos_sin(lower);
            let mut previous = transform_world_point(
                frame,
                Vec3::new(0.0, -radius * cs.sine, radius * cs.cosine),
            );

            for index in 0..slice_count {
                let t = (index as f32 + 1.0) / slice_count as f32;
                let phi = lerp_float(lower, upper, t);
                cs = compute_cos_sin(phi);
                let vertex = transform_world_point(
                    frame,
                    Vec3::new(0.0, -radius * cs.sine, radius * cs.cosine),
                );

                if index == 0 {
                    draw_segment(frame.p, previous, debug_colors::CYAN);
                }
                if index == slice_count - 1 {
                    draw_segment(vertex, frame.p, debug_colors::CYAN);
                }
                draw_segment(previous, vertex, debug_colors::CYAN);
                previous = vertex;
            }
        }

        draw_segment(
            offset_pos(frame_b.p, -0.5 * scale * matrix_b.cz),
            offset_pos(frame_b.p, 0.5 * scale * matrix_b.cz),
            debug_colors::MAGENTA,
        );

        if let Some(draw_point) = draw_point {
            draw_point(frame_a.p, 5.0, debug_colors::GRAY);
        }
        if let Some(draw_point) = draw_point {
            draw_point(frame_b.p, 5.0, debug_colors::DIM_GRAY);
        }
    }
}

/// Swift's `debugGeometry(for:)`.
fn debug_geometry_for(shape: &Shape) -> DebugShapeGeometry {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            // Swift heap-allocated one capsule and handed the renderer an owned pointer that is
            // never freed; `Box::into_raw` reproduces that exactly (one boxed copy, leaked so
            // the pointer stays valid across frames just like the Swift allocation).
            let allocated = Box::into_raw(Box::new(*capsule));
            DebugShapeGeometry::Capsule(allocated as *const Capsule)
        }
        ShapeGeometry::Compound(compound) => DebugShapeGeometry::Compound(compound.clone()),
        ShapeGeometry::HeightField(field) => DebugShapeGeometry::HeightField(field.clone()),
        ShapeGeometry::Hull(hull) => {
            // Hull is a view; expose its header for renderers that need the shape data.
            // `Hull::header` yields a reference into the hull's own allocation (the header sits
            // at its front), so taking its address is exactly the Swift
            // `UnsafePointer(hull.header)`; the pointer lives as long as the shape's geometry.
            DebugShapeGeometry::Hull(hull.header() as *const HullData)
        }
        ShapeGeometry::Mesh(mesh) => {
            let allocated = Box::into_raw(Box::new(mesh.clone()));
            DebugShapeGeometry::Mesh(allocated as *const Mesh)
        }
        ShapeGeometry::Sphere(sphere) => {
            let allocated = Box::into_raw(Box::new(*sphere));
            DebugShapeGeometry::Sphere(allocated as *const Sphere)
        }
    }
}

/// Swift's `debugColor(for:)`. Hue carries the body state, material carries its energy.
/// Diagnostic states keep a saturated hue so they pop.
impl World {
    fn debug_color_for(&self, shape: &Shape) -> HexColor {
        let material = shape.material(0);
        if material.custom_color != 0 {
            // May already carry a packed material preset, pass through unchanged.
            return material.custom_color;
        }

        let body = &self.bodies[shape.body_id as usize];
        let sim =
            self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];

        let rgb: HexColor;
        let mut preset = DebugMaterial::Default;

        if body.type_ == BodyType::DynamicBody && body.mass == 0.0 {
            rgb = debug_colors::RED;
        } else if body.set_index == SetType::DisabledSet.raw_value() {
            rgb = debug_colors::SLATE_GRAY;
        } else if shape.sensor_index != b3::NULL_INDEX {
            rgb = debug_colors::WHEAT;
        } else if (sim.flags & BodyFlags::HAD_TIME_OF_IMPACT.0) != 0 {
            rgb = debug_colors::LIME;
        } else if (sim.flags & BodyFlags::IS_FAST.0) != 0
            && body.set_index == SetType::AwakeSet.raw_value()
        {
            rgb = debug_colors::TURQUOISE;
        } else if (body.flags & BodyFlags::IS_SPEED_CAPPED.0) != 0 {
            rgb = debug_colors::YELLOW;
        } else if body.type_ == BodyType::StaticBody {
            rgb = debug_colors::DARK_GRAY;
            preset = DebugMaterial::Matte;
        } else if body.type_ == BodyType::KinematicBody {
            if body.set_index == SetType::AwakeSet.raw_value() {
                rgb = debug_colors::STEEL_BLUE;
                preset = DebugMaterial::Metallic;
            } else {
                rgb = debug_colors::LIGHT_STEEL_BLUE;
                preset = DebugMaterial::Matte;
            }
        } else if body.set_index == SetType::AwakeSet.raw_value() {
            rgb = debug_colors::TAN;
            preset = DebugMaterial::Soft;
        } else {
            rgb = debug_colors::LIGHT_SLATE_GRAY;
            preset = DebugMaterial::Dead;
        }

        make_debug_color(rgb, preset)
    }
}
