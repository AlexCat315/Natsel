//! Ported from Tests/NatselTests/Simulation/MoverTOITests.swift.

use crate::collision::hull::{make_box_hull, make_offset_box_hull};
use crate::collision::time_of_impact::time_of_impact;
use crate::math::functions::{
    abs_float, is_normalized, length, make_quat_from_axis_angle, mul_add, mul_quat, offset_pos,
    sub_pos, to_relative_transform,
};
use crate::math::types::{Pos, Quat, Vec3, WorldTransform};
use crate::simulation::World;
use crate::types::body_def::BodyDef;
use crate::types::geometry::{Capsule, Sphere};
use crate::types::ids::BodyID;
use crate::types::queries::{QueryFilter, ShapeProxy, TOIInput, TOIOutput};
use crate::types::shape_def::ShapeDef;
use crate::base::constants as b3;

fn standing_mover() -> Capsule {
    Capsule::new(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        0.25,
    )
}

fn identity_at(x: f32, y: f32, z: f32) -> WorldTransform {
    WorldTransform::new(Pos::new(x as f64, y as f64, z as f64), Quat::IDENTITY)
}

fn make_world() -> (World, BodyID) {
    let mut world = World::default();
    let def = BodyDef::default();
    let body = world.create_body(&def);
    (world, body)
}

// "a mover sweeping into a box reports the free fraction and the face normal"
#[test]
fn hits_box() {
    let (mut world, body) = make_world();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let shape = world.create_hull_shape(body, &ShapeDef::default(), &box_hull);

    // Face at x = 4.5 and radius 0.25, so 4.25 of the 10 unit sweep is free.
    let transform = identity_at(5.0, 0.0, 0.0);
    let result = world.body_time_of_impact_mover(
        body,
        Pos::ZERO,
        &standing_mover(),
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        transform,
        transform,
    );

    assert!(abs_float(result.fraction - 0.425) < 1e-2);
    assert!(is_normalized(result.normal));
    assert!(abs_float(result.normal.x + 1.0) < 1e-3);

    // The hit shape must come back identified.
    assert_eq!(result.shape_id.index1, shape.index1);
    assert_eq!(result.shape_id.generation, shape.generation);
}

// "sweeping parallel to the gap reports no hit"
#[test]
fn separated() {
    let (mut world, body) = make_world();
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    // Sweeping along +Y holds the X gap at 4.25 for the whole interval.
    let transform = identity_at(5.0, 0.0, 0.0);
    let result = world.body_time_of_impact_mover(
        body,
        Pos::ZERO,
        &standing_mover(),
        Vec3::new(0.0, 10.0, 0.0),
        &QueryFilter::default_filter(),
        transform,
        transform,
    );

    assert!(abs_float(result.fraction - 1.0) < 1e-6);
    assert_eq!(result.shape_id.index1, 0);
}

// "a mover starting inside the box is ignored"
#[test]
fn overlapped() {
    let (mut world, body) = make_world();
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    let transform = identity_at(0.0, 0.0, 0.0);
    let result = world.body_time_of_impact_mover(
        body,
        Pos::ZERO,
        &standing_mover(),
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        transform,
        transform,
    );

    // Overlap should be ignored.
    assert!(abs_float(result.fraction - 1.0) < 1e-6);
    assert_eq!(result.shape_id.index1, 0);
}

// "two shapes on the sweep line resolve to the nearer one in either list order"
#[test]
fn closest_shape() {
    let mut world = World::default();
    let def = BodyDef::default();
    let near_first_body = world.create_body(&def);
    let near_last_body = world.create_body(&def);

    let shape_def = ShapeDef::default();
    let near_sphere = Sphere::new(Vec3::new(5.0, 0.0, 0.0), 0.5);
    let far_sphere = Sphere::new(Vec3::new(9.0, 0.0, 0.0), 0.5);

    let near_first_hit = world.create_sphere_shape(near_first_body, &shape_def, near_sphere);
    let _ = world.create_sphere_shape(near_first_body, &shape_def, far_sphere);

    let _ = world.create_sphere_shape(near_last_body, &shape_def, far_sphere);
    let near_last_hit = world.create_sphere_shape(near_last_body, &shape_def, near_sphere);

    let transform = identity_at(0.0, 0.0, 0.0);
    let translation = Vec3::new(10.0, 0.0, 0.0);

    let near_first = world.body_time_of_impact_mover(
        near_first_body,
        Pos::ZERO,
        &standing_mover(),
        translation,
        &QueryFilter::default_filter(),
        transform,
        transform,
    );
    let near_last = world.body_time_of_impact_mover(
        near_last_body,
        Pos::ZERO,
        &standing_mover(),
        translation,
        &QueryFilter::default_filter(),
        transform,
        transform,
    );

    assert!(abs_float(near_first.fraction - 0.425) < 1e-2);
    assert_eq!(near_first.shape_id.index1, near_first_hit.index1);

    assert!(abs_float(near_last.fraction - near_first.fraction) < 1e-4);
    assert_eq!(near_last.shape_id.index1, near_last_hit.index1);
}

// "a shape the sweep clears does not erase an earlier hit"
#[test]
fn keeps_hit_after_miss() {
    let (mut world, body) = make_world();

    let shape_def = ShapeDef::default();
    let off_path = Sphere::new(Vec3::new(5.0, 0.0, 20.0), 0.5);
    let on_path = Sphere::new(Vec3::new(5.0, 0.0, 0.0), 0.5);
    let _ = world.create_sphere_shape(body, &shape_def, off_path);
    let on_path_shape = world.create_sphere_shape(body, &shape_def, on_path);

    let transform = identity_at(0.0, 0.0, 0.0);
    let result = world.body_time_of_impact_mover(
        body,
        Pos::ZERO,
        &standing_mover(),
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        transform,
        transform,
    );

    assert!(abs_float(result.fraction - 0.425) < 1e-2);
    assert_eq!(result.shape_id.index1, on_path_shape.index1);
}

// "offsetting the mover and body together keeps the impact pose"
#[test]
fn mover_offset() {
    let (mut world, body) = make_world();
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    let translation = Vec3::new(10.0, 0.0, 0.0);

    let at_origin = standing_mover();
    let transform_a = identity_at(5.0, 0.0, 0.0);
    let result_a = world.body_time_of_impact_mover(
        body,
        Pos::ZERO,
        &at_origin,
        translation,
        &QueryFilter::default_filter(),
        transform_a,
        transform_a,
    );

    let offset = Capsule::new(
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(2.0, 1.0, 0.0),
        0.25,
    );
    let transform_b = identity_at(7.0, 0.0, 0.0);
    let result_b = world.body_time_of_impact_mover(
        body,
        Pos::ZERO,
        &offset,
        translation,
        &QueryFilter::default_filter(),
        transform_b,
        transform_b,
    );

    assert!(abs_float(result_b.fraction - result_a.fraction) < 1e-4);
    assert!(abs_float(result_b.point.x - result_a.point.x - 2.0) < 1e-3);
}

// "a bar spinning a quarter turn reaches a mover its start pose clears"
#[test]
fn rotating_body() {
    let (mut world, body) = make_world();
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(2.0, 0.25, 0.25));

    // The query origin puts the mover at world (0,0,2.1), just inside the swept end of the bar.
    let origin = Pos::new(0.0, 0.0, 2.1);
    let transform1 = identity_at(0.0, 0.0, 0.0);
    let transform2 = WorldTransform::new(
        Pos::ZERO,
        make_quat_from_axis_angle(Vec3::AXIS_Y, 0.5 * b3::PI),
    );

    let spinning = world.body_time_of_impact_mover(
        body,
        origin,
        &standing_mover(),
        Vec3::ZERO,
        &QueryFilter::default_filter(),
        transform1,
        transform2,
    );
    assert!(spinning.fraction > 0.0 && spinning.fraction < 1.0);

    // Holding the start pose leaves the bar along X and well clear.
    let still = world.body_time_of_impact_mover(
        body,
        origin,
        &standing_mover(),
        Vec3::ZERO,
        &QueryFilter::default_filter(),
        transform1,
        transform1,
    );
    assert!(abs_float(still.fraction - 1.0) < 1e-6);
    assert_eq!(still.shape_id.index1, 0);
}

// "a huge origin does not shift the fraction"
#[test]
fn far_from_origin() {
    let (mut world, body) = make_world();
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    let origin = Pos::new(1.0e6, -2.0e6, 5.0e5);
    let transform = WorldTransform::new(
        offset_pos(origin, Vec3::new(5.0, 0.0, 0.0)),
        Quat::IDENTITY,
    );

    let result = world.body_time_of_impact_mover(
        body,
        origin,
        &standing_mover(),
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        transform,
        transform,
    );

    assert!(abs_float(result.fraction - 0.425) < 1e-2);
    assert!(abs_float(result.normal.x + 1.0) < 1e-3);
    assert!(abs_float((result.point.x - origin.x) - 4.4) < 0.5);
}

// "a zero mask rejects every category"
#[test]
fn filter() {
    let (mut world, body) = make_world();
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    let mut query_filter = QueryFilter::default_filter();
    query_filter.mask_bits = 0;

    let transform = identity_at(5.0, 0.0, 0.0);
    let result = world.body_time_of_impact_mover(
        body,
        Pos::ZERO,
        &standing_mover(),
        Vec3::new(10.0, 0.0, 0.0),
        &query_filter,
        transform,
        transform,
    );

    assert!(abs_float(result.fraction - 1.0) < 1e-6);
}

// "a buried shape is skipped but the sweep still hits the other one"
#[test]
fn overlap_skips_shape() {
    let (mut world, body) = make_world();

    let shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    // Face at x = 4.5 as in the direct hit.
    let on_path = world.create_hull_shape(
        body,
        &shape_def,
        &make_offset_box_hull(0.5, 0.5, 0.5, Vec3::new(5.0, 0.0, 0.0)),
    );

    let transform = identity_at(0.0, 0.0, 0.0);
    let result = world.body_time_of_impact_mover(
        body,
        Pos::ZERO,
        &standing_mover(),
        Vec3::new(10.0, 0.0, 0.0),
        &QueryFilter::default_filter(),
        transform,
        transform,
    );

    assert!(abs_float(result.fraction - 0.425) < 1e-2);
    assert_eq!(result.shape_id.index1, on_path.index1);
}

// MARK: - Sweep equivalence

struct XorShift {
    state: u32,
}

impl XorShift {
    fn new() -> Self {
        Self { state: 0x9E37_79B9 }
    }

    fn next(&mut self) -> u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        self.state
    }

    fn float(&mut self, lower: f32, upper: f32) -> f32 {
        let x = self.next();
        let unit = (x >> 8) as f32 / 16_777_216.0;
        lower + (upper - lower) * unit
    }

    fn vector(&mut self, lower: f32, upper: f32) -> Vec3 {
        Vec3::new(
            self.float(lower, upper),
            self.float(lower, upper),
            self.float(lower, upper),
        )
    }

    fn direction(&mut self) -> Vec3 {
        loop {
            let v = self.vector(-1.0, 1.0);
            let len = length(v);
            if 0.1 < len && len < 1.0 {
                return (1.0 / len) * v;
            }
        }
    }

    fn quat(&mut self) -> Quat {
        let direction = self.direction();
        let angle = self.float(0.0, b3::PI);
        make_quat_from_axis_angle(direction, angle)
    }
}

/// The mover sweep follows the solver's continuous rule: any time of impact strictly inside the
/// sweep counts, whatever the root finder reports. A capsule pivoting around a hull vertex can
/// exhaust the iteration cap, and that must not read as a clean miss.
// "the body TOI matches the raw sweep across 2000 random poses"
#[test]
fn matches_sweep() {
    let (mut world, body) = make_world();
    let bar = make_box_hull(2.0, 0.25, 0.25);
    let shape = world.create_hull_shape(body, &ShapeDef::default(), &bar);

    let mut random = XorShift::new();
    let mut hit_count = 0;
    let mut failures: Vec<String> = Vec::new();

    let hull_points = bar.points().to_vec();
    if hull_points.is_empty() {
        assert!(false, "hull has no points");
        return;
    }

    for i in 0..2000 {
        let c1 = random.vector(-0.2, 0.2);
        let c2 = mul_add(c1, random.float(0.05, 1.5), random.direction());
        let radius = random.float(0.05, 0.5);
        let mover = Capsule::new(c1, c2, radius);

        let o = random.vector(-3.5, 3.5);
        let origin = Pos::new(o.x as f64, o.y as f64, o.z as f64);
        let translation = random.float(0.0, 3.0) * random.direction();

        let q1 = random.quat();
        let transform1 = WorldTransform::new(Pos::ZERO, q1);

        let spin_direction = random.direction();
        let spin_angle = random.float(0.001, b3::PI);
        let spin = make_quat_from_axis_angle(spin_direction, spin_angle);
        let shift = random.vector(-1.0, 1.0);
        let transform2 = WorldTransform::new(Pos::from_vec3(shift), mul_quat(spin, q1));

        let result = world.body_time_of_impact_mover(
            body,
            origin,
            &mover,
            translation,
            &QueryFilter::default_filter(),
            transform1,
            transform2,
        );

        // The same sweep the body query builds. A static body keeps its center at the origin.
        let xf1 = to_relative_transform(transform1, origin);
        let xf2 = to_relative_transform(transform2, origin);

        let mut input = TOIInput::default();
        input.sweep_a.c1 = xf1.p;
        input.sweep_a.c2 = xf2.p;
        input.sweep_a.q1 = transform1.q;
        input.sweep_a.q2 = transform2.q;
        input.sweep_a.local_center = Vec3::ZERO;
        input.sweep_b.c1 = Vec3::ZERO;
        input.sweep_b.c2 = translation;
        input.sweep_b.q1 = Quat::IDENTITY;
        input.sweep_b.q2 = Quat::IDENTITY;
        input.sweep_b.local_center = Vec3::ZERO;
        input.max_fraction = 1.0;

        let capsule_points = [c1, c2];
        let raw: TOIOutput = {
            input.proxy_a = ShapeProxy::new(Some(hull_points.as_ptr()), bar.vertex_count(), 0.0);
            input.proxy_b = ShapeProxy::new(Some(capsule_points.as_ptr()), 2, radius);
            time_of_impact(&input)
        };

        if raw.fraction > 0.0 && raw.fraction < 1.0 {
            if abs_float(result.fraction - raw.fraction) > 1e-6 {
                failures.push(format!(
                    "sample {}: fraction {} vs {}",
                    i, result.fraction, raw.fraction
                ));
            } else if result.shape_id.index1 != shape.index1 {
                failures.push(format!("sample {}: shape id", i));
            } else if !is_normalized(result.normal) {
                failures.push(format!("sample {}: normal not unit", i));
            } else if crate::tests::world_position_assertions::distance(
                sub_pos(result.point, origin),
                raw.point,
            ) > 1e-4
            {
                failures.push(format!("sample {}: point mismatch", i));
            }
            hit_count += 1;
        } else if abs_float(result.fraction - 1.0) > 1e-6 {
            failures.push(format!(
                "sample {}: expected miss, got {}",
                i, result.fraction
            ));
        } else if result.shape_id.index1 != 0 {
            failures.push(format!("sample {}: miss carried a shape id", i));
        }

        if failures.len() >= 3 {
            break;
        }
    }

    assert!(
        failures.is_empty(),
        "{:?}",
        &failures[..failures.len().min(3)]
    );
    assert!(hit_count > 0);
}
