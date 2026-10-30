//! Ported from Tests/NatselTests/PathJointTests.swift.

use std::sync::Arc;

use crate::collision::hull::make_box_hull;
use crate::math::functions::{compute_cos_sin, dot, inv_transform_world_point, length, rotate_vector};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::World;
use crate::simulation::path_joint::{
    ConstraintPath, PathJointDef, PathMotorMode, PathPoint, PathRotationConstraint,
};
use crate::tests::world_position_assertions::distance;
use crate::tests::SWIFT_FLOAT_PI;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::{BodyID, JointID};
use crate::types::shape_def::ShapeDef;

struct PathRig {
    world: World,
    ground: BodyID,
    body: BodyID,
}

impl PathRig {
    fn new(gravity: Vec3, position: Vec3) -> Self {
        let mut world = World::default();
        world.gravity = gravity;
        world.enable_sleep = false;
        let ground = world.create_body(&BodyDef::default());
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::from_vec3(position);
        let body = world.create_body(&def);
        let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.2, 0.1, 0.1));
        PathRig { world, ground, body }
    }

    fn joint(
        &mut self,
        path: Arc<ConstraintPath>,
        configure: impl FnOnce(&mut PathJointDef),
    ) -> JointID {
        let mut def = PathJointDef::default();
        def.base.body_id_a = self.ground;
        def.base.body_id_b = self.body;
        def.path = Some(path);
        configure(&mut def);
        self.world.create_path_joint(&def)
    }

    fn run(&mut self, steps: i32, each: impl Fn()) {
        for _ in 0..steps {
            self.world.step(1.0 / 60.0, 4);
            each();
        }
    }
}

/// A circle of radius `r` in the xz plane, counter-clockwise seen from above.
/// Swift's `circle(_ r: Float, _ points: Int = 12)`; the default is passed explicitly.
const DEFAULT_CIRCLE_POINTS: i32 = 12;

fn circle(r: f32, points: i32) -> Arc<ConstraintPath> {
    let positions: Vec<Vec3> = (0..points)
        .map(|i| {
            let a = 2.0 * SWIFT_FLOAT_PI * (i as f32) / (points as f32);
            let cs = compute_cos_sin(a);
            Vec3::new(r * cs.cosine, 0.0, -r * cs.sine)
        })
        .collect();
    Arc::new(
        ConstraintPath::through(&positions, Vec3::new(0.0, 1.0, 0.0), true).unwrap(),
    )
}

fn line(from: Vec3, to: Vec3) -> Arc<ConstraintPath> {
    Arc::new(
        ConstraintPath::new(
            vec![
                PathPoint::new(from, to - from, Vec3::new(0.0, 1.0, 0.0)),
                PathPoint::new(to, to - from, Vec3::new(0.0, 1.0, 0.0)),
            ],
            false,
        )
        .unwrap(),
    )
}

#[test]
fn geometry() {
    // "path geometry: interpolation, closest point, and arc length"
    let straight = line(Vec3::new(0.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0));
    assert_eq!(straight.max_fraction(), 1.0);
    assert!((straight.length() - 4.0).abs() < 1.0e-4);
    assert!((straight.closest_fraction_to(Vec3::new(1.0, 3.0, 0.0), 0.0) - 0.25).abs() < 1.0e-4);
    assert_eq!(straight.closest_fraction_to(Vec3::new(-2.0, 0.0, 0.0), 0.0), 0.0);
    assert_eq!(straight.closest_fraction_to(Vec3::new(9.0, 0.0, 0.0), 0.0), 1.0);
    let (fp, ft, fn_, fb) = straight.point_at(0.5);
    assert!(distance(fp, Vec3::new(2.0, 0.0, 0.0)) < 1.0e-5);
    assert!(distance(ft, Vec3::new(1.0, 0.0, 0.0)) < 1.0e-5);
    assert!(distance(fn_, Vec3::new(0.0, 1.0, 0.0)) < 1.0e-5);
    assert!(distance(fb, Vec3::new(0.0, 0.0, 1.0)) < 1.0e-5);

    let ring = circle(3.0, 16);
    assert_eq!(ring.max_fraction(), 16.0);
    // Catmull-Rom passes through its points; the loop length is close to 2 pi r.
    for i in 0..16 {
        let p = ring.point_at(i as f32).0;
        assert!((length(p) - 3.0).abs() < 1.0e-4);
    }
    assert!((ring.length() - 2.0 * SWIFT_FLOAT_PI * 3.0).abs() < 0.05);
    // The search agrees with brute force sampling, for queries off the curve and on the seam.
    for query in [
        Vec3::new(3.6, 0.2, -0.5),
        Vec3::new(-1.0, 1.0, 2.5),
        Vec3::new(2.9, -0.3, 0.01),
        Vec3::new(0.1, 0.0, 0.1),
    ] {
        let found = ring.closest_fraction_to(query, 8.0);
        let mut best = f32::INFINITY;
        for k in 0..16000 {
            best = best.min(distance(ring.point_at((16.0 * (k as f32)) / 16000.0).0, query));
        }
        assert!(
            distance(ring.point_at(found).0, query) <= best + 1.0e-4,
            "query {query:?} found {found}"
        );
    }
    assert!((ring.closest_fraction_to(ring.point_at(16.25).0, 0.0) - 0.25).abs() < 1.0e-3);
}

#[test]
fn straight() {
    // "bead stays on a straight path and slides freely"
    let mut rig = PathRig::new(Vec3::new(5.0, -10.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    let joint = rig.joint(line(Vec3::new(-10.0, 1.0, 0.0), Vec3::new(10.0, 1.0, 0.0)), |_| {});
    for _ in 0..60 {
        rig.world.step(1.0 / 60.0, 4);
        let p = rig.world.body_position(rig.body);
        assert!((p.y - 1.0).abs() < 0.01 && p.z.abs() < 0.01);
    }
    // Free sliding: x = 0.5 a t^2 = 2.5 m after one second.
    assert!((rig.world.body_position(rig.body).x - 2.5).abs() < 0.1);
    let force = rig.world.joint_constraint_force(joint).unwrap();
    assert!((force.y - 10.0 * rig.world.body_mass(rig.body)).abs() < 0.5);
    assert!(force.x.abs() < 0.1);
}

#[test]
fn end_stop() {
    // "an open path stops the point at its ends"
    let mut rig = PathRig::new(Vec3::new(10.0, -10.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
    let joint = rig.joint(line(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0)), |_| {});
    rig.run(120, || {});
    assert!((rig.world.body_position(rig.body).x - 2.0).abs() < 0.01);
    assert_eq!(rig.world.path_joint_fraction(joint), Some(1.0));
    assert!(length(rig.world.body_linear_velocity(rig.body)) < 0.05);
}

#[test]
fn friction() {
    // "friction holds the point until gravity beats it"
    let mut rig = PathRig::new(Vec3::new(3.0, -10.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
    let mass = rig.world.body_mass(rig.body);
    let joint = rig.joint(line(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(5.0, 0.0, 0.0)), |def| {
        def.max_friction_force = 4.0 * mass;
    });
    rig.run(60, || {});
    assert!(rig.world.body_position(rig.body).x.abs() < 0.01);
    rig.world.set_path_joint_max_friction(joint, 2.0 * mass);
    rig.run(60, || {});
    // Net acceleration 1 m/s^2: 0.5 m after one second.
    assert!((rig.world.body_position(rig.body).x - 0.5).abs() < 0.05);
}

#[test]
fn loop_motor() {
    // "velocity motor drives around a loop and follows the path frame"
    let ring = circle(3.0, DEFAULT_CIRCLE_POINTS);
    let mut rig = PathRig::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0));
    let joint = rig.joint(Arc::clone(&ring), |def| {
        def.motor_mode = PathMotorMode::Velocity;
        def.target_velocity = 4.0;
        def.max_motor_force = 1000.0;
        def.rotation_constraint = PathRotationConstraint::ConstrainToPath;
    });
    let start = ring.point_at(0.0);
    let mut last: f32 = 0.0;
    let mut wraps = 0i32;
    for _ in 0..360 {
        rig.world.step(1.0 / 60.0, 4);
        let p = rig.world.body_position(rig.body);
        assert!(
            (f64::from(length(Vec3::new(p.x as f32, 0.0, p.z as f32))) - 3.0).abs() < 0.03
                && p.y.abs() < 0.01
        );
        let f = rig.world.path_joint_fraction(joint).unwrap();
        if f < last - 6.0 {
            wraps += 1;
        }
        last = f;
        // The body turns with the path frame.
        let frame = ring.point_at(f);
        let q = rig.world.body_transform(rig.body).q;
        assert!(dot(rotate_vector(q, start.1), frame.1) > 0.999);
        assert!(dot(rotate_vector(q, start.2), frame.2) > 0.999);
    }
    assert!((length(rig.world.body_linear_velocity(rig.body)) - 4.0).abs() < 0.05);
    // 4 m/s for 6 s on a 18.85 m loop.
    assert_eq!(wraps, 1);
}

#[test]
fn position_motor() {
    // "position motor reaches a target fraction"
    let positions = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 1.0),
        Vec3::new(4.0, 0.0, 0.0),
        Vec3::new(6.0, 0.0, -1.0),
    ];
    let path = Arc::new(
        ConstraintPath::through(&positions, Vec3::new(0.0, 1.0, 0.0), false).unwrap(),
    );
    let mut rig = PathRig::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
    let joint = rig.joint(Arc::clone(&path), |def| {
        def.motor_mode = PathMotorMode::Position;
        def.target_fraction = 2.5;
        def.max_motor_force = 200.0;
        def.motor_hertz = 2.0;
        def.motor_damping_ratio = 1.0;
    });
    rig.run(240, || {});
    assert!((rig.world.path_joint_fraction(joint).unwrap() - 2.5).abs() < 0.01);
    assert!(distance(rig.world.body_position(rig.body), path.point_at(2.5).0) < 0.02);
    rig.world.set_path_joint_target_fraction(joint, 0.5);
    rig.run(240, || {});
    assert!((rig.world.path_joint_fraction(joint).unwrap() - 0.5).abs() < 0.01);
}

#[test]
fn rotations() {
    // "rotation constraints"
    let ring = circle(3.0, DEFAULT_CIRCLE_POINTS);
    fn spin(ring: &Arc<ConstraintPath>, rotation: PathRotationConstraint) -> (PathRig, JointID) {
        let mut rig = PathRig::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0));
        let joint = rig.joint(Arc::clone(ring), |def| {
            def.rotation_constraint = rotation;
            def.motor_mode = PathMotorMode::Velocity;
            def.target_velocity = 3.0;
            def.max_motor_force = 1000.0;
        });
        rig.world
            .set_body_angular_velocity(rig.body, Vec3::new(2.0, 3.0, 4.0));
        (rig, joint)
    }

    // Fully constrained: the body keeps its orientation while it circles.
    {
        let (mut rig, _) = spin(&ring, PathRotationConstraint::FullyConstrained);
        for _ in 0..120 {
            rig.world.step(1.0 / 60.0, 4);
            assert!(rig.world.body_transform(rig.body).q.s > 0.999);
        }
    }
    // Hinged about a path axis: body B's copy of that axis (its direction at the start, since the
    // body starts unrotated) stays on the path axis while the body spins about it.
    let start = ring.point_at(0.0);
    for (rotation, axis) in [
        (PathRotationConstraint::ConstrainAroundTangent, 0),
        (PathRotationConstraint::ConstrainAroundNormal, 1),
        (PathRotationConstraint::ConstrainAroundBinormal, 2),
    ] {
        let (mut rig, joint) = spin(&ring, rotation);
        for _ in 0..120 {
            rig.world.step(1.0 / 60.0, 4);
            let frame = ring.point_at(rig.world.path_joint_fraction(joint).unwrap());
            let path_axis = [frame.1, frame.2, frame.3][axis];
            let body_axis = rotate_vector(
                rig.world.body_transform(rig.body).q,
                [start.1, start.2, start.3][axis],
            );
            assert!(dot(body_axis, path_axis) > 0.995, "{rotation:?}");
        }
        // It still spins about that axis.
        assert!(length(rig.world.body_angular_velocity(rig.body)) > 0.5, "{rotation:?}");
    }
}

#[test]
fn dynamic_carrier() {
    // "path on a moving body and joint lifetime"
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -10.0, 0.0);
    world.enable_sleep = false;
    let mut def = BodyDef::default();
    let carrier = world.create_body(&def);
    let _ = world.create_hull_shape(
        carrier,
        &ShapeDef::default(),
        &make_box_hull(2.0, 0.1, 0.5),
    );
    def.position = Pos::new(0.5, 0.3, 0.0);
    let bead = world.create_body(&def);
    let _ = world.create_sphere_shape(
        bead,
        &ShapeDef::default(),
        Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.1),
    );
    let mut joint_def = PathJointDef::default();
    joint_def.base.body_id_a = carrier;
    joint_def.base.body_id_b = bead;
    joint_def.base.local_frame_a = Transform::new(Vec3::new(0.0, 0.3, 0.0), Quat::IDENTITY);
    joint_def.path = Some(line(Vec3::new(-2.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0)));
    let id = world.create_path_joint(&joint_def);
    world
        .set_body_angular_velocity(carrier, Vec3::new(0.0, 0.0, 1.0));
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }
    // The bead stays on the line through the carrier's frame.
    let local = inv_transform_world_point(world.body_transform(carrier), world.body_position(bead));
    assert!((local.y - 0.3).abs() < 0.01 && local.z.abs() < 0.01);

    world.destroy_joint(id.index1 - 1, true);
    assert!(world.path_joint_fraction(id).is_none());
    // The slot is reused.
    let again = world.create_path_joint(&joint_def);
    assert!(world.path_joint_fraction(again).is_some());
    assert!(Arc::ptr_eq(
        world.path_joint_path(again).as_ref().unwrap(),
        joint_def.path.as_ref().unwrap()
    ));
}
