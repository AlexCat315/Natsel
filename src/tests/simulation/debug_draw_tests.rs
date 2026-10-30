//! Ported from Tests/NatselTests/Simulation/DebugDrawTests.swift.
//!
//! Debug draw walks the world and reports what the caller asked to see.

use std::cell::RefCell;
use std::ffi::c_void;
use std::rc::Rc;
use std::sync::Arc;

use crate::collision::hull::make_box_hull;
use crate::math::types::{AABB, Pos, Vec3, WorldTransform};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::debug_draw::{
    DebugDraw, DebugMaterial, DebugShape, HexColor, make_debug_color,
};
use crate::types::ids::BodyID;
use crate::types::joint_defs::{
    DistanceJointDef, ParallelJointDef, PrismaticJointDef, RevoluteJointDef, SphericalJointDef,
    WeldJointDef, WheelJointDef,
};
use crate::types::shape_def::ShapeDef;

fn make_scene() -> (World, BodyID) {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -1.0, 0.0);
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(10.0, 1.0, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.6, 0.0);
    let box_ = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(box_, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }
    (world, box_)
}

// "shapes are reported with a factory-produced handle"
#[test]
fn shapes() {
    let (mut world, _) = make_scene();

    let created = Rc::new(RefCell::new(0usize));
    let drawn = Rc::new(RefCell::new(0usize));
    let bounds_drawn = Rc::new(RefCell::new(0usize));
    let last_transform = Rc::new(RefCell::new(WorldTransform::default()));

    let mut options = DebugDraw::new();
    {
        let created = created.clone();
        options.create_debug_shape = Some(Arc::new(move |_shape: &DebugShape| {
            *created.borrow_mut() += 1;
            // Any non-nil token stands in for a renderer handle.
            Some((0x1000 + *created.borrow()) as *mut c_void)
        }));
    }
    {
        let drawn = drawn.clone();
        let last_transform = last_transform.clone();
        options.draw_shape = Some(Arc::new(
            move |_user: *mut c_void, transform: WorldTransform, _color: HexColor| {
                *drawn.borrow_mut() += 1;
                *last_transform.borrow_mut() = transform;
            },
        ));
    }
    {
        let bounds_drawn = bounds_drawn.clone();
        options.draw_bounds = Some(Arc::new(move |_aabb: AABB, _color: HexColor| {
            *bounds_drawn.borrow_mut() += 1;
        }));
    }

    world.draw(&options, u64::MAX);

    // Two shapes: the ground and the box.
    assert_eq!(*created.borrow(), 2);
    assert_eq!(*drawn.borrow(), 2);

    // The handle is cached on the shape, so a second pass reuses it.
    *created.borrow_mut() = 0;
    *drawn.borrow_mut() = 0;
    world.draw(&options, u64::MAX);
    assert_eq!(*created.borrow(), 0);
    assert_eq!(*drawn.borrow(), 2);

    // Bounds are only emitted when the option is on.
    options.draw_bounds_option = true;
    *bounds_drawn.borrow_mut() = 0;
    world.draw(&options, u64::MAX);
    assert_eq!(*bounds_drawn.borrow(), 2);
}

// "culling bounds limit what is reported"
#[test]
fn culling() {
    let (mut world, _) = make_scene();

    let drawn = Rc::new(RefCell::new(0usize));
    let mut options = DebugDraw::new();
    options.create_debug_shape = Some(Arc::new(|_shape: &DebugShape| {
        Some(0x1000 as *mut c_void)
    }));
    {
        let drawn = drawn.clone();
        options.draw_shape = Some(Arc::new(
            move |_user: *mut c_void, _transform: WorldTransform, _color: HexColor| {
                *drawn.borrow_mut() += 1;
            },
        ));
    }

    // The ground spans ±10, so a tight box around the upper body still only touches the box
    // if the ground's fat AABB reaches it. Use bounds well above the ground.
    options.drawing_bounds = AABB::new(
        Vec3::new(-1.0, 0.2, -1.0),
        Vec3::new(1.0, 2.0, 1.0),
    );
    world.draw(&options, u64::MAX);
    assert_eq!(*drawn.borrow(), 1);

    // Far away bounds see nothing.
    *drawn.borrow_mut() = 0;
    options.drawing_bounds = AABB::new(
        Vec3::new(100.0, 100.0, 100.0),
        Vec3::new(110.0, 110.0, 110.0),
    );
    world.draw(&options, u64::MAX);
    assert_eq!(*drawn.borrow(), 0);
}

// "contact normals are emitted when requested"
#[test]
fn contacts() {
    let (mut world, _) = make_scene();

    let segments = Rc::new(RefCell::new(0usize));
    let strings: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));

    let mut options = DebugDraw::new();
    options.draw_shapes = false;
    options.draw_contacts = true;
    options.draw_contact_normals = true;
    {
        let segments = segments.clone();
        options.draw_segment = Some(Arc::new(move |_a: Pos, _b: Pos, _color: HexColor| {
            *segments.borrow_mut() += 1;
        }));
    }
    {
        let strings = strings.clone();
        options.draw_string = Some(Arc::new(move |_p: Pos, s: &str, _color: HexColor| {
            strings.borrow_mut().push(s.to_string());
        }));
    }

    world.draw(&options, u64::MAX);

    assert!(*segments.borrow() > 0);
    // Normal labels carry the separation in cm.
    assert!(strings.borrow().iter().any(|s| s.contains(".")));
}

// "graph colors emit one point per contact point"
#[test]
fn graph_colors() {
    let (mut world, _) = make_scene();

    let points = Rc::new(RefCell::new(0usize));
    let mut options = DebugDraw::new();
    options.draw_shapes = false;
    options.draw_contacts = true;
    options.draw_graph_colors = true;
    {
        let points = points.clone();
        options.draw_point = Some(Arc::new(move |_p: Pos, _r: f32, _color: HexColor| {
            *points.borrow_mut() += 1;
        }));
    }

    world.draw(&options, u64::MAX);
    assert!(*points.borrow() > 0);
}

// "body names and mass annotations are reported"
#[test]
fn annotations() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.name = Some("test_body".to_string());
    let body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let strings: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
    let transforms = Rc::new(RefCell::new(0usize));

    let mut options = DebugDraw::new();
    options.draw_shapes = false;
    options.draw_body_names = true;
    options.draw_mass = true;
    {
        let strings = strings.clone();
        options.draw_string = Some(Arc::new(move |_p: Pos, s: &str, _color: HexColor| {
            strings.borrow_mut().push(s.to_string());
        }));
    }
    {
        let transforms = transforms.clone();
        options.draw_transform = Some(Arc::new(move |_t: WorldTransform| {
            *transforms.borrow_mut() += 1;
        }));
    }

    world.draw(&options, u64::MAX);

    assert!(strings.borrow().iter().any(|s| s == "test_body"));
    assert!(strings.borrow().iter().any(|s| s.contains("1000")));
    assert_eq!(*transforms.borrow(), 1);
}

// "joints connect their anchors with segments"
#[test]
fn joints() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    let anchor = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 5.0, 0.0);
    let bob = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(bob, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    let mut joint_def = DistanceJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.length = 1.0;
    let _ = world.create_distance_joint(&joint_def);

    let segments = Rc::new(RefCell::new(0usize));
    let points = Rc::new(RefCell::new(0usize));
    let mut options = DebugDraw::new();
    options.draw_shapes = false;
    options.draw_joints = true;
    {
        let segments = segments.clone();
        options.draw_segment = Some(Arc::new(move |_a: Pos, _b: Pos, _c: HexColor| {
            *segments.borrow_mut() += 1;
        }));
    }
    {
        let points = points.clone();
        options.draw_point = Some(Arc::new(move |_p: Pos, _r: f32, _c: HexColor| {
            *points.borrow_mut() += 1;
        }));
    }

    world.draw(&options, u64::MAX);
    // A distance joint draws its own axis plus anchor markers.
    assert_eq!(*segments.borrow(), 1);
    assert_eq!(*points.borrow(), 2);
}

// "material presets pack into the high byte"
#[test]
fn color_packing() {
    let rgb: HexColor = 0xFF8000;
    let plain = make_debug_color(rgb, DebugMaterial::Default);
    assert_eq!(plain, rgb);

    let glossy = make_debug_color(rgb, DebugMaterial::Glossy);
    assert_eq!(glossy & 0x00FF_FFFF, rgb);
    assert_eq!(glossy >> 24, DebugMaterial::Glossy as u32);

    let metallic = make_debug_color(rgb, DebugMaterial::Metallic);
    assert_eq!(metallic >> 24, 5);
}

// MARK: - Per-type joint drawing

struct Counts {
    segments: usize,
    points: usize,
    boxes: usize,
}

fn draw_counts(world: &mut World) -> Counts {
    let counts = Rc::new(RefCell::new(Counts {
        segments: 0,
        points: 0,
        boxes: 0,
    }));
    let mut options = DebugDraw::new();
    options.draw_shapes = false;
    options.draw_joints = true;
    {
        let counts = counts.clone();
        options.draw_segment = Some(Arc::new(move |_a: Pos, _b: Pos, _c: HexColor| {
            counts.borrow_mut().segments += 1;
        }));
    }
    {
        let counts = counts.clone();
        options.draw_point = Some(Arc::new(move |_p: Pos, _r: f32, _c: HexColor| {
            counts.borrow_mut().points += 1;
        }));
    }
    {
        let counts = counts.clone();
        options.draw_box = Some(Arc::new(move |_h: Vec3, _t: WorldTransform, _c: HexColor| {
            counts.borrow_mut().boxes += 1;
        }));
    }
    world.draw(&options, u64::MAX);
    let borrowed = counts.borrow();
    Counts {
        segments: borrowed.segments,
        points: borrowed.points,
        boxes: borrowed.boxes,
    }
}

fn make_joint_world() -> (World, BodyID, BodyID) {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    let anchor = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 5.0, 0.0);
    let bob = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(bob, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    (world, anchor, bob)
}

// "a distance joint draws its limit markers and spring rest point"
#[test]
fn distance_joint_detail() {
    let (mut world, anchor, bob) = make_joint_world();

    let mut joint_def = DistanceJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.length = 1.0;
    joint_def.enable_limit = true;
    joint_def.min_length = 0.5;
    joint_def.max_length = 2.0;
    joint_def.enable_spring = true;
    joint_def.hertz = 4.0;
    let _ = world.create_distance_joint(&joint_def);

    let counts = draw_counts(&mut world);
    // Limit points (2) + anchor points (2) + spring rest point (1).
    assert_eq!(counts.points, 5);
    // The limit segment plus the main axis.
    assert_eq!(counts.segments, 2);
}

// "a revolute joint draws frame axes and the limit wedge"
#[test]
fn revolute_joint_detail() {
    let (mut world, anchor, bob) = make_joint_world();
    let (mut plain_world, plain_anchor, plain_bob) = make_joint_world();

    let mut joint_def = RevoluteJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.enable_limit = true;
    joint_def.lower_angle = -0.5;
    joint_def.upper_angle = 0.5;
    let _ = world.create_revolute_joint(&joint_def);

    // Same joint without a limit for comparison.
    let mut plain_def = RevoluteJointDef::default();
    plain_def.base.body_id_a = plain_anchor;
    plain_def.base.body_id_b = plain_bob;
    let _ = plain_world.create_revolute_joint(&plain_def);

    let limited = draw_counts(&mut world);
    let plain = draw_counts(&mut plain_world);

    // Three frame axes either way.
    assert_eq!(plain.segments, 3);
    // The wedge adds its 16 slices plus the two closing spokes plus the twist marker.
    assert!(limited.segments > plain.segments + 15);
}

// "a prismatic joint draws its axis and limits"
#[test]
fn prismatic_joint_detail() {
    let (mut world, anchor, bob) = make_joint_world();

    let mut joint_def = PrismaticJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.enable_limit = true;
    joint_def.lower_translation = -0.5;
    joint_def.upper_translation = 0.5;
    let _ = world.create_prismatic_joint(&joint_def);

    let counts = draw_counts(&mut world);
    // Two perpendicular axes plus the limit axis.
    assert_eq!(counts.segments, 3);
    // Two limit points plus the frame B marker.
    assert_eq!(counts.points, 3);
}

// "a spherical joint draws its cone and twist wedges"
#[test]
fn spherical_joint_detail() {
    let (mut world, anchor, bob) = make_joint_world();

    let mut joint_def = SphericalJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.enable_cone_limit = true;
    joint_def.cone_angle = 0.4;
    joint_def.enable_twist_limit = true;
    joint_def.lower_twist_angle = -0.3;
    joint_def.upper_twist_angle = 0.3;
    let _ = world.create_spherical_joint(&joint_def);

    let counts = draw_counts(&mut world);
    // Three frame axes, the cone axis, the twist wedge and the cone fan all contribute.
    assert!(counts.segments > 40);
}

// "a weld joint draws two boxes"
#[test]
fn weld_joint_detail() {
    let (mut world, anchor, bob) = make_joint_world();

    let mut joint_def = WeldJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    let _ = world.create_weld_joint(&joint_def);

    let counts = draw_counts(&mut world);
    assert_eq!(counts.boxes, 2);
    assert_eq!(counts.segments, 0);
}

// "a parallel joint draws one axis per frame"
#[test]
fn parallel_joint_detail() {
    let (mut world, anchor, bob) = make_joint_world();

    let mut joint_def = ParallelJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    let _ = world.create_parallel_joint(&joint_def);

    let counts = draw_counts(&mut world);
    assert_eq!(counts.segments, 2);
}

// "a wheel joint draws its suspension and spin axle"
#[test]
fn wheel_joint_detail() {
    let (mut world, anchor, bob) = make_joint_world();

    let mut joint_def = WheelJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.enable_suspension_limit = true;
    joint_def.lower_suspension_limit = -0.3;
    joint_def.upper_suspension_limit = 0.3;
    let _ = world.create_wheel_joint(&joint_def);

    let counts = draw_counts(&mut world);
    // Link, suspension axis, two limit ticks, spin axle.
    assert!(counts.segments >= 5);
    assert_eq!(counts.points, 2);
}
