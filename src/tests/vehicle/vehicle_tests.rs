//! Vehicles: wheeled cars, tracked vehicles, motorcycles, and their helpers.
//!
//! Ported from `Tests/NatselTests/Vehicle/VehicleTests.swift`.

use crate::base::constants as b3;
use crate::collision::hull::{make_box_hull, make_offset_box_hull, make_transformed_box_hull};
use crate::math::functions::{
    atan2_approx, compute_cos_sin, distance_pos, make_quat_from_axis_angle, rotate_vector,
};
use crate::math::types::{Pos, Transform, Vec2, Vec3};
use crate::simulation::world_runtime::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;
use crate::vehicle::vehicle_drivetrain::limited_slip_fractions;
use crate::vehicle::vehicle_types::{
    LinearCurve, MotorcycleControllerSettings, TrackedVehicleControllerSettings, VehicleAntiRollBar,
    VehicleCollisionTester, VehicleControllerSettings, VehicleDef, VehicleDifferentialSettings,
    VehicleID,
    WheeledVehicleControllerSettings, WheelSettings,
};

/// A 1500 kg car on a flat static ground, z forward, y up, x left.
fn make_car(
    world: &mut World,
    tester: VehicleCollisionTester,
    position: Vec3,
    anti_roll: bool,
    max_pitch_roll: f32,
) -> (VehicleID, BodyID) {
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::from_vec3(position);
    let body = world.create_body(&body_def);

    let half_width: f32 = 0.9;
    let half_length: f32 = 2.0;
    let half_height: f32 = 0.25;
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1500.0 / (8.0 * half_width * half_length * half_height);
    let _ = world.create_hull_shape(
        body,
        &shape_def,
        &make_box_hull(half_width, half_height, half_length),
    );

    let mut def = VehicleDef::default();
    def.body_id = body;
    def.collision_tester = tester;
    def.max_pitch_roll_angle = max_pitch_roll;
    let positions = [
        Vec3::new(half_width, -0.1, half_length - 0.4),
        Vec3::new(-half_width, -0.1, half_length - 0.4),
        Vec3::new(half_width, -0.1, -half_length + 0.4),
        Vec3::new(-half_width, -0.1, -half_length + 0.4),
    ];
    for (i, p) in positions.iter().enumerate() {
        let mut wheel = WheelSettings::new();
        wheel.position = *p;
        wheel.radius = 0.35;
        wheel.width = 0.25;
        wheel.suspension_min_length = 0.1;
        wheel.suspension_max_length = 0.4;
        wheel.max_steer_angle = if i < 2 { 30.0 * b3::DEG_TO_RAD } else { 0.0 };
        wheel.max_hand_brake_torque = if i < 2 { 0.0 } else { 4000.0 };
        def.wheels.push(wheel);
    }
    if anti_roll {
        def.anti_roll_bars = vec![
            VehicleAntiRollBar {
                left_wheel: 0,
                right_wheel: 1,
                ..VehicleAntiRollBar::default()
            },
            VehicleAntiRollBar {
                left_wheel: 2,
                right_wheel: 3,
                ..VehicleAntiRollBar::default()
            },
        ];
    }
    let mut controller = WheeledVehicleControllerSettings::new();
    controller.differentials = vec![
        VehicleDifferentialSettings {
            left_wheel: 0,
            right_wheel: 1,
            engine_torque_ratio: 0.5,
            ..VehicleDifferentialSettings::default()
        },
        VehicleDifferentialSettings {
            left_wheel: 2,
            right_wheel: 3,
            engine_torque_ratio: 0.5,
            ..VehicleDifferentialSettings::default()
        },
    ];
    def.controller = VehicleControllerSettings::Wheeled(controller);
    (world.create_vehicle(&def), body)
}

fn make_ground(world: &mut World, friction: f32) -> BodyID {
    let ground = world.create_body(&BodyDef::default());
    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = friction;
    let _ = world.create_hull_shape(
        ground,
        &shape_def,
        &make_offset_box_hull(2000.0, 1.0, 2000.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    ground
}

fn make_car_default(world: &mut World) -> (VehicleID, BodyID) {
    make_car(
        world,
        VehicleCollisionTester::Ray,
        Vec3::new(0.0, 1.2, 0.0),
        true,
        b3::PI,
    )
}

#[test]
// "car settles on its suspension"
fn settles() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let (car, body) = make_car_default(&mut world);
    assert!(car.is_null() == false);

    for _ in 0..180 {
        world.step(1.0 / 60.0, 4);
    }

    let p = world.body_position(body);
    assert!(world.body_linear_velocity(body).y.abs() < 0.05);
    // Chassis bottom is above the ground: the wheels carry the car.
    assert!(p.y - 0.25 > 0.15);
    let mut total_force: f32 = 0.0;
    for w in 0..4 {
        let state = world.vehicle_wheel_state(car, w).expect("wheel state");
        assert!(state.has_contact);
        assert!(state.suspension_length > 0.1 && state.suspension_length < 0.4);
        total_force += state.suspension_force;
    }
    // The suspension carries the weight.
    assert!((total_force - 1500.0 * 10.0).abs() < 0.05 * 1500.0 * 10.0);
}

#[test]
// "throttle accelerates, shifts gears, and brakes stop the car" (arguments: tester = .ray)
fn drive_ray() {
    drive(VehicleCollisionTester::Ray);
}

#[test]
// "throttle accelerates, shifts gears, and brakes stop the car" (arguments: tester = .sphere)
fn drive_sphere() {
    drive(VehicleCollisionTester::Sphere);
}

#[test]
// "throttle accelerates, shifts gears, and brakes stop the car" (arguments: tester = .cylinder(0.5))
fn drive_cylinder() {
    drive(VehicleCollisionTester::Cylinder(0.5));
}

fn drive(tester: VehicleCollisionTester) {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let (car, body) = make_car(&mut world, tester, Vec3::new(0.0, 1.2, 0.0), true, b3::PI);
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }

    let mut max_gear: i32 = 0;
    for _ in 0..600 {
        world.set_vehicle_driver_input(car, 1.0, 0.0, 0.0, 0.0);
        world.step(1.0 / 60.0, 4);
        max_gear = max_gear.max(
            world
                .vehicle_drivetrain_state(car)
                .expect("drivetrain state")
                .current_gear,
        );
    }
    let speed = world.vehicle_forward_speed(car);
    assert!(speed > 15.0);
    assert!(max_gear >= 2);
    let rpm = world
        .vehicle_drivetrain_state(car)
        .expect("drivetrain state")
        .engine_rpm;
    assert!(rpm >= 1000.0 && rpm <= 6000.0);
    // It drove straight.
    assert!(world.body_position(body).x.abs() < 1.0);

    for _ in 0..600 {
        world.set_vehicle_driver_input(car, 0.0, 0.0, 1.0, 0.0);
        world.step(1.0 / 60.0, 4);
    }
    assert!(world.vehicle_forward_speed(car).abs() < 0.1);
}

#[test]
// "reverse input drives backward"
fn reverse() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let (car, body) = make_car_default(&mut world);
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }
    let z0 = world.body_position(body).z;
    for _ in 0..240 {
        world.set_vehicle_driver_input(car, -1.0, 0.0, 0.0, 0.0);
        world.step(1.0 / 60.0, 4);
    }
    assert!(
        world
            .vehicle_drivetrain_state(car)
            .expect("drivetrain state")
            .current_gear
            < 0
    );
    assert!(world.body_position(body).z < z0 - 5.0);
}

#[test]
// "steering turns the car right"
fn steer() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let (car, body) = make_car_default(&mut world);
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }
    for _ in 0..240 {
        world.set_vehicle_driver_input(car, 0.5, 1.0, 0.0, 0.0);
        world.step(1.0 / 60.0, 4);
    }
    // With x to the left, a right turn moves toward negative x and yaws clockwise from above.
    assert!(world.body_position(body).x < -2.0);
    assert!(world.body_angular_velocity(body).y < 0.0);
    assert!(
        world
            .vehicle_wheel_state(car, 0)
            .expect("wheel state")
            .steer_angle
            < 0.0
    );
}

#[test]
// "pitch/roll limit keeps a tipped car upright"
fn pitch_roll() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let (_, body) = make_car(
        &mut world,
        VehicleCollisionTester::Ray,
        Vec3::new(0.0, 3.0, 0.0),
        true,
        30.0 * b3::DEG_TO_RAD,
    );
    world.set_body_angular_velocity(body, Vec3::new(0.0, 0.0, 8.0));
    for _ in 0..240 {
        world.step(1.0 / 60.0, 4);
    }
    let up = rotate_vector(world.body_rotation(body), Vec3::new(0.0, 1.0, 0.0));
    assert!(up.y > compute_cos_sin(32.0 * b3::DEG_TO_RAD).cosine);
}

#[test]
// "parked car with hand brake holds on a slope"
fn hand_brake() {
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = 1.0;
    let slope = make_quat_from_axis_angle(Vec3::new(1.0, 0.0, 0.0), 10.0 * b3::DEG_TO_RAD);
    let _ = world.create_hull_shape(
        ground,
        &shape_def,
        &make_transformed_box_hull(100.0, 1.0, 100.0, Transform::new(Vec3::new(0.0, -1.0, 0.0), slope)),
    );
    let (car, body) = make_car(
        &mut world,
        VehicleCollisionTester::Ray,
        Vec3::new(0.0, 1.4, 0.0),
        true,
        b3::PI,
    );
    world.set_body_transform(body, Pos::new(0.0, 1.4, 0.0), slope);
    for _ in 0..60 {
        world.set_vehicle_driver_input(car, 0.0, 0.0, 1.0, 1.0);
        world.step(1.0 / 60.0, 4);
    }
    let p0 = world.body_position(body);
    for _ in 0..240 {
        world.set_vehicle_driver_input(car, 0.0, 0.0, 1.0, 1.0);
        world.step(1.0 / 60.0, 4);
    }
    assert!(distance_pos(world.body_position(body), p0) < 0.05);
}

#[test]
// "car pushes a dynamic box it drives onto and the box feels the load"
fn dynamic_ground() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(0.0, 0.25, 0.0);
    let platform = world.create_body(&body_def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 100.0;
    shape_def.base_material.friction = 1.0;
    let _ = world.create_hull_shape(platform, &shape_def, &make_box_hull(3.0, 0.25, 4.0));

    let (car, _) = make_car(
        &mut world,
        VehicleCollisionTester::Ray,
        Vec3::new(0.0, 1.6, 0.0),
        true,
        b3::PI,
    );
    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
    }
    let state = world.vehicle_wheel_state(car, 0).expect("wheel state");
    assert!(state.has_contact);
    assert!(state.contact_body_id == platform);
    assert!(world.body_position(platform).y < 0.26);
}

#[test]
// "vehicle sleeps when idle and wakes on input"
fn sleeping() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let (car, body) = make_car_default(&mut world);
    for _ in 0..300 {
        world.step(1.0 / 60.0, 4);
    }
    assert!(world.is_body_awake_id(body) == false);
    world.set_vehicle_driver_input(car, 1.0, 0.0, 0.0, 0.0);
    assert!(world.is_body_awake_id(body));
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }
    assert!(world.vehicle_forward_speed(car) > 1.0);
}

#[test]
// "destroying the chassis removes the vehicle"
fn destroy_chassis() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let (car, body) = make_car_default(&mut world);
    world.step(1.0 / 60.0, 4);
    world.destroy_body(body);
    world.step(1.0 / 60.0, 4);
    assert!(world.vehicle_is_valid(car) == false);
    assert!(world.vehicle_count() == 0);
}

#[test]
// "tracked vehicle drives and turns on the spot"
fn tracked() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(0.0, 1.2, 0.0);
    let body = world.create_body(&body_def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 8000.0 / (8.0 * 1.5 * 0.5 * 3.0);
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(1.5, 0.5, 3.0));

    let mut def = VehicleDef::default();
    def.body_id = body;
    let mut controller = TrackedVehicleControllerSettings::new();
    controller.engine.max_torque = 3000.0;
    controller.transmission.clutch_strength = 200.0;
    controller.tracks[0].wheels = Vec::new();
    controller.tracks[1].wheels = Vec::new();
    for side in 0..2 {
        let x: f32 = if side == 0 { 1.4 } else { -1.4 };
        for k in 0..5 {
            let mut wheel = WheelSettings::new();
            wheel.position = Vec3::new(x, -0.4, -2.4 + 1.2 * (k as f32));
            wheel.radius = 0.4;
            wheel.suspension_min_length = 0.2;
            wheel.suspension_max_length = 0.5;
            wheel.suspension_frequency = 1.5;
            wheel.track_lateral_friction = 1.0;
            controller.tracks[side].wheels.push(def.wheels.len() as i32);
            def.wheels.push(wheel);
        }
        controller.tracks[side].driven_wheel = controller.tracks[side].wheels[4];
    }
    def.controller = VehicleControllerSettings::Tracked(controller);
    let tank = world.create_vehicle(&def);
    assert!(tank.is_null() == false);

    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }
    for _ in 0..300 {
        world.set_tracked_vehicle_driver_input(tank, 1.0, 1.0, 1.0, 0.0);
        world.step(1.0 / 60.0, 4);
    }
    assert!(world.vehicle_forward_speed(tank) > 3.0);
    assert!(world.body_position(body).x.abs() < 1.0);

    for _ in 0..300 {
        world.set_tracked_vehicle_driver_input(tank, 0.0, 0.0, 0.0, 1.0);
        world.step(1.0 / 60.0, 4);
    }
    let p = world.body_position(body);
    for _ in 0..240 {
        world.set_tracked_vehicle_driver_input(tank, 1.0, 1.0, -1.0, 0.0);
        world.step(1.0 / 60.0, 4);
    }
    // Pivot turn: yaw rate without much travel.
    assert!(world.body_angular_velocity(body).y.abs() > 0.2);
    assert!(distance_pos(world.body_position(body), p) < 2.0);
}

#[test]
// "motorcycle stays upright and leans into turns"
fn motorcycle() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(0.0, 0.9, 0.0);
    let body = world.create_body(&body_def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 240.0 / (8.0 * 0.2 * 0.3 * 0.8);
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.2, 0.3, 0.8));

    let mut def = VehicleDef::default();
    def.body_id = body;
    let mut front = WheelSettings::new();
    front.position = Vec3::new(0.0, -0.2, 0.75);
    front.radius = 0.31;
    front.suspension_min_length = 0.1;
    front.suspension_max_length = 0.3;
    front.max_steer_angle = 30.0 * b3::DEG_TO_RAD;
    front.inertia = 0.2;
    let mut rear = front.clone();
    rear.position = Vec3::new(0.0, -0.2, -0.75);
    rear.max_steer_angle = 0.0;
    def.wheels = vec![front, rear];

    let mut controller = MotorcycleControllerSettings::new();
    controller.wheeled.engine.max_torque = 150.0;
    controller.wheeled.engine.max_rpm = 9000.0;
    controller.wheeled.transmission.shift_up_rpm = 7000.0;
    controller.wheeled.differentials = vec![VehicleDifferentialSettings {
        left_wheel: -1,
        right_wheel: 1,
        ..VehicleDifferentialSettings::default()
    }];
    controller.wheeled.differentials[0].differential_ratio = 1.93 * 40.0 / 16.0;
    def.controller = VehicleControllerSettings::Motorcycle(controller);
    let bike = world.create_vehicle(&def);

    for _ in 0..300 {
        world.set_vehicle_driver_input(bike, 0.6, 0.0, 0.0, 0.0);
        world.step(1.0 / 60.0, 4);
    }
    let up = rotate_vector(world.body_rotation(body), Vec3::new(0.0, 1.0, 0.0));
    assert!(up.y > 0.95);
    assert!(world.vehicle_forward_speed(bike) > 5.0);

    for _ in 0..60 {
        world.set_vehicle_driver_input(bike, 0.3, 0.3, 0.0, 0.0);
        world.step(1.0 / 60.0, 4);
    }
    let lean = world
        .vehicle_drivetrain_state(bike)
        .expect("drivetrain state")
        .lean_angle;
    assert!(lean > 2.0 * b3::DEG_TO_RAD);
    let up2 = rotate_vector(world.body_rotation(body), Vec3::new(0.0, 1.0, 0.0));
    assert!(up2.y > 0.5);
}

#[test]
// "limited slip and curves"
fn helpers() {
    let curve = LinearCurve::new(&[
        Vec2::new(1.0, 2.0),
        Vec2::new(0.0, 0.0),
        Vec2::new(2.0, 1.0),
    ]);
    assert!(curve.evaluate(-1.0) == 0.0);
    assert!(curve.evaluate(0.5) == 1.0);
    assert!(curve.evaluate(1.5) == 1.5);
    assert!(curve.evaluate(9.0) == 1.0);

    let open = limited_slip_fractions(0.5, 10.0, 1.0, f32::MAX);
    assert!(open.0 == 0.5);
    let locked = limited_slip_fractions(0.5, 10.0, 1.0, 1.4);
    assert!(locked.0 == 0.0 && locked.1 == 1.0);
    let partial = limited_slip_fractions(0.5, 1.2, 1.0, 1.4);
    assert!(partial.0 < 0.5 && partial.0 > 0.0);
}

#[test]
// "wheel transform follows suspension and steering"
fn wheel_transform() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let (car, _) = make_car_default(&mut world);
    for _ in 0..60 {
        world.set_vehicle_driver_input(car, 0.0, 1.0, 0.0, 0.0);
        world.step(1.0 / 60.0, 4);
    }
    let local = world
        .vehicle_wheel_local_transform(car, 0, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0))
        .expect("wheel transform");
    let state = world.vehicle_wheel_state(car, 0).expect("wheel state");
    let settings = world.vehicle_wheel_settings(car, 0).expect("wheel settings");
    assert!((local.p.y - (settings.position.y - state.suspension_length)).abs() < 1.0e-4);
    // The axle (model x) is rotated by the steering angle about up.
    let axle = rotate_vector(local.q, Vec3::new(1.0, 0.0, 0.0));
    assert!(axle.y.abs() < 1.0e-3);
    assert!((atan2_approx(-axle.z, axle.x) - state.steer_angle).abs() < 0.01);
}

#[test]
// "debug draw shows wheels and contacts"
fn debug_draw() {
    let mut world = World::default();
    let _ = make_ground(&mut world, 1.0);
    let _ = make_car_default(&mut world);
    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }
    let segments = std::rc::Rc::new(std::cell::RefCell::new(0usize));
    let points = std::rc::Rc::new(std::cell::RefCell::new(0usize));
    let mut options = crate::types::debug_draw::DebugDraw::new();
    options.draw_shapes = false;
    let seg = segments.clone();
    options.draw_segment = Some(std::sync::Arc::new(move |_a: Pos, _b: Pos, _c| {
        *seg.borrow_mut() += 1;
    }));
    let pts = points.clone();
    options.draw_point = Some(std::sync::Arc::new(move |_a: Pos, _b: f32, _c| {
        *pts.borrow_mut() += 1;
    }));
    world.draw(&options, u64::MAX);
    // Per wheel: suspension, 16 rim segments, spoke, two contact lines.
    assert!(*segments.borrow() >= 4 * 20);
    assert!(*points.borrow() >= 4);
    options.draw_vehicles = false;
    *segments.borrow_mut() = 0;
    world.draw(&options, u64::MAX);
    assert!(*segments.borrow() == 0);
}

#[test]
// "parallel wheel casts match serial"
fn parallel_casts() {
    fn run(workers: i32) -> Vec<Pos> {
        let mut world = World::default();
        world.worker_count = workers;
        let _ = make_ground(&mut world, 1.0);
        let mut cars: Vec<(VehicleID, BodyID)> = Vec::new();
        for k in 0..10 {
            cars.push(make_car(
                &mut world,
                VehicleCollisionTester::Cylinder(0.5),
                Vec3::new((k as f32) * 4.0, 1.2, 0.0),
                true,
                b3::PI,
            ));
        }
        for step in 0..120 {
            for (i, car) in cars.iter().enumerate() {
                let right = if (step / 30 + i) % 2 == 0 { 0.5 } else { -0.5 };
                world.set_vehicle_driver_input(car.0, 0.8, right, 0.0, 0.0);
            }
            world.step(1.0 / 60.0, 4);
        }
        cars.iter()
            .map(|car| world.body_position(car.1))
            .collect()
    }

    assert!(run(1) == run(4));
}
