//! Ported from Tests/NatselTests/Simulation/RestitutionQualityTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::functions::abs_float;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;

const TIME_STEP: f32 = 1.0 / 60.0;
const SUB_STEPS: i32 = 4;
const IMPACT_SPEED: f32 = 10.0;
const RESTITUTION_ITERATIONS: i32 = 2;

fn make_world(gravity_y: f32) -> World {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, gravity_y, 0.0);
    world.enable_sleep = false;
    world.restitution_iterations = RESTITUTION_ITERATIONS;
    world
}

/// Ground with its top surface at y = 0
fn make_ground(world: &mut World, restitution: f32) {
    let mut body_def = BodyDef::default();
    body_def.position = Pos::new(0.0, -1.0, 0.0);
    let ground = world.create_body(&body_def);

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = restitution;
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(40.0, 1.0, 40.0));
}

fn make_ball(
    world: &mut World,
    x: f32,
    y: f32,
    velocity_y: f32,
    restitution: f32,
    density: f32,
) -> BodyID {
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(x as f64, y as f64, 0.0);
    body_def.linear_velocity = Vec3::new(0.0, velocity_y, 0.0);
    let ball = world.create_body(&body_def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = density;
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = restitution;
    let _ = world.create_sphere_shape(ball, &shape_def, Sphere::new(Vec3::ZERO, 0.5));
    ball
}

/// Two free spheres closing head on with no gravity. The coefficient is the ratio of relative
/// normal speeds at the contact, a mass independent definition.
fn measure_head_on(restitution: f32, density_b: f32) -> (f32, f32) {
    let mut world = make_world(0.0);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = restitution;

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(-1.0, 0.0, 0.0);
    body_def.linear_velocity = Vec3::new(0.5 * IMPACT_SPEED, 0.0, 0.0);
    let body_a = world.create_body(&body_def);
    let _ = world.create_sphere_shape(body_a, &shape_def, Sphere::new(Vec3::ZERO, 0.5));

    body_def.position = Pos::new(1.0, 0.0, 0.0);
    body_def.linear_velocity = Vec3::new(-0.5 * IMPACT_SPEED, 0.0, 0.0);
    let body_b = world.create_body(&body_def);
    shape_def.density = density_b;
    let _ = world.create_sphere_shape(body_b, &shape_def, Sphere::new(Vec3::ZERO, 0.5));

    let mass_a = world.body_mass_data(body_a).mass;
    let mass_b = world.body_mass_data(body_b).mass;
    let momentum_0 =
        mass_a * world.body_linear_velocity(body_a).x + mass_b * world.body_linear_velocity(body_b).x;

    for _ in 0..120 {
        world.step(TIME_STEP, SUB_STEPS);
    }

    let v_a = world.body_linear_velocity(body_a).x;
    let v_b = world.body_linear_velocity(body_b).x;
    let momentum_1 = mass_a * v_a + mass_b * v_b;

    (
        (v_b - v_a) / IMPACT_SPEED,
        (momentum_1 - momentum_0) / (mass_a + mass_b),
    )
}

/// Ball driven into static ground with no gravity, so the measurement carries no gravity bias.
fn measure_ground_bounce(restitution: f32, gap: f32) -> f32 {
    let mut world = make_world(0.0);
    make_ground(&mut world, 0.0);

    let ball = make_ball(&mut world, 0.0, 0.5 + gap, -IMPACT_SPEED, restitution, 1.0);
    for _ in 0..60 {
        world.step(TIME_STEP, SUB_STEPS);
    }

    world.body_linear_velocity(ball).y / IMPACT_SPEED
}

/// Perfectly elastic ball dropped under gravity. Returns the apex heights.
fn measure_drop(drop_height: f32) -> Vec<f32> {
    let mut world = make_world(-10.0);
    make_ground(&mut world, 0.0);

    let ball = make_ball(&mut world, 0.0, drop_height, 0.0, 1.0, 1.0);
    let mut apexes: Vec<f32> = Vec::new();
    let mut previous_speed: f32 = 0.0;

    let mut step = 0;
    while step < 4000 && apexes.len() < 4 {
        world.step(TIME_STEP, SUB_STEPS);

        let speed = world.body_linear_velocity(ball).y;
        if previous_speed > 0.0 && speed <= 0.0 {
            apexes.push(world.body_transform(ball).p.y as f32);
        }
        previous_speed = speed;
        step += 1;
    }

    apexes
}

// "head-on bounce matches the requested coefficient for any mass ratio"
#[test]
fn head_on() {
    let densities: [f32; 3] = [1.0, 10.0, 100.0];
    let restitutions: [f32; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];
    let tolerance: f32 = 0.02;

    for density in densities {
        for restitution in restitutions {
            let (coefficient, momentum_error) = measure_head_on(restitution, density);
            assert!(abs_float(coefficient - restitution) <= tolerance);
            assert!(abs_float(momentum_error) <= 1e-4);
        }
    }
}

// "ground bounce is independent of the impact gap"
#[test]
fn ground_bounce() {
    for restitution in [0.5f32, 0.9] {
        for gap in [0.0f32, 0.25, 0.5, 0.75] {
            let measured = measure_ground_bounce(restitution, gap);
            assert!(abs_float(measured - restitution) <= 0.02);
        }
    }
}

// "elastic ball returns to nearly its drop height"
#[test]
fn elastic_drop() {
    let apexes = measure_drop(4.0);
    assert!(apexes.len() >= 2);
    assert!(apexes[0] > 3.9 && apexes[0] < 4.05);
    // Successive apexes stay near the original height with no energy pumping.
    for apex in &apexes {
        assert!(*apex > 3.9 && *apex < 4.05);
    }
}
