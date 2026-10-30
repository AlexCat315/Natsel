//! Ported from Tests/NatselTests/Simulation/RestitutionPhaseTests.swift.
//!
//! The core bounce/restitution correctness contract. Every tolerance and step count is
//! load-bearing: they encode the invariant that restitution never returns more speed or energy
//! than it received.

use crate::collision::hull::{make_box_hull, make_cube_hull};
use crate::math::functions::{
    abs_float, dot, inv_rotate_vector, length, make_quat_from_axis_angle, max_float, min_float,
    mul_mv, to_vec3,
};
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;

const TIME_STEP: f32 = 1.0 / 60.0;
const SUB_STEPS: i32 = 4;
const RESTITUTION_ITERATIONS: i32 = 4;
const IMPACT_SPEED: f32 = 5.0;

fn make_world(gravity_y: f32, iterations: Option<i32>) -> World {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, gravity_y, 0.0);
    world.enable_sleep = false;
    if let Some(iterations) = iterations {
        world.restitution_iterations = iterations;
    }
    world
}

/// Ground with its top surface at y = 0.
fn make_ground(world: &mut World, restitution: f32) {
    let mut def = BodyDef::default();
    def.position = Pos::new(0.0, -1.0, 0.0);
    let ground = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = restitution;
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(40.0, 1.0, 40.0));
}

fn make_ball(world: &mut World, x: f32, y: f32, velocity_y: f32, restitution: f32) -> BodyID {
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(x as f64, y as f64, 0.0);
    def.linear_velocity = Vec3::new(0.0, velocity_y, 0.0);
    let ball = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = restitution;
    let _ = world.create_sphere_shape(ball, &shape_def, Sphere::new(Vec3::ZERO, 0.5));
    ball
}

/// Ball driven into static ground with no gravity so the measurement carries no gravity bias.
/// The gap shifts where inside the time step the impact lands.
fn measure_ground_bounce(restitution: f32, gap: f32) -> f32 {
    let mut world = make_world(0.0, None);
    make_ground(&mut world, restitution);
    let ball = make_ball(&mut world, 0.0, 0.5 + gap, -IMPACT_SPEED, restitution);

    for _ in 0..60 {
        world.step(TIME_STEP, SUB_STEPS);
    }
    world.body_linear_velocity(ball).y / IMPACT_SPEED
}

/// Impactor driven onto a column of resting balls pinned against the ground. A supported target
/// has infinite effective mass along the normal, so the coefficient must not depend on the
/// column height.
fn measure_supported_bounce(restitution: f32, support_count: i32, propagate: bool) -> f32 {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_sleep = false;
    world.enable_restitution_propagation = propagate;
    world.restitution_iterations = RESTITUTION_ITERATIONS;

    make_ground(&mut world, 0.0);

    for i in 0..support_count {
        let _ = make_ball(&mut world, 0.0, 0.5 + i as f32, 0.0, 0.0);
    }

    let y = 0.5 + support_count as f32 + 0.5 * IMPACT_SPEED * TIME_STEP;
    let ball = make_ball(&mut world, 0.0, y, -IMPACT_SPEED, restitution);

    for _ in 0..60 {
        world.step(TIME_STEP, SUB_STEPS);
    }
    world.body_linear_velocity(ball).y / IMPACT_SPEED
}

/// The recovered coefficient must not depend on where inside the time step the impact lands.
// "bounce coefficient is stable across impact phases"
#[test]
fn phase() {
    let sample_count: i32 = 16;
    let tolerance: f32 = 0.05;

    for restitution in [0.25f32, 0.5, 0.9] {
        let mut minimum = f32::MAX;
        let mut maximum = -f32::MAX;
        let mut sum: f32 = 0.0;

        for i in 0..sample_count {
            // Sweep the gap across one step of travel so the impact lands at every phase.
            let gap =
                (i as f32 / sample_count as f32) * IMPACT_SPEED * TIME_STEP;
            let measured = measure_ground_bounce(restitution, gap);
            minimum = min_float(minimum, measured);
            maximum = max_float(maximum, measured);
            sum += measured;
        }

        let mean = sum / sample_count as f32;
        let spread = maximum - minimum;

        assert!(spread <= tolerance, "e {} spread {}", restitution, spread);
        assert!(
            abs_float(mean - restitution) <= tolerance,
            "e {} mean {}",
            restitution,
            mean
        );
    }
}

/// A supported target has infinite effective mass along the normal, so the coefficient must not
/// depend on the column height.
// "supported bounce does not depend on column height"
#[test]
fn supported() {
    for propagate in [false, true] {
        let floor_factor: f32 = if propagate { 0.75 } else { 0.5 };

        for restitution in [0.5f32, 0.9] {
            for count in 0..=3 {
                let measured = measure_supported_bounce(restitution, count, propagate);
                let floor = floor_factor * restitution;

                assert!(
                    measured <= restitution + 0.01,
                    "e {} n {}",
                    restitution,
                    count
                );
                assert!(
                    measured >= floor,
                    "e {} n {} got {}",
                    restitution,
                    count,
                    measured
                );
            }
        }
    }
}

// "threshold gates restitution for slow contacts"
#[test]
fn threshold() {
    // With a high restitution threshold a slow approach does not bounce.
    let mut world = make_world(0.0, None);
    make_ground(&mut world, 0.9);
    world.restitution_threshold = 10.0;

    let ball = make_ball(&mut world, 0.0, 0.5, -1.0, 0.9);
    for _ in 0..60 {
        world.step(TIME_STEP, SUB_STEPS);
    }

    // The approach speed is below the threshold, so no restitution is applied.
    let v = world.body_linear_velocity(ball).y;
    assert!(v <= 0.01);
}

// "a fast contact above the threshold still bounces"
#[test]
fn threshold_above() {
    let mut world = make_world(0.0, None);
    make_ground(&mut world, 0.9);
    world.restitution_threshold = 1.0;

    let ball = make_ball(&mut world, 0.0, 0.5, -IMPACT_SPEED, 0.9);
    for _ in 0..60 {
        world.step(TIME_STEP, SUB_STEPS);
    }

    let measured = world.body_linear_velocity(ball).y / IMPACT_SPEED;
    assert!(measured > 0.85);
}

// MARK: - Head-on collisions

/// Two free spheres closing head on with no gravity. The coefficient is the ratio of relative
/// normal speeds at the contact, which is the mass independent definition.
fn measure_head_on(restitution: f32, density_b: f32) -> (f32, f32) {
    let mut world = make_world(0.0, None);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = restitution;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(-1.0, 0.0, 0.0);
    def.linear_velocity = Vec3::new(0.5 * IMPACT_SPEED, 0.0, 0.0);
    let id_a = world.create_body(&def);
    let _ = world.create_sphere_shape(id_a, &shape_def, Sphere::new(Vec3::ZERO, 0.5));

    def.position = Pos::new(1.0, 0.0, 0.0);
    def.linear_velocity = Vec3::new(-0.5 * IMPACT_SPEED, 0.0, 0.0);
    let id_b = world.create_body(&def);
    shape_def.density = density_b;
    let _ = world.create_sphere_shape(id_b, &shape_def, Sphere::new(Vec3::ZERO, 0.5));

    let mass_a = world.body_mass(id_a);
    let mass_b = world.body_mass(id_b);
    let momentum_0 =
        mass_a * world.body_linear_velocity(id_a).x + mass_b * world.body_linear_velocity(id_b).x;

    for _ in 0..120 {
        world.step(TIME_STEP, SUB_STEPS);
    }

    let v_a = world.body_linear_velocity(id_a).x;
    let v_b = world.body_linear_velocity(id_b).x;
    let momentum_1 = mass_a * v_a + mass_b * v_b;

    let momentum_error = (momentum_1 - momentum_0) / (mass_a + mass_b);
    ((v_b - v_a) / IMPACT_SPEED, momentum_error)
}

/// The coefficient must not depend on the mass ratio, and momentum must be conserved.
// "head-on restitution is mass independent and conserves momentum"
#[test]
fn head_on() {
    let tolerance: f32 = 0.02;
    let momentum_tolerance: f32 = 1e-4;

    for density in [1.0f32, 10.0, 100.0] {
        for restitution in [0.0f32, 0.25, 0.5, 0.75, 1.0] {
            let result = measure_head_on(restitution, density);
            assert!(
                abs_float(result.0 - restitution) <= tolerance,
                "density {} e {} got {}",
                density,
                restitution,
                result.0
            );
            assert!(
                abs_float(result.1) <= momentum_tolerance,
                "density {} e {} momentum drift {}",
                density,
                restitution,
                result.1
            );
        }
    }
}

// MARK: - Elastic overshoot

struct OvershootResult {
    first_apex: f32,
    first_energy_ratio: f32,
    peak_energy_ratio: f32,
    speed_ratio: f32,
    bounce_count: i32,
}

impl Default for OvershootResult {
    fn default() -> Self {
        Self {
            first_apex: 0.0,
            first_energy_ratio: 0.0,
            peak_energy_ratio: 0.0,
            speed_ratio: 0.0,
            bounce_count: 0,
        }
    }
}

fn measure_energy(world: &World, body: BodyID) -> f32 {
    let mass_data = world.body_mass_data(body);
    let v = world.body_linear_velocity(body);
    let gravity_scale = world.body_gravity_scale(body);
    let gravity = gravity_scale * world.gravity;
    let h = TIME_STEP / SUB_STEPS as f32;

    // The inertia tensor is in the body frame, so bring the angular velocity back to it.
    let w_local = inv_rotate_vector(world.body_rotation(body), world.body_angular_velocity(body));

    let kinetic = 0.5 * mass_data.mass * dot(v, v)
        + 0.5 * dot(w_local, mul_mv(mass_data.inertia, w_local));
    let potential = -mass_data.mass * dot(gravity, to_vec3(world.body_world_center(body)));
    let phase = 0.5 * h * mass_data.mass * dot(v, gravity);

    kinetic + potential + phase
}

/// A unit cube dropped flat on a floor narrower than itself, perfectly elastic, so the four
/// contact points sit inboard of the box corners. A perfectly elastic bounce cannot come back
/// higher than it was dropped from and cannot gain energy.
fn measure_overshoot(continuous: bool) -> OvershootResult {
    let mut world = make_world(-10.0, Some(RESTITUTION_ITERATIONS));

    let mut floor_def = BodyDef::default();
    floor_def.position = Pos::new(0.0, -0.25, 0.0);
    let floor = world.create_body(&floor_def);
    let _ = world.create_hull_shape(
        floor,
        &ShapeDef::default(),
        &make_box_hull(0.375, 0.25, 0.375),
    );

    let drop_height: f32 = 10.0;

    let mut box_def = BodyDef::default();
    box_def.type_ = BodyType::DynamicBody;
    box_def.position = Pos::new(0.0, drop_height as f64, 0.0);
    if continuous {
        box_def.safety_factor = 0.1;
    }
    let box_body = world.create_body(&box_def);

    let mut box_shape = ShapeDef::default();
    box_shape.base_material.restitution = 1.0;
    let _ = world.create_hull_shape(box_body, &box_shape, &make_cube_hull(0.5));

    let mut result = OvershootResult::default();
    let start_energy = measure_energy(&world, box_body);

    let mut impact_speed: f32 = 0.0;
    let mut rebound_speed: f32 = 0.0;
    let mut previous_speed: f32 = 0.0;

    let mut step = 0;
    while step < 600 && result.bounce_count < 2 {
        let before = world.body_linear_velocity(box_body).y;
        world.step(TIME_STEP, SUB_STEPS);
        let speed = world.body_linear_velocity(box_body).y;

        if rebound_speed == 0.0 && before < 0.0 && speed > 0.0 {
            impact_speed = -before;
            rebound_speed = speed;
        }

        if rebound_speed == 0.0 {
            previous_speed = speed;
            step += 1;
            continue;
        }

        // Every step, not just at the apexes, because the impact step is where gain appears.
        let ratio = measure_energy(&world, box_body) / start_energy;
        if ratio > result.peak_energy_ratio {
            result.peak_energy_ratio = ratio;
        }

        if previous_speed > 0.0 && speed <= 0.0 {
            if result.bounce_count == 0 {
                result.first_apex = world.body_transform(box_body).p.y as f32;
                result.first_energy_ratio = ratio;
            }
            result.bounce_count += 1;
        }

        previous_speed = speed;
        step += 1;
    }

    result.speed_ratio = if impact_speed != 0.0 {
        rebound_speed / impact_speed
    } else {
        0.0
    };
    result
}

/// The impulse must never return more speed than it received. That is the restitution invariant.
// "a perfectly elastic box never gains speed or energy"
#[test]
fn overshoot() {
    let drop_height: f32 = 10.0;
    let speed_tolerance: f32 = 1e-5;

    let discrete = measure_overshoot(false);
    let continuous = measure_overshoot(true);

    // Perfectly elastic returns exactly the speed it received.
    assert!(
        discrete.speed_ratio <= 1.0 + speed_tolerance,
        "discrete {}",
        discrete.speed_ratio
    );
    assert!(
        continuous.speed_ratio <= 1.0 + speed_tolerance,
        "continuous {}",
        continuous.speed_ratio
    );

    // With continuous collision the cube lands on the surface and stays under its start.
    assert!(
        continuous.first_apex <= drop_height,
        "continuous apex {}",
        continuous.first_apex
    );
    assert!(
        continuous.peak_energy_ratio <= 1.001,
        "continuous energy {}",
        continuous.peak_energy_ratio
    );

    // The discrete run is gated more loosely: it lands already inside the floor.
    assert!(
        discrete.first_apex <= drop_height + 0.15,
        "discrete apex {}",
        discrete.first_apex
    );
    assert!(
        discrete.first_energy_ratio <= 1.02,
        "discrete energy {}",
        discrete.first_energy_ratio
    );

    assert!(discrete.bounce_count >= 2);
    assert!(continuous.bounce_count >= 1);
}

// MARK: - Energy conservation

#[derive(Clone, Debug)]
struct EnergyScene {
    name: &'static str,
    safety_factor: f32,
    friction: f32,
    angle: f32,
    spin: f32,
    height: f32,
    velocity_y: f32,
    half_width: f32,
    half_height: f32,
    bullet: bool,
    stack: bool,
    step_count: i32,
}

impl EnergyScene {
    /// Swift's memberwise init defaults `bullet = false`, `stack = false`, `stepCount = 300`.
    #[allow(clippy::too_many_arguments)]
    fn new(
        name: &'static str,
        safety_factor: f32,
        friction: f32,
        angle: f32,
        spin: f32,
        height: f32,
        velocity_y: f32,
        half_width: f32,
        half_height: f32,
        step_count: i32,
    ) -> Self {
        Self {
            name,
            safety_factor,
            friction,
            angle,
            spin,
            height,
            velocity_y,
            half_width,
            half_height,
            bullet: false,
            stack: false,
            step_count,
        }
    }

    fn with_flags(mut self, bullet: bool, stack: bool) -> Self {
        self.bullet = bullet;
        self.stack = stack;
        self
    }
}

/// Peak energy over the run relative to the start, for every dynamic body in the scene.
/// Perfectly elastic and the energy may still only go down.
fn measure_peak_energy(scene: &EnergyScene) -> f32 {
    let mut world = make_world(-10.0, None);

    let mut ground_def = BodyDef::default();
    ground_def.position = Pos::new(0.0, -1.0, 0.0);
    let ground = world.create_body(&ground_def);
    let mut ground_shape = ShapeDef::default();
    ground_shape.base_material.friction = scene.friction;
    let _ = world.create_hull_shape(ground, &ground_shape, &make_box_hull(40.0, 1.0, 40.0));

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.base_material.friction = scene.friction;
    shape_def.base_material.restitution = 1.0;
    let box_hull = make_box_hull(scene.half_width, scene.half_height, scene.half_width);

    let mut bodies: Vec<BodyID> = Vec::new();

    if scene.stack {
        // A resting box under the dropped one.
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, scene.half_height as f64, 0.0);
        def.safety_factor = scene.safety_factor;
        let body = world.create_body(&def);
        let _ = world.create_hull_shape(body, &shape_def, &box_hull);
        bodies.push(body);
    }

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, scene.height as f64, 0.0);
    def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Z, scene.angle);
    def.angular_velocity = Vec3::new(0.0, 0.0, scene.spin);
    def.linear_velocity = Vec3::new(0.0, scene.velocity_y, 0.0);
    def.safety_factor = scene.safety_factor;
    def.is_bullet = scene.bullet;
    let dropped = world.create_body(&def);
    let _ = world.create_hull_shape(dropped, &shape_def, &box_hull);
    bodies.push(dropped);

    let mut start: f32 = 0.0;
    for body in &bodies {
        start += measure_energy(&world, *body);
    }

    let mut peak = start;
    for _ in 0..scene.step_count {
        world.step(TIME_STEP, SUB_STEPS);
        let mut energy: f32 = 0.0;
        for body in &bodies {
            energy += measure_energy(&world, *body);
        }
        peak = max_float(peak, energy);
    }

    peak / start
}

/// Perfectly elastic material with friction, dropped every way that used to gain energy.
/// Energy must never rise above the start: this is the load bearing property of restitution.
// "peak energy never exceeds the starting energy"
#[test]
fn energy_never_grows() {
    let scenes: Vec<EnergyScene> = vec![
        EnergyScene::new(
            "spinning edge", 0.01, 0.6, -1.43, 7.9, 0.7, -1.9, 0.5, 0.5, 300,
        ),
        EnergyScene::new(
            "spinning edge fast", 0.01, 0.6, -0.8, 12.0, 3.0, -8.0, 0.5, 0.5, 300,
        ),
        EnergyScene::new("tumble", 0.01, 0.6, -0.3, 3.0, 8.0, 0.0, 0.5, 0.5, 600),
        EnergyScene::new(
            "tumble discrete", 0.5, 0.6, -0.3, 3.0, 8.0, 0.0, 0.5, 0.5, 600,
        ),
        EnergyScene::new(
            "flat with spin", 0.01, 0.6, 0.0, 2.0, 2.0, -5.0, 0.5, 0.5, 300,
        ),
        EnergyScene::new(
            "flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.5, 0.5, 1200,
        ),
        EnergyScene::new("edge", 0.01, 0.6, -0.2, 0.0, 5.0, 0.0, 0.5, 0.5, 600),
        EnergyScene::new("two box", 0.02, 0.6, 0.0, 0.0, 4.0, 0.0, 0.5, 0.5, 1500)
            .with_flags(true, true),
        EnergyScene::new(
            "two box discrete", 0.5, 0.6, 0.0, 0.0, 4.0, 0.0, 0.5, 0.5, 1500,
        )
        .with_flags(false, true),
        EnergyScene::new(
            "tall flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.5, 1.0, 1200,
        ),
        EnergyScene::new(
            "tall flat friction", 0.01, 0.6, 0.0, 0.0, 10.0, 0.0, 0.5, 1.0, 1200,
        ),
        EnergyScene::new(
            "narrow flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.25, 1.0, 1200,
        ),
        EnergyScene::new(
            "narrow flat friction", 0.01, 0.6, 0.0, 0.0, 10.0, 0.0, 0.25, 1.0, 1200,
        ),
    ];

    // The soft contact position correction lifts a body that lands inside the ground, so
    // continuous collision keeps the tolerance tight. A four point flat landing does not
    // converge at the default two stage iterations the way a two point landing does.
    let tolerance: f32 = 5.0e-3;
    let discrete_tolerance: f32 = 2.0e-2;

    for scene in &scenes {
        let ratio = measure_peak_energy(scene);
        let limit = if scene.safety_factor < 0.1 {
            tolerance
        } else {
            discrete_tolerance
        };
        assert!(ratio <= 1.0 + limit, "{} peak {}", scene.name, ratio);
    }
}

// MARK: - Drop impulse accounting

#[derive(Clone, Debug, Default)]
struct ImpulseResult {
    worst_error: f32,
    approach_speed: f32,
    first_separation: f32,
    bounce_impulse: f32,
    contact_steps: i32,
    toi_steps: i32,
    toi_impulse_steps: i32,
}

/// Drop a ball and account for every contact impulse: the measured normal impulse must equal
/// the momentum change minus the gravity contribution for that step.
fn measure_drop_impulse(restitution: f32, drop_height: f32) -> ImpulseResult {
    let mut world = make_world(-10.0, None);
    make_ground(&mut world, 0.0);

    let ball = make_ball(&mut world, 0.0, 0.5 + drop_height, 0.0, restitution);

    let mass = world.body_mass(ball);
    let gravity_y = world.gravity.y;

    let mut result = ImpulseResult::default();
    let mut touched = false;
    let mut bouncing = false;

    for _ in 0..240 {
        let speed_before = world.body_linear_velocity(ball).y;
        world.step(TIME_STEP, SUB_STEPS);
        let speed_after = world.body_linear_velocity(ball).y;

        let mut measured: f32 = 0.0;
        let mut separation: f32 = 0.0;
        let contact_data = world.body_contact_data(ball, 4);
        for contact in &contact_data {
            for manifold in &contact.manifolds {
                if manifold.point_count <= 0 {
                    continue;
                }
                for p in 0..(manifold.point_count as usize) {
                    measured += manifold.points[p].total_normal_impulse;
                    separation = manifold.points[p].separation;
                }
            }
        }

        // Transient body flags are cleared and rewritten every step.
        if world.is_body_time_of_impact(ball) {
            result.toi_steps += 1;
            if measured != 0.0 {
                result.toi_impulse_steps += 1;
            }
            continue;
        }

        let expected = mass * (speed_after - speed_before) - mass * gravity_y * TIME_STEP;
        result.worst_error = max_float(result.worst_error, abs_float(measured - expected));

        if !contact_data.is_empty() {
            if !touched {
                touched = true;
                bouncing = true;
                result.approach_speed = -speed_before;
                result.first_separation = separation;
            }
            if bouncing {
                result.bounce_impulse += measured;
                result.contact_steps += 1;
            }
        } else {
            bouncing = false;
        }
    }

    result
}

/// The measured contact impulse must match the momentum change to solver precision, and the
/// bounce impulse must stay within the analytic reversal plus the weight carried over the
/// contact steps.
// "contact impulse accounts for momentum change and gravity"
#[test]
fn impulse_accounting() {
    // The impact speed at which continuous collision engages.
    let mut probe_world = make_world(-10.0, None);
    make_ground(&mut probe_world, 0.0);
    let probe = make_ball(&mut probe_world, 0.0, 0.5, 0.0, 0.0);
    let mass = probe_world.body_mass(probe);
    let fast_speed = probe_world.body_safety_factor(probe) * probe_world.body_min_extent(probe)
        / TIME_STEP;

    let weight_impulse = mass * 10.0 * TIME_STEP;

    for drop_height in [0.25f32, 1.0, 4.0, 16.0] {
        let impact_speed = (20.0 * drop_height).sqrt();
        let expect_toi = impact_speed > fast_speed;
        let tolerance = 1e-5 * mass * (10.0 + impact_speed);

        for restitution in [0.0f32, 0.5, 1.0] {
            let result = measure_drop_impulse(restitution, drop_height);

            let reversal = (1.0 + restitution) * mass * result.approach_speed;
            let lower = reversal - weight_impulse;
            let upper = reversal + weight_impulse * result.contact_steps as f32;

            assert!(
                result.worst_error <= tolerance,
                "drop {} e {} error {}",
                drop_height,
                restitution,
                result.worst_error
            );

            assert!(
                result.contact_steps > 0,
                "drop {} e {} no contact",
                drop_height,
                restitution
            );
            assert!(
                result.toi_impulse_steps == 0,
                "drop {} e {} impulse during TOI",
                drop_height,
                restitution
            );

            let slack = result.contact_steps as f32 * tolerance;
            assert!(
                result.bounce_impulse >= lower - slack
                    && result.bounce_impulse <= upper + slack,
                "drop {} e {} bounce {} in [{}, {}]",
                drop_height,
                restitution,
                result.bounce_impulse,
                lower,
                upper
            );

            // Continuous collision engages exactly when the impact speed clears the threshold.
            assert_eq!(
                result.toi_steps > 0, expect_toi,
                "drop {} e {} toi {} expected {}",
                drop_height, restitution, result.toi_steps, expect_toi
            );
        }
    }
}

// MARK: - Sampled normal velocity

fn measure_first_touch_normal_velocity(enable_hit_events: bool) -> f32 {
    let mut world = make_world(0.0, None);
    make_ground(&mut world, 0.0);

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.6, 0.0);
    let ball = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = 0.0;
    shape_def.enable_hit_events = enable_hit_events;
    let _ = world.create_sphere_shape(ball, &shape_def, Sphere::new(Vec3::ZERO, 0.5));

    let mut normal_velocity = f32::MAX;

    // Hold the closing speed so the sampled value does not depend on which step the pair is
    // created.
    for _ in 0..20 {
        world.set_body_linear_velocity(ball, Vec3::new(0.0, -IMPACT_SPEED, 0.0));
        world.step(TIME_STEP, SUB_STEPS);

        let contacts = world.body_contact_data(ball, 1);
        if let Some(first) = contacts.first() {
            if !first.manifolds.is_empty() {
                let manifold = &first.manifolds[0];
                if manifold.point_count > 0 {
                    normal_velocity = manifold.points[0].normal_velocity;
                    break;
                }
            }
        }
    }
    normal_velocity
}

/// The approach speed is only sampled when hit events are enabled; otherwise the field stays
/// zero. This guards the shared prepare path from paying for a sample nobody reads.
// "normal velocity is only sampled when hit events are enabled"
#[test]
fn normal_velocity_sampling() {
    let quiet = measure_first_touch_normal_velocity(false);
    let published = measure_first_touch_normal_velocity(true);

    assert_eq!(quiet, 0.0);

    // The closing speed is reported as a negative normal velocity, at nearly full magnitude.
    assert!(
        published <= -0.8 * IMPACT_SPEED,
        "published {}",
        published
    );
}

// MARK: - Flat landing and spin

/// A flat box landing on all four corners at once.
fn measure_flat_bounce(restitution: f32, sub_steps: i32) -> (f32, f32) {
    let mut world = make_world(0.0, Some(RESTITUTION_ITERATIONS));
    make_ground(&mut world, 0.0);

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(
        0.0,
        (0.25 + 0.5 * IMPACT_SPEED * TIME_STEP) as f64,
        0.0,
    );
    def.linear_velocity = Vec3::new(0.0, -IMPACT_SPEED, 0.0);
    let box_body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = restitution;
    let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(1.0, 0.25, 1.0));

    for _ in 0..60 {
        world.step(TIME_STEP, sub_steps);
    }

    let speed = world.body_linear_velocity(box_body).y;
    let spin = length(world.body_angular_velocity(box_body));
    (speed / IMPACT_SPEED, spin)
}

/// A symmetric four point landing. The stage solves the points in sequence, so a single sweep
/// leaves the body tilted; four stage iterations bring the coefficient within a few thousandths.
// "four point flat landing recovers the coefficient without spin"
#[test]
fn flat_landing() {
    let speed_tolerance: f32 = 0.01;
    let spin_tolerance: f32 = 0.03;

    for restitution in [0.5f32, 0.9] {
        for sub_steps in [4i32, 8] {
            let result = measure_flat_bounce(restitution, sub_steps);

            // Only the shipping sub step count is a gate; the wider count is informational.
            if sub_steps != SUB_STEPS {
                continue;
            }

            assert!(
                abs_float(result.0 - restitution) <= speed_tolerance,
                "e {} coefficient {}",
                restitution,
                result.0
            );
            assert!(
                result.1 <= spin_tolerance,
                "e {} spin {}",
                restitution,
                result.1
            );
        }
    }
}

/// A flat box carrying spin about a horizontal axis. All four corners stay in contact and the
/// normal constraints are linear in the body velocity, so perfect restitution reverses linear
/// and angular velocity alike.
fn measure_spinning_bounce(spin: f32, restitution: f32) -> (f32, f32) {
    let mut world = make_world(0.0, Some(RESTITUTION_ITERATIONS));
    make_ground(&mut world, 0.0);

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(
        0.0,
        (0.25 + 0.5 * IMPACT_SPEED * TIME_STEP) as f64,
        0.0,
    );
    def.linear_velocity = Vec3::new(0.0, -IMPACT_SPEED, 0.0);
    def.angular_velocity = Vec3::new(spin, 0.0, 0.0);
    let box_body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.base_material.friction = 0.0;
    shape_def.base_material.restitution = restitution;
    let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(1.0, 0.25, 1.0));

    for _ in 0..60 {
        world.step(TIME_STEP, SUB_STEPS);
    }

    let speed = world.body_linear_velocity(box_body).y;
    let residual = length(world.body_angular_velocity(box_body));
    (speed / IMPACT_SPEED, residual)
}

// "spin about a horizontal axis does not disturb the flat bounce"
#[test]
fn spin_along_axis() {
    for spin in [0.0f32, 0.5, 1.0, 2.0] {
        let result = measure_spinning_bounce(spin, 0.9);
        // The bounce coefficient stays close to the nominal value regardless of spin.
        assert!(
            abs_float(result.0 - 0.9) <= 0.05,
            "spin {} coefficient {}",
            spin,
            result.0
        );
    }
}

// MARK: - Multi-bounce apexes and resting stability

/// A perfectly elastic ball dropped under gravity. Records each bounce apex.
fn measure_apexes(drop_height: f32, capacity: usize) -> Vec<f32> {
    let mut world = make_world(-10.0, None);
    make_ground(&mut world, 0.0);

    let ball = make_ball(&mut world, 0.0, drop_height, 0.0, 1.0);

    let mut apexes: Vec<f32> = Vec::new();
    let mut previous_speed: f32 = 0.0;

    let mut step = 0;
    while step < 4000 && apexes.len() < capacity {
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

/// A later apex above the first, or a first apex above the drop height, is energy from nowhere.
// "perfectly elastic drops never exceed their drop height"
#[test]
fn drop_apexes() {
    for drop_height in [40.0f32, 20.0, 10.0] {
        let apexes = measure_apexes(drop_height, 6);
        assert!(
            apexes.len() >= 6,
            "drop {} only {} apexes",
            drop_height,
            apexes.len()
        );
        if apexes.len() < 6 {
            continue;
        }

        let highest = apexes.iter().copied().fold(0.0f32, f32::max);
        assert!(
            highest <= 1.001 * apexes[0],
            "drop {} highest {} first {}",
            drop_height,
            highest,
            apexes[0]
        );
        assert!(
            apexes[0] <= 1.001 * drop_height,
            "drop {} first apex {}",
            drop_height,
            apexes[0]
        );
        // The ball must still be bouncing at the end of the run.
        assert!(
            apexes[apexes.len() - 1] >= 0.8 * apexes[0],
            "drop {} decayed",
            drop_height
        );
    }
}

/// A tall stack of nearly elastic boxes must settle and stay settled rather than creep.
// "a resting elastic stack does not drift or jitter"
#[test]
fn resting_stack() {
    let mut world = make_world(-10.0, None);

    // The ground itself is bouncy here.
    let mut ground_def = BodyDef::default();
    ground_def.position = Pos::new(0.0, -1.0, 0.0);
    let ground = world.create_body(&ground_def);
    let mut ground_shape = ShapeDef::default();
    ground_shape.base_material.friction = 0.0;
    ground_shape.base_material.restitution = 0.9;
    let _ = world.create_hull_shape(ground, &ground_shape, &make_box_hull(40.0, 1.0, 40.0));

    let box_count = 10;
    let mut boxes: Vec<BodyID> = Vec::new();

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.base_material.friction = 0.6;
    shape_def.base_material.restitution = 0.9;

    let cube = make_cube_hull(0.5);
    for i in 0..box_count {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, (0.5 + i as f32) as f64, 0.0);
        let box_body = world.create_body(&def);
        let _ = world.create_hull_shape(box_body, &shape_def, &cube);
        boxes.push(box_body);
    }

    // Let the stack settle.
    for _ in 0..240 {
        world.step(TIME_STEP, SUB_STEPS);
    }

    let settled = world.body_transform(boxes[(box_count - 1) as usize]).p.y as f32;
    let mut drift: f32 = 0.0;
    let mut peak_speed: f32 = 0.0;

    for _ in 0..300 {
        world.step(TIME_STEP, SUB_STEPS);
        let y = world.body_transform(boxes[(box_count - 1) as usize]).p.y as f32;
        drift = max_float(drift, (y - settled).abs());
        for box_body in &boxes {
            peak_speed = max_float(peak_speed, length(world.body_linear_velocity(*box_body)));
        }
    }

    assert!(drift < 0.01, "drift {}", drift);
    assert!(peak_speed < 0.05, "peak speed {}", peak_speed);
}
