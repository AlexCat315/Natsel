//! Ported from Tests/NatselTests/Simulation/WideStageTests.swift.

use crate::collision::hull::make_box_hull;
use crate::collision::mesh::procedural::create_wave_mesh;
use crate::math::functions::min_float;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::solver_integration::{make_soft, StepContext};
use crate::simulation::world_runtime::SetType;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::shape_def::ShapeDef;

/// `ImpulseDeterminismTests.makeMeshMix(_:)`, reproduced here because the ported helper in
/// `impulse_determinism_tests` is private to that module. Literals match the Swift original.
fn make_mesh_mix(wide: bool) -> World {
    let mut world = World::default();
    world.enable_wide_solver = wide;
    let mesh = create_wave_mesh(40, 40, 0.5, 0.3, 0.2, 0.3).expect("wave mesh");
    let mut ground_def = BodyDef::default();
    ground_def.position = Pos::new(-10.0, 0.0, -10.0);
    let ground = world.create_body(&ground_def);
    let _ = world.create_mesh_shape(ground, &ShapeDef::default(), mesh, Vec3::ONE);

    let small_box = make_box_hull(0.2, 0.2, 0.2);
    let large_box = make_box_hull(0.9, 0.3, 0.9);
    let sphere = Sphere::new(Vec3::ZERO, 0.3);
    let capsule = Capsule::new(Vec3::new(-0.5, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0), 0.2);

    let mut seed: u32 = 314_159;
    for i in 0..64 {
        let row = i / 8;
        let column = i % 8;
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(2.2 * column as f64 - 7.7, 1.2, 2.2 * row as f64 - 7.7);
        let id = world.create_body(&def);

        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let mut shape_def = ShapeDef::default();
        shape_def.density = 1.0;
        shape_def.base_material.rolling_resistance = if (seed >> 28) < 5 { 0.2 } else { 0.0 };
        match (seed >> 24) & 3 {
            0 => {
                let _ = world.create_hull_shape(id, &shape_def, &large_box);
            }
            1 => {
                let _ = world.create_hull_shape(id, &shape_def, &small_box);
            }
            2 => {
                let _ = world.create_sphere_shape(id, &shape_def, sphere);
            }
            _ => {
                let _ = world.create_capsule_shape(id, &shape_def, capsule);
            }
        }

        let sign: f32 = if (i & 1) != 0 { -1.0 } else { 1.0 };
        world.set_body_linear_velocity(id, Vec3::new(-0.0, -0.5, -0.0));
        world.set_body_angular_velocity(id, Vec3::new(-0.0, sign * 0.5, -0.0));
    }
    world
}

/// Run `action` on the scalar path and then on the wide path from the same prepared state, and
/// require every body velocity and every stored impulse to agree bit for bit. Only one path runs
/// at a time, so this needs no borrow splitting.
fn compare(w: &mut World, phase: &str, action: &mut dyn FnMut(&mut World)) {
    let awake = SetType::AwakeSet.raw_value() as usize;

    let states = w.solver_sets[awake].body_states.clone();
    let pool = w.solver_contacts.manifold_pool.clone();

    w.enable_wide_solver = false;
    action(w);
    let expected = w.solver_sets[awake].body_states.clone();
    let expected_pool = w.solver_contacts.manifold_pool.clone();

    w.solver_sets[awake].body_states = states;
    w.solver_contacts.manifold_pool = pool;
    w.enable_wide_solver = true;
    action(w);
    w.unpack_wide_impulses();

    let actual = w.solver_sets[awake].body_states.clone();
    for i in 0..expected.len() {
        for (a, b) in [
            (expected[i].linear_velocity, actual[i].linear_velocity),
            (expected[i].angular_velocity, actual[i].angular_velocity),
        ] {
            assert!(
                a.x.to_bits() == b.x.to_bits()
                    && a.y.to_bits() == b.y.to_bits()
                    && a.z.to_bits() == b.z.to_bits(),
                "{phase} body {i} {a:?} != {b:?}"
            );
        }
    }

    for i in 0..expected_pool.len() {
        let a = expected_pool[i];
        let b = w.solver_contacts.manifold_pool[i];
        assert!(
            a.friction_impulse.x.to_bits() == b.friction_impulse.x.to_bits()
                && a.friction_impulse.y.to_bits() == b.friction_impulse.y.to_bits()
                && a.twist_impulse.to_bits() == b.twist_impulse.to_bits()
                && a.rolling_impulse == b.rolling_impulse,
            "{phase} manifold {i} friction {:?} != {:?}, rolling {:?} != {:?}",
            a.friction_impulse,
            b.friction_impulse,
            a.rolling_impulse,
            b.rolling_impulse
        );
        for pi in 0..a.point_count as usize {
            assert!(
                a.points[pi].normal_impulse == b.points[pi].normal_impulse,
                "{phase} manifold {i} point {pi} normal"
            );
        }
    }
}

// "mesh stages preserve every body velocity and impulse"
#[test]
fn stages() {
    let mut w = make_mesh_mix(false);
    w.prepare_scalar_for_wide_colors = true;
    for _ in 0..69 {
        w.step(1.0f32 / 60.0, 4);
    }
    w.enable_wide_solver = true;
    w.update_broad_phase_pairs();

    let mut ctx = StepContext::default();
    ctx.dt = 1.0f32 / 60.0;
    ctx.h = ctx.dt / 4.0;
    ctx.inv_dt = 60.0;
    ctx.inv_h = 240.0;
    ctx.sub_step_count = 4;
    ctx.enable_warm_starting = true;
    ctx.max_linear_velocity = w.max_linear_speed;
    let hertz = min_float(w.contact_hertz, 0.125 * ctx.inv_h);
    ctx.contact_softness = make_soft(hertz, w.contact_damping_ratio, ctx.h);
    ctx.static_softness = make_soft(2.0 * hertz, 0.5 * w.contact_damping_ratio, ctx.h);

    w.collide(&mut ctx);
    w.prepare_all_contacts(&ctx);

    for substep in 0..4 {
        w.integrate_velocities(ctx.h);
        let mut colors: Vec<i32> = vec![OVERFLOW_INDEX];
        colors.extend(0..OVERFLOW_INDEX);
        for ci in colors {
            let ctx_warm = ctx.clone();
            compare(&mut w, &format!("warm {substep} color {ci}"), &mut |w: &mut World| {
                w.warm_start_joints(ci, &ctx_warm);
            });
            for mesh in [false, true] {
                compare(
                    &mut w,
                    &format!("warm {substep} color {ci} mesh {mesh}"),
                    &mut |w: &mut World| {
                        w.warm_start_contacts(ci, mesh);
                    },
                );
            }
        }
        let ctx_push = ctx.clone();
        compare(&mut w, &format!("push {substep}"), &mut |w: &mut World| {
            w.push_all(&ctx_push);
        });
        w.integrate_positions(ctx.h, ctx.inv_dt);
        let ctx_solve = ctx.clone();
        compare(&mut w, &format!("solve {substep}"), &mut |w: &mut World| {
            w.solve_all(&ctx_solve);
        });
    }
}
