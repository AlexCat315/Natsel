//! Ported from Tests/NatselTests/Simulation/WidePrepareTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::functions::min_float;
use crate::math::types::Pos;
use crate::simulation::World;
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::solver_integration::make_soft;
use crate::simulation::solver_integration::StepContext;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::shape_def::ShapeDef;

// "direct SoA prepare matches the scalar prepare lane by lane"
#[test]
fn lanes_match_scalar() {
    let mut w = World::default();
    let mut ground = BodyDef::default();
    let ground_id = w.create_body(&ground);
    let _ = w.create_hull_shape(ground_id, &ShapeDef::default(), &make_box_hull(50.0, 1.0, 50.0));
    for i in 0..12 {
        for j in 0..4 {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(
                j as f64 * 1.1 - 1.5,
                1.5 + i as f64,
                (i % 3) as f64 * 0.3,
            );
            let body = w.create_body(&def);
            let _ = w.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));
        }
    }
    // Swift re-typed the ground `BodyDef` after the shapes were created; the port assigns the
    // same field on the definition copy, which is unused afterwards.
    ground.type_ = BodyType::StaticBody;
    w.prepare_scalar_for_wide_colors = true;

    let mut checked = 0;
    for _ in 0..40 {
        w.step(1.0f32 / 60.0, 4);
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

        for ci in 0..OVERFLOW_INDEX {
            let list = &w.solver_contacts.constraints[ci as usize];
            for c in &w.solver_contacts.wide_constraints[ci as usize] {
                for lane in 0..4usize {
                    let id = c.contact_ids[lane];
                    let Some(s) = list.iter().find(|item| item.contact_id == id) else {
                        continue;
                    };
                    // Every field below must agree bit for bit; the Swift helper compared
                    // `bitPattern` and reported the offending field name.
                    let eq = |a: f32, b: f32, what: &str| {
                        assert!(
                            a.to_bits() == b.to_bits(),
                            "{what} contact {id}: {a} vs {b}"
                        );
                    };
                    eq(c.inv_mass_a[lane], s.inv_mass_a, "invMassA");
                    eq(c.inv_ia.xy[lane], s.inv_ia.cxy, "invIA.xy");
                    eq(c.rolling_mass.xx[lane], s.rolling_mass.cxx, "rollingMass");

                    for offset in 0..s.manifold_count as usize {
                        let m = &w.solver_contacts.wide_manifolds[c.manifold_start + offset];
                        let sm = &w.solver_contacts.manifold_pool[s.manifold_start as usize + offset];
                        eq(m.tangent1.x[lane], sm.tangent1.x, "t1x");
                        eq(m.tangent1.y[lane], sm.tangent1.y, "t1y");
                        eq(m.tangent1.z[lane], sm.tangent1.z, "t1z");
                        eq(m.tangent2.x[lane], sm.tangent2.x, "t2x");
                        eq(m.tangent_velocity1[lane], sm.tangent_velocity1, "tv1");
                        eq(m.twist_impulse[lane], sm.twist_impulse, "twist");
                        eq(m.friction_impulse.x[lane], sm.friction_impulse.x, "fx");
                        eq(m.friction_impulse.y[lane], sm.friction_impulse.y, "fy");
                        eq(m.center_a.x[lane], sm.center_a.x, "cAx");
                        eq(m.center_b.y[lane], sm.center_b.y, "cBy");
                        eq(m.twist_mass[lane], sm.twist_mass, "twistMass");
                        eq(m.tangent_mass.xx[lane], sm.tangent_mass.cx.x, "tmxx");
                        eq(m.tangent_mass.xy[lane], sm.tangent_mass.cx.y, "tmxy");
                        eq(m.tangent_mass.yy[lane], sm.tangent_mass.cy.y, "tmyy");

                        for pi in 0..sm.point_count as usize {
                            let p = &m.points[pi];
                            let sp = &sm.points[pi];
                            eq(p.r_a.x[lane], sp.r_a.x, "rAx");
                            eq(p.base_separation[lane], sp.base_separation, "baseSep");
                            eq(p.normal_impulse[lane], sp.normal_impulse, "normalImpulse");
                            eq(p.normal_mass[lane], sp.normal_mass, "normalMass");
                            eq(p.lever_arm[lane], sp.lever_arm, "leverArm");
                            eq(p.relative_velocity[lane], sp.relative_velocity, "relVel");
                            checked += 1;
                        }
                    }
                }
            }
        }
    }
    assert!(checked > 0);
}

// "skipping scalar twins does not change the simulation"
fn make(twins: bool) -> World {
    let mut w = World::default();
    let ground_id = w.create_body(&BodyDef::default());
    let _ = w.create_hull_shape(ground_id, &ShapeDef::default(), &make_box_hull(50.0, 1.0, 50.0));
    for i in 0..12 {
        for j in 0..4 {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(
                j as f64 * 1.1 - 1.5,
                1.5 + i as f64,
                (i % 3) as f64 * 0.3,
            );
            let body = w.create_body(&def);
            let _ = w.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));
        }
    }
    w.prepare_scalar_for_wide_colors = twins;
    w
}

#[test]
fn scalar_twins_are_inert() {
    let mut a = make(false);
    let mut b = make(true);
    for step in 0..120 {
        a.step(1.0f32 / 60.0, 4);
        b.step(1.0f32 / 60.0, 4);
        assert!(
            a.world_state_hash() == b.world_state_hash(),
            "step {step}"
        );
    }
}
