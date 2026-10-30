//! Ported from Tests/NatselTests/Simulation/OverflowColorTests.swift.

use crate::base::constants as b3;
use crate::collision::hull::{make_box_hull, Hull};
use crate::math::functions::compute_cos_sin;
use crate::math::types::{Pos, Vec3};
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;

const RING_COUNT: i32 = 5;
const PER_RING: i32 = 5;

fn create_overflow_color_pile(world: &mut World) -> (BodyID, i32) {
    // Static ground (top surface at y = 0)
    let mut def = BodyDef::default();
    def.position = Pos::new(0.0, -1.0, 0.0);
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(20.0, 1.0, 20.0));

    // Tall, heavy hub so neighbors can ring around it in several vertical layers.
    let hub_half_x: f32 = 0.5;
    let hub_half_y: f32 = 2.5;
    let hub_half_z: f32 = 0.5;

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, hub_half_y as f64, 0.0);
    let hub = world.create_body(&def);

    shape_def = ShapeDef::default();
    shape_def.density = 50.0;
    let _ = world.create_hull_shape(
        hub,
        &shape_def,
        &make_box_hull(hub_half_x, hub_half_y, hub_half_z),
    );

    // Neighbors: vertical rings, each box slightly overlapping the hub.
    let neighbor_half: f32 = 0.2;
    let ring_radius = hub_half_x + neighbor_half - 0.03;
    let neighbor_shape = ShapeDef::default();

    let ring_spacing: f32 = 0.5;
    let base_y = neighbor_half + 0.05;

    let mut neighbor_count = 0;
    let neighbor_box: Hull = make_box_hull(neighbor_half, neighbor_half, neighbor_half);
    for ring in 0..RING_COUNT {
        let y = base_y + ring_spacing * ring as f32;

        // Offset alternate rings by half a slot so neighbors don't sit directly above each other.
        let theta_offset: f32 = if (ring & 1) != 0 {
            b3::PI / PER_RING as f32
        } else {
            0.0
        };

        for slot in 0..PER_RING {
            let theta = theta_offset + (2.0 * b3::PI * slot as f32) / PER_RING as f32;
            let cs = compute_cos_sin(theta);

            def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(
                (ring_radius * cs.cosine) as f64,
                y as f64,
                (ring_radius * cs.sine) as f64,
            );
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(body, &neighbor_shape, &neighbor_box);
            neighbor_count += 1;
        }
    }

    (hub, neighbor_count)
}

// "a pile around one hub populates the overflow color"
#[test]
fn overflow_color_pile() {
    let mut world = World::default();
    let (_, neighbors) = create_overflow_color_pile(&mut world);
    assert_eq!(neighbors, RING_COUNT * PER_RING);

    // One step would trip the asserts; several steps also exercise warm starting.
    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }

    let counters = world.counters();

    // Confirm the scene actually populated the overflow color. Without this a future change
    // to graph coloring could silently turn the test into a no-op.
    let overflow_index = b3::GRAPH_COLOR_COUNT as usize - 1;
    assert!(counters.color_counts[overflow_index] > 0);
    assert_eq!(counters.color_counts[overflow_index], 5);

    assert_eq!(counters.body_count, 27);
    assert_eq!(counters.contact_count, 65);
}

// "shared overflow bodies preserve scalar solver order"
#[test]
fn overflow_matches_scalar() {
    let mut scalar = World::default();
    let mut wide = World::default();
    scalar.enable_wide_solver = false;
    wide.enable_wide_solver = true;
    scalar.enable_sleep = false;
    wide.enable_sleep = false;
    let _ = create_overflow_color_pile(&mut scalar);
    let _ = create_overflow_color_pile(&mut wide);
    let mut saw_overflow = false;
    for step in 0..60 {
        scalar.step(1.0 / 60.0, 4);
        wide.step(1.0 / 60.0, 4);
        saw_overflow = saw_overflow
            || !wide.constraint_graph.colors[OVERFLOW_INDEX as usize]
                .contacts
                .is_empty();
        assert!(!wide.uses_wide_contacts(OVERFLOW_INDEX, false));
        assert!(!wide.uses_wide_contacts(OVERFLOW_INDEX, true));
        assert_eq!(
            scalar.world_state_hash(),
            wide.world_state_hash(),
            "step {}",
            step
        );
    }
    assert!(saw_overflow);
}

// "color counts sum to the awake contact total"
#[test]
fn color_counts_sum() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    for i in 0..3 {
        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, (0.6 + i as f32 * 1.05) as f64, 0.0);
        let box_body = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    }

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }

    let counters = world.counters();
    let mut color_sum: i32 = 0;
    for i in 0..b3::GRAPH_COLOR_COUNT as usize {
        color_sum += counters.color_counts[i];
    }

    // Every graph constraint is accounted for in exactly one color.
    assert!(color_sum > 0);
    assert!(counters.awake_contact_count >= color_sum);
}
