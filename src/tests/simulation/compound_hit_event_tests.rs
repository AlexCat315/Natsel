//! Ported from Tests/NatselTests/Simulation/CompoundHitEventTests.swift.

use crate::collision::compound::compound::Compound;
use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{CompoundDef, CompoundHullDef, Sphere};
use crate::types::shape_def::{ShapeDef, SurfaceMaterial};

const HULL_MATERIAL_A: u64 = 11;
const HULL_MATERIAL_B: u64 = 22;
const SPHERE_MATERIAL: u64 = 99;
const HULL_CENTER_X: f32 = 3.0;

fn make_compound() -> Option<Compound> {
    let mut definition = CompoundDef::default();

    let mut material_a = SurfaceMaterial::default_material();
    material_a.user_material_id = HULL_MATERIAL_A;

    let mut material_b = SurfaceMaterial::default_material();
    material_b.user_material_id = HULL_MATERIAL_B;

    let mut hull_a = CompoundHullDef::default();
    hull_a.hull = Some(make_box_hull(1.0, 1.0, 1.0));
    hull_a.transform = Transform::new(
        Vec3::new(-HULL_CENTER_X, 0.0, 0.0),
        Quat::IDENTITY,
    );
    hull_a.material = material_a;
    definition.hulls.push(hull_a);

    let mut hull_b = CompoundHullDef::default();
    hull_b.hull = Some(make_box_hull(1.0, 1.0, 1.0));
    hull_b.transform = Transform::new(
        Vec3::new(HULL_CENTER_X, 0.0, 0.0),
        Quat::IDENTITY,
    );
    hull_b.material = material_b;
    definition.hulls.push(hull_b);

    Compound::new(&definition)
}

struct RunResult {
    hit_count: i32,
    material_a: u64,
    material_b: u64,
}

fn run(side: i32) -> Option<RunResult> {
    let compound = make_compound()?;

    let expected_hull_material = if side == 0 {
        HULL_MATERIAL_A
    } else {
        HULL_MATERIAL_B
    };
    let spawn_x: f32 = if side == 0 {
        -HULL_CENTER_X
    } else {
        HULL_CENTER_X
    };

    let mut world = World::default();
    world.hit_event_threshold = 1.0;

    // Static body holds the compound
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let compound_body = world.create_body(&def);
    let _ = world.create_compound_shape(compound_body, &ShapeDef::default(), &compound);

    // Sphere driven straight down onto the chosen child
    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.gravity_scale = 0.0;
    def.position = Pos::new(f64::from(spawn_x), 3.0, 0.0);
    def.linear_velocity = Vec3::new(0.0, -30.0, 0.0);
    let sphere_body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.enable_hit_events = true;
    shape_def.base_material.user_material_id = SPHERE_MATERIAL;
    let _ = world.create_sphere_shape(
        sphere_body,
        &shape_def,
        &Sphere::new(Vec3::ZERO, 0.5),
    );

    let mut hit_count = 0;
    let mut captured_a: u64 = 0;
    let mut captured_b: u64 = 0;

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);

        let events = world.contact_events();
        if !events.hit_events.is_empty() && hit_count == 0 {
            captured_a = events.hit_events[0].user_material_id_a;
            captured_b = events.hit_events[0].user_material_id_b;
        }
        hit_count += events.hit_events.len() as i32;
    }

    // Verify the struck child's material is reported, not materials[0].
    let _ = expected_hull_material;
    Some(RunResult {
        hit_count,
        material_a: captured_a,
        material_b: captured_b,
    })
}

// "hit event reports the struck compound child material"
#[test]
fn compound_child_material() {
    for side in 0..2 {
        let expected: u64 = if side == 0 {
            HULL_MATERIAL_A
        } else {
            HULL_MATERIAL_B
        };
        let result = match run(side) {
            Some(result) => result,
            None => {
                assert!(false, "compound creation failed");
                return;
            }
        };

        assert!(result.hit_count >= 1);
        // Sphere material on one side
        assert!(
            result.material_a == SPHERE_MATERIAL || result.material_b == SPHERE_MATERIAL
        );
        // Struck compound child's material on the other side.
        assert!(result.material_a == expected || result.material_b == expected);
    }
}
