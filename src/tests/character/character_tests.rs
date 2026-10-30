//! Kinematic character controller.
//!
//! Ported from `Tests/NatselTests/Character/CharacterTests.swift`.
//!
//! Borrowing note: Swift held the world as an `unowned` class reference, so a test could update the
//! character and then step the same world in one loop. The port stores `&'w mut World` inside
//! [`CharacterVirtual`], so each scene takes one raw pointer to the world and hands the character a
//! reborrow of it. This is the aliasing Swift relied on unchecked exclusivity for; see
//! `MIGRATION.md` "Types and memory".

use crate::base::constants as b3;
use crate::character::character_virtual::{
    CharacterDef, CharacterGroundState, CharacterVirtual,
};
use crate::collision::hull::{make_box_hull, make_offset_box_hull, make_transformed_box_hull};
use crate::math::functions::{
    compute_cos_sin, make_quat_from_axis_angle, mul_add, offset_pos,
};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::world_runtime::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::shape_def::{QueryFilter, ShapeDef};

fn make_world() -> World {
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(50.0, 1.0, 50.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    world
}

/// Typical game loop: gravity when airborne, walking velocity when grounded.
fn drive(
    world: *mut World,
    character: &mut CharacterVirtual<'_>,
    walk: Vec3,
    steps: i32,
    extended: bool,
) {
    let dt: f32 = 1.0 / 60.0;
    for _ in 0..steps {
        // SAFETY: the world outlives the character for the whole scene and is only used for the
        // gravity read and the step, exactly as Swift's `unowned let world` allowed.
        let gravity = unsafe { (*world).gravity };
        let mut v = character.linear_velocity;
        if character.ground_state() == CharacterGroundState::OnGround {
            v = walk;
        } else {
            v.x = walk.x;
            v.z = walk.z;
            v = mul_add(v, dt, gravity);
        }
        character.linear_velocity = v;
        if extended {
            character.extended_update(dt, gravity);
        } else {
            character.update(dt);
        }
        // SAFETY: as above; `extended_update` / `update` have returned, so the character holds no
        // live borrow of the world at this point.
        unsafe { (*world).step(dt, 4) };
    }
}

fn add_static_box(world: &mut World, center: Vec3, half: Vec3, rotation: Quat) {
    let body = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        body,
        &ShapeDef::default(),
        &make_transformed_box_hull(half.x, half.y, half.z, Transform::new(center, rotation)),
    );
}

#[test]
// "falls, lands, and walks on flat ground"
fn walk() {
    let mut world = make_world();
    let world_ptr: *mut World = &mut world;
    let mut def = CharacterDef::new();
    def.position = Pos::new(0.0, 2.0, 0.0);
    let mut character = character_shared(world_ptr, &def);

    drive(world_ptr, &mut character, Vec3::ZERO, 120, true);
    assert!(character.ground_state() == CharacterGroundState::OnGround);
    assert!(character.ground_normal().y > 0.99);
    assert!(character.position().y.abs() < 0.05);

    let start = character.position();
    drive(world_ptr, &mut character, Vec3::new(2.0, 0.0, 0.0), 60, true);
    assert!((character.position().x - start.x - 2.0).abs() < 0.1);
    assert!((character.position().y - start.y).abs() < 0.02);
    assert!(character.ground_state() == CharacterGroundState::OnGround);
}

/// Swift's `CharacterVirtual(world:def)!`: the character holds the world as an unowned reference,
/// so the lifetime is left for the caller to infer from the pointer rather than tied to a borrow.
fn character_shared<'a>(world: *mut World, def: &CharacterDef) -> CharacterVirtual<'a> {
    // SAFETY: the pointer addresses a live `World` that the test keeps alive for every use of the
    // returned character; see the borrowing note at the top of the file.
    unsafe { CharacterVirtual::new(&mut *world, def) }.expect("character")
}

#[test]
// "walls stop the character and it slides along them"
fn wall() {
    let mut world = make_world();
    add_static_box(
        &mut world,
        Vec3::new(2.0, 1.0, 0.0),
        Vec3::new(0.2, 1.0, 5.0),
        Quat::IDENTITY,
    );
    let world_ptr: *mut World = &mut world;
    let mut def = CharacterDef::new();
    def.position = Pos::new(0.0, 0.05, 0.0);
    let mut character = character_shared(world_ptr, &def);

    drive(world_ptr, &mut character, Vec3::ZERO, 30, true);
    drive(world_ptr, &mut character, Vec3::new(2.0, 0.0, 2.0), 120, true);
    // Stopped by the wall face at x = 1.8, minus radius and padding.
    assert!(character.position().x < 1.8 - 0.3 + 0.01);
    assert!(character.position().x > 1.8 - 0.3 - 0.06);
    // Kept sliding along z.
    assert!(character.position().z > 3.0);
    assert!(character.contacts().iter().any(|c| c.normal.x < -0.9));
}

#[test]
// "walkable slopes are climbed and steep slopes are not" (arguments: degrees = 25, climbable = true)
fn slopes_25_degrees() {
    slopes(25.0, true);
}

#[test]
// "walkable slopes are climbed and steep slopes are not" (arguments: degrees = 65, climbable = false)
fn slopes_65_degrees() {
    slopes(65.0, false);
}

fn slopes(degrees: f32, climbable: bool) {
    let mut world = make_world();
    let angle = degrees * b3::DEG_TO_RAD;
    // A ramp rising toward +x, starting at x = 1.
    let rotation = make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), angle);
    let cs = compute_cos_sin(angle);
    add_static_box(
        &mut world,
        Vec3::new(
            1.0 + 3.0 * cs.cosine,
            3.0 * cs.sine - 0.5 * cs.cosine,
            0.0,
        ),
        Vec3::new(3.0, 0.5, 3.0),
        rotation,
    );
    let world_ptr: *mut World = &mut world;
    let mut def = CharacterDef::new();
    def.position = Pos::new(-1.0, 0.05, 0.0);
    def.enable_stair_stepping = false;
    let mut character = character_shared(world_ptr, &def);

    drive(world_ptr, &mut character, Vec3::ZERO, 30, true);
    drive(world_ptr, &mut character, Vec3::new(2.0, 0.0, 0.0), 180, true);
    if climbable {
        assert!(character.position().y > 1.0);
    } else {
        assert!(character.position().y < 0.3);
    }
}

#[test]
// "stairs are climbed with extended update, tall ledges are not"
fn stairs() {
    for (step_height, climbs) in [(0.25f32, true), (0.7f32, false)] {
        let mut world = make_world();
        for k in 0..4 {
            add_static_box(
                &mut world,
                Vec3::new(
                    2.0 + 0.5 * (k as f32) + 2.5,
                    step_height * ((k + 1) as f32) * 0.5,
                    0.0,
                ),
                Vec3::new(2.5, step_height * ((k + 1) as f32) * 0.5, 3.0),
                Quat::IDENTITY,
            );
        }
        let world_ptr: *mut World = &mut world;
        let mut def = CharacterDef::new();
        def.position = Pos::new(0.0, 0.05, 0.0);
        let mut character = character_shared(world_ptr, &def);

        drive(world_ptr, &mut character, Vec3::ZERO, 30, true);
        drive(world_ptr, &mut character, Vec3::new(2.0, 0.0, 0.0), 180, true);
        if climbs {
            assert!(
                character.position().y > (3.0 * step_height) as f64,
                "step {step_height}"
            );
            assert!(character.ground_state() == CharacterGroundState::OnGround);
        } else {
            assert!(character.position().y < 0.1, "step {step_height}");
            assert!(character.position().x < 2.0);
        }
    }
}

#[test]
// "sticks to the floor walking down a slope"
fn stick_to_floor() {
    let mut world = make_world();
    let angle: f32 = 20.0 * b3::DEG_TO_RAD;
    let rotation = make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), -angle);
    // Down ramp from x = 0 toward +x, top at y = 2.
    let cs = compute_cos_sin(angle);
    add_static_box(
        &mut world,
        Vec3::new(
            4.0 * cs.cosine,
            2.0 - 4.0 * cs.sine - 0.5 * cs.cosine,
            0.0,
        ),
        Vec3::new(4.0, 0.5, 3.0),
        rotation,
    );
    add_static_box(
        &mut world,
        Vec3::new(-2.0, 1.0, 0.0),
        Vec3::new(2.0, 1.0, 3.0),
        Quat::IDENTITY,
    );
    let world_ptr: *mut World = &mut world;
    let mut def = CharacterDef::new();
    def.position = Pos::new(-1.0, 2.05, 0.0);
    let mut character = character_shared(world_ptr, &def);

    drive(world_ptr, &mut character, Vec3::ZERO, 30, true);
    let mut airborne = 0;
    let dt: f32 = 1.0 / 60.0;
    for _ in 0..90 {
        // SAFETY: as in `drive`; the world outlives the character.
        let gravity = unsafe { (*world_ptr).gravity };
        character.linear_velocity = if character.ground_state() == CharacterGroundState::OnGround {
            Vec3::new(4.0, 0.0, 0.0)
        } else {
            mul_add(character.linear_velocity, dt, gravity)
        };
        character.extended_update(dt, gravity);
        // SAFETY: as above.
        unsafe { (*world_ptr).step(dt, 4) };
        if character.ground_state() != CharacterGroundState::OnGround
            && character.position().x > 0.3
            && character.position().x < 6.0
        {
            airborne += 1;
        }
    }
    assert!(airborne <= 2);
    assert!(character.position().x > 3.0);
}

#[test]
// "rides a moving platform"
fn platform() {
    let mut world = make_world();
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::KinematicBody;
    body_def.position = Pos::new(0.0, 1.0, 0.0);
    body_def.linear_velocity = Vec3::new(1.0, 0.0, 0.0);
    let platform = world.create_body(&body_def);
    let _ = world.create_hull_shape(platform, &ShapeDef::default(), &make_box_hull(2.0, 0.1, 2.0));

    let world_ptr: *mut World = &mut world;
    let mut def = CharacterDef::new();
    def.position = Pos::new(0.0, 1.15, 0.0);
    let mut character = character_shared(world_ptr, &def);

    drive(world_ptr, &mut character, Vec3::ZERO, 120, true);
    assert!(character.ground_state() == CharacterGroundState::OnGround);
    assert!(character.ground_body_id() == platform);
    assert!((character.ground_velocity().x - 1.0).abs() < 1.0e-3);
    // SAFETY: live world behind the pointer; the character carries no tracked borrow.
    let offset = character.position().x - unsafe { (*world_ptr).body_position(platform) }.x;
    assert!(offset.abs() < 0.1);
}

#[test]
// "pushes dynamic bodies and is seen through its inner body"
fn pushing() {
    let mut world = make_world();
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(1.5, 0.25, 0.0);
    let box_ = world.create_body(&body_def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 100.0;
    shape_def.base_material.friction = 0.2;
    let _ = world.create_hull_shape(
        box_,
        &shape_def,
        &make_box_hull(0.25, 0.25, 0.25),
    );

    let world_ptr: *mut World = &mut world;
    let mut def = CharacterDef::new();
    def.position = Pos::new(0.0, 0.05, 0.0);
    let mut character = character_shared(world_ptr, &def);
    assert!(character.inner_body_id().is_null() == false);

    drive(world_ptr, &mut character, Vec3::ZERO, 10, true);
    drive(world_ptr, &mut character, Vec3::new(1.5, 0.0, 0.0), 180, true);
    // SAFETY: as above.
    assert!(unsafe { (*world_ptr).body_position(box_) }.x > 2.0);

    // A ray finds the inner body.
    // Swift's `mulAdd(character.position, 1, Vec3(0, 1, 0)) + Vec3(-3, 0, 0)`.
    let origin = offset_pos(
        offset_pos(character.position(), Vec3::new(0.0, 1.0, 0.0)),
        Vec3::new(-3.0, 0.0, 0.0),
    );
    // SAFETY: live world behind the pointer.
    let hit = unsafe {
        (*world_ptr).cast_ray_closest(
            origin,
            Vec3::new(6.0, 0.0, 0.0),
            QueryFilter::default_filter(),
        )
    };
    assert!(hit.hit);
    // SAFETY: as above.
    assert!(unsafe { (*world_ptr).shape_body(hit.shape_id) } == character.inner_body_id());
    character.destroy();
    // SAFETY: as above.
    assert!(unsafe { (*world_ptr).body_is_valid(character.inner_body_id()) } == false);
}

#[test]
// "crouching shape changes are refused under a low ceiling"
fn crouch() {
    let mut world = make_world();
    add_static_box(
        &mut world,
        Vec3::new(3.0, 1.5, 0.0),
        Vec3::new(1.0, 0.2, 2.0),
        Quat::IDENTITY,
    );
    let world_ptr: *mut World = &mut world;
    let mut def = CharacterDef::new();
    def.position = Pos::new(0.0, 0.05, 0.0);
    let mut character = character_shared(world_ptr, &def);

    drive(world_ptr, &mut character, Vec3::ZERO, 30, true);
    assert!(character.set_shape(0.3, 1.0, 0.0));
    drive(world_ptr, &mut character, Vec3::new(1.5, 0.0, 0.0), 120, true);
    assert!(character.position().x > 2.5);
    // Under the ceiling (bottom at 1.3) standing up would overlap.
    assert!(character.set_shape(0.3, 1.8, 0.0) == false);
    assert!(character.height() == 1.0);
}
