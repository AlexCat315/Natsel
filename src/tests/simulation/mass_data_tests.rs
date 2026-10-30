//! Ported from Tests/NatselTests/Simulation/MassDataTests.swift.

use crate::base::constants::PI;
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Matrix3, Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::MassData;

fn diag_inertia() -> Matrix3 {
    Matrix3::new(
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(0.0, 4.0, 0.0),
        Vec3::new(0.0, 0.0, 8.0),
    )
}

#[test]
fn round_trip() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(5.0, -3.0, 2.0);
    let body = world.create_body(&def);

    let center = Vec3::new(0.1, 0.2, 0.3);
    let mut mass_data = MassData {
        mass: 0.0,
        center: Vec3::ZERO,
        inertia: Matrix3::ZERO,
    };
    mass_data.mass = 3.0;
    mass_data.center = center;
    mass_data.inertia = diag_inertia();
    world.set_body_mass_data(body, mass_data);

    assert!((world.body_mass(body) - 3.0).abs() < 1e-6);
    assert!((world.body_inverse_mass(body) - 1.0 / 3.0).abs() < 1e-6);

    let md = world.body_mass_data(body);
    assert!((md.mass - 3.0).abs() < 1e-6);
    assert!((md.center.x - center.x).abs() < 1e-6);
    assert!((md.center.y - center.y).abs() < 1e-6);
    assert!((md.center.z - center.z).abs() < 1e-6);
    assert!((md.inertia.cx.x - 2.0).abs() < 1e-6);
    assert!((md.inertia.cy.y - 4.0).abs() < 1e-6);
    assert!((md.inertia.cz.z - 8.0).abs() < 1e-6);

    let local_center = world.body_local_center(body);
    assert!((local_center.x - center.x).abs() < 1e-6);
    assert!((local_center.y - center.y).abs() < 1e-6);
    assert!((local_center.z - center.z).abs() < 1e-6);

    let local_inertia = world.body_local_rotational_inertia(body);
    assert!((local_inertia.cx.x - 2.0).abs() < 1e-6);
    assert!((local_inertia.cy.y - 4.0).abs() < 1e-6);
    assert!((local_inertia.cz.z - 8.0).abs() < 1e-6);

    let inv_world = world.body_world_inverse_rotational_inertia(body);
    assert!((inv_world.cx.x - 0.5).abs() < 1e-5);
    assert!((inv_world.cy.y - 0.25).abs() < 1e-5);
    assert!((inv_world.cz.z - 0.125).abs() < 1e-5);
    assert!(inv_world.cy.x.abs() < 1e-5);
    assert!(inv_world.cz.x.abs() < 1e-5);
    assert!(inv_world.cx.y.abs() < 1e-5);
    assert!(inv_world.cz.y.abs() < 1e-5);
    assert!(inv_world.cx.z.abs() < 1e-5);
    assert!(inv_world.cy.z.abs() < 1e-5);

    let world_center = world.body_world_center(body);
    assert!((world_center.x - (5.0 + center.x as f64)).abs() < 1e-5);
    assert!((world_center.y - (-3.0 + center.y as f64)).abs() < 1e-5);
    assert!((world_center.z - (2.0 + center.z as f64)).abs() < 1e-5);
}

#[test]
fn world_inertia_rotated() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Z, 0.5 * PI);
    let body = world.create_body(&def);

    let mut mass_data = MassData {
        mass: 0.0,
        center: Vec3::ZERO,
        inertia: Matrix3::ZERO,
    };
    mass_data.mass = 1.0;
    mass_data.inertia = diag_inertia();
    world.set_body_mass_data(body, mass_data);

    let local_inertia = world.body_local_rotational_inertia(body);
    assert!((local_inertia.cx.x - 2.0).abs() < 1e-6);
    assert!((local_inertia.cy.y - 4.0).abs() < 1e-6);
    assert!((local_inertia.cz.z - 8.0).abs() < 1e-6);

    let inv_world = world.body_world_inverse_rotational_inertia(body);
    assert!((inv_world.cx.x - 0.25).abs() < 1e-4);
    assert!((inv_world.cy.y - 0.5).abs() < 1e-4);
    assert!((inv_world.cz.z - 0.125).abs() < 1e-4);
    assert!(inv_world.cy.x.abs() < 1e-4);
    assert!(inv_world.cz.x.abs() < 1e-4);
    assert!(inv_world.cx.y.abs() < 1e-4);
    assert!(inv_world.cz.y.abs() < 1e-4);
    assert!(inv_world.cx.z.abs() < 1e-4);
    assert!(inv_world.cy.z.abs() < 1e-4);
}

#[test]
fn fixed_rotation() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.motion_locks.angular_x = true;
    def.motion_locks.angular_y = true;
    def.motion_locks.angular_z = true;
    let body = world.create_body(&def);

    let mut mass_data = MassData {
        mass: 0.0,
        center: Vec3::ZERO,
        inertia: Matrix3::ZERO,
    };
    mass_data.mass = 5.0;
    mass_data.inertia = diag_inertia();
    world.set_body_mass_data(body, mass_data);

    assert!((world.body_mass(body) - 5.0).abs() < 1e-6);
    assert!((world.body_inverse_mass(body) - 0.2).abs() < 1e-6);

    let local_inertia = world.body_local_rotational_inertia(body);
    assert!(local_inertia.cx.x.abs() < 1e-6);
    assert!(local_inertia.cy.y.abs() < 1e-6);
    assert!(local_inertia.cz.z.abs() < 1e-6);

    let inv_world = world.body_world_inverse_rotational_inertia(body);
    assert!(inv_world.cx.x.abs() < 1e-6);
    assert!(inv_world.cy.y.abs() < 1e-6);
    assert!(inv_world.cz.z.abs() < 1e-6);

    let md = world.body_mass_data(body);
    assert!(md.inertia.cx.x.abs() < 1e-6);
    assert!(md.inertia.cy.y.abs() < 1e-6);
    assert!(md.inertia.cz.z.abs() < 1e-6);
}

#[test]
fn zero_mass() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);

    let mass_data = MassData {
        mass: 0.0,
        center: Vec3::ZERO,
        inertia: Matrix3::ZERO,
    };
    world.set_body_mass_data(body, mass_data);

    assert!(world.body_inverse_mass(body).abs() < 1e-6);

    let local_inertia = world.body_local_rotational_inertia(body);
    assert!(local_inertia.cx.x.abs() < 1e-6);
    assert!(local_inertia.cy.y.abs() < 1e-6);
    assert!(local_inertia.cz.z.abs() < 1e-6);

    let inv_world = world.body_world_inverse_rotational_inertia(body);
    assert!(inv_world.cx.x.abs() < 1e-6);
    assert!(inv_world.cy.y.abs() < 1e-6);
    assert!(inv_world.cz.z.abs() < 1e-6);
}

#[test]
fn consistent_velocity() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(7.0, 1.0, -4.0);
    let body = world.create_body(&def);

    let omega = Vec3::new(1.0, 2.0, 4.0);
    world.set_body_linear_velocity(body, Vec3::new(1.0, -2.0, 3.0));
    world.set_body_angular_velocity(body, omega);

    let center = Vec3::new(0.5, 0.25, 0.125);
    let mut mass_data = MassData {
        mass: 0.0,
        center: Vec3::ZERO,
        inertia: Matrix3::ZERO,
    };
    mass_data.mass = 3.0;
    mass_data.center = center;
    mass_data.inertia = diag_inertia();
    world.set_body_mass_data(body, mass_data);

    let v = world.body_linear_velocity(body);
    assert!((v.x - (1.0 - 0.75)).abs() < 1e-6);
    assert!((v.y - (-2.0 + 1.875)).abs() < 1e-6);
    assert!((v.z - (3.0 - 0.75)).abs() < 1e-6);
}
