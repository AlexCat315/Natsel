//! Soft bodies: generation, simulation, coupling, queries, lifecycle, and debug draw.
//!
//! Ported from `Tests/NatselTests/SoftBody/SoftBodyTests.swift`.

use crate::collision::height_field::factories::create_grid_height_field;
use crate::collision::hull::{make_box_hull, make_offset_box_hull};
use crate::collision::mesh::procedural::create_grid_mesh;
use crate::math::functions::{is_valid_position, min_float};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_runtime::{SoftBodyDef, SoftBodyID};
use crate::soft_body::soft_body_settings::{
    dihedral_angle, SoftBodyBendType, SoftBodyEdge, SoftBodyLRAType, SoftBodySettings,
    SoftBodySkinned, SoftBodyVertex, SoftBodyVertexAttributes,
};
use crate::soft_body::soft_body_solver::dihedral_gradients;
use crate::tests::world_position_assertions::distance;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::debug_draw::DebugDraw;
use crate::collision::compound::compound::Compound;
use crate::types::geometry::{CompoundDef, CompoundHullDef, Sphere};
use crate::types::ids::BodyID;
use crate::types::shape_def::QueryFilter;
use crate::types::shape_def::ShapeDef;

fn make_ground(world: &mut World) -> BodyID {
    let ground = world.create_body(&BodyDef::default());
    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = 0.6;
    let _ = world.create_hull_shape(
        ground,
        &shape_def,
        &make_offset_box_hull(50.0, 1.0, 50.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    ground
}

fn step(world: &mut World, count: i32) {
    for _ in 0..count {
        world.step(1.0 / 60.0, 4);
    }
}

fn min_y(world: &World, id: SoftBodyID) -> f32 {
    world
        .soft_body_vertex_positions(id)
        .iter()
        .fold(f32::MAX, |acc, p| min_float(acc, p.y as f32))
}

fn cloth_settings(
    columns: i32,
    rows: i32,
    spacing: f32,
    attributes: SoftBodyVertexAttributes,
) -> SoftBodySettings {
    SoftBodySettings::make_cloth(
        columns,
        rows,
        spacing,
        attributes,
        SoftBodyBendType::Distance,
        |_x, _z| false,
    )
}

#[test]
// "dihedral gradients match finite differences"
fn dihedral_gradient() {
    let points = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.1, 0.0),
        Vec3::new(0.4, 0.3, 0.9),
        Vec3::new(0.6, -0.4, -0.8),
    ];
    let g = dihedral_gradients(points[0], points[1], points[2], points[3]);
    assert!(g.valid);
    let analytic = [g.g0, g.g1, g.g2, g.g3];
    let eps: f32 = 1.0e-3;
    for v in 0..4 {
        for axis in 0..3 {
            let mut plus = points;
            let mut minus = points;
            let d = Vec3::new(
                if axis == 0 { eps } else { 0.0 },
                if axis == 1 { eps } else { 0.0 },
                if axis == 2 { eps } else { 0.0 },
            );
            plus[v] = plus[v] + d;
            minus[v] = minus[v] - d;
            let numeric = (dihedral_angle(plus[0], plus[1], plus[2], plus[3])
                - dihedral_angle(minus[0], minus[1], minus[2], minus[3]))
                / (2.0 * eps);
            let a = if axis == 0 {
                analytic[v].x
            } else if axis == 1 {
                analytic[v].y
            } else {
                analytic[v].z
            };
            assert!(
                (numeric - a).abs() < 0.02,
                "vertex {v} axis {axis}: {numeric} vs {a}"
            );
        }
    }
    // Coplanar unfolded triangles have zero angle.
    assert!(
        (dihedral_angle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.5, 0.0, 1.0),
            Vec3::new(0.5, 0.0, -1.0)
        ))
        .abs()
            < 1.0e-6
    );
}

#[test]
// "cloth generation creates stretch, shear, and bend constraints"
fn cloth_settings_test() {
    let settings = SoftBodySettings::make_cloth(
        5,
        4,
        0.1,
        SoftBodyVertexAttributes::new(0.0, 0.0, 1.0e-3, SoftBodyLRAType::None, 1.0),
        SoftBodyBendType::Dihedral,
        |_x, _z| false,
    );
    assert!(settings.validate().is_none());
    assert_eq!(settings.vertices.len(), 20);
    assert_eq!(settings.faces.len(), 2 * 4 * 3);
    // Grid edges: horizontal 4*4 + vertical 5*3, plus two diagonals per quad.
    assert_eq!(settings.edges.len(), 16 + 15 + 2 * 12);
    assert!(settings.dihedral_bends.len() > 0);
    for edge in settings.edges.iter() {
        let a = settings.vertices[edge.vertex.x as usize].position;
        let b = settings.vertices[edge.vertex.y as usize].position;
        assert!((distance(a, b) - edge.rest_length).abs() < 1.0e-5);
    }
    for bend in settings.dihedral_bends.iter() {
        assert!(bend.initial_angle.abs() < 1.0e-4);
    }
}

#[test]
// "pinned cloth hangs without stretching"
fn hanging_cloth() {
    let mut world = World::default();
    let mut attributes = SoftBodyVertexAttributes::new(0.0, 0.0, 1.0e-4, SoftBodyLRAType::None, 1.0);
    attributes.lra_type = SoftBodyLRAType::Geodesic;
    let settings = SoftBodySettings::make_cloth(
        20,
        20,
        0.05,
        attributes,
        SoftBodyBendType::Dihedral,
        |x, z| z == 0 && (x == 0 || x == 19),
    );
    assert!(settings.long_range_attachments.len() > 0);
    let mut def = SoftBodyDef::with_settings(settings.clone(), Pos::new(0.0, 2.0, 0.0), Quat::IDENTITY);
    def.iteration_count = 10;
    let cloth = world.create_soft_body(&def);
    assert!(cloth.is_null() == false);

    step(&mut world, 240);
    let positions = world.soft_body_vertex_positions(cloth);
    // Pinned corners did not move.
    let mut pinned_count = 0;
    for (i, v) in settings.vertices.iter().enumerate() {
        if v.inv_mass == 0.0 {
            assert!(
                distance(
                    positions[i],
                    Vec3::new(v.position.x, 2.0, v.position.z)
                ) < 1.0e-5
            );
            pinned_count += 1;
        }
    }
    assert_eq!(pinned_count, 2);
    // The cloth fell, but no edge stretched far.
    assert!(min_y(&world, cloth) < 1.7);
    let mut max_stretch: f32 = 0.0;
    let mut mean_stretch: f32 = 0.0;
    for edge in settings.edges.iter() {
        let l = distance(
            positions[edge.vertex.x as usize],
            positions[edge.vertex.y as usize],
        );
        max_stretch = max_stretch.max(l / edge.rest_length);
        mean_stretch += l / edge.rest_length / settings.edges.len() as f32;
    }
    assert!(max_stretch < 1.1);
    assert!(mean_stretch < 1.005);
    for p in positions {
        assert!(is_valid_position(p));
    }
}

#[test]
// "cloth drapes over a static sphere and box"
fn drape() {
    let mut world = World::default();
    let _ = make_ground(&mut world);
    let obstacle = world.create_body(&BodyDef::default());
    let _ = world.create_sphere_shape(
        obstacle,
        &ShapeDef::default(),
        Sphere::new(Vec3::new(0.0, 0.5, 0.0), 0.5),
    );
    let _ = world.create_hull_shape(
        obstacle,
        &ShapeDef::default(),
        &make_offset_box_hull(0.3, 0.3, 0.3, Vec3::new(1.2, 0.3, 0.0)),
    );

    let settings = cloth_settings(
        30,
        30,
        0.1,
        SoftBodyVertexAttributes::new(0.0, 1.0e-5, 1.0e-3, SoftBodyLRAType::None, 1.0),
    );
    let mut def = SoftBodyDef::with_settings(settings, Pos::new(0.5, 1.5, 0.0), Quat::IDENTITY);
    def.vertex_radius = 0.01;
    let cloth = world.create_soft_body(&def);
    step(&mut world, 300);

    let positions = world.soft_body_vertex_positions(cloth);
    let center = Vec3::new(0.0, 0.5, 0.0);
    for p in positions.iter() {
        assert!(distance(*p, center) > 0.5 - 0.02);
        assert!(p.y > -0.02);
        // Not inside the box.
        let local = *p - Vec3::new(1.2, 0.3, 0.0);
        let inside = local.x.abs() < 0.28 && local.y.abs() < 0.28 && local.z.abs() < 0.28;
        assert!(inside == false);
    }
    // Something rests on top of the sphere.
    assert!(positions.iter().any(|p| p.y > 0.95));
    assert!(world.soft_body_contacts(cloth).len() >= 2);
}

#[test]
// "soft cube keeps its volume, comes to rest, and sleeps"
fn soft_cube() {
    let mut world = World::default();
    let _ = make_ground(&mut world);
    let settings = SoftBodySettings::make_cube(5, 0.1, 1.0e-5, 0.0);
    assert!(settings.validate().is_none());
    for v in settings.volumes.iter() {
        assert!(v.six_rest_volume > 0.0);
    }
    let rest_volume =
        settings.volumes.iter().fold(0.0f32, |acc, v| acc + v.six_rest_volume) / 6.0;
    assert!((rest_volume - 0.4 * 0.4 * 0.4).abs() < 1.0e-4);

    let mut def = SoftBodyDef::with_settings(settings, Pos::new(0.0, 1.0, 0.0), Quat::IDENTITY);
    def.vertex_radius = 0.01;
    let cube = world.create_soft_body(&def);
    let initial_volume = world.soft_body_volume(cube);
    assert!((initial_volume - rest_volume).abs() < 1.0e-4);

    step(&mut world, 600);
    assert!(world.soft_body_volume(cube) > 0.8 * rest_volume);
    assert!(min_y(&world, cube) > -0.01);
    assert!(world.soft_body_is_awake(cube) == false);
}

#[test]
// "pressure inflates a balloon"
fn balloon() {
    fn volume(pressure: f32) -> f32 {
        let mut world = World::default();
        world.gravity = Vec3::ZERO;
        let settings = SoftBodySettings::make_sphere(
            0.5,
            2,
            1.0e-3,
            1.0e-3,
            SoftBodyBendType::Dihedral,
        );
        let mut def = SoftBodyDef::with_settings(settings, Pos::ZERO, Quat::IDENTITY);
        def.pressure = pressure;
        def.enable_sleep = false;
        let id = world.create_soft_body(&def);
        step(&mut world, 180);
        world.soft_body_volume(id)
    }

    let flat = volume(0.0);
    let inflated = volume(2000.0);
    assert!(inflated > 1.2 * flat);
}

#[test]
// "two way coupling: a box rests on a hammock and pushes it down"
fn hammock() {
    let mut world = World::default();
    let settings = SoftBodySettings::make_cloth(
        25,
        25,
        0.08,
        SoftBodyVertexAttributes::new(1.0e-6, 1.0e-5, 1.0e-2, SoftBodyLRAType::None, 1.0),
        SoftBodyBendType::Distance,
        |x, z| (x == 0 || x == 24) && (z == 0 || z == 24),
    );
    let mut def = SoftBodyDef::with_settings(settings, Pos::new(0.0, 1.0, 0.0), Quat::IDENTITY);
    def.vertex_radius = 0.02;
    def.friction = 0.5;
    def.linear_damping = 1.0;
    let cloth = world.create_soft_body(&def);

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(0.0, 1.5, 0.0);
    let box_ = world.create_body(&body_def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 50.0;
    let _ = world.create_hull_shape(box_, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    let mut touched_box = false;
    for _ in 0..600 {
        world.step(1.0 / 60.0, 4);
        touched_box = touched_box
            || world
                .soft_body_contacts(cloth)
                .iter()
                .any(|c| c.body_id == box_);
    }
    // Supported by the cloth: below the pins but far above free fall, and settled.
    let mut lowest: f32 = f32::MAX;
    let mut highest: f32 = -f32::MAX;
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
        lowest = min_float(lowest, world.body_position(box_).y as f32);
        highest = highest.max(world.body_position(box_).y as f32);
    }
    assert!(lowest > 0.2 && highest < 1.15);
    assert!(highest - lowest < 0.05);
    assert!(touched_box);
    // The cloth sags under the box.
    assert!(min_y(&world, cloth) < 0.95);
}

#[test]
// "cloth collides with mesh, height field, and compound shapes"
fn triangle_geometry() {
    for kind in 0..3 {
        let mut world = World::default();
        let ground = world.create_body(&BodyDef::default());
        match kind {
            0 => {
                let mesh =
                    create_grid_mesh(20, 20, 0.5, 0, false).expect("grid mesh");
                let _ = world.create_mesh_shape(ground, &ShapeDef::default(), mesh, Vec3::ONE);
            }
            1 => {
                let field = create_grid_height_field(21, 21, Vec3::new(0.5, 1.0, 0.5), false)
                    .expect("grid height field");
                let _ = world.create_height_field_shape(ground, &ShapeDef::default(), field);
            }
            _ => {
                let mut compound_def = CompoundDef::default();
                let mut hull_def = CompoundHullDef::default();
                hull_def.hull = Some(make_box_hull(5.0, 0.5, 5.0));
                hull_def.transform = Transform::new(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY);
                compound_def.hulls = vec![hull_def];
                let compound = Compound::new(&compound_def).expect("compound");
                let _ = world.create_compound_shape(ground, &ShapeDef::default(), &compound);
            }
        }
        let settings = cloth_settings(10, 10, 0.1, SoftBodyVertexAttributes::default());
        let center = if kind == 1 {
            Vec3::new(5.0, 0.5, 5.0)
        } else {
            Vec3::new(0.0, 0.5, 0.0)
        };
        let mut def = SoftBodyDef::with_settings(settings, Pos::from_vec3(center), Quat::IDENTITY);
        def.vertex_radius = 0.01;
        let cloth = world.create_soft_body(&def);
        step(&mut world, 180);
        assert!(min_y(&world, cloth) > -0.02, "geometry {kind}");
        assert!(min_y(&world, cloth) < 0.1, "geometry {kind}");
    }
}

#[test]
// "ray casts hit soft body faces"
fn ray_cast() {
    let mut world = World::default();
    let settings = cloth_settings(5, 5, 0.25, SoftBodyVertexAttributes::default());
    let cloth = world.create_soft_body(&SoftBodyDef::with_settings(
        settings,
        Pos::new(0.0, 1.0, 0.0),
        Quat::IDENTITY,
    ));
    let hit = world.cast_ray_closest_soft_body(
        Pos::new(0.1, 3.0, 0.1),
        Vec3::new(0.0, -4.0, 0.0),
        QueryFilter::default_filter(),
    );
    assert!(hit.hit);
    assert!(hit.soft_body_id == cloth);
    assert!(((hit.point.y - 1.0) as f32).abs() < 1.0e-4);
    assert!(hit.normal.y > 0.99);
    // Single sided faces are missed from below.
    let below =
        world.cast_ray_soft_body(cloth, Pos::new(0.1, -1.0, 0.1), Vec3::new(0.0, 4.0, 0.0));
    assert!(below.hit == false);
}

#[test]
// "kinematic skinned vertices follow the skeleton"
fn skinning() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    let mut settings = SoftBodySettings::make_rope(5, 0.1, 0.0, false);
    settings.inverse_bind_matrices = vec![Transform::IDENTITY];
    for i in 0..5 {
        let vertex = i as u32;
        settings.vertices[i as usize].inv_mass = if i == 0 { 0.0 } else { 1.0 };
        settings.skinned.push(SoftBodySkinned::new(
            vertex,
            crate::soft_body::soft_body_settings::SoftBodyU32x4::new(0, 0, 0, 0),
            crate::soft_body::soft_body_settings::SoftBodyF32x4::new(1.0, 0.0, 0.0, 0.0),
            if i == 0 { 0.0 } else { 0.05 },
            0.0,
            0.0,
        ));
    }
    settings.calculate_rest_values();
    let rope = world.create_soft_body(&SoftBodyDef::with_settings(
        settings.clone(),
        Pos::ZERO,
        Quat::IDENTITY,
    ));
    let offset = Vec3::new(1.0, 0.0, 0.0);
    world.set_soft_body_skin_transforms_from_transforms(
        rope,
        &[Transform::new(offset, Quat::IDENTITY)],
        false,
    );
    step(&mut world, 120);
    let positions = world.soft_body_vertex_positions(rope);
    for (i, v) in settings.vertices.iter().enumerate() {
        let target = v.position + offset;
        let tolerance = if v.inv_mass == 0.0 { 1.0e-5 } else { 0.051 };
        assert!(distance(positions[i], target) < tolerance);
    }
}

#[test]
// "parallel simulation matches serial"
fn parallel() {
    fn run(workers: i32) -> Vec<Pos> {
        let mut world = World::default();
        world.worker_count = workers;
        let _ = make_ground(&mut world);
        let mut ids: Vec<SoftBodyID> = Vec::new();
        for k in 0..4 {
            let settings = cloth_settings(16, 16, 0.05, SoftBodyVertexAttributes::default());
            ids.push(world.create_soft_body(&SoftBodyDef::with_settings(
                settings,
                Pos::new(k as f64, 1.0, 0.0),
                Quat::IDENTITY,
            )));
        }
        let mut body_def = BodyDef::default();
        body_def.type_ = BodyType::DynamicBody;
        body_def.position = Pos::new(0.0, 0.5, 0.0);
        let box_ = world.create_body(&body_def);
        let _ = world.create_hull_shape(box_, &ShapeDef::default(), &make_box_hull(0.2, 0.2, 0.2));
        step(&mut world, 120);
        let mut result: Vec<Pos> = ids
            .iter()
            .flat_map(|id| world.soft_body_vertex_positions(*id))
            .collect();
        result.push(world.body_position(box_));
        result
    }

    assert!(run(1) == run(4));
}

#[test]
// "sleeping soft body wakes when a body hits it"
fn wake_on_hit() {
    let mut world = World::default();
    let _ = make_ground(&mut world);
    let settings = cloth_settings(10, 10, 0.1, SoftBodyVertexAttributes::default());
    let cloth = world.create_soft_body(&SoftBodyDef::with_settings(
        settings,
        Pos::new(0.0, 0.05, 0.0),
        Quat::IDENTITY,
    ));
    step(&mut world, 120);
    assert!(world.soft_body_is_awake(cloth) == false);

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(0.0, 1.0, 0.0);
    let ball = world.create_body(&body_def);
    let _ = world.create_sphere_shape(
        ball,
        &ShapeDef::default(),
        Sphere::new(Vec3::ZERO, 0.1),
    );
    let mut woke = false;
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
        woke = woke || world.soft_body_is_awake(cloth);
    }
    assert!(woke);
}

#[test]
// "invalid settings are rejected and destroyed ids are invalid"
fn lifecycle() {
    let mut world = World::default();
    let mut settings = SoftBodySettings::new();
    assert!(
        world
            .create_soft_body(&SoftBodyDef::with_settings(settings.clone(), Pos::ZERO, Quat::IDENTITY))
            .is_null()
    );
    settings.vertices = vec![SoftBodyVertex::new(Vec3::ZERO, Vec3::ZERO, 1.0)];
    settings.edges = vec![SoftBodyEdge::new(0, 3, 0.0)];
    assert!(
        world
            .create_soft_body(&SoftBodyDef::with_settings(settings, Pos::ZERO, Quat::IDENTITY))
            .is_null()
    );

    let rope = world.create_soft_body(&SoftBodyDef::with_settings(
        SoftBodySettings::make_rope(3, 0.1, 0.0, true),
        Pos::ZERO,
        Quat::IDENTITY,
    ));
    assert_eq!(world.soft_body_count(), 1);
    world.destroy_soft_body(rope);
    assert!(world.soft_body_is_valid(rope) == false);
    assert_eq!(world.soft_body_count(), 0);
    let again = world.create_soft_body(&SoftBodyDef::with_settings(
        SoftBodySettings::make_rope(3, 0.1, 0.0, true),
        Pos::ZERO,
        Quat::IDENTITY,
    ));
    assert!(again != rope);
    assert!(world.soft_body_is_valid(again));
}

#[test]
// "debug draw shows soft body edges"
fn debug_draw() {
    let mut world = World::default();
    let settings = SoftBodySettings::make_cloth(
        4,
        4,
        0.1,
        SoftBodyVertexAttributes::default(),
        SoftBodyBendType::Distance,
        |x, z| x == 0 && z == 0,
    );
    let _ = world.create_soft_body(&SoftBodyDef::with_settings(
        settings.clone(),
        Pos::ZERO,
        Quat::IDENTITY,
    ));
    let segments = std::rc::Rc::new(std::cell::RefCell::new(0usize));
    let points = std::rc::Rc::new(std::cell::RefCell::new(0usize));
    let mut options = DebugDraw::new();
    let seg = segments.clone();
    options.draw_segment = Some(std::sync::Arc::new(move |_a: Pos, _b: Pos, _c| {
        *seg.borrow_mut() += 1;
    }));
    let pts = points.clone();
    options.draw_point = Some(std::sync::Arc::new(move |_a: Pos, _b: f32, _c| {
        *pts.borrow_mut() += 1;
    }));
    world.draw(&options, u64::MAX);
    assert_eq!(*segments.borrow(), settings.edges.len());
    assert_eq!(*points.borrow(), 1);
}
