//! Ported from Tests/NatselTests/Simulation/MoverGeometryTests.swift.

use crate::collision::capsule::collide_mover_and_capsule;
use crate::collision::height_field::planes::collide_mover_and_height_field;
use crate::collision::hull::{make_box_hull, make_offset_box_hull, make_transformed_box_hull};
use crate::collision::hull_mover::collide_mover_and_hull;
use crate::collision::mesh::overlap::collide_mover_and_mesh;
use crate::collision::sphere::collide_mover_and_sphere;
use crate::math::functions::{abs_float, is_normalized};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{
    Capsule, CompoundDef, CompoundHullDef, CompoundMeshDef, HeightField, HeightFieldDef, Mesh,
    Sphere, TriangleMesh,
};
use crate::types::ids::BodyID;
use crate::types::queries::{PlaneResult, QueryFilter, ShapeProxy};
use crate::types::shape_def::{ShapeDef, SurfaceMaterial};
use crate::collision::mesh::build::MeshBuildOptions;

// MARK: Sphere

// "mover far from a sphere reports nothing"
#[test]
fn sphere_separated() {
    let shape = Sphere::new(Vec3::ZERO, 0.5);
    let mover = Capsule::new(
        Vec3::new(4.0, 3.0, 0.0),
        Vec3::new(6.0, 3.0, 0.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_sphere(&mut result, &shape, &mover), 0);
}

// "mover near a sphere pushes it out along the surface normal"
#[test]
fn sphere_touching() {
    let shape = Sphere::new(Vec3::ZERO, 0.5);
    // Mover core segment runs along X at y = 0.6, leaving it 0.1 inside the 0.7 combined radius.
    let mover = Capsule::new(
        Vec3::new(-1.0, 0.6, 0.0),
        Vec3::new(1.0, 0.6, 0.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_sphere(&mut result, &shape, &mover), 1);
    assert!(is_normalized(result.plane.normal));
    // Push-out points from the sphere straight up toward the mover.
    assert!(result.plane.normal.y > 0.99);
    assert!(abs_float(result.plane.offset - 0.1) < 1e-5);
}

// "mover through a sphere center still yields a unit normal"
#[test]
fn sphere_deep_overlap() {
    let shape = Sphere::new(Vec3::ZERO, 0.5);
    // Mover axis runs straight through the sphere center: the bug case where GJK reports a
    // zero normal.
    let mover = Capsule::new(
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_sphere(&mut result, &shape, &mover), 1);
    assert!(is_normalized(result.plane.normal));
    // The fallback axis is perpendicular to the mover axis (X).
    assert!(abs_float(result.plane.normal.x) < 1e-5);
    // Deepest possible penetration: the full combined radius.
    assert!(abs_float(result.plane.offset - 0.7) < 1e-5);
}

// MARK: Capsule

// "mover far from a capsule reports nothing"
#[test]
fn capsule_separated() {
    let shape = Capsule::new(
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.3,
    );
    let mover = Capsule::new(
        Vec3::new(-1.0, 5.0, 0.0),
        Vec3::new(1.0, 5.0, 0.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_capsule(&mut result, &shape, &mover), 0);
}

// "parallel capsule mover pushes along the face normal"
#[test]
fn capsule_touching() {
    let shape = Capsule::new(
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.3,
    );
    // Parallel mover 0.4 above, leaving it 0.1 inside the 0.5 combined radius.
    let mover = Capsule::new(
        Vec3::new(-1.0, 0.4, 0.0),
        Vec3::new(1.0, 0.4, 0.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_capsule(&mut result, &shape, &mover), 1);
    assert!(is_normalized(result.plane.normal));
    assert!(result.plane.normal.y > 0.99);
    assert!(abs_float(result.plane.offset - 0.1) < 1e-5);
}

// "crossing capsule cores yield the perpendicular axis"
#[test]
fn capsule_deep_overlap() {
    // Shape capsule along X, mover capsule along Z; their core segments cross exactly at the
    // origin, so GJK reports a zero normal.
    let shape = Capsule::new(
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.3,
    );
    let mover = Capsule::new(
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_capsule(&mut result, &shape, &mover), 1);
    assert!(is_normalized(result.plane.normal));
    // The separating axis of two crossing segments is perpendicular to both.
    assert!(abs_float(result.plane.normal.x) < 1e-5);
    assert!(abs_float(result.plane.normal.z) < 1e-5);
    assert!(abs_float(result.plane.offset - 0.5) < 1e-5);
}

// "coincident capsule cores fall back to a perpendicular of the mover axis"
#[test]
fn capsule_parallel_overlap() {
    // Mover core segment coincides with the shape core segment: the cross-product axis
    // degenerates, so a perpendicular of the mover axis is used instead.
    let shape = Capsule::new(
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.3,
    );
    let mover = Capsule::new(
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_capsule(&mut result, &shape, &mover), 1);
    assert!(is_normalized(result.plane.normal));
    // The fallback axis is perpendicular to the mover axis (X).
    assert!(abs_float(result.plane.normal.x) < 1e-5);
    assert!(abs_float(result.plane.offset - 0.5) < 1e-5);
}

// MARK: Hull

// "mover far from a hull reports nothing"
#[test]
fn hull_separated() {
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let mover = Capsule::new(
        Vec3::new(-0.3, 5.0, 0.0),
        Vec3::new(0.3, 5.0, 0.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_hull(&mut result, &box_hull, &mover), 0);
}

// "mover above a hull face pushes along +Y"
#[test]
fn hull_touching() {
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    // Mover core segment above the +Y face; the 0.2 radius reaches 0.1 into it.
    let mover = Capsule::new(
        Vec3::new(-0.3, 0.6, 0.0),
        Vec3::new(0.3, 0.6, 0.0),
        0.2,
    );

    let mut result = PlaneResult::default();
    assert_eq!(collide_mover_and_hull(&mut result, &box_hull, &mover), 1);
    assert!(is_normalized(result.plane.normal));
    assert!(result.plane.normal.y > 0.99);
    assert!(abs_float(result.plane.offset - 0.1) < 1e-4);
}

// "a mover fully inside a hull drops the plane instead of a zero normal"
#[test]
fn hull_deep_overlap() {
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    // Mover core segment lies entirely inside the box, so GJK reports overlap.
    let mover = Capsule::new(
        Vec3::new(-0.2, 0.0, 0.0),
        Vec3::new(0.2, 0.0, 0.0),
        0.1,
    );

    let mut result = PlaneResult::default();
    // The overlap guard drops the plane rather than emit a zero normal.
    assert_eq!(collide_mover_and_hull(&mut result, &box_hull, &mover), 0);
}

// MARK: - Material routing

/// Two separated upward facing triangles on the y = 0 plane, one per material.
fn make_two_material_mesh() -> Option<TriangleMesh> {
    let vertices = [
        Vec3::new(-3.0, 0.0, -1.0),
        Vec3::new(-2.0, 0.0, 1.0),
        Vec3::new(-1.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, -1.0),
        Vec3::new(2.0, 0.0, 1.0),
        Vec3::new(3.0, 0.0, -1.0),
    ];
    let indices: [i32; 6] = [0, 1, 2, 3, 4, 5];
    TriangleMesh::new(
        &vertices,
        &indices,
        Some(&[0, 1]),
        MeshBuildOptions::default(),
    )
}

fn make_material(friction: f32, user_id: u64) -> SurfaceMaterial {
    let mut m = SurfaceMaterial::default_material();
    m.friction = friction;
    m.user_material_id = user_id;
    m
}

// "mesh mover reports the per triangle material index"
#[test]
fn world_mesh_materials() {
    let Some(mesh) = make_two_material_mesh() else {
        assert!(false, "mesh creation failed");
        return;
    };

    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.materials = vec![make_material(0.2, 1), make_material(0.8, 2)];
    let _ = world.create_mesh_shape(body, &shape_def, mesh.clone(), Vec3::new(1.0, 1.0, 1.0));

    world.step(1.0 / 60.0, 1);

    // Mover hanging just above the first triangle so its radius reaches 0.05 into the surface.
    let mover_a = Capsule::new(
        Vec3::new(-2.0, 0.15, 0.0),
        Vec3::new(-2.0, 0.35, 0.0),
        0.2,
    );

    let mut planes_a: Vec<PlaneResult> = Vec::new();
    world.collide_mover(Pos::ZERO, &mover_a, &QueryFilter::default_filter(), |_, planes| {
        planes_a.extend_from_slice(planes);
        true
    });

    assert_eq!(planes_a.len(), 1);
    assert!(planes_a[0].plane.normal.y > 0.99);
    assert!(abs_float(planes_a[0].plane.offset - 0.05) < 1e-4);
    assert!(abs_float(planes_a[0].point.y) < 1e-4);
    assert_eq!(planes_a[0].material_index, 0);
    assert!(planes_a[0].triangle_index >= 0);
    assert!((planes_a[0].triangle_index as usize) < mesh.triangle_count());

    // Same mover over the second triangle.
    let mover_b = Capsule::new(
        Vec3::new(2.0, 0.15, 0.0),
        Vec3::new(2.0, 0.35, 0.0),
        0.2,
    );

    let mut planes_b: Vec<PlaneResult> = Vec::new();
    world.collide_mover(Pos::ZERO, &mover_b, &QueryFilter::default_filter(), |_, planes| {
        planes_b.extend_from_slice(planes);
        true
    });

    assert_eq!(planes_b.len(), 1);
    assert!(planes_b[0].plane.normal.y > 0.99);
    assert_eq!(planes_b[0].material_index, 1);
    // The two movers hit different triangles.
    assert_ne!(planes_b[0].triangle_index, planes_a[0].triangle_index);
}

// "a convex shape reports material index zero"
#[test]
fn convex_material_index() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.materials = vec![make_material(0.5, 7)];
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(1.0, 0.5, 1.0));

    let mover = Capsule::new(
        Vec3::new(0.0, 0.6, 0.0),
        Vec3::new(0.0, 0.8, 0.0),
        0.2,
    );

    let mut planes: Vec<PlaneResult> = Vec::new();
    world.collide_mover(Pos::ZERO, &mover, &QueryFilter::default_filter(), |_, found| {
        planes.extend_from_slice(found);
        true
    });

    assert!(!planes.is_empty());
    assert_eq!(planes[0].material_index, 0);
}

// MARK: - Mesh winding

// "mesh mover culls the backside of a triangle"
#[test]
fn mesh_backside() {
    let Some(data) = make_two_material_mesh() else {
        assert!(false, "mesh creation failed");
        return;
    };
    let mesh = Mesh::new(Some(data.clone()), Vec3::new(1.0, 1.0, 1.0));

    // On the front of the left triangle, so a plane comes back.
    let above = Capsule::new(
        Vec3::new(-2.0, 0.15, 0.0),
        Vec3::new(-2.0, 0.35, 0.0),
        0.2,
    );
    let front = collide_mover_and_mesh(&mesh, above, 4);
    assert_eq!(front.len(), 1);
    assert!(front[0].plane.normal.y > 0.99);
    assert_eq!(front[0].child_index, 0);
    assert!(front[0].triangle_index >= 0);
    assert!((front[0].triangle_index as usize) < data.triangle_count());

    // The same spot seen from behind the face is culled.
    let below = Capsule::new(
        Vec3::new(-2.0, -0.35, 0.0),
        Vec3::new(-2.0, -0.15, 0.0),
        0.2,
    );
    assert!(collide_mover_and_mesh(&mesh, below, 4).is_empty());
}

// "mirrored mesh scale keeps the front up and swaps the material"
#[test]
fn mesh_mirrored_scale() {
    let Some(data) = make_two_material_mesh() else {
        assert!(false, "mesh creation failed");
        return;
    };
    // Reflecting the scale flips the triangle winding, and collision swaps it back, so the
    // front stays up. Local x = -2 maps onto the source right triangle, material 1.
    let mesh = Mesh::new(Some(data), Vec3::new(-1.0, 1.0, 1.0));

    let above = Capsule::new(
        Vec3::new(-2.0, 0.15, 0.0),
        Vec3::new(-2.0, 0.35, 0.0),
        0.2,
    );
    let front = collide_mover_and_mesh(&mesh, above, 4);
    assert_eq!(front.len(), 1);
    assert!(front[0].plane.normal.y > 0.99);
    assert_eq!(front[0].material_index, 1);

    let below = Capsule::new(
        Vec3::new(-2.0, -0.35, 0.0),
        Vec3::new(-2.0, -0.15, 0.0),
        0.2,
    );
    assert!(collide_mover_and_mesh(&mesh, below, 4).is_empty());
}

// MARK: - Height field mover

/// Flat 3x3 vertex field at y = 0, one cell material per entry.
fn make_flat_field(materials: &[u8], clockwise: bool) -> Option<HeightField> {
    let mut definition = HeightFieldDef::default();
    definition.heights = vec![0.0; 9];
    definition.material_indices = materials.to_vec();
    definition.scale = Vec3::new(1.0, 1.0, 1.0);
    definition.count_x = 3;
    definition.count_z = 3;
    definition.global_minimum_height = -1.0;
    definition.global_maximum_height = 1.0;
    definition.clockwise_winding = clockwise;
    HeightField::new(&definition)
}

// "height field mover culls the underside"
#[test]
fn height_field_backside() {
    let Some(field) = make_flat_field(&[0, 0, 0, 0], false) else {
        assert!(false, "field creation failed");
        return;
    };

    // Standing on the front (upper) face of the default winding.
    let above = Capsule::new(
        Vec3::new(0.3, 0.15, 0.25),
        Vec3::new(0.3, 0.35, 0.25),
        0.2,
    );
    let front = collide_mover_and_height_field(&field, &above, 4);
    assert_eq!(front.len(), 1);
    assert!(front[0].plane.normal.y > 0.99);
    assert!(abs_float(front[0].plane.offset - 0.05) < 1e-4);

    // Under the surface is the back side and gets culled.
    let below = Capsule::new(
        Vec3::new(0.3, -0.35, 0.25),
        Vec3::new(0.3, -0.15, 0.25),
        0.2,
    );
    assert!(collide_mover_and_height_field(&field, &below, 4).is_empty());
}

// "height field mover reports triangle, child and material indices"
#[test]
fn height_field_report() {
    let Some(field) = make_flat_field(&[1, 2, 0, 0], false) else {
        assert!(false, "field creation failed");
        return;
    };

    // (0.3, 0.25) sits on the x + z <= 1 side of cell (0,0), which holds triangle 0.
    let first = Capsule::new(
        Vec3::new(0.3, 0.15, 0.25),
        Vec3::new(0.3, 0.35, 0.25),
        0.2,
    );
    let a = collide_mover_and_height_field(&field, &first, 4);
    assert_eq!(a.len(), 1);
    assert_eq!(a[0].triangle_index, 0);
    assert_eq!(a[0].child_index, 0);
    assert_eq!(a[0].material_index, 1);

    // (1.3, 0.3) sits on the x + z <= 2 side of cell (0,1), which holds triangle 2.
    let second = Capsule::new(
        Vec3::new(1.3, 0.15, 0.3),
        Vec3::new(1.3, 0.35, 0.3),
        0.2,
    );
    let b = collide_mover_and_height_field(&field, &second, 4);
    assert_eq!(b.len(), 1);
    assert_eq!(b[0].triangle_index, 2);
    assert_eq!(b[0].material_index, 2);
}

// "a clockwise height field faces down"
#[test]
fn height_field_clockwise() {
    let Some(field) = make_flat_field(&[0, 0, 0, 0], true) else {
        assert!(false, "field creation failed");
        return;
    };

    // The winding is flipped, so the contactable side is underneath.
    let below = Capsule::new(
        Vec3::new(0.3, -0.35, 0.25),
        Vec3::new(0.3, -0.15, 0.25),
        0.2,
    );
    let hit = collide_mover_and_height_field(&field, &below, 4);
    assert_eq!(hit.len(), 1);
    assert!(hit[0].plane.normal.y < -0.99);
    assert!(abs_float(hit[0].plane.offset - 0.05) < 1e-4);
    assert!(hit[0].triangle_index == 0 || hit[0].triangle_index == 1);

    let above = Capsule::new(
        Vec3::new(0.3, 0.15, 0.25),
        Vec3::new(0.3, 0.35, 0.25),
        0.2,
    );
    assert!(collide_mover_and_height_field(&field, &above, 4).is_empty());
}

// MARK: - Body mover queries

// "body mover reports the base material for every convex shape"
#[test]
fn body_material_indices() {
    let mut world = World::default();
    world.enable_sleep = false;

    let def = BodyDef::default();
    let body = world.create_body(&def);

    // One shape of each convex type, spaced out along X.
    let shape_def = ShapeDef::default();

    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.5, 0.0), 0.5));

    let _ = world.create_hull_shape(
        body,
        &shape_def,
        &make_transformed_box_hull(
            0.5,
            0.5,
            0.5,
            Transform::new(Vec3::new(5.0, 0.0, 0.0), Quat::IDENTITY),
        ),
    );

    let _ = world.create_capsule_shape(
        body,
        &shape_def,
        Capsule::new(
            Vec3::new(9.0, 0.0, 0.0),
            Vec3::new(11.0, 0.0, 0.0),
            0.3,
        ),
    );

    let body_transform = crate::math::types::WorldTransform::new(Pos::ZERO, Quat::IDENTITY);

    // Convex shapes report the base material as index 0.
    let movers = [
        Capsule::new(
            Vec3::new(0.0, 1.15, 0.0),
            Vec3::new(0.0, 1.35, 0.0),
            0.2,
        ),
        Capsule::new(
            Vec3::new(5.0, 0.65, 0.0),
            Vec3::new(5.0, 0.85, 0.0),
            0.2,
        ),
        Capsule::new(
            Vec3::new(10.0, 0.45, 0.0),
            Vec3::new(10.0, 0.65, 0.0),
            0.2,
        ),
    ];

    for (i, mover) in movers.iter().enumerate() {
        let planes = world.body_collide_mover(
            body,
            Pos::ZERO,
            mover,
            &QueryFilter::default_filter(),
            body_transform,
            4,
        );
        assert_eq!(planes.len(), 1, "mover {}", i);
        if planes.len() != 1 {
            continue;
        }
        assert_ne!(planes[0].shape_id.index1, 0);
        assert!(planes[0].result.plane.normal.y > 0.99, "mover {}", i);
        assert_eq!(planes[0].result.material_index, 0, "mover {}", i);
        assert_eq!(planes[0].result.child_index, 0, "mover {}", i);
    }
}

// "body mover skips mesh and compound shapes"
#[test]
fn body_skips_mesh_and_compound() {
    let Some(data) = make_two_material_mesh() else {
        assert!(false, "mesh creation failed");
        return;
    };

    let mut world = World::default();
    world.enable_sleep = false;

    let def = BodyDef::default();
    let body = world.create_body(&def);

    // A mesh and a compound under the same mover; only convex shapes answer.
    let shape_def = ShapeDef::default();
    let _ = world.create_mesh_shape(body, &shape_def, data, Vec3::new(1.0, 1.0, 1.0));

    let planes = world.body_collide_mover(
        body,
        Pos::ZERO,
        &Capsule::new(
            Vec3::new(-2.0, 0.15, 0.0),
            Vec3::new(-2.0, 0.35, 0.0),
            0.2,
        ),
        &QueryFilter::default_filter(),
        crate::math::types::WorldTransform::new(Pos::ZERO, Quat::IDENTITY),
        4,
    );

    // The mesh answers through the world query, not the body query.
    assert!(planes.is_empty());
}

// MARK: - Compound material remapping

/// The hull material and the two mesh materials are distinct, so the compound material table
/// gets one slot per material. Children bake in hull then mesh order.
// "compound mover remaps child material indices into the shared table"
#[test]
fn world_compound_mesh_materials() {
    let Some(mesh) = make_two_material_mesh() else {
        assert!(false, "mesh creation failed");
        return;
    };

    let mesh_materials = vec![make_material(0.3, 101), make_material(0.6, 202)];
    let hull_material = make_material(0.9, 303);

    let mut definition = CompoundDef::default();

    let mut mesh_child = CompoundMeshDef::default();
    mesh_child.mesh_data = Some(mesh);
    mesh_child.transform = Transform::IDENTITY;
    mesh_child.scale = Vec3::new(1.0, 1.0, 1.0);
    mesh_child.materials = mesh_materials;
    definition.meshes.push(mesh_child);

    let mut hull_child = CompoundHullDef::default();
    hull_child.hull = Some(make_box_hull(0.5, 0.5, 0.5));
    hull_child.transform = Transform::new(Vec3::new(-8.0, 0.0, 0.0), Quat::IDENTITY);
    hull_child.material = hull_material;
    definition.hulls.push(hull_child);

    let Some(compound) = crate::collision::compound::compound::Compound::new(&definition) else {
        assert!(false, "compound creation failed");
        return;
    };
    let compound_materials = compound.materials();
    assert_eq!(compound_materials.len(), 3);
    assert_eq!(compound_materials[0].user_material_id, 303);
    assert_eq!(compound_materials[1].user_material_id, 101);
    assert_eq!(compound_materials[2].user_material_id, 202);

    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let body = world.create_body(&def);
    let _ = world.create_compound_shape(body, &ShapeDef::default(), &compound);

    world.step(1.0 / 60.0, 1);

    // Mover on top of the hull child face at y = 0.5.
    let hull_mover = Capsule::new(
        Vec3::new(-8.0, 0.65, 0.0),
        Vec3::new(-8.0, 0.85, 0.0),
        0.2,
    );
    let mut hull_planes: Vec<PlaneResult> = Vec::new();
    world.collide_mover(Pos::ZERO, &hull_mover, &QueryFilter::default_filter(), |_, planes| {
        hull_planes.extend_from_slice(planes);
        true
    });

    assert_eq!(hull_planes.len(), 1);
    assert!(hull_planes[0].plane.normal.y > 0.99);
    assert_eq!(hull_planes[0].child_index, 0);
    assert_eq!(hull_planes[0].material_index, 0);
    assert_eq!(
        compound_materials[hull_planes[0].material_index as usize].user_material_id,
        303
    );

    // The mesh reports triangle material 1, which the compound remaps through the child
    // material table to the shared slot of meshMaterials[1].
    let mesh_mover = Capsule::new(
        Vec3::new(2.0, 0.15, 0.0),
        Vec3::new(2.0, 0.35, 0.0),
        0.2,
    );
    let mut mesh_planes: Vec<PlaneResult> = Vec::new();
    world.collide_mover(Pos::ZERO, &mesh_mover, &QueryFilter::default_filter(), |_, planes| {
        mesh_planes.extend_from_slice(planes);
        true
    });

    assert_eq!(mesh_planes.len(), 1);
    assert!(mesh_planes[0].plane.normal.y > 0.99);
    assert_eq!(mesh_planes[0].child_index, 1);
    assert_eq!(mesh_planes[0].material_index, 2);
    assert_eq!(
        compound_materials[mesh_planes[0].material_index as usize].user_material_id,
        202
    );
}

// "mesh material indices clamp to the last valid slot"
#[test]
fn world_mesh_material_clamp() {
    let Some(mesh) = make_two_material_mesh() else {
        assert!(false, "mesh creation failed");
        return;
    };

    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let body = world.create_body(&def);

    // Fewer shape materials than the mesh indexes: out of range indices clamp.
    let mut shape_def = ShapeDef::default();
    shape_def.materials = vec![make_material(0.5, 42)];
    let _ = world.create_mesh_shape(body, &shape_def, mesh, Vec3::new(1.0, 1.0, 1.0));

    world.step(1.0 / 60.0, 1);

    // The mover over the second triangle would index material 1, which does not exist.
    let mover = Capsule::new(
        Vec3::new(2.0, 0.15, 0.0),
        Vec3::new(2.0, 0.35, 0.0),
        0.2,
    );
    let mut planes: Vec<PlaneResult> = Vec::new();
    world.collide_mover(Pos::ZERO, &mover, &QueryFilter::default_filter(), |_, found| {
        planes.extend_from_slice(found);
        true
    });

    assert_eq!(planes.len(), 1);
    assert_eq!(planes[0].material_index, 0);
}
