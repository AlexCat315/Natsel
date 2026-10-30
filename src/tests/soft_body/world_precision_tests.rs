//! Soft body world precision.

use crate::math::functions::{length, offset_pos, sub_pos};
use crate::math::types::{Pos, Quat, Vec3};
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_runtime::SoftBodyDef;
use crate::soft_body::soft_body_settings::{
    SoftBodyBendType, SoftBodySettings, SoftBodyVertexAttributes,
};

fn cloth(columns: i32, rows: i32, spacing: f32) -> SoftBodySettings {
    SoftBodySettings::make_cloth(
        columns,
        rows,
        spacing,
        SoftBodyVertexAttributes::default(),
        SoftBodyBendType::Distance,
        |_, _| false,
    )
}

#[test]
// "distant pieces sharing a float bounding cell keep independent precise frames"
fn separated_pieces() {
    let mut world = World::default();
    world.worker_count = 4;
    let mut def = SoftBodyDef::default();
    def.settings = cloth(3, 3, 0.001);
    def.gravity_scale = 0.0;
    def.enable_sleep = false;
    def.position = Pos::new(1.0e12, 1.0e12, -1.0e12);
    let a = world.create_soft_body(&def);
    def.position.x += 16384.0;
    let b = world.create_soft_body(&def);

    let before = [
        world.soft_body_vertex_positions(a),
        world.soft_body_vertex_positions(b),
    ];
    for _ in 0..5 {
        world.step(1.0 / 60.0, 4);
    }
    for (index, id) in [a, b].iter().enumerate() {
        let after = world.soft_body_vertex_positions(*id);
        for vertex in 0..after.len() {
            let drift = length(sub_pos(after[vertex], before[index][vertex]));
            assert!(
                drift < 0.00025,
                "piece {} vertex {} drifted by {}",
                index,
                vertex,
                drift
            );
        }
    }
}

#[test]
// "teleport and render buffers retain shape detail across huge distances"
fn teleport() {
    let mut world = World::default();
    let mut def = SoftBodyDef::default();
    def.settings = cloth(3, 3, 0.125);
    def.position = Pos::new(0.0, 2.0, 0.0);
    let id = world.create_soft_body(&def);

    let old_center = world.soft_body_center_of_mass(id);
    let offsets: Vec<Vec3> = world
        .soft_body_vertex_positions(id)
        .iter()
        .map(|p| sub_pos(*p, old_center))
        .collect();

    let target = Pos::new(1.0e12 + 0.375, -1.0e12 + 0.125, 1.0e12 + 0.5);
    world.set_soft_body_transform(id, target, Quat::IDENTITY);
    let positions = world.soft_body_vertex_positions(id);
    for vertex in 0..positions.len() {
        let drift = length(sub_pos(positions[vertex], target) - offsets[vertex]);
        assert!(drift < 0.00025, "vertex {} drifted by {}", vertex, drift);
    }

    let matched = world
        .with_soft_body_local_vertex_positions(id, |origin, vertices| {
            vertices
                .iter()
                .enumerate()
                .all(|(index, local)| offset_pos(origin, *local) == positions[index])
        })
        .unwrap_or(false);
    assert!(matched, "local vertices did not reconstruct the world buffer");
}
