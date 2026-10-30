//! Ported from Tests/NatselTests/Simulation/RagdollDeterminismTests.swift.

use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::BodyDef;
use crate::types::shape_def::ShapeDef;
use crate::collision::mesh::procedural::{create_grid_mesh, create_torus_mesh};
use crate::collision::mesh::storage::TriangleMesh;

use crate::tests::simulation::determinism_scene::hash_transform;
use crate::tests::simulation::human::{create_human, Human};

const GRID_SIZE: f32 = 15.0;
const GRID_COUNT: i32 = 2;
const GROUP_SIZE: i32 = 2;

struct Group {
    humans: Vec<Human>,
}

impl Default for Group {
    fn default() -> Self {
        Self {
            humans: vec![Human::default(); 2],
        }
    }
}

struct FallingRagdollData {
    groups: Vec<Group>,
    #[allow(dead_code)]
    grid_mesh: TriangleMesh,
    #[allow(dead_code)]
    torus_mesh: TriangleMesh,
}

// "falling ragdolls are repeatable"
#[test]
fn falling_ragdolls() {
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();
        let data = create_falling_ragdolls(&mut world);
        let mut hash: u32 = 0;
        let mut sleep_step = 0;
        let mut step_count = 0;
        let mut done = false;
        for _ in 0..500 {
            if done {
                break;
            }
            world.step(1.0 / 60.0, 4);
            if hash == 0 && world.body_events().is_empty() && world.awake_body_count() == 0 {
                hash = crate::base::constants::HASH_INIT;
                for i in 0..GRID_COUNT {
                    for j in 0..GRID_COUNT {
                        for k in 0..GROUP_SIZE {
                            let group = &data.groups[(i * GRID_COUNT + j) as usize];
                            let human = &group.humans[k as usize];
                            for bone in &human.bones {
                                hash = hash_transform(hash, &world.body_transform(bone.body_id));
                            }
                        }
                    }
                }
                sleep_step = step_count;
            }
            step_count += 1;
            done = hash != 0;
        }

        assert!(done);
        samples.push(sleep_step as u64);
        samples.push(u64::from(hash));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

fn create_falling_ragdolls(world: &mut World) -> FallingRagdollData {
    let half_mesh_grid_rows = 4;
    let mesh_grid_cell_width = GRID_SIZE / (2 * half_mesh_grid_rows) as f32;
    let grid_mesh = create_grid_mesh(
        2 * half_mesh_grid_rows,
        2 * half_mesh_grid_rows,
        mesh_grid_cell_width,
        0,
        true,
    )
    .expect("grid mesh");
    let torus_mesh =
        create_torus_mesh(16, 16, 0.25 * GRID_SIZE, 1.0).expect("torus mesh");

    let span = GRID_SIZE * GRID_COUNT as f32;
    let mut body_def = BodyDef::default();
    let shape_def = ShapeDef::default();
    let mut data = FallingRagdollData {
        groups: Vec::new(),
        grid_mesh: grid_mesh.clone(),
        torus_mesh: torus_mesh.clone(),
    };
    body_def.position.x = (-0.5 * span + 0.5 * GRID_SIZE) as f64;

    for i in 0..GRID_COUNT {
        body_def.position.z = (-0.5 * span + 0.5 * GRID_SIZE) as f64;
        for j in 0..GRID_COUNT {
            let body = world.create_body(&body_def);
            let _ = world.create_mesh_shape(body, &shape_def, grid_mesh.clone(), Vec3::ONE);
            let _ = world.create_mesh_shape(body, &shape_def, torus_mesh.clone(), Vec3::ONE);

            let mut group = Group::default();
            let mut position = Pos::new(
                f64::from(-0.5f32 * span + GRID_SIZE * (j as f32 + 0.5)),
                15.0,
                f64::from(-0.5f32 * span + GRID_SIZE * (i as f32 + 0.5)),
            );
            for k in 0..GROUP_SIZE {
                create_human(
                    &mut group.humans[k as usize],
                    world,
                    position,
                    5.0,
                    1.0,
                    0.7,
                    i * GRID_COUNT + j,
                    None,
                    false,
                );
                position.x += 0.75;
            }
            data.groups.push(group);

            body_def.position.z += GRID_SIZE as f64;
        }
        body_def.position.x += GRID_SIZE as f64;
    }

    data
}
