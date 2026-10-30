//! Ported from Tests/NatselTests/Simulation/DeterminismTests.swift.

use crate::math::types::Pos;
use crate::simulation::World;
use crate::tests::simulation::determinism_scene::{
    create_mesh_drop, create_query_spawn, create_wave_pile, update_mesh_drop, update_query_spawn,
    update_wave_pile, QuerySpawnScene, SceneRandom,
};

// "wave pile is repeatable"
#[test]
fn wave_pile() {
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();
        let scene = create_wave_pile(&mut world);
        let mut step_count = 0;
        let mut sleep_step = 0;
        let mut hash: u32 = 0;
        let mut done = false;
        for _ in 0..500 {
            if done {
                continue;
            }
            world.step(1.0f32 / 60.0, 4);
            done = update_wave_pile(&world, &scene, &mut step_count, &mut sleep_step, &mut hash);
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

// "mesh drop is repeatable"
#[test]
fn mesh_drop() {
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();
        let scene = create_mesh_drop(&mut world, Pos::ZERO);
        let mut step_count = 0;
        let mut sleep_step = 0;
        let mut hash: u32 = 0;
        let mut done = false;
        for _ in 0..400 {
            if done {
                continue;
            }
            world.step(1.0f32 / 60.0, 4);
            done = update_mesh_drop(&world, &scene, &mut step_count, &mut sleep_step, &mut hash);
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

// "query spawn is repeatable"
#[test]
fn query_spawn() {
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();
        let mut scene: QuerySpawnScene = create_query_spawn(&mut world);
        let mut random = SceneRandom::new(71_689);
        let mut step_count = 0;
        let mut sleep_step = 0;
        let mut hash: u32 = 0;
        let mut done = false;
        for _ in 0..1000 {
            if done {
                continue;
            }
            world.step(1.0f32 / 60.0, 4);
            done = update_query_spawn(
                &mut world,
                &mut scene,
                &mut random,
                &mut step_count,
                &mut sleep_step,
                &mut hash,
            );
        }
        assert!(done);
        assert_eq!(scene.spawn_count, 50);
        samples.push(sleep_step as u64);
        samples.push(u64::from(hash));
        samples.push(scene.query_hit_count as u64);
        samples.push(u64::from(scene.query_hash));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
