//! Natsel performance measurements for version and configuration comparisons.
//!
//! Ported from `Sources/Benchmarks/main.swift`.

use std::io::Write;
use std::time::Instant;

use natsel::simulation::mesh_contact::update::MESH_TRACE_ENABLED;
use natsel::simulation::world_runtime::World;
use natsel::types::world_def::Profile;

// `benchmarks.rs` is the crate root, so a bare `mod scenes;` resolves against `src/bin/`, not
// `src/bin/benchmarks/`. The `#[path]` attributes keep the scene files in their own directory.
#[path = "benchmarks/feature_scenes.rs"]
pub mod feature_scenes;
#[path = "benchmarks/quality_scenes.rs"]
pub mod quality_scenes;
#[path = "benchmarks/scenes.rs"]
pub mod scenes;

use feature_scenes::{
    create_cloth, create_cloth_stack, create_path_beads, create_ragdoll_motors, create_rollback,
    create_self_colliding_cloth64, create_soft_cubes, create_vehicles, destroy_ragdoll_motors,
    destroy_rollback, destroy_self_colliding_cloth64, destroy_vehicles, step_ragdoll_motors,
    step_rollback, step_vehicles,
};
use quality_scenes::{
    create_sparse_soft_bodies, create_soft_queries, destroy_soft_queries, step_soft_queries,
};
use scenes::{
    create_convex_pile, create_joint_grid, create_large_pyramid, create_many_pyramids,
    create_sleep, create_spinner, create_trees25, create_washer, destroy_spinner, destroy_trees,
    record_counters, step_sleep, world_counters,
};

/// One benchmark scenario. Swift's closures over `World` become fn pointers, which is enough
/// because no scene captures state.
struct Benchmark {
    name: &'static str,
    create: fn(&mut World),
    step: Option<fn(&mut World, usize)>,
    destroy: Option<fn()>,
    total_step_count: usize,
    warmup_step_count: usize,
}

fn benchmarks() -> Vec<Benchmark> {
    vec![
        Benchmark {
            name: "large_pyramid",
            create: create_large_pyramid,
            step: None,
            destroy: None,
            total_step_count: 200,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "many_pyramids",
            create: create_many_pyramids,
            step: None,
            destroy: None,
            total_step_count: 100,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "joint_grid",
            create: create_joint_grid,
            step: None,
            destroy: None,
            total_step_count: 100,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "spinner",
            create: create_spinner,
            step: None,
            destroy: Some(destroy_spinner),
            total_step_count: 800,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "sleep",
            create: create_sleep,
            step: Some(step_sleep),
            destroy: None,
            total_step_count: 300,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "trees25",
            create: create_trees25,
            step: None,
            destroy: Some(destroy_trees),
            total_step_count: 500,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "washer",
            create: create_washer,
            step: None,
            destroy: None,
            total_step_count: 1000,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "convex_pile",
            create: create_convex_pile,
            step: None,
            destroy: None,
            total_step_count: 500,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "vehicles",
            create: create_vehicles,
            step: Some(step_vehicles),
            destroy: Some(destroy_vehicles),
            total_step_count: 600,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "cloth",
            create: create_cloth,
            step: None,
            destroy: None,
            total_step_count: 300,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "cloth_self64",
            create: create_self_colliding_cloth64,
            step: None,
            destroy: None,
            total_step_count: 240,
            warmup_step_count: 60,
        },
        Benchmark {
            name: "soft_cubes",
            create: create_soft_cubes,
            step: None,
            destroy: None,
            total_step_count: 300,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "cloth_stack",
            create: create_cloth_stack,
            step: None,
            destroy: None,
            total_step_count: 300,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "path_beads",
            create: create_path_beads,
            step: None,
            destroy: None,
            total_step_count: 300,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "ragdoll_motors",
            create: create_ragdoll_motors,
            step: Some(step_ragdoll_motors),
            destroy: Some(destroy_ragdoll_motors),
            total_step_count: 300,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "rollback",
            create: create_rollback,
            step: Some(step_rollback),
            destroy: Some(destroy_rollback),
            total_step_count: 200,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "soft_queries",
            create: create_soft_queries,
            step: Some(step_soft_queries),
            destroy: Some(destroy_soft_queries),
            total_step_count: 30,
            warmup_step_count: 1,
        },
        Benchmark {
            name: "sparse_soft",
            create: create_sparse_soft_bodies,
            step: None,
            destroy: None,
            total_step_count: 100,
            warmup_step_count: 1,
        },
    ]
}

/// Swift's `var acc = Profile()` accumulator for the averaged phase report.
#[derive(Clone, Copy, Default)]
struct PhaseAccumulator {
    step: f32,
    pairs: f32,
    collide: f32,
    constraints: f32,
    prepare_constraints: f32,
    warm_start: f32,
    solve_impulses: f32,
    relax_impulses: f32,
    restitution: f32,
    store_impulses: f32,
    integrate_velocities: f32,
    integrate_positions: f32,
    transforms: f32,
    hit_events: f32,
    sleep_islands: f32,
    refit: f32,
    bullets: f32,
}

fn main() {
    let list = benchmarks();

    let mut single_benchmark: i32 = -1;
    let mut run_all_benchmarks = false;
    let default_benchmark_names: [&str; 2] = ["many_pyramids", "joint_grid"];
    let mut run_count: i32 = 4;
    let mut record_steps = false;
    // Parse diagnostics once; environment dictionary construction is not physics-step work.
    let trace_steps = std::env::var("NATSEL_TRACE").unwrap_or_default() == "1";
    let mesh_trace_step: Option<i64> = std::env::var("NATSEL_MESHTRACE")
        .ok()
        .and_then(|value| value.parse::<i64>().ok());
    let dump_step: Option<i64> = std::env::var("NATSEL_DUMP")
        .ok()
        .and_then(|value| value.parse::<i64>().ok());
    let record_phases = std::env::var("NATSEL_PHASE").unwrap_or_default() == "1";
    let record_hash = std::env::var("NATSEL_HASH").unwrap_or_default() == "1";

    let args: Vec<String> = std::env::args().collect();
    let mut i = 1usize;
    while i < args.len() {
        let arg = &args[i];
        if arg.starts_with("-b=") || arg.starts_with("--benchmark=") {
            let value = arg.splitn(2, '=').nth(1).unwrap_or("").to_owned();
            if let Ok(index) = value.parse::<i32>() {
                single_benchmark = index;
            } else if let Some(index) = list
                .iter()
                .position(|benchmark| benchmark.name.to_lowercase() == value.to_lowercase())
            {
                single_benchmark = index as i32;
            } else {
                println!("No benchmark named \"{}\"", value);
                std::process::exit(1);
            }
        } else if arg.starts_with("-r=") || arg.starts_with("--repeats=") {
            let value = arg.splitn(2, '=').nth(1).unwrap_or("");
            run_count = value.parse::<i32>().unwrap_or(run_count);
        } else if arg == "-s" || arg == "--record-steps" {
            record_steps = true;
        } else if arg == "--all" {
            run_all_benchmarks = true;
        } else if arg == "-l" || arg == "--list" {
            for (index, benchmark) in list.iter().enumerate() {
                println!("{} {}", index, benchmark.name);
            }
            std::process::exit(0);
        }
        i += 1;
    }

    if !(run_count > 0
        && (single_benchmark == -1
            || (single_benchmark >= 0 && (single_benchmark as usize) < list.len())))
    {
        println!("Choose a valid benchmark and a positive repeat count.");
        std::process::exit(1);
    }

    println!("Starting benchmarks");
    println!("======================================");

    let benchmark_environment: std::collections::HashMap<String, String> =
        std::env::vars().collect();

    for (index, benchmark) in list.iter().enumerate() {
        if single_benchmark != -1 && index != single_benchmark as usize {
            continue;
        }
        if single_benchmark == -1
            && !run_all_benchmarks
            && !default_benchmark_names.contains(&benchmark.name)
        {
            continue;
        }

        let step_count = benchmark.total_step_count;
        println!("benchmark: {}, steps = {}", benchmark.name, step_count);

        let mut min_time = f64::MAX;
        let mut measured_worker_count: i32 = 1;

        for run_index in 0..run_count {
            let mut world = World::default();
            if let Some(wide) = benchmark_environment.get("NATSEL_WIDE") {
                world.enable_wide_solver = wide == "1";
            }
            if let Some(workers) = benchmark_environment
                .get("NATSEL_WORKERS")
                .and_then(|value| value.parse::<i32>().ok())
            {
                world.worker_count = workers;
            }
            measured_worker_count = world.worker_count;
            if benchmark_environment.get("NATSEL_CCD").map(String::as_str) == Some("0") {
                world.enable_continuous = false;
            }
            (benchmark.create)(&mut world);
            if let Some(probe) = benchmark_environment.get("NATSEL_PROBE") {
                if benchmark.name == "spinner" {
                    world.disable_body(scenes::spinner_probe_body(if probe == "nomesh" {
                        0
                    } else {
                        1
                    }));
                }
            }
            if trace_steps {
                println!(
                    "create {:x} bodies {}",
                    world.world_state_hash(),
                    world.body_count()
                );
            }

            let warmup_count =
                std::cmp::max(1, std::cmp::min(step_count, benchmark.warmup_step_count));
            for step_index in 0..warmup_count {
                if let Some(step) = benchmark.step {
                    step(&mut world, step_index);
                }
                world.step(1.0 / 60.0, 4);
            }

            let mut acc = PhaseAccumulator::default();
            let mut step_samples: Vec<u64> = Vec::new();
            if record_steps {
                step_samples.reserve(step_count - warmup_count);
            }
            let start = Instant::now();
            for step_index in warmup_count..step_count {
                if let Some(step) = benchmark.step {
                    step(&mut world, step_index);
                }
                let step_started = if record_steps {
                    Some(Instant::now())
                } else {
                    None
                };
                world.step(1.0 / 60.0, 4);
                if let Some(step_started) = step_started {
                    step_samples.push(step_started.elapsed().as_nanos() as u64);
                }
                MESH_TRACE_ENABLED.store(
                    mesh_trace_step == Some((step_index + 1) as i64),
                    std::sync::atomic::Ordering::Relaxed,
                );
                if trace_steps {
                    println!("step {} {:x}", step_index, world.world_state_hash());
                }
                if dump_step == Some(step_index as i64) {
                    for line in world.body_state_dump() {
                        println!("B {}", line);
                    }
                    for line in world.contact_dump() {
                        println!("{}", line);
                    }
                }
                let pr: Profile = world.profile;
                acc.pairs += pr.pairs;
                acc.collide += pr.collide;
                acc.prepare_constraints += pr.prepare_constraints;
                acc.warm_start += pr.warm_start;
                acc.solve_impulses += pr.solve_impulses;
                acc.relax_impulses += pr.relax_impulses;
                acc.store_impulses += pr.store_impulses;
                acc.integrate_velocities += pr.integrate_velocities;
                acc.integrate_positions += pr.integrate_positions;
                acc.transforms += pr.transforms;
                acc.hit_events += pr.hit_events;
                acc.sleep_islands += pr.sleep_islands;
                acc.constraints += pr.constraints;
                acc.restitution += pr.restitution;
                acc.refit += pr.refit;
                acc.bullets += pr.bullets;
                acc.step += pr.step;
            }
            let elapsed = start.elapsed();
            let n = std::cmp::max(1, step_count - warmup_count) as f32;
            if record_phases {
                println!(
                    "AVG {} step={:.3} pairs={:.3} collide={:.3} prepare={:.3} warm={:.3} push={:.3} solve={:.3} store={:.3} intV={:.3} intP={:.3} finalize={:.2} hit={:.2} sleep={:.2} restit={:.2} refit={:.2} bullets={:.2}",
                    benchmark.name,
                    acc.step / n,
                    acc.pairs / n,
                    acc.collide / n,
                    acc.prepare_constraints / n,
                    acc.warm_start / n,
                    acc.solve_impulses / n,
                    acc.relax_impulses / n,
                    acc.store_impulses / n,
                    acc.integrate_velocities / n,
                    acc.integrate_positions / n,
                    acc.transforms / n,
                    acc.hit_events / n,
                    acc.sleep_islands / n,
                    acc.restitution / n,
                    acc.refit / n,
                    acc.bullets / n
                );
            }

            let ms = elapsed.as_nanos() as f64 / 1_000_000.0;
            if record_phases {
                let p: Profile = world.profile;
                println!(
                    "PHASE steps={} pairs={} collide={} prepare={} warm={} push={} solve={} store={} intVel={} intPos={} substeps={} hit={} jnt={} finalize={} refit={} sleep={} total={}",
                    step_count,
                    p.pairs,
                    p.collide,
                    p.prepare_constraints,
                    p.warm_start,
                    p.solve_impulses,
                    p.relax_impulses,
                    p.store_impulses,
                    p.integrate_velocities,
                    p.integrate_positions,
                    p.constraints,
                    p.hit_events,
                    p.joint_events,
                    p.transforms,
                    p.refit,
                    p.sleep_islands,
                    ms
                );
            }
            println!("run {} : {} (ms)", run_index, ms);
            if record_steps && !step_samples.is_empty() {
                step_samples.sort_unstable();
                let percentile = |fraction: f64| -> f64 {
                    step_samples[((step_samples.len() - 1) as f64 * fraction) as usize] as f64
                        / 1_000_000.0
                };
                println!(
                    "step_ms p50={:.3} p95={:.3} p99={:.3}",
                    percentile(0.50),
                    percentile(0.95),
                    percentile(0.99)
                );
            }
            std::io::stdout().flush().ok();
            min_time = min_time.min(ms);

            if run_index == run_count - 1 {
                record_counters(&world);
            }
            if record_hash {
                println!("state: {:x}", world.world_state_hash());
            }
            match (benchmark.name, benchmark.destroy) {
                // Swift's `destroySelfCollidingCloth64` reads the live vertex positions before
                // releasing the world, so it needs the world and the hash flag, not a fn pointer.
                ("cloth_self64", _) => destroy_self_colliding_cloth64(&world, record_hash),
                (_, Some(destroy)) => destroy(),
                (_, None) => {}
            }
        }

        let (body, shape, contact) = world_counters();
        println!("body {} / shape {} / contact {}", body, shape, contact);
        println!("");
        println!(
            "{}.csv: threads,ms {},{}",
            benchmark.name, measured_worker_count, min_time
        );
    }

    println!("======================================");
    println!("All benchmarks complete!");
}
