//! Ported from Tests/NatselTests/Simulation/RecycleCounterTests.swift.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::functions::{abs_float, min_float};
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

/// Swift's `RejectFlag`: an `NSLock` guarded `Bool` the pre-solve callback reads.
struct RejectFlag {
    reject: AtomicBool,
}

impl RejectFlag {
    fn new() -> Self {
        Self {
            reject: AtomicBool::new(true),
        }
    }

    fn reject(&self) -> bool {
        self.reject.load(Ordering::SeqCst)
    }

    fn set_reject(&self, value: bool) {
        self.reject.store(value, Ordering::SeqCst);
    }
}

fn count_recycled_resting(enable_speculative: bool) -> i32 {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.5, 0.0);
    let box_body = world.create_body(&def);
    shape_def.enable_speculative_contact = enable_speculative;
    let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let mut recycled_count: i32 = 0;
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
        recycled_count += world.counters().recycled_contact_count;
    }
    recycled_count
}

// "speculative points enable recycling; disabling them stops it"
#[test]
fn speculative_recycle() {
    assert!(count_recycled_resting(true) > 0);
    assert_eq!(count_recycled_resting(true), 59);
    assert_eq!(count_recycled_resting(false), 0);
}

/// A rejected pre-solve frees the manifold while the shapes may already touch. The next step
/// must rebuild the contact and ask again instead of recycling the empty pair.
// "pre-solve rejection does not let the body sink through"
#[test]
fn pre_solve_reject_recycle() {
    let mut world = World::default();
    world.enable_sleep = false;

    let flag = Arc::new(RejectFlag::new());
    let callback_flag = Arc::clone(&flag);
    world.set_pre_solve_callback(Some(Arc::new(
        move |_shape_a, _shape_b, _point, _normal| !callback_flag.reject(),
    )));

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    let radius: f32 = 0.25;
    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, radius as f64, 0.0);
    let body = world.create_body(&def);

    shape_def.enable_pre_solve_events = true;
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::ZERO, radius));

    // Reject while the shapes first meet.
    world.step(1.0 / 60.0, 4);
    flag.set_reject(false);

    let mut min_separation = f32::MAX;
    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
        let p = world.body_transform(body).p;
        min_separation = min_float(min_separation, p.y as f32 - radius);
    }

    // The sphere must stay out of the ground within the linear slop.
    assert!(min_separation > -b3::linear_slop());
    assert!(abs_float(min_separation - (-0.00105443597)) < 1e-6);
}
