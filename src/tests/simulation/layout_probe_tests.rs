//! Ported from Tests/NatselTests/Simulation/LayoutProbeTests.swift.

use std::mem::{offset_of, size_of};

use crate::math::types::Matrix3;
use crate::simulation::world_runtime::BodySim;
use crate::types::collision_types::{Manifold, ManifoldPoint};

// "SoA gathers rely on these offsets"
#[test]
fn offsets() {
    assert_eq!(offset_of!(ManifoldPoint, anchor_a), 0);
    assert_eq!(offset_of!(ManifoldPoint, anchor_b), 12);
    assert_eq!(offset_of!(ManifoldPoint, separation), 24);
    assert_eq!(offset_of!(ManifoldPoint, normal_impulse), 28);
    let normal = offset_of!(Manifold, normal);
    assert_eq!(offset_of!(Manifold, twist_impulse), normal + 12);
    assert_eq!(offset_of!(Manifold, friction_impulse), normal + 16);
    assert_eq!(offset_of!(Manifold, rolling_impulse), normal + 28);
    assert_eq!(size_of::<Matrix3>(), 36);
    println!(
        "LAYOUT manifold.normal={} points={} mp.stride={} invInertia={} invMass={} bodySim={}",
        normal,
        offset_of!(Manifold, points),
        size_of::<ManifoldPoint>(),
        offset_of!(BodySim, inv_inertia_world),
        offset_of!(BodySim, inv_mass),
        size_of::<BodySim>(),
    );
}
