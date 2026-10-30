//! Ported from Tests/NatselTests/Collision/SupportWideTests.swift.
//!
//! The wide support query embeds the vertex index in the low mantissa bits and takes a plain
//! numeric minimum, so each lane must agree with the scalar oracle bit for bit.

use crate::collision::convex_manifold::{get_support_wide, HULL_BIT_COUNT};
use crate::math::types::Vec3;

#[test]
fn support() {
    // "support values and embedded vertex indices match the scalar oracle"
    for count in [1usize, 3, 4, 8, 12, 32, 64, 128] {
        let mut seed: u32 = 12345;
        let mut random = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let value = ((seed % 2001) as i32 - 1000) as f32;
            value / 64.0
        };
        let mut x: Vec<f32> = (0..count).map(|_| random()).collect();
        let mut y: Vec<f32> = (0..count).map(|_| random()).collect();
        let mut z: Vec<f32> = (0..count).map(|_| random()).collect();
        let mask: u32 = (1u32 << HULL_BIT_COUNT) - 1;
        for pass in 0..64 {
            let normal = if pass == 0 {
                Vec3::ZERO
            } else {
                Vec3::new(random(), random(), random())
            };
            if pass == 32 {
                x = vec![x[0]; count];
                y = vec![y[0]; count];
                z = vec![z[0]; count];
            }
            let bias: f32 = 10000.0;
            let mut minimum = f32::INFINITY;
            for i in 0..count {
                let d = normal.x * x[i] + normal.y * y[i] + normal.z * z[i];
                let value = bias - d;
                let augmented =
                    f32::from_bits((value.to_bits() & !mask) | ((i as u32) & mask));
                if augmented < minimum {
                    minimum = augmented;
                }
            }
            let index = (minimum.to_bits() & mask) as usize;
            let expected = normal.x * x[index] + normal.y * y[index] + normal.z * z[index];
            let (support, vertex_index) = get_support_wide(
                normal,
                &x,
                &y,
                &z,
                count as i32,
                bias,
            );
            assert_eq!(vertex_index, index as i32);
            assert_eq!(support.to_bits(), expected.to_bits());
        }
    }
}
