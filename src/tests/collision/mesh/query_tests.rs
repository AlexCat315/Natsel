//! Ported from Tests/NatselTests/Collision/Mesh/QueryTests.swift.
//!
//! Scaled shape casts, transformed overlaps, ordered triangle queries, and mover planes must be
//! repeatable across all nine geometry/scale combinations.

use crate::collision::mesh::overlap::{
    collide_mover_and_mesh, overlap_mesh, query_mesh, shape_cast_mesh,
};
use crate::collision::mesh::procedural::{
    create_grid_mesh, create_torus_mesh, create_wave_mesh,
};
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Transform, Vec3};
use crate::types::geometry::{Capsule, Mesh};
use crate::types::queries::{CastOutput, ShapeCastInput, ShapeProxy};

/// Swift's `QueryDigest`: FNV over positions, words, floats, vectors, and cast outputs.
struct QueryDigest {
    value: u64,
}

impl QueryDigest {
    fn new() -> Self {
        Self {
            value: 14695981039346656037,
        }
    }

    fn add_u32(&mut self, word: u32) {
        for byte in 0..4 {
            self.value ^= ((word >> (8 * byte)) & 255) as u64;
            self.value = self.value.wrapping_mul(1099511628211);
        }
    }

    fn add_f32(&mut self, scalar: f32) {
        self.add_u32(scalar.to_bits());
    }

    fn add_vec3(&mut self, vector: Vec3) {
        self.add_f32(vector.x);
        self.add_f32(vector.y);
        self.add_f32(vector.z);
    }

    fn add_cast_output(&mut self, out: &CastOutput) {
        self.add_vec3(out.normal);
        self.add_vec3(out.point);
        self.add_f32(out.fraction);
        self.add_u32(out.iterations as u32);
        self.add_u32(out.triangle_index as u32);
        self.add_u32(out.child_index as u32);
        self.add_u32(out.material_index as u32);
        self.add_u32(if out.hit { 1 } else { 0 });
    }
}

#[test]
fn queries() {
    // "scaled casts, transformed overlaps, triangle queries, and mover planes are repeatable"
    // Swift's `arguments: 0..<9` expands to one case per geometry/scale combination.
    for scenario in 0..9 {
        fn snapshot(scenario: usize) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let geometry = match scenario / 3 {
                0 => create_grid_mesh(5, 4, 0.75, 3, true),
                1 => create_wave_mesh(5, 4, 0.75, 0.5, 0.3, 0.2),
                _ => create_torus_mesh(7, 9, 1.25, 0.375),
            };
            let data = geometry.expect("mesh creation failed");
            samples.push(data.hash());
            let scales = [
                Vec3::ONE,
                Vec3::new(-2.0, 0.5, 1.5),
                Vec3::new(1.0, -1.0, 1.0),
            ];
            let mesh = Mesh::new(Some(data), scales[scenario % 3]);
            let transform = Transform::new(
                Vec3::new(0.125, -0.25, 0.5),
                make_quat_from_axis_angle(Vec3::AXIS_Y, 0.35),
            );
            let mut casts = QueryDigest::new();
            let mut overlaps = QueryDigest::new();
            let mut queries = QueryDigest::new();
            let mut movers = QueryDigest::new();
            for i in 0..80i32 {
                let x = (i % 10) as f32 * 0.5 - 2.25;
                let z = (i % 8) as f32 * 0.5 - 1.75;
                let y: f32 = if i < 40 { 2.0 } else { -2.0 };
                let mut points = [Vec3::new(x, y, z), Vec3::new(x + 0.125, y + 0.5, z - 0.25)];
                let mut input = ShapeCastInput::default();
                input.translation = Vec3::new(0.125, if i < 40 { -4.0 } else { 4.0 }, -0.25);
                input.max_fraction = if i % 3 == 0 { 0.5 } else { 1.0 };
                input.can_encroach = i % 2 == 0;
                input.proxy = ShapeProxy::new(Some(points.as_ptr()), 2, 0.25);
                casts.add_cast_output(&shape_cast_mesh(&mesh, input));

                points[0].y = (i % 5) as f32 * 0.2 - 0.3;
                points[1].y = points[0].y + 0.5;
                let proxy = ShapeProxy::new(Some(points.as_ptr()), 2, 0.25);
                overlaps.add_u32(if overlap_mesh(&mesh, transform, proxy) {
                    1
                } else {
                    0
                });

                let center = Vec3::new(x, (i % 3) as f32 * 0.3 - 0.3, z);
                let extent = Vec3::new(0.375, 0.2, 0.375);
                let mut count = 0;
                let limit = if i % 4 == 0 { 3 } else { 100000 };
                query_mesh(
                    &mesh,
                    crate::math::types::AABB::new(center - extent, center + extent),
                    |a, b, c, index| {
                        queries.add_vec3(a);
                        queries.add_vec3(b);
                        queries.add_vec3(c);
                        queries.add_u32(index as u32);
                        count += 1;
                        count < limit
                    },
                );
                queries.add_u32(count as u32);

                let capsule = Capsule::new(points[0], points[1], 0.4);
                let planes = collide_mover_and_mesh(&mesh, capsule, (i % 5) as usize);
                movers.add_u32(planes.len() as u32);
                for plane in planes.iter() {
                    movers.add_vec3(plane.plane.normal);
                    movers.add_f32(plane.plane.offset);
                    movers.add_vec3(plane.point);
                    movers.add_u32(plane.triangle_index as u32);
                    movers.add_u32(plane.child_index as u32);
                    movers.add_u32(plane.material_index as u32);
                }
            }
            samples.push(casts.value);
            samples.push(overlaps.value);
            samples.push(queries.value);
            samples.push(movers.value);
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}
