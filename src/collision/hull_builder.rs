//! Convex hull construction (incremental) and procedural hulls.

use std::collections::{HashMap, HashSet};

use crate::base::constants as b3;
use crate::collision::hull::{hull_from_faces, Hull};
use crate::collision::internal_math::*;
use crate::math::functions::*;
use crate::math::types::{Transform, Vec3};

fn dedupe_points(points: &[Vec3], eps: f32) -> Vec<Vec3> {
    let mut out: Vec<Vec3> = Vec::with_capacity(points.len());
    for p in points {
        if !is_valid_vec3(*p) {
            continue;
        }
        let mut unique = true;
        for q in &out {
            if distance_squared(*p, *q) <= eps * eps {
                unique = false;
                break;
            }
        }
        if unique {
            out.push(*p);
        }
    }
    out
}

fn face_normal(points: &[Vec3], face: &[u32]) -> Vec3 {
    let a = points[face[0] as usize];
    let b = points[face[1] as usize];
    let c = points[face[2] as usize];
    normalize(cross(b - a, c - a))
}

fn face_center(points: &[Vec3], face: &[u32]) -> Vec3 {
    let mut c = Vec3::ZERO;
    for &i in face {
        c += points[i as usize];
    }
    c / face.len() as f32
}

fn orient_face(points: &[Vec3], face: &mut Vec<u32>, interior: Vec3) {
    let normal = face_normal(points, face);
    let center = face_center(points, face);
    if dot(normal, center - interior) < 0.0 {
        face.reverse();
    }
}

/// Merge coplanar triangles into polygon faces.
fn merge_coplanar(points: &[Vec3], triangles: Vec<Vec<u32>>, eps: f32) -> Vec<Vec<u32>> {
    let mut used = vec![false; triangles.len()];
    let mut result = Vec::with_capacity(triangles.len());

    for i in 0..triangles.len() {
        if used[i] {
            continue;
        }
        let normal_i = face_normal(points, &triangles[i]);
        let center_i = face_center(points, &triangles[i]);
        let offset_i = dot(normal_i, center_i);
        used[i] = true;
        let mut group = vec![i];
        for j in (i + 1)..triangles.len() {
            if used[j] {
                continue;
            }
            let normal_j = face_normal(points, &triangles[j]);
            if dot(normal_i, normal_j) < 0.999 {
                continue;
            }
            let center_j = face_center(points, &triangles[j]);
            if abs_float(dot(normal_i, center_j) - offset_i) > eps {
                continue;
            }
            used[j] = true;
            group.push(j);
        }

        if group.len() == 1 {
            result.push(triangles[i].clone());
            continue;
        }

        // Collect boundary edges of the coplanar group.
        let mut edges: HashSet<(u32, u32)> = HashSet::new();
        for &g in &group {
            let f = &triangles[g];
            for k in 0..f.len() {
                edges.insert((f[k], f[(k + 1) % f.len()]));
            }
        }
        let boundary: Vec<(u32, u32)> = edges
            .iter()
            .filter(|(a, b)| !edges.contains(&(*b, *a)))
            .copied()
            .collect();

        let mut next: HashMap<u32, u32> = HashMap::new();
        let mut ok = true;
        for &(a, b) in &boundary {
            if next.insert(a, b).is_some() {
                ok = false;
                break;
            }
        }
        if !ok || boundary.len() < 3 {
            for &g in &group {
                result.push(triangles[g].clone());
            }
            continue;
        }

        let start = boundary[0].0;
        let mut poly = Vec::with_capacity(boundary.len());
        let mut cursor = start;
        while poly.len() <= boundary.len() {
            poly.push(cursor);
            let Some(&n) = next.get(&cursor) else {
                ok = false;
                break;
            };
            cursor = n;
            if cursor == start {
                break;
            }
        }
        if !ok || poly.len() != boundary.len() || poly.len() < 3 {
            for &g in &group {
                result.push(triangles[g].clone());
            }
            continue;
        }
        result.push(poly);
    }

    result
}

fn build_from_points(points: &[Vec3]) -> Option<Hull> {
    if points.len() < 4 {
        return None;
    }

    let mut i0 = 0usize;
    for i in 1..points.len() {
        if points[i].x < points[i0].x || (points[i].x == points[i0].x && points[i].y < points[i0].y) {
            i0 = i;
        }
    }

    let mut i1 = usize::MAX;
    let mut best = -1.0f32;
    for i in 0..points.len() {
        if i == i0 {
            continue;
        }
        let d = distance_squared(points[i], points[i0]);
        if d > best {
            best = d;
            i1 = i;
        }
    }

    let mut i2 = usize::MAX;
    let mut best = -1.0f32;
    for i in 0..points.len() {
        if i == i0 || i == i1 {
            continue;
        }
        let n = cross(points[i1] - points[i0], points[i] - points[i0]);
        let area = length_squared(n);
        if area > best {
            best = area;
            i2 = i;
        }
    }

    let mut i3 = usize::MAX;
    let mut best = -1.0f32;
    for i in 0..points.len() {
        if i == i0 || i == i1 || i == i2 {
            continue;
        }
        let v = abs_float(signed_volume(
            points[i0],
            points[i1],
            points[i2],
            points[i],
        ));
        if v > best {
            best = v;
            i3 = i;
        }
    }

    if i1 == usize::MAX || i2 == usize::MAX || i3 == usize::MAX || best <= 1e-12 {
        return None;
    }

    let scale = max_element(points[i1] - points[i0]);
    let eps = max_float(1e-6 * scale, 1e-9);

    let interior = (points[i0] + points[i1] + points[i2] + points[i3]) / 4.0;
    let mut faces: Vec<Vec<u32>> = vec![
        vec![i0 as u32, i1 as u32, i2 as u32],
        vec![i0 as u32, i3 as u32, i1 as u32],
        vec![i1 as u32, i3 as u32, i2 as u32],
        vec![i2 as u32, i3 as u32, i0 as u32],
    ];
    for f in &mut faces {
        orient_face(points, f, interior);
    }

    let mut processed = [false; 4];
    processed[0] = true;
    // Track the tetra indices in a set so they are not re-added.
    let tetra = [i0, i1, i2, i3];

    for index in 0..points.len() {
        if tetra.contains(&index) {
            continue;
        }
        let p = points[index];
        let mut visible = vec![false; faces.len()];
        let mut any_visible = false;
        for (fi, f) in faces.iter().enumerate() {
            let normal = face_normal(points, f);
            if dot(normal, p - points[f[0] as usize]) > eps {
                visible[fi] = true;
                any_visible = true;
            }
        }
        if !any_visible {
            continue;
        }

        // Collect horizon edges.
        let mut edge_count: HashMap<(u32, u32), i32> = HashMap::new();
        for (fi, f) in faces.iter().enumerate() {
            if !visible[fi] {
                continue;
            }
            for k in 0..f.len() {
                let e = (f[k], f[(k + 1) % f.len()]);
                *edge_count.entry(e).or_insert(0) += 1;
            }
        }
        let mut horizon = Vec::new();
        for (&(a, b), &count) in &edge_count {
            if count > 0 && !edge_count.contains_key(&(b, a)) {
                horizon.push((a, b));
            }
        }
        if horizon.is_empty() {
            continue;
        }

        // Remove visible faces.
        let mut kept = Vec::with_capacity(faces.len());
        for (fi, f) in faces.into_iter().enumerate() {
            if !visible[fi] {
                kept.push(f);
            }
        }
        faces = kept;

        for (a, b) in horizon {
            if a == b {
                continue;
            }
            let mut f = vec![a, b, index as u32];
            let normal = face_normal(points, &f);
            if length_squared(normal) > 1e-12 {
                orient_face(points, &mut f, interior);
                faces.push(f);
            }
        }
    }

    // Drop degenerate faces and merge coplanar triangles.
    faces.retain(|f| {
        f.len() >= 3
            && length_squared(cross(
                points[f[1] as usize] - points[f[0] as usize],
                points[f[2] as usize] - points[f[0] as usize],
            )) > eps * eps
    });
    let mut faces = merge_coplanar(points, faces, eps * 4.0);

    // Remove unreferenced vertices.
    let mut used = vec![false; points.len()];
    for f in &faces {
        for &v in f {
            used[v as usize] = true;
        }
    }
    let mut remap = vec![u32::MAX; points.len()];
    let mut compact = Vec::new();
    for (i, &u) in used.iter().enumerate() {
        if u {
            remap[i] = compact.len() as u32;
            compact.push(points[i]);
        }
    }
    for f in &mut faces {
        for v in f.iter_mut() {
            *v = remap[*v as usize];
        }
    }

    // Re-orient using the compact interior.
    let interior = compact.iter().copied().fold(Vec3::ZERO, |a, b| a + b) / compact.len() as f32;
    for f in &mut faces {
        orient_face(&compact, f, interior);
    }

    if compact.len() > b3::MAX_HULL_VERTICES as usize || faces.len() > b3::MAX_HULL_FACES as usize {
        return None;
    }

    hull_from_faces(&compact, &faces)
}

/// Create a convex hull from a point set.
pub fn create_hull(points: &[Vec3], max_vertex_count: i32) -> Option<Hull> {
    if points.len() < 4 || max_vertex_count < 4 {
        return None;
    }
    let eps = 1e-6;
    let deduped = dedupe_points(points, eps);
    if deduped.len() > max_vertex_count as usize {
        // Keep the extreme points deterministically.
        let mut trimmed = deduped.clone();
        trimmed.truncate(max_vertex_count as usize);
        return build_from_points(&trimmed);
    }
    build_from_points(&deduped)
}

fn ring_points(radius: f32, y: f32, sides: i32, offset: f32) -> Vec<Vec3> {
    let mut out = Vec::with_capacity(sides as usize);
    for i in 0..sides {
        let angle = offset + 2.0 * b3::PI * i as f32 / sides as f32;
        out.push(Vec3::new(radius * angle.cos(), y, radius * angle.sin()));
    }
    out
}

/// A capped cylinder hull.
pub fn create_cylinder(height: f32, radius: f32, y_offset: f32, sides: i32) -> Option<Hull> {
    if height <= 0.0 || radius <= 0.0 || sides < 3 {
        return None;
    }
    let y1 = -0.5 * height + y_offset;
    let y2 = 0.5 * height + y_offset;
    let mut points = ring_points(radius, y1, sides, 0.0);
    points.extend(ring_points(radius, y2, sides, 0.0));
    create_hull(&points, b3::MAX_HULL_VERTICES)
}

/// A capped cone hull.
pub fn create_cone(height: f32, radius1: f32, radius2: f32, slices: i32) -> Option<Hull> {
    if height <= 0.0 || radius1 < 0.0 || radius2 < 0.0 || slices < 3 {
        return None;
    }
    if radius1 == 0.0 && radius2 == 0.0 {
        return None;
    }
    let y1 = -0.5 * height;
    let y2 = 0.5 * height;
    let mut points = Vec::new();
    if radius1 > 0.0 {
        points.extend(ring_points(radius1, y1, slices, 0.0));
    } else {
        points.push(Vec3::new(0.0, y1, 0.0));
    }
    if radius2 > 0.0 {
        points.extend(ring_points(radius2, y2, slices, 0.5 * b3::PI / slices as f32));
    } else {
        points.push(Vec3::new(0.0, y2, 0.0));
    }
    create_hull(&points, b3::MAX_HULL_VERTICES)
}

fn lcg(state: &mut u64) -> f32 {
    *state = state
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*state >> 40) as f32) / ((1u64 << 24) as f32)
}

/// A deterministic irregular rock hull.
pub fn create_rock(radius: f32) -> Option<Hull> {
    let mut state: u64 = 0x9E3779B97F4A7C15;
    let mut points = Vec::with_capacity(32);
    for _ in 0..32 {
        let z = -1.0 + 2.0 * lcg(&mut state);
        let theta = 2.0 * b3::PI * lcg(&mut state);
        let r = 0.85 + 0.3 * lcg(&mut state);
        let s = (1.0 - z * z).sqrt();
        points.push(Vec3::new(r * s * theta.cos() * radius, r * z * radius, r * s * theta.sin() * radius));
    }
    create_hull(&points, b3::MAX_HULL_VERTICES)
}

/// A deterministic complex hull.
pub fn create_complex_hull(radius: f32) -> Option<Hull> {
    let mut state: u64 = 0xD1B54A32D192ED03;
    let mut points = Vec::with_capacity(64);
    for _ in 0..64 {
        points.push(Vec3::new(
            (-1.0 + 2.0 * lcg(&mut state)) * radius,
            (-1.0 + 2.0 * lcg(&mut state)) * radius,
            (-1.0 + 2.0 * lcg(&mut state)) * radius,
        ));
    }
    create_hull(&points, b3::MAX_HULL_VERTICES)
}

/// Clone a hull and apply a transform and scale.
pub fn clone_and_transform_hull(original: &Hull, transform: Transform, scale: Vec3) -> Option<Hull> {
    let points = original.points();
    let transformed: Vec<Vec3> = points
        .iter()
        .map(|p| {
            let s = Vec3::new(p.x * scale.x, p.y * scale.y, p.z * scale.z);
            transform_point(transform, s)
        })
        .collect();
    create_hull(&transformed, b3::MAX_HULL_VERTICES)
}
