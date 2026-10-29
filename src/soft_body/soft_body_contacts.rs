//! Soft body self collision and soft body vs soft body contact.
//!
//! Once per step, contacts are found between vertices and triangles (or vertices and vertices for
//! bodies without faces, such as ropes) that are within the collision thickness plus the distance
//! both sides can travel in the step. A uniform grid of triangle bounds finds the candidates. Each
//! substep projects the contacts with XPBD: a vertex keeps `thickness` from the triangle on the side
//! it started on, with the correction shared by the vertex and the triangle's vertices through the
//! barycentric weights, and friction removes relative tangential motion up to mu times the
//! correction. Self contacts skip a vertex's own triangles and those of its edge neighbors, and pairs
//! already closer than the thickness in the rest pose.
//!
//! Soft bodies that may touch are simulated together, substep by substep, on one thread, so the
//! result does not depend on the worker count.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodyContacts.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::soft_body::soft_body_runtime::SoftBodyRecord`: fields `id: i32`, `vertex_count: i32`,
//!   `settings: SoftBodySettings` (with `vertices: Vec<SoftBodyVertex>` where
//!   `vertex.position: Vec3`, `faces: Vec<SoftBodyFace>` where `face.vertex` is a `u32` triple with
//!   `.x`/`.y`/`.z` and `face.is_degenerate() -> bool`, `edges: Vec<SoftBodyEdge>` with
//!   `edge.vertex.x`/`edge.vertex.y`), `positions: Vec<Vec3>`, `velocities: Vec<Vec3>`,
//!   `inv_masses: Vec<f32>`, `bounds: AABB`, `filter: Filter`, `friction: f32`, `gravity_scale: f32`,
//!   `max_speed: f32`, `vertex_radius: f32`, `collision_thickness: f32`, `average_edge_length: f32`,
//!   `enable_self_collision: bool`, `enable_edge_collision: bool`, `collide_with_soft_bodies: bool`,
//!   `neighbor_starts: Vec<i32>`, `neighbors: Vec<u32>`, `surface_vertices: Vec<i32>`,
//!   `closed_surface: bool`, `mesh_edges: Vec<UInt2>` (the `SIMD2<UInt32>` element type with `.x`
//!   and `.y`), `excluded_face_starts`/`excluded_faces`/`excluded_edge_starts`/`excluded_edges:
//!   Vec<i32>`, `contact_face_tree: SoftContactTree`, `contact_edge_tree: SoftContactTree`,
//!   `self_exclusion_distance: f32`, `face_sides: HashMap<u64, f32>`.
//! - `crate::soft_body::soft_body_edge_collision::SoftBodyRecord::build_mesh_edges` (called through
//!   the record).
//! - `crate::soft_body::soft_contact_tree::SoftContactTree::{refit, visit}` - landed; Swift's
//!   defaults (`afterFeature = -1`, `referenceExclusion = nil`, `referenceBounds = { [] }`) are
//!   passed explicitly below.
//!
//! Call-site contract for `soft_body_solver.rs` (owned by another agent):
//! - `World::detect_soft_contacts(records, group, dt, &mut contacts, &mut edge_contacts, parallel,
//!   detection_workers)` - Swift's `parallel: Bool = false` and `detectionWorkers: Int = 1` defaults
//!   become required arguments; `group` is `&[usize]` (Swift `[Int]` body slots).
//! - `solve_soft_contacts(&contacts, &bodies, max_recovery)` and
//!   `solve_soft_edge_contacts(&edge_contacts, &bodies, max_recovery)` take `&[SoftContactBody]`;
//!   `SoftContactBody { x: *mut Vec3, x0: *mut Vec3, w: *const f32 }` carries Swift's
//!   `withUnsafe*(Mutable)BufferPointer` fields as raw pointers (the buffers never reallocate during
//!   a step; `previous_positions` is Swift's `x0`).
//!
//! Borrowing adaptations (behaviour identical):
//! - Swift passed `SoftBodyRecord` by mutable raw pointer and DetectionBody values by COW copy. The
//!   detection phase only reads the records - the sole writes (exclusion building and the final
//!   `faceSides` history update) happen while no shared borrow is live - so shared references
//!   derived from the raw pointers inside the private find functions observe exactly the values
//!   Swift's `pointee` reads did. `DetectionBody::face_tree`/`edge_tree`/`grid` hold owned copies
//!   where Swift's COW shared the record's storage; nothing mutates the record's trees after the
//!   copy.
//! - Swift's `DispatchQueue.concurrentPerform` becomes `thread::scope` spawning one task per
//!   iteration; every task writes only its own contact list and reads the records, and the lists are
//!   concatenated in index order, so the result is independent of the worker count exactly as in
//!   Swift.
//! - `entries.sort { $0.item < $1.item }` becomes `sort_by_key` (Rust's stable sort): two entries of
//!   one cell always carry different items (a feature occupies a cell at most once), so the chains
//!   built afterwards are identical to Swift's.

use std::collections::{HashMap, HashSet};
use std::ops::Range;
use std::thread;

use crate::math::functions::{
    aabb_inflate, aabb_overlaps, aabb_union, abs_float, cross, dot, get_length_and_normalize,
    length, length_squared, max_float, min_float, min_vec3, max_vec3, mul_add, mul_sub, normalize,
    segment_distance,
};
use crate::math::types::{AABB, Vec3};
use crate::simulation::shape_runtime::should_shapes_collide;
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_runtime::SoftBodyRecord;
use crate::soft_body::soft_contact_tree::SoftContactTree;

/// A vertex against a triangle, or against another vertex when the triangle slots repeat it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SoftContact {
    /// Group local body slots and vertex indices.
    pub vertex_body: i32,
    pub vertex: i32,
    pub triangle_body: i32,
    pub a: i32,
    pub b: i32,
    pub c: i32,
    /// Side of the triangle the vertex started on (+1 or -1); zero marks a vertex-vertex contact.
    pub side: f32,
    /// Face index in the triangle body, for the contact history.
    pub face: i32,
    pub thickness: f32,
    pub friction: f32,
}

/// Positions and inverse masses of one body during the contact projection.
#[derive(Clone, Copy, Debug)]
pub struct SoftContactBody {
    pub x: *mut Vec3,
    pub x0: *mut Vec3,
    pub w: *const f32,
}

/// Closest point on a triangle with barycentric coordinates (Ericson, Real-Time Collision
/// Detection 5.1.5). `interior` is true when the point projects inside the face.
#[inline(always)]
pub(crate) fn closest_triangle_barycentric(a: Vec3, b: Vec3, c: Vec3, p: Vec3) -> (f32, f32, f32, bool) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (1.0, 0.0, 0.0, false);
    }
    let bp = p - b;
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (0.0, 1.0, 0.0, false);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (1.0 - v, v, 0.0, false);
    }
    let cp = p - c;
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (0.0, 0.0, 1.0, false);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (1.0 - w, 0.0, w, false);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (0.0, 1.0 - w, w, false);
    }
    let denominator = 1.0 / (va + vb + vc);
    let v = vb * denominator;
    let w = vc * denominator;
    (1.0 - v - w, v, w, true)
}

struct CellEntry {
    key: i64,
    item: i32,
}

/// Exact cell keys with feature chains. Query order is independent of dictionary bucket order.
pub(crate) struct CellGrid {
    cell_size: f32,
    entries: Vec<CellEntry>,
    heads: HashMap<i64, isize>,
    next: Vec<isize>,
    last_item: i32,
    ordered_items: bool,
}

impl CellGrid {
    pub(crate) fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            entries: Vec::new(),
            heads: HashMap::new(),
            next: Vec::new(),
            last_item: i32::MIN,
            ordered_items: true,
        }
    }

    #[inline(always)]
    fn cell(&self, p: Vec3) -> (i64, i64, i64) {
        let inv = 1.0 / self.cell_size;
        (
            (p.x * inv).floor() as i64,
            (p.y * inv).floor() as i64,
            (p.z * inv).floor() as i64,
        )
    }

    #[inline(always)]
    fn key(x: i64, y: i64, z: i64) -> i64 {
        let mask: i64 = (1 << 21) - 1;
        ((x & mask) << 42) | ((y & mask) << 21) | (z & mask)
    }

    pub(crate) fn insert(&mut self, box_: AABB, item: i32) {
        if item < self.last_item {
            self.ordered_items = false;
        }
        self.last_item = item;
        let lower = self.cell(box_.lower_bound);
        let upper = self.cell(box_.upper_bound);
        for x in lower.0..=upper.0 {
            for y in lower.1..=upper.1 {
                for z in lower.2..=upper.2 {
                    self.entries.push(CellEntry {
                        key: Self::key(x, y, z),
                        item,
                    });
                }
            }
        }
    }

    pub(crate) fn finish(&mut self) {
        // Features are normally inserted in ascending index order. A stable per-cell chain then
        // needs no global cell sort, and lookups avoid a binary search for every visited cell.
        if !self.ordered_items {
            self.entries.sort_by_key(|entry| entry.item);
        }
        self.ordered_items = true;
        self.last_item = self.entries.last().map(|entry| entry.item).unwrap_or(i32::MIN);
        self.heads.clear();
        self.heads.reserve(self.entries.len() / 4);
        self.next = vec![-1isize; self.entries.len()];
        for index in (0..self.entries.len()).rev() {
            let previous = self.heads.insert(self.entries[index].key, index as isize);
            self.next[index] = previous.unwrap_or(-1);
        }
    }

    /// Items in every cell the box overlaps. An item spanning several of those cells is visited
    /// once per cell; callers deduplicate.
    pub(crate) fn visit_box<F>(&self, box_: AABB, mut body: F)
    where
        F: FnMut(i32),
    {
        let lower = self.cell(box_.lower_bound);
        let upper = self.cell(box_.upper_bound);
        for x in lower.0..=upper.0 {
            for y in lower.1..=upper.1 {
                for z in lower.2..=upper.2 {
                    self.visit_key(Self::key(x, y, z), &mut body);
                }
            }
        }
    }

    /// Items in the cell holding `p`.
    pub(crate) fn visit_point<F>(&self, p: Vec3, mut body: F)
    where
        F: FnMut(i32),
    {
        let c = self.cell(p);
        self.visit_key(Self::key(c.0, c.1, c.2), &mut body);
    }

    fn visit_key<F>(&self, key: i64, body: &mut F)
    where
        F: FnMut(i32),
    {
        let mut index = *self.heads.get(&key).unwrap_or(&-1);
        while index >= 0 {
            let slot = index as usize;
            body(self.entries[slot].item);
            index = self.next[slot];
        }
    }
}
impl SoftBodyRecord {
    /// Collision thickness for self and soft-soft contact.
    pub(crate) fn contact_thickness(&self) -> f32 {
        if self.collision_thickness > 0.0 {
            return self.collision_thickness;
        }
        max_float(2.0 * self.vertex_radius, 0.25 * self.average_edge_length)
    }

    /// Edge neighbors in compressed rows, built on first use.
    pub(crate) fn build_neighbors(&mut self) {
        if !self.neighbor_starts.is_empty() {
            return;
        }
        let n = self.vertex_count as usize;
        let mut lists: Vec<Vec<u32>> = vec![Vec::new(); n];
        for edge in self.settings.edges.iter() {
            lists[edge.vertex.x as usize].push(edge.vertex.y);
            lists[edge.vertex.y as usize].push(edge.vertex.x);
        }
        self.neighbor_starts.reserve(n + 1);
        for list in lists {
            self.neighbor_starts.push(self.neighbors.len() as i32);
            let mut list = list;
            list.sort();
            self.neighbors.extend_from_slice(&list);
        }
        self.neighbor_starts.push(self.neighbors.len() as i32);
    }

    /// For self contact: the faces each vertex ignores and the edges each edge ignores, found once
    /// in the rest pose. A vertex ignores its own faces, its edge neighbors' faces, and faces closer
    /// than the exclusion distance at rest (the surface's own neighborhood, not a fold); an edge
    /// likewise. Detection then rejects these candidates with a binary search.
    pub(crate) fn build_self_exclusions(&mut self) {
        let exclusion = max_float(self.contact_thickness(), 2.0 * self.average_edge_length);
        if exclusion == self.self_exclusion_distance && !self.excluded_face_starts.is_empty() {
            return;
        }
        self.self_exclusion_distance = exclusion;
        self.build_neighbors();
        self.build_mesh_edges();
        let rest: Vec<Vec3> = self.settings.vertices.iter().map(|v| v.position).collect();
        // Swift read `settings.faces` / `settings.meshEdges` through copy-on-write borrows while it
        // appended to the exclusion rows; Rust snapshot feature vertices the same way.
        let faces: Vec<[u32; 3]> = self
            .settings
            .faces
            .iter()
            .map(|f| [f.vertex.x, f.vertex.y, f.vertex.z])
            .collect();
        let mesh_edges: Vec<[u32; 2]> = self.mesh_edges.iter().map(|e| [e.x, e.y]).collect();
        let pad = Vec3::new(exclusion, exclusion, exclusion);
        let cell = max_float(max_float(1.5 * self.average_edge_length, exclusion), 1.0e-3);

        self.excluded_face_starts.clear();
        self.excluded_faces.clear();
        if !faces.is_empty() {
            // Unpadded features in the grid, padded queries: each feature sits in few cells.
            let mut grid = CellGrid::new(cell);
            for (f, face) in faces.iter().enumerate() {
                let a = rest[face[0] as usize];
                let b = rest[face[1] as usize];
                let c = rest[face[2] as usize];
                grid.insert(
                    AABB::new(min_vec3(a, min_vec3(b, c)), max_vec3(a, max_vec3(b, c))),
                    f as i32,
                );
            }
            grid.finish();
            let mut list: Vec<i32> = Vec::new();
            let mut stamp: Vec<i32> = vec![-1; faces.len()];
            for v in 0..self.vertex_count as usize {
                list.clear();
                let r = rest[v];
                let vertex = v as i32;
                grid.visit_box(AABB::new(r - pad, r + pad), |f| {
                    if stamp[f as usize] == vertex {
                        return;
                    }
                    stamp[f as usize] = vertex;
                    let face = faces[f as usize];
                    let ia = face[0] as i32;
                    let ib = face[1] as i32;
                    let ic = face[2] as i32;
                    let mut excluded = self.are_neighbors(vertex, ia)
                        || self.are_neighbors(vertex, ib)
                        || self.are_neighbors(vertex, ic);
                    if !excluded {
                        let ra = rest[ia as usize];
                        let rb = rest[ib as usize];
                        let rc = rest[ic as usize];
                        let (bu, bv, bw, _interior) = closest_triangle_barycentric(ra, rb, rc, r);
                        let closest = bu * ra + bv * rb + bw * rc;
                        excluded = length_squared(r - closest) < exclusion * exclusion;
                    }
                    if excluded {
                        list.push(f);
                    }
                });
                self.excluded_face_starts.push(self.excluded_faces.len() as i32);
                list.sort();
                self.excluded_faces.extend_from_slice(&list);
            }
            self.excluded_face_starts.push(self.excluded_faces.len() as i32);
        }

        self.excluded_edge_starts.clear();
        self.excluded_edges.clear();
        let mut grid = CellGrid::new(cell);
        for (e, edge) in mesh_edges.iter().enumerate() {
            let a = rest[edge[0] as usize];
            let b = rest[edge[1] as usize];
            grid.insert(AABB::new(min_vec3(a, b), max_vec3(a, b)), e as i32);
        }
        grid.finish();
        let mut list: Vec<i32> = Vec::new();
        let mut stamp: Vec<i32> = vec![-1; mesh_edges.len()];
        for (e, edge) in mesh_edges.iter().enumerate() {
            list.clear();
            let i0 = edge[0] as i32;
            let i1 = edge[1] as i32;
            let a = rest[i0 as usize];
            let b = rest[i1 as usize];
            let box_ = AABB::new(min_vec3(a, b) - pad, max_vec3(a, b) + pad);
            grid.visit_box(box_, |f| {
                if stamp[f as usize] == e as i32 {
                    return;
                }
                stamp[f as usize] = e as i32;
                let other = mesh_edges[f as usize];
                let j0 = other[0] as i32;
                let j1 = other[1] as i32;
                let c = rest[j0 as usize];
                let d = rest[j1 as usize];
                if !aabb_overlaps(box_, AABB::new(min_vec3(c, d), max_vec3(c, d))) {
                    return;
                }
                let mut excluded = self.are_neighbors(i0, j0)
                    || self.are_neighbors(i0, j1)
                    || self.are_neighbors(i1, j0)
                    || self.are_neighbors(i1, j1);
                if !excluded {
                    let result = segment_distance(a, b, c, d);
                    excluded = length_squared(result.point1 - result.point2) < exclusion * exclusion;
                }
                if excluded {
                    list.push(f);
                }
            });
            self.excluded_edge_starts.push(self.excluded_edges.len() as i32);
            list.sort();
            self.excluded_edges.extend_from_slice(&list);
        }
        self.excluded_edge_starts.push(self.excluded_edges.len() as i32);
    }

    #[inline(always)]
    fn contains(list: &[i32], start: i32, end: i32, value: i32) -> bool {
        let mut low = start as usize;
        let mut high = end as usize;
        while low < high {
            let mid = (low + high) / 2;
            if list[mid] < value {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        low < (end as usize) && list[low] == value
    }

    #[inline(always)]
    pub(crate) fn excludes_face(&self, vertex: i32, face: i32) -> bool {
        Self::contains(
            &self.excluded_faces,
            self.excluded_face_starts[vertex as usize],
            self.excluded_face_starts[vertex as usize + 1],
            face,
        )
    }

    #[inline(always)]
    pub(crate) fn excludes_edge(&self, edge: i32, other: i32) -> bool {
        Self::contains(
            &self.excluded_edges,
            self.excluded_edge_starts[edge as usize],
            self.excluded_edge_starts[edge as usize + 1],
            other,
        )
    }

    #[inline(always)]
    pub(crate) fn are_neighbors(&self, i: i32, j: i32) -> bool {
        if i == j {
            return true;
        }
        let start = self.neighbor_starts[i as usize] as usize;
        let end = self.neighbor_starts[i as usize + 1] as usize;
        for k in start..end {
            if self.neighbors[k] == j as u32 {
                return true;
            }
        }
        false
    }
}

#[inline(always)]
fn aabb_contains_point(box_: AABB, p: Vec3) -> bool {
    p.x >= box_.lower_bound.x
        && p.y >= box_.lower_bound.y
        && p.z >= box_.lower_bound.z
        && p.x <= box_.upper_bound.x
        && p.y <= box_.upper_bound.y
        && p.z <= box_.upper_bound.z
}

/// Most triangles kept per vertex and body: the closest ones. A vertex resting on a surface is
/// near several triangles; the nearest few already hold it, and more only cost iterations.
const MAX_TRIANGLES_PER_VERTEX: usize = 4;

impl SoftBodyRecord {
    /// Vertices on the surface: all of them for bodies without faces.
    /// Also decides whether the faces form a closed, consistently wound surface: then every
    /// directed edge has its reverse in another face.
    pub(crate) fn build_surface_vertices(&mut self) {
        if !self.surface_vertices.is_empty() {
            return;
        }
        if self.settings.faces.is_empty() {
            self.surface_vertices = (0..self.vertex_count).collect();
            return;
        }
        let mut used = vec![false; self.vertex_count as usize];
        let mut directed: HashSet<u64> = HashSet::new();
        let mut duplicate = false;
        for face in self.settings.faces.iter() {
            used[face.vertex.x as usize] = true;
            used[face.vertex.y as usize] = true;
            used[face.vertex.z as usize] = true;
            let pairs = [
                (face.vertex.x, face.vertex.y),
                (face.vertex.y, face.vertex.z),
                (face.vertex.z, face.vertex.x),
            ];
            for (a, b) in pairs {
                if !directed.insert((a as u64) << 32 | b as u64) {
                    duplicate = true;
                }
            }
        }
        self.surface_vertices = (0..self.vertex_count).filter(|v| used[*v as usize]).collect();
        self.closed_surface = !duplicate
            && directed.iter().all(|d| directed.contains(&((*d & 0xffff_ffff) << 32 | (*d >> 32))));
    }
}

/// Per body data shared by every pair during detection.
struct DetectionBody {
    /// How far each vertex can move in the step.
    vertex_margins: Vec<f32>,
    /// Where each vertex can be during the step: from its position along its velocity, padded by
    /// what gravity and velocity changes can add.
    vertex_boxes: Vec<AABB>,
    /// Bounds of each face padded by its vertices' margins and the largest thickness into it.
    face_boxes: Vec<AABB>,
    face_margins: Vec<f32>,
    grid: Option<CellGrid>,
    face_tree: Option<SoftContactTree>,
    /// +1 or -1 for a closed surface wound outward or inward, 0 for an open one.
    closed_side: f32,
    /// Edge bounds padded like the faces, and their grid.
    edge_boxes: Vec<AABB>,
    edge_margins: Vec<f32>,
    edge_tree: Option<SoftContactTree>,
}

impl Default for DetectionBody {
    fn default() -> Self {
        Self {
            vertex_margins: Vec::new(),
            vertex_boxes: Vec::new(),
            face_boxes: Vec::new(),
            face_margins: Vec::new(),
            grid: None,
            face_tree: None,
            closed_side: 0.0,
            edge_boxes: Vec::new(),
            edge_margins: Vec::new(),
            edge_tree: None,
        }
    }
}

#[derive(Clone, Copy)]
struct Pair {
    slot_p: usize,
    slot_t: usize,
    thickness: f32,
}

/// Task context for Swift's `DispatchQueue.concurrentPerform` over the vertex pairs.
struct DetectBuffers<'a> {
    records: *mut SoftBodyRecord,
    group: &'a [usize],
    shared: &'a [DetectionBody],
    pairs: &'a [Pair],
    lists: *mut Vec<SoftContact>,
}

// SAFETY: `records` addresses the group's live, initialized records for the whole scope. During
// detection the records are read-only (the exclusion building happened earlier and the `faceSides`
// history is written only after the scope joins), `group`, `shared` and `pairs` are never mutated,
// and every task writes only `lists[k]` for its own distinct `k`. This is the contract Swift's
// `nonisolated(unsafe)` `@Sendable` closure captured.
unsafe impl Send for DetectBuffers<'_> {}
unsafe impl Sync for DetectBuffers<'_> {}

impl DetectBuffers<'_> {
    fn detect(&self, k: usize, list: &mut Vec<SoftContact>, source_range: Option<Range<usize>>) {
        let pair = self.pairs[k];
        // SAFETY: records are read-only during detection (see the `Send`/`Sync` note above) and
        // the group slots index live records.
        unsafe {
            let p = self.records.add(self.group[pair.slot_p]);
            let t = self.records.add(self.group[pair.slot_t]);
            let friction = ((*p).friction * (*t).friction).sqrt();
            if (*t).settings.faces.is_empty() {
                find_vertex_vertex_contacts(
                    p,
                    t,
                    pair.slot_p as i32,
                    pair.slot_t as i32,
                    pair.thickness,
                    friction,
                    pair.slot_p == pair.slot_t,
                    &self.shared[pair.slot_p],
                    &self.shared[pair.slot_t],
                    list,
                    source_range,
                );
            } else {
                find_vertex_triangle_contacts(
                    p,
                    t,
                    pair.slot_p as i32,
                    pair.slot_t as i32,
                    pair.thickness,
                    friction,
                    pair.slot_p == pair.slot_t,
                    &self.shared[pair.slot_p],
                    &self.shared[pair.slot_t],
                    list,
                    source_range,
                );
            }
        }
    }
}

/// Task context for Swift's `DispatchQueue.concurrentPerform` over the edge pairs.
struct EdgeDetectBuffers<'a> {
    records: *mut SoftBodyRecord,
    group: &'a [usize],
    shared: &'a [DetectionBody],
    pairs: &'a [Pair],
    lists: *mut Vec<SoftEdgeContact>,
}

// SAFETY: same contract as `DetectBuffers`: read-only records and per-task disjoint output lists.
unsafe impl Send for EdgeDetectBuffers<'_> {}
unsafe impl Sync for EdgeDetectBuffers<'_> {}

impl EdgeDetectBuffers<'_> {
    fn detect(&self, k: usize, list: &mut Vec<SoftEdgeContact>, source_range: Option<Range<usize>>) {
        let pair = self.pairs[k];
        // SAFETY: records are read-only during detection (see the `Send`/`Sync` note above).
        unsafe {
            let p = self.records.add(self.group[pair.slot_p]);
            let t = self.records.add(self.group[pair.slot_t]);
            let friction = ((*p).friction * (*t).friction).sqrt();
            find_edge_edge_contacts(
                p,
                t,
                pair.slot_p as i32,
                pair.slot_t as i32,
                pair.thickness,
                friction,
                pair.slot_p == pair.slot_t,
                &self.shared[pair.slot_p],
                &self.shared[pair.slot_t],
                list,
                source_range,
            );
        }
    }
}
impl World {
    /// Find vertex-triangle and vertex-vertex contacts within a group of soft bodies.
    ///
    /// A vertex and a feature can meet when their distance is below the thickness plus how far
    /// each can move: its own fastest speed and what gravity adds over the step (a tighter bound
    /// from relative velocities misses contacts when one body stops on the ground mid step). Each
    /// feature is padded by its own motion in the grid and each vertex queries the cells its own
    /// motion box covers, so slow vertices test few candidates.
    ///
    /// Swift's defaulted `parallel: Bool = false` and `detectionWorkers: Int = 1` are required
    /// arguments; call sites converted from Swift pass them explicitly.
    pub(crate) fn detect_soft_contacts(
        &self,
        records: *mut SoftBodyRecord,
        group: &[usize],
        dt: f32,
        contacts: &mut Vec<SoftContact>,
        edge_contacts: &mut Vec<SoftEdgeContact>,
        parallel: bool,
        detection_workers: usize,
    ) {
        contacts.clear();
        edge_contacts.clear();

        let mut pairs: Vec<Pair> = Vec::new();
        let mut body_margins = vec![0.0f32; group.len()];
        // SAFETY: `records` addresses live records at every `group` index for the whole call (the
        // Swift `UnsafeMutablePointer` contract); each statement borrows exactly one record.
        unsafe {
            for (slot, index) in group.iter().enumerate() {
                let gain =
                    length(self.gravity) * abs_float((*records.add(*index)).gravity_scale) * dt;
                body_margins[slot] = ((*records.add(*index)).max_speed + gain) * dt;
            }
        }
        let mut target_thickness = vec![0.0f32; group.len()];
        // SAFETY: as above; the mutable borrow for the build calls ends before the inner read loop.
        unsafe {
            for slot_p in 0..group.len() {
                let index_p = group[slot_p];
                {
                    let record_p = &mut *records.add(index_p);
                    if record_p.enable_self_collision {
                        record_p.build_neighbors();
                        if record_p.enable_edge_collision || !record_p.settings.faces.is_empty() {
                            record_p.build_self_exclusions();
                        }
                    }
                    record_p.build_surface_vertices();
                }
                for slot_t in 0..group.len() {
                    let p = records.add(index_p);
                    let t = records.add(group[slot_t]);
                    let p_faces = !(*p).settings.faces.is_empty();
                    let t_faces = !(*t).settings.faces.is_empty();
                    if slot_p == slot_t {
                        if !(*p).enable_self_collision {
                            continue;
                        }
                    } else {
                        if !((*p).collide_with_soft_bodies
                            && (*t).collide_with_soft_bodies
                            && should_shapes_collide(&(*p).filter, &(*t).filter))
                        {
                            continue;
                        }
                        // Vertex-vertex pairs between face-less bodies are found once.
                        if !p_faces && !t_faces && slot_t < slot_p {
                            continue;
                        }
                        // Faces of P are tested against vertices of T in the mirrored pass.
                        if !t_faces && p_faces {
                            continue;
                        }
                    }
                    let thickness = max_float((*p).contact_thickness(), (*t).contact_thickness());
                    let reach = thickness + body_margins[slot_p] + body_margins[slot_t];
                    if !aabb_overlaps(aabb_inflate((*p).bounds, reach), (*t).bounds) {
                        continue;
                    }
                    pairs.push(Pair {
                        slot_p,
                        slot_t,
                        thickness,
                    });
                    target_thickness[slot_t] = max_float(target_thickness[slot_t], thickness);
                }
            }
        }
        // Edge-edge pairs, unordered: an edge passing between two vertices of the other surface
        // touches no vertex or face there.
        let mut edge_pairs: Vec<Pair> = Vec::new();
        // SAFETY: as above; the mutable borrows for `build_mesh_edges` are statement-scoped.
        unsafe {
            for slot_p in 0..group.len() {
                let index_p = group[slot_p];
                let p = records.add(index_p);
                for slot_t in slot_p..group.len() {
                    let t = records.add(group[slot_t]);
                    if slot_p == slot_t {
                        if !(*p).enable_self_collision {
                            continue;
                        }
                    } else {
                        if !((*p).collide_with_soft_bodies
                            && (*t).collide_with_soft_bodies
                            && should_shapes_collide(&(*p).filter, &(*t).filter))
                        {
                            continue;
                        }
                    }
                    if !(*p).enable_edge_collision || !(*t).enable_edge_collision {
                        continue;
                    }
                    (*p).build_mesh_edges();
                    (*t).build_mesh_edges();
                    if (*p).mesh_edges.is_empty() || (*t).mesh_edges.is_empty() {
                        continue;
                    }
                    let thickness = max_float((*p).contact_thickness(), (*t).contact_thickness());
                    let reach = thickness + body_margins[slot_p] + body_margins[slot_t];
                    if !aabb_overlaps(aabb_inflate((*p).bounds, reach), (*t).bounds) {
                        continue;
                    }
                    edge_pairs.push(Pair {
                        slot_p,
                        slot_t,
                        thickness,
                    });
                }
            }
        }
        if pairs.is_empty() && edge_pairs.is_empty() {
            // Nothing in reach: forget the old sides.
            // SAFETY: no shared borrows of the records are alive here.
            unsafe {
                for index in group {
                    (*records.add(*index)).face_sides.clear();
                }
            }
            return;
        }

        // Margins of every body in a pair, and a grid per target.
        let mut data: Vec<DetectionBody> = vec![DetectionBody::default(); group.len()];
        let mut needed = vec![false; group.len()];
        for pair in pairs.iter().chain(edge_pairs.iter()) {
            needed[pair.slot_p] = true;
            needed[pair.slot_t] = true;
        }
        let gravity_length = length(self.gravity);
        // SAFETY: statement-scoped record reads; `data` is world-local storage.
        unsafe {
            for slot in 0..group.len() {
                if !needed[slot] {
                    continue;
                }
                let record = records.add(group[slot]);
                let gain = gravity_length * abs_float((*record).gravity_scale) * dt * dt;
                data[slot].vertex_margins = (*record)
                    .velocities
                    .iter()
                    .map(|v| length(*v) * dt + gain)
                    .collect();
                let mut vertex_boxes = Vec::with_capacity((*record).velocities.len());
                for i in 0..(*record).velocities.len() {
                    let x = (*record).positions[i];
                    let v = (*record).velocities[i];
                    let end = mul_add(x, dt, v);
                    // A quarter of the step's travel in any direction covers turns and pushes.
                    let slack = gain + 0.25 * length(v) * dt;
                    vertex_boxes.push(aabb_inflate(
                        AABB::new(min_vec3(x, end), max_vec3(x, end)),
                        slack,
                    ));
                }
                data[slot].vertex_boxes = vertex_boxes;
            }
        }
        // SAFETY: the `refit` calls are the only record writes in this phase and each is
        // statement-scoped; the tree is copied out afterwards, matching Swift's copy-on-write
        // share (nothing mutates the record's trees after the copy).
        unsafe {
            for pair in pairs.iter() {
                if data[pair.slot_t].grid.is_some() || data[pair.slot_t].face_tree.is_some() {
                    continue;
                }
                let slot = pair.slot_t;
                let t = records.add(group[slot]);
                if (*t).settings.faces.is_empty() {
                    let mut grid = CellGrid::new(max_float(
                        max_float((*t).average_edge_length, target_thickness[slot]),
                        1.0e-3,
                    ));
                    let count = (*t).positions.len();
                    // Swift bound `data[slot].vertexMargins` and discarded it (`_ = margins`); the
                    // per-vertex sweep boxes carry the padding instead.
                    for j in 0..count {
                        let sweep = data[slot].vertex_boxes[j];
                        grid.insert(aabb_inflate(sweep, target_thickness[slot]), j as i32);
                    }
                    grid.finish();
                    data[slot].grid = Some(grid);
                } else {
                    let face_count = (*t).settings.faces.len();
                    let mut boxes = vec![AABB::default(); face_count];
                    let mut face_margins = vec![0.0f32; face_count];
                    let face_verts: Vec<[u32; 3]> = (*t)
                        .settings
                        .faces
                        .iter()
                        .map(|f| [f.vertex.x, f.vertex.y, f.vertex.z])
                        .collect();
                    for f in 0..face_count {
                        let v = face_verts[f];
                        let ia = v[0] as usize;
                        let ib = v[1] as usize;
                        let ic = v[2] as usize;
                        let margin = max_float(
                            data[slot].vertex_margins[ia],
                            max_float(
                                data[slot].vertex_margins[ib],
                                data[slot].vertex_margins[ic],
                            ),
                        );
                        let sweep_a = data[slot].vertex_boxes[ia];
                        let sweep_b = data[slot].vertex_boxes[ib];
                        let sweep_c = data[slot].vertex_boxes[ic];
                        face_margins[f] = margin;
                        boxes[f] = aabb_inflate(
                            aabb_union(sweep_a, aabb_union(sweep_b, sweep_c)),
                            target_thickness[slot],
                        );
                    }
                    let rest: Vec<Vec3> =
                        (*t).settings.vertices.iter().map(|v| v.position).collect();
                    (*t).contact_face_tree.refit(&boxes, || {
                        face_verts
                            .iter()
                            .map(|v| {
                                let a = rest[v[0] as usize];
                                let b = rest[v[1] as usize];
                                let c = rest[v[2] as usize];
                                AABB::new(
                                    min_vec3(a, min_vec3(b, c)),
                                    max_vec3(a, max_vec3(b, c)),
                                )
                            })
                            .collect()
                    });
                    data[slot].face_tree = Some((*t).contact_face_tree.clone());
                    data[slot].face_boxes = boxes;
                    data[slot].face_margins = face_margins;
                    if (*t).closed_surface {
                        let origin = (*t).positions[0];
                        let mut six_volume: f32 = 0.0;
                        for v in face_verts.iter() {
                            six_volume += dot(
                                (*t).positions[v[0] as usize] - origin,
                                cross(
                                    (*t).positions[v[1] as usize] - origin,
                                    (*t).positions[v[2] as usize] - origin,
                                ),
                            );
                        }
                        data[slot].closed_side = if six_volume >= 0.0 { 1.0 } else { -1.0 };
                    }
                }
            }
        }

        // Edge grids of the edge pair targets.
        // SAFETY: as in the face phase; `refit` writes only the record's edge tree and the copy
        // out follows it.
        unsafe {
            for pair in edge_pairs.iter() {
                if data[pair.slot_t].edge_tree.is_some() {
                    continue;
                }
                let slot = pair.slot_t;
                let t = records.add(group[slot]);
                let mut thickness = 0.0f32;
                for q in edge_pairs.iter() {
                    if q.slot_t == slot {
                        thickness = max_float(thickness, q.thickness);
                    }
                }
                let edge_count = (*t).mesh_edges.len();
                let mut boxes = vec![AABB::default(); edge_count];
                let mut edge_margins = vec![0.0f32; edge_count];
                let medges: Vec<[u32; 2]> = (*t).mesh_edges.iter().map(|e| [e.x, e.y]).collect();
                for e in 0..edge_count {
                    let v = medges[e];
                    let ix = v[0] as usize;
                    let iy = v[1] as usize;
                    let margin =
                        max_float(data[slot].vertex_margins[ix], data[slot].vertex_margins[iy]);
                    let box_x = data[slot].vertex_boxes[ix];
                    let box_y = data[slot].vertex_boxes[iy];
                    edge_margins[e] = margin;
                    boxes[e] = aabb_inflate(aabb_union(box_x, box_y), thickness);
                }
                let rest: Vec<Vec3> =
                    (*t).settings.vertices.iter().map(|v| v.position).collect();
                (*t).contact_edge_tree.refit(&boxes, || {
                    medges
                        .iter()
                        .map(|v| {
                            let a = rest[v[0] as usize];
                            let b = rest[v[1] as usize];
                            AABB::new(min_vec3(a, b), max_vec3(a, b))
                        })
                        .collect()
                });
                data[slot].edge_tree = Some((*t).contact_edge_tree.clone());
                data[slot].edge_boxes = boxes;
                data[slot].edge_margins = edge_margins;
            }
        }

        // Each pair fills its own list; the lists are joined in pair order, so the result does not
        // depend on the threads.
        let shared = data;
        let shared_pairs = pairs;
        let shared_edge_pairs = edge_pairs;
        let mut lists: Vec<Vec<SoftContact>> = vec![Vec::new(); shared_pairs.len()];
        let detect_buffers = DetectBuffers {
            records,
            group,
            shared: &shared,
            pairs: &shared_pairs,
            lists: lists.as_mut_ptr(),
        };
        if parallel && shared_pairs.len() > 1 {
            thread::scope(|scope| {
                for k in 0..lists.len() {
                    scope.spawn(move || {
                        // SAFETY: `lists` addresses the live `lists` storage for the whole scope
                        // (the `Send`/`Sync` contract of `DetectBuffers`) and each task owns the
                        // distinct element `k`.
                        let list = unsafe { &mut *detect_buffers.lists.add(k) };
                        detect_buffers.detect(k, list, None);
                    });
                }
            });
        } else if shared_pairs.len() == 1 && detection_workers > 1 {
            // SAFETY: read-only snapshot taken before the chunked pass.
            let count =
                unsafe { (*records.add(group[shared_pairs[0].slot_p])).surface_vertices.len() };
            let chunks = std::cmp::min(detection_workers, std::cmp::max(1, count / 256));
            let mut parts: Vec<Vec<SoftContact>> = vec![Vec::new(); chunks];
            let part_buffers = DetectBuffers {
                records,
                group,
                shared: &shared,
                pairs: &shared_pairs,
                lists: parts.as_mut_ptr(),
            };
            thread::scope(|scope| {
                for k in 0..chunks {
                    scope.spawn(move || {
                        // SAFETY: each task owns the distinct element `k` of `parts` for the whole
                        // scope (the `Send`/`Sync` contract of `DetectBuffers`).
                        let list = unsafe { &mut *part_buffers.lists.add(k) };
                        part_buffers.detect(
                            0,
                            list,
                            Some(count * k / chunks..count * (k + 1) / chunks),
                        );
                    });
                }
            });
            lists[0] = parts.into_iter().flatten().collect();
        } else {
            for k in 0..shared_pairs.len() {
                detect_buffers.detect(k, &mut lists[k], None);
            }
        }
        for list in lists {
            contacts.extend(list);
        }

        let mut edge_lists: Vec<Vec<SoftEdgeContact>> = vec![Vec::new(); shared_edge_pairs.len()];
        let edge_buffers = EdgeDetectBuffers {
            records,
            group,
            shared: &shared,
            pairs: &shared_edge_pairs,
            lists: edge_lists.as_mut_ptr(),
        };
        if parallel && shared_edge_pairs.len() > 1 {
            thread::scope(|scope| {
                for k in 0..edge_lists.len() {
                    scope.spawn(move || {
                        // SAFETY: each task owns the distinct element `k` of `edge_lists` for the
                        // whole scope (the `Send`/`Sync` contract of `EdgeDetectBuffers`).
                        let list = unsafe { &mut *edge_buffers.lists.add(k) };
                        edge_buffers.detect(k, list, None);
                    });
                }
            });
        } else if shared_edge_pairs.len() == 1 && detection_workers > 1 {
            // SAFETY: read-only snapshot taken before the chunked pass.
            let count =
                unsafe { (*records.add(group[shared_edge_pairs[0].slot_p])).mesh_edges.len() };
            let chunks = std::cmp::min(detection_workers, std::cmp::max(1, count / 256));
            let mut parts: Vec<Vec<SoftEdgeContact>> = vec![Vec::new(); chunks];
            let part_buffers = EdgeDetectBuffers {
                records,
                group,
                shared: &shared,
                pairs: &shared_edge_pairs,
                lists: parts.as_mut_ptr(),
            };
            thread::scope(|scope| {
                for k in 0..chunks {
                    scope.spawn(move || {
                        // SAFETY: each task owns the distinct element `k` of `parts` for the whole
                        // scope (the `Send`/`Sync` contract of `EdgeDetectBuffers`).
                        let list = unsafe { &mut *part_buffers.lists.add(k) };
                        part_buffers.detect(
                            0,
                            list,
                            Some(count * k / chunks..count * (k + 1) / chunks),
                        );
                    });
                }
            });
            edge_lists[0] = parts.into_iter().flatten().collect();
        } else {
            for k in 0..shared_edge_pairs.len() {
                edge_buffers.detect(k, &mut edge_lists[k], None);
            }
        }
        for list in edge_lists {
            edge_contacts.extend(list);
        }

        // Remember the sides for the next step.
        // SAFETY: every detection borrow has ended; these are the step's only record writes.
        unsafe {
            for index in group {
                (*records.add(*index)).face_sides.clear();
            }
            for contact in contacts.iter() {
                if contact.side == 0.0 {
                    continue;
                }
                let p = records.add(group[contact.vertex_body as usize]);
                let t = records.add(group[contact.triangle_body as usize]);
                if (*t).closed_surface && !(*t).settings.faces.is_empty() {
                    continue;
                }
                let key = soft_contact_key((*t).id, contact.vertex, contact.face);
                (*p).face_sides.insert(key, contact.side);
            }
        }
    }
}
unsafe fn find_vertex_triangle_contacts(
    p: *mut SoftBodyRecord,
    t: *mut SoftBodyRecord,
    slot_p: i32,
    slot_t: i32,
    thickness: f32,
    friction: f32,
    self_pair: bool,
    p_data: &DetectionBody,
    t_data: &DetectionBody,
    contacts: &mut Vec<SoftContact>,
    source_range: Option<Range<usize>>,
) {
    let Some(tree) = &t_data.face_tree else { return };
    // SAFETY: detection only reads the records (their trees and exclusion rows were finalized
    // before this pass, and the `faceSides` history is written after it joins); the two pointers
    // address live records for the whole call and may alias each other for a self pair, which is
    // sound because both borrows are shared.
    unsafe {
        let p_rec = &*p;
        let t_rec = &*t;
        let faces = &t_rec.settings.faces;
        let tx = &t_rec.positions;
        let tw = &t_rec.inv_masses;
        let px = &p_rec.positions;
        let pw = &p_rec.inv_masses;
        let excluded_starts = &p_rec.excluded_face_starts;
        let excluded_faces = &p_rec.excluded_faces;
        let rest = &p_rec.settings.vertices;
        let exclusion = max_float(thickness, 2.0 * p_rec.average_edge_length);
        let face_margins = &t_data.face_margins;
        let vertex_margins = &p_data.vertex_margins;
        // Swift's `faceMargins.max() ?? 0`; the fold in array order agrees with `max()` for
        // finite values.
        let mut margin_max: f32 = 0.0;
        if !face_margins.is_empty() {
            margin_max = face_margins[0];
            for &m in face_margins.iter().skip(1) {
                margin_max = max_float(margin_max, m);
            }
        }
        let target_bounds = aabb_inflate(t_rec.bounds, thickness + margin_max);

        // Faces already tested for the current vertex.
        let mut stamp: Vec<i32> = vec![-1; faces.len()];
        // The closest few triangles of the current vertex: (distance squared, face, side).
        let mut best: Vec<(f32, i32, f32)> = Vec::with_capacity(MAX_TRIANGLES_PER_VERTEX + 1);

        let vertices = &p_rec.surface_vertices;
        let range = source_range.unwrap_or(0..vertices.len());
        for source in range {
            let vertex32 = vertices[source];
            let vertex = vertex32 as usize;
            let x = px[vertex];
            let margin = vertex_margins[vertex];
            let query = p_data.vertex_boxes[vertex];
            if !aabb_overlaps(query, target_bounds) {
                continue;
            }
            best.clear();
            // Pre-stamp the rest neighborhood once instead of binary-searching its exclusion row
            // for each spatial candidate. The same stamp also deduplicates features across cells.
            if self_pair {
                for offset in (excluded_starts[vertex] as usize)
                    ..(excluded_starts[vertex + 1] as usize)
                {
                    stamp[excluded_faces[offset] as usize] = vertex32;
                }
            }
            let reference_exclusion = if self_pair {
                Some((rest[vertex].position, exclusion * exclusion))
            } else {
                None
            };
            tree.visit(query, -1, reference_exclusion, |f| {
                if stamp[f as usize] == vertex32 {
                    return;
                }
                let face = &faces[f as usize];
                if face.is_degenerate() {
                    return;
                }
                let ia = face.vertex.x as i32;
                let ib = face.vertex.y as i32;
                let ic = face.vertex.z as i32;
                // Static against static never moves.
                if pw[vertex] == 0.0
                    && tw[ia as usize] == 0.0
                    && tw[ib as usize] == 0.0
                    && tw[ic as usize] == 0.0
                {
                    return;
                }
                let a = tx[ia as usize];
                let b = tx[ib as usize];
                let c = tx[ic as usize];
                let (bu, bv, bw, _interior) = closest_triangle_barycentric(a, b, c, x);
                let d = x - (bu * a + bv * b + bw * c);
                let distance_squared = length_squared(d);
                let reach = thickness + margin + face_margins[f as usize];
                if !(distance_squared < reach * reach) {
                    return;
                }
                if best.len() == MAX_TRIANGLES_PER_VERTEX {
                    let worst = best[MAX_TRIANGLES_PER_VERTEX - 1];
                    if !(distance_squared < worst.0
                        || (distance_squared == worst.0 && f < worst.1))
                    {
                        return;
                    }
                }
                // The outside of a closed surface is always the valid side. On an open surface the
                // side is the one the vertex is on now; a vertex can legitimately pass round a
                // free border to the other side. Only when it rests too close to tell does a pair
                // in contact last step keep its side, since a pair squeezed slightly inside the
                // thickness can read the wrong one and be pushed through.
                let mut side = t_data.closed_side;
                if side == 0.0 {
                    let normal = cross(b - a, c - a);
                    let gap = dot(d, normal) / max_float(length(normal), 1.0e-12);
                    side = if gap >= 0.0 { 1.0 } else { -1.0 };
                    if abs_float(gap) < AMBIGUOUS_GAP * thickness {
                        if let Some(remembered) =
                            p_rec.face_sides.get(&soft_contact_key(t_rec.id, vertex32, f))
                        {
                            side = *remembered;
                        }
                    }
                }
                // Insert sorted by distance, then face index.
                let mut k = best.len();
                best.push((distance_squared, f, side));
                while k > 0
                    && (best[k - 1].0 > distance_squared
                        || (best[k - 1].0 == distance_squared && best[k - 1].1 > f))
                {
                    best.swap(k, k - 1);
                    k -= 1;
                }
                if best.len() > MAX_TRIANGLES_PER_VERTEX {
                    best.pop();
                }
            });
            for &(_distance, f, side) in best.iter() {
                let face = &faces[f as usize];
                let mut contact = SoftContact::default();
                contact.vertex_body = slot_p;
                contact.vertex = vertex32;
                contact.triangle_body = slot_t;
                contact.a = face.vertex.x as i32;
                contact.b = face.vertex.y as i32;
                contact.c = face.vertex.z as i32;
                contact.side = side;
                contact.face = f;
                contact.thickness = thickness;
                contact.friction = friction;
                contacts.push(contact);
            }
        }
    }
}

unsafe fn find_vertex_vertex_contacts(
    p: *mut SoftBodyRecord,
    t: *mut SoftBodyRecord,
    slot_p: i32,
    slot_t: i32,
    thickness: f32,
    friction: f32,
    self_pair: bool,
    p_data: &DetectionBody,
    t_data: &DetectionBody,
    contacts: &mut Vec<SoftContact>,
    source_range: Option<Range<usize>>,
) {
    let Some(grid) = &t_data.grid else { return };
    // SAFETY: as in `find_vertex_triangle_contacts` - shared borrows of live, read-only records.
    unsafe {
        let p_rec = &*p;
        let t_rec = &*t;
        let px = &p_rec.positions;
        let pw = &p_rec.inv_masses;
        let tx = &t_rec.positions;
        let tw = &t_rec.inv_masses;
        let rest = &p_rec.settings.vertices;
        let rest_exclusion = max_float(thickness, 2.0 * p_rec.average_edge_length);
        let mut stamp: Vec<i32> = vec![-1; tx.len()];
        let range = source_range.unwrap_or(0..px.len());
        for i in range {
            let x = px[i];
            let margin = p_data.vertex_margins[i];
            let query = p_data.vertex_boxes[i];
            grid.visit_box(query, |j| {
                if stamp[j as usize] == i as i32 {
                    return;
                }
                stamp[j as usize] = i as i32;
                if self_pair {
                    if j <= i as i32 || p_rec.are_neighbors(i as i32, j) {
                        return;
                    }
                    if length_squared(rest[i].position - rest[j as usize].position)
                        < rest_exclusion * rest_exclusion
                    {
                        return;
                    }
                }
                if pw[i] == 0.0 && tw[j as usize] == 0.0 {
                    return;
                }
                let reach = thickness + margin + t_data.vertex_margins[j as usize];
                if !(length_squared(x - tx[j as usize]) < reach * reach) {
                    return;
                }
                let mut contact = SoftContact::default();
                contact.vertex_body = slot_p;
                contact.vertex = i as i32;
                contact.triangle_body = slot_t;
                contact.a = j;
                contact.b = j;
                contact.c = j;
                contact.side = 0.0;
                contact.thickness = thickness;
                contact.friction = friction;
                contacts.push(contact);
            });
        }
    }
}

/// One Gauss-Seidel pass over the contacts. A contact stops approach motion at the thickness, but
/// a vertex that starts the substep inside the thickness is pushed out by at most `max_recovery`:
/// projecting it out at once would turn the push into velocity and pump energy into piles.
pub fn solve_soft_contacts(
    contacts: &[SoftContact],
    bodies: &[SoftContactBody],
    max_recovery: f32,
) {
    // SAFETY: every `SoftContactBody` was built from a record's live `positions`,
    // `previous_positions` and `inv_masses` buffers (Swift's `withUnsafe*BufferPointer` fields)
    // that do not reallocate during the step, and contact indices are inside those buffers
    // because the contacts were generated from the same records.
    unsafe {
        let target = |thickness: f32, start_separation: f32| -> f32 {
            min_float(thickness, start_separation + max_recovery)
        };

        for contact in contacts {
            let pb = bodies[contact.vertex_body as usize];
            let tb = bodies[contact.triangle_body as usize];
            let i = contact.vertex as usize;
            let wp = *pb.w.add(i);

            if contact.side == 0.0 {
                // Vertex-vertex.
                let j = contact.a as usize;
                let wq = *tb.w.add(j);
                let total = wp + wq;
                if !(total > 0.0) {
                    continue;
                }
                let d = *pb.x.add(i) - *tb.x.add(j);
                let len = length(d);
                let c = len - target(contact.thickness, length(*pb.x0.add(i) - *tb.x0.add(j)));
                if !(c < 0.0 && len > 1.0e-9) {
                    continue;
                }
                let n = (1.0 / len) * d;
                let mut correction = (-c / total) * n;
                // Friction against the relative tangential motion of this substep.
                let motion = (*pb.x.add(i) - *pb.x0.add(i)) - (*tb.x.add(j) - *tb.x0.add(j));
                let tangent = motion - dot(motion, n) * n;
                let tangent_length = length(tangent);
                if tangent_length > 1.0e-9 {
                    correction = correction
                        - (min_float(contact.friction * (-c) / tangent_length, 1.0) / total)
                            * tangent;
                }
                *pb.x.add(i) = mul_add(*pb.x.add(i), wp, correction);
                *tb.x.add(j) = mul_sub(*tb.x.add(j), wq, correction);
                continue;
            }

            let ia = contact.a as usize;
            let ib = contact.b as usize;
            let ic = contact.c as usize;
            let a = *tb.x.add(ia);
            let b = *tb.x.add(ib);
            let c = *tb.x.add(ic);
            let x = *pb.x.add(i);
            let (bu, bv, bw, interior) = closest_triangle_barycentric(a, b, c, x);
            let q = bu * a + bv * b + bw * c;
            let mut n: Vec3;
            let mut separation: f32;
            if interior {
                let (len, normalized) = get_length_and_normalize(cross(b - a, c - a));
                if !(len > 1.0e-12) {
                    continue;
                }
                n = contact.side * normalized;
                separation = dot(x - q, n);
            } else {
                // Beside the face: keep the distance to the closest edge or corner.
                let d = x - q;
                separation = length(d);
                if !(separation > 1.0e-9) {
                    continue;
                }
                n = (1.0 / separation) * d;
            }
            let start =
                *pb.x0.add(i) - (bu * *tb.x0.add(ia) + bv * *tb.x0.add(ib) + bw * *tb.x0.add(ic));
            let violation = separation - target(contact.thickness, dot(start, n));
            if !(violation < 0.0) {
                continue;
            }

            let wa = *tb.w.add(ia) * bu;
            let wb = *tb.w.add(ib) * bv;
            let wc = *tb.w.add(ic) * bw;
            let total = wp + wa * bu + wb * bv + wc * bw;
            if !(total > 0.0) {
                continue;
            }
            let mut correction = (-violation / total) * n;

            let motion = (x - *pb.x0.add(i))
                - (bu * (a - *tb.x0.add(ia))
                    + bv * (b - *tb.x0.add(ib))
                    + bw * (c - *tb.x0.add(ic)));
            let tangent = motion - dot(motion, n) * n;
            let tangent_length = length(tangent);
            if tangent_length > 1.0e-9 {
                correction = correction
                    - (min_float(contact.friction * (-violation) / tangent_length, 1.0) / total)
                        * tangent;
            }

            *pb.x.add(i) = mul_add(x, wp, correction);
            *tb.x.add(ia) = mul_sub(*tb.x.add(ia), wa, correction);
            *tb.x.add(ib) = mul_sub(*tb.x.add(ib), wb, correction);
            *tb.x.add(ic) = mul_sub(*tb.x.add(ic), wc, correction);
        }
    }
}

/// Two edges kept apart: the closest points of the segments stay `thickness` apart on the side
/// they started on, along the normal of the plane both edges span.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SoftEdgeContact {
    pub body_p: i32,
    pub p0: i32,
    pub p1: i32,
    pub body_t: i32,
    pub t0: i32,
    pub t1: i32,
    pub side: f32,
    pub thickness: f32,
    pub friction: f32,
    /// Both bodies have faces: when the closest points reach an edge end the vertex-face contacts
    /// own the configuration and this contact rests.
    pub interior_only: bool,
}

// Swift's member defaults; `side` starts at +1, unlike the zero-initialized fields.
impl Default for SoftEdgeContact {
    fn default() -> Self {
        Self {
            body_p: 0,
            p0: 0,
            p1: 0,
            body_t: 0,
            t0: 0,
            t1: 0,
            side: 1.0,
            thickness: 0.0,
            friction: 0.0,
            interior_only: false,
        }
    }
}

/// Closer than this fraction of the thickness along its normal, a pair's side cannot be read.
const AMBIGUOUS_GAP: f32 = 0.5;

/// Key of a pair in a body's contact history: the other body's id and both feature indices.
#[inline(always)]
pub(crate) fn soft_contact_key(other_body: i32, feature: i32, other_feature: i32) -> u64 {
    ((other_body as u16 as u64) << 48)
        | (((feature as u32 & 0xff_ffff) as u64) << 24)
        | (other_feature as u32 & 0xff_ffff) as u64
}

/// Most edges kept per edge and body.
const MAX_EDGES_PER_EDGE: usize = 4;

/// Edges this close to parallel have no stable common normal; vertex contacts cover them.
const PARALLEL_SINE: f32 = 0.05;
unsafe fn find_edge_edge_contacts(
    p: *mut SoftBodyRecord,
    t: *mut SoftBodyRecord,
    slot_p: i32,
    slot_t: i32,
    thickness: f32,
    friction: f32,
    self_pair: bool,
    p_data: &DetectionBody,
    t_data: &DetectionBody,
    contacts: &mut Vec<SoftEdgeContact>,
    source_range: Option<Range<usize>>,
) {
    let Some(tree) = &t_data.edge_tree else { return };
    // SAFETY: as in `find_vertex_triangle_contacts` - shared borrows of live, read-only records;
    // the two pointers may alias for a self pair.
    unsafe {
        let p_rec = &*p;
        let t_rec = &*t;
        let p_edges = &p_rec.mesh_edges;
        let t_edges = &t_rec.mesh_edges;
        let px = &p_rec.positions;
        let tx = &t_rec.positions;
        let pw = &p_rec.inv_masses;
        let tw = &t_rec.inv_masses;
        let excluded_starts = &p_rec.excluded_edge_starts;
        let excluded_edges = &p_rec.excluded_edges;
        let rest = &p_rec.settings.vertices;
        let exclusion = max_float(thickness, 2.0 * p_rec.average_edge_length);
        // Both with faces: an edge's end points are covered by the vertex-face contacts.
        let interior_only =
            !p_rec.settings.faces.is_empty() && !t_rec.settings.faces.is_empty();
        let edge_margins = &t_data.edge_margins;
        let mut stamp: Vec<i32> = vec![-1; t_edges.len()];
        let limit = MAX_EDGES_PER_EDGE;
        let mut best: Vec<(f32, i32, f32)> = Vec::with_capacity(limit + 1);

        let range = source_range.unwrap_or(0..p_edges.len());
        for e in range {
            let edge = p_edges[e];
            let i0 = edge.x as usize;
            let i1 = edge.y as usize;
            let a0 = px[i0];
            let a1 = px[i1];
            let axis_a = a1 - a0;
            let axis_length_squared_a = length_squared(axis_a);
            let margin = max_float(p_data.vertex_margins[i0], p_data.vertex_margins[i1]);
            let query = aabb_union(p_data.vertex_boxes[i0], p_data.vertex_boxes[i1]);
            best.clear();
            if self_pair {
                for offset in (excluded_starts[e] as usize)..(excluded_starts[e + 1] as usize) {
                    let other = excluded_edges[offset];
                    if (other as usize) > e {
                        stamp[other as usize] = e as i32;
                    }
                }
            }
            let reference_exclusion = if self_pair {
                Some((rest[i0].position, exclusion * exclusion))
            } else {
                None
            };
            tree.visit(
                query,
                if self_pair { e as i32 } else { -1 },
                reference_exclusion,
                |f| {
                    if stamp[f as usize] == e as i32 {
                        return;
                    }
                    let other = t_edges[f as usize];
                    let j0 = other.x as usize;
                    let j1 = other.y as usize;
                    if pw[i0] == 0.0
                        && pw[i1] == 0.0
                        && tw[j0] == 0.0
                        && tw[j1] == 0.0
                    {
                        return;
                    }
                    let b0 = tx[j0];
                    let b1 = tx[j1];
                    // Parallel, coplanar and out-of-reach pairs need no closest-segment solve. Use
                    // the unnormalized common normal so these rejections also avoid
                    // normalization/sqrt.
                    let axis_b = b1 - b0;
                    let normal = cross(axis_a, axis_b);
                    let normal_squared = length_squared(normal);
                    if !(normal_squared
                        > PARALLEL_SINE
                            * PARALLEL_SINE
                            * max_float(
                                axis_length_squared_a * length_squared(axis_b),
                                1.0e-18,
                            ))
                    {
                        return;
                    }
                    let signed_gap = dot(a0 - b0, normal);
                    let gap_squared = signed_gap * signed_gap;
                    let reach = thickness + margin + edge_margins[f as usize];
                    let unreadable_gap = AMBIGUOUS_GAP * thickness;
                    if !(gap_squared < reach * reach * normal_squared
                        && gap_squared
                            >= unreadable_gap * unreadable_gap * normal_squared)
                    {
                        return;
                    }
                    let result = segment_distance(a0, a1, b0, b1);
                    if interior_only {
                        let s = result.fraction1;
                        let u = result.fraction2;
                        if s <= 0.02 || s >= 0.98 || u <= 0.02 || u >= 0.98 {
                            return;
                        }
                    }
                    let d = result.point1 - result.point2;
                    let distance_squared = length_squared(d);
                    if !(distance_squared < reach * reach) {
                        return;
                    }
                    if best.len() == limit {
                        let worst = best[limit - 1];
                        if !(distance_squared < worst.0
                            || (distance_squared == worst.0 && f < worst.1))
                        {
                            return;
                        }
                    }
                    // Edges closer than half the thickness along their common normal have no
                    // readable side, and a wrong side pushes them through each other: leave such
                    // pairs to the vertex-face contacts this step. (A remembered side does not
                    // help: the common normal flips as edges turn past parallel, which folds do
                    // all the time.)
                    let side: f32 = if signed_gap >= 0.0 { 1.0 } else { -1.0 };
                    let mut k = best.len();
                    best.push((distance_squared, f, side));
                    while k > 0
                        && (best[k - 1].0 > distance_squared
                            || (best[k - 1].0 == distance_squared && best[k - 1].1 > f))
                    {
                        best.swap(k, k - 1);
                        k -= 1;
                    }
                    if best.len() > limit {
                        best.pop();
                    }
                },
            );
            for &(_distance, f, side) in best.iter() {
                let other = &t_edges[f as usize];
                contacts.push(SoftEdgeContact {
                    body_p: slot_p,
                    p0: i0 as i32,
                    p1: i1 as i32,
                    body_t: slot_t,
                    t0: other.x as i32,
                    t1: other.y as i32,
                    side,
                    thickness,
                    friction,
                    interior_only,
                });
            }
        }
    }
}

/// One Gauss-Seidel pass over the edge contacts, with the same recovery limit as the vertex
/// contacts.
pub fn solve_soft_edge_contacts(
    contacts: &[SoftEdgeContact],
    bodies: &[SoftContactBody],
    max_recovery: f32,
) {
    // SAFETY: as in `solve_soft_contacts` - the body pointers address the records' position and
    // inverse-mass buffers, which do not reallocate during the step, and contact indices come from
    // those same records.
    unsafe {
        for contact in contacts {
            let pb = bodies[contact.body_p as usize];
            let tb = bodies[contact.body_t as usize];
            let i0 = contact.p0 as usize;
            let i1 = contact.p1 as usize;
            let j0 = contact.t0 as usize;
            let j1 = contact.t1 as usize;
            let a0 = *pb.x.add(i0);
            let a1 = *pb.x.add(i1);
            let b0 = *tb.x.add(j0);
            let b1 = *tb.x.add(j1);
            let result = segment_distance(a0, a1, b0, b1);
            let s = result.fraction1;
            let u = result.fraction2;
            let d = result.point1 - result.point2;

            // Crossing configuration: both closest points inside their edges and the edges not
            // near parallel. Then the side is the one remembered along the common normal.
            // Otherwise the common normal says nothing about the gap: fall back to the plain
            // distance, or leave the pair to the vertex-face contacts.
            let crossing = cross(a1 - a0, b1 - b0);
            let sine2 = length_squared(crossing)
                / max_float(
                    length_squared(a1 - a0) * length_squared(b1 - b0),
                    1.0e-18,
                );
            let interior = s > 0.0
                && s < 1.0
                && u > 0.0
                && u < 1.0
                && sine2 > PARALLEL_SINE * PARALLEL_SINE;
            let mut n: Vec3;
            let mut separation: f32;
            if interior {
                n = contact.side * normalize(crossing);
                separation = dot(d, n);
            } else {
                if contact.interior_only {
                    continue;
                }
                separation = length(d);
                if !(separation > 1.0e-9) {
                    continue;
                }
                n = (1.0 / separation) * d;
            }

            // Where the same points were at the start of the substep.
            let start = ((1.0 - s) * *pb.x0.add(i0) + s * *pb.x0.add(i1))
                - ((1.0 - u) * *tb.x0.add(j0) + u * *tb.x0.add(j1));
            let target = min_float(contact.thickness, dot(start, n) + max_recovery);
            let violation = separation - target;
            if !(violation < 0.0) {
                continue;
            }

            let wa0 = *pb.w.add(i0) * (1.0 - s);
            let wa1 = *pb.w.add(i1) * s;
            let wb0 = *tb.w.add(j0) * (1.0 - u);
            let wb1 = *tb.w.add(j1) * u;
            let total = wa0 * (1.0 - s) + wa1 * s + wb0 * (1.0 - u) + wb1 * u;
            if !(total > 0.0) {
                continue;
            }
            let mut correction = (-violation / total) * n;

            let motion = (result.point1 - result.point2) - start;
            let tangent = motion - dot(motion, n) * n;
            let tangent_length = length(tangent);
            if tangent_length > 1.0e-9 {
                correction = correction
                    - (min_float(
                        contact.friction * (-violation) / tangent_length,
                        1.0,
                    ) / total) * tangent;
            }

            *pb.x.add(i0) = mul_add(*pb.x.add(i0), wa0, correction);
            *pb.x.add(i1) = mul_add(*pb.x.add(i1), wa1, correction);
            *tb.x.add(j0) = mul_sub(*tb.x.add(j0), wb0, correction);
            *tb.x.add(j1) = mul_sub(*tb.x.add(j1), wb1, correction);
        }
    }
}
