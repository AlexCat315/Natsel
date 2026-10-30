//! Shared soft body settings: the rest state of the particles and the constraints between them.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodySettings.swift`.
//!
//! A soft body is a set of particles connected by XPBD constraints: edges (stretch and shear),
//! dihedral or distance bending, tetrahedral volume, long range attachments, skinning, and an
//! optional internal pressure for closed surfaces. Settings are a value type whose arrays are shared
//! by every instance created from them.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"): this file only defines the value types and their generation /
//! rest-value / optimization / validation / factory logic; `crate::soft_body::soft_body_runtime::
//! SoftBodyRecord` (owned by another pass) stores `settings: SoftBodySettings` and reads
//! `settings.faces` / `settings.edges` / `settings.vertices` / `settings.skinned` /
//! `settings.patches` / `settings.inverse_bind_matrices`. The `vertex` tuples below keep Swift's
//! `SIMD2/3/4<UInt32>` `.x` / `.y` / `.z` / `.w` member access exactly as `soft_body_ray_cache.rs`,
//! `soft_body_queries.rs` and `world_debug_draw_extras.rs` already expect, and
//! `settings.edges.map { $0.vertex }` (`soft_body_edge_collision.rs`) yields a `Vec<SoftBodyEdgeU32x2>`.
//!
//! Swift defaulted several public arguments (`createConstraints`'s `bendType`/`angleTolerance`, the
//! factory `attributes`/`bendType`/`pinned`/`spacing`-adjacent defaults, and the value initializers).
//! Following the `world_debug_draw_extras.rs` precedent for defaulted Swift arguments, those defaults
//! become explicit parameters here; the literal default values are kept (`DEFAULT_ANGLE_TOLERANCE`,
//! the `Default` impls reproduce every Swift property default) so ported callers pass the same values
//! the Swift defaults produced.

use std::collections::{HashMap, HashSet};
use std::ops::Range;

use crate::math::functions::{
    atan2_approx, compute_cos_sin, cross, distance, distance_squared, dot, is_valid_vec3, length,
    max_vec3, min_vec3, normalize,
};
use crate::math::types::{Transform, Vec3};

/// Swift `Float(angleTolerance)` default of `createConstraints`.
const DEFAULT_ANGLE_TOLERANCE: f32 = 0.13962634;

// MARK: - SIMD value tuples

/// A two lane `UInt32` tuple: Swift's `SIMD2<UInt32>`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct SoftBodyU32x2 {
    pub x: u32,
    pub y: u32,
}

impl SoftBodyU32x2 {
    #[inline]
    pub const fn new(x: u32, y: u32) -> Self {
        Self { x, y }
    }
}

/// Sibling modules (`soft_body_edge_collision`, `soft_body_collision`) name this tuple `U32x2`;
/// keep both spellings pointing at the one type.
pub type U32x2 = SoftBodyU32x2;

/// A three lane `UInt32` tuple: Swift's `SIMD3<UInt32>`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct SoftBodyU32x3 {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

impl SoftBodyU32x3 {
    #[inline]
    pub const fn new(x: u32, y: u32, z: u32) -> Self {
        Self { x, y, z }
    }
}

/// A four lane `UInt32` tuple: Swift's `SIMD4<UInt32>`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct SoftBodyU32x4 {
    pub x: u32,
    pub y: u32,
    pub z: u32,
    pub w: u32,
}

impl SoftBodyU32x4 {
    #[inline]
    pub const fn new(x: u32, y: u32, z: u32, w: u32) -> Self {
        Self { x, y, z, w }
    }
}

impl core::ops::Index<usize> for SoftBodyU32x4 {
    type Output = u32;
    #[inline]
    fn index(&self, lane: usize) -> &u32 {
        match lane {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => &self.w,
        }
    }
}

/// A four lane `Float` tuple: Swift's `SIMD4<Float>`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SoftBodyF32x4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl SoftBodyF32x4 {
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }
}

impl Default for SoftBodyF32x4 {
    #[inline]
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0)
    }
}

impl core::ops::Index<usize> for SoftBodyF32x4 {
    type Output = f32;
    #[inline]
    fn index(&self, lane: usize) -> &f32 {
        match lane {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => &self.w,
        }
    }
}

// MARK: - Constraint primitives

/// A particle in the rest configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SoftBodyVertex {
    pub position: Vec3,
    pub velocity: Vec3,
    /// Zero makes the vertex kinematic: it is moved only by the user or by skinning.
    pub inv_mass: f32,
}

impl Default for SoftBodyVertex {
    #[inline]
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            inv_mass: 1.0,
        }
    }
}

impl SoftBodyVertex {
    #[inline]
    pub fn new(position: Vec3, velocity: Vec3, inv_mass: f32) -> Self {
        Self {
            position,
            velocity,
            inv_mass,
        }
    }
}

/// A triangle of the soft body surface, counter clockwise when seen from outside.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct SoftBodyFace {
    pub vertex: SoftBodyU32x3,
}

impl SoftBodyFace {
    #[inline]
    pub fn new(v0: u32, v1: u32, v2: u32) -> Self {
        Self {
            vertex: SoftBodyU32x3::new(v0, v1, v2),
        }
    }

    #[inline]
    pub fn is_degenerate(&self) -> bool {
        self.vertex.x == self.vertex.y || self.vertex.y == self.vertex.z || self.vertex.x == self.vertex.z
    }
}

/// Keeps two vertices at a rest length.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SoftBodyEdge {
    pub vertex: SoftBodyU32x2,
    pub rest_length: f32,
    /// Inverse stiffness (m/N). Zero is rigid.
    pub compliance: f32,
}

impl Default for SoftBodyEdge {
    #[inline]
    fn default() -> Self {
        Self {
            vertex: SoftBodyU32x2::new(0, 0),
            rest_length: 1.0,
            compliance: 0.0,
        }
    }
}

impl SoftBodyEdge {
    #[inline]
    pub fn new(v0: u32, v1: u32, compliance: f32) -> Self {
        Self {
            vertex: SoftBodyU32x2::new(v0, v1),
            rest_length: 1.0,
            compliance,
        }
    }
}

/// Keeps the angle between two triangles sharing the edge (v0, v1). The triangles are
/// (v0, v1, v2) and (v1, v0, v3).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SoftBodyDihedralBend {
    pub vertex: SoftBodyU32x4,
    pub compliance: f32,
    /// Rest angle in radians.
    pub initial_angle: f32,
}

impl SoftBodyDihedralBend {
    #[inline]
    pub fn new(v0: u32, v1: u32, v2: u32, v3: u32, compliance: f32) -> Self {
        Self {
            vertex: SoftBodyU32x4::new(v0, v1, v2, v3),
            compliance,
            initial_angle: 0.0,
        }
    }
}

/// Keeps the volume of a tetrahedron.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SoftBodyVolume {
    pub vertex: SoftBodyU32x4,
    /// Six times the signed rest volume.
    pub six_rest_volume: f32,
    pub compliance: f32,
}

impl Default for SoftBodyVolume {
    #[inline]
    fn default() -> Self {
        Self {
            vertex: SoftBodyU32x4::new(0, 0, 0, 0),
            six_rest_volume: 1.0,
            compliance: 0.0,
        }
    }
}

impl SoftBodyVolume {
    #[inline]
    pub fn new(v0: u32, v1: u32, v2: u32, v3: u32, compliance: f32) -> Self {
        Self {
            vertex: SoftBodyU32x4::new(v0, v1, v2, v3),
            six_rest_volume: 1.0,
            compliance,
        }
    }
}

/// Long range attachment: keeps a vertex within `maxDistance` of a kinematic anchor vertex. These
/// make long hanging cloth inextensible without extra iterations.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SoftBodyLRA {
    pub anchor: u32,
    pub vertex: u32,
    pub max_distance: f32,
}

impl SoftBodyLRA {
    #[inline]
    pub fn new(anchor: u32, vertex: u32, max_distance: f32) -> Self {
        Self {
            anchor,
            vertex,
            max_distance,
        }
    }
}

/// Binds a vertex to an animated skeleton. Up to four joint weights.
/// Not `Copy`: Swift's value type carries the `normalFaces` copy-on-write array, which becomes an
/// owned `Vec` here.
#[derive(Clone, Debug, PartialEq)]
#[repr(C)]
pub struct SoftBodySkinned {
    pub vertex: u32,
    pub joints: SoftBodyU32x4,
    pub weights: SoftBodyF32x4,
    /// How far the vertex may move from its skinned position. Zero pins it to the skin.
    pub max_distance: f32,
    /// A sphere behind the skinned surface the vertex may not enter, at
    /// `backStopDistance + backStopRadius` below the skinned position along the skinned normal.
    pub back_stop_distance: f32,
    pub back_stop_radius: f32,
    /// Faces around the vertex, used to compute the skinned normal.
    pub(crate) normal_faces: Vec<u32>,
}

impl Default for SoftBodySkinned {
    #[inline]
    fn default() -> Self {
        Self {
            vertex: 0,
            joints: SoftBodyU32x4::new(0, 0, 0, 0),
            weights: SoftBodyF32x4::new(1.0, 0.0, 0.0, 0.0),
            max_distance: 0.0,
            back_stop_distance: 0.0,
            back_stop_radius: 0.0,
            normal_faces: Vec::new(),
        }
    }
}

impl SoftBodySkinned {
    #[inline]
    pub fn new(
        vertex: u32,
        joints: SoftBodyU32x4,
        weights: SoftBodyF32x4,
        max_distance: f32,
        back_stop_distance: f32,
        back_stop_radius: f32,
    ) -> Self {
        Self {
            vertex,
            joints,
            weights,
            max_distance,
            back_stop_distance,
            back_stop_radius,
            normal_faces: Vec::new(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoftBodyLRAType {
    None,
    /// Straight line distance to the nearest kinematic vertex.
    Euclidean,
    /// Distance along the edges to the nearest kinematic vertex. Better for folded cloth.
    Geodesic,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoftBodyBendType {
    None,
    /// A distance constraint between the far vertices of adjacent triangles.
    Distance,
    /// An angle constraint between adjacent triangles. Allows creases to keep their shape.
    Dihedral,
}

/// Per vertex inputs for automatic constraint creation. The compliance of a constraint is the
/// largest compliance of its vertices.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVertexAttributes {
    /// Compliance of the triangle edges.
    pub compliance: f32,
    /// Compliance of quad diagonals. Infinity creates no shear constraints.
    pub shear_compliance: f32,
    /// Compliance of bending. Infinity creates no bend constraints.
    pub bend_compliance: f32,
    pub lra_type: SoftBodyLRAType,
    /// Scales the rest distance of the long range attachment, allowing a little stretch.
    pub lra_max_distance_multiplier: f32,
}

impl Default for SoftBodyVertexAttributes {
    #[inline]
    fn default() -> Self {
        Self {
            compliance: 0.0,
            shear_compliance: 0.0,
            bend_compliance: f32::INFINITY,
            lra_type: SoftBodyLRAType::None,
            lra_max_distance_multiplier: 1.0,
        }
    }
}

impl SoftBodyVertexAttributes {
    #[inline]
    pub fn new(
        compliance: f32,
        shear_compliance: f32,
        bend_compliance: f32,
        lra_type: SoftBodyLRAType,
        lra_max_distance_multiplier: f32,
    ) -> Self {
        Self {
            compliance,
            shear_compliance,
            bend_compliance,
            lra_type,
            lra_max_distance_multiplier,
        }
    }
}

/// Unordered edge key: Swift's `EdgeKey`, `a <= b`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct EdgeKey {
    a: u32,
    b: u32,
}

impl EdgeKey {
    #[inline]
    fn new(x: u32, y: u32) -> Self {
        Self {
            a: core::cmp::min(x, y),
            b: core::cmp::max(x, y),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct SoftBodySettings {
    pub vertices: Vec<SoftBodyVertex>,
    pub faces: Vec<SoftBodyFace>,
    pub edges: Vec<SoftBodyEdge>,
    pub dihedral_bends: Vec<SoftBodyDihedralBend>,
    pub volumes: Vec<SoftBodyVolume>,
    pub long_range_attachments: Vec<SoftBodyLRA>,
    pub skinned: Vec<SoftBodySkinned>,
    /// Inverse bind pose of each skeleton joint, used with `skinned`.
    pub inverse_bind_matrices: Vec<Transform>,

    /// Vertex index ranges grouped by spatial locality, used to batch collision detection.
    pub(crate) patches: Vec<Range<usize>>,
    /// Vertex permutation applied by `optimize`, old index -> new index.
    pub vertex_remap: Vec<u32>,
}

impl SoftBodySettings {
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    // MARK: Constraint generation

    /// Create edge, shear, bend, and long range attachment constraints from the faces.
    /// `attributes` has one entry per vertex, or a single entry used for all vertices.
    pub fn create_constraints(
        &mut self,
        attributes: &[SoftBodyVertexAttributes],
        bend_type: SoftBodyBendType,
        angle_tolerance: f32,
    ) {
        if attributes.is_empty() {
            return;
        }
        let attribute = |v: u32| -> SoftBodyVertexAttributes {
            if attributes.len() == 1 {
                attributes[0]
            } else {
                attributes[v as usize]
            }
        };

        // Faces around each edge.
        let mut edge_faces: HashMap<EdgeKey, Vec<usize>> = HashMap::new();
        let mut edge_order: Vec<EdgeKey> = Vec::new();
        for (f, face) in self.faces.iter().enumerate() {
            if face.is_degenerate() {
                continue;
            }
            let v = [face.vertex.x, face.vertex.y, face.vertex.z];
            for e in 0..3 {
                let key = EdgeKey::new(v[e], v[(e + 1) % 3]);
                if !edge_faces.contains_key(&key) {
                    edge_order.push(key);
                }
                edge_faces.entry(key).or_default().push(f);
            }
        }

        // Disjoint field borrows: Swift mutated `edges` and `dihedralBends` while reading `faces`
        // and `vertices`; the fields are distinct, so split the borrow and keep the same order.
        let SoftBodySettings {
            vertices,
            faces,
            edges,
            dihedral_bends,
            long_range_attachments,
            ..
        } = self;

        let opposite = |face: &SoftBodyFace, key: EdgeKey| -> u32 {
            for v in [face.vertex.x, face.vertex.y, face.vertex.z] {
                if v != key.a && v != key.b {
                    return v;
                }
            }
            face.vertex.x
        };

        let position = |v: u32| -> Vec3 { vertices[v as usize].position };

        let face_normal = |face: &SoftBodyFace| -> Vec3 {
            let a = position(face.vertex.x);
            let b = position(face.vertex.y);
            let c = position(face.vertex.z);
            normalize(cross(b - a, c - a))
        };

        let mut existing: HashSet<EdgeKey> = HashSet::new();
        for edge in edges.iter() {
            existing.insert(EdgeKey::new(edge.vertex.x, edge.vertex.y));
        }

        let mut add_edge = |a: u32, b: u32, compliance: f32| {
            if !compliance.is_finite() || a == b {
                return;
            }
            let key = EdgeKey::new(a, b);
            if !existing.insert(key) {
                return;
            }
            edges.push(SoftBodyEdge::new(a, b, compliance));
        };

        // A shared edge is a quad diagonal when it is the longest edge of both triangles and the
        // triangles are nearly coplanar.
        let cos_tolerance = compute_cos_sin(angle_tolerance).cosine;
        let mut diagonals: HashSet<EdgeKey> = HashSet::new();
        for key in edge_order.iter() {
            let key = *key;
            let Some(list) = edge_faces.get(&key) else {
                continue;
            };
            if list.len() != 2 {
                continue;
            }
            let f0 = faces[list[0]];
            let f1 = faces[list[1]];
            if !(dot(face_normal(&f0), face_normal(&f1)) >= cos_tolerance) {
                continue;
            }
            let length = distance_squared(position(key.a), position(key.b));
            let mut longest = true;
            for face in [f0, f1] {
                let o = opposite(&face, key);
                if distance_squared(position(o), position(key.a)) > length
                    || distance_squared(position(o), position(key.b)) > length
                {
                    longest = false;
                }
            }
            if longest {
                diagonals.insert(key);
            }
        }

        // Stretch edges, with quad diagonals treated as shear.
        for key in edge_order.iter() {
            let key = *key;
            let shear = diagonals.contains(&key);
            let a = attribute(key.a);
            let b = attribute(key.b);
            let compliance = if shear {
                fmax(a.shear_compliance, b.shear_compliance)
            } else {
                fmax(a.compliance, b.compliance)
            };
            add_edge(key.a, key.b, compliance);
        }

        // Shear across each quad, and bending across every other shared edge.
        for key in edge_order.iter() {
            let key = *key;
            let Some(list) = edge_faces.get(&key) else {
                continue;
            };
            if list.len() != 2 {
                continue;
            }
            let o0 = opposite(&faces[list[0]], key);
            let o1 = opposite(&faces[list[1]], key);
            if o0 == o1 {
                continue;
            }
            if diagonals.contains(&key) {
                let compliance = fmax(attribute(o0).shear_compliance, attribute(o1).shear_compliance);
                add_edge(o0, o1, compliance);
                continue;
            }

            let compliance = fmax(
                fmax(attribute(key.a).bend_compliance, attribute(key.b).bend_compliance),
                fmax(attribute(o0).bend_compliance, attribute(o1).bend_compliance),
            );
            if !compliance.is_finite() {
                continue;
            }
            match bend_type {
                SoftBodyBendType::None => {}
                SoftBodyBendType::Distance => {
                    add_edge(o0, o1, compliance);
                }
                SoftBodyBendType::Dihedral => {
                    // Orient the shared edge as it appears in the first face: (v0, v1, v2).
                    let f0 = faces[list[0]];
                    let v = [f0.vertex.x, f0.vertex.y, f0.vertex.z];
                    let mut e0 = key.a;
                    let mut e1 = key.b;
                    for i in 0..3 {
                        if v[i] == key.b && v[(i + 1) % 3] == key.a {
                            e0 = key.b;
                            e1 = key.a;
                        }
                    }
                    dihedral_bends.push(SoftBodyDihedralBend::new(e0, e1, o0, o1, compliance));
                }
            }
        }

        // The `add_edge` closure's mutable borrow of `edges` ends here; from this point the
        // remaining passes only read `edges`/`vertices`, exactly as Swift's snapshot did.
        drop(add_edge);

        // Long range attachments to the nearest kinematic vertex.
        let euclidean: Vec<usize> = (0..vertices.len())
            .filter(|&v| attribute(v as u32).lra_type == SoftBodyLRAType::Euclidean)
            .collect();
        let geodesic: Vec<usize> = (0..vertices.len())
            .filter(|&v| attribute(v as u32).lra_type == SoftBodyLRAType::Geodesic)
            .collect();
        let anchors: Vec<usize> = (0..vertices.len())
            .filter(|&v| vertices[v].inv_mass == 0.0)
            .collect();
        if !anchors.is_empty() {
            for &v in euclidean.iter() {
                if !(vertices[v].inv_mass > 0.0) {
                    continue;
                }
                let mut best: i64 = -1;
                let mut best_distance: f32 = f32::MAX;
                for &a in anchors.iter() {
                    let d = distance_squared(vertices[v].position, vertices[a].position);
                    if d < best_distance {
                        best_distance = d;
                        best = a as i64;
                    }
                }
                if best >= 0 {
                    let multiplier = attribute(v as u32).lra_max_distance_multiplier;
                    long_range_attachments.push(SoftBodyLRA::new(
                        best as u32,
                        v as u32,
                        best_distance.sqrt() * multiplier,
                    ));
                }
            }
            if !geodesic.is_empty() {
                let (distances, nearest) = geodesic_distances(&edges[..], &vertices[..], &anchors);
                for &v in geodesic.iter() {
                    if !(vertices[v].inv_mass > 0.0 && nearest[v] >= 0) {
                        continue;
                    }
                    let multiplier = attribute(v as u32).lra_max_distance_multiplier;
                    long_range_attachments.push(SoftBodyLRA::new(
                        nearest[v] as u32,
                        v as u32,
                        distances[v] * multiplier,
                    ));
                }
            }
        }

        self.calculate_rest_values();
    }

    // MARK: Rest values

    /// Compute rest lengths, angles, volumes, and attachment distances from the vertex positions.
    /// Attachments with a positive distance keep it.
    pub fn calculate_rest_values(&mut self) {
        // Swift read `vertices` and wrote the constraint arrays; bind the disjoint `vertices` field
        // so the constraint-field writes stay separate borrows.
        let verts = &self.vertices;
        let p = |v: u32| -> Vec3 { verts[v as usize].position };
        for i in 0..self.edges.len() {
            let (a, b) = (self.edges[i].vertex.x, self.edges[i].vertex.y);
            self.edges[i].rest_length = distance(p(a), p(b));
        }
        for i in 0..self.dihedral_bends.len() {
            let b = self.dihedral_bends[i];
            self.dihedral_bends[i].initial_angle =
                dihedral_angle(p(b.vertex.x), p(b.vertex.y), p(b.vertex.z), p(b.vertex.w));
        }
        for i in 0..self.volumes.len() {
            let t = self.volumes[i];
            let x0 = p(t.vertex.x);
            self.volumes[i].six_rest_volume = dot(
                p(t.vertex.y) - x0,
                cross(p(t.vertex.z) - x0, p(t.vertex.w) - x0),
            );
        }
        for i in 0..self.long_range_attachments.len() {
            if !(self.long_range_attachments[i].max_distance <= 0.0) {
                continue;
            }
            let a = self.long_range_attachments[i];
            self.long_range_attachments[i].max_distance = distance(p(a.anchor), p(a.vertex));
        }
        self.calculate_skinned_normal_faces();
    }

    pub(crate) fn calculate_skinned_normal_faces(&mut self) {
        if self.skinned.is_empty() {
            return;
        }
        let mut faces_of_vertex: Vec<Vec<u32>> = vec![Vec::new(); self.vertices.len()];
        for (f, face) in self.faces.iter().enumerate() {
            for v in [face.vertex.x, face.vertex.y, face.vertex.z] {
                faces_of_vertex[v as usize].push(f as u32);
            }
        }
        for i in 0..self.skinned.len() {
            self.skinned[i].normal_faces = faces_of_vertex[self.skinned[i].vertex as usize].clone();
        }
    }

    // MARK: Optimization

    /// Reorder for the solver. Vertices are sorted along a space filling curve so collision can be
    /// batched in compact patches; edges are sorted outward from the kinematic vertices so stiffness
    /// propagates from the anchors in a single sweep. `vertexRemap` records the permutation.
    pub fn optimize(&mut self) {
        if self.vertices.is_empty() {
            return;
        }
        let n = self.vertices.len();

        // Morton order of the rest positions.
        let mut lower = self.vertices[0].position;
        let mut upper = lower;
        for v in self.vertices.iter() {
            lower = min_vec3(lower, v.position);
            upper = max_vec3(upper, v.position);
        }
        let extent = max_vec3(upper - lower, Vec3::new(1.0e-6, 1.0e-6, 1.0e-6));
        let morton = |p: Vec3| -> u32 {
            let q = (p - lower)
                * Vec3::new(1023.0 / extent.x, 1023.0 / extent.y, 1023.0 / extent.z);
            expand_bits(q.x as u32) | (expand_bits(q.y as u32) << 1) | (expand_bits(q.z as u32) << 2)
        };
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| {
            let ma = morton(self.vertices[a].position);
            let mb = morton(self.vertices[b].position);
            if ma != mb {
                ma.cmp(&mb)
            } else {
                a.cmp(&b)
            }
        });
        let mut remap = vec![0u32; n];
        for (new_index, &old_index) in order.iter().enumerate() {
            remap[old_index] = new_index as u32;
        }
        self.apply_vertex_permutation(&order, &remap);
        if self.vertex_remap.is_empty() {
            self.vertex_remap = remap;
        } else {
            let mapped: Vec<u32> = self.vertex_remap.iter().map(|&v| remap[v as usize]).collect();
            self.vertex_remap = mapped;
        }

        // Edges sorted by graph distance from the kinematic vertices.
        let mut depth = vec![i64::MAX; n];
        let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
        for e in self.edges.iter() {
            adjacency[e.vertex.x as usize].push(e.vertex.y as usize);
            adjacency[e.vertex.y as usize].push(e.vertex.x as usize);
        }
        let mut queue: Vec<usize> = Vec::new();
        for i in 0..n {
            if self.vertices[i].inv_mass == 0.0 {
                depth[i] = 0;
                queue.push(i);
            }
        }
        let mut head = 0usize;
        while head < queue.len() {
            let v = queue[head];
            head += 1;
            for &w in adjacency[v].iter() {
                if depth[w] == i64::MAX {
                    depth[w] = depth[v] + 1;
                    queue.push(w);
                }
            }
        }
        let edge_depth: Vec<i64> = self
            .edges
            .iter()
            .map(|e| core::cmp::min(depth[e.vertex.x as usize], depth[e.vertex.y as usize]))
            .collect();
        let mut edge_order: Vec<usize> = (0..self.edges.len()).collect();
        edge_order.sort_by(|&a, &b| {
            if edge_depth[a] != edge_depth[b] {
                edge_depth[a].cmp(&edge_depth[b])
            } else {
                a.cmp(&b)
            }
        });
        let new_edges: Vec<SoftBodyEdge> = edge_order.iter().map(|&i| self.edges[i]).collect();
        self.edges = new_edges;

        self.build_patches();
    }

    fn apply_vertex_permutation(&mut self, order: &[usize], remap: &[u32]) {
        let new_vertices: Vec<SoftBodyVertex> = order.iter().map(|&i| self.vertices[i]).collect();
        self.vertices = new_vertices;
        let r = |v: u32| -> u32 { remap[v as usize] };
        for i in 0..self.faces.len() {
            let v = self.faces[i].vertex;
            self.faces[i].vertex = SoftBodyU32x3::new(r(v.x), r(v.y), r(v.z));
        }
        for i in 0..self.edges.len() {
            let v = self.edges[i].vertex;
            self.edges[i].vertex = SoftBodyU32x2::new(r(v.x), r(v.y));
        }
        for i in 0..self.dihedral_bends.len() {
            let v = self.dihedral_bends[i].vertex;
            self.dihedral_bends[i].vertex = SoftBodyU32x4::new(r(v.x), r(v.y), r(v.z), r(v.w));
        }
        for i in 0..self.volumes.len() {
            let v = self.volumes[i].vertex;
            self.volumes[i].vertex = SoftBodyU32x4::new(r(v.x), r(v.y), r(v.z), r(v.w));
        }
        for i in 0..self.long_range_attachments.len() {
            let a = self.long_range_attachments[i];
            self.long_range_attachments[i].anchor = r(a.anchor);
            self.long_range_attachments[i].vertex = r(a.vertex);
        }
        for i in 0..self.skinned.len() {
            let vertex = self.skinned[i].vertex;
            self.skinned[i].vertex = r(vertex);
        }
        self.calculate_skinned_normal_faces();
    }

    /// Patches of consecutive vertices for collision batching.
    pub(crate) fn build_patches(&mut self) {
        let size = 32;
        let mut patches = Vec::new();
        let mut start = 0usize;
        while start < self.vertices.len() {
            patches.push(start..core::cmp::min(start + size, self.vertices.len()));
            start += size;
        }
        self.patches = patches;
    }

    // MARK: Validation

    /// Check indices and values. Returns a description of the first problem found.
    pub fn validate(&self) -> Option<String> {
        let n = self.vertices.len() as u32;
        if !(n > 0) {
            return Some("no vertices".to_string());
        }
        for v in self.vertices.iter() {
            if !is_valid_vec3(v.position) || !is_valid_vec3(v.velocity) || !(v.inv_mass >= 0.0) {
                return Some("invalid vertex".to_string());
            }
        }
        for f in self.faces.iter() {
            if f.vertex.x >= n || f.vertex.y >= n || f.vertex.z >= n {
                return Some("face index out of range".to_string());
            }
        }
        for e in self.edges.iter() {
            if e.vertex.x >= n || e.vertex.y >= n || !(e.compliance >= 0.0) || e.vertex.x == e.vertex.y {
                return Some("invalid edge".to_string());
            }
        }
        for b in self.dihedral_bends.iter() {
            if b.vertex.x >= n
                || b.vertex.y >= n
                || b.vertex.z >= n
                || b.vertex.w >= n
                || !(b.compliance >= 0.0)
            {
                return Some("invalid dihedral bend".to_string());
            }
        }
        for t in self.volumes.iter() {
            if t.vertex.x >= n
                || t.vertex.y >= n
                || t.vertex.z >= n
                || t.vertex.w >= n
                || !(t.compliance >= 0.0)
            {
                return Some("invalid volume".to_string());
            }
        }
        for a in self.long_range_attachments.iter() {
            if a.anchor >= n || a.vertex >= n || !(a.max_distance >= 0.0) {
                return Some("invalid long range attachment".to_string());
            }
        }
        let joint_count = self.inverse_bind_matrices.len() as u32;
        for s in self.skinned.iter() {
            if !(s.vertex < n) {
                return Some("skinned vertex out of range".to_string());
            }
            for k in 0..4 {
                if s.weights[k] > 0.0 && s.joints[k] >= joint_count {
                    return Some("skinned joint out of range".to_string());
                }
            }
        }
        None
    }

    // MARK: Factories

    /// A rectangular cloth in the xz plane centered on the origin.
    /// `pinned` returns true for grid coordinates that are kinematic.
    /// Swift defaulted `attributes` to `SoftBodyVertexAttributes()`, `bendType` to `.distance`, and
    /// `pinned` to `{ _, _ in false }`; ported callers pass those explicitly.
    pub fn make_cloth<F>(columns: i32, rows: i32, spacing: f32, attributes: SoftBodyVertexAttributes, bend_type: SoftBodyBendType, pinned: F) -> SoftBodySettings
    where
        F: Fn(i32, i32) -> bool,
    {
        let mut settings = SoftBodySettings::new();
        if !(columns >= 2 && rows >= 2) {
            return settings;
        }
        let offset_x = -0.5 * ((columns - 1) as f32) * spacing;
        let offset_z = -0.5 * ((rows - 1) as f32) * spacing;
        for z in 0..rows {
            for x in 0..columns {
                let position = Vec3::new(
                    offset_x + (x as f32) * spacing,
                    0.0,
                    offset_z + (z as f32) * spacing,
                );
                let inv_mass = if pinned(x, z) { 0.0 } else { 1.0 };
                settings
                    .vertices
                    .push(SoftBodyVertex::new(position, Vec3::ZERO, inv_mass));
            }
        }
        let index = |x: i32, z: i32| -> u32 { (x + z * columns) as u32 };
        for z in 0..(rows - 1) {
            for x in 0..(columns - 1) {
                // Faces point up (+y).
                settings
                    .faces
                    .push(SoftBodyFace::new(index(x, z), index(x, z + 1), index(x + 1, z + 1)));
                settings.faces.push(SoftBodyFace::new(
                    index(x, z),
                    index(x + 1, z + 1),
                    index(x + 1, z),
                ));
            }
        }
        settings.create_constraints(&[attributes], bend_type, DEFAULT_ANGLE_TOLERANCE);
        settings.optimize();
        settings
    }

    /// A solid box of tetrahedra with `count` vertices per side.
    pub fn make_cube(count: i32, spacing: f32, compliance: f32, volume_compliance: f32) -> SoftBodySettings {
        let mut settings = SoftBodySettings::new();
        if !(count >= 2) {
            return settings;
        }
        let offset = -0.5 * ((count - 1) as f32) * spacing;
        // Swift computed the flat index in 64-bit `Int` before the `UInt32` cast; use `i64` here to
        // keep that intermediate from overflowing on large counts.
        let index =
            |x: i32, y: i32, z: i32| -> u32 { (x as i64 + y as i64 * count as i64 + z as i64 * count as i64 * count as i64) as u32 };
        for z in 0..count {
            for y in 0..count {
                for x in 0..count {
                    let p = Vec3::new(
                        offset + (x as f32) * spacing,
                        offset + (y as f32) * spacing,
                        offset + (z as f32) * spacing,
                    );
                    settings.vertices.push(SoftBodyVertex::new(p, Vec3::ZERO, 1.0));
                }
            }
        }

        // Five tetrahedra per cell, mirrored on alternate cells so the faces match.
        let mut edge_set: HashSet<u64> = HashSet::new();
        for z in 0..(count - 1) {
            for y in 0..(count - 1) {
                for x in 0..(count - 1) {
                    let mut c: Vec<u32> = Vec::new();
                    for k in 0..8 {
                        c.push(index(
                            x + (k & 1),
                            y + ((k >> 1) & 1),
                            z + ((k >> 2) & 1),
                        ));
                    }
                    let tets: &[[usize; 4]] = if (x + y + z) % 2 == 0 {
                        &[[0, 1, 2, 4], [1, 3, 2, 7], [1, 4, 5, 7], [2, 4, 7, 6], [1, 2, 4, 7]]
                    } else {
                        &[[0, 1, 3, 5], [0, 3, 2, 6], [0, 5, 4, 6], [3, 5, 6, 7], [0, 3, 6, 5]]
                    };
                    for t in tets {
                        let mut volume = SoftBodyVolume::new(c[t[0]], c[t[1]], c[t[2]], c[t[3]], volume_compliance);
                        let negative = {
                            let verts = &settings.vertices;
                            let px = |i: u32| -> Vec3 { verts[i as usize].position };
                            let x0 = px(volume.vertex.x);
                            dot(
                                px(volume.vertex.y) - x0,
                                cross(px(volume.vertex.z) - x0, px(volume.vertex.w) - x0),
                            ) < 0.0
                        };
                        if negative {
                            volume.vertex = SoftBodyU32x4::new(
                                volume.vertex.y,
                                volume.vertex.x,
                                volume.vertex.z,
                                volume.vertex.w,
                            );
                        }
                        settings.volumes.push(volume);
                        for i in 0..4 {
                            for j in (i + 1)..4 {
                                let a = c[t[i]];
                                let b = c[t[j]];
                                let key = (core::cmp::min(a, b) as u64) << 32 | (core::cmp::max(a, b) as u64);
                                if edge_set.insert(key) {
                                    settings.edges.push(SoftBodyEdge::new(a, b, compliance));
                                }
                            }
                        }
                    }
                }
            }
        }

        // Boundary faces, outward facing.
        let last = count - 1;
        let mut quad = |a: u32, b: u32, c: u32, d: u32| {
            settings.faces.push(SoftBodyFace::new(a, b, c));
            settings.faces.push(SoftBodyFace::new(a, c, d));
        };
        for i in 0..last {
            for j in 0..last {
                quad(index(i, j, 0), index(i, j + 1, 0), index(i + 1, j + 1, 0), index(i + 1, j, 0));
                quad(index(i, j, last), index(i + 1, j, last), index(i + 1, j + 1, last), index(i, j + 1, last));
                quad(index(0, i, j), index(0, i, j + 1), index(0, i + 1, j + 1), index(0, i + 1, j));
                quad(index(last, i, j), index(last, i + 1, j), index(last, i + 1, j + 1), index(last, i, j + 1));
                quad(index(i, 0, j), index(i + 1, 0, j), index(i + 1, 0, j + 1), index(i, 0, j + 1));
                quad(index(i, last, j), index(i, last, j + 1), index(i + 1, last, j + 1), index(i + 1, last, j));
            }
        }

        settings.calculate_rest_values();
        settings.optimize();
        settings
    }

    /// A closed sphere surface. Combine with `SoftBodyDef.pressure` for a balloon.
    pub fn make_sphere(radius: f32, subdivisions: i32, compliance: f32, bend_compliance: f32, bend_type: SoftBodyBendType) -> SoftBodySettings {
        let mut settings = SoftBodySettings::new();

        // Icosahedron.
        let t: f32 = (1.0 + (5.0_f32).sqrt()) / 2.0;
        let mut points: Vec<Vec3> = [
            Vec3::new(-1.0, t, 0.0),
            Vec3::new(1.0, t, 0.0),
            Vec3::new(-1.0, -t, 0.0),
            Vec3::new(1.0, -t, 0.0),
            Vec3::new(0.0, -1.0, t),
            Vec3::new(0.0, 1.0, t),
            Vec3::new(0.0, -1.0, -t),
            Vec3::new(0.0, 1.0, -t),
            Vec3::new(t, 0.0, -1.0),
            Vec3::new(t, 0.0, 1.0),
            Vec3::new(-t, 0.0, -1.0),
            Vec3::new(-t, 0.0, 1.0),
        ]
        .iter()
        .map(|&p| normalize(p))
        .collect();
        let mut triangles: Vec<SoftBodyU32x3> = [
            SoftBodyU32x3::new(0, 11, 5),
            SoftBodyU32x3::new(0, 5, 1),
            SoftBodyU32x3::new(0, 1, 7),
            SoftBodyU32x3::new(0, 7, 10),
            SoftBodyU32x3::new(0, 10, 11),
            SoftBodyU32x3::new(1, 5, 9),
            SoftBodyU32x3::new(5, 11, 4),
            SoftBodyU32x3::new(11, 10, 2),
            SoftBodyU32x3::new(10, 7, 6),
            SoftBodyU32x3::new(7, 1, 8),
            SoftBodyU32x3::new(3, 9, 4),
            SoftBodyU32x3::new(3, 4, 2),
            SoftBodyU32x3::new(3, 2, 6),
            SoftBodyU32x3::new(3, 6, 8),
            SoftBodyU32x3::new(3, 8, 9),
            SoftBodyU32x3::new(4, 9, 5),
            SoftBodyU32x3::new(2, 4, 11),
            SoftBodyU32x3::new(6, 2, 10),
            SoftBodyU32x3::new(8, 6, 7),
            SoftBodyU32x3::new(9, 8, 1),
        ]
        .to_vec();
        for _ in 0..subdivisions.max(0) {
            let mut midpoints: HashMap<u64, u32> = HashMap::new();
            let mut midpoint = |a: u32, b: u32| -> u32 {
                let key = (core::cmp::min(a, b) as u64) << 32 | (core::cmp::max(a, b) as u64);
                if let Some(&m) = midpoints.get(&key) {
                    return m;
                }
                points.push(normalize(points[a as usize] + points[b as usize]));
                let m = (points.len() - 1) as u32;
                midpoints.insert(key, m);
                m
            };
            let mut next: Vec<SoftBodyU32x3> = Vec::new();
            for tri in triangles.iter() {
                let a = midpoint(tri.x, tri.y);
                let b = midpoint(tri.y, tri.z);
                let c = midpoint(tri.z, tri.x);
                next.push(SoftBodyU32x3::new(tri.x, a, c));
                next.push(SoftBodyU32x3::new(tri.y, b, a));
                next.push(SoftBodyU32x3::new(tri.z, c, b));
                next.push(SoftBodyU32x3::new(a, b, c));
            }
            triangles = next;
        }

        settings.vertices = points
            .iter()
            .map(|&p| SoftBodyVertex::new(radius * p, Vec3::ZERO, 1.0))
            .collect();
        settings.faces = triangles
            .iter()
            .map(|&tri| SoftBodyFace::new(tri.x, tri.y, tri.z))
            .collect();
        settings.create_constraints(
            &[SoftBodyVertexAttributes::new(
                compliance,
                compliance,
                bend_compliance,
                SoftBodyLRAType::None,
                1.0,
            )],
            bend_type,
            DEFAULT_ANGLE_TOLERANCE,
        );
        settings.optimize();
        settings
    }

    /// A chain of vertices along -y from the origin. The first vertex is kinematic when `pinned`.
    pub fn make_rope(count: i32, spacing: f32, compliance: f32, pinned: bool) -> SoftBodySettings {
        let mut settings = SoftBodySettings::new();
        for i in 0..count.max(0) {
            let position = Vec3::new(0.0, -(i as f32) * spacing, 0.0);
            let inv_mass = if i == 0 && pinned { 0.0 } else { 1.0 };
            settings
                .vertices
                .push(SoftBodyVertex::new(position, Vec3::ZERO, inv_mass));
            if i > 0 {
                settings
                    .edges
                    .push(SoftBodyEdge::new((i - 1) as u32, i as u32, compliance));
            }
        }
        if pinned {
            for i in 1..count.max(1) {
                settings
                    .long_range_attachments
                    .push(SoftBodyLRA::new(0, i as u32, 0.0));
            }
        }
        settings.calculate_rest_values();
        settings.optimize();
        settings
    }
}

// MARK: - Geometry helpers

/// Signed dihedral angle around the edge (x0, x1) between triangles (x0, x1, x2) and (x1, x0, x3).
/// Zero when the triangles are coplanar and unfolded.
#[inline(always)]
pub(crate) fn dihedral_angle(x0: Vec3, x1: Vec3, x2: Vec3, x3: Vec3) -> f32 {
    let e = x1 - x0;
    let n1 = cross(e, x2 - x0);
    let n2 = cross(x3 - x0, e);
    let len = length(e);
    if len > 0.0 {
        atan2_approx(dot(cross(n1, n2), e) / len, dot(n1, n2))
    } else {
        0.0
    }
}

/// Morton bit interleave of a 10-bit lane: Swift's local `expand`.
#[inline]
fn expand_bits(mut v: u32) -> u32 {
    v &= 0x3ff;
    v = (v | (v << 16)) & 0x0300_00ff;
    v = (v | (v << 8)) & 0x0300_f00f;
    v = (v | (v << 4)) & 0x030c_30c3;
    v = (v | (v << 2)) & 0x0924_9249;
    v
}

/// Swift's `max(_:_:)` on `Float`: `x >= y ? x : y`, so a NaN operand propagates the way the stdlib
/// `max` does rather than `f32::max`'s "return the non-NaN argument".
#[inline]
fn fmax(a: f32, b: f32) -> f32 {
    if a >= b { a } else { b }
}

/// Multi source Dijkstra over the edges. Swift's private `geodesicDistances`; it only reads the
/// edge/vertex storage, so the caller passes those slices to keep the borrow disjoint from the
/// `long_range_attachments` writes in `create_constraints`.
fn geodesic_distances(
    edges: &[SoftBodyEdge],
    vertices: &[SoftBodyVertex],
    sources: &[usize],
) -> (Vec<f32>, Vec<i64>) {
    let mut adjacency: Vec<Vec<(usize, f32)>> = vec![Vec::new(); vertices.len()];
    for edge in edges {
        let a = edge.vertex.x as usize;
        let b = edge.vertex.y as usize;
        let d = distance(vertices[a].position, vertices[b].position);
        adjacency[a].push((b, d));
        adjacency[b].push((a, d));
    }

    let mut distances = vec![f32::MAX; vertices.len()];
    let mut nearest = vec![-1i64; vertices.len()];
    let mut heap = BinaryHeap::default();
    for &s in sources {
        distances[s] = 0.0;
        nearest[s] = s as i64;
        heap.push(0.0, s);
    }
    while let Some((d, v)) = heap.pop() {
        if d > distances[v] {
            continue;
        }
        for &(n, w) in adjacency[v].iter() {
            if d + w < distances[n] {
                let nd = d + w;
                distances[n] = nd;
                nearest[n] = nearest[v];
                heap.push(nd, n);
            }
        }
    }
    (distances, nearest)
}

/// A small binary min heap of (distance, vertex).
#[derive(Default)]
struct BinaryHeap {
    items: Vec<(f32, usize)>,
}

impl BinaryHeap {
    fn push(&mut self, d: f32, v: usize) {
        self.items.push((d, v));
        let mut i = self.items.len() - 1;
        while i > 0 {
            let parent = (i - 1) / 2;
            if self.items[parent].0 <= self.items[i].0 {
                break;
            }
            self.items.swap(parent, i);
            i = parent;
        }
    }

    fn pop(&mut self) -> Option<(f32, usize)> {
        let first = *self.items.first()?;
        let last = self.items.pop()?;
        if !self.items.is_empty() {
            self.items[0] = last;
            let mut i = 0usize;
            loop {
                let l = 2 * i + 1;
                let r = l + 1;
                let mut smallest = i;
                if l < self.items.len() && self.items[l].0 < self.items[smallest].0 {
                    smallest = l;
                }
                if r < self.items.len() && self.items[r].0 < self.items[smallest].0 {
                    smallest = r;
                }
                if smallest == i {
                    break;
                }
                self.items.swap(smallest, i);
                i = smallest;
            }
        }
        Some(first)
    }
}
