//! Convex hull construction (incremental QuickHull) and procedural hulls.
//!
//! Faithful port of `Sources/Natsel/Collision/HullBuilder.swift`. Vertex, half-edge, and face
//! ordering is part of the deterministic contract: the finalized hull arrays are laid out by the
//! builder's intrusive list order, and mass properties accumulate over that order. The Swift
//! implementation uses raw pointers; this port keeps the same node identity and free-list reuse
//! order with indices into arenas.

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_log, b3_validate};
use crate::collision::hull::{
    hull_soa_stride, is_valid_hull, update_hull_bounds, update_hull_bulk_properties,
    update_hull_edge_cosines, Hull,
};
use crate::collision::internal_math::*;
use crate::math::functions::*;
use crate::math::types::*;
use crate::types::geometry::{HullFace, HullHalfEdge, HullVertex};

// MARK: - Nodes

const MARK_VISIBLE: i32 = 0;
const MARK_DELETE: i32 = 1;

/// Final hull is index-encoded with `u8`, so counts are capped at 256.
const HULL_MAX_COUNT: i32 = 256;

type Node = u32;
const NIL: Node = u32::MAX;
/// Marker for "this neighbor is the list head".
const HEAD: Node = u32::MAX - 1;

const HEAD_NONE: u32 = u32::MAX;
const HEAD_ORPHANED: u32 = 0;
const HEAD_VERTEX: u32 = 1;
const HEAD_FACE: u32 = 2;
/// Conflict list heads start at this index; face `f` uses `CONFLICT_HEAD_BASE + f`.
const CONFLICT_HEAD_BASE: u32 = 3;

#[derive(Clone, Copy)]
struct ListHead {
    prev: Node,
    next: Node,
}

impl ListHead {
    fn new() -> Self {
        Self { prev: HEAD, next: HEAD }
    }
}

#[derive(Clone, Copy)]
struct QhVertex {
    prev: Node,
    next: Node,
    head: u32,
    conflict_face: Node,
    position: Vec3,
    reachable: bool,
}

#[derive(Clone, Copy)]
struct QhHalfEdge {
    prev: Node,
    next: Node,
    origin: Node,
    face: Node,
    twin: Node,
}

impl QhHalfEdge {
    fn new() -> Self {
        Self {
            prev: NIL,
            next: NIL,
            origin: NIL,
            face: NIL,
            twin: NIL,
        }
    }
}

#[derive(Clone, Copy)]
struct QhFace {
    prev: Node,
    next: Node,
    head: u32,
    edge: Node,
    mark: i32,
    area: f32,
    plane: Plane,
    centroid: Vec3,
    max_conflict_distance: f32,
    max_conflict: Node,
    flipped: bool,
}

impl QhFace {
    fn new() -> Self {
        Self {
            prev: NIL,
            next: NIL,
            head: HEAD_NONE,
            edge: NIL,
            mark: MARK_VISIBLE,
            area: 0.0,
            plane: Plane::new(Vec3::ZERO, 0.0),
            centroid: Vec3::ZERO,
            max_conflict_distance: 0.0,
            max_conflict: NIL,
            flipped: false,
        }
    }
}

/// One frame of the iterative horizon DFS.
struct HorizonFrame {
    face: Node,
    start_edge: Node,
    edge: Node,
    started: bool,
}

// MARK: - Work sizes

struct HullWorkSizes {
    clamped_max_count: i32,
    vertex_capacity: i32,
    edge_capacity: i32,
    face_capacity: i32,
}

impl HullWorkSizes {
    fn new(point_count: i32, clamped_max_count: i32) -> Self {
        // Vertices: 4 initial hull vertices + at most one per remaining input point.
        let vertex_capacity = point_count + 4;
        // Edges and faces use free-list recycling; capacity is proportional to live hull size.
        let edge_capacity = std::cmp::max(48, 24 * clamped_max_count - 48);
        let face_capacity = std::cmp::max(16, 5 * clamped_max_count - 10);
        Self {
            clamped_max_count,
            vertex_capacity,
            edge_capacity,
            face_capacity,
        }
    }
}

// MARK: - Builder

struct HullBuilder {
    tolerance: f32,
    min_radius: f32,
    min_outside: f32,

    interior_point: Vec3,

    // List sentinels: [orphaned, vertex, face, conflict lists...].
    heads: Vec<ListHead>,

    vertices: Vec<QhVertex>,
    vertex_capacity: i32,
    vertex_count: i32,

    edges: Vec<QhHalfEdge>,
    edge_capacity: i32,
    edge_count: i32,
    // LIFO free list, built with `edges[i].next`.
    edge_free_head: Node,

    faces: Vec<QhFace>,
    face_capacity: i32,
    face_count: i32,
    // LIFO free list, built with `faces[i].next`.
    face_free_head: Node,

    // Reusable scratch buffers.
    horizon: Vec<Node>,
    cone: Vec<Node>,
    merged_faces: Vec<Node>,
    horizon_stack: Vec<HorizonFrame>,

    // Final counts of the constructed hull.
    final_vertex_count: i32,
    final_half_edge_count: i32,
    final_face_count: i32,
}

impl HullBuilder {
    fn new(sizes: &HullWorkSizes) -> Self {
        let mut heads = Vec::with_capacity(3 + sizes.face_capacity as usize);
        for _ in 0..3 + sizes.face_capacity as usize {
            heads.push(ListHead::new());
        }
        let horizon_capacity = std::cmp::max(6, 3 * sizes.clamped_max_count - 6) as usize;
        let merged_capacity = std::cmp::max(4, 2 * sizes.clamped_max_count - 4) as usize;
        Self {
            tolerance: 0.0,
            min_radius: 0.0,
            min_outside: 0.0,
            interior_point: Vec3::ZERO,
            heads,
            vertices: Vec::with_capacity(sizes.vertex_capacity as usize),
            vertex_capacity: sizes.vertex_capacity,
            vertex_count: 0,
            edges: Vec::with_capacity(sizes.edge_capacity as usize),
            edge_capacity: sizes.edge_capacity,
            edge_count: 0,
            edge_free_head: NIL,
            faces: Vec::with_capacity(sizes.face_capacity as usize),
            face_capacity: sizes.face_capacity,
            face_count: 0,
            face_free_head: NIL,
            horizon: Vec::with_capacity(horizon_capacity),
            cone: Vec::with_capacity(horizon_capacity),
            merged_faces: Vec::with_capacity(merged_capacity),
            horizon_stack: Vec::with_capacity(merged_capacity),
            final_vertex_count: 0,
            final_half_edge_count: 0,
            final_face_count: 0,
        }
    }

    // MARK: List operations

    fn list_empty(&self, head: u32) -> bool {
        self.heads[head as usize].next == HEAD
    }

    fn vertex_list_contains(vertex: &QhVertex) -> bool {
        vertex.prev != NIL && vertex.next != NIL
    }

    /// Swift's `listPushBack(head, node)` inserts the node *before* `head.prev`, i.e. between the
    /// current tail and its predecessor. The first node pushed into an empty list therefore stays
    /// at the tail forever. Replicating that quirk is required for identical list ordering.
    fn link_vertex_back(&mut self, head: u32, vertex: Node) {
        let before = self.heads[head as usize].prev;
        let where_prev = if before == HEAD {
            HEAD
        } else {
            self.vertices[before as usize].prev
        };
        {
            let v = &mut self.vertices[vertex as usize];
            v.prev = where_prev;
            v.next = before;
            v.head = head;
        }
        if where_prev == HEAD {
            self.heads[head as usize].next = vertex;
        } else {
            self.vertices[where_prev as usize].next = vertex;
        }
        if before == HEAD {
            self.heads[head as usize].prev = vertex;
        } else {
            self.vertices[before as usize].prev = vertex;
        }
    }

    fn unlink_vertex(&mut self, vertex: Node) {
        let v = self.vertices[vertex as usize];
        let head = v.head;
        if v.prev == HEAD {
            self.heads[head as usize].next = v.next;
        } else {
            self.vertices[v.prev as usize].next = v.next;
        }
        if v.next == HEAD {
            self.heads[head as usize].prev = v.prev;
        } else {
            self.vertices[v.next as usize].prev = v.prev;
        }
        let v = &mut self.vertices[vertex as usize];
        v.prev = NIL;
        v.next = NIL;
        v.head = HEAD_NONE;
    }

    fn face_list_contains(face: &QhFace) -> bool {
        face.prev != NIL && face.next != NIL
    }

    fn link_face_back(&mut self, face: Node) {
        let head = HEAD_FACE;
        let before = self.heads[head as usize].prev;
        let where_prev = if before == HEAD {
            HEAD
        } else {
            self.faces[before as usize].prev
        };
        {
            let f = &mut self.faces[face as usize];
            f.prev = where_prev;
            f.next = before;
            f.head = head;
        }
        if where_prev == HEAD {
            self.heads[head as usize].next = face;
        } else {
            self.faces[where_prev as usize].next = face;
        }
        if before == HEAD {
            self.heads[head as usize].prev = face;
        } else {
            self.faces[before as usize].prev = face;
        }
    }

    fn unlink_face(&mut self, face: Node) {
        let f = self.faces[face as usize];
        let head = f.head;
        if f.prev == HEAD {
            self.heads[head as usize].next = f.next;
        } else {
            self.faces[f.prev as usize].next = f.next;
        }
        if f.next == HEAD {
            self.heads[head as usize].prev = f.prev;
        } else {
            self.faces[f.next as usize].prev = f.prev;
        }
        let f = &mut self.faces[face as usize];
        f.prev = NIL;
        f.next = NIL;
        f.head = HEAD_NONE;
    }

    // MARK: Pool management

    fn new_vertex(&mut self, position: Vec3) -> Node {
        b3_assert(self.vertex_count < self.vertex_capacity);
        let vertex = self.vertex_count as Node;
        self.vertex_count += 1;
        self.vertices.push(QhVertex {
            prev: NIL,
            next: NIL,
            head: HEAD_NONE,
            conflict_face: NIL,
            position,
            reachable: false,
        });
        vertex
    }

    fn new_edge(&mut self) -> Node {
        let edge;
        if self.edge_free_head != NIL {
            edge = self.edge_free_head;
            self.edge_free_head = self.edges[edge as usize].next;
        } else {
            b3_assert(self.edge_count < self.edge_capacity);
            edge = self.edge_count as Node;
            self.edge_count += 1;
            self.edges.push(QhHalfEdge::new());
        }
        edge
    }

    fn retire_edge(&mut self, edge: Node) {
        self.edges[edge as usize].next = self.edge_free_head;
        self.edge_free_head = edge;
    }

    fn new_face(&mut self, v1: Node, v2: Node, v3: Node) -> Node {
        let face;
        if self.face_free_head != NIL {
            face = self.face_free_head;
            self.face_free_head = self.faces[face as usize].next;
        } else {
            b3_assert(self.face_count < self.face_capacity);
            face = self.face_count as Node;
            self.face_count += 1;
            self.faces.push(QhFace::new());
        }

        {
            let f = &mut self.faces[face as usize];
            f.prev = NIL;
            f.next = NIL;
            f.max_conflict = NIL;
            f.max_conflict_distance = 0.0;
        }

        let edge1 = self.new_edge();
        let edge2 = self.new_edge();
        let edge3 = self.new_edge();

        let p1 = self.vertices[v1 as usize].position;
        let p2 = self.vertices[v2 as usize].position;
        let p3 = self.vertices[v3 as usize].position;

        let cross_normal = cross(p2 - p1, p3 - p1);
        let (length, normal) = get_length_and_normalize(cross_normal);
        let plane = Plane::new(normal, dot(normal, p1));
        let area = 0.5 * length;

        {
            let f = &mut self.faces[face as usize];
            f.edge = edge1;
            f.mark = MARK_VISIBLE;
            f.area = area;
            f.centroid = (1.0 / 3.0) * (p1 + (p2 + p3));
            f.plane = plane;
            f.flipped = plane_separation(plane, self.interior_point) > 0.0;
        }
        self.heads[(CONFLICT_HEAD_BASE + face) as usize] = ListHead::new();

        {
            let e1 = &mut self.edges[edge1 as usize];
            e1.prev = edge3;
            e1.next = edge2;
            e1.origin = v1;
            e1.face = face;
            e1.twin = NIL;
        }
        {
            let e2 = &mut self.edges[edge2 as usize];
            e2.prev = edge1;
            e2.next = edge3;
            e2.origin = v2;
            e2.face = face;
            e2.twin = NIL;
        }
        {
            let e3 = &mut self.edges[edge3 as usize];
            e3.prev = edge2;
            e3.next = edge1;
            e3.origin = v3;
            e3.face = face;
            e3.twin = NIL;
        }

        face
    }

    /// Remove face from its list (if present) and add it to the free list.
    fn retire_face(&mut self, face: Node) {
        if Self::face_list_contains(&self.faces[face as usize]) {
            self.unlink_face(face);
        }

        {
            let f = &mut self.faces[face as usize];
            f.edge = NIL;
            b3_validate(f.prev == NIL);
            f.next = self.face_free_head;
        }
        self.face_free_head = face;
    }
}

// MARK: - Geometry helpers

fn build_bounds(vertices: &[Vec3]) -> AABB {
    let mut bounds = bounds3_empty();
    for v in vertices {
        bounds.lower_bound = min_vec3(bounds.lower_bound, *v);
        bounds.upper_bound = max_vec3(bounds.upper_bound, *v);
    }
    bounds
}

fn find_farthest_points_along_cardinal_axes(
    tolerance: f32,
    vertices: &[Vec3],
) -> Option<(i32, i32)> {
    let v0 = vertices[0];
    let mut min_pt = [v0, v0, v0];
    let mut max_pt = [v0, v0, v0];
    let mut min_index: [i32; 3] = [0, 0, 0];
    let mut max_index: [i32; 3] = [0, 0, 0];

    for (i, v) in vertices.iter().enumerate().skip(1) {
        if v.x < min_pt[0].x {
            min_pt[0] = *v;
            min_index[0] = i as i32;
        } else if v.x > max_pt[0].x {
            max_pt[0] = *v;
            max_index[0] = i as i32;
        }

        if v.y < min_pt[1].y {
            min_pt[1] = *v;
            min_index[1] = i as i32;
        } else if v.y > max_pt[1].y {
            max_pt[1] = *v;
            max_index[1] = i as i32;
        }

        if v.z < min_pt[2].z {
            min_pt[2] = *v;
            min_index[2] = i as i32;
        } else if v.z > max_pt[2].z {
            max_pt[2] = *v;
            max_index[2] = i as i32;
        }
    }

    let distance = Vec3::new(
        max_pt[0].x - min_pt[0].x,
        max_pt[1].y - min_pt[1].y,
        max_pt[2].z - min_pt[2].z,
    );

    let max_element = max_element_index(distance) as usize;

    if get_by_index(distance, max_element) > 2.0 * tolerance {
        Some((min_index[max_element], max_index[max_element]))
    } else {
        None
    }
}

fn find_farthest_point_from_line(
    index1: i32,
    index2: i32,
    tolerance: f32,
    vertices: &[Vec3],
) -> i32 {
    let a = vertices[index1 as usize];
    let b = vertices[index2 as usize];

    // |ap x ab|^2 / |ab|^2 is the squared perpendicular distance from p to the line.
    // Compares against (2 * tolerance)^2.
    let ab = b - a;
    let ab_length_sqr = dot(ab, ab);
    b3_assert(ab_length_sqr > 0.0);

    let inv_ab_length_sqr = 1.0 / ab_length_sqr;
    let mut max_distance_sqr = 4.0 * tolerance * tolerance;
    let mut max_index = b3::NULL_INDEX;

    for (i, v) in vertices.iter().enumerate() {
        if i as i32 == index1 || i as i32 == index2 {
            continue;
        }

        let ap = *v - a;
        let c = cross(ap, ab);
        let distance_sqr = dot(c, c) * inv_ab_length_sqr;
        if distance_sqr > max_distance_sqr {
            max_distance_sqr = distance_sqr;
            max_index = i as i32;
        }
    }

    max_index
}

fn find_farthest_point_from_plane(
    index1: i32,
    index2: i32,
    index3: i32,
    tolerance: f32,
    vertices: &[Vec3],
) -> i32 {
    let a = vertices[index1 as usize];
    let b = vertices[index2 as usize];
    let c = vertices[index3 as usize];

    let plane = make_plane_from_points(a, b, c);

    let mut max_distance = 2.0 * tolerance;
    let mut max_index = b3::NULL_INDEX;

    for (i, v) in vertices.iter().enumerate() {
        if i as i32 == index1 || i as i32 == index2 || i as i32 == index3 {
            continue;
        }

        let distance = abs_float(plane_separation(plane, *v));
        if distance > max_distance {
            max_distance = distance;
            max_index = i as i32;
        }
    }

    max_index
}

// MARK: - Builder geometry

impl HullBuilder {
    fn is_edge_convex(&self, edge: Node, tolerance: f32) -> bool {
        let face = self.edges[edge as usize].face;
        let twin_face = self.edges[self.edges[edge as usize].twin as usize].face;
        let distance = plane_separation(
            self.faces[face as usize].plane,
            self.faces[twin_face as usize].centroid,
        );
        distance < -tolerance
    }

    fn is_edge_concave(&self, edge: Node, tolerance: f32) -> bool {
        let face = self.edges[edge as usize].face;
        let twin_face = self.edges[self.edges[edge as usize].twin as usize].face;
        let distance = plane_separation(
            self.faces[face as usize].plane,
            self.faces[twin_face as usize].centroid,
        );
        distance > tolerance
    }

    fn vertex_count_of_face(&self, face: Node) -> i32 {
        let mut count = 0;
        let start = self.faces[face as usize].edge;
        let mut edge = start;
        loop {
            count += 1;
            edge = self.edges[edge as usize].next;
            if edge == start {
                break;
            }
        }
        count
    }

    fn link_face(&mut self, face: Node, index: i32, twin: Node) {
        b3_assert(face != self.edges[twin as usize].face);

        let mut edge = self.faces[face as usize].edge;
        let mut i = index;
        while i > 0 {
            b3_assert(self.edges[edge as usize].face == face);
            edge = self.edges[edge as usize].next;
            i -= 1;
        }

        b3_assert(edge != twin);
        self.edges[edge as usize].twin = twin;
        self.edges[twin as usize].twin = edge;
    }

    fn link_faces(&mut self, face1: Node, index1: i32, face2: Node, index2: i32) {
        b3_assert(face1 != face2);

        let mut edge1 = self.faces[face1 as usize].edge;
        let mut i1 = index1;
        while i1 > 0 {
            edge1 = self.edges[edge1 as usize].next;
            i1 -= 1;
        }

        let mut edge2 = self.faces[face2 as usize].edge;
        let mut i2 = index2;
        while i2 > 0 {
            edge2 = self.edges[edge2 as usize].next;
            i2 -= 1;
        }

        b3_assert(edge1 != edge2);
        self.edges[edge1 as usize].twin = edge2;
        self.edges[edge2 as usize].twin = edge1;
    }

    fn newell_plane(&mut self, face: Node) {
        let mut count = 0.0f32;
        let mut centroid = Vec3::ZERO;
        let mut normal = Vec3::ZERO;

        let start = self.faces[face as usize].edge;
        let mut edge = start;
        b3_assert(self.edges[edge as usize].face == face);

        // Use the first vertex as the origin to reduce round-off.
        let origin = self.vertices[self.edges[edge as usize].origin as usize].position;

        loop {
            let edge_origin = self.edges[edge as usize].origin;
            let twin = self.edges[edge as usize].twin;
            b3_assert(self.edges[twin as usize].twin == edge);

            let v1 = self.vertices[edge_origin as usize].position - origin;
            let v2 = self.vertices[self.edges[twin as usize].origin as usize].position - origin;

            count += 1.0;
            centroid = centroid + v1;
            normal.x += (v1.y - v2.y) * (v1.z + v2.z);
            normal.y += (v1.z - v2.z) * (v1.x + v2.x);
            normal.z += (v1.x - v2.x) * (v1.y + v2.y);

            edge = self.edges[edge as usize].next;
            if edge == start {
                break;
            }
        }

        b3_assert(count > 0.0);
        centroid = (1.0 / count) * centroid;
        centroid = centroid + origin;

        let length = length(normal);
        b3_validate(length > 0.0);
        normal = (1.0 / length) * normal;

        let f = &mut self.faces[face as usize];
        f.centroid = centroid;
        f.plane = make_plane_from_normal_and_point(normal, centroid);
        f.area = 0.5 * length;
    }

    fn compute_tolerance(&mut self, points: &[Vec3]) {
        let bounds = build_bounds(points);
        let max_abs = max_vec3(abs_vec3(bounds.lower_bound), abs_vec3(bounds.upper_bound));

        let max_sum = max_abs.x + max_abs.y + max_abs.z;
        let max_coord = max_float(max_abs.x, max_float(max_abs.y, max_abs.z));
        let max_distance = min_float(b3::SQRT3 * max_coord, max_sum);

        let tolerance = (3.0 * max_distance * 1.01 + max_coord) * f32::EPSILON;

        self.tolerance = tolerance;
        self.min_radius = 4.0 * self.tolerance;
        self.min_outside = 2.0 * self.min_radius;
        b3_assert(self.min_radius < self.min_outside + 3.0 * f32::EPSILON);
    }

    fn build_initial_hull(&mut self, points: &[Vec3]) -> bool {
        let Some((index1, index2)) =
            find_farthest_points_along_cardinal_axes(self.tolerance, points)
        else {
            return false;
        };
        if index1 < 0 || index2 < 0 {
            return false;
        }

        let index3 = find_farthest_point_from_line(index1, index2, self.tolerance, points);
        if index3 < 0 {
            return false;
        }

        let index4 =
            find_farthest_point_from_plane(index1, index2, index3, self.tolerance, points);
        if index4 < 0 {
            return false;
        }

        let v1 = points[index1 as usize] - points[index4 as usize];
        let v2 = points[index2 as usize] - points[index4 as usize];
        let v3 = points[index3 as usize] - points[index4 as usize];

        let mut idx2 = index2;
        let mut idx3 = index3;
        if scalar_triple_product(v1, v2, v3) < 0.0 {
            std::mem::swap(&mut idx2, &mut idx3);
        }

        self.interior_point = Vec3::ZERO;
        self.interior_point = self.interior_point + points[index1 as usize];
        self.interior_point = self.interior_point + points[idx2 as usize];
        self.interior_point = self.interior_point + points[idx3 as usize];
        self.interior_point = self.interior_point + points[index4 as usize];
        self.interior_point = 0.25 * self.interior_point;

        let vertex1 = self.new_vertex(points[index1 as usize]);
        self.link_vertex_back(HEAD_VERTEX, vertex1);
        let vertex2 = self.new_vertex(points[idx2 as usize]);
        self.link_vertex_back(HEAD_VERTEX, vertex2);
        let vertex3 = self.new_vertex(points[idx3 as usize]);
        self.link_vertex_back(HEAD_VERTEX, vertex3);
        let vertex4 = self.new_vertex(points[index4 as usize]);
        self.link_vertex_back(HEAD_VERTEX, vertex4);

        let face1 = self.new_face(vertex1, vertex2, vertex3);
        self.link_face_back(face1);
        let face2 = self.new_face(vertex4, vertex2, vertex1);
        self.link_face_back(face2);
        let face3 = self.new_face(vertex4, vertex3, vertex2);
        self.link_face_back(face3);
        let face4 = self.new_face(vertex4, vertex1, vertex3);
        self.link_face_back(face4);

        self.link_faces(face1, 0, face2, 1);
        self.link_faces(face1, 1, face3, 1);
        self.link_faces(face1, 2, face4, 1);

        self.link_faces(face2, 0, face3, 2);
        self.link_faces(face3, 0, face4, 2);
        self.link_faces(face4, 0, face2, 2);

        for (index, point) in points.iter().enumerate() {
            if index as i32 == index1
                || index as i32 == idx2
                || index as i32 == idx3
                || index as i32 == index4
            {
                continue;
            }

            let mut max_distance = self.min_outside;
            let mut max_face = NIL;

            let mut node = self.heads[HEAD_FACE as usize].next;
            while node != HEAD {
                let face = node;
                let distance = plane_separation(self.faces[face as usize].plane, *point);
                if distance > max_distance {
                    max_distance = distance;
                    max_face = face;
                }
                node = self.faces[face as usize].next;
            }

            if max_face != NIL {
                let vertex = self.new_vertex(*point);
                self.vertices[vertex as usize].conflict_face = max_face;
                self.link_vertex_back(CONFLICT_HEAD_BASE + max_face, vertex);
                if max_distance > self.faces[max_face as usize].max_conflict_distance {
                    self.faces[max_face as usize].max_conflict_distance = max_distance;
                    self.faces[max_face as usize].max_conflict = vertex;
                }
            }
        }

        true
    }

    /// Recompute the farthest-conflict cache after a face's plane changes.
    fn recache_conflicts(&mut self, face: Node) {
        let mut max_vertex = NIL;
        let mut max_distance = self.min_outside;

        let head = CONFLICT_HEAD_BASE + face;
        let plane = self.faces[face as usize].plane;
        let mut node = self.heads[head as usize].next;
        while node != HEAD {
            let vertex = node;
            let distance = plane_separation(plane, self.vertices[vertex as usize].position);
            if distance > max_distance {
                max_distance = distance;
                max_vertex = vertex;
            }
            node = self.vertices[vertex as usize].next;
        }

        let f = &mut self.faces[face as usize];
        f.max_conflict = max_vertex;
        f.max_conflict_distance = max_distance;
    }

    fn next_conflict_vertex(&self) -> Node {
        let mut max_vertex = NIL;
        let mut max_distance = self.min_outside;

        let mut face_node = self.heads[HEAD_FACE as usize].next;
        while face_node != HEAD {
            let face = face_node;
            let f = &self.faces[face as usize];
            if f.max_conflict != NIL && f.max_conflict_distance > max_distance {
                max_distance = f.max_conflict_distance;
                max_vertex = f.max_conflict;
            }
            face_node = self.faces[face as usize].next;
        }

        max_vertex
    }

    /// Move every conflict vertex of `face` onto the orphaned list and clear their `conflict_face`.
    fn drain_conflict_list(&mut self, face: Node) {
        let head = CONFLICT_HEAD_BASE + face;
        let mut node = self.heads[head as usize].next;
        while node != HEAD {
            let orphan = node;
            node = self.vertices[orphan as usize].next;

            self.vertices[orphan as usize].conflict_face = NIL;
            self.unlink_vertex(orphan);
            self.link_vertex_back(HEAD_ORPHANED, orphan);
        }
        b3_assert(self.list_empty(head));
    }

    /// Mark a face for deletion, drain its conflict list, and populate a fresh DFS frame for it.
    fn enter_horizon_face(&mut self, face: Node, entry_edge: Node) -> HorizonFrame {
        self.faces[face as usize].mark = MARK_DELETE;
        self.drain_conflict_list(face);

        if entry_edge != NIL {
            HorizonFrame {
                face,
                start_edge: entry_edge,
                edge: self.edges[entry_edge as usize].next,
                started: false,
            }
        } else {
            let edge = self.faces[face as usize].edge;
            HorizonFrame {
                face,
                start_edge: edge,
                edge,
                started: false,
            }
        }
    }

    fn build_horizon(&mut self, apex: Node, seed: Node) {
        self.horizon_stack.clear();

        let frame = self.enter_horizon_face(seed, NIL);
        self.horizon_stack.push(frame);

        while !self.horizon_stack.is_empty() {
            let top = self.horizon_stack.len() - 1;

            if self.horizon_stack[top].started
                && self.horizon_stack[top].edge == self.horizon_stack[top].start_edge
            {
                self.horizon_stack.pop();
                continue;
            }
            self.horizon_stack[top].started = true;

            let edge = self.horizon_stack[top].edge;
            let twin = self.edges[edge as usize].twin;
            self.horizon_stack[top].edge = self.edges[edge as usize].next;

            let twin_face = self.edges[twin as usize].face;
            if self.faces[twin_face as usize].mark != MARK_VISIBLE {
                continue;
            }

            let distance = plane_separation(
                self.faces[twin_face as usize].plane,
                self.vertices[apex as usize].position,
            );
            if distance > self.min_radius {
                let new_frame = self.enter_horizon_face(twin_face, twin);
                self.horizon_stack.push(new_frame);
            } else {
                self.horizon.push(edge);
            }
        }
    }

    fn build_cone(&mut self, apex: Node) {
        for i in 0..self.horizon.len() {
            let edge = self.horizon[i];
            b3_assert(self.edges[self.edges[edge as usize].twin as usize].twin == edge);

            let edge_origin = self.edges[edge as usize].origin;
            let twin = self.edges[edge as usize].twin;
            let twin_origin = self.edges[twin as usize].origin;
            let face = self.new_face(apex, edge_origin, twin_origin);
            self.cone.push(face);

            self.link_face(face, 1, twin);
        }

        let mut face1 = self.cone[self.cone.len() - 1];
        for i in 0..self.cone.len() {
            let face2 = self.cone[i];
            self.link_faces(face1, 2, face2, 0);
            face1 = face2;
        }
    }

    /// Retire half-edges in the half-open ring range [begin, end).
    fn destroy_edges(&mut self, begin: Node, end: Node) {
        let mut edge = begin;
        while edge != end {
            let next = self.edges[edge as usize].next;
            self.retire_edge(edge);
            edge = next;
        }
    }

    fn connect_edges(&mut self, prev: Node, next: Node) {
        b3_assert(prev != next);
        b3_assert(self.edges[prev as usize].face == self.edges[next as usize].face);

        let prev_twin = self.edges[prev as usize].twin;
        let next_twin = self.edges[next as usize].twin;

        // If both shared neighbors are the same face, prev and next together would orphan it.
        if self.edges[prev_twin as usize].face == self.edges[next_twin as usize].face {
            let face = self.edges[next as usize].face;
            // next is redundant.
            if self.faces[face as usize].edge == next {
                self.faces[face as usize].edge = prev;
            }

            let twin;
            if self.vertex_count_of_face(self.edges[prev_twin as usize].face) == 3 {
                // Capture all 3 half-edges of the dead triangle before the rewire overwrites
                // prev.twin.
                let dead_edge0 = prev_twin;
                let dead_edge1 = next_twin;
                let dead_edge2 = self.edges[next_twin as usize].prev;

                twin = self.edges[dead_edge2 as usize].twin;
                b3_assert(
                    self.faces[self.edges[twin as usize].face as usize].mark != MARK_DELETE,
                );

                let opposing_face = self.edges[prev_twin as usize].face;
                self.faces[opposing_face as usize].mark = MARK_DELETE;
                self.merged_faces.push(opposing_face);

                let next_next = self.edges[next as usize].next;
                self.edges[prev as usize].next = next_next;
                self.edges[next_next as usize].prev = prev;

                self.edges[prev as usize].twin = twin;
                self.edges[twin as usize].twin = prev;

                // Drop the redundant vertex (slot abandoned in the bump allocator).
                self.unlink_vertex(self.edges[next as usize].origin);

                // Retire the 3 half-edges of the dead triangle now that the rewire is complete.
                self.retire_edge(dead_edge0);
                self.retire_edge(dead_edge1);
                self.retire_edge(dead_edge2);
            } else {
                twin = next_twin;

                let twin_face = self.edges[twin as usize].face;
                if self.faces[twin_face as usize].edge == prev_twin {
                    self.faces[twin_face as usize].edge = twin;
                }

                let twin_next = self.edges[prev_twin as usize].next;
                self.edges[twin as usize].next = twin_next;
                self.edges[twin_next as usize].prev = twin;
                // prev.twin slot is retired to the edge free list.
                self.retire_edge(prev_twin);

                let next_next = self.edges[next as usize].next;
                self.edges[prev as usize].next = next_next;
                self.edges[next_next as usize].prev = prev;

                self.edges[prev as usize].twin = twin;
                self.edges[twin as usize].twin = prev;

                // Drop the redundant vertex (slot abandoned in the bump allocator).
                self.unlink_vertex(self.edges[next as usize].origin);
            }

            // Twin.face changed shape; recompute its plane and refresh its cached max conflict.
            let twin_face = self.edges[twin as usize].face;
            self.newell_plane(twin_face);
            self.recache_conflicts(twin_face);
        } else {
            self.edges[prev as usize].next = next;
            self.edges[next as usize].prev = prev;
        }
    }

    fn absorb_faces(&mut self, face: Node) {
        for i in 0..self.merged_faces.len() {
            let merged = self.merged_faces[i];
            b3_assert(self.faces[merged as usize].mark == MARK_DELETE);
            let head = CONFLICT_HEAD_BASE + merged;

            let mut node = self.heads[head as usize].next;
            while node != HEAD {
                let vertex = node;
                node = self.vertices[vertex as usize].next;

                self.unlink_vertex(vertex);

                let distance = plane_separation(
                    self.faces[face as usize].plane,
                    self.vertices[vertex as usize].position,
                );
                if distance > self.min_outside {
                    self.link_vertex_back(CONFLICT_HEAD_BASE + face, vertex);
                    self.vertices[vertex as usize].conflict_face = face;
                    if distance > self.faces[face as usize].max_conflict_distance {
                        self.faces[face as usize].max_conflict_distance = distance;
                        self.faces[face as usize].max_conflict = vertex;
                    }
                } else {
                    self.link_vertex_back(HEAD_ORPHANED, vertex);
                    self.vertices[vertex as usize].conflict_face = NIL;
                }
            }

            b3_assert(self.list_empty(head));

            // Conflict list is now drained. Retire this face to the free list.
            self.retire_face(merged);
        }
    }

    fn connect_faces(&mut self, edge: Node) {
        let face = self.edges[edge as usize].face;
        let twin = self.edges[edge as usize].twin;

        let mut edge_prev = self.edges[edge as usize].prev;
        let mut edge_next = self.edges[edge as usize].next;
        let mut twin_prev = self.edges[twin as usize].prev;
        let mut twin_next = self.edges[twin as usize].next;
        let twin_face = self.edges[twin as usize].face;

        while self.edges[self.edges[edge_prev as usize].twin as usize].face == twin_face {
            b3_assert(self.edges[edge_prev as usize].twin == twin_next);
            b3_assert(self.edges[twin_next as usize].twin == edge_prev);

            edge_prev = self.edges[edge_prev as usize].prev;
            twin_next = self.edges[twin_next as usize].next;
        }
        b3_assert(self.edges[edge_prev as usize].face != self.edges[twin_next as usize].face);

        while self.edges[self.edges[edge_next as usize].twin as usize].face == twin_face {
            b3_assert(self.edges[edge_next as usize].twin == twin_prev);
            b3_assert(self.edges[twin_prev as usize].twin == edge_next);

            edge_next = self.edges[edge_next as usize].next;
            twin_prev = self.edges[twin_prev as usize].prev;
        }
        b3_assert(self.edges[edge_next as usize].face != self.edges[twin_prev as usize].face);

        self.faces[face as usize].edge = edge_prev;

        // Discard opposing face. mergedFaces is single-buffered: connectFaces does not nest.
        self.merged_faces.clear();
        self.merged_faces.push(twin_face);
        self.faces[twin_face as usize].mark = MARK_DELETE;
        self.faces[twin_face as usize].edge = NIL;

        let absorbed_end = self.edges[twin_prev as usize].next;
        let mut absorbed = twin_next;
        while absorbed != absorbed_end {
            self.edges[absorbed as usize].face = face;
            absorbed = self.edges[absorbed as usize].next;
        }

        let destroy_begin = self.edges[edge_prev as usize].next;
        self.destroy_edges(destroy_begin, edge_next);
        let destroy_begin = self.edges[twin_prev as usize].next;
        self.destroy_edges(destroy_begin, twin_next);

        self.connect_edges(edge_prev, twin_next);
        self.connect_edges(twin_prev, edge_next);

        self.newell_plane(face);
        // Existing conflicts now have stale distances under the new plane; absorbFaces will then
        // add more incrementally and update the cache as it goes.
        self.recache_conflicts(face);

        self.absorb_faces(face);
    }

    fn merge_concave(&mut self, face: Node) -> bool {
        let start = self.faces[face as usize].edge;
        let mut edge = start;

        loop {
            let twin = self.edges[edge as usize].twin;

            if self.is_edge_concave(edge, self.min_radius)
                || self.is_edge_concave(twin, self.min_radius)
            {
                self.connect_faces(edge);
                return true;
            }

            edge = self.edges[edge as usize].next;
            if edge == self.faces[face as usize].edge {
                break;
            }
        }

        false
    }

    fn merge_coplanar(&mut self, face: Node) -> bool {
        let start = self.faces[face as usize].edge;
        let mut edge = start;

        loop {
            let twin = self.edges[edge as usize].twin;

            if !self.is_edge_convex(edge, self.min_radius)
                || !self.is_edge_convex(twin, self.min_radius)
            {
                self.connect_faces(edge);
                return true;
            }

            edge = self.edges[edge as usize].next;
            if edge == self.faces[face as usize].edge {
                break;
            }
        }

        false
    }

    fn merge_faces(&mut self) {
        for i in 0..self.cone.len() {
            let face = self.cone[i];
            if self.faces[face as usize].mark == MARK_VISIBLE
                && self.faces[face as usize].flipped
            {
                self.faces[face as usize].flipped = false;

                let mut best_area = 0.0f32;
                let mut best_edge = NIL;

                let mut edge = self.faces[face as usize].edge;
                loop {
                    let twin = self.edges[edge as usize].twin;
                    let area = self.faces[self.edges[twin as usize].face as usize].area;
                    if area > best_area {
                        best_area = area;
                        best_edge = edge;
                    }

                    edge = self.edges[edge as usize].next;
                    if edge == self.faces[face as usize].edge {
                        break;
                    }
                }

                b3_assert(best_edge != NIL);
                self.connect_faces(best_edge);
            }
        }

        for i in 0..self.cone.len() {
            let face = self.cone[i];
            if self.faces[face as usize].mark == MARK_VISIBLE {
                while self.merge_concave(face) {}
            }
        }

        for i in 0..self.cone.len() {
            let face = self.cone[i];
            if self.faces[face as usize].mark == MARK_VISIBLE {
                while self.merge_coplanar(face) {}
            }
        }
    }

    fn resolve_vertices(&mut self) {
        let mut node = self.heads[HEAD_ORPHANED as usize].next;
        while node != HEAD {
            let vertex = node;
            node = self.vertices[vertex as usize].next;
            self.unlink_vertex(vertex);

            let mut max_distance = self.min_outside;
            let mut max_face = NIL;

            for i in 0..self.cone.len() {
                let face = self.cone[i];
                if self.faces[face as usize].mark == MARK_VISIBLE {
                    let distance = plane_separation(
                        self.faces[face as usize].plane,
                        self.vertices[vertex as usize].position,
                    );
                    if distance > max_distance {
                        max_distance = distance;
                        max_face = face;
                    }
                }
            }

            if max_face != NIL {
                b3_assert(self.faces[max_face as usize].mark == MARK_VISIBLE);
                self.link_vertex_back(CONFLICT_HEAD_BASE + max_face, vertex);
                self.vertices[vertex as usize].conflict_face = max_face;
                if max_distance > self.faces[max_face as usize].max_conflict_distance {
                    self.faces[max_face as usize].max_conflict_distance = max_distance;
                    self.faces[max_face as usize].max_conflict = vertex;
                }
            }
            // Otherwise: vertex is interior to the hull. Its slot in the bump pool is abandoned.
        }

        b3_assert(self.list_empty(HEAD_ORPHANED));
    }

    fn resolve_faces(&mut self) {
        // Splice deleted faces out of the face list. Faces already retired by absorbFaces are no
        // longer on faceList, so guard with listContains before removing.
        let mut node = self.heads[HEAD_FACE as usize].next;
        while node != HEAD {
            let face = node;
            node = self.faces[face as usize].next;

            if self.faces[face as usize].mark == MARK_DELETE
                && Self::face_list_contains(&self.faces[face as usize])
            {
                b3_assert(self.list_empty(CONFLICT_HEAD_BASE + face));

                // Each half-edge is owned by exactly one face, so ring walks over the dead region
                // retire every interior edge exactly once. Merge deleted faces are already off
                // the face list.
                let start = self.faces[face as usize].edge;
                let mut edge = start;
                loop {
                    let next = self.edges[edge as usize].next;
                    self.retire_edge(edge);
                    edge = next;
                    if edge == start {
                        break;
                    }
                }

                self.retire_face(face);
            }
        }

        for i in 0..self.cone.len() {
            let face = self.cone[i];
            if self.faces[face as usize].mark == MARK_DELETE {
                continue;
            }
            self.link_face_back(face);
        }
    }

    fn add_vertex_to_hull(&mut self, vertex: Node) {
        let face = self.vertices[vertex as usize].conflict_face;
        b3_assert(face != NIL);
        self.vertices[vertex as usize].conflict_face = NIL;
        self.unlink_vertex(vertex);
        self.link_vertex_back(HEAD_VERTEX, vertex);

        self.horizon.clear();
        self.build_horizon(vertex, face);
        b3_assert(self.horizon.len() >= 3);

        self.cone.clear();
        self.build_cone(vertex);
        b3_assert(self.cone.len() >= 3);

        self.merge_faces();
        self.resolve_vertices();
        self.resolve_faces();
    }

    fn clean_hull(&mut self, origin: Vec3) {
        let mut face_count: i32 = 0;
        let mut half_edge_count: i32 = 0;

        let mut face_node = self.heads[HEAD_FACE as usize].next;
        while face_node != HEAD {
            let face = face_node;
            let start = self.faces[face as usize].edge;
            let mut edge = start;

            loop {
                let origin_vertex = self.edges[edge as usize].origin;
                self.vertices[origin_vertex as usize].reachable = true;
                edge = self.edges[edge as usize].next;
                half_edge_count += 1;
                if edge == start {
                    break;
                }
            }

            let f = &mut self.faces[face as usize];
            f.plane.offset += dot(f.plane.normal, origin);
            f.centroid = f.centroid + origin;
            face_count += 1;

            face_node = self.faces[face as usize].next;
        }

        let mut vertex_count: i32 = 0;
        let mut node = self.heads[HEAD_VERTEX as usize].next;
        while node != HEAD {
            let vertex = node;
            node = self.vertices[vertex as usize].next;

            if !self.vertices[vertex as usize].reachable {
                self.unlink_vertex(vertex);
            } else {
                self.vertices[vertex as usize].position =
                    self.vertices[vertex as usize].position + origin;
                vertex_count += 1;
            }
        }

        self.interior_point = self.interior_point + origin;

        self.final_vertex_count = vertex_count;
        self.final_half_edge_count = half_edge_count;
        self.final_face_count = face_count;
    }

    fn has_hull(&self) -> bool {
        let v = self.final_vertex_count;
        let e = self.final_half_edge_count / 2;
        let f = self.final_face_count;
        v - e + f == 2 && f >= 4
    }
}

// MARK: - Construction

/// Build the entire hull. Returns true iff the result satisfies Euler's identity.
fn construct_hull(
    builder: &mut HullBuilder,
    points: &[Vec3],
    max_vertex_count: i32,
    origin: Vec3,
    shifted_points: &mut [Vec3],
) -> bool {
    let point_count = points.len() as i32;
    if point_count < 4 {
        return false;
    }

    for i in 0..points.len() {
        shifted_points[i] = points[i] - origin;
    }

    builder.compute_tolerance(shifted_points);
    let have_initial_hull = builder.build_initial_hull(shifted_points);
    if !have_initial_hull {
        return false;
    }

    let mut budget = clamp_int(max_vertex_count - 4, 0, HULL_MAX_COUNT - 4);

    let mut vertex = builder.next_conflict_vertex();
    while vertex != NIL && budget > 0 {
        builder.add_vertex_to_hull(vertex);
        vertex = builder.next_conflict_vertex();
        budget -= 1;
    }

    builder.clean_hull(origin);

    builder.has_hull()
}

/// Walk the builder lists into the final hull arrays, stamping indices.
fn emit_hull(b: &HullBuilder) -> Option<Hull> {

    // Walk lists into temp arrays, stamping finalIndex on each node so the resolution pass below
    // is O(E + F) instead of O(E^2 + F^2).
    let mut temp_vertices: Vec<Node> = Vec::with_capacity(HULL_MAX_COUNT as usize);
    let mut node = b.heads[HEAD_VERTEX as usize].next;
    while node != HEAD {
        let vertex = node;
        temp_vertices.push(vertex);
        node = b.vertices[vertex as usize].next;
    }
    // Stamping requires mutability; rebuild a lookup instead of touching the builder.
    let mut vertex_final = vec![b3::NULL_INDEX; b.vertices.len()];
    for (i, &v) in temp_vertices.iter().enumerate() {
        vertex_final[v as usize] = i as i32;
    }

    // Collect edges in twin-paired order (i, i+1) by stamping each pair as we discover it.
    let mut temp_faces: Vec<Node> = Vec::with_capacity(HULL_MAX_COUNT as usize);
    let mut temp_edges: Vec<Node> = Vec::with_capacity(HULL_MAX_COUNT as usize);
    let mut face_final = vec![b3::NULL_INDEX; b.faces.len()];
    let mut edge_final = vec![b3::NULL_INDEX; b.edges.len()];

    let mut face_node = b.heads[HEAD_FACE as usize].next;
    while face_node != HEAD {
        let face = face_node;
        face_final[face as usize] = temp_faces.len() as i32;
        temp_faces.push(face);

        let start = b.faces[face as usize].edge;
        let mut edge = start;
        loop {
            if edge_final[edge as usize] < 0 {
                edge_final[edge as usize] = temp_edges.len() as i32;
                temp_edges.push(edge);
                let twin = b.edges[edge as usize].twin;
                edge_final[twin as usize] = temp_edges.len() as i32;
                temp_edges.push(twin);
            }
            edge = b.edges[edge as usize].next;
            if edge == start {
                break;
            }
        }

        face_node = b.faces[face as usize].next;
    }

    let vertex_count = temp_vertices.len() as i32;
    let face_count = temp_faces.len() as i32;
    let edge_count = temp_edges.len() as i32;

    let mut points = Vec::with_capacity(temp_vertices.len());
    let mut vertices = Vec::with_capacity(temp_vertices.len());
    for &v in &temp_vertices {
        points.push(b.vertices[v as usize].position);
        vertices.push(HullVertex { edge: 0 });
    }

    let mut edges = Vec::with_capacity(temp_edges.len());
    for (index, &e) in temp_edges.iter().enumerate() {
        let edge = &b.edges[e as usize];
        let next = edge_final[edge.next as usize] as u8;
        let twin = edge_final[edge.twin as usize] as u8;
        let face = face_final[edge.face as usize] as u8;
        let origin = vertex_final[edge.origin as usize] as u8;
        edges.push(HullHalfEdge {
            next,
            twin,
            origin,
            face,
        });
        vertices[origin as usize].edge = index as u8;
    }

    let mut faces = Vec::with_capacity(temp_faces.len());
    let mut planes = Vec::with_capacity(temp_faces.len());
    for &f in &temp_faces {
        let face = &b.faces[f as usize];
        let edge = edge_final[face.edge as usize] as u8;
        faces.push(HullFace { edge });
        planes.push(face.plane);
    }

    let mut hull = Hull::allocate(vertex_count, edge_count, face_count)?;

    hull.points_mut().copy_from_slice(&points);
    hull.vertices_mut().copy_from_slice(&vertices);
    hull.edges_mut().copy_from_slice(&edges);
    hull.faces_mut().copy_from_slice(&faces);
    hull.planes_mut().copy_from_slice(&planes);

    let soa_vertex_count = hull_soa_stride(vertex_count) as usize;
    let soa_normal_count = hull_soa_stride(face_count) as usize;

    {
        let h = hull.header();
        let h_vertex_off = h.soa_vertex_offset as usize;
        let h_normal_off = h.soa_normal_offset as usize;
        unsafe {
            let vx = hull.storage().offset_ptr::<f32>(h_vertex_off);
            let vy = hull.storage().offset_ptr::<f32>(h_vertex_off + soa_vertex_count * 4);
            let vz = hull.storage().offset_ptr::<f32>(h_vertex_off + 2 * soa_vertex_count * 4);
            for i in 0..vertex_count as usize {
                *vx.add(i) = points[i].x;
                *vy.add(i) = points[i].y;
                *vz.add(i) = points[i].z;
            }
            for i in vertex_count as usize..soa_vertex_count {
                *vx.add(i) = *vx;
                *vy.add(i) = *vy;
                *vz.add(i) = *vz;
            }

            let nx = hull.storage().offset_ptr::<f32>(h_normal_off);
            let ny = hull.storage().offset_ptr::<f32>(h_normal_off + soa_normal_count * 4);
            let nz = hull.storage().offset_ptr::<f32>(h_normal_off + 2 * soa_normal_count * 4);
            for i in 0..face_count as usize {
                *nx.add(i) = planes[i].normal.x;
                *ny.add(i) = planes[i].normal.y;
                *nz.add(i) = planes[i].normal.z;
            }
            for i in face_count as usize..soa_normal_count {
                *nx.add(i) = 0.0;
                *ny.add(i) = 0.0;
                *nz.add(i) = 0.0;
            }
        }
    }

    update_hull_edge_cosines(&mut hull);

    update_hull_bounds(&mut hull);
    let success = update_hull_bulk_properties(&mut hull);
    if !success {
        return None;
    }

    if !is_valid_hull(&hull) {
        return None;
    }

    hull.compute_hash();
    Some(hull)
}

/// Create a convex hull from a point cloud.
pub fn create_hull(points: &[Vec3], max_vertex_count: i32) -> Option<Hull> {
    if points.len() < 4 {
        return None;
    }

    let point_count = points.len() as i32;
    let origin = points[0];
    let clamped_max_count = clamp_int(max_vertex_count, 4, b3::MAX_HULL_VERTICES);

    // Single allocation for all working memory (arenas are separate Vecs in this port).
    let sizes = HullWorkSizes::new(point_count, clamped_max_count);

    let mut builder = HullBuilder::new(&sizes);

    let mut shifted_points = vec![Vec3::ZERO; points.len()];

    let ok = construct_hull(
        &mut builder,
        points,
        clamped_max_count,
        origin,
        &mut shifted_points,
    );

    if !ok {
        return None;
    }

    if builder.final_vertex_count > b3::MAX_HULL_VERTICES {
        b3_log(&format!(
            "hull final vertex count of {} exceeds limit of {}",
            builder.final_vertex_count,
            b3::MAX_HULL_VERTICES
        ));
        return None;
    }

    if builder.final_face_count > b3::MAX_HULL_FACES {
        b3_log(&format!(
            "hull final face count of {} exceeds limit of {}",
            builder.final_face_count,
            b3::MAX_HULL_FACES
        ));
        return None;
    }

    let max_half_edge_count = 2 * b3::MAX_HULL_EDGES;
    if builder.final_half_edge_count > max_half_edge_count {
        b3_log(&format!(
            "hull final half edge count of {} exceeds limit of {}",
            builder.final_half_edge_count, max_half_edge_count
        ));
        return None;
    }

    emit_hull(&builder)
}

/// A capped cylinder hull. Two rings of `sides` points from `y_offset` to `y_offset + height`,
/// interleaved and advanced by an accumulated angle, exactly as Swift builds them.
pub fn create_cylinder(height: f32, radius: f32, y_offset: f32, sides: i32) -> Option<Hull> {
    b3_assert(height > 0.0);
    b3_assert(radius > 0.0);
    b3_assert(3 <= sides && sides <= 32);
    if !(height > 0.0 && radius > 0.0 && sides >= 3) {
        return None;
    }

    let point_count = 2 * sides;
    let mut points = vec![Vec3::ZERO; point_count as usize];

    let mut alpha: f32 = 0.0;
    let delta_alpha = 2.0 * b3::PI / sides as f32;

    for index in 0..sides as usize {
        let cs = compute_cos_sin(alpha);

        points[2 * index] = Vec3::new(radius * cs.cosine, y_offset, radius * cs.sine);
        points[2 * index + 1] =
            Vec3::new(radius * cs.cosine, y_offset + height, radius * cs.sine);

        alpha += delta_alpha;
    }

    create_hull(&points, point_count)
}

/// A capped cone hull.
pub fn create_cone(height: f32, radius1: f32, radius2: f32, slices: i32) -> Option<Hull> {
    b3_assert(height > 0.0);
    b3_assert(radius1 > 0.0);
    b3_assert(radius2 > 0.0);
    b3_assert(4 <= slices && slices <= 32);
    if !(height > 0.0 && radius1 > 0.0 && radius2 > 0.0 && slices >= 4) {
        return None;
    }

    let point_count = 2 * slices;
    let mut points = vec![Vec3::ZERO; point_count as usize];

    let mut alpha: f32 = 0.0;
    let delta_alpha = 2.0 * b3::PI / slices as f32;

    for index in 0..slices as usize {
        let cs = compute_cos_sin(alpha);

        points[2 * index] = Vec3::new(radius1 * cs.cosine, 0.0, radius1 * cs.sine);
        points[2 * index + 1] = Vec3::new(radius2 * cs.cosine, height, radius2 * cs.sine);

        alpha += delta_alpha;
    }

    create_hull(&points, point_count)
}

/// A deterministic irregular rock hull.
pub fn create_rock(radius: f32) -> Option<Hull> {
    // Swift's Fibonacci lattice: 10 points, golden-ratio azimuthal step, cosine/sine advanced by
    // a fixed rotation each iteration.
    let point_count = 10usize;

    // Golden ratio.
    let phi = (1.0 + 5.0f32.sqrt()) / 2.0;

    // Azimuthal angle.
    let theta = 2.0 * b3::PI / phi;

    let mut cs = CosSin::new(1.0, 0.0);
    let delta_cs = compute_cos_sin(theta);

    let mut points = Vec::with_capacity(point_count);
    for i in 0..point_count {
        // Z coordinate.
        let z = 1.0 - (2.0 * i as f32 + 1.0) / point_count as f32;
        // Radius in xy-plane.
        let radius_xy = (1.0 - z * z).sqrt();

        points.push(Vec3::new(
            radius * radius_xy * cs.cosine,
            radius * radius_xy * cs.sine,
            radius * z,
        ));

        let cs0 = cs;
        cs.cosine = delta_cs.cosine * cs0.cosine - delta_cs.sine * cs0.sine;
        cs.sine = delta_cs.sine * cs0.cosine + delta_cs.cosine * cs0.sine;
    }

    create_hull(&points, point_count as i32)
}

/// Swift's `nextComplexHullRandom(_:)`: a 32-bit LCG with the PEEL constants.
fn next_complex_hull_random(state: &mut u32) -> u32 {
    *state = state.wrapping_mul(2_147_001_325).wrapping_add(715_136_305);
    *state
}

/// Swift's `complexHullRandomFloat(_:)`.
fn complex_hull_random_float(state: &mut u32) -> f32 {
    ((next_complex_hull_random(state) & 0xffff) as f32) / 65535.0 - 0.5
}

/// Swift's `complexHullRandomDirection(_:)`: rejection sampling inside a radius-0.5 cube.
fn complex_hull_random_direction(state: &mut u32) -> Vec3 {
    let mut point = Vec3::ZERO;
    loop {
        point.x = complex_hull_random_float(state);
        point.y = complex_hull_random_float(state);
        point.z = complex_hull_random_float(state);
        if !(dot(point, point) > 0.25) {
            break;
        }
    }
    normalize(point)
}

/// Hull of 32 pseudo random points on a sphere. From the convex pile hull in PEEL.
pub fn create_complex_hull(radius: f32) -> Option<Hull> {
    let point_count = 32usize;

    let mut points = Vec::with_capacity(point_count);
    let mut state: u32 = 42;
    for _ in 0..point_count {
        let direction = complex_hull_random_direction(&mut state);
        points.push(radius * direction);
    }

    create_hull(&points, point_count as i32)
}

// MARK: - Clone and transform

/// Clone and transform a hull. Supports non-uniform and mirroring scale.
pub fn clone_and_transform_hull(original: &Hull, transform: Transform, scale: Vec3) -> Option<Hull> {
    if !is_valid_hull(original) {
        return None;
    }

    let mut hull = original.deep_clone()?;

    let safe_scale = safe_scale(scale);

    let face_count = hull.face_count() as usize;
    let vertex_count = hull.vertex_count() as usize;

    if safe_scale.x * safe_scale.y * safe_scale.z < 0.0 {
        let faces_snapshot: Vec<HullFace> = hull.faces().to_vec();
        {
            let edges = hull.edges_mut();
            // Reflected: reverse edge winding for each face.
            for face in faces_snapshot.iter().take(face_count) {
                let start_edge_index = face.edge as i32;
                let mut current_edge_index = start_edge_index;
                let mut prev_edge_index: i32 = b3::NULL_INDEX;

                loop {
                    let edge = &edges[current_edge_index as usize];
                    if edge.next as i32 == start_edge_index {
                        prev_edge_index = current_edge_index;
                        break;
                    }
                    current_edge_index = edge.next as i32;
                    if current_edge_index == start_edge_index {
                        break;
                    }
                }

                b3_assert(prev_edge_index != b3::NULL_INDEX);

                current_edge_index = start_edge_index;
                loop {
                    let edge_next = edges[current_edge_index as usize].next as i32;
                    edges[current_edge_index as usize].next = prev_edge_index as u8;

                    let twin = edges[current_edge_index as usize].twin as i32;
                    if current_edge_index < twin {
                        let twin_index = twin as usize;
                        let origin = edges[current_edge_index as usize].origin;
                        edges[current_edge_index as usize].origin = edges[twin_index].origin;
                        edges[twin_index].origin = origin;
                    }

                    prev_edge_index = current_edge_index;
                    current_edge_index = edge_next;
                    if current_edge_index == start_edge_index {
                        break;
                    }
                }
            }
        }

        let edge_twins: Vec<u8> = hull.edges().iter().map(|e| e.twin).collect();
        {
            let vertices = hull.vertices_mut();
            for vertex in vertices.iter_mut().take(vertex_count) {
                let edge = vertex.edge as usize;
                vertex.edge = edge_twins[edge];
            }
        }
    }

    let matrix = make_matrix_from_quat(transform.q);

    {
        let points = hull.points_mut();
        for p in points.iter_mut().take(vertex_count) {
            let scaled = Vec3::new(
                safe_scale.x * p.x,
                safe_scale.y * p.y,
                safe_scale.z * p.z,
            );
            *p = mul_mv(matrix, scaled) + transform.p;
        }
    }

    let final_points: Vec<Vec3> = hull.points().to_vec();
    let soa_vertex_count = hull_soa_stride(vertex_count as i32) as usize;

    {
        let h = hull.header();
        let h_vertex_off = h.soa_vertex_offset as usize;
        let h_normal_off = h.soa_normal_offset as usize;
        unsafe {
            let vx = hull.storage().offset_ptr::<f32>(h_vertex_off);
            let vy = hull.storage().offset_ptr::<f32>(h_vertex_off + soa_vertex_count * 4);
            let vz = hull.storage().offset_ptr::<f32>(h_vertex_off + 2 * soa_vertex_count * 4);
            for i in 0..vertex_count {
                *vx.add(i) = final_points[i].x;
                *vy.add(i) = final_points[i].y;
                *vz.add(i) = final_points[i].z;
            }
            for i in vertex_count..soa_vertex_count {
                *vx.add(i) = *vx;
                *vy.add(i) = *vy;
                *vz.add(i) = *vz;
            }

            let edges_snapshot: Vec<HullHalfEdge> = hull.edges().to_vec();
            let faces_snapshot: Vec<HullFace> = hull.faces().to_vec();

            let mut planes = Vec::with_capacity(face_count);
            let mut normals = Vec::with_capacity(face_count);
            for face in faces_snapshot.iter().take(face_count) {
                let mut count = 0.0f32;
                let mut centroid = Vec3::ZERO;
                let mut normal = Vec3::ZERO;

                let start_edge_index = face.edge as usize;
                let mut current_edge_index = start_edge_index;

                b3_assert(edges_snapshot[start_edge_index].face as usize == planes.len());
                b3_assert((edges_snapshot[start_edge_index].origin as usize) < vertex_count);

                let origin = final_points[edges_snapshot[start_edge_index].origin as usize];

                loop {
                    let edge = &edges_snapshot[current_edge_index];
                    let twin = &edges_snapshot[edge.twin as usize];
                    b3_assert(twin.twin as usize == current_edge_index);

                    let v1 = final_points[edge.origin as usize] - origin;
                    let v2 = final_points[twin.origin as usize] - origin;

                    count += 1.0;
                    centroid = centroid + v1;
                    normal.x += (v1.y - v2.y) * (v1.z + v2.z);
                    normal.y += (v1.z - v2.z) * (v1.x + v2.x);
                    normal.z += (v1.x - v2.x) * (v1.y + v2.y);

                    current_edge_index = edge.next as usize;
                    if current_edge_index == start_edge_index {
                        break;
                    }
                }

                b3_assert(count > 0.0);
                centroid = (1.0 / count) * centroid;
                centroid = centroid + origin;

                let area = length(normal);
                b3_assert(area > 0.0);
                normal = (1.0 / area) * normal;

                planes.push(make_plane_from_normal_and_point(normal, centroid));
                normals.push(normal);
            }

            hull.planes_mut().copy_from_slice(&planes);

            let soa_normal_count = hull_soa_stride(face_count as i32) as usize;
            let nx = hull.storage().offset_ptr::<f32>(h_normal_off);
            let ny = hull.storage().offset_ptr::<f32>(h_normal_off + soa_normal_count * 4);
            let nz = hull.storage().offset_ptr::<f32>(h_normal_off + 2 * soa_normal_count * 4);
            for i in 0..face_count {
                *nx.add(i) = normals[i].x;
                *ny.add(i) = normals[i].y;
                *nz.add(i) = normals[i].z;
            }
            for i in face_count..soa_normal_count {
                *nx.add(i) = 0.0;
                *ny.add(i) = 0.0;
                *nz.add(i) = 0.0;
            }
        }
    }

    update_hull_edge_cosines(&mut hull);

    update_hull_bounds(&mut hull);
    let success = update_hull_bulk_properties(&mut hull);
    if !success {
        return None;
    }

    // Must ensure the hash is 0 so it doesn't contribute to itself.
    hull.compute_hash();

    b3_validate(is_valid_hull(&hull));

    Some(hull)
}
