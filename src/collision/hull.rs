//! Convex hull storage, properties, and box hull templates.

use core::ptr;

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::base::hash::hash64_non_zero;
use crate::collision::geometry_queries::box_inertia;
use crate::collision::geometry_storage::GeometryStorage;
use crate::collision::internal_math::*;
use crate::math::functions::*;
use crate::math::types::*;
use crate::types::geometry::{HullData, HullFace, HullHalfEdge, HullVertex};

/// A convex hull.
///
/// The hull owns a zeroed allocation holding the [`HullData`] header followed by the vertex,
/// point, edge, face, structure-of-arrays, and edge-cosine arrays.
#[derive(Clone)]
pub struct Hull {
    storage: GeometryStorage,
}

impl core::fmt::Debug for Hull {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Hull")
            .field("vertex_count", &self.vertex_count())
            .field("face_count", &self.face_count())
            .field("hash", &self.hash())
            .finish()
    }
}

impl Hull {
    pub fn from_storage(storage: GeometryStorage) -> Self {
        Self { storage }
    }

    pub fn storage(&self) -> &GeometryStorage {
        &self.storage
    }

    pub fn byte_count(&self) -> i32 {
        self.storage.byte_count() as i32
    }

    fn header_ptr(&self) -> *mut HullData {
        unsafe { self.storage.as_ptr::<HullData>() }
    }

    pub fn header(&self) -> &HullData {
        unsafe { &*self.header_ptr() }
    }

    pub fn header_mut(&mut self) -> &mut HullData {
        unsafe { &mut *self.header_ptr() }
    }

    pub fn vertex_count(&self) -> i32 {
        self.header().vertex_count
    }

    pub fn half_edge_count(&self) -> i32 {
        self.header().edge_count
    }

    pub fn face_count(&self) -> i32 {
        self.header().face_count
    }

    pub fn aabb(&self) -> AABB {
        self.header().aabb
    }

    pub fn surface_area(&self) -> f32 {
        self.header().surface_area
    }

    pub fn volume(&self) -> f32 {
        self.header().volume
    }

    pub fn inner_radius(&self) -> f32 {
        self.header().inner_radius
    }

    pub fn center(&self) -> Vec3 {
        self.header().center
    }

    pub fn central_inertia(&self) -> Matrix3 {
        self.header().central_inertia
    }

    pub fn hash(&self) -> u64 {
        self.header().hash
    }

    pub fn version(&self) -> u64 {
        self.header().version
    }

    unsafe fn array<T>(&self, offset: i32, count: i32) -> &'static mut [T] {
        let ptr = self.storage.offset_ptr::<T>(offset as usize);
        std::slice::from_raw_parts_mut(ptr, count as usize)
    }

    pub fn vertices(&self) -> &[HullVertex] {
        let h = self.header();
        unsafe { self.array::<HullVertex>(h.vertex_offset, h.vertex_count) }
    }

    pub fn vertices_mut(&mut self) -> &mut [HullVertex] {
        let h = self.header();
        unsafe { self.array::<HullVertex>(h.vertex_offset, h.vertex_count) }
    }

    pub fn points(&self) -> &[Vec3] {
        let h = self.header();
        unsafe { self.array::<Vec3>(h.point_offset, h.vertex_count) }
    }

    pub fn points_mut(&mut self) -> &mut [Vec3] {
        let h = self.header();
        unsafe { self.array::<Vec3>(h.point_offset, h.vertex_count) }
    }

    pub fn edges(&self) -> &[HullHalfEdge] {
        let h = self.header();
        unsafe { self.array::<HullHalfEdge>(h.edge_offset, h.edge_count) }
    }

    pub fn edges_mut(&mut self) -> &mut [HullHalfEdge] {
        let h = self.header();
        unsafe { self.array::<HullHalfEdge>(h.edge_offset, h.edge_count) }
    }

    pub fn planes(&self) -> &[Plane] {
        let h = self.header();
        unsafe { self.array::<Plane>(h.plane_offset, h.face_count) }
    }

    pub fn planes_mut(&mut self) -> &mut [Plane] {
        let h = self.header();
        unsafe { self.array::<Plane>(h.plane_offset, h.face_count) }
    }

    pub fn faces(&self) -> &[HullFace] {
        let h = self.header();
        unsafe { self.array::<HullFace>(h.face_offset, h.face_count) }
    }

    pub fn faces_mut(&mut self) -> &mut [HullFace] {
        let h = self.header();
        unsafe { self.array::<HullFace>(h.face_offset, h.face_count) }
    }

    pub fn edge_cosines(&self) -> &[f32] {
        let h = self.header();
        unsafe { self.array::<f32>(h.edge_cosine_offset, h.edge_count / 2) }
    }

    fn soa_len(&self) -> (usize, usize) {
        (
            hull_soa_stride(self.vertex_count()) as usize,
            hull_soa_stride(self.face_count()) as usize,
        )
    }

    /// Structure-of-arrays vertex x/y/z.
    pub fn soa_vertices(&self) -> (&[f32], &[f32], &[f32]) {
        let (n, _) = self.soa_len();
        let h = self.header();
        unsafe {
            let vx = std::slice::from_raw_parts(self.storage.offset_ptr::<f32>(h.soa_vertex_offset as usize), n);
            let vy = std::slice::from_raw_parts(self.storage.offset_ptr::<f32>(h.soa_vertex_offset as usize + n * 4), n);
            let vz = std::slice::from_raw_parts(self.storage.offset_ptr::<f32>(h.soa_vertex_offset as usize + 2 * n * 4), n);
            (vx, vy, vz)
        }
    }

    /// Structure-of-arrays face normal x/y/z.
    pub fn soa_normals(&self) -> (&[f32], &[f32], &[f32]) {
        let (_, n) = self.soa_len();
        let h = self.header();
        unsafe {
            let nx = std::slice::from_raw_parts(self.storage.offset_ptr::<f32>(h.soa_normal_offset as usize), n);
            let ny = std::slice::from_raw_parts(self.storage.offset_ptr::<f32>(h.soa_normal_offset as usize + n * 4), n);
            let nz = std::slice::from_raw_parts(self.storage.offset_ptr::<f32>(h.soa_normal_offset as usize + 2 * n * 4), n);
            (nx, ny, nz)
        }
    }

    pub fn vertex_positions(&self) -> Vec<Vec3> {
        self.points().to_vec()
    }
}

/// The stride of the structure-of-arrays arrays.
#[inline]
pub fn hull_soa_stride(count: i32) -> i32 {
    (count + 7) & !7
}

/// Byte offsets for a hull allocation and its trailing arrays.
#[derive(Clone, Copy, Debug, Default)]
pub struct HullLayout {
    pub byte_count: usize,
    pub vertex_offset: usize,
    pub point_offset: usize,
    pub edge_offset: usize,
    pub plane_offset: usize,
    pub face_offset: usize,
    pub soa_vertex_offset: usize,
    pub soa_normal_offset: usize,
    pub edge_cosine_offset: usize,
}

impl HullLayout {
    pub fn new(vertex_count: i32, edge_count: i32, face_count: i32) -> Self {
        let soa_vertex_count = hull_soa_stride(vertex_count);
        let soa_normal_count = hull_soa_stride(face_count);

        let mut layout = Self::default();
        let mut n = align_up8(std::mem::size_of::<HullData>());

        layout.vertex_offset = n;
        n += align_up8(vertex_count as usize * std::mem::size_of::<HullVertex>());

        layout.point_offset = n;
        n += align_up8(vertex_count as usize * std::mem::size_of::<Vec3>());

        layout.edge_offset = n;
        n += align_up8(edge_count as usize * std::mem::size_of::<HullHalfEdge>());

        layout.plane_offset = n;
        n += align_up8(face_count as usize * std::mem::size_of::<Plane>());

        layout.face_offset = n;
        n += align_up8(face_count as usize * std::mem::size_of::<HullFace>());

        layout.soa_vertex_offset = n;
        n += align_up8(3 * soa_vertex_count as usize * std::mem::size_of::<f32>());

        layout.soa_normal_offset = n;
        n += align_up8(3 * soa_normal_count as usize * std::mem::size_of::<f32>());

        layout.edge_cosine_offset = n;
        n += align_up8(edge_count as usize / 2 * std::mem::size_of::<f32>());

        layout.byte_count = n;
        layout
    }
}

impl Hull {
    /// Allocate a zeroed hull for the given counts and fill in the header offsets.
    pub fn allocate(vertex_count: i32, edge_count: i32, face_count: i32) -> Option<Hull> {
        let layout = HullLayout::new(vertex_count, edge_count, face_count);
        let storage = GeometryStorage::new(layout.byte_count)?;
        let mut hull = Hull::from_storage(storage);

        {
            let h = hull.header_mut();
            h.version = b3::HULL_VERSION;
            h.vertex_offset = layout.vertex_offset as i32;
            h.point_offset = layout.point_offset as i32;
            h.edge_offset = layout.edge_offset as i32;
            h.plane_offset = layout.plane_offset as i32;
            h.face_offset = layout.face_offset as i32;
            h.soa_vertex_offset = layout.soa_vertex_offset as i32;
            h.soa_normal_offset = layout.soa_normal_offset as i32;
            h.edge_cosine_offset = layout.edge_cosine_offset as i32;
            h.vertex_count = vertex_count;
            h.edge_count = edge_count;
            h.face_count = face_count;
            h.byte_count = layout.byte_count as i32;
        }

        Some(hull)
    }

    /// Clone the raw bytes of this hull into a new hull.
    pub fn deep_clone(&self) -> Option<Hull> {
        let layout = self.storage.byte_count();
        let new_storage = GeometryStorage::new(layout)?;
        unsafe {
            ptr::copy_nonoverlapping(self.storage.base(), new_storage.base(), layout);
        }
        Some(Hull::from_storage(new_storage))
    }

    /// Recompute the content hash.
    pub fn compute_hash(&mut self) {
        self.header_mut().hash = 0;
        let bytes = self.storage.byte_count();
        // The hash covers the whole storage, header included, so the zeroed `hash` field is part of
        // the input; read it before taking the mutable header borrow that writes the result.
        let hash = unsafe {
            hash64_non_zero(std::slice::from_raw_parts(self.storage.base(), bytes))
        };
        self.header_mut().hash = hash;
    }
}

// MARK: - Properties

/// Recompute the dot product of the two adjacent face normals for each full edge.
pub fn update_hull_edge_cosines(hull: &mut Hull) {
    let edge_count = hull.half_edge_count() as usize;
    let edges = hull.edges();
    let planes = hull.planes();
    let cosines = unsafe {
        let h = hull.header();
        let ptr = hull.storage().offset_ptr::<f32>(h.edge_cosine_offset as usize);
        std::slice::from_raw_parts_mut(ptr, edge_count / 2)
    };

    for i in 0..edge_count / 2 {
        cosines[i] = dot(planes[edges[2 * i].face as usize].normal, planes[edges[2 * i + 1].face as usize].normal);
    }
}

/// Recompute the local AABB from the hull points.
pub fn update_hull_bounds(hull: &mut Hull) {
    let points = hull.points();
    b3_assert(!points.is_empty());
    let mut bounds = AABB::new(points[0], points[0]);
    for p in &points[1..] {
        bounds.lower_bound = min_vec3(bounds.lower_bound, *p);
        bounds.upper_bound = max_vec3(bounds.upper_bound, *p);
    }
    hull.header_mut().aabb = bounds;
}

/// Compute the volume, centroid, surface area, inner radius, and inertia tensor.
pub fn update_hull_bulk_properties(hull: &mut Hull) -> bool {
    let face_count = hull.face_count();
    let points = hull.points();
    let edges = hull.edges();
    let faces = hull.faces();
    let planes = hull.planes();

    if points.is_empty() || faces.is_empty() || edges.is_empty() {
        return false;
    }

    let mut area: f32 = 0.0;
    let mut volume: f32 = 0.0;
    let mut center = Vec3::ZERO;

    let origin = points[0];
    let (mut xx, mut xy, mut yy, mut xz, mut zz, mut yz) = (0.0f32, 0.0, 0.0, 0.0, 0.0, 0.0);

    for face_index in 0..face_count as usize {
        let edge1 = faces[face_index].edge as usize;
        let mut edge2 = edges[edge1].next as usize;
        let mut edge3 = edges[edge2].next as usize;

        let v1 = points[edges[edge1].origin as usize] - origin;

        while edge3 != edge1 {
            let v2 = points[edges[edge2].origin as usize] - origin;
            let v3 = points[edges[edge3].origin as usize] - origin;

            area += length(cross(v2 - v1, v3 - v1));
            let det = scalar_triple_product(v1, v2, v3);
            volume += det;

            let v4 = v1 + (v2 + v3);
            center += det * v4;

            xx += det * (v1.x * v1.x + v2.x * v2.x + v3.x * v3.x + v4.x * v4.x);
            yy += det * (v1.y * v1.y + v2.y * v2.y + v3.y * v3.y + v4.y * v4.y);
            zz += det * (v1.z * v1.z + v2.z * v2.z + v3.z * v3.z + v4.z * v4.z);
            xy += det * (v1.x * v1.y + v2.x * v2.y + v3.x * v3.y + v4.x * v4.y);
            xz += det * (v1.x * v1.z + v2.x * v2.z + v3.x * v3.z + v4.x * v4.z);
            yz += det * (v1.y * v1.z + v2.y * v2.z + v3.y * v3.z + v4.y * v4.z);

            edge2 = edge3;
            edge3 = edges[edge3].next as usize;
        }
    }

    b3_validate(volume > 0.0);

    let local_center = if volume > 0.0 { (0.25 / volume) * center } else { Vec3::ZERO };
    center = local_center + origin;

    let mut radius = f32::MAX;
    for plane in planes.iter() {
        let distance = plane_separation(*plane, center);
        b3_validate(distance < 0.0);
        radius = min_float(radius, -distance);
    }

    let mut inertia = Matrix3::default();
    inertia.cx.x = yy + zz;
    inertia.cy.x = -xy;
    inertia.cz.x = -xz;
    inertia.cx.y = -xy;
    inertia.cy.y = xx + zz;
    inertia.cz.y = -yz;
    inertia.cx.z = -xz;
    inertia.cy.z = -yz;
    inertia.cz.z = xx + yy;

    let mass = volume / 6.0;
    let central_inertia = sub_mm(mul_sm(1.0 / 120.0, inertia), steiner(mass, local_center));

    {
        let h = hull.header_mut();
        h.center = center;
        h.central_inertia = central_inertia;
        h.volume = mass;
        h.surface_area = 0.5 * area;
        h.inner_radius = radius;
    }

    mass > 0.0 && volume > 0.0 && area > 0.0 && radius > 0.0
}

/// Verify the hull topology and bulk properties.
pub fn is_valid_hull(hull: &Hull) -> bool {
    if hull.version() != b3::HULL_VERSION {
        return false;
    }
    let v = hull.vertex_count() as usize;
    let e = hull.half_edge_count() as usize / 2;
    let f = hull.face_count() as usize;

    if v < 4 || e < 6 || f < 4 {
        return false;
    }
    if v as i64 - e as i64 + f as i64 != 2 {
        return false;
    }

    let edges = hull.edges();
    for (index, vertex) in hull.vertices().iter().enumerate() {
        if edges[vertex.edge as usize].origin as usize != index {
            return false;
        }
    }

    if hull.half_edge_count() % 2 != 0 {
        return false;
    }
    let mut index = 0;
    while index < hull.half_edge_count() as usize {
        if edges[index].twin as usize != index + 1 || edges[index + 1].twin as usize != index {
            return false;
        }
        index += 2;
    }

    let faces = hull.faces();
    let planes = hull.planes();
    for face_index in 0..f {
        let base = faces[face_index].edge as usize;
        if plane_separation(planes[face_index], hull.center()) >= 0.0 {
            return false;
        }
        let mut edge_index = base;
        loop {
            let edge = edges[edge_index];
            let next = edges[edge.next as usize];
            let twin = edges[edge.twin as usize];
            if edge.face as usize != face_index || twin.twin as usize != edge_index {
                return false;
            }
            if next.origin != twin.origin {
                return false;
            }
            edge_index = edge.next as usize;
            if edge_index == base {
                break;
            }
        }
    }

    hull.volume() > 0.0 && hull.surface_area() > 0.0 && hull.inner_radius() > 0.0
}

// MARK: - Support queries

/// Find the vertex that is farthest in the given direction.
pub fn find_hull_support_vertex(hull: &Hull, direction: Vec3) -> i32 {
    let (vx, vy, vz) = hull.soa_vertices();
    let mut best_index = -1;
    let mut best_dot = -f32::MAX;
    for i in 0..hull.vertex_count() as usize {
        let d = direction.x * vx[i] + direction.y * vy[i] + direction.z * vz[i];
        if d > best_dot {
            best_dot = d;
            best_index = i as i32;
        }
    }
    b3_assert(best_index >= 0);
    best_index
}

/// Find the face whose normal is most aligned with the given direction.
pub fn find_hull_support_face(hull: &Hull, direction: Vec3) -> i32 {
    let planes = hull.planes();
    let mut best_index = -1;
    let mut best_dot = -f32::MAX;
    for (i, plane) in planes.iter().enumerate() {
        let d = dot(plane.normal, direction);
        if d > best_dot {
            best_dot = d;
            best_index = i as i32;
        }
    }
    b3_assert(best_index >= 0);
    best_index
}

// MARK: - Building from faces

/// Build a hull from points and outward-wound face polygons. Each face is a list of point indices.
pub fn hull_from_faces(points: &[Vec3], face_polygons: &[Vec<u32>]) -> Option<Hull> {
    if points.len() < 4 || face_polygons.len() < 4 {
        return None;
    }
    if points.len() > b3::MAX_HULL_VERTICES as usize || face_polygons.len() > b3::MAX_HULL_FACES as usize {
        return None;
    }

    let vertex_count = points.len() as i32;

    // Emit one directed edge per face side, remembering which face and position it came from.
    // Each `poly[i] -> poly[i+1]` edge is later paired with the one running the other way.
    struct Slot {
        origin: u8,
        face: u8,
        /// Index into `polygons` of the polygon this edge belongs to.
        polygon: usize,
        /// Position of this edge within its polygon.
        position: usize,
    }

    let mut slots: Vec<Slot> = Vec::new();
    let mut polygons: Vec<&[u32]> = Vec::new();

    for poly in face_polygons {
        if poly.len() < 3 {
            return None;
        }
        let polygon = polygons.len();
        polygons.push(poly.as_slice());
        for (k, &origin) in poly.iter().enumerate() {
            slots.push(Slot {
                origin: origin as u8,
                face: polygon as u8,
                polygon,
                position: k,
            });
        }
    }

    if slots.len() > u8::MAX as usize {
        return None;
    }

    // Pair every directed edge with the reversed one. Hull topology requires each twin pair to
    // occupy adjacent slots `(2k, 2k + 1)` with `twin == index ^ 1`; `is_valid_hull`, the manifold
    // feature queries and `hull_builder` all rely on that invariant.
    use std::collections::HashMap;
    let mut partner: HashMap<(u8, u8), usize> = HashMap::new();
    for (index, slot) in slots.iter().enumerate() {
        let poly = polygons[slot.polygon];
        let dest = poly[(slot.position + 1) % poly.len()] as u8;
        if partner.insert((slot.origin, dest), index).is_some() {
            // A directed edge appears twice: the face winding is not manifold.
            return None;
        }
    }

    let mut twin_of: Vec<usize> = vec![usize::MAX; slots.len()];
    for (index, slot) in slots.iter().enumerate() {
        let poly = polygons[slot.polygon];
        let dest = poly[(slot.position + 1) % poly.len()] as u8;
        match partner.get(&(dest, slot.origin)) {
            Some(&twin) => {
                twin_of[index] = twin;
                twin_of[twin] = index;
            }
            None => return None,
        }
    }

    // Renumber so that twins land in adjacent pairs.
    let mut order: Vec<usize> = Vec::with_capacity(slots.len());
    let mut placed = vec![false; slots.len()];
    for index in 0..slots.len() {
        if placed[index] {
            continue;
        }
        let twin = twin_of[index];
        placed[index] = true;
        placed[twin] = true;
        order.push(index);
        order.push(twin);
    }
    let mut new_index = vec![0usize; slots.len()];
    for (position, &old) in order.iter().enumerate() {
        new_index[old] = position;
    }

    let mut half_edges: Vec<HullHalfEdge> = vec![
        HullHalfEdge {
            next: 0,
            twin: 0,
            origin: 0,
            face: 0,
        };
        slots.len()
    ];
    for (position, &old) in order.iter().enumerate() {
        let slot = &slots[old];
        half_edges[position] = HullHalfEdge {
            next: 0,
            twin: (position ^ 1) as u8,
            origin: slot.origin,
            face: slot.face,
        };
    }
    // `next` follows the polygon winding, which the renumbering above scrambles; resolve it
    // through a (polygon, position) -> new index table.
    let mut slot_of: Vec<usize> = vec![0; slots.len()];
    for (position, &old) in order.iter().enumerate() {
        slot_of[old] = position;
    }
    for (position, &old) in order.iter().enumerate() {
        let slot = &slots[old];
        let len = polygons[slot.polygon].len();
        let next_position = (slot.position + 1) % len;
        // The slot holding the next side of the same polygon.
        let next_old = (0..slots.len())
            .find(|&candidate| {
                slots[candidate].polygon == slot.polygon && slots[candidate].position == next_position
            })
            .expect("every polygon side has a slot");
        half_edges[position].next = slot_of[next_old] as u8;
    }

    let face_count = face_polygons.len() as i32;
    let edge_count = half_edges.len() as i32;

    let hull = Hull::allocate(vertex_count, edge_count, face_count)?;

    let mut hull = hull;
    hull.points_mut().copy_from_slice(points);

    {
        let mut first_edge = vec![0u8; points.len()];
        for (i, e) in half_edges.iter().enumerate() {
            let o = e.origin as usize;
            if first_edge[o] == 0 {
                first_edge[o] = i as u8;
            }
        }
        // Ensure every vertex has an outgoing edge.
        for (v, edge) in first_edge.iter().enumerate() {
            let mut found = false;
            for e in half_edges.iter() {
                if e.origin as usize == v {
                    found = true;
                    break;
                }
            }
            if !found {
                return None;
            }
            let _ = edge;
        }

        let vertices = hull.vertices_mut();
        for (i, edge) in first_edge.iter().enumerate() {
            vertices[i].edge = *edge;
        }
        hull.edges_mut().copy_from_slice(&half_edges);

        let faces = hull.faces_mut();
        // Each face starts at its lowest-numbered side after the twin-adjacency renumbering.
        for i in 0..face_polygons.len() {
            let first = (0..slots.len())
                .filter(|&candidate| slots[candidate].polygon == i)
                .map(|candidate| slot_of[candidate])
                .min()
                .expect("every face has sides");
            faces[i].edge = first as u8;
        }

        let mut planes = Vec::with_capacity(face_polygons.len());
        for poly in face_polygons {
            let p0 = points[poly[0] as usize];
            let p1 = points[poly[1] as usize];
            let p2 = points[poly[2] as usize];
            planes.push(make_plane_from_points(p0, p1, p2));
        }
        hull.planes_mut().copy_from_slice(&planes);

        let (n_v, n_f) = (hull_soa_stride(vertex_count) as usize, hull_soa_stride(face_count) as usize);
        let h_vertex_off = hull.header().soa_vertex_offset as usize;
        let h_normal_off = hull.header().soa_normal_offset as usize;
        unsafe {
            let vx = hull.storage().offset_ptr::<f32>(h_vertex_off);
            let vy = hull.storage().offset_ptr::<f32>(h_vertex_off + n_v * 4);
            let vz = hull.storage().offset_ptr::<f32>(h_vertex_off + 2 * n_v * 4);
            for i in 0..points.len() {
                *vx.add(i) = points[i].x;
                *vy.add(i) = points[i].y;
                *vz.add(i) = points[i].z;
            }
            let nx = hull.storage().offset_ptr::<f32>(h_normal_off);
            let ny = hull.storage().offset_ptr::<f32>(h_normal_off + n_f * 4);
            let nz = hull.storage().offset_ptr::<f32>(h_normal_off + 2 * n_f * 4);
            for i in 0..planes.len() {
                *nx.add(i) = planes[i].normal.x;
                *ny.add(i) = planes[i].normal.y;
                *nz.add(i) = planes[i].normal.z;
            }
        }
    }

    update_hull_bounds(&mut hull);
    if !update_hull_bulk_properties(&mut hull) {
        return None;
    }
    update_hull_edge_cosines(&mut hull);
    hull.compute_hash();
    Some(hull)
}

// MARK: - Box hulls

const BOX_VERTEX_EDGES: [u8; 8] = [8, 1, 0, 9, 13, 3, 5, 11];

// Each entry is (next, twin, origin, face).
const BOX_EDGE_DATA: [(u8, u8, u8, u8); 24] = [
    (2, 1, 2, 0), (17, 0, 1, 5), (4, 3, 1, 0), (20, 2, 5, 3),
    (6, 5, 5, 0), (23, 4, 6, 4), (0, 7, 6, 0), (18, 6, 2, 2),
    (10, 9, 0, 1), (21, 8, 3, 5), (12, 11, 3, 1), (16, 10, 7, 2),
    (14, 13, 7, 1), (19, 12, 4, 4), (8, 15, 4, 1), (22, 14, 0, 3),
    (7, 17, 3, 2), (9, 16, 2, 5), (11, 19, 6, 2), (5, 18, 7, 4),
    (15, 21, 1, 3), (1, 20, 0, 5), (3, 23, 4, 3), (13, 22, 5, 4),
];

const BOX_FACE_EDGES: [u8; 6] = [0, 8, 16, 20, 19, 21];

/// Make a transformed box as a hull.
pub fn make_transformed_box_hull(hx: f32, hy: f32, hz: f32, transform: Transform) -> Hull {
    b3_assert(is_valid_transform(transform));

    let min_h = 0.2 * b3::linear_slop();
    let h = max_vec3(Vec3::new(min_h, min_h, min_h), Vec3::new(hx, hy, hz));

    let mut hull = Hull::allocate(8, 24, 6).expect("failed to allocate box hull");

    {
        let vertices = hull.vertices_mut();
        for i in 0..8 {
            vertices[i].edge = BOX_VERTEX_EDGES[i];
        }
        let edges = hull.edges_mut();
        for i in 0..24 {
            let e = BOX_EDGE_DATA[i];
            edges[i] = HullHalfEdge {
                next: e.0,
                twin: e.1,
                origin: e.2,
                face: e.3,
            };
        }
        let faces = hull.faces_mut();
        for i in 0..6 {
            faces[i].edge = BOX_FACE_EDGES[i];
        }

        let aabb = aabb_transform(transform, AABB::new(-h, h));
        let surface_area = 8.0 * (h.x * h.y + h.x * h.z + h.y * h.z);
        let volume = 8.0 * h.x * h.y * h.z;
        let inner_radius = min_float(h.x, min_float(h.y, h.z));
        let central_inertia = rotate_inertia(transform.q, box_inertia(volume, -h, h));
        let lower = -h;
        let upper = h;

        let planes = [
            transform_plane(transform, make_plane_from_normal_and_point(-Vec3::AXIS_X, lower)),
            transform_plane(transform, make_plane_from_normal_and_point(Vec3::AXIS_X, upper)),
            transform_plane(transform, make_plane_from_normal_and_point(-Vec3::AXIS_Y, lower)),
            transform_plane(transform, make_plane_from_normal_and_point(Vec3::AXIS_Y, upper)),
            transform_plane(transform, make_plane_from_normal_and_point(-Vec3::AXIS_Z, lower)),
            transform_plane(transform, make_plane_from_normal_and_point(Vec3::AXIS_Z, upper)),
        ];
        hull.planes_mut().copy_from_slice(&planes);

        let points = [
            transform_point(transform, Vec3::new(h.x, h.y, h.z)),
            transform_point(transform, Vec3::new(-h.x, h.y, h.z)),
            transform_point(transform, Vec3::new(-h.x, -h.y, h.z)),
            transform_point(transform, Vec3::new(h.x, -h.y, h.z)),
            transform_point(transform, Vec3::new(h.x, h.y, -h.z)),
            transform_point(transform, Vec3::new(-h.x, h.y, -h.z)),
            transform_point(transform, Vec3::new(-h.x, -h.y, -h.z)),
            transform_point(transform, Vec3::new(h.x, -h.y, -h.z)),
        ];
        hull.points_mut().copy_from_slice(&points);

        {
            let hd = hull.header_mut();
            hd.aabb = aabb;
            hd.surface_area = surface_area;
            hd.volume = volume;
            hd.inner_radius = inner_radius;
            hd.center = transform.p;
            hd.central_inertia = central_inertia;
        }

        let (n_v, n_f) = (hull_soa_stride(8) as usize, hull_soa_stride(6) as usize);
        let hv = hull.header().soa_vertex_offset as usize;
        let hn = hull.header().soa_normal_offset as usize;
        unsafe {
            let vx = hull.storage().offset_ptr::<f32>(hv);
            let vy = hull.storage().offset_ptr::<f32>(hv + n_v * 4);
            let vz = hull.storage().offset_ptr::<f32>(hv + 2 * n_v * 4);
            for i in 0..8 {
                *vx.add(i) = points[i].x;
                *vy.add(i) = points[i].y;
                *vz.add(i) = points[i].z;
            }
            let nx = hull.storage().offset_ptr::<f32>(hn);
            let ny = hull.storage().offset_ptr::<f32>(hn + n_f * 4);
            let nz = hull.storage().offset_ptr::<f32>(hn + 2 * n_f * 4);
            for i in 0..6 {
                *nx.add(i) = planes[i].normal.x;
                *ny.add(i) = planes[i].normal.y;
                *nz.add(i) = planes[i].normal.z;
            }
            for i in 6..n_f {
                *nx.add(i) = 0.0;
                *ny.add(i) = 0.0;
                *nz.add(i) = 0.0;
            }
        }
    }

    hull.compute_hash();
    hull
}

pub fn make_box_hull(hx: f32, hy: f32, hz: f32) -> Hull {
    make_transformed_box_hull(hx, hy, hz, Transform::IDENTITY)
}

pub fn make_cube_hull(half_width: f32) -> Hull {
    make_box_hull(half_width, half_width, half_width)
}

pub fn make_offset_box_hull(hx: f32, hy: f32, hz: f32, offset: Vec3) -> Hull {
    make_transformed_box_hull(hx, hy, hz, Transform::new(offset, Quat::IDENTITY))
}

/// Scale a box frame while keeping the rotation orthogonal.
pub fn scale_box(half_widths: &mut Vec3, transform: &mut Transform, post_scale: Vec3, min_half_width: f32) {
    b3_assert(is_valid_float(min_half_width) && min_half_width > 0.0);
    let mut q = transform.q;

    if post_scale.x < 0.0 || post_scale.y < 0.0 || post_scale.z < 0.0 {
        let mut m = make_matrix_from_quat(q);
        m.cx.x *= post_scale.x;
        m.cy.x *= post_scale.x;
        m.cz.x *= post_scale.x;
        m.cx.y *= post_scale.y;
        m.cy.y *= post_scale.y;
        m.cz.y *= post_scale.y;
        m.cx.z *= post_scale.z;
        m.cy.z *= post_scale.z;
        m.cz.z *= post_scale.z;
        m.cx = normalize(m.cx);
        m.cy = normalize(m.cy);
        m.cz = normalize(m.cz);
        if post_scale.x < 0.0 { m.cx = -m.cx; }
        if post_scale.y < 0.0 { m.cy = -m.cy; }
        if post_scale.z < 0.0 { m.cz = -m.cz; }
        q = make_quat_from_matrix(m);
    }

    let abs_scale = abs_vec3(post_scale);
    let h = *half_widths;
    let p1 = abs_scale * rotate_vector(q, -h);
    let p2 = abs_scale * rotate_vector(q, h);

    let local_p1 = inv_rotate_vector(q, p1);
    let local_p2 = inv_rotate_vector(q, p2);

    let lower = min_vec3(local_p1, local_p2);
    let upper = max_vec3(local_p1, local_p2);
    let scaled = 0.5 * (upper - lower);
    let limit = Vec3::new(min_half_width, min_half_width, min_half_width);
    *half_widths = max_vec3(scaled, limit);
    transform.p = post_scale * transform.p;
    transform.q = q;
}

pub fn make_scaled_box_hull(half_widths: Vec3, transform: Transform, post_scale: Vec3) -> Hull {
    b3_assert(is_valid_transform(transform));
    let mut h = half_widths;
    let mut xf = transform;
    scale_box(&mut h, &mut xf, post_scale, 4.0 * b3::linear_slop());
    make_transformed_box_hull(h.x, h.y, h.z, xf)
}
