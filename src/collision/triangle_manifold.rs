//! Triangle versus sphere, capsule, and hull collision.

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::collision::distance::shape_distance;
use crate::collision::geometry_queries::closest_point_on_triangle;
use crate::collision::hull::{find_hull_support_vertex, Hull};
use crate::collision::internal_math::{
    make_plane_from_normal_and_point, make_plane_from_points, plane_separation,
};
use crate::collision::manifold::{
    clip_polygon, find_incident_face, flip_pair, make_feature_pair, ClipVertex, FeatureOwner,
    SeparatingAxis, MAX_CLIP_POINTS, FEATURE_PAIR_SINGLE,
};
use crate::math::functions::*;
use crate::math::types::{Plane, Transform, Vec3};
use crate::types::collision_types::{
    LocalManifold, SATCache, SeparatingFeature, TriangleFeature,
};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::queries::{DistanceInput, ShapeProxy, SimplexCache};

/// Triangle data in the local space of the other shape.
#[derive(Clone, Copy, Debug, Default)]
pub struct TriangleData {
    pub v1: Vec3,
    pub v2: Vec3,
    pub v3: Vec3,
    pub e1: Vec3,
    pub e2: Vec3,
    pub e3: Vec3,
    pub plane: Plane,
    pub flags: i32,
}

/// Indexed by the 3-bit vertex mask.
const TRIANGLE_FEATURES: [TriangleFeature; 8] = [
    TriangleFeature::FeatureNone, // 000 (unreachable)
    TriangleFeature::FeatureVertex1,  // 001
    TriangleFeature::FeatureVertex2,  // 010
    TriangleFeature::FeatureEdge1,  // 011 v1,v2
    TriangleFeature::FeatureVertex3,  // 100
    TriangleFeature::FeatureEdge3,  // 101 v1,v3
    TriangleFeature::FeatureEdge2,  // 110 v2,v3
    TriangleFeature::FeatureTriangleFace, // 111
];

/// Map the GJK simplex cache to the triangle feature that generated it.
pub fn get_triangle_feature(cache: &SimplexCache) -> TriangleFeature {
    let count = cache.count as i32;
    b3_assert(0 < count && count < 4);

    // Bit i set means triangle vertex i participates in the simplex.
    let mut mask = 0usize;
    for i in 0..count as usize {
        b3_assert(cache.index_a[i] < 3);
        mask |= 1usize << cache.index_a[i];
    }

    TRIANGLE_FEATURES[mask]
}

// MARK: - Triangle and sphere

/// Triangle versus sphere.
pub fn collide_triangle_and_sphere(
    manifold: &mut LocalManifold,
    capacity: i32,
    triangle_a: &[Vec3; 3],
    sphere_b: Sphere,
) {
    manifold.point_count = 0;

    if capacity == 0 {
        return;
    }

    let center = sphere_b.center;
    let v1 = triangle_a[0];
    let v2 = triangle_a[1];
    let v3 = triangle_a[2];
    let plane = make_plane_from_points(v1, v2, v3);

    let offset = plane_separation(plane, center);
    if offset < 0.0 {
        // Cull back side collision
        return;
    }

    let radius = sphere_b.radius;

    // Closest point on triangle to sphere center
    let closest = closest_point_on_triangle(v1, v2, v3, center);

    // Test separating axis
    let squared_distance = distance_squared(closest.point, center);
    let speculative_distance = b3::speculative_distance();
    let max_distance = radius + speculative_distance;
    if squared_distance > max_distance * max_distance {
        return;
    }

    let distance = squared_distance.sqrt();

    // Normal points from triangle to sphere
    let normal = if distance * distance > 1000.0 * f32::MIN_POSITIVE {
        (1.0 / distance) * (center - closest.point)
    } else {
        normalize(cross(v2 - v1, v3 - v1))
    };

    // contact point mid-way
    // p = 0.5 * (c + q) - 0.5 * r * n
    let contact_point = 0.5 * ((center - radius * normal) + closest.point);

    manifold.normal = normal;
    manifold.point_count = 1;
    manifold.feature = closest.feature;
    manifold.squared_distance = squared_distance;

    let mp = manifold.points;
    b3_assert(!mp.is_null());
    unsafe {
        (*mp).point = contact_point;
        (*mp).separation = distance - radius;
        (*mp).pair = FEATURE_PAIR_SINGLE;
    }
}

// MARK: - Triangle and capsule

/// Clip a segment against the three side planes of a triangle face.
pub fn clip_segment_to_triangle_face(
    segment: &mut [ClipVertex],
    points: &[Vec3; 3],
    plane: Plane,
) -> bool {
    let mut vertex1 = points[2];
    for i in 0..3 {
        let vertex2 = points[i];
        let tangent = normalize(vertex2 - vertex1);
        let binormal = cross(tangent, plane.normal);

        let clip_plane = make_plane_from_normal_and_point(binormal, vertex1);

        let mut vertex_count: i32 = 0;
        let p1 = segment[0];
        let p2 = segment[1];

        let distance1 = plane_separation(clip_plane, p1.position);
        let distance2 = plane_separation(clip_plane, p2.position);

        // If the points are behind the plane
        if distance1 <= 0.0 {
            segment[vertex_count as usize] = p1;
            vertex_count += 1;
        }
        if distance2 <= 0.0 {
            segment[vertex_count as usize] = p2;
            vertex_count += 1;
        }

        // If the points are on different sides of the plane
        if (distance1 > 0.0) != (distance2 > 0.0) {
            // Find intersection point of edge and plane
            let t = distance1 / (distance1 - distance2);
            segment[vertex_count as usize].position = lerp(p1.position, p2.position, t);
            segment[vertex_count as usize].pair = if distance1 > 0.0 { p1.pair } else { p2.pair };
            vertex_count += 1;
        }

        if vertex_count != 2 {
            return false;
        }

        vertex1 = vertex2;
    }

    true
}

/// Triangle face direction versus a capsule.
pub fn query_triangle_face_and_capsule(plane: Plane, capsule: Capsule) -> SeparatingAxis {
    let separation1 = plane_separation(plane, capsule.center1);
    let separation2 = plane_separation(plane, capsule.center2);

    let mut axis = SeparatingAxis::default();
    axis.normal = plane.normal;

    if separation1 < separation2 {
        axis.separation = separation1;
        axis.index_a = 0;
        axis.index_b = 0;
    } else {
        axis.separation = separation2;
        axis.index_a = 0;
        axis.index_b = 1;
    }

    axis
}

/// Triangle edges versus a capsule edge.
pub fn query_triangle_and_capsule_edges(
    vertices: &[Vec3; 3],
    plane: Plane,
    capsule: Capsule,
) -> SeparatingAxis {
    // Work in the local space of the capsule
    let p1 = capsule.center1;
    let p2 = capsule.center2;
    let capsule_edge = p2 - p1;

    // Find axis of minimum penetration
    let mut max_normal = Vec3::ZERO;
    let mut max_separation = -f32::MAX;
    let mut max_index1: i32 = b3::NULL_INDEX;
    let mut max_index2: i32 = b3::NULL_INDEX;
    let squared_tolerance: f32 = 0.005 * 0.005;

    let mut edge_index: i32 = 2;
    let a = dot(capsule_edge, plane.normal);
    let mut v1 = vertices[2];
    for index in 0..3usize {
        let v2 = vertices[index];
        let triangle_edge = v2 - v1;
        let side_normal = normalize(cross(triangle_edge, plane.normal));

        // Pretend the triangle edge embeds a zero area face with a side normal. This
        // provides a way to find an edge-edge normal that points outward from
        // the triangle.
        let b = dot(capsule_edge, side_normal);

        // Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
        if a * a + b * b < squared_tolerance * length_squared(capsule_edge) {
            v1 = v2;
            edge_index = index as i32;
            continue;
        }

        // Similar to hull vs hull
        let axis = if a * b <= 0.0 {
            let t = b / (b - a);
            lerp(side_normal, plane.normal, t)
        } else {
            let t = b / (a + b);
            lerp(side_normal, -plane.normal, t)
        };

        b3_validate(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
        let axis = normalize(axis);
        let separation = dot(axis, p1 - v1);

        if separation > max_separation {
            // Note: We don't exit early if we find a separating axis here since we want to
            // find the best one for caching and account for the convex radius later.
            max_normal = axis;
            max_separation = separation;
            max_index1 = edge_index;
            max_index2 = 0;
        }

        v1 = v2;
        edge_index = index as i32;
    }

    // Save result
    SeparatingAxis {
        normal: max_normal,
        separation: max_separation,
        index_a: max_index1,
        index_b: max_index2,
        kind: SeparatingFeature::InvalidAxis,
    }
}

/// Build the face contact between a triangle and a capsule.
pub fn build_triangle_and_capsule_face_contact(
    manifold: &mut LocalManifold,
    triangle: &[Vec3; 3],
    plane: Plane,
    capsule: Capsule,
) {
    b3_assert(manifold.point_count == 0);

    let mut segment = [ClipVertex::default(); 2];
    segment[0].position = capsule.center1;
    segment[0].separation = 0.0;
    segment[0].pair = make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0);
    segment[1].position = capsule.center2;
    segment[1].separation = 0.0;
    segment[1].pair = make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1);

    let have_points = clip_segment_to_triangle_face(&mut segment, triangle, plane);
    if !have_points {
        return;
    }

    let radius = capsule.radius;
    let distance1 = plane_separation(plane, segment[0].position);
    let distance2 = plane_separation(plane, segment[1].position);

    let speculative_distance = b3::speculative_distance();
    if distance1 > speculative_distance + radius && distance2 > speculative_distance + radius {
        return;
    }

    // Average points. Half-way between capsule bottom and triangle plane.
    let point1 = mul_sub(segment[0].position, 0.5 * (distance1 + capsule.radius), plane.normal);
    let point2 = mul_sub(segment[1].position, 0.5 * (distance2 + capsule.radius), plane.normal);

    manifold.normal = plane.normal;
    manifold.feature = TriangleFeature::FeatureTriangleFace;
    manifold.point_count = 2;

    let pt = manifold.points;
    b3_assert(!pt.is_null());
    unsafe {
        (*pt.add(0)).point = point1;
        (*pt.add(0)).separation = distance1 - capsule.radius;
        (*pt.add(0)).pair = segment[0].pair;

        (*pt.add(1)).point = point2;
        (*pt.add(1)).separation = distance2 - capsule.radius;
        (*pt.add(1)).pair = segment[1].pair;
    }
}

/// Build the edge contact between a triangle and a capsule.
pub fn build_triangle_and_capsule_edge_contact(
    manifold: &mut LocalManifold,
    triangle: &[Vec3; 3],
    plane: Plane,
    capsule: Capsule,
    query: SeparatingAxis,
) {
    b3_assert(0 <= query.index_a && query.index_a < 3);

    let p1 = capsule.center1;
    let p2 = capsule.center2;
    let capsule_edge = p2 - p1;

    let v1 = triangle[query.index_a as usize];
    let v2 = triangle[(query.index_a as usize + 1) % 3];
    let triangle_edge = v2 - v1;

    let side_normal = normalize(cross(triangle_edge, plane.normal));

    // Pretend the triangle edge embeds a zero area face with a side normal. This
    // provides a way to find an edge-edge normal that points outward from
    // the triangle.
    let a = dot(capsule_edge, plane.normal);
    let b = dot(capsule_edge, side_normal);

    // Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
    let squared_tolerance: f32 = 0.005 * 0.005;
    if a * a + b * b < squared_tolerance * length_squared(capsule_edge) {
        return;
    }

    // Similar to hull vs hull
    let normal = query.normal;
    let result = line_distance(v1, triangle_edge, p1, capsule_edge);

    if result.fraction1 < 0.0
        || 1.0 < result.fraction1
        || result.fraction2 < 0.0
        || 1.0 < result.fraction2
    {
        // closest point beyond end points
        return;
    }

    let point = lerp(
        result.point1,
        mul_sub(result.point2, capsule.radius, normal),
        0.5,
    );
    let separation = dot(normal, p1 - v1);

    manifold.normal = normal;
    manifold.point_count = 1;

    manifold.feature = if query.index_a == 0 {
        TriangleFeature::FeatureEdge1
    } else if query.index_a == 1 {
        TriangleFeature::FeatureEdge2
    } else {
        TriangleFeature::FeatureEdge3
    };

    let pt = manifold.points;
    b3_assert(!pt.is_null());
    unsafe {
        (*pt).point = point;
        (*pt).separation = separation - capsule.radius;
        (*pt).pair = make_feature_pair(
            FeatureOwner::ShapeA,
            query.index_a,
            FeatureOwner::ShapeB,
            query.index_b,
        );
    }
}

/// Triangle versus capsule.
pub fn collide_triangle_and_capsule(
    manifold: &mut LocalManifold,
    capacity: i32,
    triangle_a: &[Vec3; 3],
    capsule_b: Capsule,
    cache: &mut SimplexCache,
) {
    manifold.point_count = 0;

    if capacity < 2 {
        return;
    }

    let v1 = triangle_a[0];
    let v2 = triangle_a[1];
    let v3 = triangle_a[2];
    let plane = make_plane_from_points(v1, v2, v3);
    let capsule_center = lerp(capsule_b.center1, capsule_b.center2, 0.5);

    let offset = plane_separation(plane, capsule_center);
    if offset < 0.0 {
        // Cull back side collision
        return;
    }

    let capsule_points = [capsule_b.center1, capsule_b.center2];

    let distance_input = DistanceInput {
        proxy_a: ShapeProxy::new(Some(triangle_a.as_ptr()), 3, 0.0),
        proxy_b: ShapeProxy::new(Some(capsule_points.as_ptr()), 2, 0.0),
        transform: Transform::IDENTITY,
        use_radii: false,
    };

    let distance_output = shape_distance(&distance_input, cache, None);

    let speculative_distance = b3::speculative_distance();
    let radius = capsule_b.radius;
    if distance_output.distance > radius + speculative_distance {
        // Shapes are separated, persist the cache
        return;
    }

    let pt = manifold.points;
    b3_assert(!pt.is_null());

    if distance_output.distance > 100.0 * f32::EPSILON {
        // Shallow penetration
        let delta = normalize(distance_output.point_b - distance_output.point_a);

        // Try to create two contact points if closest points difference is nearly parallel to
        // face normal
        let k_tolerance: f32 = 0.2;
        let cos_angle = abs_float(dot(plane.normal, delta));
        if cos_angle > k_tolerance {
            // Clip capsule segment against side planes of reference face
            let mut segment = [ClipVertex::default(); 2];
            segment[0].position = capsule_b.center1;
            segment[0].separation = 0.0;
            segment[0].pair = make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0);
            segment[1].position = capsule_b.center2;
            segment[1].separation = 0.0;
            segment[1].pair = make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1);

            let have_points = clip_segment_to_triangle_face(&mut segment, triangle_a, plane);

            if have_points {
                let distance1 = plane_separation(plane, segment[0].position);
                let distance2 = plane_separation(plane, segment[1].position);

                let normal = plane.normal;
                let point1 = mul_sub(segment[0].position, 0.5 * (radius + distance1), normal);
                let point2 = mul_sub(segment[1].position, 0.5 * (radius + distance2), normal);

                // Normal points from triangle to capsule
                manifold.normal = normal;
                manifold.feature = TriangleFeature::FeatureTriangleFace;
                manifold.point_count = 2;

                unsafe {
                    (*pt.add(0)).point = point1;
                    (*pt.add(0)).separation = distance1 - radius;
                    (*pt.add(0)).pair = segment[0].pair;

                    (*pt.add(1)).point = point2;
                    (*pt.add(1)).separation = distance2 - radius;
                    (*pt.add(1)).pair = segment[1].pair;
                }

                return;
            }
        }

        // Create contact from closest points.
        let point = lerp(
            distance_output.point_a,
            mul_sub(distance_output.point_b, radius, delta),
            0.5,
        );

        // Normal points from triangle to capsule.
        manifold.normal = delta;
        manifold.point_count = 1;
        manifold.feature = get_triangle_feature(cache);

        unsafe {
            (*pt).point = point;
            (*pt).separation = distance_output.distance - radius;
            (*pt).pair = FEATURE_PAIR_SINGLE;
        }

        return;
    }

    // Deep penetration

    let face_query = query_triangle_face_and_capsule(plane, capsule_b);
    if face_query.separation > radius {
        // Shapes are separated. Should be impossible for a reasonable capsule radius.
        return;
    }

    let edge_query = query_triangle_and_capsule_edges(triangle_a, plane, capsule_b);
    if edge_query.separation > radius {
        // Shapes are separated. Should be impossible for a reasonable capsule radius.
        return;
    }

    // Create face contact
    let mut face_separation = face_query.separation - radius;
    build_triangle_and_capsule_face_contact(manifold, triangle_a, plane, capsule_b);
    b3_validate(manifold.point_count == 0 || manifold.point_count == 2);
    if manifold.point_count == 2 {
        // This becomes the clipped separation.
        unsafe {
            face_separation = min_float((*pt).separation, (*pt.add(1)).separation);
        }
    }

    if edge_query.index_a == b3::NULL_INDEX {
        // No valid edge contact.
        return;
    }

    // Face contact can be empty if it does not realize the axis of minimum penetration.
    // Create edge contact if face contact fails or edge contact is significantly better!
    let linear_slop = b3::linear_slop();
    let edge_separation = edge_query.separation - radius;
    if manifold.point_count == 0 || edge_separation > face_separation + linear_slop {
        // Edge contact
        build_triangle_and_capsule_edge_contact(manifold, triangle_a, plane, capsule_b, edge_query);
    }
}

// MARK: - Triangle and hull

/// The support vertex of a triangle.
pub fn get_triangle_support(points: &[Vec3; 3], direction: Vec3) -> i32 {
    let mut index: i32 = 0;
    let mut distance = dot(points[0], direction);

    let d = dot(points[1], direction);
    if d > distance {
        distance = d;
        index = 1;
    }

    let d = dot(points[2], direction);
    if d > distance {
        return 2;
    }

    index
}

/// Triangle face direction versus a hull.
pub fn query_triangle_face(triangle: &TriangleData, hull: &Hull) -> SeparatingAxis {
    let hull_points = hull.points();
    let plane = triangle.plane;
    let normal = -plane.normal;
    let vertex_index = find_hull_support_vertex(hull, normal);
    let support = hull_points[vertex_index as usize];
    let separation = plane_separation(plane, support);

    SeparatingAxis {
        normal: plane.normal,
        separation,
        index_a: 0,
        index_b: vertex_index,
        kind: SeparatingFeature::FaceAxisA,
    }
}

/// Hull face direction versus a triangle.
pub fn query_hull_face(triangle: &TriangleData, hull: &Hull) -> SeparatingAxis {
    let hull_planes = hull.planes();
    let face_count = hull.face_count();

    let triangle_points = [triangle.v1, triangle.v2, triangle.v3];

    let mut max_normal = Vec3::ZERO;
    let mut max_face_separation = -f32::INFINITY;
    let mut max_face_index: i32 = b3::NULL_INDEX;
    let mut max_vertex_index: i32 = b3::NULL_INDEX;

    for face_index in 0..face_count as usize {
        let plane = hull_planes[face_index];

        let vertex_index = get_triangle_support(&triangle_points, -plane.normal);
        let support = triangle_points[vertex_index as usize];
        let separation = plane_separation(plane, support);
        if separation > max_face_separation {
            max_normal = plane.normal;
            max_face_separation = separation;
            max_face_index = face_index as i32;
            max_vertex_index = vertex_index;
        }
    }

    // Normal points from triangle to hull
    SeparatingAxis {
        normal: -max_normal,
        separation: max_face_separation,
        index_a: max_vertex_index,
        index_b: max_face_index,
        kind: SeparatingFeature::FaceAxisB,
    }
}

/// Triangle edge versus hull edge.
pub fn query_triangle_and_hull_edges(triangle: &TriangleData, hull: &Hull) -> SeparatingAxis {
    let mut result = SeparatingAxis {
        normal: Vec3::ZERO,
        separation: -f32::INFINITY,
        index_a: b3::NULL_INDEX,
        index_b: b3::NULL_INDEX,
        kind: SeparatingFeature::EdgePairAxis,
    };

    let triangle_points = [triangle.v1, triangle.v2, triangle.v3];
    let triangle_edges = [triangle.e1, triangle.e2, triangle.e3];

    let tri_normal = triangle.plane.normal;

    let hull_edges = hull.edges();
    let hull_points = hull.points();
    let hull_planes = hull.planes();
    let edge_count = hull.half_edge_count();
    let squared_tolerance: f32 = 0.005 * 0.005;

    let mut i = 0usize;
    while i < edge_count as usize {
        let edge = hull_edges[i];
        let twin = hull_edges[i + 1];
        b3_assert(edge.twin as usize == i + 1 && twin.twin as usize == i);

        let hull_point = hull_points[edge.origin as usize];
        let hull_edge = hull_points[twin.origin as usize] - hull_point;

        let hull_normal1 = hull_planes[edge.face as usize].normal;
        let hull_normal2 = hull_planes[twin.face as usize].normal;

        for j in 0..3usize {
            let tri_edge = triangle_edges[j];

            let cab = dot(hull_normal1, tri_edge);
            let dab = dot(hull_normal2, tri_edge);
            let bcd = dot(tri_normal, hull_edge);
            if cab * dab >= 0.0 || cab * bcd <= 0.0 {
                continue;
            }

            // Avoid nearly parallel edges that may lead to invalid separation values at the noise
            // floor.
            if max_float(cab * cab, dab * dab) < squared_tolerance * length_squared(tri_edge) {
                continue;
            }

            // Similar to hull vs hull
            // dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
            // Normal points out of hull by construction.
            let t = cab / (cab - dab);
            let mut axis = lerp(hull_normal1, hull_normal2, t);
            b3_validate(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
            axis = normalize(axis);
            let separation = dot(axis, triangle_points[j] - hull_point);

            if separation > result.separation {
                // Note: We don't exit early if we find a separating axis here since we want to
                // find the best one for caching.
                // Flip normal to point from triangle to hull.
                result.normal = -axis;
                result.separation = separation;
                result.index_a = j as i32;
                result.index_b = i as i32;
            }
        }

        i += 2;
    }

    result
}

/// Clip the triangle face against a hull face.
#[allow(clippy::too_many_arguments)]
pub fn collide_hull_face(
    manifold: &mut LocalManifold,
    point_capacity: i32,
    triangle: &TriangleData,
    hull: &Hull,
    query: SeparatingAxis,
    cache: &mut SATCache,
    enable_speculative: bool,
) -> f32 {
    b3_validate(query.kind == SeparatingFeature::FaceAxisB);
    b3_validate(0 <= query.index_a && query.index_a < 3);
    b3_validate(0 <= query.index_b && query.index_b < hull.face_count());

    manifold.point_count = 0;

    let hull_faces = hull.faces();
    let hull_edges = hull.edges();
    let hull_planes = hull.planes();
    let hull_points = hull.points();

    // Reference hull face
    let ref_plane = hull_planes[query.index_b as usize];

    // Build clip polygon from triangle face (the incident face)
    let mut buffer1 = [ClipVertex::default(); 64];
    let mut buffer2 = [ClipVertex::default(); 64];

    let v1 = triangle.v1;
    let v2 = triangle.v2;
    let v3 = triangle.v3;
    buffer1[0].position = v1;
    buffer1[0].separation = plane_separation(ref_plane, v1);
    buffer1[0].pair = make_feature_pair(FeatureOwner::ShapeB, 2, FeatureOwner::ShapeB, 0);
    buffer1[1].position = v2;
    buffer1[1].separation = plane_separation(ref_plane, v2);
    buffer1[1].pair = make_feature_pair(FeatureOwner::ShapeB, 0, FeatureOwner::ShapeB, 1);
    buffer1[2].position = v3;
    buffer1[2].separation = plane_separation(ref_plane, v3);
    buffer1[2].pair = make_feature_pair(FeatureOwner::ShapeB, 1, FeatureOwner::ShapeB, 2);

    let mut point_count: i32 = 3;

    // Clip triangle face against side planes of reference face
    let mut use_buffer1_as_input = true;
    let mut use_buffer1_as_output = false;

    let face = hull_faces[query.index_b as usize];
    let mut edge_index = face.edge as i32;

    loop {
        let edge = hull_edges[edge_index as usize];
        let next_edge_index = edge.next as i32;
        let next = hull_edges[next_edge_index as usize];
        let vertex1 = hull_points[edge.origin as usize];
        let vertex2 = hull_points[next.origin as usize];
        let tangent = normalize(vertex2 - vertex1);
        let binormal = cross(tangent, ref_plane.normal);

        let clip_plane = make_plane_from_normal_and_point(binormal, vertex1);

        if use_buffer1_as_input {
            use_buffer1_as_output = true;
            point_count = clip_polygon(
                &mut buffer2,
                &buffer1,
                point_count,
                clip_plane,
                edge_index,
                ref_plane,
            );
        } else {
            use_buffer1_as_output = false;
            point_count = clip_polygon(
                &mut buffer1,
                &buffer2,
                point_count,
                clip_plane,
                edge_index,
                ref_plane,
            );
        }
        b3_assert(point_count <= MAX_CLIP_POINTS);

        if point_count < 3 {
            // Using a stale cache
            *cache = SATCache::default();
            return query.separation;
        }

        use_buffer1_as_input = !use_buffer1_as_input;
        edge_index = next_edge_index;
        if edge_index == face.edge as i32 {
            break;
        }
    }

    point_count = min_int(point_count, point_capacity);
    let mut min_separation = f32::MAX;
    let mut final_point_count: i32 = 0;

    let input: &[ClipVertex] = if use_buffer1_as_output {
        &buffer2
    } else {
        &buffer1
    };

    let pt = manifold.points;
    b3_assert(!pt.is_null());

    for i in 0..point_count as usize {
        let clip_point = input[i];
        min_separation = min_float(min_separation, clip_point.separation);

        if !enable_speculative && clip_point.separation > 0.0 {
            continue;
        }

        // Move point onto hull face for improved culling
        let point = mul_sub(clip_point.position, clip_point.separation, ref_plane.normal);

        unsafe {
            (*pt.add(final_point_count as usize)).point = point;
            (*pt.add(final_point_count as usize)).separation = clip_point.separation;
            (*pt.add(final_point_count as usize)).pair = flip_pair(&clip_point.pair);
        }

        final_point_count += 1;
    }

    let speculative_distance = if enable_speculative {
        b3::speculative_distance()
    } else {
        0.0
    };
    if min_separation > speculative_distance {
        // This can occur with a stale SAT cache
        manifold.point_count = 0;
        *cache = SATCache::default();
        return min_separation;
    }

    manifold.point_count = final_point_count;
    manifold.normal = -ref_plane.normal;
    manifold.feature = TriangleFeature::FeatureHullFace;

    // Save cache
    cache.separation = min_separation;
    cache.type_ = SeparatingFeature::FaceAxisB;
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;
    min_separation
}

/// Clip the incident hull face against the triangle face.
#[allow(clippy::too_many_arguments)]
pub fn collide_triangle_face(
    manifold: &mut LocalManifold,
    point_capacity: i32,
    triangle: &TriangleData,
    hull: &Hull,
    query: SeparatingAxis,
    cache: &mut SATCache,
    enable_speculative: bool,
) -> f32 {
    b3_validate(query.kind == SeparatingFeature::FaceAxisA);
    b3_validate(query.index_a == 0);
    b3_validate(0 <= query.index_b && query.index_b < hull.vertex_count());
    b3_validate(manifold.point_count == 0);

    let hull_faces = hull.faces();
    let hull_edges = hull.edges();
    let hull_points = hull.points();

    // Find incident face
    let ref_plane = triangle.plane;

    let inc_face = find_incident_face(hull, ref_plane.normal, query.index_b);

    // Build clip polygon from incident face
    let big_capacity = (2 * MAX_CLIP_POINTS) as usize;
    let mut buffer1 = [ClipVertex::default(); 128];
    let mut buffer2 = [ClipVertex::default(); 128];

    let mut point_count: i32 = 0;
    let face = hull_faces[inc_face as usize];
    let mut hull_edge_index = face.edge as i32;

    loop {
        let edge = hull_edges[hull_edge_index as usize];

        let next_edge_index = edge.next as i32;
        let next = hull_edges[next_edge_index as usize];

        let hull_point = hull_points[next.origin as usize];
        buffer1[point_count as usize].position = hull_point;
        buffer1[point_count as usize].separation = plane_separation(ref_plane, hull_point);
        buffer1[point_count as usize].pair = make_feature_pair(
            FeatureOwner::ShapeB,
            hull_edge_index,
            FeatureOwner::ShapeB,
            next_edge_index,
        );

        point_count += 1;
        hull_edge_index = next_edge_index;
        if hull_edge_index == face.edge as i32 || point_count >= big_capacity as i32 {
            break;
        }
    }

    b3_assert(point_count >= 3);

    // Clip incident face against side planes of reference face (triangle)
    let mut use_buffer1_as_input = true;
    let mut use_buffer1_as_output = false;

    let triangle_points = [triangle.v1, triangle.v2, triangle.v3];
    let triangle_edges = [triangle.e1, triangle.e2, triangle.e3];

    for i in 0..3usize {
        if point_count == 0 {
            continue;
        }

        let side_normal = normalize(cross(triangle_edges[i], ref_plane.normal));

        let clip_plane = make_plane_from_normal_and_point(side_normal, triangle_points[i]);

        if use_buffer1_as_input {
            use_buffer1_as_output = true;
            point_count = clip_polygon(
                &mut buffer2,
                &buffer1,
                point_count,
                clip_plane,
                i as i32,
                ref_plane,
            );
        } else {
            use_buffer1_as_output = false;
            point_count = clip_polygon(
                &mut buffer1,
                &buffer2,
                point_count,
                clip_plane,
                i as i32,
                ref_plane,
            );
        }
        b3_assert(point_count <= big_capacity as i32);

        use_buffer1_as_input = !use_buffer1_as_input;
    }

    if point_count == 0 {
        // Triangle face clipped away. Invalidate cache.
        *cache = SATCache::default();
        return f32::MAX;
    }

    point_count = min_int(point_count, point_capacity);

    let mut min_separation = f32::MAX;

    let mut final_point_count: i32 = 0;
    let input: &[ClipVertex] = if use_buffer1_as_output {
        &buffer2
    } else {
        &buffer1
    };
    let pt = manifold.points;
    b3_assert(!pt.is_null());

    for i in 0..point_count as usize {
        let clip_point = input[i];
        min_separation = min_float(min_separation, clip_point.separation);

        if !enable_speculative && clip_point.separation > 0.0 {
            continue;
        }

        // Move point onto triangle surface for improved culling
        let point = clip_point.position;

        unsafe {
            (*pt.add(final_point_count as usize)).point = point;
            (*pt.add(final_point_count as usize)).separation = clip_point.separation;
            (*pt.add(final_point_count as usize)).pair = clip_point.pair;
        }

        final_point_count += 1;
    }

    let speculative_distance = if enable_speculative {
        b3::speculative_distance()
    } else {
        0.0
    };
    if min_separation >= speculative_distance {
        // This can happen if the objects move apart while re-using a cached axis
        *cache = SATCache::default();
        return min_separation;
    }

    manifold.point_count = final_point_count;
    manifold.normal = ref_plane.normal;
    manifold.feature = TriangleFeature::FeatureTriangleFace;

    // Save cache
    cache.separation = min_separation;
    cache.type_ = SeparatingFeature::FaceAxisA;
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;
    min_separation
}

/// Build the edge contact between a triangle edge and a hull edge.
pub fn collide_triangle_and_hull_edges(
    manifold: &mut LocalManifold,
    capacity: i32,
    triangle_point: Vec3,
    triangle_edge: Vec3,
    hull: &Hull,
    query: SeparatingAxis,
    cache: &mut SATCache,
) {
    b3_validate(query.kind == SeparatingFeature::EdgePairAxis);
    b3_validate(0 <= query.index_a && query.index_a < 3);
    b3_validate(0 <= query.index_b && query.index_b < hull.half_edge_count());
    b3_validate(query.separation <= 2.0 * b3::speculative_distance());

    let p_a = triangle_point;
    let e_a = triangle_edge;

    let edges_b = hull.edges();
    let points_b = hull.points();
    let edge_b = edges_b[query.index_b as usize];
    let twin_b = edges_b[edge_b.twin as usize];
    let p_b = points_b[edge_b.origin as usize];
    let q_b = points_b[twin_b.origin as usize];
    let e_b = q_b - p_b;

    // Get the closest points between the infinite edge lines
    let result = line_distance(p_a, e_a, p_b, e_b);

    // Is one of the closest points outside of the associated edge segment?
    if capacity == 0
        || result.fraction1 < 0.0
        || 1.0 < result.fraction1
        || result.fraction2 < 0.0
        || 1.0 < result.fraction2
    {
        // Invalid edge pair, no points generated
        b3_assert(manifold.point_count == 0);
        *cache = SATCache::default();
        return;
    }

    let separation = dot(query.normal, p_b - p_a);
    let point = 0.5 * (result.point1 + result.point2);

    let pt = manifold.points;
    b3_assert(!pt.is_null());
    unsafe {
        (*pt).point = point;
        (*pt).separation = separation;
        (*pt).pair = make_feature_pair(
            FeatureOwner::ShapeA,
            query.index_a,
            FeatureOwner::ShapeB,
            query.index_b,
        );
    }

    // Save cache
    cache.separation = separation;
    cache.type_ = SeparatingFeature::EdgePairAxis;
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;

    manifold.normal = query.normal;
    manifold.point_count = 1;

    manifold.feature = if query.index_a == 0 {
        TriangleFeature::FeatureEdge1
    } else if query.index_a == 1 {
        TriangleFeature::FeatureEdge2
    } else {
        TriangleFeature::FeatureEdge3
    };
}

/// Triangle versus hull. The triangle is in the local space of the hull.
#[allow(clippy::too_many_arguments)]
pub fn collide_triangle_and_hull(
    manifold: &mut LocalManifold,
    capacity: i32,
    v1: Vec3,
    v2: Vec3,
    v3: Vec3,
    triangle_flags: i32,
    hull_b: &Hull,
    cache: &mut SATCache,
    enable_speculative: bool,
) {
    manifold.point_count = 0;
    manifold.feature = TriangleFeature::FeatureNone;

    if capacity < 4 {
        return;
    }

    let triangle_plane = make_plane_from_points(v1, v2, v3);
    let linear_slop = b3::linear_slop();

    let offset = plane_separation(triangle_plane, hull_b.center());
    if cache.type_ == SeparatingFeature::BacksideAxis {
        // Use hysteresis to avoid jitter on wavy meshes
        if abs_float(cache.separation - offset) < linear_slop {
            return;
        }

        cache.type_ = SeparatingFeature::InvalidAxis;
    }

    if offset < -linear_slop {
        // Cull back side collision. Cache offset to add hysteresis.
        cache.type_ = SeparatingFeature::BacksideAxis;
        cache.separation = offset;
        return;
    }

    let triangle_points = [v1, v2, v3];
    let triangle_edges = [v2 - v1, v3 - v2, v1 - v3];

    let triangle = TriangleData {
        v1,
        v2,
        v3,
        e1: triangle_edges[0],
        e2: triangle_edges[1],
        e3: triangle_edges[2],
        plane: triangle_plane,
        flags: triangle_flags,
    };

    let edges = hull_b.edges();
    let hull_planes = hull_b.planes();
    let hull_points = hull_b.points();

    let speculative_distance = if enable_speculative {
        b3::speculative_distance()
    } else {
        0.0
    };
    cache.hit = 1;

    // Attempt to use the cache to speed up collision
    match cache.type_ {
        SeparatingFeature::FaceAxisA => {
            b3_assert(cache.index_a == 0);

            let vertex_index = find_hull_support_vertex(hull_b, -triangle_plane.normal);
            let support = hull_points[vertex_index as usize];
            let separation = plane_separation(triangle_plane, support);
            if separation > speculative_distance {
                // Cache hit, shapes are separated
                return;
            }

            let face_query = SeparatingAxis {
                normal: triangle_plane.normal,
                separation,
                index_a: cache.index_a as i32,
                index_b: vertex_index,
                kind: SeparatingFeature::FaceAxisA,
            };

            // Read cache but don't modify it
            let mut local_cache = *cache;
            let clipped_separation = collide_triangle_face(
                manifold,
                capacity,
                &triangle,
                hull_b,
                face_query,
                &mut local_cache,
                enable_speculative,
            );

            if manifold.point_count > 0 && abs_float(cache.separation - clipped_separation) < linear_slop
            {
                // Cache hit, contact points generated
                return;
            }

            // Invalidate cache and fall through
            manifold.point_count = 0;
            *cache = SATCache::default();
        }

        SeparatingFeature::FaceAxisB => {
            b3_assert((cache.index_b as i32) < hull_b.face_count());

            let plane = hull_planes[cache.index_b as usize];

            // Get triangle support point
            let mut vertex_index: i32 = 0;
            let mut distance = -dot(v1, plane.normal);
            for i in 1..3usize {
                let d = -dot(triangle_points[i], plane.normal);
                if d > distance {
                    distance = d;
                    vertex_index = i as i32;
                }
            }

            let support = triangle_points[vertex_index as usize];

            // Separation of triangle support point with hull plane
            let separation = plane_separation(plane, support);
            if separation > speculative_distance {
                // Cache hit, shapes are separated
                return;
            }

            // Deep overlap may lead to an invalid cache
            let is_deep = separation < -2.0 * linear_slop;

            // Don't persist deep cache or allow separation to change too much
            if !is_deep {
                // Try to rebuild contact from last features
                let face_query = SeparatingAxis {
                    normal: -plane.normal,
                    separation,
                    index_a: vertex_index,
                    index_b: cache.index_b as i32,
                    kind: SeparatingFeature::FaceAxisB,
                };

                // Read cache but don't modify it
                let mut local_cache = *cache;
                let clipped_separation = collide_hull_face(
                    manifold,
                    capacity,
                    &triangle,
                    hull_b,
                    face_query,
                    &mut local_cache,
                    enable_speculative,
                );

                // Cache reuse is only successful if it creates contact points and the clipped
                // separation didn't change much.
                if manifold.point_count > 0
                    && abs_float(cache.separation - clipped_separation) < linear_slop
                {
                    // Cache hit, contact points generated
                    return;
                }
            }

            // Invalidate cache and fall through
            manifold.point_count = 0;
            *cache = SATCache::default();
        }

        SeparatingFeature::EdgePairAxis => {
            b3_assert(cache.index_a < 3);
            let index_a = cache.index_a as i32;

            let tri_point = triangle_points[index_a as usize];
            let tri_edge = triangle_edges[index_a as usize];

            b3_assert((cache.index_b as i32) < hull_b.half_edge_count() - 1);
            let index_b = cache.index_b as i32;

            let edge2 = edges[index_b as usize];
            let twin2 = edges[index_b as usize + 1];
            b3_assert(edge2.twin as i32 == index_b + 1 && twin2.twin as i32 == index_b);

            let hull_point = hull_points[edge2.origin as usize];
            let hull_edge = hull_points[twin2.origin as usize] - hull_point;
            let hull_normal1 = hull_planes[edge2.face as usize].normal;
            let hull_normal2 = hull_planes[twin2.face as usize].normal;

            // Confirm the edge pair is still a Minkowski face.
            // See "Collision Detection of Convex Polyhedra Based on Duality Transformation"
            // Simplified for triangle versus hull.
            let cab = dot(hull_normal1, tri_edge);
            let dab = dot(hull_normal2, tri_edge);
            let bcd = dot(triangle_plane.normal, hull_edge);

            if cab * dab < 0.0 && cab * bcd > 0.0 {
                let squared_tolerance: f32 = 0.005 * 0.005;

                // Avoid nearly parallel edges that may lead to invalid separation values at the
                // noise floor.
                if max_float(cab * cab, dab * dab)
                    >= squared_tolerance * length_squared(tri_edge)
                {
                    // Similar to hull vs hull
                    let t = cab / (cab - dab);
                    let mut axis = lerp(hull_normal1, hull_normal2, t);
                    b3_validate(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
                    axis = normalize(axis);
                    let separation = dot(axis, tri_point - hull_point);
                    if separation > speculative_distance {
                        // Cache hit, shapes are separated
                        return;
                    }

                    if abs_float(cache.separation - separation) < linear_slop {
                        // Try to rebuild contact from last features
                        // Flip normal to point from triangle to hull
                        let edge_query = SeparatingAxis {
                            normal: -axis,
                            index_a,
                            index_b,
                            separation,
                            kind: SeparatingFeature::EdgePairAxis,
                        };

                        // Read cache but don't modify it
                        let mut local_cache = *cache;
                        collide_triangle_and_hull_edges(
                            manifold,
                            capacity,
                            tri_point,
                            tri_edge,
                            hull_b,
                            edge_query,
                            &mut local_cache,
                        );

                        if manifold.point_count > 0 {
                            // Cache hit, contact point generated
                            return;
                        }
                    }
                }
            }

            // Invalidate cache and fall through
            *cache = SATCache::default();
        }

        SeparatingFeature::ManualFaceAxisA => {
            let query = query_triangle_face(&triangle, hull_b);
            let _ = collide_triangle_face(
                manifold,
                capacity,
                &triangle,
                hull_b,
                query,
                cache,
                enable_speculative,
            );
            return;
        }

        SeparatingFeature::ManualFaceAxisB => {
            let query = query_hull_face(&triangle, hull_b);
            let _ = collide_hull_face(
                manifold,
                capacity,
                &triangle,
                hull_b,
                query,
                cache,
                enable_speculative,
            );
            return;
        }

        SeparatingFeature::ManualEdgePairAxis => {
            let query = query_triangle_and_hull_edges(&triangle, hull_b);
            if query.index_a != b3::NULL_INDEX {
                let triangle_point = triangle_points[query.index_a as usize];
                let triangle_edge = triangle_edges[query.index_a as usize];
                collide_triangle_and_hull_edges(
                    manifold,
                    capacity,
                    triangle_point,
                    triangle_edge,
                    hull_b,
                    query,
                    cache,
                );
            }
            return;
        }

        SeparatingFeature::InvalidAxis => {}
        SeparatingFeature::BacksideAxis | SeparatingFeature::ClosestPointsAxis => {
            b3_assert(false);
        }
    }

    // Cache miss
    cache.hit = 0;

    // Find axis of minimum penetration
    let face_query_a = query_triangle_face(&triangle, hull_b);
    if face_query_a.separation > speculative_distance {
        // Separating axis found
        cache.separation = face_query_a.separation;
        cache.type_ = SeparatingFeature::FaceAxisA;
        cache.index_a = face_query_a.index_a as u8;
        cache.index_b = face_query_a.index_b as u8;
        return;
    }

    let face_query_b = query_hull_face(&triangle, hull_b);
    if face_query_b.separation > speculative_distance {
        // Separating axis found
        cache.separation = face_query_b.separation;
        cache.type_ = SeparatingFeature::FaceAxisB;
        cache.index_a = face_query_b.index_a as u8;
        cache.index_b = face_query_b.index_b as u8;
        return;
    }

    let edge_query = query_triangle_and_hull_edges(&triangle, hull_b);
    if edge_query.separation > speculative_distance {
        // Separating axis found
        cache.separation = edge_query.separation;
        cache.type_ = SeparatingFeature::EdgePairAxis;
        cache.index_a = edge_query.index_a as u8;
        cache.index_b = edge_query.index_b as u8;
        return;
    }

    // Don't admit a hull face significantly opposed to the triangle face.
    // Need a tolerance to avoid ghost collisions.
    let pushing_down = dot(face_query_b.normal, triangle_plane.normal) < -0.25;
    let clip_separation = if face_query_b.separation >= face_query_a.separation && !pushing_down {
        collide_hull_face(
            manifold,
            capacity,
            &triangle,
            hull_b,
            face_query_b,
            cache,
            enable_speculative,
        )
    } else {
        collide_triangle_face(
            manifold,
            capacity,
            &triangle,
            hull_b,
            face_query_a,
            cache,
            enable_speculative,
        )
    };

    // Does an edge axis exist?
    if edge_query.index_a != b3::NULL_INDEX {
        // When axes are aligned the edge separation can be garbage.
        // If a face axis has positive separation there may be no points.
        let max_face_separation = max_float(face_query_a.separation, face_query_b.separation);

        if (manifold.point_count == 0 && edge_query.separation > max_face_separation)
            || (manifold.point_count == 1 && edge_query.separation > clip_separation + linear_slop)
        {
            b3_assert(0 <= edge_query.index_a && edge_query.index_a < 3);
            let triangle_point = triangle_points[edge_query.index_a as usize];
            let triangle_edge = triangle_edges[edge_query.index_a as usize];
            manifold.point_count = 0;
            collide_triangle_and_hull_edges(
                manifold,
                capacity,
                triangle_point,
                triangle_edge,
                hull_b,
                edge_query,
                cache,
            );
        }
    }

    // Using the speculative distance means that sometimes there are no valid contact points from
    // SAT. In this fall back to GJK. This is important to prevent tunneling in rare cases.
    if manifold.point_count == 0 {
        let triangle_b = [v1, v2, v3];

        let distance_input = DistanceInput {
            proxy_a: ShapeProxy::new(Some(triangle_b.as_ptr()), 3, 0.0),
            proxy_b: ShapeProxy::new(Some(hull_points.as_ptr()), hull_b.vertex_count(), 0.0),
            transform: Transform::IDENTITY,
            use_radii: false,
        };

        let mut simplex_cache = SimplexCache::default();
        let distance_output = shape_distance(&distance_input, &mut simplex_cache, None);

        if distance_output.distance > 0.0 {
            b3_assert(0 < simplex_cache.count && simplex_cache.count <= 3);

            manifold.point_count = 1;
            manifold.feature = get_triangle_feature(&simplex_cache);
            manifold.normal = distance_output.normal;

            let pt = manifold.points;
            b3_assert(!pt.is_null());
            unsafe {
                (*pt).point = distance_output.point_b;
                (*pt).separation = distance_output.distance;

                // This feature pair is not accurate but maybe it doesn't matter
                (*pt).pair = FEATURE_PAIR_SINGLE;
            }
        }

        // No way to cache this scenario
        *cache = SATCache::default();
    }
}
