//! Sphere and capsule collision routines.

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::collision::convex_manifold::{clip_segment, clip_segment_to_hull_face, is_within_segments};
use crate::collision::distance::{get_point_support, shape_distance};
use crate::collision::hull::{find_hull_support_face, Hull};
use crate::collision::internal_math::plane_separation;
use crate::collision::manifold::{
    make_feature_pair, ClipVertex, FeatureOwner, SeparatingAxis, FEATURE_PAIR_SINGLE,
};
use crate::math::functions::*;
use crate::math::types::{Plane, Transform, Vec3};
use crate::types::collision_types::{LocalManifold, SeparatingFeature};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::queries::{DistanceInput, ShapeProxy, SimplexCache};

// MARK: - Sphere contacts

/// Sphere versus sphere.
pub fn collide_spheres(
    manifold: &mut LocalManifold,
    capacity: i32,
    sphere_a: Sphere,
    sphere_b: Sphere,
    transform_b_to_a: Transform,
) {
    if capacity == 0 {
        return;
    }

    let center1 = sphere_a.center;
    let center2 = transform_point(transform_b_to_a, sphere_b.center);

    let total_radius = sphere_a.radius + sphere_b.radius;
    let max_distance = total_radius + b3::speculative_distance();
    let offset = center2 - center1;
    let distance_sq = length_squared(offset);

    if distance_sq > max_distance * max_distance {
        // We found a separating axis
        return;
    }

    let mut normal = Vec3::new(0.0, 1.0, 0.0);
    let distance = distance_sq.sqrt();
    if distance * distance > 1000.0 * f32::MIN_POSITIVE {
        normal = (1.0 / distance) * offset;
    }

    // Contact at the midpoint
    // 0.5 * ( ((c1 + rA*n) + c2) - rB*n )
    let point = 0.5
        * (mul_sub(
            mul_add(center1, sphere_a.radius, normal) + center2,
            sphere_b.radius,
            normal,
        ));

    // Manifold in frame A
    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = manifold.points;
    b3_assert(!pt.is_null());
    unsafe {
        (*pt).point = point;
        (*pt).separation = distance - total_radius;
        (*pt).pair = FEATURE_PAIR_SINGLE;
    }
}

/// Capsule versus sphere.
pub fn collide_capsule_and_sphere(
    manifold: &mut LocalManifold,
    capacity: i32,
    capsule_a: Capsule,
    sphere_b: Sphere,
    transform_b_to_a: Transform,
) {
    manifold.point_count = 0;

    if capacity < 1 {
        return;
    }

    let center = transform_point(transform_b_to_a, sphere_b.center);
    let center1 = capsule_a.center1;
    let center2 = capsule_a.center2;

    let total_radius = sphere_b.radius + capsule_a.radius;
    let max_distance = total_radius + b3::speculative_distance();

    let closest_point = point_to_segment_distance(center1, center2, center);
    let offset = center - closest_point;
    let distance_sq = length_squared(offset);

    if distance_sq > max_distance * max_distance {
        // We found a separating axis
        return;
    }

    let mut normal = Vec3::new(0.0, 1.0, 0.0);
    let distance = distance_sq.sqrt();
    if distance * distance > 1000.0 * f32::MIN_POSITIVE {
        normal = (1.0 / distance) * offset;
    }

    // Contact at the midpoint
    // 0.5 * (((center - sB*n) + closestPoint) + cA*n)
    let point = 0.5
        * mul_add(
            mul_sub(center, sphere_b.radius, normal) + closest_point,
            capsule_a.radius,
            normal,
        );

    // Manifold in frame A
    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = manifold.points;
    b3_assert(!pt.is_null());
    unsafe {
        (*pt).point = point;
        (*pt).separation = distance - total_radius;
        (*pt).pair = FEATURE_PAIR_SINGLE;
    }
}

/// Hull versus sphere.
pub fn collide_hull_and_sphere(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &Hull,
    sphere_b: Sphere,
    transform_b_to_a: Transform,
    cache: &mut SimplexCache,
) {
    manifold.point_count = 0;

    if capacity == 0 {
        return;
    }

    let center = transform_point(transform_b_to_a, sphere_b.center);

    let speculative_distance = b3::speculative_distance();

    // Work in shapeA coordinates

    let distance_input = DistanceInput {
        proxy_a: ShapeProxy::new(Some(hull_a.points().as_ptr()), hull_a.vertex_count(), 0.0),
        proxy_b: ShapeProxy::new(Some(&center as *const Vec3), 1, 0.0),
        transform: Transform::IDENTITY,
        use_radii: false,
    };

    let distance_output = shape_distance(&distance_input, cache, None);

    let radius_a: f32 = 0.0;
    let radius_b = sphere_b.radius;
    let radius = radius_a + radius_b;

    if distance_output.distance > radius + speculative_distance {
        // We found a separating axis
        *cache = SimplexCache::default();
        return;
    }

    let pt = manifold.points;
    b3_assert(!pt.is_null());

    if distance_output.distance > 100.0 * f32::EPSILON {
        // Shallow penetration
        let normal = normalize(distance_output.point_b - distance_output.point_a);

        // cA is the projection of the sphere center onto the hull (pointA if radiusA == 0).
        let c_a = mul_add(
            center,
            radius_a - dot(center - distance_output.point_a, normal),
            normal,
        );

        // cB is the deepest point on the sphere with respect to the reference face
        let c_b = mul_sub(center, radius_b, normal);

        let point = lerp(c_a, c_b, 0.5);

        // Manifold in frame A
        manifold.normal = normal;
        manifold.point_count = 1;

        unsafe {
            (*pt).point = point;
            (*pt).separation = distance_output.distance - radius;
            (*pt).pair = FEATURE_PAIR_SINGLE;
        }
    } else {
        // Deep penetration
        let mut best_index: i32 = -1;
        let mut best_distance = -f32::MAX;
        let planes = hull_a.planes();

        for index in 0..hull_a.face_count() as usize {
            let plane = planes[index];

            let distance = plane_separation(plane, center);
            if distance > best_distance {
                best_index = index as i32;
                best_distance = distance;
            }
        }
        b3_assert(best_index >= 0);

        let normal = planes[best_index as usize].normal;

        // cA is the projection of the sphere center onto the hull
        let c_a = mul_add(
            center,
            radius_a - dot(center - distance_output.point_a, normal),
            normal,
        );

        // cB is the deepest point on the sphere with respect to the reference face
        let c_b = mul_sub(center, radius_b, normal);

        let point = lerp(c_a, c_b, 0.5);

        // Manifold in frame A
        manifold.normal = normal;
        manifold.point_count = 1;

        unsafe {
            (*pt).point = point;
            (*pt).separation = best_distance - radius;
            (*pt).pair = FEATURE_PAIR_SINGLE;
        }
    }
}

// MARK: - Capsule contacts

/// Capsule versus capsule.
pub fn collide_capsules(
    manifold: &mut LocalManifold,
    capacity: i32,
    capsule_a: Capsule,
    capsule_b: Capsule,
    transform_b_to_a: Transform,
) {
    manifold.point_count = 0;

    if capacity < 2 {
        return;
    }

    // Work in shapeA coordinates
    let center_a1 = capsule_a.center1;
    let center_a2 = capsule_a.center2;
    let center_b1 = transform_point(transform_b_to_a, capsule_b.center1);
    let center_b2 = transform_point(transform_b_to_a, capsule_b.center2);

    let radius = capsule_a.radius + capsule_b.radius;
    let max_distance = radius + b3::speculative_distance();

    let result = segment_distance(center_a1, center_a2, center_b1, center_b2);
    let offset = result.point2 - result.point1;
    let distance_squared = length_squared(offset);
    let linear_slop = b3::linear_slop();
    let min_distance = 0.01 * linear_slop;

    if distance_squared > max_distance * max_distance || distance_squared < min_distance * min_distance
    {
        // We found a separating axis
        return;
    }

    let segment_a = center_a2 - center_a1;
    let (length_a, edge_a) = get_length_and_normalize(segment_a);
    if length_a < b3::min_capsule_length() {
        return;
    }

    let segment_b = center_b2 - center_b1;
    let (length_b, edge_b) = get_length_and_normalize(segment_b);
    if length_b < b3::min_capsule_length() {
        return;
    }

    // Parallel edges: |eA x eB| = sin(alpha)
    let alpha_tol: f32 = 0.05;
    let alpha_tol_sqr = alpha_tol * alpha_tol;
    let axis = cross(edge_a, edge_b);

    // Try to create two contact points if the capsules are nearly parallel
    if length_squared(axis) < alpha_tol_sqr {
        // Clip segment B against side planes of segment A

        // Side planes of A
        let planes_a = [
            Plane::new(-edge_a, -dot(edge_a, capsule_a.center1)),
            Plane::new(edge_a, dot(edge_a, capsule_a.center2)),
        ];

        // Clip points for B
        let mut vertices_b = [ClipVertex::default(); 2];
        vertices_b[0].position = center_b1;
        vertices_b[0].separation = 0.0;
        vertices_b[0].pair = make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0);
        vertices_b[1].position = center_b2;
        vertices_b[1].separation = 0.0;
        vertices_b[1].pair = make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1);

        let mut point_count = clip_segment(&mut vertices_b, planes_a[0]);
        if point_count == 2 {
            point_count = clip_segment(&mut vertices_b, planes_a[1]);
        }

        if point_count == 2 {
            // Closest points on A to the clipped points on B.
            let closest_point1 = point_to_segment_distance(center_a1, center_a2, vertices_b[0].position);
            let closest_point2 = point_to_segment_distance(center_a1, center_a2, vertices_b[1].position);

            let distance1 = distance(closest_point1, vertices_b[0].position);
            let distance2 = distance(closest_point2, vertices_b[1].position);
            if distance1 <= radius && distance2 <= radius {
                if distance1 < min_distance || distance2 < min_distance {
                    // Avoid divide by zero
                    return;
                }

                let normal1 = (1.0 / distance1) * (vertices_b[0].position - closest_point1);
                let normal2 = (1.0 / distance2) * (vertices_b[1].position - closest_point2);
                let normal = normalize(normal1 + normal2);
                let radius_a = capsule_a.radius;
                let radius_b = capsule_b.radius;

                // Contact is at the midpoint: 0.5 * (((vB.pos + rA*nK) + cP) - rB*n)
                let point1 = 0.5
                    * mul_sub(
                        mul_add(vertices_b[0].position, radius_a, normal1) + closest_point1,
                        radius_b,
                        normal,
                    );
                let point2 = 0.5
                    * mul_sub(
                        mul_add(vertices_b[1].position, radius_a, normal2) + closest_point2,
                        radius_b,
                        normal,
                    );

                // Manifold in frame A
                manifold.normal = normal;
                manifold.point_count = 2;

                let pt = manifold.points;
                b3_assert(!pt.is_null());
                unsafe {
                    (*pt.add(0)).point = point1;
                    (*pt.add(0)).separation = distance1 - radius;
                    (*pt.add(0)).pair = vertices_b[0].pair;

                    (*pt.add(1)).point = point2;
                    (*pt.add(1)).separation = distance2 - radius;
                    (*pt.add(1)).pair = vertices_b[1].pair;
                }

                return;
            }
        }
    }

    let (distance_value, normal) = get_length_and_normalize(offset);
    // Contact at the midpoint 0.5 * (((p1 + rA*n) + p2) - rB*n)
    let point = 0.5
        * mul_sub(
            mul_add(result.point1, capsule_a.radius, normal) + result.point2,
            capsule_b.radius,
            normal,
        );

    // Manifold in frame A
    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = manifold.points;
    b3_assert(!pt.is_null());
    unsafe {
        (*pt).point = point;
        (*pt).separation = distance_value - radius;
        (*pt).pair = FEATURE_PAIR_SINGLE;
    }
}

// MARK: - Hull and capsule

/// Query the face direction between a hull and a capsule.
pub fn query_face_direction_hull_and_capsule(
    hull: &Hull,
    capsule: Capsule,
    capsule_transform: Transform,
) -> SeparatingAxis {
    let mut max_face_index: i32 = -1;
    let mut max_vertex_index: i32 = -1;
    let mut max_face_separation = -f32::MAX;
    let planes = hull.planes();

    let capsule_points = [
        transform_point(capsule_transform, capsule.center1),
        transform_point(capsule_transform, capsule.center2),
    ];

    for face_index in 0..hull.face_count() as usize {
        let plane = planes[face_index];

        let vertex_index = get_point_support(&capsule_points, -plane.normal);
        let support = capsule_points[vertex_index as usize];
        let separation = plane_separation(plane, support);
        if separation > max_face_separation {
            max_vertex_index = vertex_index;
            max_face_index = face_index as i32;
            max_face_separation = separation;
        }
    }

    SeparatingAxis {
        normal: planes[max_face_index as usize].normal,
        separation: max_face_separation,
        index_a: max_face_index,
        index_b: max_vertex_index,
        kind: SeparatingFeature::FaceAxisA,
    }
}

/// Query the edge direction between a hull and a capsule.
pub fn query_edge_direction_hull_and_capsule(
    hull: &Hull,
    capsule: Capsule,
    capsule_transform: Transform,
) -> SeparatingAxis {
    // Find axis of minimum penetration
    let mut max_normal = Vec3::ZERO;
    let mut max_separation = -f32::MAX;
    let mut max_index_a: i32 = b3::NULL_INDEX;
    let mut max_index_b: i32 = b3::NULL_INDEX;

    // We perform all computations in local space of the hull
    let p_a = transform_point(capsule_transform, capsule.center1);
    let q_a = transform_point(capsule_transform, capsule.center2);
    let e_a = q_a - p_a;

    let edges = hull.edges();
    let points = hull.points();
    let planes = hull.planes();
    let squared_tolerance = b3::PARALLEL_EDGE_TOLERANCE * b3::PARALLEL_EDGE_TOLERANCE;

    let mut index = 0usize;
    while index < hull.half_edge_count() as usize {
        let edge = edges[index];
        let twin = edges[index + 1];
        b3_assert(edge.twin as usize == index + 1 && twin.twin as usize == index);

        let q_b = points[twin.origin as usize];
        let u_b = planes[edge.face as usize].normal;
        let v_b = planes[twin.face as usize].normal;

        // An isolated edge (e.g. like in a capsule) defines a circle through the origin on the
        // Gauss map. So testing for overlap between this circle and the arc AB simplifies to a
        // plane test.
        let cba = dot(u_b, e_a);
        let dba = dot(v_b, e_a);

        if cba * dba < 0.0 {
            // Avoid nearly parallel edges that may lead to invalid separation values at the noise
            // floor.
            if max_float(cba * cba, dba * dba) < squared_tolerance * length_squared(e_a) {
                index += 2;
                continue;
            }

            // The intersection of the arcs on the Gauss map is the edge pair axis. Cast the arc of
            // hull B (from uB to vB) against the plane containing the arc of hull A:
            // dot(uB + t * (vB - uB), eA) == 0
            // then t = cba / (cba - dba)
            let t = cba / (cba - dba);
            let mut axis = lerp(u_b, v_b, t);
            b3_validate(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
            axis = normalize(axis);
            let separation = dot(axis, q_a - q_b);

            if separation > max_separation {
                // Note: We don't exit early if we find a separating axis here since we want to
                // find the best one for caching and account for the convex radius later.
                max_normal = axis;
                max_separation = separation;
                max_index_a = 0;
                max_index_b = index as i32;
            }
        }

        index += 2;
    }

    // Save result
    SeparatingAxis {
        normal: max_normal,
        separation: max_separation,
        index_a: max_index_a,
        index_b: max_index_b,
        kind: SeparatingFeature::EdgePairAxis,
    }
}

/// Build a two point contact between a hull face and a capsule.
pub fn build_hull_face_and_capsule_contact(
    manifold: &mut LocalManifold,
    hull_a: &Hull,
    capsule_b: Capsule,
    transform_b_to_a: Transform,
    query: SeparatingAxis,
) -> bool {
    // Work in shapeA coordinates
    let planes = hull_a.planes();

    // Clip the capsule edge against the side planes of the reference face
    let ref_face = query.index_a;
    let ref_plane = planes[ref_face as usize];

    let mut segment_b = [ClipVertex::default(); 2];
    segment_b[0].position = transform_point(transform_b_to_a, capsule_b.center1);
    segment_b[0].separation = 0.0;
    segment_b[0].pair = make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0);
    segment_b[1].position = transform_point(transform_b_to_a, capsule_b.center2);
    segment_b[1].separation = 0.0;
    segment_b[1].pair = make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1);

    let point_count = clip_segment_to_hull_face(&mut segment_b, hull_a, ref_face);
    if point_count < 2 {
        return false;
    }

    let distance1 = plane_separation(ref_plane, segment_b[0].position);
    let distance2 = plane_separation(ref_plane, segment_b[1].position);
    let speculative_distance = b3::speculative_distance();

    if distance1 <= speculative_distance || distance2 <= speculative_distance {
        let normal = ref_plane.normal;
        let point1 = mul_sub(
            segment_b[0].position,
            0.5 * (distance1 + capsule_b.radius),
            normal,
        );
        let point2 = mul_sub(
            segment_b[1].position,
            0.5 * (distance2 + capsule_b.radius),
            normal,
        );

        // Manifold in frame A
        manifold.normal = normal;
        manifold.point_count = 2;

        let pt = manifold.points;
        b3_assert(!pt.is_null());
        unsafe {
            (*pt.add(0)).point = point1;
            (*pt.add(0)).separation = distance1 - capsule_b.radius;
            (*pt.add(0)).pair = segment_b[0].pair;

            (*pt.add(1)).point = point2;
            (*pt.add(1)).separation = distance2 - capsule_b.radius;
            (*pt.add(1)).pair = segment_b[1].pair;
        }

        return true;
    }

    false
}

/// Build a single point edge contact between a hull edge and a capsule.
pub fn build_hull_and_capsule_edge_contact(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &Hull,
    capsule_b: Capsule,
    transform_b_to_a: Transform,
    query: SeparatingAxis,
) -> bool {
    if capacity < 1 {
        return false;
    }

    // Work in shapeA coordinates

    let pc = transform_point(transform_b_to_a, capsule_b.center1);
    let qc = transform_point(transform_b_to_a, capsule_b.center2);
    let ec = qc - pc;

    let edges = hull_a.edges();
    let points = hull_a.points();

    let edge2 = edges[query.index_b as usize];
    let twin2 = edges[edge2.twin as usize];
    let ph = points[edge2.origin as usize];
    let qh = points[twin2.origin as usize];
    let eh = qh - ph;

    let normal = query.normal;

    let result = line_distance(ph, eh, pc, ec);

    if !is_within_segments(&result) {
        // closest point beyond end points
        return false;
    }

    let point = 0.5 * (mul_sub(result.point1, capsule_b.radius, normal) + result.point2);
    let separation = dot(normal, result.point2 - result.point1);
    b3_validate(abs_float(separation - query.separation) < b3::linear_slop());

    // Manifold in frame A
    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = manifold.points;
    b3_assert(!pt.is_null());
    unsafe {
        (*pt).point = point;
        (*pt).separation = separation - capsule_b.radius;
        (*pt).pair = make_feature_pair(
            FeatureOwner::ShapeA,
            query.index_a,
            FeatureOwner::ShapeB,
            query.index_b,
        );
    }
    true
}

/// Hull versus capsule.
pub fn collide_hull_and_capsule(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &Hull,
    capsule_b: Capsule,
    transform_b_to_a: Transform,
    cache: &mut SimplexCache,
) {
    manifold.point_count = 0;

    if capacity < 2 {
        return;
    }

    // Work in shapeA coordinates
    let capsule_points = [capsule_b.center1, capsule_b.center2];

    let distance_input = DistanceInput {
        proxy_a: ShapeProxy::new(Some(hull_a.points().as_ptr()), hull_a.vertex_count(), 0.0),
        proxy_b: ShapeProxy::new(Some(capsule_points.as_ptr()), 2, 0.0),
        transform: transform_b_to_a,
        use_radii: false,
    };

    let distance_output = shape_distance(&distance_input, cache, None);

    let speculative_distance = b3::speculative_distance();

    if distance_output.distance > capsule_b.radius + speculative_distance {
        // We found a separating axis
        *cache = SimplexCache::default();
        return;
    }

    let pt = manifold.points;
    b3_assert(!pt.is_null());

    if distance_output.distance > 100.0 * f32::EPSILON {
        let planes = hull_a.planes();

        // Shallow penetration
        let delta = distance_output.normal;
        let ref_face = find_hull_support_face(hull_a, delta);
        let ref_plane = planes[ref_face as usize];

        // Try to create two contact points if closest points difference is nearly parallel to face
        // normal
        let k_tolerance: f32 = 0.998;
        if abs_float(dot(ref_plane.normal, delta)) > k_tolerance {
            // Clip capsule segment against side planes of reference face
            let mut vertices_b = [ClipVertex::default(); 2];
            vertices_b[0].position = transform_point(transform_b_to_a, capsule_b.center1);
            vertices_b[0].separation = 0.0;
            vertices_b[0].pair =
                make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0);
            vertices_b[1].position = transform_point(transform_b_to_a, capsule_b.center2);
            vertices_b[1].separation = 0.0;
            vertices_b[1].pair =
                make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1);

            let point_count = clip_segment_to_hull_face(&mut vertices_b, hull_a, ref_face);

            if point_count == 2 {
                let distance1 = plane_separation(ref_plane, vertices_b[0].position);
                let distance2 = plane_separation(ref_plane, vertices_b[1].position);
                if distance1 <= capsule_b.radius + speculative_distance
                    || distance2 <= capsule_b.radius + speculative_distance
                {
                    let normal = ref_plane.normal;
                    let point1 =
                        mul_sub(vertices_b[0].position, 0.5 * (capsule_b.radius + distance1), normal);
                    let point2 =
                        mul_sub(vertices_b[1].position, 0.5 * (capsule_b.radius + distance2), normal);

                    // Manifold in frame A
                    manifold.normal = normal;
                    manifold.point_count = 2;

                    unsafe {
                        (*pt.add(0)).point = point1;
                        (*pt.add(0)).separation = distance1 - capsule_b.radius;
                        (*pt.add(0)).pair = vertices_b[0].pair;

                        (*pt.add(1)).point = point2;
                        (*pt.add(1)).separation = distance2 - capsule_b.radius;
                        (*pt.add(1)).pair = vertices_b[1].pair;
                    }

                    return;
                }
            }
        }

        // Create contact from closest points
        let point = 0.5
            * (mul_sub(distance_output.point_a, capsule_b.radius, delta) + distance_output.point_b);

        // Manifold in frame A
        manifold.normal = delta;
        manifold.point_count = 1;

        unsafe {
            (*pt).point = point;
            (*pt).separation = distance_output.distance - capsule_b.radius;
            (*pt).pair = FEATURE_PAIR_SINGLE;
        }
        return;
    }

    // Deep penetration

    let face_query = query_face_direction_hull_and_capsule(hull_a, capsule_b, transform_b_to_a);
    if face_query.separation > capsule_b.radius {
        // We found a separating axis
        return;
    }

    let edge_query = query_edge_direction_hull_and_capsule(hull_a, capsule_b, transform_b_to_a);
    if edge_query.separation > capsule_b.radius {
        // We found a separating axis
        return;
    }

    // Create face contact
    let mut face_separation = face_query.separation - capsule_b.radius;
    let _ = build_hull_face_and_capsule_contact(manifold, hull_a, capsule_b, transform_b_to_a, face_query);
    b3_validate(manifold.point_count == 0 || manifold.point_count == 2);
    if manifold.point_count == 2 {
        // This becomes the clipped separation.
        unsafe {
            face_separation = min_float((*pt).separation, (*pt.add(1)).separation);
        }
    }

    // Is there a valid edge-edge axis?
    if edge_query.index_a == b3::NULL_INDEX {
        return;
    }

    // Face contact can be empty if it does not realize the axis of minimum penetration.
    // Create edge contact if face contact fails or edge contact is significantly better!
    let linear_slop = b3::linear_slop();
    let edge_separation = edge_query.separation - capsule_b.radius;
    if manifold.point_count == 0 || edge_separation > face_separation + linear_slop {
        // Edge contact
        let _ = build_hull_and_capsule_edge_contact(
            manifold,
            capacity,
            hull_a,
            capsule_b,
            transform_b_to_a,
            edge_query,
        );
    }
}
