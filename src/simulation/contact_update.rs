//! Convex contact update.
//!
//! Translated from `Sources/Natsel/Simulation/ContactUpdate.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::compound_contact`: `World::update_compound_contact(contact_id, shape_a,
//!   xf_a, shape_b, xf_b, is_fast, scratch, cs) -> bool` (Swift passes the two shapes by value;
//!   the Rust entry point borrows them since every use is a read or an owned local copy).
//! - `crate::simulation::mesh_contact::update`: `World::compute_mesh_manifolds(contact_id,
//!   shape_a, transform_a, shape_b, transform_b, is_fast, material_indices, scratch, cs) -> bool`
//!   where `material_indices: Option<&[u16]>` replaces the Swift defaulted `materialIndices:
//!   [UInt16]? = nil` parameter (this call site passes `None`).
//! - `crate::simulation::contact_solver`: `World::state_at` / `World::set_state_at` /
//!   `World::with_solver_states` buffer helpers (not used by this file, listed because the two
//!   contact files share the solver vocabulary).

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::convex_manifold::collide_hulls_with_scratch;
use crate::collision::convex_manifold_shapes::{
    collide_capsule_and_sphere, collide_capsules, collide_hull_and_capsule,
    collide_hull_and_sphere, collide_spheres,
};
use crate::collision::manifold::make_feature_id;
use crate::math::functions::{
    inv_mul_world_transforms, make_matrix_from_quat, max_float, mul_mv, offset_pos, rotate_vector,
    sub_pos,
};
use crate::math::types::{Vec3, WorldTransform};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::contact_payload::{ContactManifolds, MeshContactScratch};
use crate::simulation::contact_store::ContactStore;
use crate::simulation::shape_runtime::{Shape, ShapeFlags, ShapeGeometry};
use crate::simulation::world_runtime::World;
use crate::types::collision_types::{LocalManifold, LocalManifoldPoint, Manifold, ManifoldPoint};
use crate::types::ids::ShapeID;
use crate::types::shape_def::ShapeType;

impl World {
    /// Compute a new manifold for a convex shape pair, recycling impulses by feature id.
    /// Returns true if the shapes are touching.
    pub(crate) fn compute_convex_manifold(
        &self,
        contact_id: i32,
        shape_a: &Shape,
        xf_a: WorldTransform,
        shape_b: &Shape,
        xf_b: WorldTransform,
        scratch: &mut MeshContactScratch,
        cs: &mut ContactStore,
    ) -> bool {
        cs.with_contact(contact_id, |contact| {
            let type_a = shape_a.type_;
            let type_b = shape_b.type_;

            // Swift wrapped this storage in `withInlineBuffer(&pointBuffer)`; in Rust the array
            // is the storage and the pointer below addresses it for exactly the same span.
            let mut point_buffer: [LocalManifoldPoint; 4] =
                [LocalManifoldPoint::default(); 4];
            let mut geom_manifold = LocalManifold::default();
            let point_capacity = b3::MAX_MANIFOLD_POINTS;

            let transform_b_to_a = inv_mul_world_transforms(xf_a, xf_b);

            // SAFETY: `point_buffer` is a live local with `b3::MAX_MANIFOLD_POINTS` elements and
            // is not moved or mutated through any other path while `geom_manifold.points` is read
            // by the collide functions, exactly the guarantee Swift's `withInlineBuffer` closure
            // gave. All reads below are bounded by `geom_manifold.point_count`, which every
            // collide routine keeps within `point_capacity`.
            geom_manifold.points = point_buffer.as_mut_ptr();

            if type_a == ShapeType::SphereShape {
                b3_assert(type_b == ShapeType::SphereShape);
                collide_spheres(
                    &mut geom_manifold,
                    point_capacity,
                    shape_a.sphere(),
                    shape_b.sphere(),
                    transform_b_to_a,
                );
            } else if type_a == ShapeType::CapsuleShape {
                if type_b == ShapeType::SphereShape {
                    collide_capsule_and_sphere(
                        &mut geom_manifold,
                        point_capacity,
                        shape_a.capsule(),
                        shape_b.sphere(),
                        transform_b_to_a,
                    );
                } else {
                    b3_assert(type_b == ShapeType::CapsuleShape);
                    collide_capsules(
                        &mut geom_manifold,
                        point_capacity,
                        shape_a.capsule(),
                        shape_b.capsule(),
                        transform_b_to_a,
                    );
                }
            } else {
                b3_assert(type_a == ShapeType::HullShape);

                // SAFETY: the type assertion above and `Shape::hull`'s invariant (`HullShape`
                // always carries `ShapeGeometry::Hull`) make the unwrap point the Swift `!`
                // would force-trap on unreachable, mirroring Swift's trap-on-nil.
                let hull_a = match shape_a.hull() {
                    Some(hull) => hull,
                    None => {
                        crate::base::core::internal_assert("hull geometry expected", file!(), line!())
                    }
                };
                if type_b == ShapeType::SphereShape {
                    collide_hull_and_sphere(
                        &mut geom_manifold,
                        point_capacity,
                        hull_a,
                        shape_b.sphere(),
                        transform_b_to_a,
                        &mut contact.simplex_cache,
                    );
                } else if type_b == ShapeType::CapsuleShape {
                    collide_hull_and_capsule(
                        &mut geom_manifold,
                        point_capacity,
                        hull_a,
                        shape_b.capsule(),
                        transform_b_to_a,
                        &mut contact.simplex_cache,
                    );
                } else {
                    b3_assert(type_b == ShapeType::HullShape);
                    let hull_b = match shape_b.hull() {
                        Some(hull) => hull,
                        None => {
                            crate::base::core::internal_assert("hull geometry expected", file!(), line!())
                        }
                    };
                    collide_hulls_with_scratch(
                        &mut geom_manifold,
                        point_capacity,
                        hull_a,
                        hull_b,
                        transform_b_to_a,
                        scratch,
                        &mut contact.sat_cache,
                    );
                    scratch.sat_calls += 1;
                    scratch.sat_cache_hits += contact.sat_cache.hit as i32;
                }
            }

            if geom_manifold.point_count == 0 {
                if contact.manifold_count > 0 {
                    contact.manifolds().remove_all(false);
                    contact.manifold_count = 0;
                }

                return false;
            }

            let mut old_points: [ManifoldPoint; 4] = [ManifoldPoint::default(); 4];
            let mut old_count: i32 = 0;

            if contact.manifold_count == 0 {
                contact
                    .manifolds()
                    .replace_with(ContactManifolds::from_manifold(Manifold::default()));
                contact.manifold_count = 1;
            } else {
                let mview = contact.manifolds();
                let first = mview.get(0);
                old_count = first.point_count;
                for i in 0..old_count as usize {
                    old_points[i] = first.points[i];
                }
            }

            b3_assert(
                0 < geom_manifold.point_count && geom_manifold.point_count <= b3::MAX_MANIFOLD_POINTS,
            );

            let mview = contact.manifolds();
            let manifold = mview.get_mut(0);

            manifold.point_count = geom_manifold.point_count;

            let matrix_a = make_matrix_from_quat(xf_a.q);
            manifold.normal = mul_mv(matrix_a, geom_manifold.normal);

            // Store point data in contact
            for i in 0..geom_manifold.point_count as usize {
                // SAFETY: `geom_manifold.points` addresses `point_buffer` (see above) and
                // `i < geom_manifold.point_count <= MAX_MANIFOLD_POINTS`.
                let source = unsafe { *geom_manifold.points.add(i) };
                let mut target = ManifoldPoint::default();

                // Contact points are computed in frame A
                target.anchor_a = mul_mv(matrix_a, source.point);
                target.anchor_b = target.anchor_a + sub_pos(xf_a.p, xf_b.p);
                target.separation = source.separation;
                target.feature_id = make_feature_id(&source.pair);
                target.triangle_index = b3::NULL_INDEX;
                target.normal_velocity = 0.0;

                manifold.points[i] = target;
            }

            // Copy impulses from old points
            for i in 0..geom_manifold.point_count as usize {
                let mut pt2 = manifold.points[i];
                pt2.total_normal_impulse = 0.0;
                pt2.persisted = false;

                for j in 0..old_count as usize {
                    let mut pt1 = old_points[j];

                    if pt2.feature_id == pt1.feature_id {
                        pt2.normal_impulse = pt1.normal_impulse;
                        pt2.persisted = true;

                        // claimed
                        pt1.feature_id = u32::MAX;
                        old_points[j] = pt1;

                        break;
                    }
                }

                if !pt2.persisted {
                    pt2.normal_impulse = 0.0;
                }

                manifold.points[i] = pt2;
            }

            true
        })
    }

    /// Update a convex contact and mix materials.
    pub(crate) fn update_convex_contact(
        &self,
        contact_id: i32,
        shape_a: &Shape,
        xf_a: WorldTransform,
        shape_b: &Shape,
        xf_b: WorldTransform,
        flip: bool,
        scratch: &mut MeshContactScratch,
        cs: &mut ContactStore,
    ) -> bool {
        // Compute new manifold
        let touching = self.compute_convex_manifold(contact_id, shape_a, xf_a, shape_b, xf_b, scratch, cs);

        if !touching {
            b3_assert(cs.get(contact_id).manifold_count == 0);
            return false;
        }

        cs.with_contact(contact_id, |contact| {
            b3_assert(contact.manifold_count == 1);

            if flip {
                // Not flipping the feature ids because they just need to match and flipping is
                // consistent.
                let mut manifold = *contact.manifolds().get(0);
                manifold.normal = -manifold.normal;
                let point_count = manifold.point_count;
                for i in 0..point_count as usize {
                    let mut mp = manifold.points[i];
                    std::mem::swap(&mut mp.anchor_a, &mut mp.anchor_b);
                    manifold.points[i] = mp;
                }
                *contact.manifolds().get_mut(0) = manifold;
            }

            let material_a = shape_a.material(0);
            let material_b = shape_b.material(0);

            // Keep these updated in case the values on the shapes are modified
            contact.friction = self.mix_friction(
                material_a.friction,
                material_a.user_material_id,
                material_b.friction,
                material_b.user_material_id,
            );
            contact.restitution = self.mix_restitution(
                material_a.restitution,
                material_a.user_material_id,
                material_b.restitution,
                material_b.user_material_id,
            );

            if material_a.rolling_resistance > 0.0 || material_b.rolling_resistance > 0.0 {
                let radius = |shape: &Shape| -> f32 {
                    match shape.geometry {
                        ShapeGeometry::Sphere(ref sphere) => sphere.radius,
                        ShapeGeometry::Capsule(ref capsule) => capsule.radius,
                        ShapeGeometry::Hull(ref hull) => 0.25 * hull.inner_radius(),
                        _ => 0.0,
                    }
                };

                let radius_a = radius(shape_a);
                let radius_b = radius(shape_b);
                let max_radius = max_float(radius_a, radius_b);
                contact.rolling_resistance =
                    max_float(material_a.rolling_resistance, material_b.rolling_resistance) * max_radius;
            } else {
                contact.rolling_resistance = 0.0;
            }

            let tangent_velocity_a = rotate_vector(xf_a.q, material_a.tangent_velocity);
            let tangent_velocity_b = rotate_vector(xf_b.q, material_b.tangent_velocity);
            contact.tangent_velocity = tangent_velocity_a - tangent_velocity_b;

            if let Some(pre_solve) = &self.pre_solve {
                if (contact.flags & ContactFlags::SIM_ENABLE_PRE_SOLVE_EVENTS.0) != 0 {
                    let id_a = ShapeID::new(
                        shape_a.id + 1,
                        self.world_id,
                        shape_a.generation,
                    );
                    let id_b = ShapeID::new(
                        shape_b.id + 1,
                        self.world_id,
                        shape_b.generation,
                    );

                    // This call assumes thread safety
                    let point = offset_pos(xf_a.p, contact.manifolds().get(0).points[0].anchor_a);
                    let normal = contact.manifolds().get(0).normal;
                    if !pre_solve(id_a, id_b, point, normal) {
                        // Disable the contact
                        contact.manifolds().remove_all(false);
                        contact.manifold_count = 0;

                        // Invalidate the recycling cache.
                        contact.flags &= !ContactFlags::RELATIVE_TRANSFORM_VALID.0;
                        return false;
                    }
                }
            }

            if (shape_a.flags & ShapeFlags::ENABLE_HIT_EVENTS.0) != 0
                || (shape_b.flags & ShapeFlags::ENABLE_HIT_EVENTS.0) != 0
            {
                contact.flags |= ContactFlags::SIM_ENABLE_HIT_EVENT.0;
            } else {
                contact.flags &= !ContactFlags::SIM_ENABLE_HIT_EVENT.0;
            }

            true
        })
    }

    /// Compatibility wrapper for callers that do not own mesh scratch.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn update_contact_default_scratch(
        &mut self,
        contact_id: i32,
        local_center_a: Vec3,
        xf_a: WorldTransform,
        local_center_b: Vec3,
        xf_b: WorldTransform,
        is_fast: bool,
        cs: &mut ContactStore,
    ) -> bool {
        // SAFETY: mirrors Swift's `withUnsafeMutablePointer(to: &meshContactScratch)`: the pointer
        // addresses the world-owned `mesh_contact_scratch` field for the duration of the call and
        // nothing else aliases it while `update_contact` borrows the world immutably. Swift relied
        // on the field borrow and the `self` borrow not being checked against each other; the raw
        // pointer records the same discipline.
        let scratch: *mut MeshContactScratch = &mut self.mesh_contact_scratch;
        unsafe {
            self.update_contact(
                contact_id,
                local_center_a,
                xf_a,
                local_center_b,
                xf_b,
                is_fast,
                &mut *scratch,
                cs,
            )
        }
    }

    /// Update the contact manifold and touching status.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn update_contact(
        &self,
        contact_id: i32,
        local_center_a: Vec3,
        xf_a: WorldTransform,
        local_center_b: Vec3,
        xf_b: WorldTransform,
        is_fast: bool,
        scratch: &mut MeshContactScratch,
        cs: &mut ContactStore,
    ) -> bool {
        let (shape_index_a, shape_index_b) = cs.with_contact(contact_id, |contact| {
            (contact.shape_id_a as usize, contact.shape_id_b as usize)
        });

        let shape_a = &self.shapes[shape_index_a];
        let shape_b = &self.shapes[shape_index_b];

        b3_assert(shape_b.type_ != ShapeType::CompoundShape);

        let type_a = shape_a.type_;

        let touching = if type_a == ShapeType::CompoundShape {
            self.update_compound_contact(contact_id, shape_a, xf_a, shape_b, xf_b, is_fast, scratch, cs)
        } else if type_a == ShapeType::MeshShape || type_a == ShapeType::HeightShape {
            let touching = self.compute_mesh_manifolds(
                contact_id,
                shape_a,
                xf_a,
                shape_b,
                xf_b,
                is_fast,
                None,
                scratch,
                cs,
            );
            let enable_hit =
                touching && ((shape_a.flags | shape_b.flags) & ShapeFlags::ENABLE_HIT_EVENTS.0) != 0;
            cs.with_contact(contact_id, |contact| {
                if enable_hit {
                    contact.flags |= ContactFlags::SIM_ENABLE_HIT_EVENT.0;
                } else {
                    contact.flags &= !ContactFlags::SIM_ENABLE_HIT_EVENT.0;
                }
            });
            touching
        } else {
            // Convex-vs-convex
            self.update_convex_contact(contact_id, shape_a, xf_a, shape_b, xf_b, false, scratch, cs)
        };

        if touching {
            let center_a = rotate_vector(xf_a.q, local_center_a);
            let center_b = rotate_vector(xf_b.q, local_center_b);

            // Adjust anchors to be relative to center of mass, editing the stored contact in place.
            cs.with_contact(contact_id, |contact| {
                let manifold_count = contact.manifold_count as usize;
                let mview = contact.manifolds();
                for i in 0..manifold_count {
                    let manifold = mview.get_mut(i);
                    for j in 0..manifold.point_count as usize {
                        manifold.points[j].anchor_a = manifold.points[j].anchor_a - center_a;
                        manifold.points[j].anchor_b = manifold.points[j].anchor_b - center_b;
                    }
                }
                contact.flags |= ContactFlags::SIM_TOUCHING.0;
            });
        } else {
            cs.with_contact(contact_id, |contact| {
                contact.flags &= !ContactFlags::SIM_TOUCHING.0
            });
        }

        touching
    }
}
