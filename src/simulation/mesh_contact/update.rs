//! Mesh narrow-phase entry point: cache refresh, triangle collision, selection, clustering,
//! persistence, and material blending for one mesh contact.
//!
//! Translated from `Sources/Natsel/Simulation/MeshContact/Update.swift`.

use std::sync::atomic::{AtomicBool, Ordering};

use crate::base::core::internal_assert;
use crate::math::types::WorldTransform;
use crate::simulation::contact_payload::{ContactManifolds, MeshContactScratch};
use crate::simulation::contact_store::ContactStore;
use crate::simulation::shape_runtime::Shape;
use crate::simulation::triangle_surface::TriangleSurface;
use crate::simulation::world_runtime::World;
use crate::types::shape_def::ShapeType;

use super::persistence::persist_mesh_manifolds;
use super::selection::{cluster_mesh_manifolds, select_mesh_manifolds};

/// Mesh narrow-phase trace switch (`nonisolated(unsafe) public var meshTraceEnabled`). A relaxed
/// atomic stands in for the Swift global so the flag is settable from the benchmarks and tests
/// without `static mut`.
pub static MESH_TRACE_ENABLED: AtomicBool = AtomicBool::new(false);

#[inline]
fn trace_hex(value: f32) -> String {
    format!("{:x}", value.to_bits())
}

impl World {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn compute_mesh_manifolds(
        &self,
        contact_id: i32,
        shape_a: &Shape,
        transform_a: WorldTransform,
        shape_b: &Shape,
        transform_b: WorldTransform,
        is_fast: bool,
        material_indices: Option<&[u16]>,
        scratch: &mut MeshContactScratch,
        cs: &mut ContactStore,
    ) -> bool {
        let surface = match TriangleSurface::from_geometry(&shape_a.geometry) {
            Some(surface) => surface,
            None => return false,
        };
        let mut contact = cs.get(contact_id);
        contact
            .mesh_contact_mut()
            .refresh(&surface, transform_a, shape_b.aabb);
        let triangles = self.collide_mesh_triangles(
            &mut contact,
            &surface,
            transform_a,
            shape_b,
            transform_b,
            is_fast,
            scratch,
        );
        let accepted = select_mesh_manifolds(
            &triangles,
            shape_b.type_ == ShapeType::SphereShape,
            scratch,
        );
        if MESH_TRACE_ENABLED.load(Ordering::Relaxed)
            && contact.shape_id_a == 0
            && contact.shape_id_b == 2
        {
            println!(
                "T count={} ids=[{}]",
                contact.mesh_contact().triangles.len(),
                contact
                    .mesh_contact()
                    .triangles
                    .iter()
                    .map(|entry| entry.triangle_index.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
            for m in &triangles {
                println!(
                    "T tri {} f={} d={} n={} {} {} p=[{}]",
                    m.triangle_index,
                    m.feature as u32,
                    trace_hex(m.squared_distance),
                    trace_hex(m.normal.x),
                    trace_hex(m.normal.y),
                    trace_hex(m.normal.z),
                    m.points
                        .iter()
                        .map(|p| format!("\"{},{}\"", trace_hex(p.point.x), trace_hex(p.separation)))
                        .collect::<Vec<_>>()
                        .join(", ")
                );
            }
            for m in accepted {
                println!("T acc {}", m.triangle_index);
            }
        }
        if accepted.is_empty() {
            contact.manifolds().remove_all(false);
            contact.manifold_count = 0;
            cs.set(contact_id, contact);
            return false;
        }
        // The selection lives inside `scratch`; hand the cluster stage an owned copy so the next
        // `&mut scratch` borrow and the cluster input do not overlap (Swift's COW arrays aliased
        // freely). `select_mesh_manifolds` sizes `selected_manifolds` to exactly the accepted set.
        let accepted = std::mem::take(&mut scratch.selected_manifolds);
        let clusters = cluster_mesh_manifolds(&accepted, scratch);
        let cluster_count = clusters.len();
        let mut clusters = std::mem::take(&mut scratch.clusters);
        clusters.truncate(cluster_count);
        let payload = match contact.payload {
            Some(payload) => payload,
            None => internal_assert("contact payload is null", file!(), line!()),
        };
        // SAFETY: `payload` addresses an initialized `ContactPayload` slot owned by the payload
        // pool for as long as the contact exists; the read ends before `replace_with` below
        // overwrites the slot, mirroring Swift's `payload!.pointee.manifolds` read.
        let previous: &ContactManifolds = unsafe { &(*payload).manifolds };
        let persisted = persist_mesh_manifolds(
            &clusters,
            previous,
            transform_a,
            transform_b,
            scratch,
        );
        contact.manifolds().replace_with(persisted);
        let manifold_count = contact.manifolds().count();
        contact.manifold_count = manifold_count as i32;
        self.mix_mesh_materials(
            &mut contact,
            &surface,
            shape_a,
            transform_a,
            shape_b,
            transform_b,
            material_indices,
        );
        cs.set(contact_id, contact);
        true
    }
}
