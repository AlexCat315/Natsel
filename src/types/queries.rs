//! Public query types.

use crate::math::types::{AABB, Plane, Pos, Quat, Transform, Vec3};
use crate::types::ids::ShapeID;

/// Low level ray cast input data.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct RayCastInput {
    pub origin: Vec3,
    pub translation: Vec3,
    pub max_fraction: f32,
}

impl RayCastInput {
    pub fn new(origin: Vec3, translation: Vec3, max_fraction: f32) -> Self {
        Self {
            origin,
            translation,
            max_fraction,
        }
    }
}

/// Result from the closest ray cast.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RayResult {
    pub shape_id: ShapeID,
    pub point: Pos,
    pub normal: Vec3,
    pub user_material_id: u64,
    pub fraction: f32,
    pub triangle_index: i32,
    pub child_index: i32,
    pub node_visits: i32,
    pub leaf_visits: i32,
    pub hit: bool,
}

/// A shape proxy is used by the GJK algorithm. It can represent a convex shape.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShapeProxy {
    pub points: Option<*const Vec3>,
    pub count: i32,
    pub radius: f32,
}

unsafe impl Send for ShapeProxy {}

impl ShapeProxy {
    pub fn new(points: Option<*const Vec3>, count: i32, radius: f32) -> Self {
        Self {
            points,
            count,
            radius,
        }
    }

    pub fn points(&self) -> &[Vec3] {
        match self.points {
            Some(p) if self.count > 0 => unsafe {
                std::slice::from_raw_parts(p, self.count as usize)
            },
            _ => &[],
        }
    }
}

/// Low level shape cast input in generic form.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShapeCastInput {
    pub proxy: ShapeProxy,
    pub translation: Vec3,
    pub max_fraction: f32,
    pub can_encroach: bool,
}

/// Input for sweeping an AABB through a dynamic tree.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct BoxCastInput {
    pub box_: AABB,
    pub translation: Vec3,
    pub max_fraction: f32,
}

/// Low level ray cast or shape-cast output data.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct CastOutput {
    pub normal: Vec3,
    pub point: Vec3,
    pub fraction: f32,
    pub iterations: i32,
    pub triangle_index: i32,
    pub child_index: i32,
    pub material_index: i32,
    pub hit: bool,
}

/// Body cast result for ray and shape casts.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BodyCastResult {
    pub shape_id: ShapeID,
    pub point: Pos,
    pub normal: Vec3,
    pub fraction: f32,
    pub triangle_index: i32,
    pub user_material_id: u64,
    pub iterations: i32,
    pub hit: bool,
}

/// Used to warm start the GJK simplex.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SimplexCache {
    pub metric: f32,
    pub count: u16,
    pub index_a: [u8; 4],
    pub index_b: [u8; 4],
}

/// Input parameters for shape casts between two proxies.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShapeCastPairInput {
    pub proxy_a: ShapeProxy,
    pub proxy_b: ShapeProxy,
    pub transform: Transform,
    pub translation_b: Vec3,
    pub max_fraction: f32,
    pub can_encroach: bool,
}

/// Input for shape distance.
#[derive(Clone, Copy, Debug, Default)]
pub struct DistanceInput {
    pub proxy_a: ShapeProxy,
    pub proxy_b: ShapeProxy,
    pub transform: Transform,
    pub use_radii: bool,
}

/// Output for shape distance.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct DistanceOutput {
    pub point_a: Vec3,
    pub point_b: Vec3,
    pub normal: Vec3,
    pub distance: f32,
    pub iterations: i32,
    pub simplex_count: i32,
}

/// Simplex vertex for debugging the GJK algorithm.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SimplexVertex {
    pub wa: Vec3,
    pub wb: Vec3,
    pub w: Vec3,
    pub a: f32,
    pub index_a: i32,
    pub index_b: i32,
}

/// Simplex from the GJK algorithm.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Simplex {
    pub vertices: [SimplexVertex; 4],
    pub count: i32,
}

/// Motion of a body/shape for TOI computation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Sweep {
    pub local_center: Vec3,
    pub c1: Vec3,
    pub c2: Vec3,
    pub q1: Quat,
    pub q2: Quat,
}

impl Sweep {
    pub fn new(local_center: Vec3, c1: Vec3, c2: Vec3, q1: Quat, q2: Quat) -> Self {
        Self {
            local_center,
            c1,
            c2,
            q1,
            q2,
        }
    }
}

/// Time of impact input.
#[derive(Clone, Copy, Debug, Default)]
pub struct TOIInput {
    pub proxy_a: ShapeProxy,
    pub proxy_b: ShapeProxy,
    pub sweep_a: Sweep,
    pub sweep_b: Sweep,
    pub max_fraction: f32,
}

/// Describes the TOI output state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum TOIState {
    #[default]
    Unknown = 0,
    Failed,
    Overlapped,
    Hit,
    Separated,
}

/// Time of impact output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct TOIOutput {
    pub state: TOIState,
    pub point: Vec3,
    pub normal: Vec3,
    pub fraction: f32,
    pub distance: f32,
    pub distance_iterations: i32,
    pub push_back_iterations: i32,
    pub root_iterations: i32,
    pub used_fallback: bool,
}

// MARK: - Character mover

/// The plane between a character mover and a shape.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct PlaneResult {
    pub plane: Plane,
    pub point: Vec3,
    pub triangle_index: i32,
    pub child_index: i32,
    pub material_index: i32,
}

/// Collision planes that can be fed to `solve_planes`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct CollisionPlane {
    pub plane: Plane,
    pub push_limit: f32,
    pub push: f32,
    pub clip_velocity: bool,
}

impl CollisionPlane {
    pub fn new(plane: Plane, push_limit: f32, push: f32, clip_velocity: bool) -> Self {
        Self {
            plane,
            push_limit,
            push,
            clip_velocity,
        }
    }
}

/// Result returned by `solve_planes`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlaneSolverResult {
    pub delta: Vec3,
    pub iteration_count: i32,
}

/// Body plane result for movers.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct BodyPlaneResult {
    pub shape_id: ShapeID,
    pub result: PlaneResult,
}

/// Body time of impact result for movers.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BodyTOIResult {
    pub point: Pos,
    pub normal: Vec3,
    pub fraction: f32,
    pub shape_id: ShapeID,
}

/// A cast against a world shape, preserving the double precision hit position.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct WorldCastOutput {
    pub normal: Vec3,
    pub point: Pos,
    pub fraction: f32,
    pub iterations: i32,
    pub triangle_index: i32,
    pub child_index: i32,
    pub material_index: i32,
    pub hit: bool,
}

impl WorldCastOutput {
    pub fn from_local(local: &CastOutput, origin: Pos) -> Self {
        Self {
            normal: local.normal,
            point: crate::math::functions::offset_pos(origin, local.point),
            fraction: local.fraction,
            iterations: local.iterations,
            triangle_index: local.triangle_index,
            child_index: local.child_index,
            material_index: local.material_index,
            hit: local.hit,
        }
    }
}
