//! Runtime data model and core body management.
//!
//! Translated from `Sources/Natsel/Simulation/WorldRuntime.swift`.
//!
//! `World` is a Swift class; here it is a plain owned `struct`. The Swift reference-type
//! members that `World` exclusively owns (`BroadPhase`, `ConstraintGraph`, `ContactPayloadPool`,
//! `HullDatabase`, `ContinuousContext`, the `StableStorage` stores, ...) become owned fields.
//! All later `impl World` blocks in sibling modules reach these fields directly; there is no
//! aliasing graph. The only genuinely shared object is `ConstraintPath` (a Swift class that
//! several path-joint slots can point at), which is kept behind an `Arc`.

use std::collections::HashMap;
use std::ffi::c_void;
use std::sync::Arc;

use crate::base::constants as b3;
use crate::base::core::{b3_alloc_typed, b3_assert, b3_free_typed, b3_validate};
use crate::base::worker_pool::WorkerPool;
use crate::collision::dynamic_tree::TREE_STACK_SIZE;
use crate::containers::id_pool::IDPool;
use crate::containers::name_cache::NameCache;
use crate::containers::stable_storage::StableStorage;
use crate::math::functions::{
    is_valid_float, is_valid_position, is_valid_quat, is_valid_vec3, max_float, max_int, sub_pos,
};
use crate::math::types::{AABB, Matrix3, Pos, Quat, Transform, Vec3, WorldTransform};
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::collision_types::SATCache;
use crate::types::events::{
    BodyMoveEvent, ContactBeginTouchEvent, ContactEndTouchEvent, ContactHitEvent, JointEvent,
    SensorBeginTouchEvent, SensorEndTouchEvent,
};
use crate::types::ids::BodyID;
use crate::types::joint_defs::JointType;
use crate::types::queries::{SimplexCache, Sweep};
use crate::types::shape_def::ShapeType;
use crate::types::world_def::{
    Capacity, CustomFilterCallback, FrictionCallback, PreSolveCallback, Profile,
    RestitutionCallback,
};

use crate::simulation::broad_phase::BroadPhase;
use crate::simulation::contact_creation::{NodePair, TraversalPair};
use crate::simulation::contact_payload::{ContactPayload, ContactPayloadPool, MeshContactScratch};
use crate::simulation::contact_solver::{ManifoldConstraint, SolverContactData};
use crate::simulation::constraint_graph::ConstraintGraph;
use crate::simulation::continuous::dynamic_impact::DynamicContinuousImpulse;
use crate::simulation::continuous::ContinuousContext;
use crate::simulation::coupling_joints::{
    GearJointSim, PulleyJointSim, RackAndPinionJointSim,
};
use crate::simulation::end_event_buffer::EndEventBuffer;
use crate::simulation::joint::DistanceJointSim;
use crate::simulation::motor_joint::MotorJointSim;
use crate::simulation::overflow_contact_solver::OverflowContactConstraint;
use crate::simulation::parallel_joint::ParallelJointSim;
use crate::simulation::path_joint::{ConstraintPath, PathJointSim};
use crate::simulation::prismatic_joint::PrismaticJointSim;
use crate::simulation::revolute_joint::RevoluteJointSim;
use crate::simulation::sensor::Sensor;
use crate::simulation::shape_creation::HullDatabase;
use crate::simulation::shape_runtime::{Shape, destroy_shape_allocations, destroy_shape_proxy};
use crate::simulation::six_dof_joint::{SixDOFJointData, SixDOFJointSim};
use crate::simulation::solver_integration::{
    FinalizeThreadData, FinalizeViews, Softness, StepContext, TaskContext,
};
use crate::simulation::solver_prepare::{
    ContactPrepareItem, ContactPrepareViews, WideJointPrepareItem,
};
use crate::simulation::solver_stages::SolverViews;
use crate::simulation::spherical_joint::SphericalJointSim;
use crate::simulation::weld_joint::WeldJointSim;
use crate::simulation::wheel_joint::WheelJointSim;
use crate::simulation::wide_joint_solver::SolverJointData;
use crate::soft_body::soft_body_ray_cache::SoftBodyRayCache;
use crate::soft_body::soft_body_runtime::SoftBodyRecord;
use crate::vehicle::vehicle_runtime::VehicleRecord;

// MARK: - Body flags

/// Body flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct BodyFlags(pub u32);

impl BodyFlags {
    pub const LOCK_LINEAR_X: BodyFlags = BodyFlags(0x0000_0001);
    pub const LOCK_LINEAR_Y: BodyFlags = BodyFlags(0x0000_0002);
    pub const LOCK_LINEAR_Z: BodyFlags = BodyFlags(0x0000_0004);
    pub const LOCK_ANGULAR_X: BodyFlags = BodyFlags(0x0000_0008);
    pub const LOCK_ANGULAR_Y: BodyFlags = BodyFlags(0x0000_0010);
    pub const LOCK_ANGULAR_Z: BodyFlags = BodyFlags(0x0000_0020);

    /// Only lives on BodySim.
    pub const IS_FAST: BodyFlags = BodyFlags(0x0000_0040);
    pub const IS_BULLET: BodyFlags = BodyFlags(0x0000_0080);
    pub const IS_SPEED_CAPPED: BodyFlags = BodyFlags(0x0000_0100);
    pub const HAD_TIME_OF_IMPACT: BodyFlags = BodyFlags(0x0000_0200);
    pub const ALLOW_FAST_ROTATION: BodyFlags = BodyFlags(0x0000_0400);
    pub const ENLARGE_BULLET_BOUNDS: BodyFlags = BodyFlags(0x0000_0800);
    pub const DYNAMIC_FLAG: BodyFlags = BodyFlags(0x0000_1000);
    pub const ENABLE_SLEEP: BodyFlags = BodyFlags(0x0000_2000);
    pub const ENABLE_CONTACT_RECYCLING: BodyFlags = BodyFlags(0x0000_4000);
    pub const DIRTY_MASS: BodyFlags = BodyFlags(0x0000_8000);
    pub const ENABLE_GYROSCOPIC_TORQUE: BodyFlags = BodyFlags(0x0001_0000);

    pub const ALL_LOCKS: BodyFlags = BodyFlags(
        Self::LOCK_LINEAR_X.0
            | Self::LOCK_LINEAR_Y.0
            | Self::LOCK_LINEAR_Z.0
            | Self::LOCK_ANGULAR_X.0
            | Self::LOCK_ANGULAR_Y.0
            | Self::LOCK_ANGULAR_Z.0,
    );

    pub const FIXED_ROTATION: BodyFlags = BodyFlags(
        Self::LOCK_ANGULAR_X.0 | Self::LOCK_ANGULAR_Y.0 | Self::LOCK_ANGULAR_Z.0,
    );

    pub const BODY_TRANSIENT_FLAGS: BodyFlags =
        BodyFlags(Self::IS_SPEED_CAPPED.0 | Self::HAD_TIME_OF_IMPACT.0);

    /// Mirrors `OptionSet.contains`.
    #[inline(always)]
    pub fn contains(self, other: BodyFlags) -> bool {
        self.0 & other.0 == other.0
    }

    /// Mirrors `OptionSet.insert`.
    #[inline(always)]
    pub fn insert(&mut self, other: BodyFlags) {
        self.0 |= other.0;
    }

    /// Mirrors `OptionSet.remove`.
    #[inline(always)]
    pub fn remove(&mut self, other: BodyFlags) {
        self.0 &= !other.0;
    }

    /// Mirrors `OptionSet.toggle`.
    #[inline(always)]
    pub fn toggle(&mut self, other: BodyFlags) {
        self.0 ^= other.0;
    }
}

impl std::ops::BitOr for BodyFlags {
    type Output = BodyFlags;
    #[inline(always)]
    fn bitor(self, rhs: BodyFlags) -> BodyFlags {
        BodyFlags(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for BodyFlags {
    #[inline(always)]
    fn bitor_assign(&mut self, rhs: BodyFlags) {
        self.0 |= rhs.0;
    }
}

impl std::ops::BitAnd for BodyFlags {
    type Output = BodyFlags;
    #[inline(always)]
    fn bitand(self, rhs: BodyFlags) -> BodyFlags {
        BodyFlags(self.0 & rhs.0)
    }
}

impl std::ops::BitXor for BodyFlags {
    type Output = BodyFlags;
    #[inline(always)]
    fn bitxor(self, rhs: BodyFlags) -> BodyFlags {
        BodyFlags(self.0 ^ rhs.0)
    }
}

impl std::ops::Not for BodyFlags {
    type Output = BodyFlags;
    #[inline(always)]
    fn not(self) -> BodyFlags {
        BodyFlags(!self.0)
    }
}

// MARK: - Solver set kinds

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub(crate) enum SetType {
    StaticSet = 0,
    DisabledSet = 1,
    AwakeSet = 2,
    FirstSleepingSet = 3,
}

impl SetType {
    /// Swift's `SetType.x.rawValue`.
    #[inline(always)]
    pub(crate) fn raw_value(self) -> i32 {
        self as i32
    }
}

// MARK: - Body state

/// Body state used in the constraint solver. 64 bytes.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct BodyState {
    pub linear_velocity: Vec3,
    pub padding1: f32,

    pub angular_velocity: Vec3,
    pub padding2: f32,

    /// Using delta position reduces round-off error far from the origin.
    pub delta_position: Vec3,

    /// BodyFlags.
    pub flags: u32,

    /// The delta rotation. Static bodies use identity.
    pub delta_rotation: Quat,
}

impl BodyState {
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            linear_velocity: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
            padding1: 0.0,
            angular_velocity: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
            padding2: 0.0,
            delta_position: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
            flags: 0,
            delta_rotation: Quat::IDENTITY,
        }
    }
}

impl Default for BodyState {
    /// Swift's `BodyState()` runs `init()` which overrides `deltaRotation` with identity, so
    /// the default value is *not* the all-zeroes derive.
    #[inline(always)]
    fn default() -> Self {
        BodyState::new()
    }
}

impl BodyState {
    pub const IDENTITY: BodyState = BodyState::new();
}

// MARK: - Body simulation

/// Body simulation data used for integration of position and velocity.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub(crate) struct BodySim {
    /// Transform for body origin.
    pub transform: WorldTransform,

    /// Center of mass position in world space.
    pub center: Pos,

    /// BodyFlags.
    pub flags: u32,

    pub min_extent: f32,
    pub max_extent: Vec3,

    /// Previous rotation and COM for TOI.
    pub rotation0: Quat,
    pub center0: Pos,

    /// Location of center of mass relative to the body origin.
    pub local_center: Vec3,

    pub force: Vec3,
    pub torque: Vec3,

    pub inv_mass: f32,

    /// Local space inertia.
    pub inertia: Matrix3,

    /// Rotational inertia about the center of mass.
    pub inv_inertia_local: Matrix3,
    pub inv_inertia_world: Matrix3,

    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,

    /// Index of Body.
    pub body_id: i32,
}

// MARK: - Body

/// Body organizational details that are not used in the solver.
#[derive(Clone, Debug)]
pub(crate) struct Body {
    pub user_data: Option<*mut c_void>,

    /// Index of solver set stored in World. May be `B3.nullIndex`.
    pub set_index: i32,

    /// Body sim and state index within set. May be `B3.nullIndex`.
    pub local_index: i32,

    /// [31 : contactId | 1 : edgeIndex]
    pub head_contact_key: i32,
    pub contact_count: i32,

    pub head_shape_id: i32,
    pub shape_count: i32,

    pub head_chain_id: i32,

    /// [31 : jointId | 1 : edgeIndex]
    pub head_joint_key: i32,
    pub joint_count: i32,

    /// All enabled dynamic and kinematic bodies are in an island.
    pub island_id: i32,

    /// Index into the island's bodies array for O(1) swap-removal.
    pub island_index: i32,

    pub sleep_threshold: f32,
    pub sleep_time: f32,
    pub sleep_velocity: f32,
    pub safety_factor: f32,
    pub mass: f32,

    pub body_move_index: i32,

    pub id: i32,

    /// BodyFlags.
    pub flags: u32,
    pub name_id: u32,

    pub type_: BodyType,

    /// Monotonically advanced when a body is allocated in this slot.
    pub generation: u16,
}

impl Default for Body {
    #[inline]
    fn default() -> Self {
        Self {
            user_data: None,
            set_index: b3::NULL_INDEX,
            local_index: b3::NULL_INDEX,
            head_contact_key: b3::NULL_INDEX,
            contact_count: 0,
            head_shape_id: b3::NULL_INDEX,
            shape_count: 0,
            head_chain_id: b3::NULL_INDEX,
            head_joint_key: b3::NULL_INDEX,
            joint_count: 0,
            island_id: b3::NULL_INDEX,
            island_index: b3::NULL_INDEX,
            sleep_threshold: 0.0,
            sleep_time: 0.0,
            sleep_velocity: 0.0,
            safety_factor: 0.0,
            mass: 0.0,
            body_move_index: b3::NULL_INDEX,
            id: b3::NULL_INDEX,
            flags: 0,
            name_id: 0,
            type_: BodyType::StaticBody,
            generation: 0,
        }
    }
}

// MARK: - Islands

/// Cached contact data stored in the island for fast contiguous iteration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub(crate) struct ContactLink {
    pub contact_id: i32,
    pub body_id_a: i32,
    pub body_id_b: i32,
}

/// Cached joint data stored in the island for fast contiguous iteration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub(crate) struct JointLink {
    pub joint_id: i32,
    pub body_id_a: i32,
    pub body_id_b: i32,
}

/// Persistent island for awake bodies, joints, and touching contacts.
#[derive(Clone, Debug)]
pub(crate) struct Island {
    /// Index of solver set stored in World. May be `B3.nullIndex`.
    pub set_index: i32,

    /// Island index within set. May be `B3.nullIndex`.
    pub local_index: i32,

    pub island_id: i32,

    /// Keeps track of how many contacts have been removed from this island.
    pub constraint_remove_count: i32,

    pub bodies: Vec<i32>,
    pub contacts: Vec<ContactLink>,
    pub joints: Vec<JointLink>,
}

impl Default for Island {
    #[inline]
    fn default() -> Self {
        Self {
            set_index: b3::NULL_INDEX,
            local_index: b3::NULL_INDEX,
            island_id: b3::NULL_INDEX,
            constraint_remove_count: 0,
            bodies: Vec::new(),
            contacts: Vec::new(),
            joints: Vec::new(),
        }
    }
}

/// Used to move islands across solver sets.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub(crate) struct IslandSim {
    pub island_id: i32,
}

// MARK: - Solver set

/// Solvers sets hold contiguous arrays of body sims, states, joints, contacts, and islands.
#[derive(Clone, Debug)]
pub(crate) struct SolverSet {
    pub body_sims: Vec<BodySim>,
    pub body_states: Vec<BodyState>,
    pub joint_sims: Vec<JointSim>,
    pub contact_indices: Vec<i32>,
    pub island_sims: Vec<IslandSim>,
    pub set_index: i32,
}

impl Default for SolverSet {
    #[inline]
    fn default() -> Self {
        Self {
            body_sims: Vec::new(),
            body_states: Vec::new(),
            joint_sims: Vec::new(),
            contact_indices: Vec::new(),
            island_sims: Vec::new(),
            set_index: b3::NULL_INDEX,
        }
    }
}

// MARK: - Contacts

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub(crate) struct ContactEdge {
    pub body_id: i32,
    pub prev_key: i32,
    pub next_key: i32,
}

/// The union of SAT and simplex caches.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum ContactCache {
    Sat(SATCache),
    Simplex(SimplexCache),
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct TriangleCache {
    pub triangle_index: i32,
    pub cache: ContactCache,
}

impl Default for TriangleCache {
    #[inline]
    fn default() -> Self {
        Self {
            triangle_index: 0,
            cache: ContactCache::Sat(SATCache::default()),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub(crate) enum ContactCacheKind {
    Convex = 0,
    Mesh = 1,
}

/// The persistent interaction between two shapes.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub(crate) struct Contact {
    pub set_index: i32,
    pub color_index: i32,
    pub local_index: i32,

    pub edges: [ContactEdge; 2],
    pub shape_id_a: i32,
    pub shape_id_b: i32,
    pub child_index: i32,

    pub island_id: i32,
    pub island_index: i32,
    pub contact_id: i32,

    pub encoded_body_sim_a: i32,
    pub encoded_body_sim_b: i32,

    pub flags: u32,

    pub payload: Option<*mut ContactPayload>,
    pub manifold_count: i32,

    pub cached_rotation_a: Quat,
    pub cached_rotation_b: Quat,
    pub cached_relative_pose: Transform,

    pub friction: f32,

    /// Convex contact cache. Two fields represent the cache because a contact's shape
    /// pair fixes which one is written.
    pub sat_cache: SATCache,
    pub simplex_cache: SimplexCache,

    pub restitution: f32,
    pub rolling_resistance: f32,
    pub tangent_velocity: Vec3,

    pub generation: u32,
}

impl Default for Contact {
    #[inline]
    fn default() -> Self {
        Self {
            set_index: b3::NULL_INDEX,
            color_index: b3::NULL_INDEX,
            local_index: b3::NULL_INDEX,
            edges: [ContactEdge::default(); 2],
            shape_id_a: b3::NULL_INDEX,
            shape_id_b: b3::NULL_INDEX,
            child_index: 0,
            island_id: b3::NULL_INDEX,
            island_index: b3::NULL_INDEX,
            contact_id: b3::NULL_INDEX,
            encoded_body_sim_a: b3::NULL_INDEX,
            encoded_body_sim_b: b3::NULL_INDEX,
            flags: 0,
            payload: None,
            manifold_count: 0,
            cached_rotation_a: Quat::default(),
            cached_rotation_b: Quat::default(),
            cached_relative_pose: Transform::default(),
            friction: 0.0,
            sat_cache: SATCache::default(),
            simplex_cache: SimplexCache::default(),
            restitution: 0.0,
            rolling_resistance: 0.0,
            tangent_velocity: Vec3::default(),
            generation: 0,
        }
    }
}

// MARK: - Joints

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub(crate) struct JointEdge {
    pub body_id: i32,
    pub prev_key: i32,
    pub next_key: i32,
}

/// Map from JointID to Joint in the solver sets.
#[derive(Clone, Debug)]
pub(crate) struct Joint {
    pub user_data: Option<*mut c_void>,

    pub set_index: i32,
    pub color_index: i32,
    pub local_index: i32,

    pub edges: [JointEdge; 2],

    pub joint_id: i32,
    pub island_id: i32,
    pub island_index: i32,

    pub draw_scale: f32,

    pub type_: JointType,

    pub generation: u16,
    pub collide_connected: bool,
}

impl Default for Joint {
    #[inline]
    fn default() -> Self {
        Self {
            user_data: None,
            set_index: b3::NULL_INDEX,
            color_index: b3::NULL_INDEX,
            local_index: b3::NULL_INDEX,
            edges: [JointEdge::default(); 2],
            joint_id: b3::NULL_INDEX,
            island_id: b3::NULL_INDEX,
            island_index: b3::NULL_INDEX,
            draw_scale: 1.0,
            type_: JointType::DistanceJoint,
            generation: 0,
            collide_connected: false,
        }
    }
}

/// The discriminant and only the active joint's POD data share one allocation-free value.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
enum JointPayload {
    Distance(DistanceJointSim),
    Revolute(RevoluteJointSim),
    Weld(WeldJointSim),
    Parallel(ParallelJointSim),
    Motor(MotorJointSim),
    Prismatic(PrismaticJointSim),
    Spherical(SphericalJointSim),
    Wheel(WheelJointSim),
    Gear(GearJointSim),
    RackAndPinion(RackAndPinionJointSim),
    Pulley(PulleyJointSim),
    SixDOF(SixDOFJointSim),
    Path(PathJointSim),
}

impl Default for JointPayload {
    #[inline]
    fn default() -> Self {
        JointPayload::Distance(DistanceJointSim::default())
    }
}

/// Solver data for a joint.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub(crate) struct JointSim {
    pub joint_id: i32,
    pub body_id_a: i32,
    pub body_id_b: i32,
    pub type_: JointType,

    // Joint frames local to body origin
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,

    pub inv_mass_a: f32,
    pub inv_mass_b: f32,
    pub inv_ia: Matrix3,
    pub inv_ib: Matrix3,

    pub constraint_hertz: f32,
    pub constraint_damping_ratio: f32,
    pub constraint_softness: Softness,

    pub force_threshold: f32,
    pub torque_threshold: f32,

    pub fixed_rotation: bool,

    // A joint owns exactly one variant. Keeping all eight payloads increased each solver
    // record to over 1.7 KB and copied unused state through every API and solver stage.
    payload: JointPayload,
}

impl Default for JointSim {
    #[inline]
    fn default() -> Self {
        Self {
            joint_id: b3::NULL_INDEX,
            body_id_a: b3::NULL_INDEX,
            body_id_b: b3::NULL_INDEX,
            type_: JointType::DistanceJoint,
            local_frame_a: Transform::default(),
            local_frame_b: Transform::default(),
            inv_mass_a: 0.0,
            inv_mass_b: 0.0,
            inv_ia: Matrix3::default(),
            inv_ib: Matrix3::default(),
            constraint_hertz: 0.0,
            constraint_damping_ratio: 0.0,
            constraint_softness: Softness::default(),
            force_threshold: 0.0,
            torque_threshold: 0.0,
            fixed_rotation: false,
            payload: JointPayload::Distance(DistanceJointSim::default()),
        }
    }
}

impl JointSim {
    #[inline(always)]
    pub(crate) fn distance(&self) -> DistanceJointSim {
        match self.payload {
            JointPayload::Distance(value) => value,
            _ => DistanceJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_distance(&mut self, value: DistanceJointSim) {
        self.payload = JointPayload::Distance(value);
    }

    #[inline(always)]
    pub(crate) fn revolute(&self) -> RevoluteJointSim {
        match self.payload {
            JointPayload::Revolute(value) => value,
            _ => RevoluteJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_revolute(&mut self, value: RevoluteJointSim) {
        self.payload = JointPayload::Revolute(value);
    }

    #[inline(always)]
    pub(crate) fn weld(&self) -> WeldJointSim {
        match self.payload {
            JointPayload::Weld(value) => value,
            _ => WeldJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_weld(&mut self, value: WeldJointSim) {
        self.payload = JointPayload::Weld(value);
    }

    #[inline(always)]
    pub(crate) fn parallel(&self) -> ParallelJointSim {
        match self.payload {
            JointPayload::Parallel(value) => value,
            _ => ParallelJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_parallel(&mut self, value: ParallelJointSim) {
        self.payload = JointPayload::Parallel(value);
    }

    #[inline(always)]
    pub(crate) fn motor(&self) -> MotorJointSim {
        match self.payload {
            JointPayload::Motor(value) => value,
            _ => MotorJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_motor(&mut self, value: MotorJointSim) {
        self.payload = JointPayload::Motor(value);
    }

    #[inline(always)]
    pub(crate) fn prismatic(&self) -> PrismaticJointSim {
        match self.payload {
            JointPayload::Prismatic(value) => value,
            _ => PrismaticJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_prismatic(&mut self, value: PrismaticJointSim) {
        self.payload = JointPayload::Prismatic(value);
    }

    #[inline(always)]
    pub(crate) fn spherical(&self) -> SphericalJointSim {
        match self.payload {
            JointPayload::Spherical(value) => value,
            _ => SphericalJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_spherical(&mut self, value: SphericalJointSim) {
        self.payload = JointPayload::Spherical(value);
    }

    #[inline(always)]
    pub(crate) fn wheel(&self) -> WheelJointSim {
        match self.payload {
            JointPayload::Wheel(value) => value,
            _ => WheelJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_wheel(&mut self, value: WheelJointSim) {
        self.payload = JointPayload::Wheel(value);
    }

    #[inline(always)]
    pub(crate) fn gear(&self) -> GearJointSim {
        match self.payload {
            JointPayload::Gear(value) => value,
            _ => GearJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_gear(&mut self, value: GearJointSim) {
        self.payload = JointPayload::Gear(value);
    }

    #[inline(always)]
    pub(crate) fn rack_and_pinion(&self) -> RackAndPinionJointSim {
        match self.payload {
            JointPayload::RackAndPinion(value) => value,
            _ => RackAndPinionJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_rack_and_pinion(&mut self, value: RackAndPinionJointSim) {
        self.payload = JointPayload::RackAndPinion(value);
    }

    #[inline(always)]
    pub(crate) fn pulley(&self) -> PulleyJointSim {
        match self.payload {
            JointPayload::Pulley(value) => value,
            _ => PulleyJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_pulley(&mut self, value: PulleyJointSim) {
        self.payload = JointPayload::Pulley(value);
    }

    #[inline(always)]
    pub(crate) fn path(&self) -> PathJointSim {
        match self.payload {
            JointPayload::Path(value) => value,
            _ => PathJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_path(&mut self, value: PathJointSim) {
        self.payload = JointPayload::Path(value);
    }

    #[inline(always)]
    pub(crate) fn six_dof(&self) -> SixDOFJointSim {
        match self.payload {
            JointPayload::SixDOF(value) => value,
            _ => SixDOFJointSim::default(),
        }
    }

    #[inline(always)]
    pub(crate) fn set_six_dof(&mut self, value: SixDOFJointSim) {
        self.payload = JointPayload::SixDOF(value);
    }
}

// MARK: - Body helpers

#[inline(always)]
pub(crate) fn encode_body_sim_index(body: &Body) -> i32 {
    if body.set_index == SetType::AwakeSet.raw_value() {
        return body.local_index;
    }

    if body.set_index == SetType::StaticSet.raw_value() {
        return -(body.local_index + 2);
    }

    b3::NULL_INDEX
}

#[inline(always)]
pub(crate) fn is_static_sim_index(encoded: i32) -> bool {
    encoded < b3::NULL_INDEX
}

#[inline(always)]
pub(crate) fn decode_awake_index(encoded: i32) -> i32 {
    if encoded >= 0 { encoded } else { b3::NULL_INDEX }
}

#[inline(always)]
pub(crate) fn sleep_body_sim_index(encoded: i32) -> i32 {
    if encoded >= 0 { b3::NULL_INDEX } else { encoded }
}

/// Make a sweep relative to a base position to keep TOI in float precision far from the origin.
#[inline(always)]
pub(crate) fn make_relative_sweep(body_sim: &BodySim, base: Pos) -> Sweep {
    let mut s = Sweep::default();
    s.c1 = sub_pos(body_sim.center0, base);
    s.c2 = sub_pos(body_sim.center, base);
    s.q1 = body_sim.rotation0;
    s.q2 = body_sim.transform.q;
    s.local_center = body_sim.local_center;
    s
}

// MARK: - World

/// The world owns all bodies, shapes, contacts, joints, and solver sets.
pub struct World {
    pub(crate) world_id: u16,
    pub(crate) generation: u16,

    /// Swift's class `BroadPhase`; only `World` holds the reference, so it is an owned field.
    pub(crate) broad_phase: BroadPhase,
    /// Swift's `let` class `ConstraintGraph`; owned here, `World` is its only reference.
    pub(crate) constraint_graph: ConstraintGraph,

    /// Reused broad-phase scratch. These arrays stay alive across steps so pair generation does
    /// not allocate and regrow a fresh list every frame.
    pub(crate) pair_key_scratch: Vec<u64>,
    pub(crate) moved_sibling_scratch: Vec<i32>,
    pub(crate) cross_seed_queue: Vec<NodePair>,
    pub(crate) static_seed_scratch: Vec<NodePair>,
    pub(crate) kinematic_seed_scratch: Vec<NodePair>,
    // SAFETY: fixed traversal stacks mirroring Swift's
    // `UnsafeMutablePointer.allocate(capacity: treeStackSize)`. The pair-generation code in
    // `contact_creation.rs` pushes/pops through unchecked pointer offsets; keeping raw storage
    // preserves that instruction sequence (no uniqueness check, no bounds check). Capacity is
    // exactly `TREE_STACK_SIZE` for both stacks and each is freed exactly once in `Drop`.
    pub(crate) broad_phase_pair_stack: *mut TraversalPair,
    pub(crate) broad_phase_proxy_stack: *mut i32,
    pub(crate) contact_index_scratch: Vec<i32>,
    /// Broad-phase candidates awaiting the batched pair gauntlet.
    /// Swift `InlineArray<32, SIMD2<Int32>>` -> 32 lane pairs of (a, b) body-proxy indices.
    pub(crate) candidate_batch: [[i32; 2]; 32],
    pub(crate) candidate_batch_count: usize,
    pub(crate) candidate_batch_size: usize,

    pub(crate) bodies: Vec<Body>,
    pub(crate) body_id_pool: IDPool,

    pub(crate) solver_sets: Vec<SolverSet>,
    pub(crate) solver_set_id_pool: IDPool,

    pub(crate) islands: Vec<Island>,
    pub(crate) island_id_pool: IDPool,

    pub(crate) shapes: Vec<Shape>,
    pub(crate) shape_id_pool: IDPool,

    pub(crate) fat_aabbs: Vec<AABB>,

    pub(crate) contacts: Vec<Contact>,
    pub(crate) contact_payloads: ContactPayloadPool,
    pub(crate) contact_id_pool: IDPool,

    pub(crate) joints: Vec<Joint>,
    pub(crate) joint_id_pool: IDPool,

    pub(crate) names: NameCache,

    /// Move events captured during a step.
    pub(crate) body_move_events: Vec<BodyMoveEvent>,

    /// Profiling data for the most recent step.
    pub profile: Profile,

    /// Sensors
    pub(crate) sensors: Vec<Sensor>,
    pub(crate) sensor_begin_events: Vec<SensorBeginTouchEvent>,
    pub(crate) sensor_end_events: EndEventBuffer<SensorEndTouchEvent>,

    /// Contact touch events
    pub(crate) contact_begin_events: Vec<ContactBeginTouchEvent>,
    pub(crate) contact_end_events: EndEventBuffer<ContactEndTouchEvent>,
    pub(crate) contact_hit_events: Vec<ContactHitEvent>,
    pub(crate) continuous_impact_events: Vec<ContactHitEvent>,
    pub(crate) joint_events: Vec<JointEvent>,

    /// Per-thread task storage.
    pub(crate) task_contexts: Vec<TaskContext>,

    /// Reused serial mesh narrow-phase scratch.
    pub(crate) mesh_contact_scratch: MeshContactScratch,

    /// Serial CCD query state. Reusing it avoids allocating a class and a sensor-hit array for
    /// every fast body every step.
    pub(crate) continuous_context: ContinuousContext,

    /// Number of worker threads for the parallel phases (broad-phase pair generation and the
    /// collide pass). The solver itself runs serially by color. Results are independent
    /// of this value: each worker owns a disjoint slice and a private contact-state bit set, and the
    /// slices are merged in contact id order.
    pub worker_count: i32,

    /// Solve independent contact batches with four SIMD lanes. Disable to use the equivalent
    /// scalar path. Overflow constraints always remain sequential.
    pub enable_wide_solver: bool,

    /// Also prepare scalar constraints for wide colors so tests can compare both paths on the
    /// same prepared state.
    pub(crate) prepare_scalar_for_wide_colors: bool,

    /// Per-step overflow contact constraints, solved with the scalar solver.
    pub(crate) overflow_contacts: Vec<OverflowContactConstraint>,
    pub(crate) overflow_manifolds: Vec<ManifoldConstraint>,

    /// Per-step solver constraint data.
    pub(crate) solver_contacts: SolverContactData,
    pub(crate) solver_joints: SolverJointData,

    /// True when any prepared contact has restitution.
    pub(crate) solve_any_restitution: bool,

    /// Reference counted store of shared hull data keyed by content.
    pub(crate) hull_database: HullDatabase,

    pub gravity: Vec3,
    pub hit_event_threshold: f32,
    pub restitution_threshold: f32,
    pub restitution_iterations: i32,
    pub enable_restitution_propagation: bool,
    pub max_linear_speed: f32,
    pub isotropic_inertia_tolerance: f32,
    pub contact_speed: f32,
    pub contact_hertz: f32,
    pub contact_damping_ratio: f32,
    pub contact_recycle_distance: f32,
    pub(crate) simd_width: i32,
    pub(crate) sat_call_count: i32,
    pub(crate) sat_cache_hit_count: i32,

    // None selects the inline default mixer. A custom callback still runs for every update.
    pub(crate) friction_callback: Option<FrictionCallback>,
    pub(crate) restitution_callback: Option<RestitutionCallback>,

    pub(crate) custom_filter: Option<CustomFilterCallback>,

    /// Optional pre-solve callback. Returning false disables the contact.
    pub(crate) pre_solve: Option<PreSolveCallback>,

    pub enable_sleep: bool,
    pub enable_warm_starting: bool,
    pub enable_continuous: bool,
    /// Also sweep convex dynamic bodies against dynamic and kinematic bodies, including bullets
    /// against bullets. Opt in when moving pairs need CCD.
    pub enable_dynamic_continuous: bool,
    pub(crate) dynamic_continuous_impulses: Vec<DynamicContinuousImpulse>,
    pub(crate) dynamic_continuous_active_bodies: Vec<i32>,
    pub enable_speculative: bool,
    pub(crate) step_index: u64,
    pub(crate) inv_h: f32,
    pub(crate) inv_dt: f32,
    pub(crate) split_island_id: i32,
    pub(crate) compound_shape_count: i32,

    /// Vehicle constraints. Solved with the joints of each substep (Vehicle/).
    pub(crate) vehicles: Vec<VehicleRecord>,
    pub(crate) vehicle_id_pool: IDPool,

    /// Persistent solver threads, created when `workerCount` exceeds one.
    pub(crate) worker_pool: Option<WorkerPool>,
    /// Solver array pointers and the step context while the staged solver runs.
    pub(crate) solver_views: SolverViews,
    pub(crate) stage_context: StepContext,
    /// The pool while a solver session runs (preparation through impulse storage).
    pub(crate) active_pool: Option<WorkerPool>,
    /// Contact preparation plan and pointers, rebuilt every step.
    pub(crate) prepare_items: Vec<ContactPrepareItem>,
    pub(crate) prepare_ids: Vec<i32>,
    pub(crate) wide_joint_items: Vec<WideJointPrepareItem>,
    pub(crate) prepare_views: ContactPrepareViews,
    /// Body finalization scratch: deferred work per awake body, enlarged proxies per shape, and
    /// island results per thread.
    pub(crate) finalize_flags: Vec<u8>,
    pub(crate) shape_move_flags: Vec<u8>,
    pub(crate) finalize_threads: Vec<FinalizeThreadData>,
    pub(crate) finalize_views: FinalizeViews,

    /// Six DOF joint state, kept out of `JointSim` so other joints stay small.
    pub(crate) six_dof_store: StableStorage<SixDOFJointData>,
    pub(crate) six_dof_pool: IDPool,

    /// Path joint state and the paths it points into, by slot.
    pub(crate) path_joint_store: StableStorage<PathJointData>,
    // `ConstraintPath` is a Swift class that several path-joint slots may reference at once,
    // so slots share one allocation through `Arc` instead of copying the path per slot.
    pub(crate) path_joint_paths: Vec<Option<Arc<ConstraintPath>>>,
    pub(crate) path_joint_pool: IDPool,

    /// Position based soft bodies. Simulated after rigid body finalization (SoftBody/).
    pub(crate) soft_bodies: Vec<SoftBodyRecord>,
    pub(crate) soft_body_id_pool: IDPool,
    /// Rebuilt lazily after vertex edits or simulation; derived data is never included in snapshots.
    pub(crate) soft_body_ray_caches: HashMap<i32, SoftBodyRayCache>,
    /// Lazily build face BVHs for repeated soft-body rays. Linear scans can be cheaper when a
    /// deforming surface receives very few rays per step; results are identical in either mode.
    pub enable_soft_body_query_acceleration: bool,

    pub(crate) locked: bool,
    pub(crate) in_use: bool,
}

impl Drop for World {
    fn drop(&mut self) {
        // SAFETY: both pointers were allocated in `World::new` with capacity `TREE_STACK_SIZE`
        // through `b3_alloc_typed`, so freeing them with `b3_free_typed` at that same count is
        // valid. Each stack is freed exactly once here.
        unsafe {
            b3_free_typed(self.broad_phase_pair_stack, TREE_STACK_SIZE as i32);
            b3_free_typed(self.broad_phase_proxy_stack, TREE_STACK_SIZE as i32);
        }
    }
}

impl World {
    /// Swift's `init(capacity:)`. Swift defaulted the argument; callers ported from `World()`
    /// should pass `Capacity::default()`.
    pub fn new(capacity: Capacity) -> Self {
        let broad_phase = BroadPhase::new(capacity);
        let constraint_graph = ConstraintGraph::new(max_int(
            16,
            capacity.static_body_count + capacity.dynamic_body_count,
        ));

        let mut solver_set_id_pool = IDPool::new();
        let solver_sets: Vec<SolverSet> = {
            // Add empty static, disabled, and awake body sets.
            let mut sets = Vec::new();
            sets.push(SolverSet::default());
            sets[0].set_index = solver_set_id_pool.alloc();

            let mut disabled_set = SolverSet::default();
            disabled_set.set_index = solver_set_id_pool.alloc();
            sets.push(disabled_set);

            let mut awake_set = SolverSet::default();
            awake_set.set_index = solver_set_id_pool.alloc();
            sets.push(awake_set);
            sets
        };

        let mut bodies: Vec<Body> = Vec::new();
        bodies.reserve(
            max_int(16, capacity.static_body_count + capacity.dynamic_body_count) as usize,
        );

        // Swift: UnsafeMutablePointer<TraversalPair>.allocate(capacity: treeStackSize).
        // SAFETY: uninitialized fixed-capacity stacks; `contact_creation.rs` initializes each
        // slot before reading it and never exceeds TREE_STACK_SIZE entries (guarded by its
        // stack-depth asserts). The pointers are freed once in `Drop`.
        let broad_phase_pair_stack =
            b3_alloc_typed::<TraversalPair>(TREE_STACK_SIZE as i32);
        let broad_phase_proxy_stack = b3_alloc_typed::<i32>(TREE_STACK_SIZE as i32);

        Self {
            world_id: 0,
            generation: 0,
            broad_phase,
            constraint_graph,
            pair_key_scratch: Vec::new(),
            moved_sibling_scratch: Vec::new(),
            cross_seed_queue: Vec::new(),
            static_seed_scratch: Vec::new(),
            kinematic_seed_scratch: Vec::new(),
            broad_phase_pair_stack,
            broad_phase_proxy_stack,
            contact_index_scratch: Vec::new(),
            candidate_batch: [[0; 2]; 32],
            candidate_batch_count: 0,
            candidate_batch_size: 32,
            bodies,
            body_id_pool: IDPool::new(),
            solver_sets,
            solver_set_id_pool,
            islands: Vec::new(),
            island_id_pool: IDPool::new(),
            shapes: Vec::new(),
            shape_id_pool: IDPool::new(),
            fat_aabbs: Vec::new(),
            contacts: Vec::new(),
            contact_payloads: ContactPayloadPool::default(),
            contact_id_pool: IDPool::new(),
            joints: Vec::new(),
            joint_id_pool: IDPool::new(),
            names: NameCache::new(),
            body_move_events: Vec::new(),
            profile: Profile::default(),
            sensors: Vec::new(),
            sensor_begin_events: Vec::new(),
            sensor_end_events: EndEventBuffer::default(),
            contact_begin_events: Vec::new(),
            contact_end_events: EndEventBuffer::default(),
            contact_hit_events: Vec::new(),
            continuous_impact_events: Vec::new(),
            joint_events: Vec::new(),
            task_contexts: vec![TaskContext::default()],
            mesh_contact_scratch: MeshContactScratch::default(),
            continuous_context: ContinuousContext::default(),
            worker_count: 1,
            enable_wide_solver: true,
            prepare_scalar_for_wide_colors: false,
            overflow_contacts: Vec::new(),
            overflow_manifolds: Vec::new(),
            solver_contacts: SolverContactData::default(),
            solver_joints: SolverJointData::default(),
            solve_any_restitution: false,
            hull_database: HullDatabase::default(),
            gravity: Vec3::new(0.0, -10.0, 0.0),
            hit_event_threshold: 1.0,
            restitution_threshold: 1.0,
            restitution_iterations: 2,
            enable_restitution_propagation: false,
            max_linear_speed: 400.0,
            isotropic_inertia_tolerance: 0.2,
            contact_speed: 3.0,
            contact_hertz: 30.0,
            contact_damping_ratio: 10.0,
            contact_recycle_distance: b3::contact_recycle_distance(),
            simd_width: 4,
            sat_call_count: 0,
            sat_cache_hit_count: 0,
            friction_callback: None,
            restitution_callback: None,
            custom_filter: None,
            pre_solve: None,
            enable_sleep: true,
            enable_warm_starting: true,
            enable_continuous: true,
            enable_dynamic_continuous: false,
            dynamic_continuous_impulses: Vec::new(),
            dynamic_continuous_active_bodies: Vec::new(),
            enable_speculative: true,
            step_index: 0,
            inv_h: 0.0,
            inv_dt: 0.0,
            split_island_id: b3::NULL_INDEX,
            compound_shape_count: 0,
            vehicles: Vec::new(),
            vehicle_id_pool: IDPool::new(),
            worker_pool: None,
            solver_views: SolverViews::default(),
            stage_context: StepContext::default(),
            active_pool: None,
            prepare_items: Vec::new(),
            prepare_ids: Vec::new(),
            wide_joint_items: Vec::new(),
            prepare_views: ContactPrepareViews::default(),
            finalize_flags: Vec::new(),
            shape_move_flags: Vec::new(),
            finalize_threads: Vec::new(),
            finalize_views: FinalizeViews::default(),
            six_dof_store: StableStorage::default(),
            six_dof_pool: IDPool::new(),
            path_joint_store: StableStorage::default(),
            path_joint_paths: Vec::new(),
            path_joint_pool: IDPool::new(),
            soft_bodies: Vec::new(),
            soft_body_id_pool: IDPool::new(),
            soft_body_ray_caches: HashMap::new(),
            enable_soft_body_query_acceleration: true,
            locked: false,
            in_use: true,
        }
    }

    // MARK: Material mixing and callbacks

    #[inline(always)]
    pub(crate) fn mix_friction(&self, a: f32, id_a: u64, b: f32, id_b: u64) -> f32 {
        if let Some(friction_callback) = &self.friction_callback {
            return friction_callback(a, id_a, b, id_b);
        }
        (a * b).sqrt()
    }

    #[inline(always)]
    pub(crate) fn mix_restitution(&self, a: f32, id_a: u64, b: f32, id_b: u64) -> f32 {
        if let Some(restitution_callback) = &self.restitution_callback {
            return restitution_callback(a, id_a, b, id_b);
        }
        max_float(a, b)
    }

    /// Set the custom shape filter. Both shapes must enable custom filtering.
    pub fn set_custom_filter(&mut self, callback: Option<CustomFilterCallback>) {
        self.custom_filter = callback;
    }

    /// Set the pre-solve callback. Returning false disables the contact for that step.
    pub fn set_pre_solve_callback(&mut self, callback: Option<PreSolveCallback>) {
        self.pre_solve = callback;
    }

    // MARK: Element access

    #[inline(always)]
    pub(crate) fn solver_set(&self, index: i32) -> &SolverSet {
        &self.solver_sets[index as usize]
    }

    #[inline(always)]
    pub(crate) fn body_sim(&self, body: &Body) -> BodySim {
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
    }

    #[inline(always)]
    pub(crate) fn body_state(&self, body: &Body) -> Option<BodyState> {
        if body.set_index == SetType::AwakeSet.raw_value() {
            return Some(
                self.solver_sets[SetType::AwakeSet.raw_value() as usize]
                    .body_states[body.local_index as usize],
            );
        }
        None
    }

    #[inline(always)]
    pub(crate) fn mutate_body_sim<F: FnOnce(&mut BodySim)>(&mut self, body: &Body, f: F) {
        let set_index = body.set_index as usize;
        let local_index = body.local_index as usize;
        // Swift copied the value-type sim out, mutated it, and wrote it back; mutating the slot
        // in place is the equivalent Rust operation.
        let sim = &mut self.solver_sets[set_index].body_sims[local_index];
        f(sim);
    }

    #[inline(always)]
    pub(crate) fn mutate_body_state<F: FnOnce(&mut BodyState)>(&mut self, body: &Body, f: F) {
        if body.set_index == SetType::AwakeSet.raw_value() {
            let local_index = body.local_index as usize;
            let state = &mut self.solver_sets[SetType::AwakeSet.raw_value() as usize]
                .body_states[local_index];
            f(state);
        }
    }

    // MARK: Ids

    pub(crate) fn make_body_id(&self, body_id: i32) -> BodyID {
        BodyID::new(
            body_id + 1,
            self.world_id,
            self.bodies[body_id as usize].generation,
        )
    }

    pub(crate) fn is_valid_body(&self, id: BodyID) -> bool {
        if id.world0 != self.world_id {
            return false;
        }
        if id.index1 < 1 || self.bodies.len() < id.index1 as usize {
            return false;
        }
        let body = &self.bodies[(id.index1 - 1) as usize];
        if body.set_index == b3::NULL_INDEX {
            // this was freed
            return false;
        }
        b3_assert(body.local_index != b3::NULL_INDEX);
        body.generation == id.generation
    }

    pub(crate) fn body(&self, id: BodyID) -> Body {
        b3_assert(self.is_valid_body(id));
        self.bodies[(id.index1 - 1) as usize].clone()
    }

    pub(crate) fn body_index(&self, id: BodyID) -> i32 {
        id.index1 - 1
    }

    // MARK: Islands

    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub(crate) fn create_island(&mut self, set_index: i32) -> i32 {
        b3_assert(
            set_index == SetType::AwakeSet.raw_value()
                || set_index >= SetType::FirstSleepingSet.raw_value(),
        );

        let island_id = self.island_id_pool.alloc();

        if island_id as usize == self.islands.len() {
            self.islands.push(Island::default());
        } else {
            b3_assert(self.islands[island_id as usize].set_index == b3::NULL_INDEX);
        }

        let mut island = Island::default();
        island.set_index = set_index;
        island.local_index =
            self.solver_sets[set_index as usize].island_sims.len() as i32;
        island.island_id = island_id;
        self.islands[island_id as usize] = island;

        let mut island_sim = IslandSim::default();
        island_sim.island_id = island_id;
        self.solver_sets[set_index as usize].island_sims.push(island_sim);

        island_id
    }

    pub(crate) fn destroy_island(&mut self, island_id: i32) {
        if self.split_island_id == island_id {
            self.split_island_id = b3::NULL_INDEX;
        }

        let mut island = self.islands[island_id as usize].clone();
        let set_index = island.set_index;
        let local_index = island.local_index;
        let last_index = self.solver_sets[set_index as usize].island_sims.len() as i32 - 1;
        b3_assert(0 <= local_index && local_index <= last_index);
        let move_island_id =
            self.solver_sets[set_index as usize].island_sims[last_index as usize].island_id;
        let moved_sim =
            self.solver_sets[set_index as usize].island_sims[last_index as usize];
        self.solver_sets[set_index as usize].island_sims[local_index as usize] = moved_sim;
        self.islands[move_island_id as usize].local_index = local_index;
        self.solver_sets[set_index as usize].island_sims.pop();

        island.bodies.clear();
        island.contacts.clear();
        island.joints.clear();
        island.constraint_remove_count = 0;
        island.local_index = b3::NULL_INDEX;
        island.island_id = b3::NULL_INDEX;
        island.set_index = b3::NULL_INDEX;
        self.islands[island_id as usize] = island;

        self.island_id_pool.free(island_id);
    }

    // MARK: Body flags

    pub(crate) fn sync_body_flags(&mut self, body_index: i32) {
        let body = self.bodies[body_index as usize].clone();
        let mut sim = self.body_sim(&body);

        // Preserve the sim only flags.
        sim.flags = (sim.flags
            & (BodyFlags::IS_FAST.0 | BodyFlags::HAD_TIME_OF_IMPACT.0))
            | (body.flags & !BodyFlags::BODY_TRANSIENT_FLAGS.0);
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize] = sim;

        if body.set_index == SetType::AwakeSet.raw_value() {
            let awake = SetType::AwakeSet.raw_value() as usize;
            let local_index = body.local_index as usize;
            let mut state = self.solver_sets[awake].body_states[local_index];
            state.flags = body.flags & !BodyFlags::BODY_TRANSIENT_FLAGS.0;
            self.solver_sets[awake].body_states[local_index] = state;
        }

        // Swift re-reads the (possibly re-located) body into the local copy here; the copy is
        // not observed afterwards, so this mirrors the dead read.
        let _body = self.bodies[body_index as usize].clone();
    }

    // MARK: Body creation

    /// Create a body. Returns a null id if the world cannot accept it.
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_body(&mut self, def: &BodyDef) -> BodyID {
        b3_assert(is_valid_position(def.position));
        b3_assert(is_valid_quat(def.rotation));
        b3_assert(is_valid_vec3(def.linear_velocity));
        b3_assert(is_valid_vec3(def.angular_velocity));
        b3_assert(is_valid_float(def.linear_damping) && def.linear_damping >= 0.0);
        b3_assert(is_valid_float(def.angular_damping) && def.angular_damping >= 0.0);
        b3_assert(is_valid_float(def.sleep_threshold) && def.sleep_threshold >= 0.0);
        b3_assert(is_valid_float(def.safety_factor) && def.safety_factor >= 0.0);
        b3_assert(is_valid_float(def.gravity_scale));

        let is_awake = (def.is_awake || !def.enable_sleep) && def.is_enabled;

        // determine the solver set
        let set_id: i32;
        if !def.is_enabled {
            set_id = SetType::DisabledSet.raw_value();
        } else if def.type_ == BodyType::StaticBody {
            set_id = SetType::StaticSet.raw_value();
        } else if is_awake {
            set_id = SetType::AwakeSet.raw_value();
        } else {
            // new set for a sleeping body in its own island
            set_id = self.solver_set_id_pool.alloc();
            if set_id as usize == self.solver_sets.len() {
                self.solver_sets.push(SolverSet::default());
            } else {
                b3_assert(self.solver_sets[set_id as usize].set_index == b3::NULL_INDEX);
            }

            self.solver_sets[set_id as usize].set_index = set_id;
        }

        b3_assert(0 <= set_id && set_id as usize < self.solver_sets.len());

        let body_id = self.body_id_pool.alloc();

        let mut lock_flags: u32 = 0;
        lock_flags |= if def.motion_locks.linear_x { BodyFlags::LOCK_LINEAR_X.0 } else { 0 };
        lock_flags |= if def.motion_locks.linear_y { BodyFlags::LOCK_LINEAR_Y.0 } else { 0 };
        lock_flags |= if def.motion_locks.linear_z { BodyFlags::LOCK_LINEAR_Z.0 } else { 0 };
        lock_flags |= if def.motion_locks.angular_x { BodyFlags::LOCK_ANGULAR_X.0 } else { 0 };
        lock_flags |= if def.motion_locks.angular_y { BodyFlags::LOCK_ANGULAR_Y.0 } else { 0 };
        lock_flags |= if def.motion_locks.angular_z { BodyFlags::LOCK_ANGULAR_Z.0 } else { 0 };

        let mut sim = BodySim::default();
        sim.transform.p = def.position;
        sim.transform.q = def.rotation;
        sim.center = def.position;
        sim.rotation0 = sim.transform.q;
        sim.center0 = sim.center;
        sim.local_center = Vec3::default();
        sim.force = Vec3::default();
        sim.torque = Vec3::default();
        sim.inv_mass = 0.0;
        sim.inertia = Matrix3::default();
        sim.inv_inertia_local = Matrix3::default();
        sim.min_extent = b3::huge();
        sim.max_extent = Vec3::default();
        sim.linear_damping = def.linear_damping;
        sim.angular_damping = def.angular_damping;
        sim.gravity_scale = def.gravity_scale;
        sim.body_id = body_id;
        sim.flags = lock_flags;
        sim.flags |= if def.is_bullet { BodyFlags::IS_BULLET.0 } else { 0 };
        sim.flags |= if def.allow_fast_rotation { BodyFlags::ALLOW_FAST_ROTATION.0 } else { 0 };
        sim.flags |= if def.type_ == BodyType::DynamicBody { BodyFlags::DYNAMIC_FLAG.0 } else { 0 };
        sim.flags |= if def.enable_sleep { BodyFlags::ENABLE_SLEEP.0 } else { 0 };
        sim.flags |= if def.enable_contact_recycling { BodyFlags::ENABLE_CONTACT_RECYCLING.0 } else { 0 };

        self.solver_sets[set_id as usize].body_sims.push(sim);
        let local_index =
            self.solver_sets[set_id as usize].body_sims.len() as i32 - 1;

        if set_id == SetType::AwakeSet.raw_value() {
            let awake = SetType::AwakeSet.raw_value() as usize;
            let mut state = BodyState::new();
            state.linear_velocity = def.linear_velocity;
            state.angular_velocity = def.angular_velocity;
            state.delta_rotation = Quat::IDENTITY;
            state.flags = sim.flags;
            self.solver_sets[awake].body_states.push(state);
        }

        if body_id as usize == self.bodies.len() {
            self.bodies.push(Body::default());
        } else {
            b3_assert(self.bodies[body_id as usize].id == b3::NULL_INDEX);
        }

        // Swift's `names.add(def.name)` mutates only the name cache; compute it up front so the
        // body-slot borrow below does not overlap it.
        let name_id = self.names.add(def.name.as_deref());

        let mut body = self.bodies[body_id as usize].clone();
        body.user_data = def.user_data;
        body.set_index = set_id;
        body.local_index = local_index;
        body.generation = body.generation.wrapping_add(1);
        body.head_shape_id = b3::NULL_INDEX;
        body.shape_count = 0;
        body.head_chain_id = b3::NULL_INDEX;
        body.head_contact_key = b3::NULL_INDEX;
        body.contact_count = 0;
        body.head_joint_key = b3::NULL_INDEX;
        body.joint_count = 0;
        body.island_id = b3::NULL_INDEX;
        body.island_index = b3::NULL_INDEX;
        body.body_move_index = b3::NULL_INDEX;
        body.id = body_id;
        body.sleep_threshold = def.sleep_threshold;
        body.sleep_time = 0.0;
        body.sleep_velocity = 0.0;
        body.safety_factor = def.safety_factor;
        body.mass = 0.0;
        body.name_id = name_id;
        body.type_ = def.type_;
        body.flags = sim.flags;
        self.bodies[body_id as usize] = body;

        // dynamic and kinematic bodies that are enabled need an island
        if set_id >= SetType::AwakeSet.raw_value() {
            self.create_island_for_body(set_id, body_id);
        }

        self.make_body_id(body_id)
    }

    pub(crate) fn create_island_for_body(&mut self, set_index: i32, body_id: i32) {
        let mut body = self.bodies[body_id as usize].clone();
        b3_assert(body.island_id == b3::NULL_INDEX);
        b3_assert(set_index != SetType::DisabledSet.raw_value());

        let island_id = self.create_island(set_index);
        self.islands[island_id as usize].bodies.push(body_id);
        body.island_id = island_id;
        body.island_index = 0;
        self.bodies[body_id as usize] = body;
    }

    pub(crate) fn remove_body_from_island(&mut self, body_id: i32) {
        let mut body = self.bodies[body_id as usize].clone();

        if body.island_id == b3::NULL_INDEX {
            b3_assert(body.island_index == b3::NULL_INDEX);
            return;
        }

        let island_id = body.island_id;
        {
            let local_index = body.island_index as usize;
            let island_body_count = self.islands[island_id as usize].bodies.len();
            let moved_body_id = self.islands[island_id as usize].bodies[island_body_count - 1];
            self.islands[island_id as usize].bodies[local_index] = moved_body_id;
            b3_validate(
                self.bodies[moved_body_id as usize].island_index
                    == island_body_count as i32 - 1,
            );
            self.bodies[moved_body_id as usize].island_index = local_index as i32;
            self.islands[island_id as usize].bodies.pop();
        }

        if self.islands[island_id as usize].bodies.is_empty() {
            b3_assert(self.islands[island_id as usize].contacts.is_empty());
            b3_assert(self.islands[island_id as usize].joints.is_empty());
            self.destroy_island(island_id);
        }

        body = self.bodies[body_id as usize].clone();
        body.island_id = b3::NULL_INDEX;
        body.island_index = b3::NULL_INDEX;
        self.bodies[body_id as usize] = body;
    }

    // MARK: Body destruction

    pub fn destroy_body(&mut self, body_id: BodyID) {
        b3_assert(self.is_valid_body(body_id));

        let id = self.body_index(body_id);

        let mut joint_key = self.bodies[id as usize].head_joint_key;
        while joint_key != b3::NULL_INDEX {
            let joint_id = joint_key >> 1;
            let edge_index = (joint_key & 1) as usize;
            joint_key = self.joints[joint_id as usize].edges[edge_index].next_key;
            self.destroy_joint(joint_id, true);
        }
        self.destroy_body_contacts(id, true);
        // Destroy the attached shapes and their broad-phase proxies.
        let mut shape_id = self.bodies[id as usize].head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let mut shape = self.shapes[shape_id as usize].clone();

            if shape.type_ == ShapeType::CompoundShape {
                b3_assert(self.compound_shape_count > 0);
                self.compound_shape_count -= 1;
            }

            if shape.sensor_index != b3::NULL_INDEX {
                self.destroy_sensor(shape_id);
            }
            // Swift class semantics: `destroyShapeProxy(&shape, broadPhase)` and
            // `destroyShapeAllocations(self, &shape)` mutate the proxy tree / hull database
            // in place. In Rust these take the owning field mutably.
            destroy_shape_proxy(&mut shape, &mut self.broad_phase);
            destroy_shape_allocations(&mut self.hull_database, &mut shape);

            self.shapes[shape_id as usize] = shape;

            // Return shape to free list.
            self.shape_id_pool.free(shape_id);
            self.shapes[shape_id as usize].id = b3::NULL_INDEX;

            shape_id = shape.next_shape_id;
        }

        self.remove_body_from_island(id);

        // Remove body sim from solver set that owns it
        let mut body = self.bodies[id as usize].clone();
        let set_index = body.set_index as usize;
        let local_index = body.local_index as usize;
        // Swift operated on a copy-on-write `set` snapshot and committed it back at the end;
        // the direct in-place mutation below produces the same final solver-set state.
        let moved_index = remove_swap_array(&mut self.solver_sets[set_index].body_sims, local_index);
        if moved_index != b3::NULL_INDEX {
            // Fix moved body index
            let moved_sim = self.solver_sets[set_index].body_sims[local_index];
            let moved_id = moved_sim.body_id;
            b3_assert(self.bodies[moved_id as usize].local_index == moved_index);
            self.bodies[moved_id as usize].local_index = body.local_index;
            self.refresh_body_contact_indices(moved_id);
        }

        // Remove body state from awake set
        if body.set_index == SetType::AwakeSet.raw_value() {
            let result =
                remove_swap_array(&mut self.solver_sets[set_index].body_states, local_index);
            b3_assert(result == moved_index);
        }
        if self.solver_sets[set_index].set_index >= SetType::FirstSleepingSet.raw_value()
            && self.solver_sets[set_index].body_sims.is_empty()
        {
            // Commit the removal before destroying the now-orphaned solver set.
            let orphan_set_index = self.solver_sets[set_index].set_index;
            self.destroy_solver_set(orphan_set_index);
        }

        // Free body and id (preserve body revision)
        self.body_id_pool.free(body.id);

        body.set_index = b3::NULL_INDEX;
        body.local_index = b3::NULL_INDEX;
        body.id = b3::NULL_INDEX;
        self.bodies[id as usize] = body;
    }

    pub(crate) fn destroy_solver_set(&mut self, set_index: i32) {
        let set = &mut self.solver_sets[set_index as usize];
        set.body_sims.clear();
        set.body_states.clear();
        set.contact_indices.clear();
        set.joint_sims.clear();
        set.island_sims.clear();

        self.solver_set_id_pool.free(set_index);
        set.set_index = b3::NULL_INDEX;
    }

    // MARK: Body state

    pub(crate) fn is_body_awake(&self, body: &Body) -> bool {
        body.set_index == SetType::AwakeSet.raw_value()
    }

    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub(crate) fn wake_body(&mut self, body_id: i32) -> bool {
        if self.bodies[body_id as usize].set_index >= SetType::FirstSleepingSet.raw_value() {
            let set_index = self.bodies[body_id as usize].set_index;
            self.wake_solver_set(set_index);
            return true;
        }

        false
    }
}

/// Swap-remove an element from a Vec. Returns the moved index or `B3.nullIndex`.
/// Swift's `index` was `Int`; Rust keeps it `usize` so the negative-index assert is
/// structurally impossible (callers pass validated indices).
pub(crate) fn remove_swap_array<T>(array: &mut Vec<T>, index: usize) -> i32 {
    b3_assert(index < array.len());
    let last = array.len() - 1;
    if index != last {
        array[index] = array.remove(last);
        // Swift wrote array[index] = array[last] and then removeLast(); the remove-then-store
        // above is the same single move for a value-type array.
        return last as i32;
    }
    array.pop();
    b3::NULL_INDEX
}
