// Kinematic character controller. The character is a capsule moved by collide-and-slide against
// the world, outside the rigid body step. It detects ground and slopes, walks up stairs, sticks to
// the floor going down slopes and steps, rides moving platforms, pushes dynamic bodies, and can own
// a kinematic inner body so rigid bodies and queries see it.
//
// Translated from `Sources/Natsel/Character/CharacterVirtual.swift`.
//
// Foreign vocabulary assumed from sibling modules (see MIGRATION.md "Shared simulation
// vocabulary"):
// - `crate::simulation::filtered_queries`: `QueryHit` and the `World` methods
//   `overlaps_ignoring`, `collide_mover_ignoring`, `closest_shape_cast_hit`, `point_velocity`
//   (Swift's `FilteredQueries.swift`).
// - `crate::simulation::shape_creation::World::create_capsule_shape`,
//   `crate::simulation::body_api::World::{set_body_transform, apply_body_linear_impulse}`,
//   `crate::simulation::body_queries::World::{body_type, set_body_target_transform}`,
//   `crate::simulation::world_runtime::World::{create_body, destroy_body, shapes, world_id,
//   make_body_id, body_index}`, `crate::simulation::world_events::World::body_is_valid`.
// - `crate::collision::mover::plane_solver::{solve_planes, clip_vector}` (Swift's `solvePlanes` /
//   `clipVector`).
//
// Swift holds the world as `unowned let world`, a non-owning class reference usable for both
// read-only queries and mutations. Rust models that as a single `&'w mut World` field: the
// callbacks below only ever read the world while they write the character's own vectors, so the
// split field borrows compile, mirroring Swift's unchecked class-exclusivity aliasing.
//
// Swift's `maxSlopeAngle` carries a `didSet` observer. Rust has no observers; `set_max_slope_angle`
// reproduces it (the stored field stays `pub` exactly as Swift's `public var`, but a direct field
// write skips the observer, so callers must use the setter). Swift's defaulted arguments
// (`setShape(maxPenetration: 0)`, `setBodyTargetTransform(wake: true)`) are passed explicitly by
// ported callers per the migration convention.

use std::ffi::c_void;

use crate::base::constants as b3;
use crate::collision::mover::plane_solver::{clip_vector, solve_planes};
use crate::math::functions::{
    compute_cos_sin, dot, is_valid_position, length, length_squared, max_float, min_float, mul_add,
    normalize, offset_pos, sub_pos,
};
use crate::math::types::{Pos, Quat, Vec3, WorldTransform};
use crate::simulation::filtered_queries::QueryHit;
use crate::simulation::shape_runtime::Shape;
use crate::simulation::world_runtime::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Capsule;
use crate::types::ids::{BodyID, ShapeID};
use crate::types::queries::{CollisionPlane, ShapeProxy};
use crate::types::shape_def::{QueryFilter, ShapeDef};

/// Ground state of the character after the last contact refresh.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum CharacterGroundState {
    /// Standing on walkable ground.
    #[default]
    OnGround = 0,
    /// Touching ground that is too steep to stand on.
    OnSteepGround = 1,
    /// Touching something, but nothing below.
    NotSupported = 2,
    /// Touching nothing.
    InAir = 3,
}

/// A shape touched by the character in the last update.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CharacterContact {
    pub shape_id: ShapeID,
    pub body_id: BodyID,
    pub position: Pos,
    /// Points from the shape toward the character.
    pub normal: Vec3,
    /// Penetration depth with the padded capsule.
    pub depth: f32,
    pub user_material_id: u64,
}

/// Creation parameters for a [`CharacterVirtual`], in Swift field order.
#[derive(Clone, Debug)]
pub struct CharacterDef {
    /// Position of the character's feet: the bottom of the capsule.
    pub position: Pos,
    pub up: Vec3,

    /// Capsule radius and total height (at least twice the radius).
    pub radius: f32,
    pub height: f32,

    /// Steepest walkable slope.
    pub max_slope_angle: f32,

    /// Used to push dynamic bodies and to weigh down the ground.
    pub mass: f32,
    /// Largest force the character can push with (N).
    pub max_strength: f32,

    /// Skin kept between the capsule and the world.
    pub character_padding: f32,

    /// Contacts this far beyond the padding are still reported, so the character knows about the
    /// ground before it lands.
    pub predictive_contact_distance: f32,

    /// Collide and slide passes per update.
    pub max_collision_iterations: i32,

    /// Highest step `extended_update` walks up.
    pub step_up_height: f32,
    /// How far `extended_update` looks down to stay on the floor.
    pub step_down_height: f32,

    /// When a stair step lands on the edge of the step, look this much farther ahead for its
    /// walkable top before giving up.
    pub stair_step_forward_test: f32,

    pub enable_stair_stepping: bool,
    pub enable_stick_to_floor: bool,

    /// Move with the body the character stands on.
    pub inherit_ground_velocity: bool,

    pub filter: QueryFilter,

    /// Create a kinematic capsule body that follows the character, so rigid bodies collide with it
    /// and queries find it. The character itself ignores it.
    pub create_inner_body: bool,
    pub inner_body_shape: ShapeDef,

    pub user_data: Option<*mut c_void>,
}

/// Swift's property initializers of `CharacterDef`.
impl CharacterDef {
    pub fn new() -> Self {
        let units = b3::length_units_per_meter();
        Self {
            position: Pos::default(),
            up: Vec3::new(0.0, 1.0, 0.0),
            radius: 0.3 * units,
            height: 1.8 * units,
            max_slope_angle: 50.0 * b3::DEG_TO_RAD,
            mass: 70.0,
            max_strength: 100.0,
            character_padding: 0.02 * units,
            predictive_contact_distance: 0.1 * units,
            max_collision_iterations: 5,
            step_up_height: 0.4 * units,
            step_down_height: 0.5 * units,
            stair_step_forward_test: 0.15 * units,
            enable_stair_stepping: true,
            enable_stick_to_floor: true,
            inherit_ground_velocity: true,
            filter: QueryFilter::default_filter(),
            create_inner_body: true,
            inner_body_shape: ShapeDef::default(),
            user_data: None,
        }
    }
}

impl Default for CharacterDef {
    fn default() -> Self {
        Self::new()
    }
}

/// Everything a character carries between updates. Save it together with
/// `World.save_state` (the inner body lives in the world) to roll both back.
///
/// Swift's `CharacterVirtual.State`; its `fileprivate` fields become `pub(crate)` so
/// `save_state` / `restore_state` can read and write them.
#[derive(Clone, Debug)]
pub struct State {
    pub(crate) position: Pos,
    pub(crate) rotation: Quat,
    pub(crate) linear_velocity: Vec3,
    pub(crate) up: Vec3,
    pub(crate) radius: f32,
    pub(crate) height: f32,
    pub(crate) ground_state: CharacterGroundState,
    pub(crate) ground_normal: Vec3,
    pub(crate) ground_position: Pos,
    pub(crate) ground_velocity: Vec3,
    pub(crate) ground_body_id: BodyID,
    pub(crate) ground_shape_id: ShapeID,
    pub(crate) ground_user_material_id: u64,
    pub(crate) contacts: Vec<CharacterContact>,
    pub(crate) inner_body_id: BodyID,
}

/// Return false to ignore a shape. Swift's `((ShapeID, BodyID) -> Bool)?`.
type ContactFilter = dyn Fn(ShapeID, BodyID) -> bool;

/// A virtual character driven by [`CharacterVirtual::update`] or
/// [`CharacterVirtual::extended_update`] once per frame.
pub struct CharacterVirtual<'w> {
    world: &'w mut World,

    /// Position of the feet. `private(set)` in Swift; read through [`Self::position`].
    position: Pos,
    /// Facing rotation. The capsule is symmetric about `up`, so this only affects the inner body.
    pub rotation: Quat,

    /// Desired velocity. The update clips it against what the character touched.
    pub linear_velocity: Vec3,

    up: Vec3,
    radius: f32,
    height: f32,
    /// Steepest walkable slope. Swift observed this with `didSet`; use
    /// [`Self::set_max_slope_angle`] to keep the cached cosine in step.
    pub max_slope_angle: f32,
    pub mass: f32,
    pub max_strength: f32,
    pub character_padding: f32,
    pub predictive_contact_distance: f32,
    pub max_collision_iterations: i32,
    pub step_up_height: f32,
    pub step_down_height: f32,
    pub stair_step_forward_test: f32,
    pub enable_stair_stepping: bool,
    pub enable_stick_to_floor: bool,
    pub inherit_ground_velocity: bool,
    pub filter: QueryFilter,
    pub user_data: Option<*mut c_void>,

    /// Return false to ignore a shape.
    pub contact_filter: Option<Box<ContactFilter>>,

    ground_state: CharacterGroundState,
    ground_normal: Vec3,
    ground_position: Pos,
    ground_velocity: Vec3,
    ground_body_id: BodyID,
    ground_shape_id: ShapeID,
    ground_user_material_id: u64,
    contacts: Vec<CharacterContact>,
    inner_body_id: BodyID,

    cos_max_slope: f32,
    inner_body_shape: Option<ShapeDef>,
    planes: Vec<CollisionPlane>,
    plane_contacts: Vec<CharacterContact>,
}

impl<'w> CharacterVirtual<'w> {
    // MARK: Accessors for Swift's `private(set)` members.

    /// Position of the feet.
    #[inline]
    pub fn position(&self) -> Pos {
        self.position
    }

    #[inline]
    pub fn up(&self) -> Vec3 {
        self.up
    }

    #[inline]
    pub fn radius(&self) -> f32 {
        self.radius
    }

    #[inline]
    pub fn height(&self) -> f32 {
        self.height
    }

    /// Swift's `didSet` observer on `maxSlopeAngle`.
    pub fn set_max_slope_angle(&mut self, angle: f32) {
        self.max_slope_angle = angle;
        self.cos_max_slope = compute_cos_sin(self.max_slope_angle).cosine;
    }

    #[inline]
    pub fn ground_state(&self) -> CharacterGroundState {
        self.ground_state
    }

    #[inline]
    pub fn ground_normal(&self) -> Vec3 {
        self.ground_normal
    }

    #[inline]
    pub fn ground_position(&self) -> Pos {
        self.ground_position
    }

    #[inline]
    pub fn ground_velocity(&self) -> Vec3 {
        self.ground_velocity
    }

    #[inline]
    pub fn ground_body_id(&self) -> BodyID {
        self.ground_body_id
    }

    #[inline]
    pub fn ground_shape_id(&self) -> ShapeID {
        self.ground_shape_id
    }

    #[inline]
    pub fn ground_user_material_id(&self) -> u64 {
        self.ground_user_material_id
    }

    #[inline]
    pub fn contacts(&self) -> &[CharacterContact] {
        &self.contacts
    }

    #[inline]
    pub fn inner_body_id(&self) -> BodyID {
        self.inner_body_id
    }

    /// Shared view of the world for read-only queries.
    #[inline]
    fn world_ref(&self) -> &World {
        &*self.world
    }

    /// Swift's failable `init?(world:_:)`. Returns `None` when the definition is invalid.
    pub fn new(world: &'w mut World, def: &CharacterDef) -> Option<Self> {
        if !(def.radius > 0.0
            && def.height >= 2.0 * def.radius
            && is_valid_position(def.position)
            && length_squared(def.up) > 0.0)
        {
            return None;
        }
        let mut character = Self {
            world,
            position: def.position,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::default(),
            up: normalize(def.up),
            radius: def.radius,
            height: def.height,
            max_slope_angle: def.max_slope_angle,
            mass: def.mass,
            max_strength: def.max_strength,
            character_padding: def.character_padding,
            predictive_contact_distance: def.predictive_contact_distance,
            max_collision_iterations: def.max_collision_iterations.max(1),
            step_up_height: def.step_up_height,
            step_down_height: def.step_down_height,
            stair_step_forward_test: def.stair_step_forward_test,
            enable_stair_stepping: def.enable_stair_stepping,
            enable_stick_to_floor: def.enable_stick_to_floor,
            inherit_ground_velocity: def.inherit_ground_velocity,
            filter: def.filter.clone(),
            user_data: def.user_data,
            contact_filter: None,
            ground_state: CharacterGroundState::InAir,
            ground_normal: Vec3::default(),
            ground_position: Pos::default(),
            ground_velocity: Vec3::default(),
            ground_body_id: BodyID::default(),
            ground_shape_id: ShapeID::default(),
            ground_user_material_id: 0,
            contacts: Vec::new(),
            inner_body_id: BodyID::default(),
            // Swift's `didSet` does not fire during init; the cosine is seeded directly.
            cos_max_slope: compute_cos_sin(def.max_slope_angle).cosine,
            inner_body_shape: None,
            planes: Vec::new(),
            plane_contacts: Vec::new(),
        };

        if def.create_inner_body {
            character.inner_body_shape = Some(def.inner_body_shape.clone());
            character.create_inner_body();
        }
        character.refresh_contacts();
        Some(character)
    }

    fn create_inner_body(&mut self) {
        // Split disjoint field borrows so the world mutation below coexists with the
        // `inner_body_shape` read, exactly as Swift's class aliasing allowed.
        let Self {
            world,
            inner_body_shape,
            position,
            rotation,
            inner_body_id,
            radius,
            height,
            up,
            ..
        } = self;
        let Some(shape_def) = inner_body_shape.as_ref() else {
            return;
        };
        let mut body_def = BodyDef::default();
        body_def.type_ = BodyType::KinematicBody;
        body_def.position = *position;
        body_def.rotation = *rotation;
        body_def.enable_sleep = false;
        let mover = Capsule::new(*radius * *up, (*height - *radius) * *up, *radius);
        let w: &mut World = &mut **world;
        *inner_body_id = w.create_body(&body_def);
        let _shape_id = w.create_capsule_shape(*inner_body_id, shape_def, mover);
    }

    // MARK: State

    /// Save everything the character carries between updates.
    pub fn save_state(&self) -> State {
        State {
            position: self.position,
            rotation: self.rotation,
            linear_velocity: self.linear_velocity,
            up: self.up,
            radius: self.radius,
            height: self.height,
            ground_state: self.ground_state,
            ground_normal: self.ground_normal,
            ground_position: self.ground_position,
            ground_velocity: self.ground_velocity,
            ground_body_id: self.ground_body_id,
            ground_shape_id: self.ground_shape_id,
            ground_user_material_id: self.ground_user_material_id,
            contacts: self.contacts.clone(),
            inner_body_id: self.inner_body_id,
        }
    }

    pub fn restore_state(&mut self, state: State) {
        self.position = state.position;
        self.rotation = state.rotation;
        self.linear_velocity = state.linear_velocity;
        self.up = state.up;
        self.radius = state.radius;
        self.height = state.height;
        self.ground_state = state.ground_state;
        self.ground_normal = state.ground_normal;
        self.ground_position = state.ground_position;
        self.ground_velocity = state.ground_velocity;
        self.ground_body_id = state.ground_body_id;
        self.ground_shape_id = state.ground_shape_id;
        self.ground_user_material_id = state.ground_user_material_id;
        self.contacts = state.contacts;
        self.inner_body_id = state.inner_body_id;
    }

    /// Remove the inner body. Call before discarding the character if the world lives on.
    pub fn destroy(&mut self) {
        if !self.inner_body_id.is_null() && self.world_ref().body_is_valid(self.inner_body_id) {
            let id = self.inner_body_id;
            self.world.destroy_body(id);
        }
        self.inner_body_id = BodyID::default();
    }

    // MARK: Shape

    /// Capsule relative to the feet.
    fn capsule(&self, r: f32) -> Capsule {
        Capsule::new(
            self.radius * self.up,
            (self.height - self.radius) * self.up,
            r,
        )
    }

    fn ignore_body(&self) -> i32 {
        if self.inner_body_id.is_null() {
            b3::NULL_INDEX
        } else {
            self.world_ref().body_index(self.inner_body_id)
        }
    }

    /// Change the capsule, for example to crouch. Fails, keeping the old shape, when the new shape
    /// would overlap the world by more than `max_penetration`. Swift defaulted
    /// `maxPenetration` to 0; ported callers pass it explicitly.
    pub fn set_shape(&mut self, new_radius: f32, new_height: f32, max_penetration: f32) -> bool {
        if !(new_radius > 0.0 && new_height >= 2.0 * new_radius) {
            return false;
        }
        let shrink = max_penetration + b3::linear_slop();
        let test_radius = max_float(new_radius - shrink, 0.5 * new_radius);
        let points = [new_radius * self.up, (new_height - new_radius) * self.up];
        let ignore_body = self.ignore_body();
        let w = self.world_ref();
        // SAFETY: `points` is a local fixed array of two `Vec3` values that outlives the call;
        // the proxy borrows it exactly as Swift's `points.withUnsafeBufferPointer` did, and
        // `overlaps_ignoring` only reads the buffer for the duration of the call.
        let blocked = unsafe {
            let proxy = ShapeProxy::new(Some(points.as_ptr()), 2, test_radius);
            w.overlaps_ignoring(self.position, proxy, &self.filter, ignore_body, |shape_index| {
                accepts_shape(w, &self.contact_filter, shape_index)
            })
        };
        if blocked == false {
            self.radius = new_radius;
            self.height = new_height;
            if !self.inner_body_id.is_null() {
                // Rebuild the inner body with the new capsule.
                self.destroy();
                self.create_inner_body();
            }
            self.refresh_contacts();
            return true;
        }
        false
    }

    /// Teleport the character.
    pub fn set_position(&mut self, new_position: Pos) {
        if !is_valid_position(new_position) {
            return;
        }
        self.position = new_position;
        if !self.inner_body_id.is_null() {
            let (id, p, r) = (self.inner_body_id, self.position, self.rotation);
            self.world.set_body_transform(id, p, r);
        }
        self.refresh_contacts();
    }

    pub fn is_supported(&self) -> bool {
        self.ground_state == CharacterGroundState::OnGround
            || self.ground_state == CharacterGroundState::OnSteepGround
    }

    /// True when `normal` is walkable.
    pub fn is_slope_walkable(&self, normal: Vec3) -> bool {
        dot(normal, self.up) >= self.cos_max_slope
    }

    // MARK: Update

    /// Move by `linear_velocity * dt` with collide and slide, then refresh ground contact.
    pub fn update(&mut self, dt: f32) {
        if dt > 0.0 {
            let mut displacement = dt * self.linear_velocity;
            if self.inherit_ground_velocity && self.ground_state == CharacterGroundState::OnGround {
                displacement = mul_add(displacement, dt, self.ground_velocity);
            }

            let start = self.position;
            let was_on_ground = self.ground_state == CharacterGroundState::OnGround;
            self.move_and_slide(displacement, was_on_ground);

            // Do not keep velocity into what stopped us.
            self.linear_velocity = clip_vector(self.linear_velocity, &self.planes);

            self.refresh_contacts();
            self.push_bodies(dt, start);
            self.follow_inner_body(dt);
        }
    }

    /// `update` plus stair walking and sticking to the floor. `gravity` gives the down direction
    /// and is used to weigh down the ground body; the caller adds it to `linear_velocity` as usual.
    pub fn extended_update(&mut self, dt: f32, gravity: Vec3) {
        // Inverted rather than `dt <= 0.0` so a NaN `dt` takes Swift's early return.
        if !(dt > 0.0) {
            return;
        }
        let start = self.position;
        let was_on_ground = self.ground_state == CharacterGroundState::OnGround;
        let desired = dt * self.linear_velocity;
        let desired_horizontal = desired - dot(desired, self.up) * self.up;

        self.update(dt);

        // Walk stairs when the horizontal motion was blocked by something steep.
        if self.enable_stair_stepping
            && was_on_ground
            && length_squared(desired_horizontal) > 1.0e-10
        {
            let moved = self.position - start;
            let achieved = dot(moved - dot(moved, self.up) * self.up, normalize(desired_horizontal));
            let wanted = length(desired_horizontal);
            if achieved < 0.9 * wanted && self.touches_steep_contact(desired_horizontal) {
                self.walk_stairs(start, desired_horizontal, achieved, dt);
            }
        }

        // Stick to the floor when walking off a step or down a slope.
        if self.enable_stick_to_floor
            && was_on_ground
            && self.ground_state != CharacterGroundState::OnGround
            && dot(self.linear_velocity, self.up) <= 1.0e-3
        {
            self.stick_to_floor();
        }

        // Weigh down a dynamic ground body.
        if self.ground_state == CharacterGroundState::OnGround
            && !self.ground_body_id.is_null()
            && self.world_ref().body_is_valid(self.ground_body_id)
            && self.world_ref().body_type(self.ground_body_id) == BodyType::DynamicBody
        {
            let weight = dt * self.mass * dot(gravity, -self.ground_normal);
            if weight > 0.0 {
                let (id, normal, point) = (self.ground_body_id, self.ground_normal, self.ground_position);
                self.world.apply_body_linear_impulse(id, (-weight) * normal, point, true);
            }
        }
        self.follow_inner_body(dt);
    }

    // MARK: Collide and slide

    /// Collision planes of the padded capsule placed at `target`.
    fn gather_planes(&mut self, target: Pos, extra: f32, grounded_slopes: bool) {
        self.planes.clear();
        self.plane_contacts.clear();
        let mover = self.capsule(self.radius + self.character_padding + extra);
        let ignore_body = self.ignore_body();
        // Split disjoint field borrows: the callback writes `planes` / `plane_contacts` while it
        // only reads the world, `up`, `cos_max_slope` and `contact_filter`, as in Swift's
        // unchecked aliasing of the class fields. `&*self.world` is a shared reborrow of the
        // `world` field alone, so it coexists with the two mutable field borrows.
        let planes = &mut self.planes;
        let plane_contacts = &mut self.plane_contacts;
        let w: &World = &*self.world;
        let up = self.up;
        let cos_max_slope = self.cos_max_slope;
        w.collide_mover_ignoring(target, &mover, &self.filter, ignore_body, |shape_index, results| {
            if !accepts_shape(w, &self.contact_filter, shape_index) {
                return;
            }
            let shape = &w.shapes[shape_index as usize];
            for result in results {
                let mut plane = CollisionPlane::default();
                plane.plane = result.plane;
                plane.plane.offset -= extra;
                plane.push_limit = f32::MAX;
                plane.clip_velocity = true;

                // A slope too steep to walk acts as a wall for a grounded character.
                let n_up = dot(result.plane.normal, up);
                if grounded_slopes && n_up > 0.0 && n_up < cos_max_slope {
                    let horizontal = result.plane.normal - n_up * up;
                    let len = length(horizontal);
                    if len > 1.0e-4 {
                        plane.plane.normal = (1.0 / len) * horizontal;
                        plane.plane.offset /= len;
                    }
                }
                planes.push(plane);

                let mut contact = CharacterContact::default();
                contact.shape_id = ShapeID::new(shape.id + 1, w.world_id, shape.generation);
                contact.body_id = w.make_body_id(shape.body_id);
                contact.position = offset_pos(target, result.point);
                contact.normal = result.plane.normal;
                contact.depth = result.plane.offset - extra;
                contact.user_material_id = shape.material(result.material_index).user_material_id;
                plane_contacts.push(contact);
            }
        });
    }

    /// Cast the capsule. Returns the free fraction of `translation`, backed off by the padding.
    fn cast_fraction(&self, origin: Pos, translation: Vec3) -> (f32, QueryHit) {
        let len = length(translation);
        if len > 1.0e-7 {
            let c = self.capsule(self.radius);
            let points = [c.center1, c.center2];
            let w = self.world_ref();
            let ignore_body = self.ignore_body();
            // SAFETY: `points` is a local fixed array of the capsule's two `Vec3` centres that
            // outlives the call; the proxy borrows it exactly as Swift's
            // `points.withUnsafeBufferPointer` did, and `closest_shape_cast_hit` only reads the
            // buffer for the duration of the call.
            let hit = unsafe {
                let proxy = ShapeProxy::new(Some(points.as_ptr()), 2, self.radius);
                w.closest_shape_cast_hit(
                    origin,
                    proxy,
                    translation,
                    &self.filter,
                    ignore_body,
                    true,
                    |_| true,
                )
            };
            if hit.hit == false {
                return (1.0, hit);
            }
            let backed = max_float(0.0, hit.fraction - self.character_padding / len);
            (backed, hit)
        } else {
            (1.0, QueryHit::default())
        }
    }

    fn move_and_slide(&mut self, displacement: Vec3, on_ground: bool) {
        let mut target = offset_pos(self.position, displacement);
        let tolerance = 0.1 * b3::linear_slop();
        // Swift: `for _ in 0..<Int(maxCollisionIterations)`; `max(0)` keeps a negative count an
        // empty range instead of trapping, matching Swift's half-open range semantics.
        for _ in 0..self.max_collision_iterations.max(0) as usize {
            self.gather_planes(target, 0.0, on_ground);
            let solved = solve_planes(Vec3::default(), &mut self.planes);
            let corrected = offset_pos(target, solved.delta);
            let translation = sub_pos(corrected, self.position);
            let (fraction, _hit) = self.cast_fraction(self.position, translation);
            let delta = fraction * translation;
            self.position = offset_pos(self.position, delta);
            target = corrected;
            if length_squared(delta) < tolerance * tolerance
                || fraction == 1.0
                    && length_squared(sub_pos(target, self.position)) < tolerance * tolerance
            {
                break;
            }
        }
    }

    // MARK: Ground

    /// Contacts and ground state at the current position.
    fn refresh_contacts(&mut self) {
        self.gather_planes(self.position, self.predictive_contact_distance, false);
        // Velocity clipping uses only touching planes.
        self.planes.retain(|plane| !(plane.plane.offset < -b3::linear_slop()));
        self.contacts = self.plane_contacts.clone();

        let mut best_up: f32 = -1.0;
        let mut best: i32 = -1;
        let mut support_sum = Vec3::default();
        let mut touching = false;
        let character_padding = self.character_padding;
        let up = self.up;
        for (i, contact) in self.contacts.iter().enumerate() {
            if contact.depth > -(character_padding + b3::linear_slop()) {
                touching = true;
                let n_up = dot(contact.normal, up);
                if n_up > 0.0 {
                    support_sum += contact.normal;
                }
                if n_up > best_up {
                    best_up = n_up;
                    best = i as i32;
                }
            }
        }

        self.ground_body_id = BodyID::default();
        self.ground_shape_id = ShapeID::default();
        self.ground_velocity = Vec3::default();
        self.ground_user_material_id = 0;
        if touching == false {
            self.ground_state = CharacterGroundState::InAir;
            self.ground_normal = Vec3::default();
            return;
        }
        if !(best >= 0 && best_up > 1.0e-3) {
            self.ground_state = CharacterGroundState::NotSupported;
            self.ground_normal = Vec3::default();
            return;
        }

        let ground = self.contacts[best as usize];
        // Two steep slopes facing each other still hold the character.
        let walkable = best_up >= self.cos_max_slope
            || (length_squared(support_sum) > 0.0
                && dot(normalize(support_sum), up) >= self.cos_max_slope);
        self.ground_state = if walkable {
            CharacterGroundState::OnGround
        } else {
            CharacterGroundState::OnSteepGround
        };
        self.ground_normal = if walkable && best_up < self.cos_max_slope {
            normalize(support_sum)
        } else {
            ground.normal
        };
        self.ground_position = ground.position;
        self.ground_body_id = ground.body_id;
        self.ground_shape_id = ground.shape_id;
        self.ground_user_material_id = ground.user_material_id;
        if self.world_ref().body_is_valid(ground.body_id) {
            let w = self.world_ref();
            self.ground_velocity = w.point_velocity(w.body_index(ground.body_id), ground.position);
        }
    }

    fn touches_steep_contact(&self, horizontal: Vec3) -> bool {
        let direction = normalize(horizontal);
        let character_padding = self.character_padding;
        let up = self.up;
        let cos_max_slope = self.cos_max_slope;
        self.contacts.iter().any(|contact| {
            contact.depth > -(character_padding + 2.0 * b3::linear_slop())
                && dot(contact.normal, up) < cos_max_slope
                && dot(contact.normal, direction) < -0.1
        })
    }

    /// Up, forward, down. Accepts the result when it lands on walkable ground and gets farther.
    fn walk_stairs(&mut self, start: Pos, horizontal: Vec3, achieved: f32, dt: f32) {
        let saved = (self.position, self.linear_velocity, self.ground_state);
        self.position = start;

        // Up, stopping at the ceiling.
        let rise = self
            .cast_fraction(self.position, self.step_up_height * self.up)
            .0
            * self.step_up_height;
        if rise > b3::linear_slop() {
            self.position = offset_pos(self.position, rise * self.up);

            // Forward.
            self.move_and_slide(horizontal, false);

            // Down onto the step.
            let drop = rise + self.character_padding + b3::linear_slop();
            let (fraction, hit) = self.cast_fraction(self.position, (-drop) * self.up);
            let direction = normalize(horizontal);
            let moved = self.position - start;
            let progress = dot(moved - dot(moved, self.up) * self.up, direction);
            if hit.hit && progress > achieved + b3::linear_slop() {
                let mut supported = self.is_slope_walkable(hit.normal);
                if supported == false && self.stair_step_forward_test > 0.0 {
                    // Landed on the edge of the step: the top must be walkable a little farther ahead.
                    let ahead = offset_pos(self.position, self.stair_step_forward_test * direction);
                    let blocked = self
                        .cast_fraction(self.position, self.stair_step_forward_test * direction)
                        .0
                        < 1.0;
                    if blocked == false {
                        let (_, test) = self.cast_fraction(ahead, (-drop) * self.up);
                        supported = test.hit && self.is_slope_walkable(test.normal);
                    }
                }
                if supported {
                    self.position = offset_pos(self.position, ((-fraction) * drop) * self.up);
                    self.refresh_contacts();
                    if self.ground_state == CharacterGroundState::OnSteepGround {
                        // Standing on the step's edge counts as ground; the next move finishes the step.
                        self.ground_state = CharacterGroundState::OnGround;
                        self.ground_normal = self.up;
                    }
                    return;
                }
            }
        }
        (self.position, self.linear_velocity, self.ground_state) = saved;
        self.refresh_contacts();
    }

    fn stick_to_floor(&mut self) {
        let (fraction, hit) = self.cast_fraction(self.position, (-self.step_down_height) * self.up);
        if hit.hit && self.is_slope_walkable(hit.normal) {
            self.position =
                offset_pos(self.position, ((-fraction) * self.step_down_height) * self.up);
            self.refresh_contacts();
        }
    }

    // MARK: Interaction

    /// Push dynamic bodies the character walked into, limited by `max_strength`.
    fn push_bodies(&mut self, dt: f32, start: Pos) {
        let velocity = (1.0 / dt) * sub_pos(self.position, start) + self.linear_velocity;
        let max_impulse = self.max_strength * dt;
        let character_padding = self.character_padding;
        let mass = self.mass;
        // Split disjoint field borrows: the loop reads `contacts` while mutating the world, as in
        // Swift's class aliasing.
        let contacts = &self.contacts;
        let w = &mut self.world;
        for contact in contacts.iter() {
            if contact.depth > -(character_padding + b3::linear_slop()) {
                if w.body_is_valid(contact.body_id)
                    && w.body_type(contact.body_id) == BodyType::DynamicBody
                {
                    let body_velocity =
                        w.point_velocity(w.body_index(contact.body_id), contact.position);
                    let approach = dot(velocity - body_velocity, -contact.normal);
                    if approach > 0.0 {
                        let impulse = min_float(mass * approach, max_impulse);
                        w.apply_body_linear_impulse(
                            contact.body_id,
                            (-impulse) * contact.normal,
                            contact.position,
                            true,
                        );
                    }
                }
            }
        }
    }

    fn follow_inner_body(&mut self, dt: f32) {
        if self.inner_body_id.is_null() == false && self.world_ref().body_is_valid(self.inner_body_id)
        {
            // Swift defaulted `wake` to true in `setBodyTargetTransform`; passed explicitly here.
            let (id, target) = (
                self.inner_body_id,
                WorldTransform::new(self.position, self.rotation),
            );
            self.world.set_body_target_transform(id, target, dt, true);
        }
    }
}

/// Swift's `CharacterVirtual.accepts(_:)`, taken as free parameters so the query callbacks above
/// can call it while they hold a mutable split borrow of the character's plane vectors.
fn accepts_shape(
    world: &World,
    contact_filter: &Option<Box<ContactFilter>>,
    shape_index: i32,
) -> bool {
    let Some(filter) = contact_filter.as_ref() else {
        return true;
    };
    let shape: &Shape = &world.shapes[shape_index as usize];
    let shape_id = ShapeID::new(shape.id + 1, world.world_id, shape.generation);
    filter(shape_id, world.make_body_id(shape.body_id))
}
