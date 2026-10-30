//! Basic vector, rotation, transform, and matrix types.

/// A 2D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub const ZERO: Vec2 = Vec2::new(0.0, 0.0);
    pub const ONE: Vec2 = Vec2::new(1.0, 1.0);
}

/// A 3D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const ZERO: Vec3 = Vec3::new(0.0, 0.0, 0.0);
    pub const ONE: Vec3 = Vec3::new(1.0, 1.0, 1.0);
    pub const AXIS_X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    pub const AXIS_Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
    pub const AXIS_Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
}

/// Cosine and sine pair.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct CosSin {
    pub cosine: f32,
    pub sine: f32,
}

impl CosSin {
    #[inline]
    pub const fn new(cosine: f32, sine: f32) -> Self {
        Self { cosine, sine }
    }
}

/// A quaternion. The default value is all zero; use [`Quat::IDENTITY`] for the unit rotation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Quat {
    pub v: Vec3,
    pub s: f32,
}

impl Quat {
    #[inline]
    pub const fn new(v: Vec3, s: f32) -> Self {
        Self { v, s }
    }

    pub const IDENTITY: Quat = Quat::new(Vec3::ZERO, 1.0);
}

/// A rigid transform.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Transform {
    pub p: Vec3,
    pub q: Quat,
}

impl Transform {
    #[inline]
    pub const fn new(p: Vec3, q: Quat) -> Self {
        Self { p, q }
    }

    pub const IDENTITY: Transform = Transform::new(Vec3::ZERO, Quat::IDENTITY);
}

/// A double precision world position. Subtract two positions before narrowing to a local vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Pos {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Pos {
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn from_vec3(v: Vec3) -> Self {
        Self {
            x: v.x as f64,
            y: v.y as f64,
            z: v.z as f64,
        }
    }

    pub const ZERO: Pos = Pos::new(0.0, 0.0, 0.0);
}

/// Double precision translation with a single precision rotation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct WorldTransform {
    pub p: Pos,
    pub q: Quat,
}

impl WorldTransform {
    #[inline]
    pub const fn new(p: Pos, q: Quat) -> Self {
        Self { p, q }
    }

    #[inline]
    pub fn from_transform(t: Transform, q: Quat) -> Self {
        Self {
            p: Pos::from_vec3(t.p),
            q,
        }
    }

    pub const IDENTITY: WorldTransform = WorldTransform::new(Pos::ZERO, Quat::IDENTITY);
}

/// A 3x3 matrix.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Matrix3 {
    pub cx: Vec3,
    pub cy: Vec3,
    pub cz: Vec3,
}

impl Matrix3 {
    #[inline]
    pub const fn new(cx: Vec3, cy: Vec3, cz: Vec3) -> Self {
        Self { cx, cy, cz }
    }

    pub const ZERO: Matrix3 = Matrix3::new(Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
    pub const IDENTITY: Matrix3 = Matrix3::new(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
}

/// Axis aligned bounding box.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct AABB {
    pub lower_bound: Vec3,
    pub upper_bound: Vec3,
}

impl AABB {
    #[inline]
    pub const fn new(lower_bound: Vec3, upper_bound: Vec3) -> Self {
        Self {
            lower_bound,
            upper_bound,
        }
    }
}

/// A plane: `separation = dot(normal, point) - offset`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Plane {
    pub normal: Vec3,
    pub offset: f32,
}

impl Plane {
    #[inline]
    pub const fn new(normal: Vec3, offset: f32) -> Self {
        Self { normal, offset }
    }
}

/// The closest points between two segments or infinite lines.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SegmentDistanceResult {
    pub point1: Vec3,
    pub fraction1: f32,
    pub point2: Vec3,
    pub fraction2: f32,
}
