

/// This is used to filter collision on shapes. It affects shape-vs-shape collision and
/// shape-versus-query collision (such as ray casts).
public struct Filter: Sendable, Equatable, BitwiseCopyable {
    /// The collision category bits. Normally you would just set one bit.
    public var categoryBits: UInt64 = 0

    /// The collision mask bits. This states the categories that this shape would accept for collision.
    public var maskBits: UInt64 = 0

    /// Collision groups allow a certain group of objects to never collide (negative) or always
    /// collide (positive). A group index of zero has no effect. Non-zero group filtering always
    /// wins against the mask bits.
    public var groupIndex: Int32 = 0

    /// All fields zero, matching C zero initialization. Use `defaultFilter` for the engine defaults.
    public init() {}

    /// Collide with everything.
    public static let defaultFilter = Filter(categoryBits: defaultCategoryBits, maskBits: defaultMaskBits)

    public init(categoryBits: UInt64, maskBits: UInt64, groupIndex: Int32 = 0) {
        self.categoryBits = categoryBits
        self.maskBits = maskBits
        self.groupIndex = groupIndex
    }
}

/// Material properties supported per triangle on meshes and height fields.
public struct SurfaceMaterial: Sendable, Equatable, BitwiseCopyable {
    /// The Coulomb (dry) friction coefficient, usually in the range [0,1].
    public var friction: Float = 0

    /// The coefficient of restitution (bounce) usually in the range [0,1].
    public var restitution: Float = 0

    /// The rolling resistance usually in the range [0,1]. This is only used for spheres and capsules.
    public var rollingResistance: Float = 0

    /// The tangent velocity for conveyor belts. This is local to the shape and will be projected
    /// onto the contact surface.
    public var tangentVelocity = Vec3()

    /// User material identifier. This is passed with query results and to friction and restitution
    /// combining functions. It is not used internally.
    public var userMaterialId: UInt64 = 0

    /// Custom debug draw color. Ignored if 0. The low 24 bits are RGB. The high byte may carry a
    /// `DebugMaterial` preset, see `makeDebugColor`.
    public var customColor: UInt32 = 0

    /// Explicit padding. Must be zero.
    var padding: UInt32 = 0

    /// All fields zero, matching C zero initialization. Use `defaultMaterial` for the
    /// engine defaults.
    public init() {}

    /// The default surface material (friction 0.6).
    public static var defaultMaterial: SurfaceMaterial {
        var m = SurfaceMaterial()
        m.friction = 0.6
        return m
    }
}

/// Shape type.
public enum ShapeType: Int32, Sendable, CaseIterable, BitwiseCopyable {
    /// A capsule is an extruded sphere
    case capsuleShape

    /// A baked compound shape composed of spheres, capsules, hulls, and meshes
    case compoundShape

    /// A height field useful for terrain
    case heightShape

    /// A convex hull
    case hullShape

    /// A triangle soup
    case meshShape

    /// A sphere with an offset
    case sphereShape

    /// The number of shape types
    static let count: Int32 = 6
}

/// Used to create a shape.
public struct ShapeDef {
    /// Optional shape name for debugging
    public var name: String?

    /// Use this to store application specific shape data.
    public var userData: UnsafeMutableRawPointer?

    /// Surface material used on mesh shapes per triangle. Ignored for convex shapes. Ignored for
    /// compound shapes.
    public var materials: [SurfaceMaterial] = []

    /// The base surface material. Ignored for compound shapes.
    public var baseMaterial = SurfaceMaterial.defaultMaterial

    /// The density, usually in kg/m^3. The default is the density of water.
    public var density: Float = 1000 / (lengthUnitsPerMeter * lengthUnitsPerMeter * lengthUnitsPerMeter)

    /// Explosion scale for explosions. non-dimensional
    public var explosionScale: Float = 1

    /// Contact filtering data.
    public var filter = Filter.defaultFilter

    /// Enable custom filtering. Only one of the two shapes needs to enable custom filtering.
    public var enableCustomFiltering = false

    /// A sensor shape generates overlap events but never generates a collision response.
    /// Sensors still contribute to the body mass if they have non-zero density.
    public var isSensor = false

    /// Enable sensor events for this shape. This applies to sensors and non-sensors. False by
    /// default, even for sensors. Only convex shapes may act as sensor visitors.
    public var enableSensorEvents = false

    /// Enable contact events for this shape. Only applies to kinematic and dynamic bodies.
    public var enableContactEvents = false

    /// Enable hit events for this shape. Only applies to kinematic and dynamic bodies.
    public var enableHitEvents = false

    /// Enable pre-solve contact events for this shape. Only applies to dynamic bodies.
    public var enablePreSolveEvents = false

    /// When shapes are created they will scan the environment for collision the next time step.
    /// This flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
    public var invokeContactCreation = true

    /// Should the body update the mass properties when this shape is created.
    public var updateBodyMass = true

    /// Enable speculative collision. Experimental: this can only disable speculative contact
    /// between hulls and triangles (meshes and height fields).
    public var enableSpeculativeContact = true

    public init() {}
}

/// The query filter is used to filter collisions between queries and shapes.
public struct QueryFilter: Sendable, Equatable {
    /// The collision category bits of this query. Normally you would just set one bit.
    public var categoryBits: UInt64 = 0

    /// The collision mask bits. This states the shape categories that this query would accept
    /// for collision.
    public var maskBits: UInt64 = 0

    /// Optional id combined with `name` to identify this query in a recording.
    public var id: UInt64 = 0

    /// Optional label combined with `id` to identify this query in a recording.
    public var name: String?

    /// All fields zero, matching C zero initialization. Use `defaultFilter` for the engine defaults.
    public init() {}

    /// Query everything.
    public static let defaultFilter = QueryFilter(categoryBits: defaultCategoryBits, maskBits: defaultMaskBits)

    public init(categoryBits: UInt64, maskBits: UInt64) {
        self.categoryBits = categoryBits
        self.maskBits = maskBits
    }
}
