/// A 2D vector.
public struct Vec2: Sendable, Equatable, BitwiseCopyable {
    public var x: Float = 0
    public var y: Float = 0

    @inlinable public init() {}

    @inlinable public init(_ x: Float, _ y: Float) {
        self.x = x
        self.y = y
    }
}

/// A 3D vector.
public struct Vec3: Sendable, Equatable, BitwiseCopyable {
    public var x: Float = 0
    public var y: Float = 0
    public var z: Float = 0

    @inlinable public init() {}

    @inlinable public init(_ x: Float, _ y: Float, _ z: Float) {
        self.x = x
        self.y = y
        self.z = z
    }

    public static let zero = Vec3(0, 0, 0)
    public static let one = Vec3(1, 1, 1)
    public static let axisX = Vec3(1, 0, 0)
    public static let axisY = Vec3(0, 1, 0)
    public static let axisZ = Vec3(0, 0, 1)
}

/// Cosine and sine pair.
/// This uses a custom implementation designed for cross-platform determinism.
public struct CosSin: Sendable, Equatable, BitwiseCopyable {
    public var cosine: Float = 0
    public var sine: Float = 0

    @inlinable public init() {}

    @inlinable public init(cosine: Float, sine: Float) {
        self.cosine = cosine
        self.sine = sine
    }
}

/// A quaternion. The default value is all zero, matching C zero initialization; use `identity`
/// for the unit rotation.
public struct Quat: Sendable, Equatable, BitwiseCopyable {
    public var v: Vec3 = Vec3()
    public var s: Float = 0

    @inlinable public init() {}

    @inlinable public init(_ v: Vec3, _ s: Float) {
        self.v = v
        self.s = s
    }

    public static let identity = Quat(Vec3(0, 0, 0), 1)
}

/// A rigid transform.
public struct Transform: Sendable, Equatable, BitwiseCopyable {
    public var p: Vec3 = Vec3()
    public var q: Quat = Quat()

    @inlinable public init() {}

    @inlinable public init(_ p: Vec3, _ q: Quat) {
        self.p = p
        self.q = q
    }

    public static let identity = Transform(Vec3(0, 0, 0), Quat(Vec3(0, 0, 0), 1))
}

/// A double precision world position. Subtract two positions before narrowing to a local vector.
public struct Pos: Sendable, Equatable, BitwiseCopyable {
    public var x: Double = 0
    public var y: Double = 0
    public var z: Double = 0
    @inlinable public init() {}
    @inlinable public init<X: BinaryFloatingPoint, Y: BinaryFloatingPoint, Z: BinaryFloatingPoint>(_ x: X, _ y: Y, _ z: Z) { self.x = Double(x); self.y = Double(y); self.z = Double(z) }
    @inlinable public init(_ v: Vec3) { x = Double(v.x); y = Double(v.y); z = Double(v.z) }
    public static let zero = Pos()
}

/// Double precision translation with a single precision rotation.
public struct WorldTransform: Sendable, Equatable, BitwiseCopyable {
    public var p = Pos()
    public var q = Quat()
    @inlinable public init() {}
    @inlinable public init(_ p: Pos, _ q: Quat) { self.p = p; self.q = q }
    @inlinable public init(_ p: Vec3, _ q: Quat) { self.p = Pos(p); self.q = q }
    public static let identity = WorldTransform(Pos(), .identity)
}

/// A 3x3 matrix.
public struct Matrix3: Sendable, Equatable, BitwiseCopyable {
    public var cx: Vec3 = Vec3()
    public var cy: Vec3 = Vec3()
    public var cz: Vec3 = Vec3()

    @inlinable public init() {}

    @inlinable public init(_ cx: Vec3, _ cy: Vec3, _ cz: Vec3) {
        self.cx = cx
        self.cy = cy
        self.cz = cz
    }

    public static let zero = Matrix3(Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0))
    public static let identity = Matrix3(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1))
}

/// Axis aligned bounding box.
public struct AABB: Sendable, Equatable, BitwiseCopyable {
    public var lowerBound: Vec3 = Vec3()
    public var upperBound: Vec3 = Vec3()

    @inlinable public init() {}

    @inlinable public init(_ lowerBound: Vec3, _ upperBound: Vec3) {
        self.lowerBound = lowerBound
        self.upperBound = upperBound
    }
}

/// A plane.
/// separation = dot(normal, point) - offset
public struct Plane: Sendable, Equatable, BitwiseCopyable {
    public var normal: Vec3 = Vec3()
    public var offset: Float = 0

    @inlinable public init() {}

    @inlinable public init(_ normal: Vec3, _ offset: Float) {
        self.normal = normal
        self.offset = offset
    }
}

/// The closest points between to segments or infinite lines.
public struct SegmentDistanceResult: Sendable, Equatable, BitwiseCopyable {
    public var point1: Vec3 = Vec3()
    public var fraction1: Float = 0
    public var point2: Vec3 = Vec3()
    public var fraction2: Float = 0

    @inlinable public init() {}
}
