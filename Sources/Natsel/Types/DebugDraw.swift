

// MARK: - Debug Shape

/// The geometry carried by a debug shape.
public enum DebugShapeGeometry: @unchecked Sendable {
    case capsule(UnsafePointer<Capsule>)
    case compound(Compound)
    case heightField(HeightField)
    case hull(UnsafePointer<HullData>)
    case mesh(UnsafePointer<Mesh>)
    case sphere(UnsafePointer<Sphere>)
}

/// This is sent to the user for debug shape creation. The user should know the type in case they
/// have custom sphere or capsule rendering.
public struct DebugShape: @unchecked Sendable {
    /// Shape id.
    public var shapeId: ShapeID

    /// Shape type.
    public var type: ShapeType

    /// The geometry data.
    public var geometry: DebugShapeGeometry

    public init(shapeId: ShapeID, type: ShapeType, geometry: DebugShapeGeometry) {
        self.shapeId = shapeId
        self.type = type
        self.geometry = geometry
    }
}

// MARK: - Color

/// A packed debug draw color. The low 24 bits are RGB; the high byte optionally carries a
/// `DebugMaterial` preset that a renderer may use to pick roughness and metalness.
public typealias HexColor = UInt32

/// PBR presets packed into the high byte of a `HexColor`.
public enum DebugMaterial: UInt32, Sendable, CaseIterable {
    case `default` = 0
    case matte = 1
    case soft = 2
    case dead = 3
    case glossy = 4
    case metallic = 5
}

/// Pack an RGB color with a material preset. The preset rides in the high byte where the color
/// converters ignore it.
@inlinable public func makeDebugColor(_ rgb: HexColor, _ material: DebugMaterial) -> HexColor {
    (rgb & 0x00FF_FFFF) | (material.rawValue << 24)
}

/// The CSS color names the debug draw path uses.
public enum DebugColors {
    public static let darkBlue: HexColor = 0x00008B
    public static let blue: HexColor = 0x0000FF
    public static let limeGreen: HexColor = 0x32CD32
    public static let lightBlue: HexColor = 0xADD8E6
    public static let magenta: HexColor = 0xFF00FF
    public static let yellow: HexColor = 0xFFFF00
    public static let red: HexColor = 0xFF0000
    public static let orange: HexColor = 0xFFA500
    public static let green: HexColor = 0x008000
    public static let cyan: HexColor = 0x00FFFF
    public static let violet: HexColor = 0xEE82EE
    public static let pink: HexColor = 0xFFC0CB
    public static let chocolate: HexColor = 0xD2691E
    public static let goldenRod: HexColor = 0xDAA520
    public static let coral: HexColor = 0xFF7F50
    public static let rosyBrown: HexColor = 0xBC8F8F
    public static let aqua: HexColor = 0x00FFFF
    public static let peru: HexColor = 0xCD853F
    public static let lime: HexColor = 0x00FF00
    public static let gold: HexColor = 0xFFD700
    public static let plum: HexColor = 0xDDA0DD
    public static let snow: HexColor = 0xFFFAFA
    public static let teal: HexColor = 0x008080
    public static let khaki: HexColor = 0xF0E68C
    public static let salmon: HexColor = 0xFA8072
    public static let peachPuff: HexColor = 0xFFDAB9
    public static let honeyDew: HexColor = 0xF0FFF0
    public static let black: HexColor = 0x000000
    public static let lightGray: HexColor = 0xD3D3D3
    public static let white: HexColor = 0xFFFFFF
    public static let skyBlue: HexColor = 0x87CEEB
    public static let slateGray: HexColor = 0x708090
    public static let wheat: HexColor = 0xF5DEB3
    public static let turquoise: HexColor = 0x40E0D0
    public static let darkGray: HexColor = 0xA9A9A9
    public static let steelBlue: HexColor = 0x4682B4
    public static let lightSteelBlue: HexColor = 0xB0C4DE
    public static let tan: HexColor = 0xD2B48C
    public static let lightSlateGray: HexColor = 0x778899
    public static let orangeRed: HexColor = 0xFF4500
    public static let lightGreen: HexColor = 0x90EE90
    public static let gray: HexColor = 0x808080
    public static let dimGray: HexColor = 0x696969
    public static let darkOrange: HexColor = 0xFF8C00
    public static let darkCyan: HexColor = 0x008B8B

    /// The per-graph-color palette. The last slot is the overflow color.
    public static let graph: [HexColor] = [
        red, orange, yellow, green, cyan, blue,
        violet, pink, chocolate, goldenRod, coral, rosyBrown,
        aqua, peru, lime, gold, plum, snow,
        teal, khaki, salmon, peachPuff, honeyDew, black,
    ]
}

// MARK: - Debug draw callbacks

/// The callbacks and options a renderer supplies to `World.draw`. All coordinates are world
/// space; shift them into your own camera frame inside the closures.
public struct DebugDraw {
    /// Draws a user shape. `userShape` is the opaque pointer your factory returned.
    public var drawShape: ((UnsafeMutableRawPointer, WorldTransform, HexColor) -> Void)?
    /// Draw a line segment.
    public var drawSegment: ((Pos, Pos, HexColor) -> Void)?
    /// Draw a transform. Choose your own length scale.
    public var drawTransform: ((WorldTransform) -> Void)?
    /// Draw a point.
    public var drawPoint: ((Pos, Float, HexColor) -> Void)?
    /// Draw a sphere.
    public var drawSphere: ((Pos, Float, HexColor, Float) -> Void)?
    /// Draw a capsule.
    public var drawCapsule: ((Pos, Pos, Float, HexColor, Float) -> Void)?
    /// Draw an axis aligned box.
    public var drawBounds: ((AABB, HexColor) -> Void)?
    /// Draw an oriented box.
    public var drawBox: ((Vec3, WorldTransform, HexColor) -> Void)?
    /// Draw a string in world space.
    public var drawString: ((Pos, String, HexColor) -> Void)?

    /// World bounds used for culling.
    public var drawingBounds = AABB(Vec3(-1e6, -1e6, -1e6), Vec3(1e6, 1e6, 1e6))
    /// Scale used when drawing forces.
    public var forceScale: Float = 1
    /// Global scaling for joint drawing.
    public var jointScale: Float = 1

    public var drawShapes = true
    public var drawJoints = true
    public var drawJointExtras = false
    public var drawBoundsOption = false
    public var drawMass = false
    public var drawSleep = false
    public var drawBodyNames = false
    public var drawContacts = false
    public var drawAnchorA = false
    public var drawGraphColors = false
    public var drawContactFeatures = false
    public var drawContactNormals = false
    public var drawContactForces = false
    public var drawIslands = false

    /// Factory invoked the first time a shape needs a renderable representation. The returned
    /// pointer is cached on the shape and passed to `drawShape`.
    public var createDebugShape: ((DebugShape) -> UnsafeMutableRawPointer?)?

    public init() {}
}
