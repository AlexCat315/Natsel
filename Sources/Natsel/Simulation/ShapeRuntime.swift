

// Runtime shape data and geometry helpers (shape.c and the hull portion of hull.c).

// MARK: - Shape flags

/// Shape flags.
public struct ShapeFlags: OptionSet, Sendable, BitwiseCopyable {
    public var rawValue: UInt8

    public init(rawValue: UInt8) {
        self.rawValue = rawValue
    }

    public static let enableSensorEvents = ShapeFlags(rawValue: 0x01)
    public static let enableContactEvents = ShapeFlags(rawValue: 0x02)
    public static let enableCustomFiltering = ShapeFlags(rawValue: 0x04)
    public static let enableHitEvents = ShapeFlags(rawValue: 0x08)
    public static let enablePreSolveEvents = ShapeFlags(rawValue: 0x10)
    public static let enableSpeculative = ShapeFlags(rawValue: 0x40)
}

// MARK: - Geometry

/// The geometry carried by a shape. This replaces the C union.
public enum ShapeGeometry: @unchecked Sendable {
    case sphere(Sphere)
    case capsule(Capsule)
    case hull(Hull)
    /// Shared triangle geometry with instance scale.
    case mesh(Mesh)
    case heightField(HeightField)
    case compound(Compound)
}

/// A shape is a piece of collision geometry attached to a body.
public struct Shape: @unchecked Sendable {
    var id: Int32 = B3.nullIndex
    var bodyId: Int32 = B3.nullIndex
    var prevShapeId: Int32 = B3.nullIndex
    var nextShapeId: Int32 = B3.nullIndex
    var sensorIndex: Int32 = B3.nullIndex
    var proxyKey: Int32 = B3.nullIndex

    var type = ShapeType.sphereShape
    var density: Float = 0
    var explosionScale: Float = 1
    var aabbMargin: Float = 0

    var aabb = AABB()
    var localCentroid = Vec3()

    var materials: [SurfaceMaterial] = []
    var filter = Filter()
    var userData: UnsafeMutableRawPointer?
    var userShape: UnsafeMutableRawPointer?

    var nameId: UInt32 = 0
    var generation: UInt16 = 0

    /// ShapeFlags.
    var flags: UInt8 = 0

    var geometry = ShapeGeometry.sphere(Sphere())

    init() {}

    // MARK: Geometry accessors

    var sphere: Sphere {
        get {
            if case .sphere(let s) = geometry { return s }
            return Sphere()
        }
        set { geometry = .sphere(newValue) }
    }

    var capsule: Capsule {
        get {
            if case .capsule(let c) = geometry { return c }
            return Capsule()
        }
        set { geometry = .capsule(newValue) }
    }

    var hull: Hull? {
        get {
            if case .hull(let h) = geometry { return h }
            return nil
        }
        set {
            if let newValue {
                geometry = .hull(newValue)
            }
        }
    }

    /// The material at an index. A single material shape presents its only material.
    func material(_ index: Int32) -> SurfaceMaterial {
        let i = Int(index)
        if i >= 0 && i < materials.count {
            return materials[i]
        }
        return SurfaceMaterial()
    }

    var materialCount: Int32 {
        Int32(materials.count)
    }
}

// MARK: - Filtering

/// Do two shape filters allow collision?
@inline(__always)
public func shouldShapesCollide(_ filterA: Filter, _ filterB: Filter) -> Bool {
    if filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 {
        return filterA.groupIndex > 0
    }

    return (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0
}

/// Does a shape filter allow a query?
@inline(__always)
public func shouldQueryCollide(_ shapeFilter: Filter, _ queryFilter: QueryFilter) -> Bool {
    (shapeFilter.categoryBits & queryFilter.maskBits) != 0
        && (shapeFilter.maskBits & queryFilter.categoryBits) != 0
}

/// Is this shape type convex?
@inline(__always)
public func isConvex(_ type: ShapeType) -> Bool {
    type == .sphereShape || type == .capsuleShape || type == .hullShape
}

// MARK: - Hull geometry helpers

/// Compute the mass properties of a hull.
func computeHullMass(_ hull: Hull, _ density: Float) -> MassData {
    var out = MassData()
    out.mass = density * hull.volume
    out.center = hull.center

    // Inertia about the center of mass
    out.inertia = mulSM(density, hull.centralInertia)
    return out
}

/// Compute the AABB of a hull.
func computeHullAABB(_ hull: Hull, _ transform: Transform) -> AABB {
    aabbTransform(transform, hull.aabb)
}

/// Compute the swept AABB of a hull.
func computeSweptHullAABB(_ hull: Hull, _ xf1: Transform, _ xf2: Transform) -> AABB {
    let aabb1 = aabbTransform(xf1, hull.aabb)
    let aabb2 = aabbTransform(xf2, hull.aabb)
    return aabbUnion(aabb1, aabb2)
}

/// Do a hull and a point cloud proxy overlap?
func overlapHull(_ hull: Hull, _ shapeTransform: Transform, _ proxy: ShapeProxy) -> Bool {
    var input = DistanceInput()
    input.proxyA = ShapeProxy(points: hull.points, count: hull.vertexCount, radius: 0)
    input.proxyB = proxy
    input.transform = invMulTransforms(shapeTransform, Transform.identity)
    input.useRadii = true

    var cache = SimplexCache()
    let output = shapeDistance(input, &cache)
    return output.distance < B3.overlapSlop
}

/// Ray cast a hull.
func rayCastHull(_ hull: Hull, _ input: RayCastInput) -> CastOutput {
    b3Assert(isValidRay(input))
    var output = CastOutput()

    var lower: Float = 0
    var upper = input.maxFraction
    var bestFace = B3.nullIndex

    let planes = hull.planes!
    for faceIndex in 0..<Int(hull.faceCount) {
        let plane = planes[faceIndex]

        let distance = plane.offset - dot(plane.normal, input.origin)
        let denominator = dot(plane.normal, input.translation)

        if denominator == 0 {
            if distance < 0 {
                return output
            }
        } else {
            let fraction = distance / denominator

            if denominator < 0 {
                if fraction > lower {
                    bestFace = Int32(faceIndex)
                    lower = fraction
                }
            } else {
                if fraction < upper {
                    upper = fraction
                }
            }

            if upper < lower {
                return output
            }
        }
    }

    if bestFace >= 0 {
        output.point = input.origin + lower * input.translation
        output.normal = planes[Int(bestFace)].normal
        output.fraction = lower
        output.hit = true
    } else {
        output.point = input.origin
        output.hit = true
    }

    return output
}

/// Shape cast a hull.
func shapeCastHull(_ hull: Hull, _ input: ShapeCastInput) -> CastOutput {
    var pairInput = ShapeCastPairInput()
    pairInput.proxyA = ShapeProxy(points: hull.points, count: hull.vertexCount, radius: 0)
    pairInput.proxyB = input.proxy
    pairInput.transform = Transform.identity
    pairInput.translationB = input.translation
    pairInput.maxFraction = input.maxFraction
    pairInput.canEncroach = input.canEncroach

    return shapeCast(pairInput)
}

/// Compute the extent of a hull about a local origin.
func computeHullExtent(_ hull: Hull, _ origin: Vec3) -> ShapeExtent {
    let points = hull.points!

    var extent = ShapeExtent()
    extent.minExtent = hull.innerRadius
    extent.maxExtent = Vec3()
    for index in 0..<Int(hull.vertexCount) {
        let point = points[index]
        extent.maxExtent = maxVec3(extent.maxExtent, absVec3(point - origin))
    }

    return extent
}

/// Project the hull surface area onto a plane.
func computeHullProjectedArea(_ hull: Hull, _ direction: Vec3) -> Float {
    var area: Float = 0

    let faceCount = hull.faceCount
    let hullFaces = hull.faces!
    let hullEdges = hull.edges!
    let hullPoints = hull.points!

    for i in 0..<Int(faceCount) {
        let face = hullFaces + i

        let baseEdge = Int32(face.pointee.edge)
        var edge = hullEdges + Int(baseEdge)
        let p1 = hullPoints[Int(edge.pointee.origin)]

        var edgeIndex = Int32(edge.pointee.next)
        edge = hullEdges + Int(edgeIndex)
        var p2 = hullPoints[Int(edge.pointee.origin)]

        edgeIndex = Int32(edge.pointee.next)

        repeat {
            edge = hullEdges + Int(edgeIndex)
            let p3 = hullPoints[Int(edge.pointee.origin)]

            let e1 = p2 - p1
            let e2 = p3 - p1
            let n = cross(e1, e2)
            let a = dot(n, direction)
            area += maxFloat(a, 0)

            p2 = p3
            edgeIndex = Int32(edge.pointee.next)
        } while edgeIndex != baseEdge
    }

    return 0.5 * area
}

// MARK: - Shape queries

/// Compute the local-space AABB of a shape under a transform.
func computeShapeAABB(_ shape: Shape, _ transform: Transform) -> AABB {
    switch shape.geometry {
    case .capsule(let capsule):
        return computeCapsuleAABB(capsule, transform)

    case .compound(let compound):
        return computeCompoundAABB(compound, transform)

    case .heightField(let field):
        return computeHeightFieldAABB(field, transform)

    case .hull(let hull):
        return computeHullAABB(hull, transform)

    case .mesh(let mesh):
        guard let data = mesh.data else { return AABB(transform.p, transform.p) }
        return computeMeshAABB(data, transform, scale: mesh.scale)

    case .sphere(let sphere):
        return computeSphereAABB(sphere, transform)
    }
}

/// Compute a world AABB for a shape inflated by an extra margin.
func computeFatShapeAABB(_ shape: Shape, _ transform: WorldTransform, _ extra: Float) -> AABB {
    let r = Vec3(extra, extra, extra)
    var aabb = computeShapeAABB(shape, transform)
    aabb.lowerBound = aabb.lowerBound - r
    aabb.upperBound = aabb.upperBound + r
    return aabb
}

/// Compute the swept AABB of a shape.
func computeSweptShapeAABB(_ shape: Shape, _ sweep: Sweep, _ time: Float) -> AABB {
    b3Assert(0 <= time && time <= 1)
    let xf1 = Transform(sweep.c1 - rotateVector(sweep.q1, sweep.localCenter), sweep.q1)
    let xf2 = getSweepTransform(sweep, time)

    switch shape.geometry {
    case .capsule(let capsule):
        return computeSweptCapsuleAABB(capsule, xf1, xf2)

    case .hull(let hull):
        return computeSweptHullAABB(hull, xf1, xf2)

    case .sphere(let sphere):
        return computeSweptSphereAABB(sphere, xf1, xf2)

    default:
        b3Assert(false)
        return AABB(xf1.p, xf1.p)
    }
}

/// The local centroid of a shape.
func getShapeCentroid(_ shape: Shape) -> Vec3 {
    switch shape.geometry {
    case .capsule(let capsule):
        return lerp(capsule.center1, capsule.center2, 0.5)
    case .sphere(let sphere):
        return sphere.center
    case .compound(let compound):
        return aabbCenter(computeCompoundAABB(compound, .identity))
    case .heightField(let field):
        return aabbCenter(computeHeightFieldAABB(field, .identity))
    case .hull(let hull):
        return hull.center
    case .mesh(let mesh):
        guard let data = mesh.data else { return .zero }
        return aabbCenter(computeMeshAABB(data, .identity, scale: mesh.scale))
    }
}

/// The shape surface area.
func getShapeArea(_ shape: Shape) -> Float {
    switch shape.geometry {
    case .capsule(let capsule):
        return 2.0 * length(capsule.center1 - capsule.center2) + 2.0 * B3.pi * capsule.radius
    case .hull(let hull):
        return hull.surfaceArea
    case .sphere(let sphere):
        return 2.0 * B3.pi * sphere.radius
    default:
        return 0
    }
}

/// The shape surface area projected onto a plane.
func getShapeProjectedArea(_ shape: Shape, _ planeNormal: Vec3) -> Float {
    switch shape.geometry {
    case .capsule(let capsule):
        let radius = capsule.radius
        let axis = capsule.center2 - capsule.center1
        let projectedLength = length(cross(axis, planeNormal))
        let cylinderArea = 2.0 * radius * projectedLength
        let sphereArea = B3.pi * radius * radius
        return sphereArea + cylinderArea

    case .hull(let hull):
        return computeHullProjectedArea(hull, planeNormal)

    case .sphere(let sphere):
        return B3.pi * sphere.radius * sphere.radius

    default:
        return 0
    }
}

/// Compute the shape mass.
func computeShapeMass(_ shape: Shape) -> MassData {
    switch shape.geometry {
    case .capsule(let capsule):
        return computeCapsuleMass(capsule, shape.density)

    case .hull(let hull):
        return computeHullMass(hull, shape.density)

    case .sphere(let sphere):
        return computeSphereMass(sphere, shape.density)

    default:
        return MassData()
    }
}

/// Compute the min and max extent of a shape about a local center.
func computeShapeExtent(_ shape: Shape, _ localCenter: Vec3) -> ShapeExtent {
    var extent = ShapeExtent()

    switch shape.geometry {
    case .capsule(let capsule):
        let radius = capsule.radius
        extent.minExtent = radius
        let c1 = capsule.center1 - localCenter
        let c2 = capsule.center2 - localCenter
        let r = Vec3(radius, radius, radius)
        extent.maxExtent = maxVec3(absVec3(c1), absVec3(c2)) + r

    case .sphere(let sphere):
        let radius = sphere.radius
        extent.minExtent = radius
        let h = absVec3(sphere.center - localCenter)
        let r = Vec3(radius, radius, radius)
        extent.maxExtent = h + r

    case .hull(let hull):
        extent = computeHullExtent(hull, localCenter)

    case .compound(let compound):
        let aabb = computeCompoundAABB(compound, .identity)
        let r1 = length(aabb.lowerBound - localCenter)
        let r2 = length(aabb.upperBound - localCenter)
        extent.minExtent = minFloat(r1, r2)
        extent.maxExtent = absVec3(farthestPointOnAABB(aabb, localCenter) - localCenter)

    case .mesh(let mesh):
        if let data = mesh.data {
            let aabb = computeMeshAABB(data, .identity, scale: mesh.scale)
            let r1 = length(aabb.lowerBound - localCenter)
            let r2 = length(aabb.upperBound - localCenter)
            extent.minExtent = minFloat(r1, r2)
            extent.maxExtent = absVec3(farthestPointOnAABB(aabb, localCenter) - localCenter)
        }

    default:
        break
    }

    return extent
}

/// Compute the shape min extent.
func computeShapeMinExtent(_ shape: Shape, _ localCenter: Vec3) -> Float {
    switch shape.geometry {
    case .capsule(let capsule):
        return capsule.radius
    case .sphere(let sphere):
        return sphere.radius
    case .hull(let hull):
        return hull.innerRadius
    default:
        return computeShapeExtent(shape, localCenter).minExtent
    }
}

/// Ray cast a shape.
func rayCastShape(_ shape: Shape, _ transform: Transform, _ input: RayCastInput) -> CastOutput {
    var localInput = input
    localInput.origin = invTransformPoint(transform, input.origin)
    localInput.translation = invRotateVector(transform.q, input.translation)

    var output = CastOutput()
    switch shape.geometry {
    case .capsule(let capsule):
        output = rayCastCapsule(capsule, localInput)
    case .sphere(let sphere):
        output = rayCastSphere(sphere, localInput)
    case .hull(let hull):
        output = rayCastHull(hull, localInput)
    case .mesh(let mesh):
        output = rayCastMesh(mesh, localInput)
    case .compound(let compound):
        output = rayCastCompound(compound, localInput)
    case .heightField(let field):
        output = rayCastHeightField(field, localInput)
    }

    output.point = transformPoint(transform, output.point)
    output.normal = rotateVector(transform.q, output.normal)
    return output
}

/// Shape cast a shape.
func shapeCastShape(_ shape: Shape, _ transform: Transform, _ input: ShapeCastInput) -> CastOutput {
    var localInput = input
    var localPoints = [Vec3](repeating: Vec3(), count: Int(B3.maxShapeCastPoints))

    let count = minInt(input.proxy.count, B3.maxShapeCastPoints)
    for i in 0..<Int(count) {
        localPoints[i] = invTransformPoint(transform, input.proxy.points![i])
    }

    var output = CastOutput()
    localPoints.withUnsafeBufferPointer { points in
        localInput.proxy.points = points.baseAddress
        localInput.proxy.count = count
        localInput.translation = invRotateVector(transform.q, input.translation)

        switch shape.geometry {
        case .capsule(let capsule):
            output = shapeCastCapsule(capsule, localInput)
        case .hull(let hull):
            output = shapeCastHull(hull, localInput)
        case .sphere(let sphere):
            output = shapeCastSphere(sphere, localInput)
        case .mesh(let mesh):
            output = shapeCastMesh(mesh, localInput)
        case .compound(let compound):
            output = shapeCastCompound(compound, localInput)
        case .heightField(let field):
            output = shapeCastHeightField(field, localInput)
        }
    }

    output.point = transformPoint(transform, output.point)
    output.normal = rotateVector(transform.q, output.normal)
    return output
}

/// Does a shape overlap a point cloud proxy?
func overlapShape(_ shape: Shape, _ transform: Transform, _ proxy: ShapeProxy) -> Bool {
    switch shape.geometry {
    case .capsule(let capsule):
        return overlapCapsule(capsule, transform, proxy)
    case .hull(let hull):
        return overlapHull(hull, transform, proxy)
    case .sphere(let sphere):
        return overlapSphere(sphere, transform, proxy)
    case .mesh(let mesh):
        return overlapMesh(mesh, transform, proxy)
    case .compound(let compound):
        return overlapCompound(compound, transform, proxy)
    case .heightField(let field):
        return overlapHeightField(field, transform, proxy)
    }
}

// MARK: - Proxies

/// Build a shape proxy.
func makeShapeProxy(_ shape: Shape) -> ShapeProxyBuffer {
    switch shape.geometry {
    case .capsule(let capsule):
        return ShapeProxyBuffer(capsule.center1, capsule.center2, radius: capsule.radius)

    case .sphere(let sphere):
        return ShapeProxyBuffer(sphere.center, radius: sphere.radius)

    case .hull(let hull):
        return ShapeProxyBuffer(hull: hull)

    default:
        b3Assert(false)
        return ShapeProxyBuffer(Vec3(), radius: 0)
    }
}

// MARK: - Destruction

/// Destroy a shape's broad-phase proxy.
func destroyShapeProxy(_ shape: inout Shape, _ bp: BroadPhase) {
    if shape.proxyKey != B3.nullIndex {
        bp.destroyProxy(shape.proxyKey)
        shape.proxyKey = B3.nullIndex
    }
}

/// Destroy per-shape allocations. Hull data returns to the world database.
func destroyShapeAllocations(_ world: World, _ shape: inout Shape) {
    if case .hull(let hull) = shape.geometry {
        world.hullDatabase.remove(hull)
    }
    shape.geometry = .sphere(Sphere())

    shape.materials.removeAll()
}
