/// Owns the point storage for a shape proxy so the proxy's pointer stays valid for the duration
/// of a query. Hull proxies borrow the hull's points and need no copy.
struct ShapeProxyBuffer: @unchecked Sendable {
    // Spheres and capsules are the only shapes that use points here, so two inline points
    // replace a heap array on every time-of-impact call.
    private var points = InlineArray<2, Vec3>(repeating: Vec3())
    private var pointCount: Int32 = 0
    private var hull: Hull?
    private let radius: Float

    init(_ a: Vec3, _ b: Vec3, radius: Float) {
        points[0] = a
        points[1] = b
        pointCount = 2
        self.hull = nil
        self.radius = radius
    }

    init(_ a: Vec3, radius: Float) {
        points[0] = a
        pointCount = 1
        self.hull = nil
        self.radius = radius
    }

    init(hull: Hull) {
        self.hull = hull
        self.radius = 0
    }

    var count: Int32 {
        if let hull {
            return hull.vertexCount
        }
        return pointCount
    }

    func withProxy<R>(_ body: (ShapeProxy) -> R) -> R {
        if let hull {
            return body(ShapeProxy(points: hull.points, count: hull.vertexCount, radius: 0))
        }
        return withInlineReadBuffer(points) {
            body(ShapeProxy(points: $0.baseAddress, count: pointCount, radius: radius))
        }
    }
}

/// Transform a proxy into a local frame using a caller-provided buffer.
func makeLocalProxy(_ proxy: ShapeProxy, _ transform: Transform, _ buffer: UnsafeMutablePointer<Vec3>) -> ShapeProxy {
    let invTransform = invertTransform(transform)
    let R = makeMatrixFromQuat(invTransform.q)

    let count = minInt(proxy.count, B3.maxShapeCastPoints)
    for i in 0..<Int(count) {
        buffer[i] = mulMV(R, proxy.points![i]) + invTransform.p
    }

    return ShapeProxy(points: buffer, count: count, radius: proxy.radius)
}

/// Compute the AABB of a proxy.
func computeProxyAABB(_ proxy: ShapeProxy) -> AABB {
    let points = proxy.points!
    var aabb = AABB(points[0], points[0])

    for i in 1..<Int(proxy.count) {
        aabb.lowerBound = minVec3(aabb.lowerBound, points[i])
        aabb.upperBound = maxVec3(aabb.upperBound, points[i])
    }

    let r = Vec3(proxy.radius, proxy.radius, proxy.radius)
    aabb.lowerBound = aabb.lowerBound - r
    aabb.upperBound = aabb.upperBound + r
    return aabb
}

