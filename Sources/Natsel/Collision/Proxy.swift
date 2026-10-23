/// Owns the point storage for a shape proxy so the proxy's pointer stays valid for the duration
/// of a query. Hull proxies borrow the hull's points and need no copy.
struct ShapeProxyBuffer: @unchecked Sendable {
    private var points: [Vec3]
    private var hull: Hull?
    private let radius: Float

    init(points: [Vec3], radius: Float) {
        self.points = points
        self.hull = nil
        self.radius = radius
    }

    init(hull: Hull) {
        self.points = []
        self.hull = hull
        self.radius = 0
    }

    var count: Int32 {
        if let hull {
            return hull.vertexCount
        }
        return Int32(points.count)
    }

    func withProxy<R>(_ body: (ShapeProxy) -> R) -> R {
        if let hull {
            return body(ShapeProxy(points: hull.points, count: hull.vertexCount, radius: 0))
        }
        return points.withUnsafeBufferPointer {
            body(ShapeProxy(points: $0.baseAddress, count: Int32($0.count), radius: radius))
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

