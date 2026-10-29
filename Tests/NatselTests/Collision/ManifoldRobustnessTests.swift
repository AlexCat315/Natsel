import Testing
internal import Foundation
@testable import Natsel

@Suite("Manifold robustness")
struct ManifoldRobustnessTests {
    private struct Random {
        var seed: UInt32 = 12345

        mutating func nextFloat(_ lower: Float, _ upper: Float) -> Float {
            seed = 1664525 &* seed &+ 1013904223
            let t = Float(seed >> 8) / Float(1 << 24)
            return lower + t * (upper - lower)
        }

        mutating func nextDirection() -> Vec3 {
            let v = Vec3(nextFloat(-1, 1), nextFloat(-1, 1), nextFloat(-1, 1))
            return normalize(v)
        }
    }

    private func exactQuat(_ axis: Vec3, _ radians: Float) -> Quat {
        let half = 0.5 * radians
        let s = Foundation.sinf(half)
        return Quat(s * axis, Foundation.cosf(half))
    }

    private func collide(
        _ hullA: Hull, _ hullB: Hull, _ transform: Transform, _ capacity: Int32 = 8
    ) -> (points: [LocalManifoldPoint], normal: Vec3) {
        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: Int(capacity))
        var manifold = LocalManifold()
        var cache = SATCache()
        var normal = Vec3()
        buffer.withUnsafeMutableBufferPointer { points in
            manifold.points = points.baseAddress
            collideHulls(&manifold, capacity, hullA, hullB, transform, &cache)
            normal = manifold.normal
        }
        let count = Int(manifold.pointCount)
        return (Array(buffer[0..<count]), normal)
    }

    /// Overlapping hulls admit no separating axis, so a positive edge separation is always noise.
    /// It surfaces as an empty manifold, which the solver reads as no contact at all.
    @Test("overlapping hulls always produce at least one penetrating point")
    func overlapNeverEmpty() {
        let hullA = makeBoxHull(0.5, 0.5, 0.5)
        let hullB = makeBoxHull(0.4, 0.6, 0.5)

        var random = Random()

        for i in 0..<2000 {
            let axis = random.nextDirection()

            // Half the samples are nearly aligned, where the edge cross products are smallest.
            let angle = (i & 1) != 0
                ? random.nextFloat(-0.01, 0.01)
                : random.nextFloat(-B3.pi, B3.pi)

            // Shorter than the smallest half width, so the center of B stays inside A.
            let offset = 0.4 * random.nextDirection()

            let transform = Transform(offset, exactQuat(axis, angle))
            let (points, normal) = collide(hullA, hullB, transform)

            #expect(points.isEmpty == false, "sample \(i) produced no points")
            #expect(absFloat(length(normal) - 1) < 1e-5, "sample \(i) normal not unit")

            // Clipping keeps points that are separated, but the deepest one must penetrate.
            let minSeparation = points.map(\.separation).min() ?? .greatestFiniteMagnitude
            #expect(minSeparation < 0, "sample \(i) min separation \(minSeparation)")
        }
    }

    @Test("manifold point storage matches the published limit")
    func maxPointCount() {
        // The portable build fixes four points per manifold; the wide build can hold more.
        #expect(B3.maxManifoldPoints >= 4)
        #expect(MemoryLayout<LocalManifoldPoint>.size > 0)

        // Collision never writes past the capacity it was given.
        let hullA = makeBoxHull(0.5, 0.5, 0.5)
        let hullB = makeBoxHull(0.5, 0.5, 0.5)
        let transform = Transform(Vec3(0, 0.99, 0), Quat.identity)

        for capacity in [1, 2, 3, 4] {
            let (points, _) = collide(hullA, hullB, transform, Int32(capacity))
            #expect(points.count <= capacity)
        }
    }

    @Test("deep overlap keeps points penetrating across many orientations")
    func deepOverlapSamples() {
        let hullA = makeBoxHull(1, 1, 1)
        let hullB = makeBoxHull(0.3, 0.3, 0.3)

        var random = Random()

        for _ in 0..<500 {
            let axis = random.nextDirection()
            let angle = random.nextFloat(-B3.pi, B3.pi)
            // Well inside A on every axis.
            let offset = 0.2 * random.nextDirection()

            let transform = Transform(offset, exactQuat(axis, angle))
            let (points, _) = collide(hullA, hullB, transform)

            #expect(points.isEmpty == false)
            let minSeparation = points.map(\.separation).min() ?? .greatestFiniteMagnitude
            #expect(minSeparation < 0)
        }
    }
}

// MARK: - Crossed edge geometry

extension ManifoldRobustnessTests {
    /// Cube A yawed 45 degrees presents an edge along y at x = +h*root2. Cube B rolled 45 degrees
    /// presents an edge along z at x = -h*root2. Sliding B along x makes those two edges the
    /// closest features, so the axis of minimum penetration is x.
    private func crossedEdgeHulls(_ halfWidth: Float) -> (Hull, Hull) {
        let a = makeTransformedBoxHull(
            halfWidth, halfWidth, halfWidth,
            Transform(Vec3(), exactQuat(.axisY, 0.25 * B3.pi))
        )
        let b = makeTransformedBoxHull(
            halfWidth, halfWidth, halfWidth,
            Transform(Vec3(), exactQuat(.axisZ, 0.25 * B3.pi))
        )
        return (a, b)
    }

    @Test("crossed edges pick the edge pair axis and the analytic point")
    func crossedEdge() {
        let (hullA, hullB) = crossedEdgeHulls(0.5)
        let root2 = (2 as Float).squareRoot()

        for d in [1.42 as Float, root2, 1.41, 1.3] {
            let expected = d - root2
            let transform = Transform(Vec3(d, 0, 0), Quat.identity)

            var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
            var manifold = LocalManifold()
            var cache = SATCache()
            buffer.withUnsafeMutableBufferPointer { points in
                manifold.points = points.baseAddress
                collideHulls(&manifold, 8, hullA, hullB, transform, &cache)
            }

            #expect(manifold.pointCount == 1)
            #expect(cache.type == .edgePairAxis)
            #expect(absFloat(manifold.normal.x - 1) < 1e-6)
            #expect(absFloat(manifold.normal.y) < 1e-6)
            #expect(absFloat(manifold.normal.z) < 1e-6)

            let point = buffer[0]
            #expect(absFloat(point.separation - expected) < 1e-5)
            #expect(absFloat(point.point.x - 0.5 * d) < 1e-5)
            #expect(absFloat(point.point.y) < 1e-5)
            #expect(absFloat(point.point.z) < 1e-5)

            // The forced edge query must agree with what the full solver chose.
            var manual = LocalManifold()
            var manualCache = SATCache()
            manualCache.type = .manualEdgePairAxis
            buffer.withUnsafeMutableBufferPointer { points in
                manual.points = points.baseAddress
                collideHulls(&manual, 8, hullA, hullB, transform, &manualCache)
            }
            #expect(manual.pointCount == 1)
            #expect(absFloat(buffer[0].separation - expected) < 1e-5)
        }
    }
}

// MARK: - Ridge crossing policy

extension ManifoldRobustnessTests {
    /// A roof face of one hull, so the normal is 45 degrees off the vertical.
    private func roofContact(
        _ manifold: LocalManifold, _ points: [LocalManifoldPoint], _ cache: SATCache,
        _ overlap: Float, _ halfRoot2: Float, _ root2: Float
    ) -> String? {
        if manifold.pointCount < 4 || manifold.pointCount > B3.maxManifoldPoints {
            return "point count \(manifold.pointCount)"
        }
        if cache.type != .faceAxisA && cache.type != .faceAxisB {
            return "cache type \(cache.type)"
        }
        if absFloat(manifold.normal.y - halfRoot2) > 1e-4 {
            return "normal.y \(manifold.normal.y)"
        }

        let minSeparation = points.map(\.separation).min() ?? 0
        if minSeparation >= -halfRoot2 * overlap + 1e-4 {
            return "separation too shallow \(minSeparation)"
        }
        if minSeparation <= -root2 * overlap - 1e-4 {
            return "separation too deep \(minSeparation)"
        }
        return nil
    }

    /// Two long roof ridges laid across each other. The axis of minimum penetration is the edge
    /// pair, but a one point edge contact is weak for stacking. The collider builds the roof face
    /// contact first and only switches to the edge contact when the edge axis beats the clipped
    /// face separation by more than the slop.
    @Test("ridge crossing keeps face contact shallow and switches to edges when deep")
    func ridgeCrossing() {
        let hullA = makeTransformedBoxHull(
            1.5, 0.1, 0.1, Transform(Vec3(), exactQuat(.axisX, 0.25 * B3.pi)))
        let hullB = makeTransformedBoxHull(
            1.5, 0.1, 0.1, Transform(Vec3(), exactQuat(.axisX, 0.25 * B3.pi)))

        let root2 = (2 as Float).squareRoot()
        let halfRoot2: Float = 0.70710678
        let ridgeY: Float = 0.1 * root2

        func run(overlap: Float, angle: Float) -> (LocalManifold, [LocalManifoldPoint], SATCache) {
            let lift = 2 * ridgeY - overlap
            let transform = Transform(Vec3(0, lift, 0), exactQuat(.axisY, angle))

            var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
            var manifold = LocalManifold()
            var cache = SATCache()
            buffer.withUnsafeMutableBufferPointer { points in
                manifold.points = points.baseAddress
                collideHulls(&manifold, 8, hullA, hullB, transform, &cache)
            }
            return (manifold, Array(buffer[0..<Int(manifold.pointCount)]), cache)
        }

        // Shallow overlap: the edge axis is better by only (root2 - 1) * overlap, inside the slop,
        // so the four point face contact carries the crossing at every angle.
        let overlap: Float = 0.01
        for angle in [0 as Float, 1e-3, 0.02, 0.1, 0.5] {
            let (manifold, points, cache) = run(overlap: overlap, angle: angle)
            if let error = roofContact(
                manifold, points, cache, overlap, halfRoot2, root2
            ) {
                Issue.record("shallow angle \(angle): \(error)")
            }
        }

        // Deep overlap at a clear crossing. The edge axis now beats the clipped face separation by
        // more than the slop, so the edge contact replaces the face contact.
        let deep: Float = 0.05
        for angle in [0.05 as Float, 0.1, 0.2, 0.5] {
            let (manifold, points, cache) = run(overlap: deep, angle: angle)
            #expect(manifold.pointCount == 1, "deep angle \(angle)")
            #expect(cache.type == .edgePairAxis, "deep angle \(angle)")
            #expect(absFloat(manifold.normal.x) < 1e-4)
            #expect(absFloat(manifold.normal.y - 1) < 1e-4)
            #expect(absFloat(manifold.normal.z) < 1e-4)
            #expect(absFloat(points[0].separation + deep) < 1e-4)
            #expect(absFloat(points[0].point.y - (ridgeY - 0.5 * deep)) < 1e-4)
            // Only has to land near the crossing, not at the end of a three meter beam.
            #expect(absFloat(points[0].point.x) < 0.01)
            #expect(absFloat(points[0].point.z) < 0.01)
        }

        // Deep overlap near parallel: a one point edge contact off a parallel pair would have a
        // normal built from noise, so the roof faces keep the contact.
        for angle in [0 as Float, 1e-4, 1e-3, 0.003] {
            let (manifold, points, cache) = run(overlap: deep, angle: angle)
            if let error = roofContact(manifold, points, cache, deep, halfRoot2, root2) {
                Issue.record("parallel angle \(angle): \(error)")
            }
        }
    }
}

// MARK: - Edge axis oracle

extension ManifoldRobustnessTests {
    /// Rebuild the half-edge segment a cached edge index refers to.
    private func hullEdgeSegment(
        _ hull: Hull, _ edgeIndex: UInt8, _ transform: Transform
    ) -> (point: Vec3, edge: Vec3) {
        guard let edges = hull.edges, let points = hull.points else {
            return (Vec3(), Vec3())
        }
        let e = edges[Int(edgeIndex)]
        let twin = edges[Int(e.twin)]
        let tail = transformPoint(transform, points[Int(e.origin)])
        let head = transformPoint(transform, points[Int(twin.origin)])
        return (tail, head - tail)
    }

    /// Verify the recovered axis, separation and point against a cross-product oracle built from
    /// the edges the solver actually latched onto.
    private func checkEdgeContact(
        _ manifold: LocalManifold, _ points: [LocalManifoldPoint],
        _ p1: Vec3, _ e1: Vec3, _ p2: Vec3, _ e2: Vec3, _ orientRef: Vec3,
        radius: Float, normalTol: Float, sepTol: Float, pointTol: Float
    ) -> String? {
        var axis = normalize(cross(e1, e2))
        if dot(axis, orientRef) < 0 {
            axis = -axis
        }

        let n = manifold.normal
        if absFloat(n.x - axis.x) > normalTol || absFloat(n.y - axis.y) > normalTol
            || absFloat(n.z - axis.z) > normalTol
        {
            return "normal \(n) vs axis \(axis)"
        }
        if absFloat(dot(n, normalize(e1))) > normalTol { return "normal not perpendicular to e1" }
        if absFloat(dot(n, normalize(e2))) > normalTol { return "normal not perpendicular to e2" }

        let closest = lineDistance(p1, e1, p2, e2)

        // Signed gap between the edge lines along the axis, less the capsule radius.
        let expectedSeparation = dot(axis, closest.point2 - closest.point1) - radius
        if absFloat(points[0].separation - expectedSeparation) > sepTol {
            return "separation \(points[0].separation) vs \(expectedSeparation)"
        }

        // Midpoint of the closest approach, pulling the first point in by the radius.
        let expectedPoint = 0.5 * (mulSub(closest.point1, radius, axis) + closest.point2)
        if absFloat(points[0].point.x - expectedPoint.x) > pointTol
            || absFloat(points[0].point.y - expectedPoint.y) > pointTol
            || absFloat(points[0].point.z - expectedPoint.z) > pointTol
        {
            return "point \(points[0].point) vs \(expectedPoint)"
        }
        return nil
    }

    @Test("edge pair axis matches the cross product oracle")
    func edgeAxisOracle() {
        let hullA = makeTransformedBoxHull(
            0.5, 0.5, 0.5, Transform(Vec3(), exactQuat(.axisY, 0.25 * B3.pi)))

        let root2 = (2 as Float).squareRoot()
        var edgeContacts = 0
        var failures: [String] = []

        for roll in [0.18 * B3.pi, 0.25 * B3.pi, 0.32 * B3.pi] {
            let hullB = makeTransformedBoxHull(
                0.5, 0.5, 0.5, Transform(Vec3(), exactQuat(.axisZ, roll)))

            for yaw in [-0.35 as Float, -0.15, 0.0, 0.15, 0.35] {
                for distance in [1.38 as Float, 1.40, root2, 1.44] {
                    let transform = Transform(Vec3(distance, 0, 0), exactQuat(.axisY, yaw))

                    var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
                    var manifold = LocalManifold()
                    var cache = SATCache()
                    buffer.withUnsafeMutableBufferPointer { pts in
                        manifold.points = pts.baseAddress
                        collideHulls(&manifold, 8, hullA, hullB, transform, &cache)
                    }

                    if cache.type != .edgePairAxis || manifold.pointCount != 1 {
                        continue
                    }

                    let (p1, e1) = hullEdgeSegment(hullA, cache.indexA, Transform(Vec3(), Quat.identity))
                    let (p2, e2) = hullEdgeSegment(hullB, cache.indexB, transform)

                    let centerA = hullA.center
                    let centerB = transformPoint(transform, hullB.center)
                    let orientRef = centerB - centerA

                    if let error = checkEdgeContact(
                        manifold, Array(buffer[0..<Int(manifold.pointCount)]),
                        p1, e1, p2, e2, orientRef,
                        radius: 0, normalTol: 2e-4, sepTol: 2e-4, pointTol: 2e-3
                    ) {
                        failures.append("roll \(roll) yaw \(yaw) d \(distance): \(error)")
                    }

                    edgeContacts += 1
                }
            }
        }

        #expect(failures.isEmpty, "\(failures.prefix(3))")
        // The sweep is only meaningful if it actually drove the edge path.
        #expect(edgeContacts >= 15)
    }
}

// MARK: - Deep edge overlap

extension ManifoldRobustnessTests {
    /// Penetrate the hull far enough that the capsule core overlaps the box so the deep path runs,
    /// while keeping the radius small enough that the edge stays the axis of minimum penetration.
    @Test("hull and capsule keep a single negative point under deep edge overlap")
    func hullCapsuleEdgeDeep() {
        let hull = makeBoxHull(0.5, 0.5, 0.5)

        // The +x +y edge runs along z between the +x and +y faces.
        let edgePoint = Vec3(0.5, 0.5, 0)
        let edgeDir = Vec3(0, 0, 1)
        let outward = normalize(Vec3(1, 1, 0))

        var count = 0
        var failures: [String] = []

        for depth in [0.12 as Float, 0.18, 0.25] {
            for radius in [0.05 as Float, 0.1, 0.2] {
                for tilt in [0 as Float, 0.25, -0.25] {
                    let capsuleDir = normalize(Vec3(1, -1, tilt))
                    let mid = mulAdd(edgePoint, -depth, outward)
                    let c1 = mulAdd(mid, -0.5, capsuleDir)
                    let c2 = mulAdd(mid, 0.5, capsuleDir)
                    let capsule = Capsule(center1: c1, center2: c2, radius: radius)

                    var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
                    var manifold = LocalManifold()
                    var cache = SimplexCache()
                    buffer.withUnsafeMutableBufferPointer { pts in
                        manifold.points = pts.baseAddress
                        collideHullAndCapsule(
                            &manifold, 8, hull, capsule, Transform(Vec3(), Quat.identity), &cache
                        )
                    }

                    let label = "d \(depth) r \(radius) tilt \(tilt)"
                    if manifold.pointCount != 1 {
                        failures.append("\(label): pointCount \(manifold.pointCount)")
                        continue
                    }
                    let sep = buffer[0].separation
                    if sep >= 0 {
                        failures.append("\(label): separation \(sep)")
                        continue
                    }

                    // Hull edge is e1, capsule axis is e2, and the normal points out of the hull.
                    if let error = checkEdgeContact(
                        manifold, [buffer[0]], edgePoint, edgeDir, c1, c2 - c1, outward,
                        radius: radius, normalTol: 1e-4, sepTol: 1e-4, pointTol: 1e-3
                    ) {
                        failures.append("\(label): \(error)")
                        continue
                    }

                    count += 1
                }
            }
        }

        #expect(failures.isEmpty, "\(failures.prefix(3))")
        #expect(count == 27)
    }
}

// MARK: - Edge cache reuse

extension ManifoldRobustnessTests {
    /// The cached edge features must survive small motion, and once the speculative distance is
    /// exceeded the cached axis alone has to report the separation.
    @Test("edge cache reuses features across small motion then reports separation")
    func edgeCache() {
        let (hullA, hullB) = crossedEdgeHulls(0.5)
        let root2 = (2 as Float).squareRoot()

        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
        var manifold = LocalManifold()
        var cache = SATCache()

        func run(_ distance: Float) -> Int32 {
            let transform = Transform(Vec3(distance, 0, 0), Quat.identity)
            buffer.withUnsafeMutableBufferPointer { points in
                manifold.points = points.baseAddress
                collideHulls(&manifold, 8, hullA, hullB, transform, &cache)
            }
            return manifold.pointCount
        }

        #expect(run(1.41) == 1)
        #expect(cache.type == .edgePairAxis)
        // Cached edges are the even half of each twin pair.
        #expect((cache.indexA & 1) == 0 && cache.indexA < UInt8(truncatingIfNeeded: hullA.halfEdgeCount))
        #expect((cache.indexB & 1) == 0 && cache.indexB < UInt8(truncatingIfNeeded: hullB.halfEdgeCount))

        let seededSeparation = cache.separation
        #expect(absFloat(seededSeparation - (1.41 - root2)) < 1e-5)

        // Small motion: the cached features still describe the contact.
        #expect(run(1.4105) == 1)
        #expect(cache.separation == seededSeparation)
        #expect(absFloat(buffer[0].separation - (1.4105 - root2)) < 1e-5)
        #expect(absFloat(manifold.normal.x - 1) < 1e-6)

        // Jump past the speculative distance: the cached axis alone reports the separation.
        #expect(run(1.5) == 0)
        #expect(cache.separation == seededSeparation)
    }
}

// MARK: - Edge endpoints

extension ManifoldRobustnessTests {
    private func collideManualEdge(
        _ hullA: Hull, _ hullB: Hull, _ transform: Transform
    ) -> (count: Int32, cache: SATCache, points: [LocalManifoldPoint]) {
        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
        var manifold = LocalManifold()
        var cache = SATCache()
        cache.type = .manualEdgePairAxis
        buffer.withUnsafeMutableBufferPointer { pts in
            manifold.points = pts.baseAddress
            collideHulls(&manifold, 8, hullA, hullB, transform, &cache)
        }
        return (manifold.pointCount, cache, Array(buffer[0..<Int(manifold.pointCount)]))
    }

    /// Past the end of an edge the pair no longer describes a contact, so the builder must reject
    /// it and clear the cache rather than clamping to a point that is not on either hull.
    @Test("an edge pair beyond its endpoint is rejected and clears the cache")
    func edgeEndpoint() {
        let (hullA, hullB) = crossedEdgeHulls(0.5)
        let root2 = (2 as Float).squareRoot()
        let d: Float = 1.41
        let expected = d - root2

        // Just inside the end of edge A.
        let inside = collideManualEdge(
            hullA, hullB, Transform(Vec3(d, 0.49, 0), Quat.identity))
        #expect(inside.count == 1)
        #expect(absFloat(inside.points[0].separation - expected) < 1e-5)
        #expect(absFloat(inside.points[0].point.y - 0.49) < 1e-5)

        // Off the end.
        let outside = collideManualEdge(
            hullA, hullB, Transform(Vec3(d, 0.55, 0), Quat.identity))
        #expect(outside.count == 0)
        #expect(outside.cache.type == .invalidAxis)

        // The true gap is a vertex to edge distance well past the speculative distance.
        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
        var manifold = LocalManifold()
        var freshCache = SATCache()
        buffer.withUnsafeMutableBufferPointer { pts in
            manifold.points = pts.baseAddress
            collideHulls(
                &manifold, 8, hullA, hullB,
                Transform(Vec3(d, 0.55, 0), Quat.identity), &freshCache
            )
        }
        #expect(manifold.pointCount == 0)
        #expect(freshCache.separation > 0)
    }
}

// MARK: - Parallel edges

extension ManifoldRobustnessTests {
    /// Two parallel faces must keep a face normal no matter how slightly they are tilted. A tiny
    /// tilt makes the edge cross product pure noise, so the collider has to reject the edge path.
    @Test("tiny tilts keep the face normal on parallel boxes")
    func parallelEdge() {
        let hullA = makeBoxHull(0.5, 0.5, 0.5)
        let hullB = makeBoxHull(0.5, 0.5, 0.5)

        let tiltAxes = [
            Vec3(0.57735027, 0.57735027, 0.57735027),
            Vec3(0.70710678, 0, 0.70710678),
            Vec3(0.26726124, 0.53452248, 0.80178373),
            Vec3(-0.48507125, 0.72760688, -0.48507125),
        ]
        // Angles that straddle the 0.005 rejection threshold.
        let tiltAngles: [Float] = [
            0, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 0.004, 0.005, 0.006, 0.01, 0.05,
        ]
        let overlap: Float = 0.01
        // A cube corner is root3/2 from the center of rotation.
        let halfDiagonal: Float = 0.87

        var failures: [String] = []

        for axis in tiltAxes {
            for angle in tiltAngles {
                let transform = Transform(Vec3(0, 1 - overlap, 0), exactQuat(axis, angle))

                var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
                var manifold = LocalManifold()
                var cache = SATCache()
                buffer.withUnsafeMutableBufferPointer { pts in
                    manifold.points = pts.baseAddress
                    collideHulls(&manifold, 8, hullA, hullB, transform, &cache)
                }

                let label = "axis \(axis) angle \(angle)"
                if manifold.pointCount < 4 || manifold.pointCount > B3.maxManifoldPoints {
                    failures.append("\(label): count \(manifold.pointCount)")
                    continue
                }
                if cache.type != .faceAxisA && cache.type != .faceAxisB {
                    failures.append("\(label): cache \(cache.type)")
                    continue
                }
                if dot(manifold.normal, .axisY) <= 0.998 {
                    failures.append("\(label): normal \(manifold.normal)")
                    continue
                }

                // The tilt can only lift or sink a face point by the arc it sweeps.
                let bound = halfDiagonal * angle + 1e-5
                for k in 0..<Int(manifold.pointCount) {
                    if absFloat(buffer[k].separation + overlap) > bound {
                        failures.append(
                            "\(label): point \(k) separation \(buffer[k].separation)")
                        break
                    }
                }
            }
        }

        #expect(failures.isEmpty, "\(failures.prefix(3))")
    }
}

// MARK: - Sphere hull seam

extension ManifoldRobustnessTests {
    /// Walk a sphere through a box face. The separation must be the analytic gap on both sides of
    /// the seam, the normal must not flip, and consecutive samples must track the step exactly.
    @Test("sphere crossing a hull face keeps separation and normal continuous")
    func sphereHullSeam() {
        let hull = makeBoxHull(0.5, 0.5, 0.5)
        let radius: Float = 0.15

        let yStart = 0.5 + radius + 0.4 * B3.speculativeDistance
        let yEnd: Float = 0.1
        let steps = 400
        let dy = (yStart - yEnd) / Float(steps)

        var previous: Float = 0
        var shallowSamples = 0
        var deepSamples = 0
        var failures: [String] = []

        for i in 0...steps {
            let y = yStart - Float(i) * dy
            let sphere = Sphere(center: Vec3(0, y, 0), radius: radius)

            var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
            var manifold = LocalManifold()
            var cache = SimplexCache()
            buffer.withUnsafeMutableBufferPointer { pts in
                manifold.points = pts.baseAddress
                collideHullAndSphere(
                    &manifold, 8, hull, sphere, Transform(Vec3(), Quat.identity), &cache
                )
            }

            if manifold.pointCount != 1 {
                failures.append("step \(i): count \(manifold.pointCount)")
                break
            }

            let separation = buffer[0].separation
            let expected = (y - 0.5) - radius

            // Separation is the analytic gap on both sides of the seam.
            if absFloat(separation - expected) > 1e-5 {
                failures.append("step \(i): separation \(separation) vs \(expected)")
                break
            }
            // Normal holds the face direction with no flip.
            let n = manifold.normal
            if absFloat(n.x) > 1e-5 || absFloat(n.y - 1) > 1e-5 || absFloat(n.z) > 1e-5 {
                failures.append("step \(i): normal \(n)")
                break
            }
            // No jump across the seam: consecutive separations track the step.
            if i > 0 && absFloat((previous - separation) - dy) > 1e-5 {
                failures.append("step \(i): jump \(previous - separation) vs \(dy)")
                break
            }
            previous = separation

            if y > 0.5 { shallowSamples += 1 } else { deepSamples += 1 }
        }

        #expect(failures.isEmpty, "\(failures.prefix(2))")
        // The sweep must straddle the surface so both the GJK and the SAT branch run.
        #expect(shallowSamples > 0)
        #expect(deepSamples > 0)
    }
}

// MARK: - Capsule hull seam

extension ManifoldRobustnessTests {
    /// The capsule analogue of the sphere seam walk: every clipped point sits at the analytic gap
    /// and the minimum separation tracks the step with no jump across the seam.
    @Test("capsule crossing a hull face keeps every point at the analytic gap")
    func capsuleHullSeam() {
        let hull = makeBoxHull(0.5, 0.5, 0.5)
        let radius: Float = 0.15
        let halfLength: Float = 0.3

        let yStart = 0.5 + radius + 0.4 * B3.speculativeDistance
        let yEnd: Float = 0.1
        let steps = 400
        let dy = (yStart - yEnd) / Float(steps)

        var previous: Float = 0
        var shallowSamples = 0
        var deepSamples = 0
        var failures: [String] = []

        for i in 0...steps {
            let y = yStart - Float(i) * dy
            let capsule = Capsule(
                center1: Vec3(-halfLength, y, 0), center2: Vec3(halfLength, y, 0), radius: radius
            )

            var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
            var manifold = LocalManifold()
            var cache = SimplexCache()
            buffer.withUnsafeMutableBufferPointer { pts in
                manifold.points = pts.baseAddress
                collideHullAndCapsule(
                    &manifold, 8, hull, capsule, Transform(Vec3(), Quat.identity), &cache
                )
            }

            let count = Int(manifold.pointCount)
            guard count >= 1 else {
                failures.append("step \(i): no points")
                break
            }

            let expected = (y - 0.5) - radius
            var badPoint: Float?
            for k in 0..<count where absFloat(buffer[k].separation - expected) > 1e-5 {
                badPoint = buffer[k].separation
                break
            }
            if let badPoint {
                failures.append("step \(i): separation \(badPoint) vs \(expected)")
                break
            }

            let n = manifold.normal
            if absFloat(n.x) > 1e-5 || absFloat(n.y - 1) > 1e-5 || absFloat(n.z) > 1e-5 {
                failures.append("step \(i): normal \(n)")
                break
            }

            let minSeparation = buffer[0..<count].map(\.separation).min() ?? 0
            if i > 0 && absFloat((previous - minSeparation) - dy) > 1e-5 {
                failures.append("step \(i): jump \(previous - minSeparation) vs \(dy)")
                break
            }
            previous = minSeparation

            if y > 0.5 { shallowSamples += 1 } else { deepSamples += 1 }
        }

        #expect(failures.isEmpty, "\(failures.prefix(2))")
        #expect(shallowSamples > 0)
        #expect(deepSamples > 0)
    }
}

// MARK: - Triangle edge pair

extension ManifoldRobustnessTests {
    /// Tipping the triangle plane about z keeps its normal off the hull edge, which the Minkowski
    /// test needs. Tipping the edge within that plane moves the arc intersection off the midpoint.
    @Test("triangle and hull recover the edge pair axis and analytic point")
    func triangleEdge() {
        let halfRoot2: Float = 0.70710678
        let hull = makeTransformedBoxHull(
            0.5, 0.5, 0.5, Transform(Vec3(), exactQuat(.axisX, 0.25 * B3.pi)))

        let beta = 20 * B3.pi / 180
        let gamma = 30 * B3.pi / 180

        let triNormal = Vec3(Foundation.sinf(beta), Foundation.cosf(beta), 0)
        let triEdge = Vec3(
            Foundation.sinf(gamma) * Foundation.cosf(beta),
            -Foundation.sinf(gamma) * Foundation.sinf(beta),
            Foundation.cosf(gamma)
        )

        // Perpendicular to both edges and pointing out of the hull.
        let axis = normalize(cross(.axisX, triEdge))
        let hullPoint = Vec3(0, -halfRoot2, 0)

        var failures: [String] = []

        for gap in [0.03 as Float, 0.01, 0.0, -0.01, -0.1] {
            let trianglePoint = mulAdd(hullPoint, gap, axis)

            let v1 = mulAdd(trianglePoint, -1, triEdge)
            let v2 = mulAdd(trianglePoint, 1, triEdge)
            let v3 = mulAdd(v1, 1.5, cross(triNormal, triEdge))

            var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
            var manifold = LocalManifold()
            var cache = SATCache()
            cache.type = .manualEdgePairAxis
            buffer.withUnsafeMutableBufferPointer { pts in
                manifold.points = pts.baseAddress
                collideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, hull, &cache, true)
            }

            let label = "gap \(gap)"
            if manifold.pointCount != 1 {
                failures.append("\(label): count \(manifold.pointCount)")
                continue
            }
            if cache.type != .edgePairAxis {
                failures.append("\(label): cache \(cache.type)")
                continue
            }

            let expectedNormal = -axis
            let expectedPoint = mulAdd(hullPoint, 0.5 * gap, axis)
            let n = manifold.normal
            let p = buffer[0]

            if absFloat(n.x - expectedNormal.x) > 1e-5
                || absFloat(n.y - expectedNormal.y) > 1e-5
                || absFloat(n.z - expectedNormal.z) > 1e-5
            {
                failures.append("\(label): normal \(n) vs \(expectedNormal)")
                continue
            }
            if absFloat(p.separation - gap) > 1e-5 {
                failures.append("\(label): separation \(p.separation)")
                continue
            }
            if absFloat(p.point.x - expectedPoint.x) > 1e-5
                || absFloat(p.point.y - expectedPoint.y) > 1e-5
                || absFloat(p.point.z - expectedPoint.z) > 1e-5
            {
                failures.append("\(label): point \(p.point) vs \(expectedPoint)")
                continue
            }
        }

        #expect(failures.isEmpty, "\(failures.prefix(2))")

        // Far separated: no contact, but the cached edge separation must still report the gap.
        let farGap: Float = 0.2
        let trianglePoint = mulAdd(hullPoint, farGap, axis)
        let v1 = mulAdd(trianglePoint, -1, triEdge)
        let v2 = mulAdd(trianglePoint, 1, triEdge)
        let v3 = mulAdd(v1, 1.5, cross(triNormal, triEdge))

        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
        var manifold = LocalManifold()
        var cache = SATCache()
        buffer.withUnsafeMutableBufferPointer { pts in
            manifold.points = pts.baseAddress
            collideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, hull, &cache, true)
        }
        #expect(manifold.pointCount == 0)
        #expect(cache.type == .edgePairAxis)
        #expect(absFloat(cache.separation - farGap) < 1e-5)
    }
}

// MARK: - Speculative margin

extension ManifoldRobustnessTests {
    /// Contact recycling treats a pair without points as separated by at least the speculative
    /// distance, so every manifold routine must report a pair inside that distance or recycling can
    /// skip real overlap. Discussion #175.
    @Test("every manifold routine reports pairs inside the speculative distance")
    func speculativeMargin() {
        let maxPoints = B3.maxManifoldPoints

        let sphere = Sphere(center: Vec3(), radius: 0.25)
        let bigSphere = Sphere(center: Vec3(), radius: 0.5)
        let capsuleX = Capsule(
            center1: Vec3(-0.5, 0, 0), center2: Vec3(0.5, 0, 0), radius: 0.25)
        let capsuleZ = Capsule(
            center1: Vec3(0, 0, -0.5), center2: Vec3(0, 0, 0.5), radius: 0.25)
        let box = makeBoxHull(0.5, 0.5, 0.5)

        // Triangle with a +y normal.
        var triangle = [Vec3(-2, 0, -2), Vec3(0, 0, 2), Vec3(2, 0, -2)]

        /// Every routine must produce points whose minimum separation is the requested gap.
        func check(_ label: String, _ gap: Float, _ points: [LocalManifoldPoint]) {
            if points.isEmpty {
                Issue.record("\(label) gap \(gap): no points")
                return
            }
            let minSeparation = points.map(\.separation).min() ?? 0
            if absFloat(minSeparation - gap) > 1e-4 {
                Issue.record("\(label) gap \(gap): separation \(minSeparation)")
            }
        }

        func collide(_ body: (inout LocalManifold) -> Void) -> [LocalManifoldPoint] {
            var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: Int(maxPoints))
            var manifold = LocalManifold()
            var out: [LocalManifoldPoint] = []
            buffer.withUnsafeMutableBufferPointer { pts in
                manifold.points = pts.baseAddress
                body(&manifold)
                out = Array(pts[0..<Int(manifold.pointCount)])
            }
            return out
        }

        for gap in [0 as Float, 0.25 * B3.speculativeDistance, 0.75 * B3.speculativeDistance] {
            let slideY75 = Transform(Vec3(0, 0.75 + gap, 0), Quat.identity)
            let slideY50 = Transform(Vec3(0, 0.5 + gap, 0), Quat.identity)
            let slideY100 = Transform(Vec3(0, 1.0 + gap, 0), Quat.identity)

            check(
                "sphere-sphere", gap,
                collide { m in collideSpheres(&m, maxPoints, bigSphere, sphere, slideY75) })

            check(
                "capsule-sphere", gap,
                collide { m in
                    collideCapsuleAndSphere(&m, maxPoints, capsuleX, sphere, slideY50)
                })

            check(
                "capsule-capsule", gap,
                collide { m in collideCapsules(&m, maxPoints, capsuleX, capsuleZ, slideY50) })

            check(
                "hull-sphere", gap,
                collide { m in
                    var cache = SimplexCache()
                    collideHullAndSphere(&m, maxPoints, box, sphere, slideY75, &cache)
                })

            check(
                "hull-capsule", gap,
                collide { m in
                    var cache = SimplexCache()
                    collideHullAndCapsule(&m, maxPoints, box, capsuleX, slideY75, &cache)
                })

            check(
                "hull-hull", gap,
                collide { m in
                    var cache = SATCache()
                    collideHulls(&m, maxPoints, box, box, slideY100, &cache)
                })

            check(
                "triangle-sphere", gap,
                collide { m in
                    let raised = Sphere(center: Vec3(0, 0.25 + gap, 0), radius: 0.25)
                    triangle.withUnsafeBufferPointer { tri in
                        collideTriangleAndSphere(&m, maxPoints, tri.baseAddress!, raised)
                    }
                })

            check(
                "triangle-capsule", gap,
                collide { m in
                    let raised = Capsule(
                        center1: Vec3(-0.5, 0.25 + gap, 0), center2: Vec3(0.5, 0.25 + gap, 0),
                        radius: 0.25)
                    var cache = SimplexCache()
                    triangle.withUnsafeBufferPointer { tri in
                        collideTriangleAndCapsule(&m, maxPoints, tri.baseAddress!, raised, &cache)
                    }
                })
        }

        // Beyond the speculative distance the closed form routines report nothing.
        let farGap = 1.5 * B3.speculativeDistance

        let farSpheres = collide { m in
            collideSpheres(
                &m, maxPoints, bigSphere, sphere,
                Transform(Vec3(0, 0.75 + farGap, 0), Quat.identity))
        }
        #expect(farSpheres.isEmpty)

        let farCapsule = collide { m in
            collideCapsuleAndSphere(
                &m, maxPoints, capsuleX, sphere,
                Transform(Vec3(0, 0.5 + farGap, 0), Quat.identity))
        }
        #expect(farCapsule.isEmpty)
    }
}
