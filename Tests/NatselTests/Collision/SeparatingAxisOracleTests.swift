import Testing
internal import Foundation
@testable import Natsel

@Suite("Separating axis oracle")
struct SeparatingAxisOracleTests {
    private struct Random {
        var seed: UInt32 = 987_654_321

        mutating func next() -> UInt32 {
            seed ^= seed << 13
            seed ^= seed >> 17
            seed ^= seed << 5
            return seed
        }

        mutating func float(_ lower: Float, _ upper: Float) -> Float {
            let unit = Float(next() >> 8) / 16_777_216
            return lower + (upper - lower) * unit
        }

        mutating func vector(_ lower: Float, _ upper: Float) -> Vec3 {
            Vec3(float(lower, upper), float(lower, upper), float(lower, upper))
        }

        mutating func direction() -> Vec3 {
            while true {
                let v = vector(-1, 1)
                let len = length(v)
                if 0.1 < len && len < 1 { return (1 / len) * v }
            }
        }

        mutating func quat() -> Quat {
            makeQuatFromAxisAngle(direction(), float(0, B3.pi))
        }
    }

    private func exactQuat(_ axis: Vec3, _ radians: Float) -> Quat {
        let half = 0.5 * radians
        return Quat(Foundation.sinf(half) * axis, Foundation.cosf(half))
    }

    private func hullPoints(_ hull: Hull) -> [Vec3] {
        guard let raw = hull.points else { return [] }
        return (0..<Int(hull.vertexCount)).map { raw[$0] }
    }

    /// Projection gap: min over B minus max over A along the given normal.
    private func separationAlong(
        _ hullA: Hull, _ hullB: Hull, _ xfB: Transform, _ n: Vec3
    ) -> Float {
        var maxA = -Float.greatestFiniteMagnitude
        for p in hullPoints(hullA) {
            maxA = maxFloat(maxA, dot(n, p))
        }
        var minB = Float.greatestFiniteMagnitude
        for p in hullPoints(hullB) {
            minB = minFloat(minB, dot(n, transformPoint(xfB, p)))
        }
        return minB - maxA
    }

    /// The true maximum separation over every face axis and edge pair. Undirected axes are tried
    /// both ways so the winner is oriented from A to B.
    private func oracleSeparation(
        _ hullA: Hull, _ hullB: Hull, _ xfB: Transform
    ) -> (separation: Float, normal: Vec3) {
        var best = -Float.greatestFiniteMagnitude
        var bestNormal = Vec3()

        func tryAxis(_ n: Vec3) {
            let s = separationAlong(hullA, hullB, xfB, n)
            if s > best {
                best = s
                bestNormal = n
            }
        }

        // Face axes of A.
        if let planesA = hullA.planes {
            for i in 0..<Int(hullA.faceCount) {
                let n = planesA[i].normal
                tryAxis(n)
                tryAxis(-n)
            }
        }
        // Face axes of B, rotated into A's frame.
        if let planesB = hullB.planes {
            for i in 0..<Int(hullB.faceCount) {
                let n = rotateVector(xfB.q, planesB[i].normal)
                tryAxis(n)
                tryAxis(-n)
            }
        }
        // Edge pairs. Half edges store adjacent twins, so the even index and its successor bound
        // one edge.
        if let edgesA = hullA.edges, let edgesB = hullB.edges {
            let pA = hullPoints(hullA)
            let pB = hullPoints(hullB)
            var i = 0
            while i < Int(hullA.halfEdgeCount) {
                let dirA = pA[Int(edgesA[i + 1].origin)] - pA[Int(edgesA[i].origin)]
                var j = 0
                while j < Int(hullB.halfEdgeCount) {
                    let dirB = rotateVector(
                        xfB.q, pB[Int(edgesB[j + 1].origin)] - pB[Int(edgesB[j].origin)])
                    let c = cross(dirA, dirB)
                    // Near parallel edges give no useful axis and cannot be the true maximum.
                    if dot(c, c) >= 1e-10 {
                        tryAxis(normalize(c))
                    }
                    j += 2
                }
                i += 2
            }
        }
        return (best, bestNormal)
    }

    @Test("the query never overstates the true separation")
    func axisOracle() {
        var random = Random()

        var separated = 0
        var penetrating = 0
        var edgeWins = 0

        for i in 0..<4000 {
            let halfA: Vec3
            let halfB: Vec3
            let reach: Float

            if i & 1 != 0 {
                // Thin crossed beams driven into deep overlap.
                halfA = Vec3(random.float(1.0, 1.6), random.float(0.08, 0.15), random.float(0.08, 0.15))
                halfB = Vec3(random.float(1.0, 1.6), random.float(0.08, 0.15), random.float(0.08, 0.15))
                reach = random.float(0, 0.4)
            } else {
                // Chunky boxes across the whole separated to overlapping range.
                halfA = random.vector(0.3, 0.8)
                halfB = random.vector(0.3, 0.8)
                reach = random.float(0, 1.7)
            }

            // Rotate A in place so it stays centered on the origin, which keeps the support bias
            // valid.
            let hullA = makeTransformedBoxHull(
                halfA.x, halfA.y, halfA.z,
                Transform(Vec3(), exactQuat(random.direction(), random.float(0, B3.pi))))
            let hullB = makeBoxHull(halfB.x, halfB.y, halfB.z)

            let xfB = Transform(
                reach * random.direction(), exactQuat(random.direction(), random.float(0, B3.pi)))

            let query = computeSeparatingAxis(hullA, hullB, xfB, true)
            let q = getBestAxis(query)

            let oracle = oracleSeparation(hullA, hullB, xfB)

            #expect(absFloat(length(q.normal) - 1) < 1e-3, "sample \(i) normal not unit")
            #expect(
                absFloat(separationAlong(hullA, hullB, xfB, q.normal) - q.separation) < 2e-3,
                "sample \(i) separation inconsistent")

            // Never overstate the true maximum: the reported axis is real.
            #expect(q.separation <= oracle.separation + 2e-3, "sample \(i) overstated")

            // Inside the speculative band the scan runs exhaustively, so the result is the maximum.
            if q.separation <= B3.speculativeDistance {
                #expect(
                    q.separation >= oracle.separation - 4e-3,
                    "sample \(i) shortfall \(oracle.separation - q.separation)")
            }

            if oracle.separation > 0 { separated += 1 } else { penetrating += 1 }
            if q.type == .edgePairAxis { edgeWins += 1 }
        }

        // The sweep must cover both regimes and actually drive the edge path.
        #expect(separated > 100)
        #expect(penetrating > 100)
        #expect(edgeWins > 20)
    }
}

// MARK: - Edge pair sweep

extension SeparatingAxisOracleTests {
    /// Crossed beam edges: the axis of minimum penetration is the edge pair across the whole
    /// separated-to-overlap range, and it must match the brute-force oracle.
    @Test("crossed beams keep the edge pair axis across the whole sweep")
    func edgePairSweep() {
        let hullA = makeTransformedBoxHull(
            0.5, 0.5, 0.5, Transform(Vec3(), exactQuat(.axisY, 0.25 * B3.pi)))
        let hullB = makeTransformedBoxHull(
            0.5, 0.5, 0.5, Transform(Vec3(), exactQuat(.axisZ, 0.25 * B3.pi)))

        let root2 = (2 as Float).squareRoot()

        for d in [1.43 as Float, 1.42, root2, 1.40, 1.38, 1.35, 1.30] {
            let expected = d - root2
            let xfB = Transform(Vec3(d, 0, 0), Quat.identity)

            let query = computeSeparatingAxis(hullA, hullB, xfB, true)
            let q = getBestAxis(query)

            #expect(q.type == .edgePairAxis, "d \(d) type \(q.type)")
            #expect(absFloat(q.separation - expected) < 1e-4, "d \(d) separation \(q.separation)")
            #expect(absFloat(q.normal.x - 1) < 1e-4, "d \(d) normal \(q.normal)")
            #expect(absFloat(q.normal.y) < 1e-4)
            #expect(absFloat(q.normal.z) < 1e-4)

            // Half edge indices are the even twin of each pair.
            #expect((q.indexA & 1) == 0)
            #expect((q.indexB & 1) == 0)

            let oracle = oracleSeparation(hullA, hullB, xfB)
            #expect(absFloat(q.separation - oracle.separation) < 1e-4, "d \(d) vs oracle")
            #expect(
                absFloat(separationAlong(hullA, hullB, xfB, q.normal) - q.separation) < 1e-4,
                "d \(d) projection mismatch")
        }
    }
}

// MARK: - Face axis selection

extension SeparatingAxisOracleTests {
    @Test("a separated pair picks body A's face axis")
    func faceAxisA() {
        let hullA = makeBoxHull(0.5, 0.5, 0.5)
        let hullB = makeBoxHull(0.5, 0.5, 0.5)
        let xfB = Transform(Vec3(1.2, 0, 0), Quat.identity)

        let q = getBestAxis(computeSeparatingAxis(hullA, hullB, xfB, true))

        #expect(q.type == .faceAxisA)
        #expect(absFloat(q.separation - 0.2) < 1e-5)
        #expect(absFloat(q.normal.x - 1) < 1e-6)
        #expect(absFloat(q.normal.y) < 1e-6)
        #expect(absFloat(q.normal.z) < 1e-6)

        if let planesA = hullA.planes {
            #expect(absFloat(planesA[Int(q.indexA)].normal.x - 1) < 1e-6)
        }
        if let pointsB = hullB.points {
            #expect(absFloat(pointsB[Int(q.indexB)].x + 0.5) < 1e-6)
        }
        #expect(absFloat(separationAlong(hullA, hullB, xfB, q.normal) - q.separation) < 1e-5)
    }

    @Test("a rotated body A hands the axis to body B")
    func faceAxisB() {
        let hullA = makeTransformedBoxHull(
            0.5, 0.5, 0.5, Transform(Vec3(), exactQuat(.axisZ, 0.25 * B3.pi)))
        let hullB = makeBoxHull(0.5, 0.5, 0.5)

        let root2 = (2 as Float).squareRoot()
        let aExtent = 0.5 * root2
        let gap: Float = 0.2
        let d = aExtent + 0.5 + gap
        let xfB = Transform(Vec3(d, 0, 0), Quat.identity)

        let q = getBestAxis(computeSeparatingAxis(hullA, hullB, xfB, true))

        #expect(q.type == .faceAxisB)
        #expect(absFloat(q.separation - gap) < 1e-5)
        #expect(absFloat(q.normal.x - 1) < 1e-5)
        #expect(absFloat(q.normal.y) < 1e-5)
        #expect(absFloat(q.normal.z) < 1e-5)

        if let planesB = hullB.planes {
            #expect(absFloat(planesB[Int(q.indexB)].normal.x + 1) < 1e-5)
        }
        if let pointsA = hullA.points {
            #expect(absFloat(pointsA[Int(q.indexA)].x - aExtent) < 1e-4)
        }
        #expect(absFloat(separationAlong(hullA, hullB, xfB, q.normal) - q.separation) < 1e-4)
    }

    @Test("a far separated pair still reports the exact gap")
    func faceFarSeparated() {
        let hullA = makeBoxHull(0.5, 0.5, 0.5)
        let hullB = makeBoxHull(0.5, 0.5, 0.5)
        let xfB = Transform(Vec3(3, 0, 0), Quat.identity)

        let q = getBestAxis(computeSeparatingAxis(hullA, hullB, xfB, true))

        #expect(q.type == .faceAxisA)
        #expect(absFloat(q.separation - 2) < 1e-5)
        #expect(absFloat(q.normal.x - 1) < 1e-6)

        let oracle = oracleSeparation(hullA, hullB, xfB)
        #expect(absFloat(q.separation - oracle.separation) < 1e-5)
    }

    /// Body A carries its origin far from the world center, so the face axis lookup has to survive
    /// the offset rather than reporting the world origin.
    @Test("an offset body A keeps the face axis lookup exact")
    func offsetFaceAxisB() {
        let cx: Float = 5
        let placeA = Transform(Vec3(cx, 0, 0), exactQuat(.axisZ, 0.25 * B3.pi))
        let hullA = makeTransformedBoxHull(0.5, 0.5, 0.5, placeA)
        let hullB = makeBoxHull(0.5, 0.5, 0.5)

        let root2 = (2 as Float).squareRoot()
        let aExtent = cx + 0.5 * root2
        let gap: Float = 0.2
        let d = aExtent + 0.5 + gap
        let xfB = Transform(Vec3(d, 0, 0), Quat.identity)

        let q = getBestAxis(computeSeparatingAxis(hullA, hullB, xfB, true))

        #expect(q.type == .faceAxisB)
        #expect(absFloat(q.separation - gap) < 1e-4)
        #expect(absFloat(q.normal.x - 1) < 1e-5)
        #expect(absFloat(q.normal.y) < 1e-5)
        #expect(absFloat(q.normal.z) < 1e-5)

        if let planesB = hullB.planes {
            #expect(absFloat(planesB[Int(q.indexB)].normal.x + 1) < 1e-5)
        }
        if let pointsA = hullA.points {
            #expect(absFloat(pointsA[Int(q.indexA)].x - aExtent) < 1e-4)
        }
    }

    /// A hull whose origin sits far from its own geometry: the support pick must not be biased by
    /// the offset, which shows up as the returned normal disagreeing with its own separation.
    @Test("offset hulls keep the reported axis consistent with its separation")
    func offsetHullOracle() {
        var random = Random(seed: 24_681_012)

        var separated = 0
        var penetrating = 0
        var edgeWins = 0

        for i in 0..<3000 {
            let halfA = random.vector(0.3, 0.8)
            let halfB = random.vector(0.3, 0.8)

            let offset = random.float(2, 6) * random.direction()
            let reach = random.float(0, 1.7)
            let dir = random.direction()

            let hullA: Hull
            let hullB: Hull
            let xfB: Transform

            if i & 1 != 0 {
                // A carries the offset in its own frame.
                let placeA = Transform(offset, exactQuat(random.direction(), random.float(0, B3.pi)))
                hullA = makeTransformedBoxHull(halfA.x, halfA.y, halfA.z, placeA)
                hullB = makeBoxHull(halfB.x, halfB.y, halfB.z)
                xfB = Transform(
                    mulAdd(offset, reach, dir),
                    exactQuat(random.direction(), random.float(0, B3.pi)))
            } else {
                // B carries the offset in its own frame, placed so its world center lands near the
                // origin.
                let qB = exactQuat(random.direction(), random.float(0, B3.pi))
                hullA = makeBoxHull(halfA.x, halfA.y, halfA.z)
                hullB = makeOffsetBoxHull(halfB.x, halfB.y, halfB.z, offset)
                xfB = Transform(reach * dir - rotateVector(qB, offset), qB)
            }

            let q = getBestAxis(computeSeparatingAxis(hullA, hullB, xfB, true))
            let oracle = oracleSeparation(hullA, hullB, xfB)

            // Consistency is the sharp bias check: a wrong support pick on an offset hull throws the
            // normal off its own separation by a vertex spacing, not a noise floor.
            let consistency = absFloat(
                separationAlong(hullA, hullB, xfB, q.normal) - q.separation)

            #expect(absFloat(length(q.normal) - 1) < 1e-3, "sample \(i) normal not unit")
            #expect(consistency < 1e-3, "sample \(i) consistency \(consistency)")
            #expect(q.separation <= oracle.separation + 3e-3, "sample \(i) overstated")

            // Only the in-band result feeds a manifold, and only there does the scan run to
            // completion. Offset coordinates cost a few more digits.
            if q.separation <= B3.speculativeDistance {
                #expect(
                    q.separation >= oracle.separation - 8e-3,
                    "sample \(i) shortfall \(oracle.separation - q.separation)")
            }

            if oracle.separation > 0 { separated += 1 } else { penetrating += 1 }
            if q.type == .edgePairAxis { edgeWins += 1 }
        }

        #expect(separated > 100)
        #expect(penetrating > 100)
        #expect(edgeWins > 20)
    }
}
