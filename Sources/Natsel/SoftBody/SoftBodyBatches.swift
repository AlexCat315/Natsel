// Four wide constraint batches. Constraints in one batch share no vertex, so the four lanes are
// solved together with SIMD and the result equals solving them one after another. Batches are
// built greedily in the settings' constraint order with a short look-ahead window, which keeps the
// anchor-outward order of the edges (see `SoftBodySettings.optimize`) nearly intact while breaking
// the dependency chain between consecutive constraints.

typealias FloatX4 = SIMD4<Float>

struct EdgeBatch: BitwiseCopyable {
    var v0 = SIMD4<Int32>(repeating: 0)
    var v1 = SIMD4<Int32>(repeating: 0)
    var restLength = FloatX4()
    var compliance = FloatX4()
    var laneCount: Int32 = 0
}

struct BendBatch: BitwiseCopyable {
    var v0 = SIMD4<Int32>(repeating: 0)
    var v1 = SIMD4<Int32>(repeating: 0)
    var v2 = SIMD4<Int32>(repeating: 0)
    var v3 = SIMD4<Int32>(repeating: 0)
    var initialAngle = FloatX4()
    var compliance = FloatX4()
    var laneCount: Int32 = 0
}

struct VolumeBatch: BitwiseCopyable {
    var v0 = SIMD4<Int32>(repeating: 0)
    var v1 = SIMD4<Int32>(repeating: 0)
    var v2 = SIMD4<Int32>(repeating: 0)
    var v3 = SIMD4<Int32>(repeating: 0)
    var sixRestVolume = FloatX4()
    var compliance = FloatX4()
    var laneCount: Int32 = 0
}

/// Group constraints into batches of up to four vertex-disjoint constraints, in order.
/// `vertices(i)` gives the vertices of constraint i and how many of the four are used.
func buildConstraintBatches(
    count: Int, vertexCount: Int, window: Int = 64, _ vertices: (Int) -> (SIMD4<Int32>, Int)
) -> [[Int]] {
    var batches: [[Int]] = []
    batches.reserveCapacity(count / 4 + 1)
    var assigned = [Bool](repeating: false, count: count)
    var mark = [Int](repeating: -1, count: vertexCount)
    var first = 0
    var batchIndex = 0
    while first < count {
        while first < count && assigned[first] { first += 1 }
        if first >= count { break }
        var batch: [Int] = []
        var i = first
        let end = min(count, first + window)
        while i < end && batch.count < 4 {
            if assigned[i] == false {
                let (vs, used) = vertices(i)
                var free = true
                for k in 0..<used where mark[Int(vs[k])] == batchIndex { free = false }
                if free {
                    for k in 0..<used { mark[Int(vs[k])] = batchIndex }
                    batch.append(i)
                    assigned[i] = true
                }
            }
            i += 1
        }
        batches.append(batch)
        batchIndex += 1
    }
    return batches
}

extension SoftBodyRecord {
    mutating func buildBatches() {
        let n = settings.vertices.count
        let edges = settings.edges
        edgeBatches = buildConstraintBatches(count: edges.count, vertexCount: n) {
            (SIMD4(Int32(edges[$0].vertex.x), Int32(edges[$0].vertex.y), 0, 0), 2)
        }.map { lanes in
            var batch = EdgeBatch()
            batch.laneCount = Int32(lanes.count)
            for (k, e) in lanes.enumerated() {
                batch.v0[k] = Int32(edges[e].vertex.x)
                batch.v1[k] = Int32(edges[e].vertex.y)
                batch.restLength[k] = edges[e].restLength
                batch.compliance[k] = edges[e].compliance
            }
            return batch
        }

        let bends = settings.dihedralBends
        bendBatches = buildConstraintBatches(count: bends.count, vertexCount: n) {
            (SIMD4<Int32>(truncatingIfNeeded: bends[$0].vertex), 4)
        }.map { lanes in
            var batch = BendBatch()
            batch.laneCount = Int32(lanes.count)
            for (k, b) in lanes.enumerated() {
                let v = bends[b].vertex
                batch.v0[k] = Int32(v.x)
                batch.v1[k] = Int32(v.y)
                batch.v2[k] = Int32(v.z)
                batch.v3[k] = Int32(v.w)
                batch.initialAngle[k] = bends[b].initialAngle
                batch.compliance[k] = bends[b].compliance
            }
            return batch
        }

        let volumes = settings.volumes
        volumeBatches = buildConstraintBatches(count: volumes.count, vertexCount: n) {
            (SIMD4<Int32>(truncatingIfNeeded: volumes[$0].vertex), 4)
        }.map { lanes in
            var batch = VolumeBatch()
            batch.laneCount = Int32(lanes.count)
            for (k, t) in lanes.enumerated() {
                let v = volumes[t].vertex
                batch.v0[k] = Int32(v.x)
                batch.v1[k] = Int32(v.y)
                batch.v2[k] = Int32(v.z)
                batch.v3[k] = Int32(v.w)
                batch.sixRestVolume[k] = volumes[t].sixRestVolume
                batch.compliance[k] = volumes[t].compliance
            }
            return batch
        }
    }
}

// MARK: - Lane helpers

/// Positions of four vertices as three component vectors. Unused lanes repeat lane zero.
struct Vec3X4 {
    var x: FloatX4
    var y: FloatX4
    var z: FloatX4

    @inline(__always)
    init(_ x: FloatX4, _ y: FloatX4, _ z: FloatX4) {
        self.x = x
        self.y = y
        self.z = z
    }

    @inline(__always)
    init(gather p: UnsafeMutableBufferPointer<Vec3>, _ index: SIMD4<Int32>) {
        let a = p[Int(index[0])]
        let b = p[Int(index[1])]
        let c = p[Int(index[2])]
        let d = p[Int(index[3])]
        x = FloatX4(a.x, b.x, c.x, d.x)
        y = FloatX4(a.y, b.y, c.y, d.y)
        z = FloatX4(a.z, b.z, c.z, d.z)
    }

    @inline(__always)
    func scatter(_ p: UnsafeMutableBufferPointer<Vec3>, _ index: SIMD4<Int32>, _ lanes: Int32) {
        if lanes == 4 {
            // Full batch: straight line stores.
            p[Int(index[0])] = Vec3(x[0], y[0], z[0])
            p[Int(index[1])] = Vec3(x[1], y[1], z[1])
            p[Int(index[2])] = Vec3(x[2], y[2], z[2])
            p[Int(index[3])] = Vec3(x[3], y[3], z[3])
            return
        }
        for k in 0..<Int(lanes) {
            p[Int(index[k])] = Vec3(x[k], y[k], z[k])
        }
    }

    @inline(__always) static func + (a: Vec3X4, b: Vec3X4) -> Vec3X4 { Vec3X4(a.x + b.x, a.y + b.y, a.z + b.z) }
    @inline(__always) static func - (a: Vec3X4, b: Vec3X4) -> Vec3X4 { Vec3X4(a.x - b.x, a.y - b.y, a.z - b.z) }
    @inline(__always) static func * (s: FloatX4, a: Vec3X4) -> Vec3X4 { Vec3X4(s * a.x, s * a.y, s * a.z) }
    @inline(__always) static prefix func - (a: Vec3X4) -> Vec3X4 { Vec3X4(-a.x, -a.y, -a.z) }
}

@inline(__always)
private func dotX4(_ a: Vec3X4, _ b: Vec3X4) -> FloatX4 { a.x * b.x + a.y * b.y + a.z * b.z }

@inline(__always)
private func crossX4(_ a: Vec3X4, _ b: Vec3X4) -> Vec3X4 {
    Vec3X4(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}

@inline(__always)
private func gatherX4(_ w: UnsafeBufferPointer<Float>, _ index: SIMD4<Int32>) -> FloatX4 {
    FloatX4(w[Int(index[0])], w[Int(index[1])], w[Int(index[2])], w[Int(index[3])])
}

@inline(__always)
private func absX4(_ a: FloatX4) -> FloatX4 { a.replacing(with: -a, where: a .< 0) }

/// Lane-wise `atan2Approx`, the same polynomial and branches.
@inline(__always)
func atan2ApproxX4(_ y: FloatX4, _ x: FloatX4) -> FloatX4 {
    let ax = absX4(x)
    let ay = absX4(y)
    let mx = pointwiseMax(ay, ax)
    let mn = pointwiseMin(ay, ax)
    let a = mn / mx.replacing(with: 1, where: mx .== 0)
    let s = a * a
    let c = s * a
    let q = s * s
    var r = 0.024840285 * q + 0.18681418
    let t = -0.094097948 * q - 0.33213072
    r = r * s + t
    r = r * c + a
    r = r.replacing(with: 1.57079637 - r, where: ay .> ax)
    r = r.replacing(with: 3.14159274 - r, where: x .< 0)
    r = r.replacing(with: -r, where: y .< 0)
    return r.replacing(with: 0, where: (x .== 0) .& (y .== 0))
}

// MARK: - Batched solvers

extension SoftBodyRecord {
    func solveEdgeBatches(_ x: UnsafeMutableBufferPointer<Vec3>, _ w: UnsafeBufferPointer<Float>, _ invH2: Float) {
        edgeBatches.withUnsafeBufferPointer { batches in
            for batch in batches {
                let a = Vec3X4(gather: x, batch.v0)
                let b = Vec3X4(gather: x, batch.v1)
                let w0 = gatherX4(w, batch.v0)
                let w1 = gatherX4(w, batch.v1)
                let d = b - a
                let len = dotX4(d, d).squareRoot()
                let denominator = w0 + w1 + batch.compliance * invH2
                let valid = (len .> 1.0e-9) .& (denominator .> 0)
                let safeLen = len.replacing(with: 1, where: .!valid)
                let safeDenominator = denominator.replacing(with: 1, where: .!valid)
                var scale = -(len - batch.restLength) / safeDenominator / safeLen
                scale = scale.replacing(with: 0, where: .!valid)
                let correction = scale * d
                (a - w0 * correction).scatter(x, batch.v0, batch.laneCount)
                (b + w1 * correction).scatter(x, batch.v1, batch.laneCount)
            }
        }
    }

    func solveBendBatches(_ x: UnsafeMutableBufferPointer<Vec3>, _ w: UnsafeBufferPointer<Float>, _ invH2: Float) {
        bendBatches.withUnsafeBufferPointer { batches in
            for batch in batches {
                let p0 = Vec3X4(gather: x, batch.v0)
                let p1 = Vec3X4(gather: x, batch.v1)
                let p2 = Vec3X4(gather: x, batch.v2)
                let p3 = Vec3X4(gather: x, batch.v3)
                let w0 = gatherX4(w, batch.v0)
                let w1 = gatherX4(w, batch.v1)
                let w2 = gatherX4(w, batch.v2)
                let w3 = gatherX4(w, batch.v3)

                let e = p1 - p0
                let len = dotX4(e, e).squareRoot()
                let n1 = crossX4(e, p2 - p0)
                let n2 = crossX4(p3 - p0, e)
                let n1Squared = dotX4(n1, n1)
                let n2Squared = dotX4(n2, n2)
                let valid = (len .> 1.0e-9) .& (n1Squared .> 1.0e-18) .& (n2Squared .> 1.0e-18)
                    .& ((w0 + w1 + w2 + w3) .> 0)
                let safeLen = len.replacing(with: 1, where: .!valid)
                let invLen = 1 / safeLen
                let m1 = (1 / n1Squared.replacing(with: 1, where: .!valid)) * n1
                let m2 = (1 / n2Squared.replacing(with: 1, where: .!valid)) * n2

                let angle = atan2ApproxX4(dotX4(crossX4(n1, n2), e) * invLen, dotX4(n1, n2))
                var c = angle - batch.initialAngle
                c = c.replacing(with: c - B3.twoPi, where: c .> B3.pi)
                c = c.replacing(with: c + B3.twoPi, where: c .< -B3.pi)

                let g2 = -(safeLen * m1)
                let g3 = -(safeLen * m2)
                let g0 = -((dotX4(p2 - p1, e) * invLen) * m1 + (dotX4(p3 - p1, e) * invLen) * m2)
                let g1 = (dotX4(p2 - p0, e) * invLen) * m1 + (dotX4(p3 - p0, e) * invLen) * m2

                let denominator = w0 * dotX4(g0, g0) + w1 * dotX4(g1, g1) + w2 * dotX4(g2, g2)
                    + w3 * dotX4(g3, g3) + batch.compliance * invH2
                let solvable = valid .& (denominator .> 1.0e-12)
                var lambda = -c / denominator.replacing(with: 1, where: .!solvable)
                lambda = lambda.replacing(with: 0, where: .!solvable)

                (p0 + (w0 * lambda) * g0).scatter(x, batch.v0, batch.laneCount)
                (p1 + (w1 * lambda) * g1).scatter(x, batch.v1, batch.laneCount)
                (p2 + (w2 * lambda) * g2).scatter(x, batch.v2, batch.laneCount)
                (p3 + (w3 * lambda) * g3).scatter(x, batch.v3, batch.laneCount)
            }
        }
    }

    func solveVolumeBatches(_ x: UnsafeMutableBufferPointer<Vec3>, _ w: UnsafeBufferPointer<Float>, _ invH2: Float) {
        volumeBatches.withUnsafeBufferPointer { batches in
            for batch in batches {
                let x0 = Vec3X4(gather: x, batch.v0)
                let x1 = Vec3X4(gather: x, batch.v1)
                let x2 = Vec3X4(gather: x, batch.v2)
                let x3 = Vec3X4(gather: x, batch.v3)
                let w0 = gatherX4(w, batch.v0)
                let w1 = gatherX4(w, batch.v1)
                let w2 = gatherX4(w, batch.v2)
                let w3 = gatherX4(w, batch.v3)
                let e1 = x1 - x0
                let e2 = x2 - x0
                let e3 = x3 - x0
                let g1 = crossX4(e2, e3)
                let g2 = crossX4(e3, e1)
                let g3 = crossX4(e1, e2)
                let g0 = -(g1 + g2 + g3)
                let c = dotX4(e1, g1) - batch.sixRestVolume
                let denominator = w0 * dotX4(g0, g0) + w1 * dotX4(g1, g1) + w2 * dotX4(g2, g2)
                    + w3 * dotX4(g3, g3) + batch.compliance * invH2
                let solvable = denominator .> 1.0e-12
                var lambda = -c / denominator.replacing(with: 1, where: .!solvable)
                lambda = lambda.replacing(with: 0, where: .!solvable)
                (x0 + (w0 * lambda) * g0).scatter(x, batch.v0, batch.laneCount)
                (x1 + (w1 * lambda) * g1).scatter(x, batch.v1, batch.laneCount)
                (x2 + (w2 * lambda) * g2).scatter(x, batch.v2, batch.laneCount)
                (x3 + (w3 * lambda) * g3).scatter(x, batch.v3, batch.laneCount)
            }
        }
    }
}
