// Four independent point-to-point joints. Angular limits, springs and motors retain
// their scalar kernels; their disabled-state cached impulses still participate in warm start.

struct Matrix3W {
    var cx = Vec3W()
    var cy = Vec3W()
    var cz = Vec3W()

    @inline(__always)
    mutating func setLane(_ lane: Int, _ m: Matrix3) {
        cx[lane] = m.cx
        cy[lane] = m.cy
        cz[lane] = m.cz
    }
}

@inline(__always)
func mulMatrixW(_ m: Matrix3W, _ a: Vec3W) -> Vec3W {
    Vec3W(
        m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
        m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
        m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z)
}

@inline(__always)
func rotateW(_ q: QuatW, _ v: Vec3W) -> Vec3W {
    let t1 = crossW(q.v, v)
    let t2 = mulAddW(t1, q.s, v)
    let t3 = crossW(q.v, t2)
    return mulAddW(v, FloatW(repeating: 2), t3)
}

/// Keep every multiplication and addition in the scalar solvePointConstraint order.
@inline(__always)
func solvePointW(
    _ mA: FloatW, _ mB: FloatW, _ iA: Matrix3W, _ iB: Matrix3W,
    _ rA: Vec3W, _ rB: Vec3W, _ rhs: Vec3W
) -> Vec3W {
    let axy = rA.z * iA.cy.y - rA.y * iA.cz.y
    let axz = rA.z * iA.cy.z - rA.y * iA.cz.z
    let ay = subW(mulW(rA.x, iA.cz), mulW(rA.z, iA.cx))
    let az = subW(mulW(rA.y, iA.cx), mulW(rA.x, iA.cy))
    let bxy = rB.z * iB.cy.y - rB.y * iB.cz.y
    let bxz = rB.z * iB.cy.z - rB.y * iB.cz.z
    let by = subW(mulW(rB.x, iB.cz), mulW(rB.z, iB.cx))
    let bz = subW(mulW(rB.y, iB.cx), mulW(rB.x, iB.cy))
    let mass = mA + mB
    let kxx = (axy * rA.z - axz * rA.y) + (bxy * rB.z - bxz * rB.y) + mass
    let kyy = (ay.z * rA.x - ay.x * rA.z) + (by.z * rB.x - by.x * rB.z) + mass
    let kzz = (az.x * rA.y - az.y * rA.x) + (bz.x * rB.y - bz.y * rB.x) + mass
    let kxy = (ay.y * rA.z - ay.z * rA.y) + (by.y * rB.z - by.z * rB.y)
    let kxz = (az.y * rA.z - az.z * rA.y) + (bz.y * rB.z - bz.z * rB.y)
    let kyz = (az.z * rA.x - az.x * rA.z) + (bz.z * rB.x - bz.x * rB.z)
    let cxx = kyy * kzz - kyz * kyz
    let cyy = kxx * kzz - kxz * kxz
    let czz = kxx * kyy - kxy * kxy
    let cxy = kxz * kyz - kxy * kzz
    let cxz = kxy * kyz - kyy * kxz
    let cyz = kxy * kxz - kxx * kyz
    let determinant = kxx * cxx + kxy * cxy + kxz * cxz
    let absDeterminant = selectW(determinant .< .zero, negW(determinant), determinant)
    let valid = absDeterminant .> FloatW(repeating: 1000 * Float.leastNormalMagnitude)
    let inv = FloatW(repeating: 1) / determinant
    let value = Vec3W(
        inv * (cxx * rhs.x + cxy * rhs.y + cxz * rhs.z),
        inv * (cxy * rhs.x + cyy * rhs.y + cyz * rhs.z),
        inv * (cxz * rhs.x + cyz * rhs.y + czz * rhs.z))
    return selectW(valid, value, Vec3W())
}

struct SphericalConstraintW {
    var jointIndices = SIMD4<Int32>(repeating: -1)
    var indexA = SIMD4<Int32>(repeating: -1)
    var indexB = SIMD4<Int32>(repeating: -1)
    var laneCount = 0
    var invMassA = FloatW.zero
    var invMassB = FloatW.zero
    var invIA = Matrix3W()
    var invIB = Matrix3W()
    var anchorA = Vec3W()
    var anchorB = Vec3W()
    var deltaCenter = Vec3W()
    var biasRate = FloatW.zero
    var massScale = FloatW.zero
    var impulseScale = FloatW.zero
    var linearImpulse = Vec3W()
    var angularImpulse = Vec3W()
}

extension World {
    struct SolverJointData {
        var batches: [[SphericalConstraintW]] = Array(repeating: [], count: Int(B3.graphColorCount))
        var starts: [[Int]] = Array(repeating: [], count: Int(B3.graphColorCount))
        /// First joint of each solver unit (one scalar joint or one wide batch) when the color has
        /// wide batches. Empty means every joint is its own unit.
        var units: [[Int32]] = Array(repeating: [], count: Int(B3.graphColorCount))
    }

    @inline(__always)
    private static func isWideSpherical(_ base: UnsafePointer<JointSim>) -> Bool {
        guard base.pointee.type == .sphericalJoint else { return false }
        let j = base.pointee.sphericalJoint
        return !j.enableSpring && !j.enableMotor && !j.enableTwistLimit && !j.enableConeLimit
    }

    /// Pack runs of plain spherical joints into four-wide batches. The batch boundaries are planned
    /// on this thread; the batches are then filled in parallel. Requires `prepareViews.joints` from
    /// `prepareAllJoints`.
    func prepareWideJoints() {
        wideJointItems.removeAll(keepingCapacity: true)
        for ci in 0..<Int(B3.graphColorCount) {
            solverJoints.batches[ci].removeAll(keepingCapacity: true)
            solverJoints.starts[ci].removeAll(keepingCapacity: true)
            solverJoints.units[ci].removeAll(keepingCapacity: true)
            guard enableWideSolver, ci != Int(overflowIndex) else { continue }
            let count = constraintGraph.colors[ci].jointSims.count
            guard count >= 4 else { continue }
            solverJoints.starts[ci].append(contentsOf: repeatElement(-1, count: count))

            var batches = 0
            constraintGraph.colors[ci].jointSims.withUnsafeBufferPointer { list in
                let base = list.baseAddress!
                var i = 0
                while i < count {
                    guard Self.isWideSpherical(base + i) else {
                        i += 1
                        continue
                    }
                    let start = i
                    while i < count && Self.isWideSpherical(base + i) { i += 1 }
                    guard i - start >= 4 else { continue }
                    for first in stride(from: start, to: i, by: 4) {
                        var item = WideJointPrepareItem()
                        item.color = Int32(ci)
                        item.first = Int32(first)
                        item.laneCount = Int32(min(4, i - first))
                        item.batch = Int32(batches)
                        wideJointItems.append(item)
                        solverJoints.starts[ci][first] = batches
                        batches += 1
                    }
                }
            }

            var placeholder = SphericalConstraintW()
            solverJoints.batches[ci].reserveCapacity(batches)
            for item in wideJointItems.suffix(batches) {
                placeholder.laneCount = Int(item.laneCount)
                solverJoints.batches[ci].append(placeholder)
            }
            buildJointUnits(ci)
        }

        guard wideJointItems.isEmpty == false else { return }
        var views = prepareViews
        views.wideJointItems = wideJointItems.withUnsafeBufferPointer { $0.baseAddress }
        views.wideJointBatches.removeAll(keepingCapacity: true)
        for ci in 0..<Int(B3.graphColorCount) {
            views.wideJointBatches.append(solverJoints.batches[ci].withUnsafeMutableBufferPointer { $0.baseAddress })
        }
        prepareViews = views
        runStage(.prepareWideJoints, 0, wideJointItems.count, minimum: Self.parallelPrepareItems)
    }

    /// Fill planned wide joint batches `range`.
    func prepareWideJointRange(_ range: Range<Int>) {
        guard let items = prepareViews.wideJointItems else { return }
        for index in range {
            let item = items[index]
            let ci = Int(item.color)
            guard let list = prepareViews.joints[ci], let out = prepareViews.wideJointBatches[ci] else { continue }
            var c = SphericalConstraintW()
            c.laneCount = Int(item.laneCount)
            for lane in 0..<c.laneCount {
                let base = list + Int(item.first) + lane
                let joint = base.pointee.sphericalJoint
                c.jointIndices[lane] = item.first + Int32(lane)
                c.indexA[lane] = joint.indexA
                c.indexB[lane] = joint.indexB
                c.invMassA[lane] = base.pointee.invMassA
                c.invMassB[lane] = base.pointee.invMassB
                c.invIA.setLane(lane, base.pointee.invIA)
                c.invIB.setLane(lane, base.pointee.invIB)
                c.anchorA[lane] = joint.frameA.p
                c.anchorB[lane] = joint.frameB.p
                c.deltaCenter[lane] = joint.deltaCenter
                c.biasRate[lane] = base.pointee.constraintSoftness.biasRate
                c.massScale[lane] = base.pointee.constraintSoftness.massScale
                c.impulseScale[lane] = base.pointee.constraintSoftness.impulseScale
                c.linearImpulse[lane] = joint.linearImpulse
                var angular = joint.springImpulse + joint.motorImpulse
                angular = angular - joint.swingImpulse * joint.swingAxis
                angular = mulAdd(angular, joint.lowerTwistImpulse - joint.upperTwistImpulse, joint.twistJacobian)
                c.angularImpulse[lane] = angular
            }
            out[Int(item.batch)] = c
        }
    }

    private func buildJointUnits(_ ci: Int) {
        let starts = solverJoints.starts[ci]
        guard starts.isEmpty == false else { return }
        var i = 0
        while i < starts.count {
            solverJoints.units[ci].append(Int32(i))
            i += starts[i] >= 0 ? solverJoints.batches[ci][starts[i]].laneCount : 1
        }
    }

    func warmStartSphericalW(_ c: SphericalConstraintW, _ states: UnsafeMutableBufferPointer<BodyState>) {
        var a = Self.gatherStatesW(states, c.indexA)
        var b = Self.gatherStatesW(states, c.indexB)
        let rA = rotateW(a.dq, c.anchorA)
        let rB = rotateW(b.dq, c.anchorB)
        a.v = subW(a.v, mulW(c.invMassA, c.linearImpulse))
        a.w = subW(a.w, mulMatrixW(c.invIA, addW(crossW(rA, c.linearImpulse), c.angularImpulse)))
        b.v = addW(b.v, mulW(c.invMassB, c.linearImpulse))
        b.w = addW(b.w, mulMatrixW(c.invIB, addW(crossW(rB, c.linearImpulse), c.angularImpulse)))
        Self.scatterStatesW(states, c.indexA, a)
        Self.scatterStatesW(states, c.indexB, b)
    }

    func solveSphericalW(
        _ c: inout SphericalConstraintW, _ states: UnsafeMutableBufferPointer<BodyState>, _ useBias: Bool
    ) {
        var a = Self.gatherStatesW(states, c.indexA)
        var b = Self.gatherStatesW(states, c.indexB)
        let rA = rotateW(a.dq, c.anchorA)
        let rB = rotateW(b.dq, c.anchorB)
        let cdot = subW(subW(addW(b.v, crossW(b.w, rB)), a.v), crossW(a.w, rA))
        var bias = Vec3W()
        var mass = FloatW(repeating: 1)
        var scale = FloatW.zero
        if useBias {
            let separation = addW(addW(subW(b.dp, a.dp), subW(rB, rA)), c.deltaCenter)
            bias = mulW(c.biasRate, separation)
            mass = c.massScale
            scale = c.impulseScale
        }
        let solution = solvePointW(c.invMassA, c.invMassB, c.invIA, c.invIB, rA, rB, addW(cdot, bias))
        let impulse = subW(mulW(negW(mass), solution), mulW(scale, c.linearImpulse))
        c.linearImpulse = addW(c.linearImpulse, impulse)
        a.v = subW(a.v, mulW(c.invMassA, impulse))
        a.w = subW(a.w, mulMatrixW(c.invIA, crossW(rA, impulse)))
        b.v = addW(b.v, mulW(c.invMassB, impulse))
        b.w = addW(b.w, mulMatrixW(c.invIB, crossW(rB, impulse)))
        Self.scatterStatesW(states, c.indexA, a)
        Self.scatterStatesW(states, c.indexB, b)
    }

    func storeWideJointImpulses() {
        guard enableWideSolver else { return }
        for ci in 0..<Int(overflowIndex) {
            let batches = solverJoints.batches[ci]
            guard !batches.isEmpty else { continue }
            constraintGraph.colors[ci].jointSims.withUnsafeMutableBufferPointer { list in
                for c in batches {
                    for lane in 0..<c.laneCount {
                        list[Int(c.jointIndices[lane])].spherical.linearImpulse = c.linearImpulse[lane]
                    }
                }
            }
        }
    }
}
