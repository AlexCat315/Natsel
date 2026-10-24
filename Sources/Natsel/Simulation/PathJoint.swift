// Path joint (Jolt's PathConstraint). A point on body B slides along a Hermite spline fixed in body
// A. Perpendicular to the path the point is held rigidly; along it the joint can be free, carry
// friction, a velocity motor, or a position motor that drives the point to a target fraction. At the
// ends of an open path the point stops. The rotation of body B can be free, hinged about one of the
// path axes, aligned with the path frame, or fixed relative to body A.
//
// The path frame at a fraction is (tangent, normal, binormal) = (x, y, z). The path lives in body A's
// `localFrameA`; the attachment point on body B is `localFrameB.p`. Rotation constraints take the
// current orientation of body B relative to the path frame as their reference.

public struct PathPoint: Sendable, Equatable {
    public var position: Vec3
    /// Hermite tangent. Its length sets how far the curve leaves the point in this direction.
    public var tangent: Vec3
    /// Up direction of the path frame at this point; made perpendicular to the tangent.
    public var normal: Vec3

    public init(position: Vec3, tangent: Vec3, normal: Vec3) {
        self.position = position
        self.tangent = tangent
        self.normal = normal
    }
}

/// One cubic segment, p(t) = a + t (b + t (c + t d)) for t in [0, 1].
struct PathSegment: BitwiseCopyable {
    var a = Vec3()
    var b = Vec3()
    var c = Vec3()
    var d = Vec3()
    var normal0 = Vec3()
    var normal1 = Vec3()
    /// Bounds of the Bezier control polygon, which contains the curve.
    var lower = Vec3()
    var upper = Vec3()
    /// Arc length before this segment, and of this segment.
    var startLength: Float = 0
    var segmentLength: Float = 0

    @inline(__always) func position(_ t: Float) -> Vec3 { a + t * (b + t * (c + t * d)) }
    @inline(__always) func derivative(_ t: Float) -> Vec3 { b + t * (2 * c + (3 * t) * d) }
    @inline(__always) func secondDerivative(_ t: Float) -> Vec3 { 2 * c + (6 * t) * d }

    /// Arc length from 0 to t, five point Gauss-Legendre.
    func arcLength(_ t: Float) -> Float {
        let nodes: (Float, Float, Float, Float, Float) = (0, -0.5384693101, 0.5384693101, -0.9061798459, 0.9061798459)
        let weights: (Float, Float, Float, Float, Float) = (0.5688888889, 0.4786286705, 0.4786286705, 0.2369268851, 0.2369268851)
        let half = 0.5 * t
        var sum = weights.0 * length(derivative(half + half * nodes.0))
        sum += weights.1 * length(derivative(half + half * nodes.1))
        sum += weights.2 * length(derivative(half + half * nodes.2))
        sum += weights.3 * length(derivative(half + half * nodes.3))
        sum += weights.4 * length(derivative(half + half * nodes.4))
        return half * sum
    }

    @inline(__always) func boundsDistanceSquared(_ p: Vec3) -> Float {
        let dx = maxFloat(maxFloat(lower.x - p.x, p.x - upper.x), 0)
        let dy = maxFloat(maxFloat(lower.y - p.y, p.y - upper.y), 0)
        let dz = maxFloat(maxFloat(lower.z - p.z, p.z - upper.z), 0)
        return dx * dx + dy * dy + dz * dz
    }

    /// Closest parameter to `p`: the best of nine samples, refined by Newton steps on the squared
    /// distance inside the neighboring sample intervals.
    func closest(_ p: Vec3) -> (t: Float, distanceSquared: Float) {
        let sampleCount = 8
        var bestT: Float = 0
        var best = distanceSquared(position(0), p)
        for k in 1...sampleCount {
            let t = Float(k) / Float(sampleCount)
            let d = distanceSquared(position(t), p)
            if d < best {
                best = d
                bestT = t
            }
        }
        let lowerT = maxFloat(bestT - 1 / Float(sampleCount), 0)
        let upperT = minFloat(bestT + 1 / Float(sampleCount), 1)
        var t = bestT
        for _ in 0..<4 {
            let e = position(t) - p
            let d1 = derivative(t)
            let g = dot(e, d1)
            let hessian = dot(d1, d1) + dot(e, secondDerivative(t))
            guard hessian > 1.0e-12 else { break }
            t = clampFloat(t - g / hessian, lowerT, upperT)
        }
        let refined = distanceSquared(position(t), p)
        return refined < best ? (t, refined) : (bestT, best)
    }
}

/// Plain path data the solver reads on any thread. Points into a `ConstraintPath`'s storage, which
/// the world keeps alive while a joint uses it.
struct PathGeometry: BitwiseCopyable {
    var segments: UnsafePointer<PathSegment>?
    var count: Int32 = 0
    var looping = false
    var totalLength: Float = 0

    init() {}

    var maxFraction: Float { Float(count) }

    /// Wrap (looping) or clamp (open) a fraction into [0, maxFraction].
    @inline(__always) func wrapFraction(_ fraction: Float) -> Float {
        let maxF = maxFraction
        if looping {
            var f = fraction.truncatingRemainder(dividingBy: maxF)
            if f < 0 { f += maxF }
            return f < maxF ? f : 0
        }
        return clampFloat(fraction, 0, maxF)
    }

    @inline(__always) func locate(_ fraction: Float) -> (Int, Float) {
        let f = wrapFraction(fraction)
        let index = min(Int(f), Int(count) - 1)
        return (index, f - Float(index))
    }

    /// Position and orthonormal frame (tangent, normal, binormal) at a fraction.
    func evaluate(_ fraction: Float) -> (position: Vec3, tangent: Vec3, normal: Vec3, binormal: Vec3) {
        guard let segments, count > 0 else { return (Vec3(), .axisX, .axisY, .axisZ) }
        let (index, t) = locate(fraction)
        let segment = segments[index]
        let position = segment.position(t)
        var tangent = segment.derivative(t)
        var len: Float = 0
        tangent = getLengthAndNormalize(&len, tangent)
        if len < 1.0e-9 {
            // Zero tangent at a cusp: use the chord of the segment.
            tangent = normalize(segment.position(1) - segment.position(0))
            if lengthSquared(tangent) == 0 { tangent = .axisX }
        }
        let up = lerp(segment.normal0, segment.normal1, t)
        var binormal = getLengthAndNormalize(&len, cross(tangent, up))
        if len < 1.0e-6 {
            binormal = normalize(cross(tangent, perpendicular(tangent)))
        }
        let normal = cross(binormal, tangent)
        return (position, tangent, normal, binormal)
    }

    /// Arc length from the start of the path to a fraction.
    func arcLength(_ fraction: Float) -> Float {
        guard let segments, count > 0 else { return 0 }
        let (index, t) = locate(fraction)
        return segments[index].startLength + segments[index].arcLength(t)
    }

    /// The fraction of the closest point on the path. Starts at the hint's segment so the bounds of
    /// most other segments reject them, and keeps the hint's segment on ties.
    func closestFraction(_ p: Vec3, _ hint: Float) -> Float {
        guard let segments, count > 0 else { return 0 }
        let n = Int(count)
        let hintIndex = locate(hint).0
        var (bestT, best) = segments[hintIndex].closest(p)
        var bestIndex = hintIndex
        for k in 1..<max(n, 1) {
            var index = hintIndex + k
            if index >= n { index -= n }
            let segment = segments[index]
            guard segment.boundsDistanceSquared(p) < best else { continue }
            let (t, d) = segment.closest(p)
            if d < best {
                best = d
                bestT = t
                bestIndex = index
            }
        }
        return wrapFraction(Float(bestIndex) + bestT)
    }
}

@inline(__always)
private func perpendicular(_ v: Vec3) -> Vec3 {
    absFloat(v.x) < 0.57735 ? Vec3(0, v.z, -v.y) : Vec3(v.y, -v.x, 0)
}

/// A cubic Hermite path (Jolt's PathConstraintPathHermite). Fractions run from 0 at the first point
/// to `maxFraction`: one unit per segment. A looping path joins the last point back to the first.
public final class ConstraintPath: @unchecked Sendable {
    public let points: [PathPoint]
    public let isLooping: Bool
    private let storage: UnsafeMutablePointer<PathSegment>
    let geometry: PathGeometry

    /// Needs two points, or three for a loop.
    public init?(points: [PathPoint], isLooping: Bool = false) {
        guard points.count >= (isLooping ? 3 : 2),
            points.allSatisfy({ isValidVec3($0.position) && isValidVec3($0.tangent) && isValidVec3($0.normal) })
        else { return nil }
        self.points = points
        self.isLooping = isLooping
        let count = isLooping ? points.count : points.count - 1
        storage = .allocate(capacity: count)
        var startLength: Float = 0
        for i in 0..<count {
            let p0 = points[i]
            let p1 = points[(i + 1) % points.count]
            var segment = PathSegment()
            segment.a = p0.position
            segment.b = p0.tangent
            segment.c = 3 * (p1.position - p0.position) - 2 * p0.tangent - p1.tangent
            segment.d = 2 * (p0.position - p1.position) + p0.tangent + p1.tangent
            segment.normal0 = p0.normal
            segment.normal1 = p1.normal
            let control1 = p0.position + (1.0 / 3.0) * p0.tangent
            let control2 = p1.position - (1.0 / 3.0) * p1.tangent
            segment.lower = minVec3(minVec3(p0.position, p1.position), minVec3(control1, control2))
            segment.upper = maxVec3(maxVec3(p0.position, p1.position), maxVec3(control1, control2))
            segment.startLength = startLength
            segment.segmentLength = segment.arcLength(1)
            startLength += segment.segmentLength
            storage.advanced(by: i).initialize(to: segment)
        }
        var geometry = PathGeometry()
        geometry.segments = UnsafePointer(storage)
        geometry.count = Int32(count)
        geometry.looping = isLooping
        geometry.totalLength = startLength
        self.geometry = geometry
    }

    /// A smooth path through `positions` with Catmull-Rom tangents and a constant up direction.
    public convenience init?(through positions: [Vec3], up: Vec3 = .axisY, isLooping: Bool = false) {
        let n = positions.count
        guard n >= 2 else { return nil }
        var points: [PathPoint] = []
        points.reserveCapacity(n)
        for i in 0..<n {
            let previous = isLooping ? positions[(i + n - 1) % n] : positions[max(i - 1, 0)]
            let next = isLooping ? positions[(i + 1) % n] : positions[min(i + 1, n - 1)]
            let scale: Float = isLooping || (i > 0 && i < n - 1) ? 0.5 : 1
            points.append(PathPoint(position: positions[i], tangent: scale * (next - previous), normal: up))
        }
        self.init(points: points, isLooping: isLooping)
    }

    deinit {
        storage.deinitialize(count: Int(geometry.count))
        storage.deallocate()
    }

    /// Largest fraction; the number of segments.
    public var maxFraction: Float { geometry.maxFraction }

    /// Total arc length.
    public var length: Float { geometry.totalLength }

    /// Position and frame at a fraction, in the path's space.
    public func point(at fraction: Float) -> (position: Vec3, tangent: Vec3, normal: Vec3, binormal: Vec3) {
        geometry.evaluate(fraction)
    }

    /// Fraction of the closest point on the path to `position`.
    public func closestFraction(to position: Vec3, hint: Float = 0) -> Float {
        geometry.closestFraction(position, hint)
    }

    /// Arc length from the start to a fraction.
    public func arcLength(at fraction: Float) -> Float { geometry.arcLength(fraction) }
}

public enum PathRotationConstraint: Sendable, BitwiseCopyable {
    /// Body B rotates freely.
    case free
    /// Body B only rotates about the path tangent.
    case constrainAroundTangent
    /// Body B only rotates about the path normal.
    case constrainAroundNormal
    /// Body B only rotates about the path binormal.
    case constrainAroundBinormal
    /// Body B follows the path frame, like a roller coaster car.
    case constrainToPath
    /// The rotation of body B relative to body A is fixed.
    case fullyConstrained
}

public enum PathMotorMode: Sendable, BitwiseCopyable {
    case off
    /// Drive the speed along the path to `targetVelocity` (m/s, positive toward larger fractions).
    case velocity
    /// Spring the point toward `targetFraction`, measured by arc length.
    case position
}

public struct PathJointDef {
    /// `localFrameA` places the path in body A; `localFrameB.p` is the point on body B that follows
    /// it. The rotation of `localFrameB` is ignored: the current orientation is the reference.
    public var base = JointDef()
    public var path: ConstraintPath?
    /// Start hint for the closest point search.
    public var pathFraction: Float = 0
    public var rotationConstraint = PathRotationConstraint.free
    /// Force (N) resisting motion along the path while the motor is off.
    public var maxFrictionForce: Float = 0
    public var motorMode = PathMotorMode.off
    public var targetVelocity: Float = 0
    public var targetFraction: Float = 0
    /// Largest motor force (N).
    public var maxMotorForce: Float = .greatestFiniteMagnitude
    /// Spring of the position motor.
    public var motorHertz: Float = 2
    public var motorDampingRatio: Float = 1

    public init() {}
}

/// Solver data kept in the joint record.
struct PathJointSim: BitwiseCopyable {
    var slot: Int32 = B3.nullIndex
    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var frameA = Transform()
    var frameB = Transform()
    var deltaCenter = Vec3()

    init() {}
}

struct PathJointData: BitwiseCopyable {
    var geometry = PathGeometry()
    var rotation = PathRotationConstraint.free
    /// Fraction of the closest point, updated by every solve.
    var fraction: Float = 0
    /// Path frame at the reference pose, relative to the path space (fully constrained).
    var targetRotation = Quat.identity

    var motorMode = PathMotorMode.off
    var targetVelocity: Float = 0
    var targetFraction: Float = 0
    var maxMotorForce: Float = 0
    var motorHertz: Float = 2
    var motorDampingRatio: Float = 1
    var maxFriction: Float = 0
    var motorSoftness = Softness()

    var perpendicularImpulse = Vec2()
    var lowerImpulse: Float = 0
    var upperImpulse: Float = 0
    var motorImpulse: Float = 0
    var frictionImpulse: Float = 0
    var angularImpulse = Vec3()

    /// World impulses of the last solve, for reactions and events.
    var linearImpulseWorld = Vec3()
    var angularImpulseWorld = Vec3()

    init() {}
}

/// Frame of a path joint at the current substep.
private struct PathFrame {
    var quatA: Quat
    var quatB: Quat
    /// Path point relative to body A's center, and attachment point relative to body B's center.
    var rA: Vec3
    var rB: Vec3
    /// From the path point to the attachment point.
    var u: Vec3
    var tangent: Vec3
    var normal: Vec3
    var binormal: Vec3
    var fraction: Float
}

@inline(__always)
private func pathAxis(_ f: PathFrame, _ i: Int) -> Vec3 { i == 0 ? f.tangent : (i == 1 ? f.normal : f.binormal) }

extension World {
    // MARK: Storage

    private func allocatePathData(_ data: PathJointData, _ path: ConstraintPath) -> Int32 {
        let slot = pathJointPool.alloc()
        if Int(slot) == pathJointStore.count {
            pathJointStore.append(data)
            pathJointPaths.append(path)
        } else {
            pathJointStore[Int(slot)] = data
            pathJointPaths[Int(slot)] = path
        }
        return slot
    }

    func releasePathData(_ slot: Int32) {
        guard slot >= 0 && Int(slot) < pathJointStore.count else { return }
        pathJointStore[Int(slot)] = PathJointData()
        pathJointPaths[Int(slot)] = nil
        pathJointPool.free(slot)
    }

    /// World transform of the path and the attachment point on body B.
    private func pathPose(_ bodyA: Int32, _ bodyB: Int32, _ frameA: Transform, _ anchorB: Vec3) -> (WorldTransform, Pos) {
        let pathTransform = mulWorldTransforms(bodyTransformQuick(bodyA), frameA)
        return (pathTransform, transformWorldPoint(bodyTransformQuick(bodyB), anchorB))
    }

    // MARK: Creation

    @discardableResult
    public func createPathJoint(_ def: PathJointDef) -> JointID {
        guard let path = def.path, isValidBody(def.base.bodyIdA), isValidBody(def.base.bodyIdB),
            def.base.bodyIdA != def.base.bodyIdB
        else { return JointID() }

        var base = def.base
        let bodyA = base.bodyIdA.index1 - 1
        let bodyB = base.bodyIdB.index1 - 1
        let (pathTransform, anchor) = pathPose(bodyA, bodyB, base.localFrameA, base.localFrameB.p)
        let local = invTransformWorldPoint(pathTransform, anchor)
        let fraction = path.geometry.closestFraction(local, def.pathFraction)
        let frame = path.geometry.evaluate(fraction)
        let localPathQ = makeQuatFromMatrix(Matrix3(frame.tangent, frame.normal, frame.binormal))
        let worldPathQ = mulQuat(pathTransform.q, localPathQ)
        // Body B's frame is the path frame at creation, so every rotation constraint starts satisfied.
        base.localFrameB.q = normalizeQuat(invMulQuat(bodyTransformQuick(bodyB).q, worldPathQ))

        var data = PathJointData()
        data.geometry = path.geometry
        data.rotation = def.rotationConstraint
        data.fraction = fraction
        data.targetRotation = localPathQ
        data.motorMode = def.motorMode
        data.targetVelocity = def.targetVelocity
        data.targetFraction = def.targetFraction
        data.maxMotorForce = maxFloat(def.maxMotorForce, 0)
        data.motorHertz = def.motorHertz
        data.motorDampingRatio = def.motorDampingRatio
        data.maxFriction = maxFloat(def.maxFrictionForce, 0)

        let pair = createJoint(base, .pathJoint)
        var sim = pair.sim
        var joint = PathJointSim()
        joint.slot = allocatePathData(data, path)
        sim.path = joint
        writeJointSim(pair.jointId, sim)
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: joints[Int(pair.jointId)].generation)
    }

    // MARK: Prepare

    func preparePathJoint(_ base: inout JointSim, _ context: StepContext) {
        let bodyA = jointPrepareBody(base.bodyIdA)
        let bodyB = jointPrepareBody(base.bodyIdB)
        let simA = jointPrepareSim(bodyA)
        let simB = jointPrepareSim(bodyB)
        base.invMassA = simA.invMass
        base.invMassB = simB.invMass
        base.invIA = simA.invInertiaWorld
        base.invIB = simB.invInertiaWorld

        var joint = base.path
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex
        joint.frameA.q = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.frameA.p = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.frameB.q = mulQuat(simB.transform.q, base.localFrameB.q)
        joint.frameB.p = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)
        joint.deltaCenter = subPos(simB.center, simA.center)
        base.path = joint

        let slot = Int(joint.slot)
        var data = pathJointStore[slot]
        data.motorSoftness = makeSoft(data.motorHertz, data.motorDampingRatio, context.h)
        if context.enableWarmStarting == false {
            data.perpendicularImpulse = Vec2()
            data.lowerImpulse = 0
            data.upperImpulse = 0
            data.motorImpulse = 0
            data.frictionImpulse = 0
            data.angularImpulse = Vec3()
        }
        if data.motorMode == .off { data.motorImpulse = 0 }
        if data.motorMode != .off || data.maxFriction <= 0 { data.frictionImpulse = 0 }
        if data.rotation == .free { data.angularImpulse = Vec3() }
        pathJointStore[slot] = data
    }

    // MARK: Geometry

    @inline(__always)
    private func pathFrame(
        _ joint: PathJointSim, _ data: PathJointData, _ stateA: BodyState, _ stateB: BodyState, search: Bool
    ) -> PathFrame {
        let quatA = mulQuat(stateA.deltaRotation, joint.frameA.q)
        let quatB = mulQuat(stateB.deltaRotation, joint.frameB.q)
        let origin = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)
        // Attachment point relative to the path origin.
        let d = ((stateB.deltaPosition - stateA.deltaPosition) + joint.deltaCenter) + (rB - origin)
        let local = invRotateVector(quatA, d)
        let fraction = search ? data.geometry.closestFraction(local, data.fraction) : data.fraction
        let e = data.geometry.evaluate(fraction)
        let p = rotateVector(quatA, e.position)
        return PathFrame(
            quatA: quatA, quatB: quatB, rA: origin + p, rB: rB, u: d - p,
            tangent: rotateVector(quatA, e.tangent), normal: rotateVector(quatA, e.normal),
            binormal: rotateVector(quatA, e.binormal), fraction: fraction)
    }

    /// Angular Jacobians of the rotation constraint, and how many there are.
    @inline(__always)
    private func pathRotationRows(_ f: PathFrame, _ data: PathJointData) -> (Vec3, Vec3, Vec3, Vec3, Int) {
        switch data.rotation {
        case .free:
            return (Vec3(), Vec3(), Vec3(), Vec3(), 0)
        case .constrainAroundTangent, .constrainAroundNormal, .constrainAroundBinormal:
            // Keep body B's copy of the hinge axis perpendicular to the other two path axes.
            let k = data.rotation == .constrainAroundTangent ? 0 : (data.rotation == .constrainAroundNormal ? 1 : 2)
            let axesB = makeMatrixFromQuat(f.quatB)
            let hingeB = k == 0 ? axesB.cx : (k == 1 ? axesB.cy : axesB.cz)
            let p1 = pathAxis(f, (k + 1) % 3)
            let p2 = pathAxis(f, (k + 2) % 3)
            return (cross(hingeB, p1), cross(hingeB, p2), Vec3(), Vec3(dot(hingeB, p1), dot(hingeB, p2), 0), 2)
        case .constrainToPath, .fullyConstrained:
            let target = data.rotation == .constrainToPath
                ? makeQuatFromMatrix(Matrix3(f.tangent, f.normal, f.binormal))
                : mulQuat(f.quatA, data.targetRotation)
            var rel = invMulQuat(target, f.quatB)
            if rel.s < 0 { rel = negateQuat(rel) }
            return (quaternionJacobian(target, rel, .axisX), quaternionJacobian(target, rel, .axisY),
                    quaternionJacobian(target, rel, .axisZ), rel.v, 3)
        }
    }

    /// Signed distance along the path from the target to the current fraction.
    @inline(__always)
    private func pathMotorError(_ data: PathJointData, _ fraction: Float) -> Float {
        var error = data.geometry.arcLength(fraction) - data.geometry.arcLength(data.targetFraction)
        if data.geometry.looping {
            let total = data.geometry.totalLength
            if error > 0.5 * total { error -= total } else if error < -0.5 * total { error += total }
        }
        return error
    }

    // MARK: Warm start

    func warmStartPathJoint(_ base: inout JointSim, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let joint = base.path
        let data = pathJointStore[Int(joint.slot)]
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        let f = pathFrame(joint, data, stateA, stateB, search: false)

        let axial = data.lowerImpulse - data.upperImpulse + data.motorImpulse + data.frictionImpulse
        let linear = data.perpendicularImpulse.x * f.normal + data.perpendicularImpulse.y * f.binormal + axial * f.tangent
        let (j0, j1, j2, _, rows) = pathRotationRows(f, data)
        var angular = Vec3()
        if rows > 0 {
            angular = data.angularImpulse.x * j0 + data.angularImpulse.y * j1 + data.angularImpulse.z * j2
        }

        stateA.linearVelocity = mulSub(stateA.linearVelocity, base.invMassA, linear)
        stateA.angularVelocity = stateA.angularVelocity - mulMV(base.invIA, cross(f.rA + f.u, linear) + angular)
        stateB.linearVelocity = mulAdd(stateB.linearVelocity, base.invMassB, linear)
        stateB.angularVelocity = stateB.angularVelocity + mulMV(base.invIB, cross(f.rB, linear) + angular)
        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
    }

    // MARK: Solve

    func solvePathJoint(
        _ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>
    ) {
        let joint = base.path
        let slot = Int(joint.slot)
        var data = pathJointStore[slot]
        guard data.geometry.count > 0 else { return }
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB
        let h = context.h
        let soft = base.constraintSoftness

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let f = pathFrame(joint, data, stateA, stateB, search: true)
        data.fraction = f.fraction
        let lever = f.rA + f.u
        var angularTotal = Vec3()
        var linearTotal = Vec3()

        // MARK: Rotation

        let (j0, j1, j2, c, rows) = pathRotationRows(f, data)
        if rows > 0 {
            let iSum = addMM(iA, iB)
            var bias = Vec3()
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias {
                bias = soft.biasRate * c
                massScale = soft.massScale
                impulseScale = soft.impulseScale
            }
            let wRel = wB - wA
            var delta = Vec3()
            if rows == 2 {
                let g0 = mulMV(iSum, j0)
                let g1 = mulMV(iSum, j1)
                let k = Matrix2(Vec2(dot(j0, g0), dot(j1, g0)), Vec2(dot(j0, g1), dot(j1, g1)))
                let rhs = Vec2(dot(wRel, j0) + bias.x, dot(wRel, j1) + bias.y)
                let solution = solve2(k, rhs)
                delta.x = -massScale * solution.x - impulseScale * data.angularImpulse.x
                delta.y = -massScale * solution.y - impulseScale * data.angularImpulse.y
            } else {
                let g0 = mulMV(iSum, j0)
                let g1 = mulMV(iSum, j1)
                let g2 = mulMV(iSum, j2)
                let k = Matrix3(Vec3(dot(j0, g0), dot(j1, g0), dot(j2, g0)),
                                Vec3(dot(j0, g1), dot(j1, g1), dot(j2, g1)),
                                Vec3(dot(j0, g2), dot(j1, g2), dot(j2, g2)))
                let cdot = Vec3(dot(wRel, j0), dot(wRel, j1), dot(wRel, j2))
                delta = (-massScale) * solve3(k, cdot + bias) - impulseScale * data.angularImpulse
            }
            data.angularImpulse = data.angularImpulse + delta
            let angular = delta.x * j0 + delta.y * j1 + delta.z * j2
            wA = wA - mulMV(iA, angular)
            wB = wB + mulMV(iB, angular)
            angularTotal = data.angularImpulse.x * j0 + data.angularImpulse.y * j1 + data.angularImpulse.z * j2
        } else {
            data.angularImpulse = Vec3()
        }

        // MARK: Along the path

        let t = f.tangent
        let sAt = cross(lever, t)
        let sBt = cross(f.rB, t)
        let kt = mA + mB + dot(sAt, mulMV(iA, sAt)) + dot(sBt, mulMV(iB, sBt))
        if kt > 0 {
            let mass = 1 / kt
            func cdot() -> Float { dot(vB - vA, t) + dot(wB, sBt) - dot(wA, sAt) }
            func apply(_ impulse: Float) {
                vA = mulSub(vA, mA * impulse, t)
                wA = wA - mulMV(iA, impulse * sAt)
                vB = mulAdd(vB, mB * impulse, t)
                wB = wB + mulMV(iB, impulse * sBt)
            }

            switch data.motorMode {
            case .velocity:
                let maxImpulse = data.maxMotorForce * h
                let old = data.motorImpulse
                data.motorImpulse = clampFloat(old - mass * (cdot() - data.targetVelocity), -maxImpulse, maxImpulse)
                apply(data.motorImpulse - old)
            case .position:
                let ms = data.motorSoftness
                let error = pathMotorError(data, f.fraction) + dot(f.u, t)
                let maxImpulse = data.maxMotorForce * h
                let old = data.motorImpulse
                let delta = -ms.massScale * mass * (cdot() + ms.biasRate * error) - ms.impulseScale * old
                data.motorImpulse = clampFloat(old + delta, -maxImpulse, maxImpulse)
                apply(data.motorImpulse - old)
            case .off:
                if data.maxFriction > 0 {
                    let maxImpulse = data.maxFriction * h
                    let old = data.frictionImpulse
                    data.frictionImpulse = clampFloat(old - mass * cdot(), -maxImpulse, maxImpulse)
                    apply(data.frictionImpulse - old)
                }
            }

            // End stops of an open path: the point may not pass the first or last point.
            let maxF = data.geometry.maxFraction
            let atStart = data.geometry.looping == false && f.fraction <= 0
            let atEnd = data.geometry.looping == false && f.fraction >= maxF
            func stopTerms(_ separation: Float) -> (Float, Float, Float) {
                if separation > 0 { return (separation * context.inv_h, 1, 0) }
                if useBias { return (soft.biasRate * separation, soft.massScale, soft.impulseScale) }
                return (0, 1, 0)
            }
            if atStart {
                let (bias, massScale, impulseScale) = stopTerms(dot(f.u, t))
                let old = data.lowerImpulse
                let delta = maxFloat(-massScale * mass * (cdot() + bias) - impulseScale * old, -old)
                data.lowerImpulse = old + delta
                apply(delta)
            } else {
                data.lowerImpulse = 0
            }
            if atEnd {
                let (bias, massScale, impulseScale) = stopTerms(-dot(f.u, t))
                let old = data.upperImpulse
                let delta = maxFloat(-massScale * mass * (-cdot() + bias) - impulseScale * old, -old)
                data.upperImpulse = old + delta
                apply(-delta)
            } else {
                data.upperImpulse = 0
            }
        }

        // MARK: Perpendicular to the path

        do {
            let n = f.normal
            let b = f.binormal
            let sAn = cross(lever, n)
            let sAb = cross(lever, b)
            let sBn = cross(f.rB, n)
            let sBb = cross(f.rB, b)
            let gAn = mulMV(iA, sAn)
            let gBn = mulMV(iB, sBn)
            let gAb = mulMV(iA, sAb)
            let gBb = mulMV(iB, sBb)
            let knn = mA + mB + dot(sAn, gAn) + dot(sBn, gBn)
            let kbb = mA + mB + dot(sAb, gAb) + dot(sBb, gBb)
            let knb = dot(sAn, gAb) + dot(sBn, gBb)
            let k = Matrix2(Vec2(knn, knb), Vec2(knb, kbb))
            let cdot = Vec2(
                dot(vB - vA, n) + dot(wB, sBn) - dot(wA, sAn),
                dot(vB - vA, b) + dot(wB, sBb) - dot(wA, sAb))
            var bias = Vec2()
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias {
                bias = Vec2(soft.biasRate * dot(f.u, n), soft.biasRate * dot(f.u, b))
                massScale = soft.massScale
                impulseScale = soft.impulseScale
            }
            let solution = solve2(k, Vec2(cdot.x + bias.x, cdot.y + bias.y))
            let delta = Vec2(
                -massScale * solution.x - impulseScale * data.perpendicularImpulse.x,
                -massScale * solution.y - impulseScale * data.perpendicularImpulse.y)
            data.perpendicularImpulse = Vec2(data.perpendicularImpulse.x + delta.x, data.perpendicularImpulse.y + delta.y)
            let impulse = delta.x * n + delta.y * b
            vA = mulSub(vA, mA, impulse)
            wA = wA - mulMV(iA, delta.x * sAn + delta.y * sAb)
            vB = mulAdd(vB, mB, impulse)
            wB = wB + mulMV(iB, delta.x * sBn + delta.y * sBb)
        }

        let axial = data.lowerImpulse - data.upperImpulse + data.motorImpulse + data.frictionImpulse
        linearTotal = data.perpendicularImpulse.x * f.normal + data.perpendicularImpulse.y * f.binormal + axial * t
        data.linearImpulseWorld = linearTotal
        data.angularImpulseWorld = angularTotal

        if (stateA.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateA.linearVelocity = vA
            stateA.angularVelocity = wA
        }
        if (stateB.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateB.linearVelocity = vB
            stateB.angularVelocity = wB
        }
        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
        pathJointStore[slot] = data
    }

    // MARK: Reactions

    func pathJointImpulses(_ sim: JointSim) -> (linear: Vec3, angular: Vec3) {
        let slot = Int(sim.path.slot)
        guard slot >= 0 && slot < pathJointStore.count else { return (Vec3(), Vec3()) }
        return (pathJointStore[slot].linearImpulseWorld, pathJointStore[slot].angularImpulseWorld)
    }

    // MARK: Accessors

    private func pathSlot(_ jointId: JointID) -> Int? {
        let index = jointId.index1 - 1
        guard jointId.world0 == worldId, index >= 0, Int(index) < joints.count else { return nil }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.generation == jointId.generation, joint.type == .pathJoint
        else { return nil }
        let slot = Int(readJointSim(index).path.slot)
        return slot >= 0 && slot < pathJointStore.count ? slot : nil
    }

    /// Fraction of the closest point on the path after the last step.
    public func pathJointFraction(_ jointId: JointID) -> Float? {
        pathSlot(jointId).map { pathJointStore[$0].fraction }
    }

    public func pathJointPath(_ jointId: JointID) -> ConstraintPath? {
        pathSlot(jointId).flatMap { pathJointPaths[$0] }
    }

    /// Replace the path. `fraction` is the search hint for the new path.
    @discardableResult
    public func setPathJointPath(_ jointId: JointID, _ path: ConstraintPath, fraction: Float = 0) -> Bool {
        guard let slot = pathSlot(jointId) else { return false }
        pathJointPaths[slot] = path
        pathJointStore[slot].geometry = path.geometry
        pathJointStore[slot].fraction = path.geometry.wrapFraction(fraction)
        pathJointStore[slot].lowerImpulse = 0
        pathJointStore[slot].upperImpulse = 0
        wakeJointBodies(jointId)
        return true
    }

    @discardableResult
    public func setPathJointMotor(_ jointId: JointID, _ mode: PathMotorMode, maxForce: Float? = nil) -> Bool {
        guard let slot = pathSlot(jointId) else { return false }
        pathJointStore[slot].motorMode = mode
        if let maxForce { pathJointStore[slot].maxMotorForce = maxFloat(maxForce, 0) }
        wakeJointBodies(jointId)
        return true
    }

    @discardableResult
    public func setPathJointTargetVelocity(_ jointId: JointID, _ velocity: Float) -> Bool {
        guard let slot = pathSlot(jointId) else { return false }
        pathJointStore[slot].targetVelocity = velocity
        wakeJointBodies(jointId)
        return true
    }

    @discardableResult
    public func setPathJointTargetFraction(_ jointId: JointID, _ fraction: Float) -> Bool {
        guard let slot = pathSlot(jointId) else { return false }
        pathJointStore[slot].targetFraction = pathJointStore[slot].geometry.wrapFraction(fraction)
        wakeJointBodies(jointId)
        return true
    }

    @discardableResult
    public func setPathJointMaxFriction(_ jointId: JointID, _ force: Float) -> Bool {
        guard let slot = pathSlot(jointId) else { return false }
        pathJointStore[slot].maxFriction = maxFloat(force, 0)
        wakeJointBodies(jointId)
        return true
    }

    @discardableResult
    public func setPathJointRotationConstraint(_ jointId: JointID, _ rotation: PathRotationConstraint) -> Bool {
        guard let slot = pathSlot(jointId) else { return false }
        pathJointStore[slot].rotation = rotation
        pathJointStore[slot].angularImpulse = Vec3()
        wakeJointBodies(jointId)
        return true
    }

    // MARK: Debug draw

    func drawPathJoint(
        _ sim: JointSim, _ xfA: WorldTransform, _ pB: Pos, _ scale: Float, _ drawSegment: (Pos, Pos, HexColor) -> Void,
        _ drawPoint: ((Pos, Float, HexColor) -> Void)?
    ) {
        let slot = Int(sim.path.slot)
        guard slot >= 0 && slot < pathJointStore.count else { return }
        let data = pathJointStore[slot]
        guard data.geometry.count > 0 else { return }
        let pathTransform = mulWorldTransforms(xfA, sim.localFrameA)
        let steps = 16 * Int(data.geometry.count)
        var previous = transformWorldPoint(pathTransform, data.geometry.evaluate(0).position)
        for i in 1...steps {
            let fraction = data.geometry.maxFraction * Float(i) / Float(steps)
            let p = transformWorldPoint(pathTransform, data.geometry.evaluate(i == steps && data.geometry.looping ? 0 : fraction).position)
            drawSegment(previous, p, DebugColors.lightBlue)
            previous = p
        }
        let e = data.geometry.evaluate(data.fraction)
        let p = transformWorldPoint(pathTransform, e.position)
        let length = 0.2 * scale
        drawSegment(p, offsetPos(p, length * rotateVector(pathTransform.q, e.tangent)), DebugColors.red)
        drawSegment(p, offsetPos(p, length * rotateVector(pathTransform.q, e.normal)), DebugColors.green)
        drawSegment(p, offsetPos(p, length * rotateVector(pathTransform.q, e.binormal)), DebugColors.blue)
        drawSegment(p, pB, DebugColors.gray)
        drawPoint?(pB, 6, DebugColors.white)
    }
}
