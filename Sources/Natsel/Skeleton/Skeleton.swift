// Skeletons and poses (Jolt's Skeleton and SkeletonPose). A skeleton is a list of named joints,
// each after its parent. A pose holds every joint's transform relative to its parent (the joint
// states) and in model space (the joint matrices); model space is relative to the pose's root
// offset, so world transforms are `rootOffset` plus the joint matrix.

public struct Skeleton: Sendable {
    public struct Joint: Sendable, Equatable {
        public var name: String
        /// Index of the parent joint, or -1 for a root.
        public var parentIndex: Int

        public init(name: String, parentIndex: Int) {
            self.name = name
            self.parentIndex = parentIndex
        }
    }

    public private(set) var joints: [Joint] = []
    private var indices: [String: Int] = [:]

    public init() {}

    public var jointCount: Int { joints.count }

    /// Add a joint after its parent. Returns its index. Names must be unique.
    @discardableResult
    public mutating func addJoint(_ name: String, parent: Int = -1) -> Int {
        precondition(parent >= -1 && parent < joints.count, "a joint must follow its parent")
        precondition(indices[name] == nil, "duplicate joint name \(name)")
        indices[name] = joints.count
        joints.append(Joint(name: name, parentIndex: parent))
        return joints.count - 1
    }

    @discardableResult
    public mutating func addJoint(_ name: String, parentName: String) -> Int {
        guard let parent = indices[parentName] else { preconditionFailure("unknown parent \(parentName)") }
        return addJoint(name, parent: parent)
    }

    public func jointIndex(_ name: String) -> Int? { indices[name] }

    public func parentIndex(_ joint: Int) -> Int { joints[joint].parentIndex }

    /// True when `ancestor` is on the parent chain of `joint` (not counting the joint itself).
    public func isAncestor(_ ancestor: Int, of joint: Int) -> Bool {
        var current = joints[joint].parentIndex
        while current >= 0 {
            if current == ancestor { return true }
            current = joints[current].parentIndex
        }
        return false
    }
}

/// A joint transform relative to its parent.
public struct SkeletonJointState: Sendable, Equatable {
    public var translation = Vec3()
    public var rotation = Quat.identity

    public init() {}

    public init(translation: Vec3, rotation: Quat) {
        self.translation = translation
        self.rotation = rotation
    }

    public init(_ transform: Transform) {
        translation = transform.p
        rotation = transform.q
    }

    public var transform: Transform { Transform(translation, rotation) }
}

public struct SkeletonPose: Sendable {
    public let skeleton: Skeleton
    /// World position of the model space origin.
    public var rootOffset = Pos()
    /// Joint transforms relative to their parents.
    public var jointStates: [SkeletonJointState]
    /// Joint transforms in model space.
    public var jointMatrices: [Transform]

    public init(skeleton: Skeleton) {
        self.skeleton = skeleton
        jointStates = Array(repeating: SkeletonJointState(), count: skeleton.jointCount)
        jointMatrices = Array(repeating: .identity, count: skeleton.jointCount)
    }

    public var jointCount: Int { jointStates.count }

    /// Joint matrices from the joint states.
    public mutating func calculateJointMatrices() {
        for i in 0..<jointStates.count {
            let local = jointStates[i].transform
            let parent = skeleton.joints[i].parentIndex
            jointMatrices[i] = parent < 0 ? local : mulTransforms(jointMatrices[parent], local)
        }
    }

    /// Joint states from the joint matrices.
    public mutating func calculateJointStates() {
        for i in 0..<jointMatrices.count {
            let parent = skeleton.joints[i].parentIndex
            let local = parent < 0 ? jointMatrices[i] : invMulTransforms(jointMatrices[parent], jointMatrices[i])
            jointStates[i] = SkeletonJointState(Transform(local.p, normalizeQuat(local.q)))
        }
    }

    public func worldTransform(_ joint: Int) -> WorldTransform {
        let model = jointMatrices[joint]
        return WorldTransform(offsetPos(rootOffset, model.p), model.q)
    }

    public mutating func setWorldTransform(_ joint: Int, _ transform: WorldTransform) {
        jointMatrices[joint] = Transform(subPos(transform.p, rootOffset), transform.q)
    }
}

/// Spherical interpolation along the shorter arc; falls back to nlerp for nearly equal rotations.
func slerpQuat(_ q1: Quat, _ q2: Quat, _ t: Float) -> Quat {
    var q2 = q2
    var cosine = dotQuat(q1, q2)
    if cosine < 0 {
        q2 = negateQuat(q2)
        cosine = -cosine
    }
    if cosine > 0.9995 { return nlerp(q1, q2, t) }
    // The engine's own trigonometry keeps results identical across platforms.
    let sine = (1 - cosine * cosine).squareRoot()
    let angle = atan2Approx(sine, cosine)
    let a = computeCosSin((1 - t) * angle).sine / sine
    let b = computeCosSin(t * angle).sine / sine
    return normalizeQuat(Quat(a * q1.v + b * q2.v, a * q1.s + b * q2.s))
}
