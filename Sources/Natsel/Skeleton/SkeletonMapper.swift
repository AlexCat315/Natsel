// Maps poses between a low detail skeleton (usually a ragdoll's) and a high detail one (usually
// an animation's), as Jolt's SkeletonMapper does. Joints with the same name are mapped through the
// offset between the two neutral poses. High detail joints between two mapped joints form a chain:
// they keep their animated local transforms and the chain is turned about its start so its end
// meets the mapped end joint. Other high detail joints follow their parents with animated locals.

public struct SkeletonMapper: Sendable {
    public struct Mapping: Sendable {
        public var lowIndex: Int
        public var highIndex: Int
        /// High detail joint relative to the low detail joint in the neutral poses.
        public var offset: Transform
    }

    public struct Chain: Sendable {
        /// High detail joints from the mapped start to the mapped end, both included.
        public var highIndices: [Int]
    }

    public let low: Skeleton
    public let high: Skeleton
    public private(set) var mappings: [Mapping] = []
    public private(set) var chains: [Chain] = []
    /// High detail joints that are neither mapped nor inside a chain.
    public private(set) var unmapped: [Int] = []
    private var highToMapping: [Int] = []
    private var highInChain: [Bool] = []

    /// `lowNeutral` and `highNeutral` are model space joint matrices of the two skeletons in the
    /// same neutral pose. Returns nil when no joint names match.
    public init?(low: Skeleton, lowNeutral: [Transform], high: Skeleton, highNeutral: [Transform]) {
        guard lowNeutral.count == low.jointCount, highNeutral.count == high.jointCount else { return nil }
        self.low = low
        self.high = high
        highToMapping = Array(repeating: -1, count: high.jointCount)
        for (lowIndex, joint) in low.joints.enumerated() {
            guard let highIndex = high.jointIndex(joint.name) else { continue }
            highToMapping[highIndex] = mappings.count
            mappings.append(Mapping(lowIndex: lowIndex, highIndex: highIndex,
                                    offset: invMulTransforms(lowNeutral[lowIndex], highNeutral[highIndex])))
        }
        guard mappings.isEmpty == false else { return nil }

        // A chain runs from a mapped joint's nearest mapped ancestor through unmapped joints.
        highInChain = Array(repeating: false, count: high.jointCount)
        for mapping in mappings {
            var path: [Int] = []
            var current = high.joints[mapping.highIndex].parentIndex
            while current >= 0 && highToMapping[current] < 0 {
                path.append(current)
                current = high.joints[current].parentIndex
            }
            guard current >= 0, path.isEmpty == false else { continue }
            let indices = [current] + path.reversed() + [mapping.highIndex]
            chains.append(Chain(highIndices: indices))
            for index in path { highInChain[index] = true }
        }
        unmapped = (0..<high.jointCount).filter { highToMapping[$0] < 0 && highInChain[$0] == false }
    }

    /// Fill the high detail pose's joint matrices from the low detail pose's joint matrices.
    /// `animated` supplies local transforms for chains and unmapped joints. Joint states of
    /// `highPose` are recomputed.
    public func map(_ lowPose: SkeletonPose, animated: SkeletonPose, into highPose: inout SkeletonPose) {
        highPose.rootOffset = lowPose.rootOffset
        var done = Array(repeating: false, count: high.jointCount)
        for mapping in mappings {
            highPose.jointMatrices[mapping.highIndex] = mulTransforms(lowPose.jointMatrices[mapping.lowIndex], mapping.offset)
            done[mapping.highIndex] = true
        }

        // Joints in skeleton order: a chain is handled when its first inner joint comes up, at
        // which point its start (an ancestor) is final.
        let chainByFirstInner = Dictionary(uniqueKeysWithValues: chains.map { ($0.highIndices[1], $0) })
        for index in 0..<high.jointCount where done[index] == false {
            if let chain = chainByFirstInner[index] {
                mapChain(chain, animated, &highPose)
                for inner in chain.highIndices.dropFirst().dropLast() { done[inner] = true }
                continue
            }
            if highInChain[index] { continue }
            let parent = high.joints[index].parentIndex
            let local = animated.jointStates[index].transform
            highPose.jointMatrices[index] = parent < 0 ? local : mulTransforms(highPose.jointMatrices[parent], local)
            done[index] = true
        }
        highPose.calculateJointStates()
    }

    private func mapChain(_ chain: Chain, _ animated: SkeletonPose, _ pose: inout SkeletonPose) {
        let indices = chain.highIndices
        let start = pose.jointMatrices[indices[0]]
        // Lay the inner joints out with the animated locals from the start.
        var matrices: [Transform] = [start]
        for k in 1..<indices.count {
            matrices.append(mulTransforms(matrices[k - 1], animated.jointStates[indices[k]].transform))
        }
        // Turn the chain about its start so the animated end meets the mapped end.
        let target = pose.jointMatrices[indices[indices.count - 1]].p - start.p
        let current = matrices[matrices.count - 1].p - start.p
        var turn = Quat.identity
        if lengthSquared(target) > 1.0e-12 && lengthSquared(current) > 1.0e-12 {
            turn = computeQuatBetweenUnitVectors(normalize(current), normalize(target))
        }
        for k in 1..<(indices.count - 1) {
            let relative = matrices[k].p - start.p
            pose.jointMatrices[indices[k]] = Transform(start.p + rotateVector(turn, relative),
                                                       normalizeQuat(mulQuat(turn, matrices[k].q)))
        }
    }

    /// Fill the low detail pose's joint matrices from the high detail pose's. Joints of the low
    /// skeleton without a match keep their transforms relative to their parents.
    public func mapReverse(_ highPose: SkeletonPose, into lowPose: inout SkeletonPose) {
        lowPose.rootOffset = highPose.rootOffset
        var mapped = Array(repeating: false, count: low.jointCount)
        for mapping in mappings {
            lowPose.jointMatrices[mapping.lowIndex] = mulTransforms(highPose.jointMatrices[mapping.highIndex],
                                                                    invertTransform(mapping.offset))
            mapped[mapping.lowIndex] = true
        }
        for index in 0..<low.jointCount where mapped[index] == false {
            let parent = low.joints[index].parentIndex
            let local = lowPose.jointStates[index].transform
            lowPose.jointMatrices[index] = parent < 0 ? local : mulTransforms(lowPose.jointMatrices[parent], local)
        }
        lowPose.calculateJointStates()
    }
}
