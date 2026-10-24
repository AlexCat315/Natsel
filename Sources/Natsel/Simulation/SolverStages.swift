// Staged constraint solver (b3SolverStep in C). Each substep runs body integration and, per graph
// color, warm starting, the biased solve, and relaxation; then restitution and impulse storage. A
// graph color holds joints and contacts that share no dynamic body, so all of a color's work can
// run on any number of threads and give the same bits as running it in order. The overflow
// constraints and vehicles can touch any body; they run on the calling thread between colors, in
// the same order as the serial solver.

enum SolverStageKind: Int32 {
    case prepareJoints
    case prepareContacts
    case prepareWideJoints
    case integrateVelocities
    case integratePositions
    case warmStart
    case push
    case relax
    case restitution
    case store
    case finalizeBodies
}

extension World {
    /// One color's solver work as raw pointers.
    struct SolverColorView {
        var joints = JointColorView()
        var constraints: UnsafePointer<ContactConstraint>?
        var scalarConvex = 0
        var scalarCount = 0
        var wide: UnsafePointer<ContactConstraintW>?
        var wideConvex = 0
        var wideCount = 0
        var useWideConvex = false
        var useWideMesh = false

        var convexItems: Int { useWideConvex ? wideConvex : scalarConvex }
        var meshItems: Int { useWideMesh ? wideCount - wideConvex : scalarCount - scalarConvex }
        /// Joint units, then convex contacts, then mesh contacts.
        var itemCount: Int { joints.unitCount + convexItems + meshItems }
    }

    /// Pointers into the solver arrays, taken once per step after preparation. The arrays are not
    /// resized until the impulses are stored.
    struct SolverViews {
        var sims = UnsafeBufferPointer<BodySim>(start: nil, count: 0)
        var states = UnsafeMutableBufferPointer<BodyState>(start: nil, count: 0)
        var contacts: UnsafeMutablePointer<Contact>?
        var manifoldPool = UnsafeMutableBufferPointer<ManifoldConstraint>(start: nil, count: 0)
        var wideManifolds: UnsafeMutablePointer<ManifoldConstraintW>?
        var colors: [SolverColorView] = []
    }

    /// Minimum items before a color stage is spread over the workers.
    static let parallelColorItems = 32
    /// Minimum bodies before an integration stage is spread over the workers.
    static let parallelBodyItems = 512
    /// Minimum joints or contact items before preparation is spread over the workers.
    static let parallelPrepareItems = 64

    func buildSolverViews() {
        var views = SolverViews()
        let awake = Int(SetType.awakeSet.rawValue)
        views.sims = solverSets[awake].bodySims.withUnsafeBufferPointer { $0 }
        views.states = solverSets[awake].bodyStates.withUnsafeMutableBufferPointer { $0 }
        views.contacts = contacts.withUnsafeMutableBufferPointer { $0.baseAddress }
        views.manifoldPool = solverContacts.manifoldPool.withUnsafeMutableBufferPointer { $0 }
        views.wideManifolds = solverContacts.wideManifolds.withUnsafeMutableBufferPointer { $0.baseAddress }

        views.colors.reserveCapacity(Int(overflowIndex))
        for ci in 0..<Int(overflowIndex) {
            var color = SolverColorView()
            let wide = enableWideSolver && solverJoints.starts[ci].isEmpty == false
            color.joints.joints = constraintGraph.colors[ci].jointSims.withUnsafeMutableBufferPointer { $0.baseAddress }
            color.joints.jointCount = constraintGraph.colors[ci].jointSims.count
            if wide {
                color.joints.starts = solverJoints.starts[ci].withUnsafeBufferPointer { $0.baseAddress }
                color.joints.batches = solverJoints.batches[ci].withUnsafeMutableBufferPointer { $0.baseAddress }
                color.joints.units = solverJoints.units[ci].withUnsafeBufferPointer { $0.baseAddress }
                color.joints.unitCount = solverJoints.units[ci].count
            } else {
                color.joints.unitCount = color.joints.jointCount
            }

            color.constraints = solverContacts.constraints[ci].withUnsafeBufferPointer { $0.baseAddress }
            color.scalarCount = solverContacts.constraints[ci].count
            color.scalarConvex = solverContacts.convexCounts[ci]
            color.wide = solverContacts.wideConstraints[ci].withUnsafeBufferPointer { $0.baseAddress }
            color.wideCount = solverContacts.wideConstraints[ci].count
            color.wideConvex = solverContacts.wideConvexCounts[ci]
            color.useWideConvex = usesWideContacts(Int32(ci), false)
            color.useWideMesh = usesWideContacts(Int32(ci), true)
            views.colors.append(color)
        }
        solverViews = views
    }

    /// The worker pool for `workerCount` threads, or nil when solving on one thread.
    /// The solver uses at most `performanceCoreCount` threads: its stages are separated by
    /// barriers, so a thread on a slower core holds every stage back.
    func solverPool() -> WorkerPool? {
        let threads = min(Int(maxInt(1, minInt(workerCount, B3.maxWorkers))), performanceCoreCount)
        guard threads > 1 else {
            workerPool = nil
            return nil
        }
        if let pool = workerPool, pool.threadCount == threads { return pool }
        let pool = WorkerPool(threadCount: threads)
        workerPool = pool
        return pool
    }

    // MARK: Execution

    /// Run one block of a stage. Called on any thread.
    func executeSolverBlock(_ stage: WorkerStage, _ block: Int, _ thread: Int) {
        let start = block * Int(stage.blockSize)
        let range = start..<min(start + Int(stage.blockSize), Int(stage.itemCount))
        executeSolverRange(SolverStageKind(rawValue: stage.kind)!, Int(stage.color), range, thread)
    }

    /// `thread` indexes per-thread scratch data; zero is the calling thread.
    func executeSolverRange(_ kind: SolverStageKind, _ ci: Int, _ range: Range<Int>, _ thread: Int = 0) {
        let context = stageContext
        switch kind {
        case .prepareJoints:
            prepareJointRange(range)
        case .prepareContacts:
            prepareContactRange(range)
        case .prepareWideJoints:
            prepareWideJointRange(range)
        case .integrateVelocities:
            integrateVelocitiesRange(solverViews.sims, solverViews.states, range, context.h)
        case .integratePositions:
            integratePositionsRange(solverViews.states, range, context.h, context.inv_dt)
        case .finalizeBodies:
            finalizeBodyRange(range, thread)
        default:
            executeColorRange(kind, ci, range)
        }
    }

    private func executeColorRange(_ kind: SolverStageKind, _ ci: Int, _ range: Range<Int>) {
        let color = solverViews.colors[ci]
        let states = solverViews.states
        let context = stageContext

        // Joint units.
        let joints = color.joints.unitCount
        let jointRange = range.clamped(to: 0..<joints)
        if jointRange.isEmpty == false {
            switch kind {
            case .warmStart: warmStartJointsRange(color.joints, jointRange, context, states)
            case .push: solveJointsRange(color.joints, jointRange, context, true, states)
            case .relax: solveJointsRange(color.joints, jointRange, context, false, states)
            default: break
            }
        }

        // Convex then mesh contacts.
        let convex = color.convexItems
        let convexRange = (range.lowerBound - joints..<range.upperBound - joints).clamped(to: 0..<convex)
        if convexRange.isEmpty == false {
            executeContactRange(kind, color, false, convexRange)
        }
        let meshStart = joints + convex
        let meshRange = (range.lowerBound - meshStart..<range.upperBound - meshStart).clamped(to: 0..<color.meshItems)
        if meshRange.isEmpty == false {
            executeContactRange(kind, color, true, meshRange)
        }
    }

    private func executeContactRange(_ kind: SolverStageKind, _ color: SolverColorView, _ mesh: Bool, _ items: Range<Int>) {
        let states = solverViews.states
        let context = stageContext
        if mesh ? color.useWideMesh : color.useWideConvex {
            guard let list = color.wide, let pool = solverViews.wideManifolds else { return }
            let offset = mesh ? color.wideConvex : 0
            let range = items.lowerBound + offset..<items.upperBound + offset
            switch kind {
            case .warmStart: warmStartContactsWideRange(list, range, states, pool)
            case .push: pushContactsWideRange(list, range, context, states, pool)
            case .relax: solveContactsWideRange(list, range, context, states, pool)
            case .restitution: applyRestitutionWideRange(list, range, context, states, pool)
            case .store:
                if let contacts = solverViews.contacts {
                    storeImpulsesWideRange(list, range, UnsafePointer(pool), contacts)
                }
            default: break
            }
        } else {
            guard let list = color.constraints else { return }
            let offset = mesh ? color.scalarConvex : 0
            let range = items.lowerBound + offset..<items.upperBound + offset
            let pool = solverViews.manifoldPool
            switch kind {
            case .warmStart: warmStartContactsBody(range, list, states, pool)
            case .push: pushContactsBody(range, list, context, states, pool)
            case .relax: solveContactsBody(range, list, context, states, pool)
            case .restitution: applyRestitutionBody(range, list, context, states, pool)
            case .store:
                if let contacts = solverViews.contacts, let base = pool.baseAddress {
                    storeImpulsesBody(range, list, UnsafePointer(base), contacts)
                }
            default: break
            }
        }
    }

    /// Run a stage over `itemCount` items, in parallel when it is large enough.
    func runStage(_ kind: SolverStageKind, _ ci: Int, _ itemCount: Int, minimum: Int) {
        guard itemCount > 0 else { return }
        guard let pool = activePool, itemCount >= minimum else {
            executeSolverRange(kind, ci, 0..<itemCount)
            return
        }
        // Several blocks per thread so faster threads take up the slack.
        let targetBlocks = 4 * pool.threadCount
        let blockSize = max(minimum / 4, (itemCount + targetBlocks - 1) / targetBlocks)
        var stage = WorkerStage()
        stage.kind = kind.rawValue
        stage.color = Int32(ci)
        stage.itemCount = Int32(itemCount)
        stage.blockSize = Int32(blockSize)
        stage.blockCount = Int32((itemCount + blockSize - 1) / blockSize)
        pool.run(stage)
    }

    private func runColors(_ kind: SolverStageKind) {
        for ci in 0..<Int(overflowIndex) {
            runStage(kind, ci, solverViews.colors[ci].itemCount, minimum: Self.parallelColorItems)
        }
    }

    private func runBodies(_ kind: SolverStageKind) {
        runStage(kind, 0, solverViews.states.count, minimum: Self.parallelBodyItems)
    }

    // MARK: Driver

    /// Substeps, restitution, and impulse storage. Prepared constraints are required. Runs on the
    /// session's worker pool when `beginSolverSession` started one.
    func solveConstraintStages(_ context: StepContext) {
        var ticks = getTicks()
        stageContext = context
        buildSolverViews()

        for _ in 0..<Int(context.subStepCount) {
            runBodies(.integrateVelocities)
            profile.integrateVelocities += getMillisecondsAndReset(&ticks)

            warmStartJoints(overflowIndex, context)
            warmStartContacts(overflowIndex, true)
            runColors(.warmStart)
            warmStartVehicles(context)
            profile.warmStart += getMillisecondsAndReset(&ticks)

            solveJoints(overflowIndex, context, true)
            pushContacts(overflowIndex, context, true)
            runColors(.push)
            solveVehicles(context, true)
            profile.solveImpulses += getMillisecondsAndReset(&ticks)

            runBodies(.integratePositions)
            profile.integratePositions += getMillisecondsAndReset(&ticks)

            solveJoints(overflowIndex, context, false)
            solveContacts(overflowIndex, context, true)
            runColors(.relax)
            solveVehicles(context, false)
            profile.relaxImpulses += getMillisecondsAndReset(&ticks)
        }

        if solveAnyRestitution && restitutionIterations > 0 {
            for _ in 0..<Int(restitutionIterations) {
                applyRestitution(overflowIndex, context, true)
                runColors(.restitution)
            }
        }
        profile.restitution = getMillisecondsAndReset(&ticks)

        storeWideJointImpulses()
        storeImpulses(overflowIndex, true)
        runColors(.store)
        profile.storeImpulses = getMillisecondsAndReset(&ticks)

        solverViews = SolverViews()
    }
}
