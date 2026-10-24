// Constraint preparation in two passes (b3PrepareContactsTask / b3PrepareJointsTask in C).
//
// Planning runs on one thread: it fixes the solver order of the contacts, gives every scalar
// contact and every four-wide batch its constraint slot and its run in the manifold pools, and sizes
// the arrays. Filling then writes only those slots, so it runs on any number of threads and lays the
// data out exactly as appending in order did.

/// One unit of contact preparation: a scalar contact or a batch of up to four contacts.
struct ContactPrepareItem: BitwiseCopyable {
    var color: Int32 = 0
    var wide = false
    /// Constraint index in the color's scalar or wide list.
    var slot: Int32 = 0
    /// First contact id in `World.prepareIds` and how many (one for scalar).
    var idStart: Int32 = 0
    var idCount: Int32 = 0
    /// Run in the scalar manifold pool or the wide manifold pool.
    var manifoldStart: Int32 = 0

    init() {}
}

/// One planned four-wide spherical joint batch.
struct WideJointPrepareItem: BitwiseCopyable {
    var color: Int32 = 0
    var first: Int32 = 0
    var laneCount: Int32 = 0
    var batch: Int32 = 0

    init() {}
}

/// Pointers and parameters for filling, taken after the arrays are sized.
struct ContactPrepareViews {
    var items: UnsafePointer<ContactPrepareItem>?
    var ids: UnsafePointer<Int32>?
    var contacts: UnsafePointer<Contact>?
    var sims = UnsafeBufferPointer<BodySim>(start: nil, count: 0)
    var states = UnsafeMutableBufferPointer<BodyState>(start: nil, count: 0)
    var manifoldPool: UnsafeMutablePointer<ManifoldConstraint>?
    var wideManifolds: UnsafeMutablePointer<ManifoldConstraintW>?
    var constraints: [UnsafeMutablePointer<ContactConstraint>?] = []
    var wideConstraints: [UnsafeMutablePointer<ContactConstraintW>?] = []
    var softness = Softness()
    var staticSoftness = Softness()
    var warmStartScale: Float = 0

    /// Joint lists of every color, with the index of each color's first joint.
    var joints: [UnsafeMutablePointer<JointSim>?] = []
    var jointStarts: [Int] = []
    /// Body table and the sims of the two sets joint bodies can be in, for joint preparation.
    var bodies: UnsafePointer<Body>?
    var staticSims: UnsafePointer<BodySim>?
    var awakeSims: UnsafePointer<BodySim>?
    var wideJointItems: UnsafePointer<WideJointPrepareItem>?
    var wideJointBatches: [UnsafeMutablePointer<SphericalConstraintW>?] = []
}

extension World {
    // MARK: Session

    /// Wake the worker pool for preparation and solving. Without one everything runs inline.
    func beginSolverSession() {
        guard let pool = solverPool() else { return }
        pool.begin { [unowned self] stage, block, thread in self.executeSolverBlock(stage, block, thread) }
        activePool = pool
    }

    func endSolverSession() {
        activePool?.end()
        activePool = nil
    }

    // MARK: Joints

    /// Prepare every joint (all colors including overflow), then pack the wide batches.
    func prepareAllJoints(_ context: StepContext) {
        stageContext = context
        var views = prepareViews
        views.joints.removeAll(keepingCapacity: true)
        views.jointStarts.removeAll(keepingCapacity: true)
        var total = 0
        for ci in 0..<Int(B3.graphColorCount) {
            views.joints.append(constraintGraph.colors[ci].jointSims.withUnsafeMutableBufferPointer { $0.baseAddress })
            views.jointStarts.append(total)
            total += constraintGraph.colors[ci].jointSims.count
        }
        views.jointStarts.append(total)
        views.bodies = bodies.withUnsafeBufferPointer { $0.baseAddress }
        views.staticSims = solverSets[Int(SetType.staticSet.rawValue)].bodySims.withUnsafeBufferPointer { $0.baseAddress }
        views.awakeSims = solverSets[Int(SetType.awakeSet.rawValue)].bodySims.withUnsafeBufferPointer { $0.baseAddress }
        prepareViews = views
        runStage(.prepareJoints, 0, total, minimum: Self.parallelPrepareItems)
        prepareWideJoints()
        prepareViews.bodies = nil
        prepareViews.staticSims = nil
        prepareViews.awakeSims = nil
    }

    /// A joint body during preparation. Reads through raw pointers: going through the Swift arrays
    /// retains their buffers, and several threads retaining one buffer serialize on its count.
    @inline(__always)
    func jointPrepareBody(_ bodyId: Int32) -> Body {
        if let table = prepareViews.bodies { return table[Int(bodyId)] }
        return bodies[Int(bodyId)]
    }

    /// The sim of a joint body during preparation. Joint bodies are awake or static.
    @inline(__always)
    func jointPrepareSim(_ body: Body) -> BodySim {
        if body.setIndex == SetType.awakeSet.rawValue, let sims = prepareViews.awakeSims {
            return sims[Int(body.localIndex)]
        }
        if body.setIndex == SetType.staticSet.rawValue, let sims = prepareViews.staticSims {
            return sims[Int(body.localIndex)]
        }
        return solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]
    }

    /// Prepare joints `range` of the flattened joint list.
    func prepareJointRange(_ range: Range<Int>) {
        let starts = prepareViews.jointStarts
        let lists = prepareViews.joints
        let context = stageContext
        var ci = 0
        for index in range {
            while starts[ci + 1] <= index { ci += 1 }
            guard let list = lists[ci] else { continue }
            prepareJoint(&list[index - starts[ci]], context)
        }
    }

    // MARK: Contacts

    /// Prepare all constraint data for this step.
    func prepareAllContacts(_ context: StepContext) {
        for i in solverContacts.constraints.indices {
            solverContacts.constraints[i].removeAll(keepingCapacity: true)
        }
        for i in solverContacts.convexCounts.indices {
            solverContacts.convexCounts[i] = 0
            solverContacts.wideConvex[i] = false
            solverContacts.wideMesh[i] = false
            solverContacts.wideConvexCounts[i] = 0
            solverContacts.wideConstraints[i].removeAll(keepingCapacity: true)
        }
        // The manifold pools keep their high-water size: planning assigns every run, filling writes
        // it, and the unused tail is never read. Clearing them would zero megabytes every step.
        solveAnyRestitution = false

        prepareAllJoints(context)

        let contactHertz = minFloat(contactHertz, 0.125 * context.inv_h)
        let contactSoftness = makeSoft(contactHertz, contactDampingRatio, context.h)
        let staticSoftness = makeSoft(2.0 * contactHertz, 0.5 * contactDampingRatio, context.h)

        // Overflow first, then colors in order.
        prepareOverflowContacts(contactSoftness, staticSoftness)
        let count = planContactPreparation()
        guard count > 0 else { return }

        var views = prepareViews
        let awake = Int(SetType.awakeSet.rawValue)
        views.items = prepareItems.withUnsafeBufferPointer { $0.baseAddress }
        views.ids = prepareIds.withUnsafeBufferPointer { $0.baseAddress }
        views.contacts = contacts.withUnsafeBufferPointer { $0.baseAddress }
        views.sims = solverSets[awake].bodySims.withUnsafeBufferPointer { $0 }
        views.states = solverSets[awake].bodyStates.withUnsafeMutableBufferPointer { $0 }
        views.manifoldPool = solverContacts.manifoldPool.withUnsafeMutableBufferPointer { $0.baseAddress }
        views.wideManifolds = solverContacts.wideManifolds.withUnsafeMutableBufferPointer { $0.baseAddress }
        views.constraints.removeAll(keepingCapacity: true)
        views.wideConstraints.removeAll(keepingCapacity: true)
        for ci in 0..<Int(overflowIndex) {
            views.constraints.append(solverContacts.constraints[ci].withUnsafeMutableBufferPointer { $0.baseAddress })
            views.wideConstraints.append(solverContacts.wideConstraints[ci].withUnsafeMutableBufferPointer { $0.baseAddress })
        }
        views.softness = contactSoftness
        views.staticSoftness = staticSoftness
        views.warmStartScale = enableWarmStarting ? 1 : 0
        prepareViews = views

        runStage(.prepareContacts, 0, count, minimum: Self.parallelPrepareItems)
    }

    /// Fix the order, slots, and manifold runs of every colored contact and size the arrays.
    /// Returns the number of preparation items.
    private func planContactPreparation() -> Int {
        prepareItems.removeAll(keepingCapacity: true)
        prepareIds.removeAll(keepingCapacity: true)
        var contactTotal = 0
        for ci in 0..<Int(overflowIndex) {
            contactTotal += constraintGraph.colors[ci].convexContacts.count + constraintGraph.colors[ci].contacts.count
        }
        guard contactTotal > 0 else { return 0 }
        prepareItems.reserveCapacity(contactTotal)
        prepareIds.reserveCapacity(contactTotal)

        var manifoldTotal: Int32 = 0
        var wideManifoldTotal: Int32 = 0
        var anyRestitution = false

        contacts.withUnsafeBufferPointer { contactStore in
            for ci in 0..<Int(overflowIndex) {
                let convexIds = constraintGraph.colors[ci].convexContacts
                let meshIds = constraintGraph.colors[ci].contacts.isEmpty ? [] : solverMeshContactIds(Int32(ci))
                guard convexIds.isEmpty == false || meshIds.isEmpty == false else { continue }

                let wideConvex = enableWideSolver && convexIds.count >= 4
                let wideMesh = enableWideSolver && meshIds.count >= 4
                solverContacts.wideConvex[ci] = wideConvex
                solverContacts.wideMesh[ci] = wideMesh
                let scalarConvex = !wideConvex || prepareScalarForWideColors
                let scalarMesh = !wideMesh || prepareScalarForWideColors
                solverContacts.convexCounts[ci] = scalarConvex ? convexIds.count : 0

                // Scalar constraints: convex then mesh.
                var slot: Int32 = 0
                for pass in 0..<2 where pass == 0 ? scalarConvex : scalarMesh {
                    for id in pass == 0 ? convexIds : meshIds {
                        let contact = contactStore[Int(id)]
                        var item = ContactPrepareItem()
                        item.color = Int32(ci)
                        item.slot = slot
                        item.idStart = Int32(prepareIds.count)
                        item.idCount = 1
                        item.manifoldStart = manifoldTotal
                        prepareItems.append(item)
                        prepareIds.append(id)
                        manifoldTotal += contact.manifoldCount
                        anyRestitution = anyRestitution || contact.restitution > 0
                        slot += 1
                    }
                }
                solverContacts.constraints[ci].append(contentsOf: repeatElement(ContactConstraint(), count: Int(slot)))

                // Wide batches: convex then mesh.
                var batch: Int32 = 0
                for pass in 0..<2 {
                    if pass == 1 { solverContacts.wideConvexCounts[ci] = Int(batch) }
                    guard pass == 0 ? wideConvex : wideMesh else { continue }
                    let ids = pass == 0 ? convexIds : meshIds
                    for first in stride(from: 0, to: ids.count, by: 4) {
                        let lanes = min(4, ids.count - first)
                        var item = ContactPrepareItem()
                        item.color = Int32(ci)
                        item.wide = true
                        item.slot = batch
                        item.idStart = Int32(prepareIds.count)
                        item.idCount = Int32(lanes)
                        item.manifoldStart = wideManifoldTotal
                        prepareItems.append(item)
                        var maxManifolds: Int32 = 0
                        for lane in 0..<lanes {
                            let id = ids[first + lane]
                            let contact = contactStore[Int(id)]
                            prepareIds.append(id)
                            maxManifolds = max(maxManifolds, contact.manifoldCount)
                            anyRestitution = anyRestitution || contact.restitution > 0
                        }
                        wideManifoldTotal += maxManifolds
                        batch += 1
                    }
                }
                solverContacts.wideConstraints[ci].append(contentsOf: repeatElement(ContactConstraintW(), count: Int(batch)))
            }
        }
        if anyRestitution { solveAnyRestitution = true }

        let pool = Int(manifoldTotal) - solverContacts.manifoldPool.count
        if pool > 0 { solverContacts.manifoldPool.append(contentsOf: repeatElement(ManifoldConstraint(), count: pool)) }
        let widePool = Int(wideManifoldTotal) - solverContacts.wideManifolds.count
        if widePool > 0 {
            solverContacts.wideManifolds.append(contentsOf: repeatElement(ManifoldConstraintW(), count: widePool))
        }
        return prepareItems.count
    }

    /// Fill items `range` of the plan. Each writes only its own slots.
    func prepareContactRange(_ range: Range<Int>) {
        let views = prepareViews
        guard let items = views.items, let ids = views.ids, let contactBase = views.contacts else { return }
        for i in range {
            let item = items[i]
            let ci = Int(item.color)
            if item.wide {
                guard let out = views.wideConstraints[ci], let pool = views.wideManifolds,
                    let sims = views.sims.baseAddress
                else { continue }
                prepareWideBatch(
                    ids + Int(item.idStart), Int(item.idCount), out + Int(item.slot),
                    pool + Int(item.manifoldStart), Int(item.manifoldStart), contactBase, sims, views.states,
                    views.softness, views.staticSoftness, views.warmStartScale)
            } else {
                guard let out = views.constraints[ci], let pool = views.manifoldPool else { continue }
                prepareContactConstraint(
                    ids[Int(item.idStart)], out + Int(item.slot), pool + Int(item.manifoldStart), item.manifoldStart,
                    contactBase, views.sims, views.states, views.softness, views.staticSoftness, views.warmStartScale)
            }
        }
    }
}
