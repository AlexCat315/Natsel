import Testing
@testable import Natsel

@Suite("Body sim locator scenes")
struct LocatorSceneTests {
    private func createBody(
        _ world: World, _ type: BodyType, _ position: Pos, _ hx: Float, _ hy: Float, _ hz: Float
    ) -> BodyID {
        var def = BodyDef()
        def.type = type
        def.position = position
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(hx, hy, hz))
        return body
    }

    private func step(_ world: World, _ count: Int) {
        for _ in 0..<count { world.step(1.0 / 60.0, subStepCount: 4) }
    }

    /// Steps until the body sleeps or the budget runs out. Returns the steps taken.
    private func stepUntilAsleep(_ world: World, _ body: BodyID, _ maxSteps: Int) -> Int {
        for i in 0..<maxSteps {
            world.step(1.0 / 60.0, subStepCount: 4)
            if world.isBodyAwake(body) == false {
                return i + 1
            }
        }
        return maxSteps
    }

    /// Sleep and wake: the first touch puts contacts in the graph, sleeping nulls the locators of
    /// the island bodies, waking restores their awake indices. The plate holds a non-touching
    /// contact across the whole cycle, the case the wake path has to fix up itself.
    // TODO(Natsel): waking a solver set does not yet refresh the non-touching contact that the
    // kinematic plate holds, so the stack is not seen awake after the wake call.
    @Test
    func sleepWake() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            _ = createBody(world, .staticBody, Pos(0, -1, 0), 5, 1, 5)

            var stack: [BodyID] = []
            for i in 0..<3 {
                stack.append(createBody(world, .dynamicBody, Pos(0, 0.5 + Float(i), 0), 0.5, 0.5, 0.5))
            }

            // Beside the top box, close enough that the fat bounds overlap and far enough that the
            // shapes never touch. Kinematic and awake, so the contact stays in the awake set while the
            // stack sleeps.
            let plate = createBody(world, .kinematicBody, Pos(Float(1.04), Float(2.5), 0), 0.5, 0.5, 0.5)
            world.enableBodySleep(plate, false)

            var steps = stepUntilAsleep(world, stack[0], 300)
            #expect(steps < 300)
            // The whole island sleeps together.
            #expect(world.isBodyAwake(stack[2]) == false)

            // Wake without contact so the non-touching contact keeps its stale locator if the wake
            // path forgets to refresh.
            world.setBodyAwake(stack[0], true)
            step(world, 5)
            // Waking one body wakes its whole solver set.
            #expect(world.isBodyAwake(stack[2]))

            steps = stepUntilAsleep(world, stack[0], 300)
            #expect(steps < 300)

            // Dropping a body on the pile drives the wake path through first touch.
            _ = createBody(world, .dynamicBody, Pos(0, 8, 0), 0.5, 0.5, 0.5)

            var woke = false
            for _ in 0..<180 {
                world.step(1.0 / 60.0, subStepCount: 4)
                woke = woke || world.isBodyAwake(stack[0])
            }
            #expect(woke)
            samples.append(UInt64(world.worldStateHash()))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    /// An island falling asleep and a body leaving the awake set both swap the last awake sim into
    /// the hole.
    @Test("awake neighbor swaps keep locators consistent")
    func awakeNeighbor() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            _ = createBody(world, .staticBody, Pos(0, -1, 0), 30, 1, 5)

            var sleepers: [BodyID] = []
            for i in 0..<2 {
                sleepers.append(
                    createBody(world, .dynamicBody, Pos(-10, 0.5 + Float(i), 0), 0.5, 0.5, 0.5)
                )
            }

            let spare = createBody(world, .dynamicBody, Pos(20, Float(0.5), 0), 0.5, 0.5, 0.5)
            world.enableBodySleep(spare, false)

            var awakeBodies: [BodyID] = []
            for i in 0..<3 {
                let body = createBody(world, .dynamicBody, Pos(10, 0.5 + Float(i), 0), 0.5, 0.5, 0.5)
                world.enableBodySleep(body, false)
                awakeBodies.append(body)
            }

            let steps = stepUntilAsleep(world, sleepers[0], 300)
            #expect(steps < 300)
            #expect(world.isBodyAwake(awakeBodies[0]))

            step(world, 10)
            world.disableBody(spare)
            step(world, 10)
            world.enableBody(spare)
            step(world, 10)

            samples.append(UInt64(world.worldStateHash()))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    /// Destroying a body swaps the last sim of its set into the hole. The moved body keeps its
    /// contacts, so its side of each one has to follow it.
    @Test("destroy swaps keep locators consistent")
    func destroyScene() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            _ = createBody(world, .staticBody, Pos(0, -1, 0), 5, 1, 5)

            var stack: [BodyID] = []
            for i in 0..<4 {
                stack.append(createBody(world, .dynamicBody, Pos(0, 0.5 + Float(i), 0), 0.5, 0.5, 0.5))
            }

            step(world, 30)
            world.destroyBody(stack[0])
            step(world, 30)

            // Three static boxes, one resting box each. Destroying the first moves the last static sim.
            var staticBodies: [BodyID] = []
            for i in 0..<3 {
                let x = 10 + 4 * Float(i)
                staticBodies.append(createBody(world, .staticBody, Pos(x, -1, 0), 1, 1, 1))
                _ = createBody(world, .dynamicBody, Pos(x, Float(0.5), 0), 0.5, 0.5, 0.5)
            }

            step(world, 30)
            world.destroyBody(staticBodies[0])
            step(world, 30)

            samples.append(UInt64(world.worldStateHash()))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    /// A kinematic body parked above a pile keeps a non-touching contact to a body that falls
    /// asleep, the one case the locator cannot name.
    // TODO(Natsel): a kinematic body driven down onto a sleeping island does not produce the
    // pair that would wake it. Needs a cross-seed audit for a moved kinematic proxy.
    @Test
    func kinematic() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            _ = createBody(world, .staticBody, Pos(0, -1, 0), 5, 1, 5)
            let box = createBody(world, .dynamicBody, Pos(0, Float(0.5), 0), 0.5, 0.5, 0.5)

            let plate = createBody(world, .kinematicBody, Pos(0, Float(1.52), 0), 1, 0.5, 1)

            let steps = stepUntilAsleep(world, box, 300)
            #expect(steps < 300)

            // Drive the plate down to touch and wake the sleeper.
            world.setBodyLinearVelocity(plate, Vec3(0, -1, 0))
            step(world, 60)

            #expect(world.isBodyAwake(box))
            samples.append(UInt64(world.worldStateHash()))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test
    func transfer() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            _ = createBody(world, .staticBody, Pos(0, -1, 0), 20, 1, 5)

            let box = createBody(world, .dynamicBody, Pos(0, Float(0.5), 0), 0.5, 0.5, 0.5)
            step(world, 30)

            world.setBodyType(box, .staticBody)
            step(world, 5)
            world.setBodyType(box, .dynamicBody)
            step(world, 5)

            world.disableBody(box)
            step(world, 5)
            world.enableBody(box)
            step(world, 30)

            // Two piles far enough apart to sleep in separate sets.
            let sleeperA = createBody(world, .dynamicBody, Pos(-12, Float(0.5), 0), 0.5, 0.5, 0.5)
            let sleeperB = createBody(world, .dynamicBody, Pos(12, Float(0.5), 0), 0.5, 0.5, 0.5)

            let steps = stepUntilAsleep(world, sleeperA, 300)
            #expect(steps < 300)
            #expect(world.isBodyAwake(sleeperB) == false)

            var jointDef = SphericalJointDef()
            jointDef.base.bodyIdA = sleeperA
            jointDef.base.bodyIdB = sleeperB
            _ = world.createSphericalJoint(jointDef)
            step(world, 30)

            samples.append(UInt64(world.worldStateHash()))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    /// The fast flag lives on the body sim and gates contact recycling, but a flag sync rebuilds
    /// the sim flags from the body row. A public setter called on a body in flight must not drop it.
    @Test("fast flag survives a motion lock change")
    func fastFlag() {
        let world = World()
        _ = createBody(world, .staticBody, Pos(0, -1, 0), 5, 1, 5)

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(-20, 2, 0)
        def.linearVelocity = Vec3(400, 0, 0)
        let bullet = world.createBody(def)

        var shapeDef = ShapeDef()
        _ = world.createSphereShape(bullet, shapeDef, Sphere(center: Vec3(), radius: 0.1))

        world.step(1.0 / 60.0, subStepCount: 4)

        #expect(world.isBodyFast(bullet))

        // Any public setter that syncs flags. The locks have to actually change or the setter
        // returns early and never syncs.
        var locks = MotionLocks()
        locks.angularX = true
        world.setBodyMotionLocks(bullet, locks)

        #expect(world.isBodyFast(bullet))

        world.step(1.0 / 60.0, subStepCount: 4)
    }
}

// MARK: - Graph index invariants

@Suite("Constraint graph index invariants")
struct GraphIndexInvariantTests {
    /// Every contact in every color must agree with its cached color and local index. A mismatch
    /// means a removal swapped the array without fixing the moved contact, which corrupts the
    /// solver's per-color iteration.
    private func assertGraphConsistent(_ world: World, _ label: String) {
        if let error = world.validateGraphIndices() {
            Issue.record("\(label): \(error)")
        }
    }

    @Test("graph indices stay consistent through stacking and sleeping")
    func stackingLifecycle() {
        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        for i in 0..<4 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 0.55 + Float(i) * 1.05, 0)
            let box = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        }

        for step in 0..<300 {
            world.step(1.0 / 60.0, subStepCount: 4)
            if step % 50 == 0 {
                assertGraphConsistent(world, "step \(step)")
            }
        }
        assertGraphConsistent(world, "settled")
    }

    @Test("graph indices stay consistent through disable and enable")
    func disableEnableLifecycle() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        var boxes: [BodyID] = []
        for i in 0..<3 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(Float(i) * 0.9 - 0.9, Float(0.6), 0)
            let box = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
            boxes.append(box)
        }

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }
        assertGraphConsistent(world, "before disable")

        world.disableBody(boxes[1])
        assertGraphConsistent(world, "after disable")
        for _ in 0..<10 { world.step(1.0 / 60.0, subStepCount: 4) }
        assertGraphConsistent(world, "disabled stepped")

        world.enableBody(boxes[1])
        assertGraphConsistent(world, "after enable")
        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }
        assertGraphConsistent(world, "re-enabled stepped")
    }

    @Test("graph indices stay consistent through a rolling sphere")
    func rollingSphere() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(20, 0.5, 20))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 1, 0)
        def.linearVelocity = Vec3(5, 0, 0)
        let ball = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createSphereShape(ball, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        for step in 0..<200 {
            world.step(1.0 / 60.0, subStepCount: 4)
            if step % 25 == 0 { assertGraphConsistent(world, "roll \(step)") }
        }
    }
}
