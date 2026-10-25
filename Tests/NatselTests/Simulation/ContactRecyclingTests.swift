import Testing
@testable import Natsel

// Contact recycling edge cases from test/test_world.c.
@Suite("Contact recycling")
struct ContactRecyclingTests {
    @Test("recycling flag round trips and syncs to the sim")
    func recyclingFlag() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        let bodyA = world.createBody(def)
        #expect(world.isBodyContactRecyclingEnabled(bodyA))

        world.enableBodyContactRecycling(bodyA, false)
        #expect(world.isBodyContactRecyclingEnabled(bodyA) == false)

        world.enableBodyContactRecycling(bodyA, true)
        #expect(world.isBodyContactRecyclingEnabled(bodyA))

        // Per-def opt-out at creation
        def.enableContactRecycling = false
        let bodyB = world.createBody(def)
        #expect(world.isBodyContactRecyclingEnabled(bodyB) == false)

        // Stepping after toggling must not trip the flag-sync validator
        world.step(1.0 / 60.0, subStepCount: 4)
    }

    /// A slow sphere closes on a static round target from just outside contact. The step is too
    /// short to make the sphere fast, so the pair is eligible for recycling. Without a speculative
    /// margin the pair went empty and recycling then let the sphere sink through.
    private func runRecycledApproach(capsuleTarget: Bool) -> Float {
        let world = World()
        world.gravity = Vec3()

        var def = BodyDef()
        def.type = .staticBody
        let target = world.createBody(def)

        var shapeDef = ShapeDef()
        let targetRadius: Float = 0.5
        if capsuleTarget {
            _ = world.createCapsuleShape(
                target, shapeDef,
                Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: targetRadius)
            )
        } else {
            _ = world.createSphereShape(
                target, shapeDef, Sphere(center: Vec3(), radius: targetRadius)
            )
        }

        let radius: Float = 0.25
        let totalRadius = targetRadius + radius
        let timeStep: Float = 1.0 / 60.0

        // Half the speculative distance per step
        let speed = 0.5 * B3.speculativeDistance / timeStep

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, totalRadius + 0.25 * B3.speculativeDistance, 0)
        def.linearVelocity = Vec3(0, -speed, 0)
        let body = world.createBody(def)
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: radius))

        var minSeparation = Float.greatestFiniteMagnitude
        for _ in 0..<20 {
            world.step(timeStep, subStepCount: 4)
            let p = world.bodyTransform(body).p
            minSeparation = minFloat(minSeparation, Float(p.y) - totalRadius)
        }

        return minSeparation
    }

    @Test("recycling does not let a slow sphere sink into a sphere target")
    func recycledSphereApproach() {
        let minSeparation = runRecycledApproach(capsuleTarget: false)
        #expect(minSeparation > -B3.linearSlop)
        #expect(absFloat(minSeparation) < 1e-4)
    }

    @Test("recycling does not let a slow sphere sink into a capsule target")
    func recycledCapsuleApproach() {
        let minSeparation = runRecycledApproach(capsuleTarget: true)
        #expect(minSeparation > -B3.linearSlop)
        #expect(absFloat(minSeparation) < 1e-4)
    }

    @Test("a recycled pair keeps producing contacts while resting")
    func restingContactPersists() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, Float(1.0), 0)
        let ball = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createSphereShape(ball, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        for _ in 0..<180 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        // The ball rests on the ground: the contact survived recycling every step.
        #expect(absFloat(world.bodyTransform(ball).p.y - 1.0) < 0.01)
        #expect(world.contactCount > 0)
    }
}

// MARK: - Enlarged proxy lifetime (C issue #149)

@Suite("Enlarged proxy lifetime")
struct EnlargedProxyLifetimeTests {
    private func countMovedNodes(_ tree: DynamicTree) -> Int {
        var count = 0
        for i in 0..<Int(tree.nodeEnd) {
            let node = tree.nodes[i]
            if isEmptyNode(node) == false && isNodeMoved(node) {
                count += 1
            }
        }
        return count
    }

    @Test("a proxy enlarged then destroyed leaves the tree consistent")
    func enlargedProxyDestroyed() {
        let world = World()
        world.gravity = Vec3()

        let tree = world.broadPhase.trees[Int(BodyType.dynamicBody.rawValue)]

        var def = BodyDef()
        def.type = .dynamicBody

        var shapeDef = ShapeDef()
        let sphere = Sphere(center: Vec3(), radius: 0.25)

        // Resting bodies keep the tree deep enough that the mover has an ancestor that outlives it
        for i in 0..<8 {
            def.position = Pos(2 * Float(i), 0, 0)
            def.linearVelocity = Vec3()
            let resting = world.createBody(def)
            _ = world.createSphereShape(resting, shapeDef, sphere)
        }

        // This body outruns its AABB margin every step
        def.position = Pos(7, 1, 0)
        def.linearVelocity = Vec3(6, 0, 0)
        let mover = world.createBody(def)
        _ = world.createSphereShape(mover, shapeDef, sphere)

        let timeStep: Float = 1.0 / 60.0
        world.step(timeStep, subStepCount: 4)

        #expect(countMovedNodes(tree) > 0)

        world.destroyBody(mover)

        // Removing the leaf re-derives the marks along its path, so the tree is already consistent
        // here and nothing is left marked to drive a rebuild.
        tree.validate()
        #expect(countMovedNodes(tree) == 0)

        world.step(timeStep, subStepCount: 4)

        #expect(countMovedNodes(tree) == 0)
        #expect(tree.dfsOrdered)
    }
}

// MARK: - Flag synchronization

@Suite("Body flag synchronization")
struct BodyFlagSyncTests {
    @Test("sleep flag round trips and survives a step")
    func sleepFlagSync() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        def.enableSleep = false
        let body = world.createBody(def)

        #expect(world.isBodySleepEnabled(body) == false)

        world.enableBodySleep(body, true)
        #expect(world.isBodySleepEnabled(body))

        world.step(1.0 / 60.0, subStepCount: 4)
        #expect(world.isBodySleepEnabled(body))
    }

    @Test("motion locks do not wipe the bullet flag")
    func bulletSurvivesMotionLocks() {
        let world = World()

        // Reproducer A: create non-bullet, set bullet, then change motion locks.
        var def = BodyDef()
        def.type = .dynamicBody
        def.isBullet = false
        let bodyA = world.createBody(def)

        #expect(world.isBodyBullet(bodyA) == false)
        world.setBodyBullet(bodyA, true)
        #expect(world.isBodyBullet(bodyA))

        var locks = MotionLocks()
        locks.linearX = true
        world.setBodyMotionLocks(bodyA, locks)

        #expect(world.isBodyBullet(bodyA))
        #expect(world.bodyMotionLocks(bodyA).linearX)

        // Reproducer B: create bullet, clear it, then change motion locks.
        def = BodyDef()
        def.type = .dynamicBody
        def.isBullet = true
        let bodyB = world.createBody(def)

        #expect(world.isBodyBullet(bodyB))
        world.setBodyBullet(bodyB, false)
        #expect(world.isBodyBullet(bodyB) == false)

        world.setBodyMotionLocks(bodyB, locks)
        #expect(world.isBodyBullet(bodyB) == false)
    }

    @Test("bullet flag survives a step")
    func bulletAfterStep() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 5, 0)
        def.linearVelocity = Vec3(20, 0, 0)
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        world.setBodyBullet(body, true)
        for _ in 0..<5 { world.step(1.0 / 60.0, subStepCount: 4) }

        #expect(world.isBodyBullet(body))
    }
}
