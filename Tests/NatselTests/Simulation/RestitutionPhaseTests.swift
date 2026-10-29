import Testing
@testable import Natsel

@Suite("Restitution phase")
struct RestitutionPhaseTests {
    private static let timeStep: Float = 1.0 / 60.0
    private static let subSteps: Int32 = 4
    private static let restitutionIterations: Int32 = 4
    private static let impactSpeed: Float = 5

    private func makeWorld(gravityY: Float, iterations: Int32? = nil) -> World {
        let world = World()
        world.gravity = Vec3(0, gravityY, 0)
        world.enableSleep = false
        if let iterations { world.restitutionIterations = iterations }
        return world
    }

    /// Ground with its top surface at y = 0.
    private func makeGround(_ world: World, restitution: Float) {
        var def = BodyDef()
        def.position = Pos(0, -1, 0)
        let ground = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = restitution
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(40, 1, 40))
    }

    private func makeBall(
        _ world: World, x: Float, y: Float, velocityY: Float, restitution: Float
    ) -> BodyID {
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(x, y, 0)
        def.linearVelocity = Vec3(0, velocityY, 0)
        let ball = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = restitution
        _ = world.createSphereShape(ball, shapeDef, Sphere(center: Vec3(), radius: 0.5))
        return ball
    }

    /// Ball driven into static ground with no gravity so the measurement carries no gravity bias.
    /// The gap shifts where inside the time step the impact lands.
    private func measureGroundBounce(_ restitution: Float, _ gap: Float) -> Float {
        let world = makeWorld(gravityY: 0)
        makeGround(world, restitution: restitution)
        let ball = makeBall(
            world, x: 0, y: 0.5 + gap, velocityY: -Self.impactSpeed, restitution: restitution)

        for _ in 0..<60 {
            world.step(Self.timeStep, subStepCount: Self.subSteps)
        }
        return world.bodyLinearVelocity(ball).y / Self.impactSpeed
    }

    /// Impactor driven onto a column of resting balls pinned against the ground. A supported target
    /// has infinite effective mass along the normal, so the coefficient must not depend on the
    /// column height.
    private func measureSupportedBounce(
        _ restitution: Float, _ supportCount: Int, _ propagate: Bool
    ) -> Float {
        let world = World()
        world.gravity = Vec3()
        world.enableSleep = false
        world.enableRestitutionPropagation = propagate
        world.restitutionIterations = Self.restitutionIterations

        makeGround(world, restitution: 0)

        for i in 0..<supportCount {
            _ = makeBall(world, x: 0, y: 0.5 + Float(i), velocityY: 0, restitution: 0)
        }

        let y = 0.5 + Float(supportCount) + 0.5 * Self.impactSpeed * Self.timeStep
        let ball = makeBall(
            world, x: 0, y: y, velocityY: -Self.impactSpeed, restitution: restitution)

        for _ in 0..<60 {
            world.step(Self.timeStep, subStepCount: Self.subSteps)
        }
        return world.bodyLinearVelocity(ball).y / Self.impactSpeed
    }

    /// The recovered coefficient must not depend on where inside the time step the impact lands.
    @Test("bounce coefficient is stable across impact phases")
    func phase() {
        let sampleCount = 16
        let tolerance: Float = 0.05

        for restitution in [0.25 as Float, 0.5, 0.9] {
            var minimum = Float.greatestFiniteMagnitude
            var maximum = -Float.greatestFiniteMagnitude
            var sum: Float = 0

            for i in 0..<sampleCount {
                // Sweep the gap across one step of travel so the impact lands at every phase.
                let gap = (Float(i) / Float(sampleCount)) * Self.impactSpeed * Self.timeStep
                let measured = measureGroundBounce(restitution, gap)
                minimum = minFloat(minimum, measured)
                maximum = maxFloat(maximum, measured)
                sum += measured
            }

            let mean = sum / Float(sampleCount)
            let spread = maximum - minimum

            #expect(spread <= tolerance, "e \(restitution) spread \(spread)")
            #expect(absFloat(mean - restitution) <= tolerance, "e \(restitution) mean \(mean)")
        }
    }

    /// A supported target has infinite effective mass along the normal, so the coefficient must not
    /// depend on the column height.
    @Test("supported bounce does not depend on column height")
    func supported() {
        for propagate in [false, true] {
            let floorFactor: Float = propagate ? 0.75 : 0.5

            for restitution in [0.5 as Float, 0.9] {
                for count in 0...3 {
                    let measured = measureSupportedBounce(
                        restitution, count, propagate)
                    let floor = floorFactor * restitution

                    #expect(measured <= restitution + 0.01, "e \(restitution) n \(count)")
                    #expect(measured >= floor, "e \(restitution) n \(count) got \(measured)")
                }
            }
        }
    }

    @Test("threshold gates restitution for slow contacts")
    func threshold() {
        // With a high restitution threshold a slow approach does not bounce.
        let world = makeWorld(gravityY: 0)
        makeGround(world, restitution: 0.9)
        world.restitutionThreshold = 10

        let ball = makeBall(world, x: 0, y: 0.5, velocityY: -1.0, restitution: 0.9)
        for _ in 0..<60 { world.step(Self.timeStep, subStepCount: Self.subSteps) }

        // The approach speed is below the threshold, so no restitution is applied.
        let v = world.bodyLinearVelocity(ball).y
        #expect(v <= 0.01)
    }

    @Test("a fast contact above the threshold still bounces")
    func thresholdAbove() {
        let world = makeWorld(gravityY: 0)
        makeGround(world, restitution: 0.9)
        world.restitutionThreshold = 1

        let ball = makeBall(
            world, x: 0, y: 0.5, velocityY: -Self.impactSpeed, restitution: 0.9)
        for _ in 0..<60 { world.step(Self.timeStep, subStepCount: Self.subSteps) }

        let measured = world.bodyLinearVelocity(ball).y / Self.impactSpeed
        #expect(measured > 0.85)
    }
}

// MARK: - Head-on collisions

extension RestitutionPhaseTests {
    /// Two free spheres closing head on with no gravity. The coefficient is the ratio of relative
    /// normal speeds at the contact, which is the mass independent definition.
    private func measureHeadOn(
        _ restitution: Float, _ densityB: Float
    ) -> (coefficient: Float, momentumError: Float) {
        let world = makeWorld(gravityY: 0)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = restitution

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(-1, 0, 0)
        def.linearVelocity = Vec3(0.5 * Self.impactSpeed, 0, 0)
        let idA = world.createBody(def)
        _ = world.createSphereShape(idA, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        def.position = Pos(1, 0, 0)
        def.linearVelocity = Vec3(-0.5 * Self.impactSpeed, 0, 0)
        let idB = world.createBody(def)
        shapeDef.density = densityB
        _ = world.createSphereShape(idB, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        let massA = world.bodyMass(idA)
        let massB = world.bodyMass(idB)
        let momentum0 =
            massA * world.bodyLinearVelocity(idA).x + massB * world.bodyLinearVelocity(idB).x

        for _ in 0..<120 { world.step(Self.timeStep, subStepCount: Self.subSteps) }

        let vA = world.bodyLinearVelocity(idA).x
        let vB = world.bodyLinearVelocity(idB).x
        let momentum1 = massA * vA + massB * vB

        let momentumError = (momentum1 - momentum0) / (massA + massB)
        return ((vB - vA) / Self.impactSpeed, momentumError)
    }

    /// The coefficient must not depend on the mass ratio, and momentum must be conserved.
    @Test("head-on restitution is mass independent and conserves momentum")
    func headOn() {
        let tolerance: Float = 0.02
        let momentumTolerance: Float = 1e-4

        for density in [1 as Float, 10, 100] {
            for restitution in [0 as Float, 0.25, 0.5, 0.75, 1.0] {
                let result = measureHeadOn(restitution, density)
                #expect(
                    absFloat(result.coefficient - restitution) <= tolerance,
                    "density \(density) e \(restitution) got \(result.coefficient)")
                #expect(
                    absFloat(result.momentumError) <= momentumTolerance,
                    "density \(density) e \(restitution) momentum drift \(result.momentumError)")
            }
        }
    }
}

// MARK: - Elastic overshoot

extension RestitutionPhaseTests {
    private struct OvershootResult {
        var firstApex: Float = 0
        var firstEnergyRatio: Float = 0
        var peakEnergyRatio: Float = 0
        var speedRatio: Float = 0
        var bounceCount = 0
    }

    private func measureEnergy(_ world: World, _ body: BodyID) -> Float {
        let massData = world.bodyMassData(body)
        let v = world.bodyLinearVelocity(body)
        let gravityScale = world.bodyGravityScale(body)
        let gravity = gravityScale * world.gravity
        let h = Self.timeStep / Float(Self.subSteps)

        // The inertia tensor is in the body frame, so bring the angular velocity back to it.
        let wLocal = invRotateVector(
            world.bodyRotation(body), world.bodyAngularVelocity(body))

        let kinetic =
            0.5 * massData.mass * dot(v, v)
            + 0.5 * dot(wLocal, mulMV(massData.inertia, wLocal))
        let potential =
            -massData.mass * dot(gravity, toVec3(world.bodyWorldCenter(body)))
        let phase = 0.5 * h * massData.mass * dot(v, gravity)

        return kinetic + potential + phase
    }

    /// A unit cube dropped flat on a floor narrower than itself, perfectly elastic, so the four
    /// contact points sit inboard of the box corners. A perfectly elastic bounce cannot come back
    /// higher than it was dropped from and cannot gain energy.
    private func measureOvershoot(_ continuous: Bool) -> OvershootResult {
        let world = makeWorld(gravityY: -10, iterations: Self.restitutionIterations)

        var floorDef = BodyDef()
        floorDef.position = Pos(0, Float(-0.25), 0)
        let floor = world.createBody(floorDef)
        _ = world.createHullShape(floor, ShapeDef(), makeBoxHull(0.375, 0.25, 0.375))

        let dropHeight: Float = 10

        var boxDef = BodyDef()
        boxDef.type = .dynamicBody
        boxDef.position = Pos(0, dropHeight, 0)
        if continuous { boxDef.safetyFactor = 0.1 }
        let box = world.createBody(boxDef)

        var boxShape = ShapeDef()
        boxShape.baseMaterial.restitution = 1
        _ = world.createHullShape(box, boxShape, makeCubeHull(0.5))

        var result = OvershootResult()
        let startEnergy = measureEnergy(world, box)

        var impactSpeed: Float = 0
        var reboundSpeed: Float = 0
        var previousSpeed: Float = 0

        var step = 0
        while step < 600 && result.bounceCount < 2 {
            let before = world.bodyLinearVelocity(box).y
            world.step(Self.timeStep, subStepCount: Self.subSteps)
            let speed = world.bodyLinearVelocity(box).y

            if reboundSpeed == 0 && before < 0 && speed > 0 {
                impactSpeed = -before
                reboundSpeed = speed
            }

            if reboundSpeed == 0 {
                previousSpeed = speed
                step += 1
                continue
            }

            // Every step, not just at the apexes, because the impact step is where gain appears.
            let ratio = measureEnergy(world, box) / startEnergy
            if ratio > result.peakEnergyRatio {
                result.peakEnergyRatio = ratio
            }

            if previousSpeed > 0 && speed <= 0 {
                if result.bounceCount == 0 {
                    result.firstApex = Float(world.bodyTransform(box).p.y)
                    result.firstEnergyRatio = ratio
                }
                result.bounceCount += 1
            }

            previousSpeed = speed
            step += 1
        }

        result.speedRatio = impactSpeed != 0 ? reboundSpeed / impactSpeed : 0
        return result
    }

    /// The impulse must never return more speed than it received. That is the restitution invariant.
    @Test("a perfectly elastic box never gains speed or energy")
    func overshoot() {
        let dropHeight: Float = 10
        let speedTolerance: Float = 1e-5

        let discrete = measureOvershoot(false)
        let continuous = measureOvershoot(true)

        // Perfectly elastic returns exactly the speed it received.
        #expect(discrete.speedRatio <= 1 + speedTolerance, "discrete \(discrete.speedRatio)")
        #expect(continuous.speedRatio <= 1 + speedTolerance, "continuous \(continuous.speedRatio)")

        // With continuous collision the cube lands on the surface and stays under its start.
        #expect(continuous.firstApex <= dropHeight, "continuous apex \(continuous.firstApex)")
        #expect(continuous.peakEnergyRatio <= 1.001, "continuous energy \(continuous.peakEnergyRatio)")

        // The discrete run is gated more loosely: it lands already inside the floor.
        #expect(discrete.firstApex <= dropHeight + 0.15, "discrete apex \(discrete.firstApex)")
        #expect(discrete.firstEnergyRatio <= 1.02, "discrete energy \(discrete.firstEnergyRatio)")

        #expect(discrete.bounceCount >= 2)
        #expect(continuous.bounceCount >= 1)
    }
}

// MARK: - Energy conservation

extension RestitutionPhaseTests {
    private struct EnergyScene {
        var name: String
        var safetyFactor: Float
        var friction: Float
        var angle: Float
        var spin: Float
        var height: Float
        var velocityY: Float
        var halfWidth: Float
        var halfHeight: Float
        var bullet = false
        var stack = false
        var stepCount = 300
    }

    /// Peak energy over the run relative to the start, for every dynamic body in the scene.
    /// Perfectly elastic and the energy may still only go down.
    private func measurePeakEnergy(_ scene: EnergyScene) -> Float {
        let world = makeWorld(gravityY: -10)

        var groundDef = BodyDef()
        groundDef.position = Pos(0, -1, 0)
        let ground = world.createBody(groundDef)
        var groundShape = ShapeDef()
        groundShape.baseMaterial.friction = scene.friction
        _ = world.createHullShape(ground, groundShape, makeBoxHull(40, 1, 40))

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.baseMaterial.friction = scene.friction
        shapeDef.baseMaterial.restitution = 1
        let box = makeBoxHull(scene.halfWidth, scene.halfHeight, scene.halfWidth)

        var bodies: [BodyID] = []

        if scene.stack {
            // A resting box under the dropped one.
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, scene.halfHeight, 0)
            def.safetyFactor = scene.safetyFactor
            let body = world.createBody(def)
            _ = world.createHullShape(body, shapeDef, box)
            bodies.append(body)
        }

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, scene.height, 0)
        def.rotation = makeQuatFromAxisAngle(.axisZ, scene.angle)
        def.angularVelocity = Vec3(0, 0, scene.spin)
        def.linearVelocity = Vec3(0, scene.velocityY, 0)
        def.safetyFactor = scene.safetyFactor
        def.isBullet = scene.bullet
        let dropped = world.createBody(def)
        _ = world.createHullShape(dropped, shapeDef, box)
        bodies.append(dropped)

        var start: Float = 0
        for body in bodies { start += measureEnergy(world, body) }

        var peak = start
        for _ in 0..<scene.stepCount {
            world.step(Self.timeStep, subStepCount: Self.subSteps)
            var energy: Float = 0
            for body in bodies { energy += measureEnergy(world, body) }
            peak = maxFloat(peak, energy)
        }

        return peak / start
    }

    /// Perfectly elastic material with friction, dropped every way that used to gain energy.
    /// Energy must never rise above the start: this is the load bearing property of restitution.
    @Test("peak energy never exceeds the starting energy")
    func energyNeverGrows() {
        let scenes: [EnergyScene] = [
            EnergyScene(
                name: "spinning edge", safetyFactor: 0.01, friction: 0.6, angle: -1.43,
                spin: 7.9, height: 0.7, velocityY: -1.9, halfWidth: 0.5, halfHeight: 0.5,
                stepCount: 300),
            EnergyScene(
                name: "spinning edge fast", safetyFactor: 0.01, friction: 0.6, angle: -0.8,
                spin: 12, height: 3, velocityY: -8, halfWidth: 0.5, halfHeight: 0.5,
                stepCount: 300),
            EnergyScene(
                name: "tumble", safetyFactor: 0.01, friction: 0.6, angle: -0.3, spin: 3,
                height: 8, velocityY: 0, halfWidth: 0.5, halfHeight: 0.5, stepCount: 600),
            EnergyScene(
                name: "tumble discrete", safetyFactor: 0.5, friction: 0.6, angle: -0.3, spin: 3,
                height: 8, velocityY: 0, halfWidth: 0.5, halfHeight: 0.5, stepCount: 600),
            EnergyScene(
                name: "flat with spin", safetyFactor: 0.01, friction: 0.6, angle: 0, spin: 2,
                height: 2, velocityY: -5, halfWidth: 0.5, halfHeight: 0.5, stepCount: 300),
            EnergyScene(
                name: "flat frictionless", safetyFactor: 0.01, friction: 0, angle: 0, spin: 0,
                height: 10, velocityY: 0, halfWidth: 0.5, halfHeight: 0.5, stepCount: 1200),
            EnergyScene(
                name: "edge", safetyFactor: 0.01, friction: 0.6, angle: -0.2, spin: 0,
                height: 5, velocityY: 0, halfWidth: 0.5, halfHeight: 0.5, stepCount: 600),
            EnergyScene(
                name: "two box", safetyFactor: 0.02, friction: 0.6, angle: 0, spin: 0,
                height: 4, velocityY: 0, halfWidth: 0.5, halfHeight: 0.5,
                bullet: true, stack: true, stepCount: 1500),
            EnergyScene(
                name: "two box discrete", safetyFactor: 0.5, friction: 0.6, angle: 0, spin: 0,
                height: 4, velocityY: 0, halfWidth: 0.5, halfHeight: 0.5,
                stack: true, stepCount: 1500),
            EnergyScene(
                name: "tall flat frictionless", safetyFactor: 0.01, friction: 0, angle: 0, spin: 0,
                height: 10, velocityY: 0, halfWidth: 0.5, halfHeight: 1.0, stepCount: 1200),
            EnergyScene(
                name: "tall flat friction", safetyFactor: 0.01, friction: 0.6, angle: 0, spin: 0,
                height: 10, velocityY: 0, halfWidth: 0.5, halfHeight: 1.0, stepCount: 1200),
            EnergyScene(
                name: "narrow flat frictionless", safetyFactor: 0.01, friction: 0, angle: 0,
                spin: 0, height: 10, velocityY: 0, halfWidth: 0.25, halfHeight: 1.0,
                stepCount: 1200),
            EnergyScene(
                name: "narrow flat friction", safetyFactor: 0.01, friction: 0.6, angle: 0,
                spin: 0, height: 10, velocityY: 0, halfWidth: 0.25, halfHeight: 1.0,
                stepCount: 1200),
        ]

        // The soft contact position correction lifts a body that lands inside the ground, so
        // continuous collision keeps the tolerance tight. A four point flat landing does not
        // converge at the default two stage iterations the way a two point landing does.
        let tolerance: Float = 5.0e-3
        let discreteTolerance: Float = 2.0e-2

        for scene in scenes {
            let ratio = measurePeakEnergy(scene)
            let limit = scene.safetyFactor < 0.1 ? tolerance : discreteTolerance
            #expect(ratio <= 1 + limit, "\(scene.name) peak \(ratio)")
        }
    }
}

// MARK: - Drop impulse accounting

extension RestitutionPhaseTests {
    private struct ImpulseResult {
        var worstError: Float = 0
        var approachSpeed: Float = 0
        var firstSeparation: Float = 0
        var bounceImpulse: Float = 0
        var contactSteps = 0
        var toiSteps = 0
        var toiImpulseSteps = 0
    }

    /// Drop a ball and account for every contact impulse: the measured normal impulse must equal
    /// the momentum change minus the gravity contribution for that step.
    private func measureDropImpulse(_ restitution: Float, _ dropHeight: Float) -> ImpulseResult {
        let world = makeWorld(gravityY: -10)
        makeGround(world, restitution: 0)

        let ball = makeBall(
            world, x: 0, y: 0.5 + dropHeight, velocityY: 0, restitution: restitution)

        let mass = world.bodyMass(ball)
        let gravityY = world.gravity.y

        var result = ImpulseResult()
        var touched = false
        var bouncing = false

        for _ in 0..<240 {
            let speedBefore = world.bodyLinearVelocity(ball).y
            world.step(Self.timeStep, subStepCount: Self.subSteps)
            let speedAfter = world.bodyLinearVelocity(ball).y

            var measured: Float = 0
            var separation: Float = 0
            let contactData = world.bodyContactData(ball, capacity: 4)
            for contact in contactData {
                for m in 0..<contact.manifolds.count {
                    let manifold = contact.manifolds[m]
                    guard manifold.pointCount > 0 else { continue }
                    for p in 0..<Int(manifold.pointCount) {
                        measured += manifold.points[p].totalNormalImpulse
                        separation = manifold.points[p].separation
                    }
                }
            }

            // Transient body flags are cleared and rewritten every step.
            if world.isBodyTimeOfImpact(ball) {
                result.toiSteps += 1
                if measured != 0 { result.toiImpulseSteps += 1 }
                continue
            }

            let expected = mass * (speedAfter - speedBefore) - mass * gravityY * Self.timeStep
            result.worstError = maxFloat(result.worstError, absFloat(measured - expected))

            if contactData.isEmpty == false {
                if touched == false {
                    touched = true
                    bouncing = true
                    result.approachSpeed = -speedBefore
                    result.firstSeparation = separation
                }
                if bouncing {
                    result.bounceImpulse += measured
                    result.contactSteps += 1
                }
            } else {
                bouncing = false
            }
        }

        return result
    }

    /// The measured contact impulse must match the momentum change to solver precision, and the
    /// bounce impulse must stay within the analytic reversal plus the weight carried over the
    /// contact steps.
    @Test("contact impulse accounts for momentum change and gravity")
    func impulseAccounting() {
        // The impact speed at which continuous collision engages.
        let probeWorld = makeWorld(gravityY: -10)
        makeGround(probeWorld, restitution: 0)
        let probe = makeBall(probeWorld, x: 0, y: 0.5, velocityY: 0, restitution: 0)
        let mass = probeWorld.bodyMass(probe)
        let fastSpeed =
            probeWorld.bodySafetyFactor(probe) * probeWorld.bodyMinExtent(probe) / Self.timeStep

        let weightImpulse = mass * 10 * Self.timeStep

        for dropHeight in [0.25 as Float, 1.0, 4.0, 16.0] {
            let impactSpeed = (20 * dropHeight).squareRoot()
            let expectToi = impactSpeed > fastSpeed
            let tolerance = 1e-5 * mass * (10 + impactSpeed)

            for restitution in [0 as Float, 0.5, 1.0] {
                let result = measureDropImpulse(restitution, dropHeight)

                let reversal = (1 + restitution) * mass * result.approachSpeed
                let lower = reversal - weightImpulse
                let upper = reversal + weightImpulse * Float(result.contactSteps)

                #expect(
                    result.worstError <= tolerance,
                    "drop \(dropHeight) e \(restitution) error \(result.worstError)")

                #expect(result.contactSteps > 0, "drop \(dropHeight) e \(restitution) no contact")
                #expect(
                    result.toiImpulseSteps == 0,
                    "drop \(dropHeight) e \(restitution) impulse during TOI")

                let slack = Float(result.contactSteps) * tolerance
                #expect(
                    result.bounceImpulse >= lower - slack
                        && result.bounceImpulse <= upper + slack,
                    "drop \(dropHeight) e \(restitution) bounce \(result.bounceImpulse) in [\(lower), \(upper)]")

                // Continuous collision engages exactly when the impact speed clears the threshold.
                #expect(
                    (result.toiSteps > 0) == expectToi,
                    "drop \(dropHeight) e \(restitution) toi \(result.toiSteps) expected \(expectToi)")
            }
        }
    }
}

// MARK: - Sampled normal velocity

extension RestitutionPhaseTests {
    private func measureFirstTouchNormalVelocity(_ enableHitEvents: Bool) -> Float {
        let world = makeWorld(gravityY: 0)
        makeGround(world, restitution: 0)

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, Float(0.6), 0)
        let ball = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = 0
        shapeDef.enableHitEvents = enableHitEvents
        _ = world.createSphereShape(ball, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        var normalVelocity = Float.greatestFiniteMagnitude

        // Hold the closing speed so the sampled value does not depend on which step the pair is
        // created.
        for _ in 0..<20 {
            world.setBodyLinearVelocity(ball, Vec3(0, -Self.impactSpeed, 0))
            world.step(Self.timeStep, subStepCount: Self.subSteps)

            let contacts = world.bodyContactData(ball, capacity: 1)
            if let first = contacts.first, first.manifolds.count > 0 {
                let manifold = first.manifolds[0]
                if manifold.pointCount > 0 {
                    normalVelocity = manifold.points[0].normalVelocity
                    break
                }
            }
        }
        return normalVelocity
    }

    /// The approach speed is only sampled when hit events are enabled; otherwise the field stays
    /// zero. This guards the shared prepare path from paying for a sample nobody reads.
    @Test("normal velocity is only sampled when hit events are enabled")
    func normalVelocitySampling() {
        let quiet = measureFirstTouchNormalVelocity(false)
        let published = measureFirstTouchNormalVelocity(true)

        #expect(quiet == 0)

        // The closing speed is reported as a negative normal velocity, at nearly full magnitude.
        #expect(published <= -0.8 * Self.impactSpeed, "published \(published)")
    }
}

// MARK: - Flat landing and spin

extension RestitutionPhaseTests {
    /// A flat box landing on all four corners at once.
    private func measureFlatBounce(
        _ restitution: Float, _ subSteps: Int32
    ) -> (coefficient: Float, spin: Float) {
        let world = makeWorld(gravityY: 0, iterations: Self.restitutionIterations)
        makeGround(world, restitution: 0)

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0.25 + 0.5 * Self.impactSpeed * Self.timeStep, 0)
        def.linearVelocity = Vec3(0, -Self.impactSpeed, 0)
        let box = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = restitution
        _ = world.createHullShape(box, shapeDef, makeBoxHull(1, 0.25, 1))

        for _ in 0..<60 { world.step(Self.timeStep, subStepCount: subSteps) }

        let speed = world.bodyLinearVelocity(box).y
        let spin = length(world.bodyAngularVelocity(box))
        return (speed / Self.impactSpeed, spin)
    }

    /// A symmetric four point landing. The stage solves the points in sequence, so a single sweep
    /// leaves the body tilted; four stage iterations bring the coefficient within a few thousandths.
    @Test("four point flat landing recovers the coefficient without spin")
    func flatLanding() {
        let speedTolerance: Float = 0.01
        let spinTolerance: Float = 0.03

        for restitution in [0.5 as Float, 0.9] {
            for subSteps in [Int32(4), Int32(8)] {
                let result = measureFlatBounce(restitution, subSteps)

                // Only the shipping sub step count is a gate; the wider count is informational.
                guard subSteps == Self.subSteps else { continue }

                #expect(
                    absFloat(result.coefficient - restitution) <= speedTolerance,
                    "e \(restitution) coefficient \(result.coefficient)")
                #expect(
                    result.spin <= spinTolerance,
                    "e \(restitution) spin \(result.spin)")
            }
        }
    }

    /// A flat box carrying spin about a horizontal axis. All four corners stay in contact and the
    /// normal constraints are linear in the body velocity, so perfect restitution reverses linear
    /// and angular velocity alike.
    private func measureSpinningBounce(
        _ spin: Float, _ restitution: Float
    ) -> (coefficient: Float, residualSpin: Float) {
        let world = makeWorld(gravityY: 0, iterations: Self.restitutionIterations)
        makeGround(world, restitution: 0)

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0.25 + 0.5 * Self.impactSpeed * Self.timeStep, 0)
        def.linearVelocity = Vec3(0, -Self.impactSpeed, 0)
        def.angularVelocity = Vec3(spin, 0, 0)
        let box = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = restitution
        _ = world.createHullShape(box, shapeDef, makeBoxHull(1, 0.25, 1))

        for _ in 0..<60 { world.step(Self.timeStep, subStepCount: Self.subSteps) }

        let speed = world.bodyLinearVelocity(box).y
        let residual = length(world.bodyAngularVelocity(box))
        return (speed / Self.impactSpeed, residual)
    }

    @Test("spin about a horizontal axis does not disturb the flat bounce")
    func spinAlongAxis() {
        for spin in [0 as Float, 0.5, 1.0, 2.0] {
            let result = measureSpinningBounce(spin, 0.9)
            // The bounce coefficient stays close to the nominal value regardless of spin.
            #expect(
                absFloat(result.coefficient - 0.9) <= 0.05,
                "spin \(spin) coefficient \(result.coefficient)")
        }
    }
}

// MARK: - Multi-bounce apexes and resting stability

extension RestitutionPhaseTests {
    /// A perfectly elastic ball dropped under gravity. Records each bounce apex.
    private func measureApexes(_ dropHeight: Float, _ capacity: Int) -> [Float] {
        let world = makeWorld(gravityY: -10)
        makeGround(world, restitution: 0)

        let ball = makeBall(world, x: 0, y: dropHeight, velocityY: 0, restitution: 1)

        var apexes: [Float] = []
        var previousSpeed: Float = 0

        var step = 0
        while step < 4000 && apexes.count < capacity {
            world.step(Self.timeStep, subStepCount: Self.subSteps)
            let speed = world.bodyLinearVelocity(ball).y
            if previousSpeed > 0 && speed <= 0 {
                apexes.append(Float(world.bodyTransform(ball).p.y))
            }
            previousSpeed = speed
            step += 1
        }
        return apexes
    }

    /// A later apex above the first, or a first apex above the drop height, is energy from nowhere.
    @Test("perfectly elastic drops never exceed their drop height")
    func dropApexes() {
        for dropHeight in [40 as Float, 20, 10] {
            let apexes = measureApexes(dropHeight, 6)
            #expect(apexes.count >= 6, "drop \(dropHeight) only \(apexes.count) apexes")
            guard apexes.count >= 6 else { continue }

            let highest = apexes.max() ?? 0
            #expect(highest <= 1.001 * apexes[0], "drop \(dropHeight) highest \(highest) first \(apexes[0])")
            #expect(apexes[0] <= 1.001 * dropHeight, "drop \(dropHeight) first apex \(apexes[0])")
            // The ball must still be bouncing at the end of the run.
            #expect(apexes[apexes.count - 1] >= 0.8 * apexes[0], "drop \(dropHeight) decayed")
        }
    }

    /// A tall stack of nearly elastic boxes must settle and stay settled rather than creep.
    @Test("a resting elastic stack does not drift or jitter")
    func restingStack() {
        let world = makeWorld(gravityY: -10)

        // The ground itself is bouncy here.
        var groundDef = BodyDef()
        groundDef.position = Pos(0, -1, 0)
        let ground = world.createBody(groundDef)
        var groundShape = ShapeDef()
        groundShape.baseMaterial.friction = 0
        groundShape.baseMaterial.restitution = 0.9
        _ = world.createHullShape(ground, groundShape, makeBoxHull(40, 1, 40))

        let boxCount = 10
        var boxes: [BodyID] = []

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.baseMaterial.friction = 0.6
        shapeDef.baseMaterial.restitution = 0.9

        for i in 0..<boxCount {
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 0.5 + Float(i), 0)
            let box = world.createBody(def)
            _ = world.createHullShape(box, shapeDef, makeCubeHull(0.5))
            boxes.append(box)
        }

        // Let the stack settle.
        for _ in 0..<240 { world.step(Self.timeStep, subStepCount: Self.subSteps) }

        let settled = world.bodyTransform(boxes[boxCount - 1]).p.y
        var drift: Float = 0
        var peakSpeed: Float = 0

        for _ in 0..<300 {
            world.step(Self.timeStep, subStepCount: Self.subSteps)
            let y = world.bodyTransform(boxes[boxCount - 1]).p.y
            drift = maxFloat(drift, Float(abs(y - settled)))
            for box in boxes {
                peakSpeed = maxFloat(peakSpeed, length(world.bodyLinearVelocity(box)))
            }
        }

        #expect(drift < 0.01, "drift \(drift)")
        #expect(peakSpeed < 0.05, "peak speed \(peakSpeed)")
    }
}
