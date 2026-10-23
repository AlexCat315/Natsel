import Testing
@testable import Natsel

// Physics-quality checks for the restitution solver, ported from test/test_restitution.c.
// These assert the measured coefficient of restitution rather than a bitwise hash.
@Suite("Restitution quality")
struct RestitutionQualityTests {
    private static let timeStep: Float = 1.0 / 60.0
    private static let subSteps: Int32 = 4
    private static let impactSpeed: Float = 10
    private static let restitutionIterations: Int32 = 2

    private func makeWorld(_ gravityY: Float) -> World {
        let world = World()
        world.gravity = Vec3(0, gravityY, 0)
        world.enableSleep = false
        world.restitutionIterations = Self.restitutionIterations
        return world
    }

    /// Ground with its top surface at y = 0
    private func makeGround(_ world: World, _ restitution: Float) {
        var bodyDef = BodyDef()
        bodyDef.position = Pos(0, -1, 0)
        let ground = world.createBody(bodyDef)

        var shapeDef = ShapeDef()
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = restitution
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(40, 1, 40))
    }

    private func makeBall(
        _ world: World, _ x: Float, _ y: Float, _ velocityY: Float, _ restitution: Float,
        density: Float = 1
    ) -> BodyID {
        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(x, y, 0)
        bodyDef.linearVelocity = Vec3(0, velocityY, 0)
        let ball = world.createBody(bodyDef)

        var shapeDef = ShapeDef()
        shapeDef.density = density
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = restitution
        _ = world.createSphereShape(ball, shapeDef, Sphere(center: Vec3(), radius: 0.5))
        return ball
    }

    /// Two free spheres closing head on with no gravity. The coefficient is the ratio of relative
    /// normal speeds at the contact, a mass independent definition.
    private func measureHeadOn(
        _ restitution: Float, _ densityB: Float
    ) -> (coefficient: Float, momentumError: Float) {
        let world = makeWorld(0)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.baseMaterial.friction = 0
        shapeDef.baseMaterial.restitution = restitution

        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(-1, 0, 0)
        bodyDef.linearVelocity = Vec3(0.5 * Self.impactSpeed, 0, 0)
        let bodyA = world.createBody(bodyDef)
        _ = world.createSphereShape(bodyA, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        bodyDef.position = Pos(1, 0, 0)
        bodyDef.linearVelocity = Vec3(-0.5 * Self.impactSpeed, 0, 0)
        let bodyB = world.createBody(bodyDef)
        shapeDef.density = densityB
        _ = world.createSphereShape(bodyB, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        let massA = world.bodyMassData(bodyA).mass
        let massB = world.bodyMassData(bodyB).mass
        let momentum0 = massA * world.bodyLinearVelocity(bodyA).x
            + massB * world.bodyLinearVelocity(bodyB).x

        for _ in 0..<120 {
            world.step(Self.timeStep, subStepCount: Self.subSteps)
        }

        let vA = world.bodyLinearVelocity(bodyA).x
        let vB = world.bodyLinearVelocity(bodyB).x
        let momentum1 = massA * vA + massB * vB

        return ((vB - vA) / Self.impactSpeed, (momentum1 - momentum0) / (massA + massB))
    }

    /// Ball driven into static ground with no gravity, so the measurement carries no gravity bias.
    private func measureGroundBounce(_ restitution: Float, _ gap: Float) -> Float {
        let world = makeWorld(0)
        makeGround(world, 0)

        let ball = makeBall(world, 0, 0.5 + gap, -Self.impactSpeed, restitution)
        for _ in 0..<60 {
            world.step(Self.timeStep, subStepCount: Self.subSteps)
        }

        return world.bodyLinearVelocity(ball).y / Self.impactSpeed
    }

    /// Perfectly elastic ball dropped under gravity. Returns the apex heights.
    private func measureDrop(_ dropHeight: Float) -> [Float] {
        let world = makeWorld(-10)
        makeGround(world, 0)

        let ball = makeBall(world, 0, dropHeight, 0, 1.0)
        var apexes: [Float] = []
        var previousSpeed: Float = 0

        for _ in 0..<4000 where apexes.count < 4 {
            world.step(Self.timeStep, subStepCount: Self.subSteps)

            let speed = world.bodyLinearVelocity(ball).y
            if previousSpeed > 0 && speed <= 0 {
                apexes.append(world.bodyTransform(ball).p.y)
            }
            previousSpeed = speed
        }

        return apexes
    }

    @Test("head-on bounce matches the requested coefficient for any mass ratio")
    func headOn() {
        let densities: [Float] = [1, 10, 100]
        let restitutions: [Float] = [0, 0.25, 0.5, 0.75, 1.0]
        let tolerance: Float = 0.02

        for density in densities {
            for restitution in restitutions {
                let (coefficient, momentumError) = measureHeadOn(restitution, density)
                #expect(absFloat(coefficient - restitution) <= tolerance)
                #expect(absFloat(momentumError) <= 1e-4)
            }
        }
    }

    @Test("ground bounce is independent of the impact gap")
    func groundBounce() {
        for restitution in [Float(0.5), 0.9] {
            for gap in [Float(0), 0.25, 0.5, 0.75] {
                let measured = measureGroundBounce(restitution, gap)
                #expect(absFloat(measured - restitution) <= 0.02)
            }
        }
    }

    @Test("elastic ball returns to nearly its drop height")
    func elasticDrop() {
        let apexes = measureDrop(4.0)
        #expect(apexes.count >= 2)
        #expect(apexes[0] > 3.9 && apexes[0] < 4.05)
        // Successive apexes stay near the original height with no energy pumping.
        for apex in apexes {
            #expect(apex > 3.9 && apex < 4.05)
        }
    }
}
