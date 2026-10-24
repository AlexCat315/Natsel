import Testing
@testable import Natsel

private func dynamicBody(_ world: World, _ position: Vec3, rotation: Quat = .identity) -> BodyID {
    var def = BodyDef()
    def.type = .dynamicBody
    def.position = position
    def.rotation = rotation
    return world.createBody(def)
}

@Suite("Buoyancy")
struct BuoyancyTests {
    @Test("sphere cap volume and centroid")
    func sphere() {
        let world = World()
        let body = dynamicBody(world, Vec3(0, 0, 0))
        _ = world.createSphereShape(body, ShapeDef(), Sphere(center: Vec3(), radius: 1))
        let half = world.bodySubmergedVolume(body, surfacePosition: Vec3(), surfaceNormal: Vec3(0, 1, 0))
        #expect(abs(half.totalVolume - 4 / 3 * B3.pi) < 1.0e-4)
        #expect(abs(half.submergedVolume - 2 / 3 * B3.pi) < 1.0e-4)
        #expect(abs(half.centerOfBuoyancy.y + 3.0 / 8.0) < 1.0e-4)

        let none = world.bodySubmergedVolume(body, surfacePosition: Vec3(0, -2, 0), surfaceNormal: Vec3(0, 1, 0))
        #expect(none.submergedVolume == 0)
        let all = world.bodySubmergedVolume(body, surfacePosition: Vec3(0, 2, 0), surfaceNormal: Vec3(0, 1, 0))
        #expect(abs(all.submergedVolume - all.totalVolume) < 1.0e-4)
    }

    @Test("hull clipping is exact for boxes in any orientation")
    func hull() {
        let world = World()
        let body = dynamicBody(world, Vec3(0, 0, 0))
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(1, 1, 1))
        let quarter = world.bodySubmergedVolume(body, surfacePosition: Vec3(0, -0.5, 0), surfaceNormal: Vec3(0, 1, 0))
        #expect(abs(quarter.totalVolume - 8) < 1.0e-4)
        #expect(abs(quarter.submergedVolume - 2) < 1.0e-4)
        #expect(abs(quarter.centerOfBuoyancy.y + 0.75) < 1.0e-4)

        // Rotated 45 degrees, surface through the center: half by symmetry.
        let tilted = dynamicBody(world, Vec3(5, 0, 0), rotation: makeQuatFromAxisAngle(Vec3(0, 0, 1), 0.25 * B3.pi))
        _ = world.createHullShape(tilted, ShapeDef(), makeBoxHull(1, 1, 1))
        let half = world.bodySubmergedVolume(tilted, surfacePosition: Vec3(5, 0, 0), surfaceNormal: Vec3(0, 1, 0))
        #expect(abs(half.submergedVolume - 4) < 1.0e-3)
        #expect(half.centerOfBuoyancy.y < -0.3)
        #expect(abs(half.centerOfBuoyancy.x - 5) < 2.0e-3)

        // A corner dipped in: a tetrahedron of volume d^3 * 2^(3/2) / 6 ... check against brute force.
        let corner = dynamicBody(world, Vec3(10, 0, 0), rotation: makeQuatFromAxisAngle(normalize(Vec3(1, 0, 1)), 0.6))
        _ = world.createHullShape(corner, ShapeDef(), makeBoxHull(1, 1, 1))
        let surface = Vec3(10, -0.9, 0)
        let exact = world.bodySubmergedVolume(corner, surfacePosition: surface, surfaceNormal: Vec3(0, 1, 0))
        var count = 0
        let n = 60
        let q = world.bodyRotation(corner)
        for i in 0..<n {
            for j in 0..<n {
                for k in 0..<n {
                    let local = Vec3(-1 + (Float(i) + 0.5) * 2 / Float(n), -1 + (Float(j) + 0.5) * 2 / Float(n),
                                     -1 + (Float(k) + 0.5) * 2 / Float(n))
                    if rotateVector(q, local).y < -0.9 { count += 1 }
                }
            }
        }
        let sampled = 8 * Float(count) / Float(n * n * n)
        #expect(abs(exact.submergedVolume - sampled) < 0.01 + 0.02 * sampled)
    }

    @Test("capsule quadrature matches the closed forms")
    func capsule() {
        let world = World()
        let horizontal = dynamicBody(world, Vec3(0, 0, 0))
        _ = world.createCapsuleShape(horizontal, ShapeDef(), Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.5))
        let total = B3.pi * 0.25 * 2 + 4 / 3 * B3.pi * 0.125
        let half = world.bodySubmergedVolume(horizontal, surfacePosition: Vec3(), surfaceNormal: Vec3(0, 1, 0))
        #expect(abs(half.totalVolume - total) < 1.0e-4)
        #expect(abs(half.submergedVolume - 0.5 * total) < 2.0e-3 * total)

        let vertical = dynamicBody(world, Vec3(5, 0, 0))
        _ = world.createCapsuleShape(vertical, ShapeDef(), Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5))
        // Surface at y = -1: only the lower hemisphere is submerged.
        let hemisphere = world.bodySubmergedVolume(vertical, surfacePosition: Vec3(5, -1, 0), surfaceNormal: Vec3(0, 1, 0))
        #expect(abs(hemisphere.submergedVolume - 2 / 3 * B3.pi * 0.125) < 3.0e-3 * total)
        #expect(abs(hemisphere.centerOfBuoyancy.y - (-1 - 3.0 / 8.0 * 0.5)) < 0.01)
    }

    @Test("a box with buoyancy 2 floats half submerged", arguments: [Float(2), Float(4)])
    func floating(_ buoyancy: Float) {
        let world = World()
        let box = dynamicBody(world, Vec3(0, 1, 0))
        _ = world.createHullShape(box, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))
        // Quadratic drag damps slowly, so compare the draft averaged over the bobbing.
        var draft: Float = 0
        for i in 0..<900 {
            world.applyBuoyancyImpulse(box, surfacePosition: Vec3(), surfaceNormal: Vec3(0, 1, 0), buoyancy: buoyancy,
                                       linearDrag: 1, angularDrag: 1, timeStep: 1.0 / 60.0)
            world.step(1.0 / 60.0, subStepCount: 4)
            if i >= 600 {
                let volume = world.bodySubmergedVolume(box, surfacePosition: Vec3(), surfaceNormal: Vec3(0, 1, 0))
                draft += volume.submergedVolume / volume.totalVolume / 300
            }
        }
        #expect(abs(draft - 1 / buoyancy) < 0.02)
    }

    @Test("a dense body sinks and a current carries a floating one")
    func sinkAndCurrent() {
        let world = World()
        let heavy = dynamicBody(world, Vec3(0, 0, 0))
        _ = world.createSphereShape(heavy, ShapeDef(), Sphere(center: Vec3(), radius: 0.3))
        let light = dynamicBody(world, Vec3(5, 0, 0))
        _ = world.createSphereShape(light, ShapeDef(), Sphere(center: Vec3(), radius: 0.3))
        let water = AABB(Vec3(-50, -50, -50), Vec3(50, 0.1, 50))
        for _ in 0..<300 {
            _ = world.applyBuoyancy(in: aabbOffset(water, Vec3()), buoyancy: 0.5, timeStep: 1.0 / 60.0,
                                    filter: .defaultFilter)
            world.applyBuoyancyImpulse(light, surfacePosition: Vec3(0, 0.1, 0), surfaceNormal: Vec3(0, 1, 0),
                                       buoyancy: 3, linearDrag: 1, fluidVelocity: Vec3(1, 0, 0), timeStep: 1.0 / 60.0)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        #expect(world.bodyPosition(heavy).y < -5)
        #expect(world.bodyPosition(light).x > 6)
        #expect(world.bodyPosition(light).y > -0.3)
    }

    @Test("a plank is righted by the center of buoyancy")
    func righting() {
        let world = World()
        let plank = dynamicBody(world, Vec3(0, 0, 0), rotation: makeQuatFromAxisAngle(Vec3(0, 0, 1), 1.2))
        _ = world.createHullShape(plank, ShapeDef(), makeBoxHull(1, 0.1, 0.5))
        for _ in 0..<900 {
            world.applyBuoyancyImpulse(plank, surfacePosition: Vec3(), surfaceNormal: Vec3(0, 1, 0), buoyancy: 2,
                                       linearDrag: 1, angularDrag: 1, timeStep: 1.0 / 60.0)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        let up = rotateVector(world.bodyRotation(plank), Vec3(0, 1, 0))
        #expect(abs(up.y) > 0.95)
    }
}
