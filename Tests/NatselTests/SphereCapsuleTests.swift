import Testing
@testable import Natsel

@Suite("Sphere and capsule helpers")
struct SphereCapsuleTests {
    @Test("sphere mass is repeatable")
    func sphereMass() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let shape = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)
            let mass = computeSphereMass(shape, 2)
            let expectedMass = 2 * (4.0 / 3.0) * B3.pi * shape.radius * shape.radius * shape.radius
            #expect(absFloat(mass.mass - expectedMass) < 0.000001)
            #expect(mass.center == shape.center)
            #expect(absFloat(mass.inertia.cx.x - 0.4 * mass.mass * shape.radius * shape.radius) < 0.000001)

            samples.append(UInt64(mass.mass.bitPattern))
            samples.append(UInt64(mass.center.x.bitPattern))
            samples.append(UInt64(mass.inertia.cx.x.bitPattern))
            samples.append(UInt64(mass.inertia.cy.y.bitPattern))
            samples.append(UInt64(mass.inertia.cz.z.bitPattern))

            let aabb = computeSphereAABB(shape, Transform.identity)
            samples.append(UInt64(aabb.lowerBound.x.bitPattern))
            samples.append(UInt64(aabb.upperBound.y.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("capsule mass is repeatable")
    func capsuleMass() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let shape = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
            let mass = computeCapsuleMass(shape, 2)
            let radius = shape.radius
            let height = length(shape.center2 - shape.center1)
            let expectedMass = 2 * B3.pi * radius * radius * (height + (4.0 / 3.0) * radius)
            #expect(absFloat(mass.mass - expectedMass) < 0.000001)

            samples.append(UInt64(mass.mass.bitPattern))
            #expect(mass.center == Vec3(0, 0, 0))
            samples.append(UInt64(mass.inertia.cx.x.bitPattern))
            samples.append(UInt64(mass.inertia.cy.y.bitPattern))
            samples.append(UInt64(mass.inertia.cz.z.bitPattern))

            let aabb = computeCapsuleAABB(shape, Transform.identity)
            samples.append(UInt64(aabb.lowerBound.x.bitPattern))
            samples.append(UInt64(aabb.upperBound.y.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("sphere ray cast is repeatable")
    func sphereRayCast() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let shape = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)
            var input = RayCastInput()
            input.origin = Vec3(-3, 0.5, 0)
            input.translation = Vec3(6, 0, 0)
            input.maxFraction = 1

            let output = rayCastSphere(shape, input)
            #expect(output.hit)
            samples.append(UInt64(output.fraction.bitPattern))
            samples.append(UInt64(output.point.x.bitPattern))
            samples.append(UInt64(output.point.y.bitPattern))
            samples.append(UInt64(output.point.z.bitPattern))
            samples.append(UInt64(output.normal.x.bitPattern))
            samples.append(UInt64(output.normal.y.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("hollow sphere only hits from inside within range")
    func hollowSphere() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let shape = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)

            var outside = RayCastInput()
            outside.origin = Vec3(-3, 0.5, 0)
            outside.translation = Vec3(6, 0, 0)
            outside.maxFraction = 1
            #expect(!rayCastHollowSphere(shape, outside).hit)

            var inside = RayCastInput()
            inside.origin = Vec3(0.1, 0.2, -0.3)
            inside.translation = Vec3(2, 0.5, 0.25)
            inside.maxFraction = 1
            let output = rayCastHollowSphere(shape, inside)
            #expect(output.hit)
            samples.append(UInt64(output.fraction.bitPattern))
            samples.append(UInt64(output.point.x.bitPattern))
            samples.append(UInt64(output.normal.x.bitPattern))
            samples.append(UInt64(output.normal.z.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("capsule ray cast is repeatable")
    func capsuleRayCast() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let shape = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
            var input = RayCastInput()
            input.origin = Vec3(2, 0.2, 0)
            input.translation = Vec3(-4, 0, 0)
            input.maxFraction = 1

            let output = rayCastCapsule(shape, input)
            #expect(output.hit)
            samples.append(UInt64(output.fraction.bitPattern))
            samples.append(UInt64(output.point.x.bitPattern))
            samples.append(UInt64(output.point.y.bitPattern))
            samples.append(UInt64(output.normal.x.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("shape casts of sphere and capsule report initial overlap")
    func shapeCasts() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            var input = ShapeCastInput()
            input.translation = Vec3(-5, 0, 0)
            input.maxFraction = 1

            let sphere = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)
            let sphereOut = input.withProxy {
                shapeCastSphere(sphere, $0)
            }
            #expect(sphereOut.hit)
            #expect(sphereOut.fraction.bitPattern == 0)
            samples.append(UInt64(sphereOut.point.x.bitPattern))

            let capsule = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
            let capsuleOut = input.withProxy {
                shapeCastCapsule(capsule, $0)
            }
            #expect(capsuleOut.hit)
            #expect(capsuleOut.fraction.bitPattern == 0)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}

private extension ShapeCastInput {
    func withProxy(_ body: (ShapeCastInput) -> CastOutput) -> CastOutput {
        Self.buffer.withUnsafeBufferPointer { buffer in
            var input = self
            input.proxy = ShapeProxy(points: buffer.baseAddress, count: 8, radius: 0)
            return body(input)
        }
    }

    static let buffer: [Vec3] = [
        Vec3(1, 1, 1), Vec3(-1, 1, 1), Vec3(-1, -1, 1), Vec3(1, -1, 1),
        Vec3(1, 1, -1), Vec3(-1, 1, -1), Vec3(-1, -1, -1), Vec3(1, -1, -1),
    ]
}
