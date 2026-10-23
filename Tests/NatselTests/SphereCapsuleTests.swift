import Testing
@testable import Natsel

@Suite("Sphere and capsule helpers")
struct SphereCapsuleTests {
    @Test("sphere mass matches the C reference")
    func sphereMass() {
        let shape = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)
        let mass = computeSphereMass(shape, 2)

        #expect(mass.mass.bitPattern == 0x3f86_0a92)
        #expect(mass.center.x.bitPattern == 0x3dcc_cccd)
        #expect(mass.inertia.cx.x.bitPattern == 0x3dd6_7750)
        #expect(mass.inertia.cy.y.bitPattern == 0x3dd6_7750)
        #expect(mass.inertia.cz.z.bitPattern == 0x3dd6_7750)

        let aabb = computeSphereAABB(shape, Transform.identity)
        #expect(aabb.lowerBound.x.bitPattern == 0xbecc_cccd)
        #expect(aabb.upperBound.y.bitPattern == 0x3f33_3333)
    }

    @Test("capsule mass matches the C reference")
    func capsuleMass() {
        let shape = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
        let mass = computeCapsuleMass(shape, 2)

        #expect(mass.mass.bitPattern == 0x4086_0a92)
        #expect(mass.center == Vec3(0, 0, 0))
        #expect(mass.inertia.cx.x.bitPattern == 0x4032_7146)
        #expect(mass.inertia.cy.y.bitPattern == 0x3efe_adaf)
        #expect(mass.inertia.cz.z.bitPattern == 0x4032_7146)

        let aabb = computeCapsuleAABB(shape, Transform.identity)
        #expect(aabb.lowerBound.x.bitPattern == 0xbf00_0000)
        #expect(aabb.upperBound.y.bitPattern == 0x3fc0_0000)
    }

    @Test("sphere ray cast matches the C reference bit for bit")
    func sphereRayCast() {
        let shape = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)
        var input = RayCastInput()
        input.origin = Vec3(-3, 0.5, 0)
        input.translation = Vec3(6, 0, 0)
        input.maxFraction = 1

        let output = rayCastSphere(shape, input)
        #expect(output.hit)
        #expect(output.fraction.bitPattern == 0x3ef1_f4cd)
        #expect(output.point.x.bitPattern == 0xbe28_865c)
        #expect(output.point.y.bitPattern == 0x3f00_0001)
        #expect(output.point.z.bitPattern == 0x3300_0000)
        #expect(output.normal.x.bitPattern == 0xbf07_7661)
        #expect(output.normal.y.bitPattern == 0x3f19_999b)
    }

    @Test("hollow sphere only hits from inside within range")
    func hollowSphere() {
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
        #expect(output.fraction.bitPattern == 0x3f00_0000)
        #expect(output.point.x.bitPattern == 0x3f14_e008)
        #expect(output.normal.x.bitPattern == 0x3f76_8cdc)
        #expect(output.normal.z.bitPattern == 0x3df6_8cdc)
    }

    @Test("capsule ray cast matches the C reference")
    func capsuleRayCast() {
        let shape = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
        var input = RayCastInput()
        input.origin = Vec3(2, 0.2, 0)
        input.translation = Vec3(-4, 0, 0)
        input.maxFraction = 1

        let output = rayCastCapsule(shape, input)
        #expect(output.hit)
        #expect(output.fraction.bitPattern == 0x3ec0_0000)
        #expect(output.point.x.bitPattern == 0x3f00_0000)
        #expect(output.point.y.bitPattern == 0x3e4c_ccd0)
        #expect(output.normal.x.bitPattern == 0x3f80_0000)
    }

    @Test("shape casts of sphere and capsule report initial overlap")
    func shapeCasts() {
        var input = ShapeCastInput()
        input.translation = Vec3(-5, 0, 0)
        input.maxFraction = 1

        let sphere = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)
        let sphereOut = input.withProxy {
            shapeCastSphere(sphere, $0)
        }
        #expect(sphereOut.hit)
        #expect(sphereOut.fraction.bitPattern == 0)
        #expect(sphereOut.point.x.bitPattern == 0x3dcc_cccd)

        let capsule = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
        let capsuleOut = input.withProxy {
            shapeCastCapsule(capsule, $0)
        }
        #expect(capsuleOut.hit)
        #expect(capsuleOut.fraction.bitPattern == 0)
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
