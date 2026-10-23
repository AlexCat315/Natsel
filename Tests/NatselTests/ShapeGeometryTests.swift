import Testing
@testable import Natsel

@Suite("Shape geometry")
struct ShapeGeometryTests {
    private var transform: Transform {
        Transform(Vec3(1, 2, 3), makeQuatFromAxisAngle(Vec3(0, 0, 1), 0.5))
    }

    @Test("hull mass matches the C reference")
    func hullMass() {
        let hull = makeBoxHull(1, 2, 3)
        let mass = computeHullMass(hull, 2)

        #expect(mass.mass.bitPattern == 0x42c0_0000)
        #expect(mass.center == Vec3(0, 0, 0))
        #expect(mass.inertia.cx.x.bitPattern == 0x43d0_0000)
        #expect(mass.inertia.cy.y.bitPattern == 0x43a0_0000)
        #expect(mass.inertia.cz.z.bitPattern == 0x4320_0000)
    }

    @Test("hull AABB and extent match the C reference")
    func hullBounds() {
        let hull = makeBoxHull(1, 2, 3)
        let aabb = computeHullAABB(hull, transform)

        #expect(aabb.lowerBound.x.bitPattern == 0xbf57_2d90)
        #expect(aabb.lowerBound.y.bitPattern == 0xbe6f_f0c0)
        #expect(aabb.lowerBound.z.bitPattern == 0)
        #expect(aabb.upperBound.x.bitPattern == 0x4035_cb64)
        #expect(aabb.upperBound.y.bitPattern == 0x4087_7f86)
        #expect(aabb.upperBound.z.bitPattern == 0x40c0_0000)

        let extent = computeHullExtent(hull, Vec3(0.2, 0.1, -0.3))
        #expect(extent.minExtent.bitPattern == 0x3f80_0000)
        #expect(extent.maxExtent.x.bitPattern == 0x3f99_999a)
        #expect(extent.maxExtent.y.bitPattern == 0x4006_6666)
        #expect(extent.maxExtent.z.bitPattern == 0x4053_3333)

        let projected = computeHullProjectedArea(hull, normalize(Vec3(1, 1, 1)))
        #expect(projected.bitPattern == 0x41cb_3a2f)
    }

    @Test("hull ray cast matches the C reference")
    func hullRayCast() {
        let hull = makeBoxHull(1, 2, 3)
        var input = RayCastInput()
        input.origin = Vec3(-3, 0.5, 0.5)
        input.translation = Vec3(6, 0, 0)
        input.maxFraction = 1

        let output = rayCastHull(hull, input)
        #expect(output.hit)
        #expect(output.fraction.bitPattern == 0x3eaa_aaab)
        #expect(output.point.x.bitPattern == 0xbf80_0000)
        #expect(output.point.y.bitPattern == 0x3f00_0000)
        #expect(output.normal.x.bitPattern == 0xbf80_0000)
        #expect(output.normal.y.bitPattern == 0x8000_0000)
    }

    @Test("sphere shape mass, AABB and extent match the C reference")
    func sphereShape() {
        var shape = Shape()
        shape.type = .sphereShape
        shape.sphere = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)
        shape.density = 2

        let mass = computeShapeMass(shape)
        #expect(mass.mass.bitPattern == 0x3f86_0a92)
        #expect(mass.inertia.cx.x.bitPattern == 0x3dd6_7750)

        let aabb = computeShapeAABB(shape, transform)
        #expect(aabb.lowerBound.x.bitPattern == 0x3efb_7854)
        #expect(aabb.lowerBound.y.bitPattern == 0x3fdc_9968)
        #expect(aabb.lowerBound.z.bitPattern == 0x400c_cccd)
        #expect(aabb.upperBound.x.bitPattern == 0x3fbe_de15)
        #expect(aabb.upperBound.y.bitPattern == 0x402e_4cb4)
        #expect(aabb.upperBound.z.bitPattern == 0x404c_cccd)

        let extent = computeShapeExtent(shape, Vec3(0, 0.1, 0))
        #expect(extent.minExtent.bitPattern == 0x3f00_0000)
        #expect(extent.maxExtent.x.bitPattern == 0x3f19_999a)
        #expect(extent.maxExtent.y.bitPattern == 0x3f19_999a)
        #expect(extent.maxExtent.z.bitPattern == 0x3f4c_cccd)
    }

    @Test("capsule shape mass, AABB and extent match the C reference")
    func capsuleShape() {
        var shape = Shape()
        shape.type = .capsuleShape
        shape.capsule = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
        shape.density = 2

        let mass = computeShapeMass(shape)
        #expect(mass.mass.bitPattern == 0x4086_0a92)
        #expect(mass.inertia.cx.x.bitPattern == 0x4032_7146)
        #expect(mass.inertia.cy.y.bitPattern == 0x3efe_adaf)

        let aabb = computeShapeAABB(shape, transform)
        #expect(aabb.lowerBound.x.bitPattern == 0x3c91_60c0)
        #expect(aabb.lowerBound.y.bitPattern == 0x3f1f_bc64)
        #expect(aabb.lowerBound.z.bitPattern == 0x4020_0000)
        #expect(aabb.upperBound.x.bitPattern == 0x3ffd_ba7d)
        #expect(aabb.upperBound.y.bitPattern == 0x4058_10e7)
        #expect(aabb.upperBound.z.bitPattern == 0x4060_0000)

        let extent = computeShapeExtent(shape, Vec3(0, 0, 0))
        #expect(extent.minExtent.bitPattern == 0x3f00_0000)
        #expect(extent.maxExtent.x.bitPattern == 0x3f00_0000)
        #expect(extent.maxExtent.y.bitPattern == 0x3fc0_0000)
        #expect(extent.maxExtent.z.bitPattern == 0x3f00_0000)
    }

    @Test("proxy AABB and shape proxy match the C reference")
    func proxyAABB() {
        let points: [Vec3] = [Vec3(0, -1, 0), Vec3(0, 1, 0)]
        let proxy = points.withUnsafeBufferPointer {
            ShapeProxy(points: $0.baseAddress, count: 2, radius: 0.5)
        }
        let aabb = computeProxyAABB(proxy)
        #expect(aabb.lowerBound.x.bitPattern == 0xbf00_0000)
        #expect(aabb.lowerBound.y.bitPattern == 0xbfc0_0000)
        #expect(aabb.upperBound.y.bitPattern == 0x3fc0_0000)

        var shape = Shape()
        shape.type = .capsuleShape
        shape.capsule = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
        let buffer = makeShapeProxy(shape)
        let capsuleAABB = buffer.withProxy { computeProxyAABB($0) }
        #expect(capsuleAABB.lowerBound.y.bitPattern == 0xbfc0_0000)
        #expect(capsuleAABB.upperBound.y.bitPattern == 0x3fc0_0000)

        var hullShape = Shape()
        hullShape.type = .hullShape
        hullShape.hull = makeBoxHull(1, 2, 3)
        let hullBuffer = makeShapeProxy(hullShape)
        #expect(hullBuffer.count == 8)
        let hullAABB = hullBuffer.withProxy { computeProxyAABB($0) }
        #expect(hullAABB.lowerBound.x.bitPattern == 0xbf80_0000)
        #expect(hullAABB.upperBound.z.bitPattern == 0x4040_0000)
    }
}
