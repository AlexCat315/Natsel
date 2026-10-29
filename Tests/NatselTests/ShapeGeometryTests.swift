import Testing
@testable import Natsel

@Suite("Shape geometry")
struct ShapeGeometryTests {
    private var transform: Transform {
        Transform(Vec3(1, 2, 3), makeQuatFromAxisAngle(Vec3(0, 0, 1), 0.5))
    }

    @Test("hull mass is repeatable")
    func hullMass() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let hull = makeBoxHull(1, 2, 3)
            let mass = computeHullMass(hull, 2)

            samples.append(UInt64(mass.mass.bitPattern))
            #expect(mass.center == Vec3(0, 0, 0))
            samples.append(UInt64(mass.inertia.cx.x.bitPattern))
            samples.append(UInt64(mass.inertia.cy.y.bitPattern))
            samples.append(UInt64(mass.inertia.cz.z.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("hull AABB and extent are repeatable")
    func hullBounds() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let hull = makeBoxHull(1, 2, 3)
            let aabb = computeHullAABB(hull, transform)

            samples.append(UInt64(aabb.lowerBound.x.bitPattern))
            samples.append(UInt64(aabb.lowerBound.y.bitPattern))
            #expect(aabb.lowerBound.z.bitPattern == 0)
            samples.append(UInt64(aabb.upperBound.x.bitPattern))
            samples.append(UInt64(aabb.upperBound.y.bitPattern))
            samples.append(UInt64(aabb.upperBound.z.bitPattern))

            let extent = computeHullExtent(hull, Vec3(0.2, 0.1, -0.3))
            samples.append(UInt64(extent.minExtent.bitPattern))
            samples.append(UInt64(extent.maxExtent.x.bitPattern))
            samples.append(UInt64(extent.maxExtent.y.bitPattern))
            samples.append(UInt64(extent.maxExtent.z.bitPattern))

            let projected = computeHullProjectedArea(hull, normalize(Vec3(1, 1, 1)))
            samples.append(UInt64(projected.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("hull ray cast is repeatable")
    func hullRayCast() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let hull = makeBoxHull(1, 2, 3)
            var input = RayCastInput()
            input.origin = Vec3(-3, 0.5, 0.5)
            input.translation = Vec3(6, 0, 0)
            input.maxFraction = 1

            let output = rayCastHull(hull, input)
            #expect(output.hit)
            samples.append(UInt64(output.fraction.bitPattern))
            samples.append(UInt64(output.point.x.bitPattern))
            samples.append(UInt64(output.point.y.bitPattern))
            samples.append(UInt64(output.normal.x.bitPattern))
            samples.append(UInt64(output.normal.y.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("sphere shape mass, AABB and extent are repeatable")
    func sphereShape() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            var shape = Shape()
            shape.type = .sphereShape
            shape.sphere = Sphere(center: Vec3(0.1, 0.2, -0.3), radius: 0.5)
            shape.density = 2

            let mass = computeShapeMass(shape)
            samples.append(UInt64(mass.mass.bitPattern))
            samples.append(UInt64(mass.inertia.cx.x.bitPattern))

            let aabb = computeShapeAABB(shape, transform)
            samples.append(UInt64(aabb.lowerBound.x.bitPattern))
            samples.append(UInt64(aabb.lowerBound.y.bitPattern))
            samples.append(UInt64(aabb.lowerBound.z.bitPattern))
            samples.append(UInt64(aabb.upperBound.x.bitPattern))
            samples.append(UInt64(aabb.upperBound.y.bitPattern))
            samples.append(UInt64(aabb.upperBound.z.bitPattern))

            let extent = computeShapeExtent(shape, Vec3(0, 0.1, 0))
            samples.append(UInt64(extent.minExtent.bitPattern))
            samples.append(UInt64(extent.maxExtent.x.bitPattern))
            samples.append(UInt64(extent.maxExtent.y.bitPattern))
            samples.append(UInt64(extent.maxExtent.z.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("capsule shape mass, AABB and extent are repeatable")
    func capsuleShape() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            var shape = Shape()
            shape.type = .capsuleShape
            shape.capsule = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
            shape.density = 2

            let mass = computeShapeMass(shape)
            samples.append(UInt64(mass.mass.bitPattern))
            samples.append(UInt64(mass.inertia.cx.x.bitPattern))
            samples.append(UInt64(mass.inertia.cy.y.bitPattern))

            let aabb = computeShapeAABB(shape, transform)
            samples.append(UInt64(aabb.lowerBound.x.bitPattern))
            samples.append(UInt64(aabb.lowerBound.y.bitPattern))
            samples.append(UInt64(aabb.lowerBound.z.bitPattern))
            samples.append(UInt64(aabb.upperBound.x.bitPattern))
            samples.append(UInt64(aabb.upperBound.y.bitPattern))
            samples.append(UInt64(aabb.upperBound.z.bitPattern))

            let extent = computeShapeExtent(shape, Vec3(0, 0, 0))
            samples.append(UInt64(extent.minExtent.bitPattern))
            samples.append(UInt64(extent.maxExtent.x.bitPattern))
            samples.append(UInt64(extent.maxExtent.y.bitPattern))
            samples.append(UInt64(extent.maxExtent.z.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("proxy AABB and shape proxy are repeatable")
    func proxyAABB() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let points: [Vec3] = [Vec3(0, -1, 0), Vec3(0, 1, 0)]
            let proxy = points.withUnsafeBufferPointer {
                ShapeProxy(points: $0.baseAddress, count: 2, radius: 0.5)
            }
            let aabb = computeProxyAABB(proxy)
            samples.append(UInt64(aabb.lowerBound.x.bitPattern))
            samples.append(UInt64(aabb.lowerBound.y.bitPattern))
            samples.append(UInt64(aabb.upperBound.y.bitPattern))

            var shape = Shape()
            shape.type = .capsuleShape
            shape.capsule = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
            let buffer = makeShapeProxy(shape)
            let capsuleAABB = buffer.withProxy { computeProxyAABB($0) }
            samples.append(UInt64(capsuleAABB.lowerBound.y.bitPattern))
            samples.append(UInt64(capsuleAABB.upperBound.y.bitPattern))

            var hullShape = Shape()
            hullShape.type = .hullShape
            hullShape.hull = makeBoxHull(1, 2, 3)
            let hullBuffer = makeShapeProxy(hullShape)
            #expect(hullBuffer.count == 8)
            let hullAABB = hullBuffer.withProxy { computeProxyAABB($0) }
            samples.append(UInt64(hullAABB.lowerBound.x.bitPattern))
            samples.append(UInt64(hullAABB.upperBound.z.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
