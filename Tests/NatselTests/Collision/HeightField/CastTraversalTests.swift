import Testing
@testable import Natsel

@Suite("Height-field query C parity")
struct HeightFieldCastTraversalTests {
    @Test("DDA casts, overlaps, ordered queries, and mover planes match C", arguments: 0..<6)
    func queries(scenario: Int) throws {
        let field = try #require(referenceHeightField(scenario))
        var rays = CollisionReferenceDigest(), casts = CollisionReferenceDigest(), overlaps = CollisionReferenceDigest()
        var queries = CollisionReferenceDigest(), movers = CollisionReferenceDigest()
        let transform = Transform(Vec3(0.125, -0.25, 0.5), makeQuatFromAxisAngle(.axisY, 0.35))
        for i in 0..<120 {
            let x = Float((i * 7) % 19) * 0.375 - 1
            let z = Float((i * 11) % 17) * 0.25 - 0.75
            let y: Float = i < 60 ? 2 : -2
            var points = [Vec3(x, y, z), Vec3(x + 0.1, y + 0.5, z - 0.15)]
            var input = ShapeCastInput()
            input.translation = Vec3(Float(i % 4 - 2) * 1.25, i < 60 ? -4 : 4, Float(i % 5 - 2) * 0.75)
            input.maxFraction = i % 3 == 0 ? 0.35 : 1
            input.canEncroach = i % 2 == 0
            points.withUnsafeBufferPointer { buffer in
                input.proxy = ShapeProxy(points: buffer.baseAddress, count: 2, radius: Float(i % 3) * 0.15)
                casts.add(shapeCastHeightField(field, input))
                let ray = RayCastInput(origin: buffer[0], translation: input.translation, maxFraction: input.maxFraction)
                rays.add(rayCastHeightField(field, ray))
            }
            points[0].y = Float(i % 7) * 0.3 - 1
            points[1].y = points[0].y + 0.5
            points.withUnsafeBufferPointer { buffer in
                let proxy = ShapeProxy(points: buffer.baseAddress, count: 2, radius: Float(i % 3) * 0.15)
                overlaps.add(overlapHeightField(field, transform, proxy) ? UInt32(1) : UInt32(0))
            }
            let center = Vec3(x, Float(i % 3) * 0.3 - 0.3, z)
            let extent = Vec3(0.45, 0.3, 0.375)
            var count = 0
            let limit = i % 4 == 0 ? 3 : 100000
            queryHeightField(field, bounds: AABB(center - extent, center + extent)) { a, b, c, index in
                queries.add(a); queries.add(b); queries.add(c); queries.add(UInt32(bitPattern: index))
                count += 1
                return count < limit
            }
            queries.add(UInt32(count))
            let capsule = Capsule(center1: points[0], center2: points[1], radius: 0.4)
            let planes = collideMoverAndHeightField(field, capsule, capacity: 1 + i % 4)
            movers.add(UInt32(planes.count))
            for plane in planes { movers.add(plane) }
        }
        let expectedRay: [UInt64] = [0xb5ba6f074b61cfec, 0xb5ba6f074b61cfec, 0xf6d7adeacd09bc79, 0xf6d7adeacd09bc79, 0x8c9fe22d8cfc5440, 0x3609dddf2ec246fa]
        let expectedCast: [UInt64] = [0x51a2b80c6596000b, 0x39e5ba440c21d09a, 0x08664e4c0862e53b, 0x08664e4c0862e53b, 0x93b013f669e22289, 0xae118dec1992e20e]
        let expectedOverlap: [UInt64] = [0xa1718e3a8df8cf65, 0xa1718e3a8df8cf65, 0xb1f7c9a4d805a0e4, 0xb1f7c9a4d805a0e4, 0x72168941cf0b1fc4, 0x72168941cf0b1fc4]
        let expectedQuery: [UInt64] = [0xdfd97851557f2601, 0x972b8265c8e60695, 0x302f91635928614b, 0x429647e5ac4aa67f, 0x7e3bbc588e6e6194, 0xb5d42bd738b905a4]
        let expectedMover: [UInt64] = [0xacd5b7202e40cd5c, 0xacd5b7202e40cd5c, 0xe8523b55763a2ac4, 0x74f4bf68462ca278, 0x89262adec25d4731, 0x55c09de091b1c860]
        #expect(rays.value == expectedRay[scenario])
        #expect(casts.value == expectedCast[scenario])
        #expect(overlaps.value == expectedOverlap[scenario])
        #expect(queries.value == expectedQuery[scenario])
        #expect(movers.value == expectedMover[scenario])
    }
}

