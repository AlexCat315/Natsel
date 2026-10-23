import Testing
@testable import Natsel

@Suite("Compound query C parity")
struct CompoundCastTraversalTests {
    @Test("Casts, overlaps, and ordered queries match C", arguments: 0..<6)
    func queries(scenario: Int) throws {
        let field = try #require(referenceCompound(scenario))
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
                casts.add(shapeCastCompound(field, input))
                let ray = RayCastInput(origin: buffer[0], translation: input.translation, maxFraction: input.maxFraction)
                rays.add(rayCastCompound(field, ray))
            }
            points[0].y = Float(i % 7) * 0.3 - 1
            points[1].y = points[0].y + 0.5
            points.withUnsafeBufferPointer { buffer in
                let proxy = ShapeProxy(points: buffer.baseAddress, count: 2, radius: Float(i % 3) * 0.15)
                overlaps.add(overlapCompound(field, transform, proxy) ? UInt32(1) : UInt32(0))
            }
            let center = Vec3(x, Float(i % 3) * 0.3 - 0.3, z)
            let extent = Vec3(0.45, 0.3, 0.375)
            var count = 0
            let limit = i % 4 == 0 ? 1 : 100000
            queryCompound(field, bounds: AABB(center - extent, center + extent)) { index in
                queries.add(UInt32(index))
                count += 1
                return count < limit
            }
            queries.add(UInt32(count))
            let planes = collideMoverAndCompound(field, Capsule(center1: points[0], center2: points[1], radius: 0.4), capacity: 1 + i % 4)
            movers.add(UInt32(planes.count))
            for plane in planes { movers.add(plane) }
        }
        let expectedRay: [UInt64] = [0x0dd868907dc05221, 0xecd8ca7fa595accb, 0xa4545adb6bf79be9, 0x6cf2db98b53198f8, 0x82c30a4b32618fe2, 0xf82656f353945cc3]
        let expectedCast: [UInt64] = [0x01b83aea054d74c2, 0x8439de7c31693f15, 0xb562b52cbdabdc2e, 0x8a413f0432c8426f, 0xb740be615db70d45, 0x2d922ed3842ac05c]
        let expectedOverlap: [UInt64] = [0xd5697502999b6465, 0xb3be82016cd2ed65, 0x9b993c340dba7eb5, 0x1962758f9c1d82c4, 0x1b9d088f69a14c25, 0x576afbc0193dcc94]
        let expectedQuery: [UInt64] = [0x6f21da622a37e315, 0x8788d5a32a41beb5, 0xe58467415d8c75e7, 0x85336ac975fa1557, 0x4e240cb8f8200f59, 0x29fe13e982c80ed8]
        let expectedMover: [UInt64] = [0x34fdefe7aeaa9342, 0xdfc505c82b2322a9, 0x298c2ae8f59cb47c, 0xf480941442ab4dff, 0xf2951d6b46e6e10d, 0xf3617c1ea0d61cc6]
        #expect(movers.value == expectedMover[scenario])
        #expect(rays.value == expectedRay[scenario])
        #expect(casts.value == expectedCast[scenario])
        #expect(overlaps.value == expectedOverlap[scenario])
        #expect(queries.value == expectedQuery[scenario])
    }
}

