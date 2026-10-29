import Testing
@testable import Natsel

@Suite("Compound query repeatability")
struct CompoundCastTraversalTests {
    @Test("Casts, overlaps, and ordered queries are repeatable", arguments: 0..<6)
    func queries(scenario: Int) throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let field = try #require(makeTestCompound(scenario))
            var rays = CollisionDigest(), casts = CollisionDigest(), overlaps = CollisionDigest()
            var queries = CollisionDigest(), movers = CollisionDigest()
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
            samples.append(UInt64(movers.value))
            samples.append(UInt64(rays.value))
            samples.append(UInt64(casts.value))
            samples.append(UInt64(overlaps.value))
            samples.append(UInt64(queries.value))
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}

