import Testing
@testable import Natsel

private func fnv(_ h: UInt64, _ bytes: UnsafeRawPointer, _ n: Int) -> UInt64 {
    var h = h
    let p = bytes.assumingMemoryBound(to: UInt8.self)
    for i in 0..<n {
        h ^= UInt64(p[i])
        h = h &* 1_099_511_628_211
    }
    return h
}

@Suite("World queries")
struct WorldQueryTests {
    @Test("overlap, ray cast, and shape cast are repeatable")
    func queries() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            world.enableSleep = false

            var def = BodyDef()
            def.type = .staticBody
            let wall = world.createBody(def)
            var shapeDef = ShapeDef()
            _ = world.createHullShape(wall, shapeDef, makeBoxHull(1, 1, 1))

            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(3, 0, 0)
            let sphere = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createSphereShape(sphere, shapeDef, Sphere(center: Vec3(), radius: 0.5))

            var h: UInt64 = 14_695_981_039_346_656_037

            // overlap AABB around the origin
            var overlapCount = 0
            var ts = world.overlapAABB(AABB(Vec3(-0.2, -0.2, -0.2), Vec3(0.2, 0.2, 0.2))) { _ in
                overlapCount += 1
                return true
            }
            #expect(overlapCount == 1)
            #expect(ts.nodeVisits == 2)
            #expect(ts.leafVisits == 1)
            h = fnv(h, &overlapCount, 4)
            h = fnv(h, &ts.nodeVisits, 4)
            h = fnv(h, &ts.leafVisits, 4)

            // overlap shape at the origin with a sphere proxy of radius 0.1
            var center = Vec3()
            let overlapResult: (count: Int, stats: TreeStats) = withUnsafePointer(to: &center) { pointer in
                let proxy = ShapeProxy(points: pointer, count: 1, radius: 0.1)
                var count = 0
                let stats = world.overlapShape(Pos(), proxy) { _ in
                    count += 1
                    return true
                }
                return (count, stats)
            }
            #expect(overlapResult.count == 1)
            #expect(overlapResult.stats.nodeVisits == 2)
            #expect(overlapResult.stats.leafVisits == 1)
            var shapeCount = Int32(overlapResult.count)
            h = fnv(h, &shapeCount, 4)
            var nodeVisits = overlapResult.stats.nodeVisits
            var leafVisits = overlapResult.stats.leafVisits
            h = fnv(h, &nodeVisits, 4)
            h = fnv(h, &leafVisits, 4)

            // ray closest
            let ray = world.castRayClosest(Pos(5, 0, 0), Vec3(-10, 0, 0))
            #expect(ray.hit)
            samples.append(UInt64(ray.fraction.bitPattern))
            #expect(ray.point == Pos(Float(3.5), 0, 0))
            #expect(ray.normal == Vec3(1, 0, 0))
            var hit = Int32(ray.hit ? 1 : 0)
            var fraction = ray.fraction
            var point = ray.point
            var normal = ray.normal
            h = fnv(h, &hit, 4)
            h = fnv(h, &fraction, 4)
            h = fnv(h, &point, MemoryLayout<Pos>.size)
            h = fnv(h, &normal, 12)

            // shape cast
            var castFraction: Float = 0
            var castPoint = Pos()
            var castNormal = Vec3()
            var castHit = false
            let castStats: TreeStats = withUnsafePointer(to: &center) { pointer in
                let proxy = ShapeProxy(points: pointer, count: 1, radius: 0.1)
                var input = ShapeCastInput()
                input.proxy = proxy
                input.translation = Vec3(-10, 0, 0)
                input.maxFraction = 1

                return world.shapeCast(Pos(5, 0, 0), input) { _, p, n, f, _, _, _ in
                    if 0 <= f && f <= 1 {
                        if castHit == false || f < castFraction {
                            castHit = true
                            castFraction = f
                            castPoint = p
                            castNormal = n
                        }
                    }
                    return f
                }
            }
            #expect(castHit)
            samples.append(UInt64(castFraction.bitPattern))
            #expect(castPoint == Pos(Float(3.5), 0, 0))
            #expect(castNormal == Vec3(1, 0, 0))
            var castHitInt = Int32(castHit ? 1 : 0)
            var castPointCopy = castPoint
            var castNormalCopy = castNormal
            h = fnv(h, &castHitInt, 4)
            h = fnv(h, &castFraction, 4)
            h = fnv(h, &castPointCopy, MemoryLayout<Pos>.size)
            h = fnv(h, &castNormalCopy, 12)
            var castNode = castStats.nodeVisits
            var castLeaf = castStats.leafVisits
            h = fnv(h, &castNode, 4)
            h = fnv(h, &castLeaf, 4)

            samples.append(UInt64(h))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
