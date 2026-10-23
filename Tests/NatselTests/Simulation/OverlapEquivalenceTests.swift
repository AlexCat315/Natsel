import Testing
@testable import Natsel

// Origin-relative equivalence of the world overlap query, ported from
// test/test_body_query.c OverlapHullProxyEquivalence.
@Suite("Overlap equivalence")
struct OverlapEquivalenceTests {
    private func makeWorld() -> (World, BodyID) {
        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(10, 0, 0)
        let body = world.createBody(def)
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(1, 1, 1))

        world.step(1.0 / 60.0, subStepCount: 1)
        return (world, body)
    }

    private func countHits(_ world: World, _ origin: Pos, _ points: [Vec3]) -> Int {
        var hits = 0
        points.withUnsafeBufferPointer { buffer in
            let proxy = ShapeProxy(points: buffer.baseAddress, count: Int32(buffer.count), radius: 0)
            _ = world.overlapShape(origin, proxy, .defaultFilter) { _ in
                hits += 1
                return true
            }
        }
        return hits
    }

    private func points(of hull: Hull) -> [Vec3] {
        guard let raw = hull.points else { return [] }
        return (0..<Int(hull.vertexCount)).map { raw[$0] }
    }

    /// A box hull proxy built around a world target with a zero origin must hit the same shapes as
    /// the same box built at the local origin and queried with the target as origin.
    @Test("a baked query box equals the same box shifted to the origin")
    func hullProxyEquivalence() {
        let (world, _) = makeWorld()

        // Overlapping target: a 10 wide query box centered on the body.
        let nearOffset = Vec3(10, 0, 0)
        let bakedNear = makeTransformedBoxHull(
            5, 5, 5, Transform(nearOffset, Quat.identity))
        let bakedNearHits = countHits(world, Pos(), points(of: bakedNear))

        let localNear = makeBoxHull(5, 5, 5)
        let localNearHits = countHits(
            world, offsetPos(Pos(), nearOffset), points(of: localNear))

        #expect(bakedNearHits == 1)
        #expect(localNearHits == bakedNearHits)

        // Clearing target: same box far from the body, both formulations agree on the miss.
        let farOffset = Vec3(100, 0, 0)
        let bakedFar = makeTransformedBoxHull(5, 5, 5, Transform(farOffset, Quat.identity))
        let bakedFarHits = countHits(world, Pos(), points(of: bakedFar))

        let localFar = makeBoxHull(5, 5, 5)
        let localFarHits = countHits(
            world, offsetPos(Pos(), farOffset), points(of: localFar))

        #expect(bakedFarHits == 0)
        #expect(localFarHits == bakedFarHits)
    }

    /// The same equivalence must hold when the baked box is rotated.
    @Test("a rotated baked query box still matches the origin-relative form")
    func hullProxyRotation() {
        let (world, _) = makeWorld()

        let rotation = makeQuatFromAxisAngle(normalize(Vec3(1, 2, 3)), 0.7)
        let offset = Vec3(10, 0, 0)

        let baked = makeTransformedBoxHull(5, 5, 5, Transform(offset, rotation))
        let bakedHits = countHits(world, Pos(), points(of: baked))

        // The origin-relative form needs the rotation in the query frame as well, so rotate the
        // local box by the same quaternion before shifting the origin.
        let rotated = makeTransformedBoxHull(5, 5, 5, Transform(Vec3(), rotation))
        let localHits = countHits(world, offsetPos(Pos(), offset), points(of: rotated))

        #expect(bakedHits == 1)
        #expect(localHits == bakedHits)
    }

    @Test("a compound under a rotated body answers overlap through its children")
    func compoundRotatedBody() {
        var definition = CompoundDef()
        for i in 0..<2 {
            var item = CompoundHullDef()
            item.hull = makeBoxHull(0.5, 0.5, 0.5)
            item.transform = Transform(Vec3(Float(i) * 2 - 1, 0, 0), Quat.identity)
            item.material = SurfaceMaterial.defaultMaterial
            definition.hulls.append(item)
        }
        guard let compound = Compound(definition) else {
            Issue.record("compound creation failed")
            return
        }

        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 0, 0)
        def.rotation = makeQuatFromAxisAngle(.axisY, 0.3)
        let body = world.createBody(def)
        _ = world.createCompoundShape(body, ShapeDef(), compound)

        world.step(1.0 / 60.0, subStepCount: 1)

        let bodyTransform = world.bodyTransform(body)

        // Query the world at each child's world position.
        for i in 0..<2 {
            let local = Vec3(Float(i) * 2 - 1, 0, 0)
            let worldPoint = transformPoint(bodyTransform, local)

            var hits = 0
            var point = worldPoint
            withUnsafePointer(to: &point) { ptr in
                let proxy = ShapeProxy(points: ptr, count: 1, radius: 0.25)
                _ = world.overlapShape(Pos(), proxy, .defaultFilter) { _ in
                    hits += 1
                    return true
                }
            }
            #expect(hits == 1, "child \(i) at \(worldPoint)")

            // The body-scoped query must agree.
            var bodyPoint = worldPoint
            let bodyHit = withUnsafePointer(to: &bodyPoint) { ptr -> Bool in
                let proxy = ShapeProxy(points: ptr, count: 1, radius: 0.25)
                return world.bodyOverlapShape(
                    body, Pos(), proxy, .defaultFilter, bodyTransform)
            }
            #expect(bodyHit)
        }

        // The gap between the two children stays empty.
        let inGap = transformPoint(bodyTransform, Vec3())
        var gapPoint = inGap
        var gapHits = 0
        withUnsafePointer(to: &gapPoint) { ptr in
            let proxy = ShapeProxy(points: ptr, count: 1, radius: 0.25)
            _ = world.overlapShape(Pos(), proxy, .defaultFilter) { _ in
                gapHits += 1
                return true
            }
        }
        #expect(gapHits == 0)
    }
}
