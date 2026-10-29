import Testing
@testable import Natsel

@Suite("Mover TOI")
struct MoverTOITests {
    private func standingMover() -> Capsule {
        Capsule(center1: Vec3(0, 0, 0), center2: Vec3(0, 1, 0), radius: 0.25)
    }

    private func identityAt(_ x: Float, _ y: Float, _ z: Float) -> WorldTransform {
        WorldTransform(Vec3(x, y, z), Quat.identity)
    }

    private func makeWorld() -> (World, BodyID) {
        let world = World()
        var def = BodyDef()
        let body = world.createBody(def)
        return (world, body)
    }

    @Test("a mover sweeping into a box reports the free fraction and the face normal")
    func hitsBox() {
        let (world, body) = makeWorld()
        let box = makeBoxHull(0.5, 0.5, 0.5)
        let shape = world.createHullShape(body, ShapeDef(), box)

        // Face at x = 4.5 and radius 0.25, so 4.25 of the 10 unit sweep is free.
        let transform = identityAt(5, 0, 0)
        let result = world.bodyTimeOfImpactMover(
            body, Pos(), standingMover(), Vec3(10, 0, 0), .defaultFilter, transform, transform
        )

        #expect(absFloat(result.fraction - 0.425) < 1e-2)
        #expect(isNormalized(result.normal))
        #expect(absFloat(result.normal.x + 1) < 1e-3)

        // The hit shape must come back identified.
        #expect(result.shapeId.index1 == shape.index1)
        #expect(result.shapeId.generation == shape.generation)
    }

    @Test("sweeping parallel to the gap reports no hit")
    func separated() {
        let (world, body) = makeWorld()
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))

        // Sweeping along +Y holds the X gap at 4.25 for the whole interval.
        let transform = identityAt(5, 0, 0)
        let result = world.bodyTimeOfImpactMover(
            body, Pos(), standingMover(), Vec3(0, 10, 0), .defaultFilter, transform, transform
        )

        #expect(absFloat(result.fraction - 1) < 1e-6)
        #expect(result.shapeId.index1 == 0)
    }

    @Test("a mover starting inside the box is ignored")
    func overlapped() {
        let (world, body) = makeWorld()
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))

        let transform = identityAt(0, 0, 0)
        let result = world.bodyTimeOfImpactMover(
            body, Pos(), standingMover(), Vec3(10, 0, 0), .defaultFilter, transform, transform
        )

        // Overlap should be ignored.
        #expect(absFloat(result.fraction - 1) < 1e-6)
        #expect(result.shapeId.index1 == 0)
    }

    @Test("two shapes on the sweep line resolve to the nearer one in either list order")
    func closestShape() {
        let world = World()
        var def = BodyDef()
        let nearFirstBody = world.createBody(def)
        let nearLastBody = world.createBody(def)

        let shapeDef = ShapeDef()
        let nearSphere = Sphere(center: Vec3(5, 0, 0), radius: 0.5)
        let farSphere = Sphere(center: Vec3(9, 0, 0), radius: 0.5)

        let nearFirstHit = world.createSphereShape(nearFirstBody, shapeDef, nearSphere)
        _ = world.createSphereShape(nearFirstBody, shapeDef, farSphere)

        _ = world.createSphereShape(nearLastBody, shapeDef, farSphere)
        let nearLastHit = world.createSphereShape(nearLastBody, shapeDef, nearSphere)

        let transform = identityAt(0, 0, 0)
        let translation = Vec3(10, 0, 0)

        let nearFirst = world.bodyTimeOfImpactMover(
            nearFirstBody, Pos(), standingMover(), translation, .defaultFilter, transform, transform)
        let nearLast = world.bodyTimeOfImpactMover(
            nearLastBody, Pos(), standingMover(), translation, .defaultFilter, transform, transform)

        #expect(absFloat(nearFirst.fraction - 0.425) < 1e-2)
        #expect(nearFirst.shapeId.index1 == nearFirstHit.index1)

        #expect(absFloat(nearLast.fraction - nearFirst.fraction) < 1e-4)
        #expect(nearLast.shapeId.index1 == nearLastHit.index1)
    }

    @Test("a shape the sweep clears does not erase an earlier hit")
    func keepsHitAfterMiss() {
        let (world, body) = makeWorld()

        let shapeDef = ShapeDef()
        let offPath = Sphere(center: Vec3(5, 0, 20), radius: 0.5)
        let onPath = Sphere(center: Vec3(5, 0, 0), radius: 0.5)
        _ = world.createSphereShape(body, shapeDef, offPath)
        let onPathShape = world.createSphereShape(body, shapeDef, onPath)

        let transform = identityAt(0, 0, 0)
        let result = world.bodyTimeOfImpactMover(
            body, Pos(), standingMover(), Vec3(10, 0, 0), .defaultFilter, transform, transform)

        #expect(absFloat(result.fraction - 0.425) < 1e-2)
        #expect(result.shapeId.index1 == onPathShape.index1)
    }

    @Test("offsetting the mover and body together keeps the impact pose")
    func moverOffset() {
        let (world, body) = makeWorld()
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))

        let translation = Vec3(10, 0, 0)

        let atOrigin = standingMover()
        let transformA = identityAt(5, 0, 0)
        let resultA = world.bodyTimeOfImpactMover(
            body, Pos(), atOrigin, translation, .defaultFilter, transformA, transformA)

        let offset = Capsule(center1: Vec3(2, 0, 0), center2: Vec3(2, 1, 0), radius: 0.25)
        let transformB = identityAt(7, 0, 0)
        let resultB = world.bodyTimeOfImpactMover(
            body, Pos(), offset, translation, .defaultFilter, transformB, transformB)

        #expect(absFloat(resultB.fraction - resultA.fraction) < 1e-4)
        #expect(absFloat(resultB.point.x - resultA.point.x - 2) < 1e-3)
    }

    @Test("a bar spinning a quarter turn reaches a mover its start pose clears")
    func rotatingBody() {
        let (world, body) = makeWorld()
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(2, 0.25, 0.25))

        // The query origin puts the mover at world (0,0,2.1), just inside the swept end of the bar.
        let origin = Pos(0, 0, Float(2.1))
        let transform1 = identityAt(0, 0, 0)
        let transform2 = WorldTransform(
            Vec3(), makeQuatFromAxisAngle(.axisY, 0.5 * B3.pi))

        let spinning = world.bodyTimeOfImpactMover(
            body, origin, standingMover(), Vec3(), .defaultFilter, transform1, transform2)
        #expect(spinning.fraction > 0 && spinning.fraction < 1)

        // Holding the start pose leaves the bar along X and well clear.
        let still = world.bodyTimeOfImpactMover(
            body, origin, standingMover(), Vec3(), .defaultFilter, transform1, transform1)
        #expect(absFloat(still.fraction - 1) < 1e-6)
        #expect(still.shapeId.index1 == 0)
    }

    @Test("a huge origin does not shift the fraction")
    func farFromOrigin() {
        let (world, body) = makeWorld()
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))

        let origin = Pos(Float(1.0e6), Float(-2.0e6), Float(5.0e5))
        let transform = WorldTransform(offsetPos(origin, Vec3(5, 0, 0)), Quat.identity)

        let result = world.bodyTimeOfImpactMover(
            body, origin, standingMover(), Vec3(10, 0, 0), .defaultFilter, transform, transform)

        #expect(absFloat(result.fraction - 0.425) < 1e-2)
        #expect(absFloat(result.normal.x + 1) < 1e-3)
        #expect(absFloat((result.point.x - origin.x) - 4.4) < 0.5)
    }

    @Test("a zero mask rejects every category")
    func filter() {
        let (world, body) = makeWorld()
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))

        var queryFilter = QueryFilter.defaultFilter
        queryFilter.maskBits = 0

        let transform = identityAt(5, 0, 0)
        let result = world.bodyTimeOfImpactMover(
            body, Pos(), standingMover(), Vec3(10, 0, 0), queryFilter, transform, transform)

        #expect(absFloat(result.fraction - 1) < 1e-6)
    }

    @Test("a buried shape is skipped but the sweep still hits the other one")
    func overlapSkipsShape() {
        let (world, body) = makeWorld()

        let shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        // Face at x = 4.5 as in the direct hit.
        let onPath = world.createHullShape(
            body, shapeDef, makeOffsetBoxHull(0.5, 0.5, 0.5, Vec3(5, 0, 0)))

        let transform = identityAt(0, 0, 0)
        let result = world.bodyTimeOfImpactMover(
            body, Pos(), standingMover(), Vec3(10, 0, 0), .defaultFilter, transform, transform)

        #expect(absFloat(result.fraction - 0.425) < 1e-2)
        #expect(result.shapeId.index1 == onPath.index1)
    }
}

// MARK: - Sweep equivalence

extension MoverTOITests {
    private struct XorShift {
        var state: UInt32 = 0x9E37_79B9

        mutating func next() -> UInt32 {
            state ^= state << 13
            state ^= state >> 17
            state ^= state << 5
            return state
        }

        mutating func float(_ lower: Float, _ upper: Float) -> Float {
            let x = next()
            let unit = Float(x >> 8) / 16_777_216
            return lower + (upper - lower) * unit
        }

        mutating func vector(_ lower: Float, _ upper: Float) -> Vec3 {
            Vec3(float(lower, upper), float(lower, upper), float(lower, upper))
        }

        mutating func direction() -> Vec3 {
            while true {
                let v = vector(-1, 1)
                let len = length(v)
                if 0.1 < len && len < 1 {
                    return (1 / len) * v
                }
            }
        }

        mutating func quat() -> Quat {
            makeQuatFromAxisAngle(direction(), float(0, B3.pi))
        }
    }

    /// The mover sweep follows the solver's continuous rule: any time of impact strictly inside the
    /// sweep counts, whatever the root finder reports. A capsule pivoting around a hull vertex can
    /// exhaust the iteration cap, and that must not read as a clean miss.
    @Test("the body TOI matches the raw sweep across 2000 random poses")
    func matchesSweep() {
        let (world, body) = makeWorld()
        let bar = makeBoxHull(2, 0.25, 0.25)
        let shape = world.createHullShape(body, ShapeDef(), bar)

        var random = XorShift()
        var hitCount = 0
        var failures: [String] = []

        guard let hullPoints = bar.points else {
            Issue.record("hull has no points")
            return
        }

        for i in 0..<2000 {
            let c1 = random.vector(-0.2, 0.2)
            let c2 = mulAdd(c1, random.float(0.05, 1.5), random.direction())
            let radius = random.float(0.05, 0.5)
            let mover = Capsule(center1: c1, center2: c2, radius: radius)

            let o = random.vector(-3.5, 3.5)
            let origin = Pos(o.x, o.y, o.z)
            let translation = random.float(0, 3) * random.direction()

            let q1 = random.quat()
            let transform1 = WorldTransform(Vec3(), q1)

            let spin = makeQuatFromAxisAngle(random.direction(), random.float(0.001, B3.pi))
            let shift = random.vector(-1, 1)
            let transform2 = WorldTransform(shift, mulQuat(spin, q1))

            let result = world.bodyTimeOfImpactMover(
                body, origin, mover, translation, .defaultFilter, transform1, transform2)

            // The same sweep the body query builds. A static body keeps its center at the origin.
            let xf1 = toRelativeTransform(transform1, origin)
            let xf2 = toRelativeTransform(transform2, origin)

            var input = TOIInput()
            input.sweepA.c1 = xf1.p
            input.sweepA.c2 = xf2.p
            input.sweepA.q1 = transform1.q
            input.sweepA.q2 = transform2.q
            input.sweepA.localCenter = Vec3()
            input.sweepB.c1 = Vec3()
            input.sweepB.c2 = translation
            input.sweepB.q1 = Quat.identity
            input.sweepB.q2 = Quat.identity
            input.sweepB.localCenter = Vec3()
            input.maxFraction = 1

            var capsulePoints = [c1, c2]
            let raw: TOIOutput = capsulePoints.withUnsafeBufferPointer { pts in
                input.proxyA = ShapeProxy(points: hullPoints, count: bar.vertexCount, radius: 0)
                input.proxyB = ShapeProxy(points: pts.baseAddress, count: 2, radius: radius)
                return timeOfImpact(input)
            }

            if raw.fraction > 0 && raw.fraction < 1 {
                if absFloat(result.fraction - raw.fraction) > 1e-6 {
                    failures.append("sample \(i): fraction \(result.fraction) vs \(raw.fraction)")
                } else if result.shapeId.index1 != shape.index1 {
                    failures.append("sample \(i): shape id")
                } else if !isNormalized(result.normal) {
                    failures.append("sample \(i): normal not unit")
                } else if distance(subPos(result.point, origin), raw.point) > 1e-4 {
                    failures.append("sample \(i): point mismatch")
                }
                hitCount += 1
            } else {
                if absFloat(result.fraction - 1) > 1e-6 {
                    failures.append("sample \(i): expected miss, got \(result.fraction)")
                } else if result.shapeId.index1 != 0 {
                    failures.append("sample \(i): miss carried a shape id")
                }
            }

            if failures.count >= 3 { break }
        }

        #expect(failures.isEmpty, "\(failures.prefix(3))")
        #expect(hitCount > 0)
    }
}
