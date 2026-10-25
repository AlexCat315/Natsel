import Testing
@testable import Natsel

private func groundWorld(workers: Int32 = 1) -> World {
    let world = World()
    world.workerCount = workers
    var def = BodyDef()
    def.position = Pos(0, -1, 0)
    _ = world.createHullShape(world.createBody(def), ShapeDef(), makeBoxHull(20, 1, 20))
    return world
}

/// Smallest distance from a vertex to a triangle of the same mesh that self collision covers: not
/// its own or an edge neighbor's triangle, and at least two edge lengths away in the rest pose.
private func minimumSelfDistance(_ world: World, _ id: SoftBodyID) -> Float {
    let record = world.softBodies[Int(id.index1 - 1)]
    var copy = record
    copy.buildNeighbors()
    var best = Float.greatestFiniteMagnitude
    let rest = record.settings.vertices.map { $0.position }
    for (i, x) in record.positions.enumerated() {
        for face in record.settings.faces {
            let ia = Int32(face.vertex.x)
            let ib = Int32(face.vertex.y)
            let ic = Int32(face.vertex.z)
            let v = Int32(i)
            if v == ia || v == ib || v == ic || copy.areNeighbors(v, ia) || copy.areNeighbors(v, ib) || copy.areNeighbors(v, ic) {
                continue
            }
            let rb = closestTriangleBarycentric(rest[Int(ia)], rest[Int(ib)], rest[Int(ic)], rest[i])
            let rq = rb.u * rest[Int(ia)] + rb.v * rest[Int(ib)] + rb.w * rest[Int(ic)]
            if distance(rest[i], rq) < 2 * record.averageEdgeLength { continue }
            let a = record.positions[Int(ia)]
            let b = record.positions[Int(ib)]
            let c = record.positions[Int(ic)]
            let bary = closestTriangleBarycentric(a, b, c, x)
            best = min(best, distance(x, bary.u * a + bary.v * b + bary.w * c))
        }
    }
    return best
}

/// A long strip hanging above the ground: it folds onto itself as it lands.
private func dropStrip(_ world: World, selfCollision: Bool) -> SoftBodyID {
    var def = SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 5, rows: 36, spacing: 0.05),
                          position: Pos(0, Float(1.0), 0), rotation: makeQuatFromAxisAngle(.axisX, 0.5 * Float.pi))
    def.enableSelfCollision = selfCollision
    def.iterationCount = 8
    def.friction = 0.5
    return world.createSoftBody(def)
}

@Suite("Soft body contacts")
struct SoftContactTests {
    @Test("closest point on a triangle with barycentric coordinates")
    func barycentric() {
        let a = Vec3(0, 0, 0)
        let b = Vec3(1, 0, 0)
        let c = Vec3(0, 1, 0)
        let inside = closestTriangleBarycentric(a, b, c, Vec3(0.25, 0.25, 2))
        #expect(inside.interior)
        #expect(abs(inside.u - 0.5) < 1.0e-6 && abs(inside.v - 0.25) < 1.0e-6 && abs(inside.w - 0.25) < 1.0e-6)
        let edge = closestTriangleBarycentric(a, b, c, Vec3(0.5, -1, 0))
        #expect(edge.interior == false && abs(edge.u - 0.5) < 1.0e-6 && abs(edge.v - 0.5) < 1.0e-6)
        let corner = closestTriangleBarycentric(a, b, c, Vec3(2, 2, 0))
        #expect(corner.interior == false)
        let q = corner.u * a + corner.v * b + corner.w * c
        #expect(distance(q, Vec3(0.5, 0.5, 0)) < 1.0e-6)
    }

    @Test("a folding strip keeps its layers apart with self collision")
    func selfCollision() {
        let free = groundWorld()
        let freeStrip = dropStrip(free, selfCollision: false)
        let guarded = groundWorld()
        let guardedStrip = dropStrip(guarded, selfCollision: true)
        let thickness = guarded.softBodyCollisionThickness(guardedStrip)
        #expect(thickness > 0)
        for _ in 0..<150 {
            free.step(1.0 / 60.0, subStepCount: 4)
            guarded.step(1.0 / 60.0, subStepCount: 4)
        }
        let freeDistance = minimumSelfDistance(free, freeStrip)
        let guardedDistance = minimumSelfDistance(guarded, guardedStrip)
        // Without self collision the layers pass through each other.
        #expect(freeDistance < 0.2 * thickness)
        #expect(guardedDistance > 0.6 * thickness, "\(guardedDistance) vs \(thickness)")
    }

    @Test("a cloth lands on another cloth instead of passing through it")
    func clothOnCloth() {
        func run(_ collide: Bool) -> (Float, Float) {
            let world = groundWorld()
            var bottom = SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 12, rows: 12, spacing: 0.1),
                                     position: Pos(0, Float(0.05), 0))
            bottom.collideWithSoftBodies = collide
            let lower = world.createSoftBody(bottom)
            var top = SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 8, rows: 8, spacing: 0.1),
                                  position: Pos(Float(0.05), Float(0.6), Float(0.05)))
            top.collideWithSoftBodies = collide
            let upper = world.createSoftBody(top)
            for _ in 0..<120 { world.step(1.0 / 60.0, subStepCount: 4) }
            let average = { (id: SoftBodyID) -> Float in
                let p = world.softBodyVertexPositions(id)
                return p.reduce(0) { $0 + Float($1.y) } / Float(p.count)
            }
            return (average(upper) - average(lower), world.softBodyCollisionThickness(upper))
        }
        let (gap, thickness) = run(true)
        let (overlap, _) = run(false)
        #expect(gap > 0.7 * thickness, "\(gap) vs \(thickness)")
        #expect(abs(overlap) < 0.2 * thickness)
    }

    @Test("ropes rest on each other through vertex contacts")
    func ropes() {
        let world = groundWorld()
        var lower = SoftBodyDef(settings: SoftBodySettings.makeRope(count: 30, spacing: 0.05, pinned: false),
                                position: Pos(0, Float(0.05), Float(0.7)), rotation: makeQuatFromAxisAngle(.axisX, 0.5 * Float.pi))
        lower.collisionThickness = 0.04
        let a = world.createSoftBody(lower)
        var upper = SoftBodyDef(settings: SoftBodySettings.makeRope(count: 30, spacing: 0.05, pinned: false),
                                position: Pos(Float(-0.7), Float(0.4), 0), rotation: makeQuatFromAxisAngle(.axisZ, 0.5 * Float.pi))
        upper.collisionThickness = 0.04
        let b = world.createSoftBody(upper)
        for _ in 0..<120 { world.step(1.0 / 60.0, subStepCount: 4) }
        // The upper rope crosses over the lower one near the origin.
        let crossing = world.softBodyVertexPositions(b).min { abs($0.x) < abs($1.x) }!
        let below = world.softBodyVertexPositions(a).min { abs($0.z - crossing.z) < abs($1.z - crossing.z) }!
        #expect(crossing.y - below.y > 0.03, "\(crossing.y) over \(below.y)")
    }

    @Test("results do not depend on the worker count")
    func threads() {
        func run(_ workers: Int32) -> [Pos] {
            let world = groundWorld(workers: workers)
            var ids: [SoftBodyID] = []
            for k in 0..<4 {
                var def = SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 10, rows: 10, spacing: 0.1),
                                      position: Pos(Float(k / 2) * 3, 0.3 + 0.4 * Float(k % 2), 0))
                def.enableSelfCollision = k == 0
                ids.append(world.createSoftBody(def))
            }
            for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
            return ids.flatMap { world.softBodyVertexPositions($0) }
        }
        let serial = run(1)
        let parallel = run(6)
        #expect(serial.count == parallel.count)
        #expect(zip(serial, parallel).allSatisfy { $0.x.bitPattern == $1.x.bitPattern && $0.y.bitPattern == $1.y.bitPattern
            && $0.z.bitPattern == $1.z.bitPattern })
    }

    @Test("a falling cloth wakes a sleeping one")
    func waking() {
        let world = groundWorld()
        let resting = world.createSoftBody(SoftBodyDef(
            settings: SoftBodySettings.makeCloth(columns: 8, rows: 8, spacing: 0.1), position: Pos(0, Float(0.02), 0)))
        for _ in 0..<120 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.softBodyIsAwake(resting) == false)
        let falling = world.createSoftBody(SoftBodyDef(
            settings: SoftBodySettings.makeCloth(columns: 6, rows: 6, spacing: 0.1), position: Pos(0, 1, 0)))
        var woke = false
        for _ in 0..<60 {
            world.step(1.0 / 60.0, subStepCount: 4)
            woke = woke || world.softBodyIsAwake(resting)
        }
        #expect(woke)
        let top = world.softBodyVertexPositions(falling).reduce(0) { $0 + $1.y } / 36
        #expect(top > 0.02)
    }
}
