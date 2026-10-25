import Testing
@testable import Natsel

/// A world with a ground far below, so anything that falls through is obvious.
private func lowGroundWorld(workers: Int32 = 1) -> World {
    let world = World()
    world.workerCount = workers
    var def = BodyDef()
    def.position = Pos(0, -3, 0)
    _ = world.createHullShape(world.createBody(def), ShapeDef(), makeBoxHull(20, 1, 20))
    return world
}

/// Height of the highest vertex: whether the body was caught at all.
private func maxY(_ world: World, _ id: SoftBodyID) -> Float {
    world.softBodyVertexPositions(id).map { Float($0.y) }.max()!
}

private func run(_ world: World, _ steps: Int) {
    for _ in 0..<steps { world.step(1.0 / 60.0, subStepCount: 4) }
}

@Suite("Soft body edge collision")
struct SoftEdgeCollisionTests {
    /// A narrow strip dropped across another narrow strip pinned at both ends. No vertex of either
    /// strip lies over a triangle of the other, so only the edges can meet.
    @Test("a narrow strip hangs on a crossing strip only through edge contacts")
    func crossingStrips() {
        func drop(edges: Bool) -> (Float, World, SoftBodyID) {
            let world = lowGroundWorld()
            // Rows 0.1 apart so no lower vertex sits at z = 0; the upper strip's vertices sit at
            // x = +-0.15, beside the lower strip.
            var lower = SoftBodyDef(settings: makeStrip(columns: 2, columnSpacing: 0.06, rows: 20, rowSpacing: 0.1,
                                                        pinned: { _, z in z == 0 || z == 19 }),
                                    position: Pos(0, Float(0.5), 0))
            lower.collisionThickness = 0.02
            lower.enableEdgeCollision = edges
            _ = world.createSoftBody(lower)
            var upper = SoftBodyDef(settings: makeStrip(columns: 2, columnSpacing: 0.06, rows: 8, rowSpacing: 0.3),
                                    position: Pos(0, Float(1.2), 0), rotation: makeQuatFromAxisAngle(.axisY, 0.5 * Float.pi))
            upper.collisionThickness = 0.02
            upper.enableEdgeCollision = edges
            let id = world.createSoftBody(upper)
            run(world, 120)
            return (maxY(world, id), world, id)
        }
        let (held, _, _) = drop(edges: true)
        let (fell, _, _) = drop(edges: false)
        // Hanging over the lower strip at y = 0.5, less its sag and the thickness.
        #expect(held > 0.35, "\(held)")
        #expect(fell < -1.5, "\(fell)")
    }

    @Test("crossing ropes catch each other between their vertices")
    func crossingRopes() {
        func drop(edges: Bool) -> Float {
            let world = lowGroundWorld()
            var lower = SoftBodyDef(settings: SoftBodySettings.makeRope(count: 10, spacing: 0.2, pinned: false),
                                    position: Pos(Float(-0.9), Float(0.5), 0), rotation: makeQuatFromAxisAngle(.axisZ, 0.5 * Float.pi))
            lower.collisionThickness = 0.03
            lower.enableEdgeCollision = edges
            lower.gravityScale = 0
            let a = world.createSoftBody(lower)
            for v in 0..<10 { world.setSoftBodyVertexInverseMass(a, Int32(v), 0) }
            var upper = SoftBodyDef(settings: SoftBodySettings.makeRope(count: 10, spacing: 0.2, pinned: false),
                                    position: Pos(0, Float(1.5), Float(0.9)), rotation: makeQuatFromAxisAngle(.axisX, 0.5 * Float.pi))
            upper.collisionThickness = 0.03
            upper.enableEdgeCollision = edges
            let b = world.createSoftBody(upper)
            run(world, 90)
            // The middle of the falling rope.
            let p = world.softBodyVertexPositions(b)
            return Float(p.min { abs($0.z) < abs($1.z) }!.y)
        }
        let held = drop(edges: true)
        let fell = drop(edges: false)
        #expect(held > 0.4 && held < 0.6, "\(held)")
        #expect(fell < -1.5, "\(fell)")
    }

    @Test("a coarse cloth drapes over a thin rod between its vertices")
    func thinRod() {
        func drop(edges: Bool, dynamicRod: Bool = false) -> (Float, World, BodyID) {
            let world = lowGroundWorld()
            var rodDef = BodyDef()
            rodDef.position = Pos(0, 1, 0)
            if dynamicRod {
                rodDef.type = .dynamicBody
                rodDef.gravityScale = 0
            }
            let rod = world.createBody(rodDef)
            _ = world.createCapsuleShape(rod, ShapeDef(), Capsule(center1: Vec3(0, 0, -2), center2: Vec3(0, 0, 2), radius: 0.01))
            // Vertices at x = +-0.125, +-0.375: the rod at x = 0 passes between them.
            var cloth = SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 8, rows: 8, spacing: 0.25),
                                    position: Pos(0, Float(2.5), 0))
            cloth.enableEdgeCollision = edges
            cloth.friction = 0.6
            let id = world.createSoftBody(cloth)
            // The rod is reached after about 33 steps; a balanced cloth later slides off a thin rod.
            run(world, dynamicRod ? 45 : 50)
            return (maxY(world, id), world, rod)
        }
        let (held, _, _) = drop(edges: true)
        let (fell, _, _) = drop(edges: false)
        #expect(held > 0.9, "\(held)")
        #expect(fell < 0.5, "\(fell)")

        // A free light rod is carried down by the heavy cloth, at about the speed momentum allows:
        // 64 kg of cloth at 5.5 m/s against a rod of 1.3 kg.
        let (_, world, rod) = drop(edges: true, dynamicRod: true)
        let v = world.bodyLinearVelocity(rod).y
        #expect(v < -4 && v > -6.5, "\(v)")
    }

    @Test("edge contacts keep the result independent of the worker count")
    func threads() {
        func simulate(_ workers: Int32) -> [Pos] {
            let world = lowGroundWorld(workers: workers)
            var rodDef = BodyDef()
            rodDef.position = Pos(0, Float(0.5), 0)
            _ = world.createCapsuleShape(world.createBody(rodDef), ShapeDef(),
                                         Capsule(center1: Vec3(-3, 0, 0), center2: Vec3(3, 0, 0), radius: 0.02))
            var ids: [SoftBodyID] = []
            for k in 0..<4 {
                var def = SoftBodyDef(settings: makeStrip(columns: 3, columnSpacing: 0.08, rows: 12, rowSpacing: 0.15),
                                      position: Pos(0.4 * Float(k) - 0.6, 1 + 0.3 * Float(k), 0),
                                      rotation: makeQuatFromAxisAngle(.axisY, 0.7 * Float(k)))
                def.enableSelfCollision = true
                ids.append(world.createSoftBody(def))
            }
            run(world, 60)
            return ids.flatMap { world.softBodyVertexPositions($0) }
        }
        let serial = simulate(1)
        let parallel = simulate(6)
        #expect(zip(serial, parallel).allSatisfy {
            $0.x.bitPattern == $1.x.bitPattern && $0.y.bitPattern == $1.y.bitPattern && $0.z.bitPattern == $1.z.bitPattern
        })
    }
}

/// A cloth with separate column and row spacing, in the xz plane centered on the origin.
private func makeStrip(
    columns: Int, columnSpacing: Float, rows: Int, rowSpacing: Float, pinned: (Int, Int) -> Bool = { _, _ in false }
) -> SoftBodySettings {
    var settings = SoftBodySettings()
    for z in 0..<rows {
        for x in 0..<columns {
            let position = Vec3((Float(x) - 0.5 * Float(columns - 1)) * columnSpacing, 0,
                                (Float(z) - 0.5 * Float(rows - 1)) * rowSpacing)
            settings.vertices.append(SoftBodyVertex(position: position, invMass: pinned(x, z) ? 0 : 1))
        }
    }
    func index(_ x: Int, _ z: Int) -> UInt32 { UInt32(x + z * columns) }
    for z in 0..<(rows - 1) {
        for x in 0..<(columns - 1) {
            settings.faces.append(SoftBodyFace(index(x, z), index(x, z + 1), index(x + 1, z + 1)))
            settings.faces.append(SoftBodyFace(index(x, z), index(x + 1, z + 1), index(x + 1, z)))
        }
    }
    settings.createConstraints([SoftBodyVertexAttributes()], bendType: .distance)
    settings.optimize()
    return settings
}
