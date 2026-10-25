import Testing
@testable import Natsel

private func makeGround(_ world: World) -> BodyID {
    let ground = world.createBody(BodyDef())
    var shapeDef = ShapeDef()
    shapeDef.baseMaterial.friction = 0.6
    _ = world.createHullShape(ground, shapeDef, makeOffsetBoxHull(50, 1, 50, Vec3(0, -1, 0)))
    return ground
}

private func step(_ world: World, _ count: Int) {
    for _ in 0..<count { world.step(1.0 / 60.0, subStepCount: 4) }
}

private func minY(_ world: World, _ id: SoftBodyID) -> Float {
    world.softBodyVertexPositions(id).reduce(Float.greatestFiniteMagnitude) { min($0, Float($1.y)) }
}

@Suite("Soft bodies")
struct SoftBodyTests {
    @Test("dihedral gradients match finite differences")
    func dihedralGradient() {
        let points = [Vec3(0, 0, 0), Vec3(1, 0.1, 0), Vec3(0.4, 0.3, 0.9), Vec3(0.6, -0.4, -0.8)]
        let g = dihedralGradients(points[0], points[1], points[2], points[3])
        #expect(g.valid)
        let analytic = [g.g0, g.g1, g.g2, g.g3]
        let eps: Float = 1.0e-3
        for v in 0..<4 {
            for axis in 0..<3 {
                var plus = points
                var minus = points
                let d = Vec3(axis == 0 ? eps : 0, axis == 1 ? eps : 0, axis == 2 ? eps : 0)
                plus[v] = plus[v] + d
                minus[v] = minus[v] - d
                let numeric = (dihedralAngle(plus[0], plus[1], plus[2], plus[3])
                    - dihedralAngle(minus[0], minus[1], minus[2], minus[3])) / (2 * eps)
                let a = axis == 0 ? analytic[v].x : axis == 1 ? analytic[v].y : analytic[v].z
                #expect(abs(numeric - a) < 0.02, "vertex \(v) axis \(axis): \(numeric) vs \(a)")
            }
        }
        // Coplanar unfolded triangles have zero angle.
        #expect(abs(dihedralAngle(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0.5, 0, 1), Vec3(0.5, 0, -1))) < 1.0e-6)
    }

    @Test("cloth generation creates stretch, shear, and bend constraints")
    func clothSettings() {
        let settings = SoftBodySettings.makeCloth(
            columns: 5, rows: 4, spacing: 0.1,
            attributes: SoftBodyVertexAttributes(compliance: 0, bendCompliance: 1.0e-3), bendType: .dihedral)
        #expect(settings.validate() == nil)
        #expect(settings.vertices.count == 20)
        #expect(settings.faces.count == 2 * 4 * 3)
        // Grid edges: horizontal 4*4 + vertical 5*3, plus two diagonals per quad.
        #expect(settings.edges.count == 16 + 15 + 2 * 12)
        #expect(settings.dihedralBends.count > 0)
        for edge in settings.edges {
            let a = settings.vertices[Int(edge.vertex.x)].position
            let b = settings.vertices[Int(edge.vertex.y)].position
            #expect(abs(distance(a, b) - edge.restLength) < 1.0e-5)
        }
        for bend in settings.dihedralBends {
            #expect(abs(bend.initialAngle) < 1.0e-4)
        }
    }

    @Test("pinned cloth hangs without stretching")
    func hangingCloth() {
        let world = World()
        var attributes = SoftBodyVertexAttributes(compliance: 0, shearCompliance: 0, bendCompliance: 1.0e-4)
        attributes.lraType = .geodesic
        let settings = SoftBodySettings.makeCloth(
            columns: 20, rows: 20, spacing: 0.05, attributes: attributes, bendType: .dihedral,
            pinned: { x, z in z == 0 && (x == 0 || x == 19) })
        #expect(settings.longRangeAttachments.count > 0)
        var def = SoftBodyDef(settings: settings, position: Pos(0, 2, 0))
        def.iterationCount = 10
        let cloth = world.createSoftBody(def)
        #expect(cloth.isNull == false)

        step(world, 240)
        let positions = world.softBodyVertexPositions(cloth)
        // Pinned corners did not move.
        var pinnedCount = 0
        for (i, v) in settings.vertices.enumerated() where v.invMass == 0 {
            #expect(distance(positions[i], Vec3(v.position.x, 2, v.position.z)) < 1.0e-5)
            pinnedCount += 1
        }
        #expect(pinnedCount == 2)
        // The cloth fell, but no edge stretched far.
        #expect(minY(world, cloth) < 1.7)
        var maxStretch: Float = 0
        var meanStretch: Float = 0
        for edge in settings.edges {
            let l = distance(positions[Int(edge.vertex.x)], positions[Int(edge.vertex.y)])
            maxStretch = max(maxStretch, l / edge.restLength)
            meanStretch += l / edge.restLength / Float(settings.edges.count)
        }
        #expect(maxStretch < 1.1)
        #expect(meanStretch < 1.005)
        for p in positions { #expect(isValidPosition(p)) }
    }

    @Test("cloth drapes over a static sphere and box")
    func drape() {
        let world = World()
        _ = makeGround(world)
        let obstacle = world.createBody(BodyDef())
        _ = world.createSphereShape(obstacle, ShapeDef(), Sphere(center: Vec3(0, 0.5, 0), radius: 0.5))
        _ = world.createHullShape(obstacle, ShapeDef(), makeOffsetBoxHull(0.3, 0.3, 0.3, Vec3(1.2, 0.3, 0)))

        let settings = SoftBodySettings.makeCloth(
            columns: 30, rows: 30, spacing: 0.1,
            attributes: SoftBodyVertexAttributes(compliance: 0, shearCompliance: 1.0e-5, bendCompliance: 1.0e-3))
        var def = SoftBodyDef(settings: settings, position: Pos(Float(0.5), Float(1.5), 0))
        def.vertexRadius = 0.01
        let cloth = world.createSoftBody(def)
        step(world, 300)

        let positions = world.softBodyVertexPositions(cloth)
        let center = Vec3(0, 0.5, 0)
        for p in positions {
            #expect(distance(p, center) > 0.5 - 0.02)
            #expect(p.y > -0.02)
            // Not inside the box.
            let local = p - Vec3(1.2, 0.3, 0)
            let inside = abs(local.x) < 0.28 && abs(local.y) < 0.28 && abs(local.z) < 0.28
            #expect(inside == false)
        }
        // Something rests on top of the sphere.
        #expect(positions.contains { $0.y > 0.95 })
        #expect(world.softBodyContacts(cloth).count >= 2)
    }

    @Test("soft cube keeps its volume, comes to rest, and sleeps")
    func softCube() {
        let world = World()
        _ = makeGround(world)
        let settings = SoftBodySettings.makeCube(count: 5, spacing: 0.1, compliance: 1.0e-5)
        #expect(settings.validate() == nil)
        for v in settings.volumes { #expect(v.sixRestVolume > 0) }
        let restVolume = settings.volumes.reduce(0) { $0 + $1.sixRestVolume } / 6
        #expect(abs(restVolume - 0.4 * 0.4 * 0.4) < 1.0e-4)

        var def = SoftBodyDef(settings: settings, position: Pos(0, 1, 0))
        def.vertexRadius = 0.01
        let cube = world.createSoftBody(def)
        let initialVolume = world.softBodyVolume(cube)
        #expect(abs(initialVolume - restVolume) < 1.0e-4)

        step(world, 600)
        #expect(world.softBodyVolume(cube) > 0.8 * restVolume)
        #expect(minY(world, cube) > -0.01)
        #expect(world.softBodyIsAwake(cube) == false)
    }

    @Test("pressure inflates a balloon")
    func balloon() {
        func volume(_ pressure: Float) -> Float {
            let world = World()
            world.gravity = Vec3()
            let settings = SoftBodySettings.makeSphere(radius: 0.5, subdivisions: 2, compliance: 1.0e-3)
            var def = SoftBodyDef(settings: settings)
            def.pressure = pressure
            def.enableSleep = false
            let id = world.createSoftBody(def)
            step(world, 180)
            return world.softBodyVolume(id)
        }
        let flat = volume(0)
        let inflated = volume(2000)
        #expect(inflated > 1.2 * flat)
    }

    @Test("two way coupling: a box rests on a hammock and pushes it down")
    func hammock() {
        let world = World()
        let settings = SoftBodySettings.makeCloth(
            columns: 25, rows: 25, spacing: 0.08,
            attributes: SoftBodyVertexAttributes(compliance: 1.0e-6, shearCompliance: 1.0e-5, bendCompliance: 1.0e-2),
            pinned: { x, z in (x == 0 || x == 24) && (z == 0 || z == 24) })
        var def = SoftBodyDef(settings: settings, position: Pos(0, 1, 0))
        def.vertexRadius = 0.02
        def.friction = 0.5
        def.linearDamping = 1
        let cloth = world.createSoftBody(def)

        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(0, Float(1.5), 0)
        let box = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        shapeDef.density = 50
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var touchedBox = false
        for _ in 0..<600 {
            world.step(1.0 / 60.0, subStepCount: 4)
            touchedBox = touchedBox || world.softBodyContacts(cloth).contains { $0.bodyId == box }
        }
        // Supported by the cloth: below the pins but far above free fall, and settled.
        var lowest: Float = .greatestFiniteMagnitude
        var highest: Float = -.greatestFiniteMagnitude
        for _ in 0..<60 {
            world.step(1.0 / 60.0, subStepCount: 4)
            lowest = min(lowest, Float(world.bodyPosition(box).y))
            highest = max(highest, Float(world.bodyPosition(box).y))
        }
        #expect(lowest > 0.2 && highest < 1.15)
        #expect(highest - lowest < 0.05)
        #expect(touchedBox)
        // The cloth sags under the box.
        #expect(minY(world, cloth) < 0.95)
    }

    @Test("cloth collides with mesh, height field, and compound shapes")
    func triangleGeometry() {
        for kind in 0..<3 {
            let world = World()
            let ground = world.createBody(BodyDef())
            switch kind {
            case 0:
                let mesh = createGridMesh(xCount: 20, zCount: 20, cellWidth: 0.5)!
                _ = world.createMeshShape(ground, ShapeDef(), mesh)
            case 1:
                let field = createGridHeightField(rowCount: 21, columnCount: 21, scale: Vec3(0.5, 1, 0.5))!
                _ = world.createHeightFieldShape(ground, ShapeDef(), field)
            default:
                var compoundDef = CompoundDef()
                var hullDef = CompoundHullDef()
                hullDef.hull = makeBoxHull(5, 0.5, 5)
                hullDef.transform = Transform(Vec3(0, -0.5, 0), .identity)
                compoundDef.hulls = [hullDef]
                let compound = Compound(compoundDef)!
                _ = world.createCompoundShape(ground, ShapeDef(), compound)
            }
            let settings = SoftBodySettings.makeCloth(columns: 10, rows: 10, spacing: 0.1)
            let center: Vec3 = kind == 1 ? Vec3(5, 0.5, 5) : Vec3(0, 0.5, 0)
            var def = SoftBodyDef(settings: settings, position: Pos(center))
            def.vertexRadius = 0.01
            let cloth = world.createSoftBody(def)
            step(world, 180)
            #expect(minY(world, cloth) > -0.02, "geometry \(kind)")
            #expect(minY(world, cloth) < 0.1, "geometry \(kind)")
        }
    }

    @Test("ray casts hit soft body faces")
    func rayCast() {
        let world = World()
        let settings = SoftBodySettings.makeCloth(columns: 5, rows: 5, spacing: 0.25)
        let cloth = world.createSoftBody(SoftBodyDef(settings: settings, position: Pos(0, 1, 0)))
        let hit = world.castRayClosestSoftBody(Pos(Float(0.1), 3, Float(0.1)), Vec3(0, -4, 0))
        #expect(hit.hit)
        #expect(hit.softBodyId == cloth)
        #expect(abs(hit.point.y - 1) < 1.0e-4)
        #expect(hit.normal.y > 0.99)
        // Single sided faces are missed from below.
        let below = world.castRaySoftBody(cloth, Pos(Float(0.1), -1, Float(0.1)), Vec3(0, 4, 0))
        #expect(below.hit == false)
    }

    @Test("kinematic skinned vertices follow the skeleton")
    func skinning() {
        let world = World()
        world.gravity = Vec3()
        var settings = SoftBodySettings.makeRope(count: 5, spacing: 0.1, pinned: false)
        settings.inverseBindMatrices = [.identity]
        for i in 0..<5 {
            let vertex = UInt32(i)
            settings.vertices[i].invMass = i == 0 ? 0 : 1
            settings.skinned.append(SoftBodySkinned(vertex: vertex, joints: SIMD4(0, 0, 0, 0), weights: SIMD4(1, 0, 0, 0),
                                                    maxDistance: i == 0 ? 0 : 0.05))
        }
        settings.calculateRestValues()
        let rope = world.createSoftBody(SoftBodyDef(settings: settings))
        let offset = Vec3(1, 0, 0)
        world.setSoftBodySkinTransforms(rope, [Transform(offset, .identity)])
        step(world, 120)
        let positions = world.softBodyVertexPositions(rope)
        for (i, v) in settings.vertices.enumerated() {
            let target = v.position + offset
            #expect(distance(positions[i], target) < (v.invMass == 0 ? 1.0e-5 : 0.051))
        }
    }

    @Test("parallel simulation matches serial")
    func parallel() {
        func run(_ workers: Int32) -> [Pos] {
            let world = World()
            world.workerCount = workers
            _ = makeGround(world)
            var ids: [SoftBodyID] = []
            for k in 0..<4 {
                let settings = SoftBodySettings.makeCloth(columns: 16, rows: 16, spacing: 0.05)
                ids.append(world.createSoftBody(SoftBodyDef(settings: settings, position: Pos(Float(k), 1, 0))))
            }
            var bodyDef = BodyDef()
            bodyDef.type = .dynamicBody
            bodyDef.position = Pos(0, Float(0.5), 0)
            let box = world.createBody(bodyDef)
            _ = world.createHullShape(box, ShapeDef(), makeBoxHull(0.2, 0.2, 0.2))
            step(world, 120)
            return ids.flatMap { world.softBodyVertexPositions($0) } + [world.bodyPosition(box)]
        }
        #expect(run(1) == run(4))
    }

    @Test("sleeping soft body wakes when a body hits it")
    func wakeOnHit() {
        let world = World()
        _ = makeGround(world)
        let settings = SoftBodySettings.makeCloth(columns: 10, rows: 10, spacing: 0.1)
        let cloth = world.createSoftBody(SoftBodyDef(settings: settings, position: Pos(0, Float(0.05), 0)))
        step(world, 120)
        #expect(world.softBodyIsAwake(cloth) == false)

        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(0, 1, 0)
        let ball = world.createBody(bodyDef)
        _ = world.createSphereShape(ball, ShapeDef(), Sphere(center: Vec3(), radius: 0.1))
        var woke = false
        for _ in 0..<60 {
            world.step(1.0 / 60.0, subStepCount: 4)
            woke = woke || world.softBodyIsAwake(cloth)
        }
        #expect(woke)
    }

    @Test("invalid settings are rejected and destroyed ids are invalid")
    func lifecycle() {
        let world = World()
        var settings = SoftBodySettings()
        #expect(world.createSoftBody(SoftBodyDef(settings: settings)).isNull)
        settings.vertices = [SoftBodyVertex(position: Vec3())]
        settings.edges = [SoftBodyEdge(0, 3)]
        #expect(world.createSoftBody(SoftBodyDef(settings: settings)).isNull)

        let rope = world.createSoftBody(SoftBodyDef(settings: SoftBodySettings.makeRope(count: 3, spacing: 0.1)))
        #expect(world.softBodyCount == 1)
        world.destroySoftBody(rope)
        #expect(world.softBodyIsValid(rope) == false)
        #expect(world.softBodyCount == 0)
        let again = world.createSoftBody(SoftBodyDef(settings: SoftBodySettings.makeRope(count: 3, spacing: 0.1)))
        #expect(again != rope)
        #expect(world.softBodyIsValid(again))
    }

    @Test("debug draw shows soft body edges")
    func debugDraw() {
        let world = World()
        let settings = SoftBodySettings.makeCloth(columns: 4, rows: 4, spacing: 0.1, pinned: { x, z in x == 0 && z == 0 })
        _ = world.createSoftBody(SoftBodyDef(settings: settings))
        var segments = 0
        var points = 0
        var options = DebugDraw()
        options.drawSegment = { _, _, _ in segments += 1 }
        options.drawPoint = { _, _, _ in points += 1 }
        world.draw(options)
        #expect(segments == settings.edges.count)
        #expect(points == 1)
    }
}
