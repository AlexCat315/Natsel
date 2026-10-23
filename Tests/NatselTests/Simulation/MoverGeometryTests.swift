import Testing
@testable import Natsel

// Mover-collide overlap handling ported from test/test_mover.c. These routines must never emit a
// plane with a degenerate normal, even when the mover deeply penetrates the shape. On deep overlap
// the GJK path returns a zero normal; the fallback must replace it with an analytic result.
@Suite("Mover geometry")
struct MoverGeometryTests {
    // MARK: Sphere

    @Test("mover far from a sphere reports nothing")
    func sphereSeparated() {
        let shape = Sphere(center: Vec3(), radius: 0.5)
        let mover = Capsule(center1: Vec3(4, 3, 0), center2: Vec3(6, 3, 0), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndSphere(&result, shape, mover) == 0)
    }

    @Test("mover near a sphere pushes it out along the surface normal")
    func sphereTouching() {
        let shape = Sphere(center: Vec3(), radius: 0.5)
        // Mover core segment runs along X at y = 0.6, leaving it 0.1 inside the 0.7 combined radius.
        let mover = Capsule(center1: Vec3(-1, 0.6, 0), center2: Vec3(1, 0.6, 0), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndSphere(&result, shape, mover) == 1)
        #expect(isNormalized(result.plane.normal))
        // Push-out points from the sphere straight up toward the mover.
        #expect(result.plane.normal.y > 0.99)
        #expect(absFloat(result.plane.offset - 0.1) < 1e-5)
    }

    @Test("mover through a sphere center still yields a unit normal")
    func sphereDeepOverlap() {
        let shape = Sphere(center: Vec3(), radius: 0.5)
        // Mover axis runs straight through the sphere center: the bug case where GJK reports a
        // zero normal.
        let mover = Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndSphere(&result, shape, mover) == 1)
        #expect(isNormalized(result.plane.normal))
        // The fallback axis is perpendicular to the mover axis (X).
        #expect(absFloat(result.plane.normal.x) < 1e-5)
        // Deepest possible penetration: the full combined radius.
        #expect(absFloat(result.plane.offset - 0.7) < 1e-5)
    }

    // MARK: Capsule

    @Test("mover far from a capsule reports nothing")
    func capsuleSeparated() {
        let shape = Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.3)
        let mover = Capsule(center1: Vec3(-1, 5, 0), center2: Vec3(1, 5, 0), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndCapsule(&result, shape, mover) == 0)
    }

    @Test("parallel capsule mover pushes along the face normal")
    func capsuleTouching() {
        let shape = Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.3)
        // Parallel mover 0.4 above, leaving it 0.1 inside the 0.5 combined radius.
        let mover = Capsule(center1: Vec3(-1, 0.4, 0), center2: Vec3(1, 0.4, 0), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndCapsule(&result, shape, mover) == 1)
        #expect(isNormalized(result.plane.normal))
        #expect(result.plane.normal.y > 0.99)
        #expect(absFloat(result.plane.offset - 0.1) < 1e-5)
    }

    @Test("crossing capsule cores yield the perpendicular axis")
    func capsuleDeepOverlap() {
        // Shape capsule along X, mover capsule along Z; their core segments cross exactly at the
        // origin, so GJK reports a zero normal.
        let shape = Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.3)
        let mover = Capsule(center1: Vec3(0, 0, -1), center2: Vec3(0, 0, 1), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndCapsule(&result, shape, mover) == 1)
        #expect(isNormalized(result.plane.normal))
        // The separating axis of two crossing segments is perpendicular to both.
        #expect(absFloat(result.plane.normal.x) < 1e-5)
        #expect(absFloat(result.plane.normal.z) < 1e-5)
        #expect(absFloat(result.plane.offset - 0.5) < 1e-5)
    }

    @Test("coincident capsule cores fall back to a perpendicular of the mover axis")
    func capsuleParallelOverlap() {
        // Mover core segment coincides with the shape core segment: the cross-product axis
        // degenerates, so a perpendicular of the mover axis is used instead.
        let shape = Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.3)
        let mover = Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndCapsule(&result, shape, mover) == 1)
        #expect(isNormalized(result.plane.normal))
        // The fallback axis is perpendicular to the mover axis (X).
        #expect(absFloat(result.plane.normal.x) < 1e-5)
        #expect(absFloat(result.plane.offset - 0.5) < 1e-5)
    }

    // MARK: Hull

    @Test("mover far from a hull reports nothing")
    func hullSeparated() {
        let box = makeBoxHull(0.5, 0.5, 0.5)
        let mover = Capsule(center1: Vec3(-0.3, 5, 0), center2: Vec3(0.3, 5, 0), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndHull(&result, box, mover) == 0)
    }

    @Test("mover above a hull face pushes along +Y")
    func hullTouching() {
        let box = makeBoxHull(0.5, 0.5, 0.5)
        // Mover core segment above the +Y face; the 0.2 radius reaches 0.1 into it.
        let mover = Capsule(center1: Vec3(-0.3, 0.6, 0), center2: Vec3(0.3, 0.6, 0), radius: 0.2)

        var result = PlaneResult()
        #expect(collideMoverAndHull(&result, box, mover) == 1)
        #expect(isNormalized(result.plane.normal))
        #expect(result.plane.normal.y > 0.99)
        #expect(absFloat(result.plane.offset - 0.1) < 1e-4)
    }

    @Test("a mover fully inside a hull drops the plane instead of a zero normal")
    func hullDeepOverlap() {
        let box = makeBoxHull(0.5, 0.5, 0.5)
        // Mover core segment lies entirely inside the box, so GJK reports overlap.
        let mover = Capsule(center1: Vec3(-0.2, 0, 0), center2: Vec3(0.2, 0, 0), radius: 0.1)

        var result = PlaneResult()
        // The overlap guard drops the plane rather than emit a zero normal.
        #expect(collideMoverAndHull(&result, box, mover) == 0)
    }
}

// MARK: - Material routing

extension MoverGeometryTests {
    /// Two separated upward facing triangles on the y = 0 plane, one per material.
    private func makeTwoMaterialMesh() -> TriangleMesh? {
        let vertices = [
            Vec3(-3, 0, -1), Vec3(-2, 0, 1), Vec3(-1, 0, -1),
            Vec3(1, 0, -1), Vec3(2, 0, 1), Vec3(3, 0, -1),
        ]
        let indices: [Int32] = [0, 1, 2, 3, 4, 5]
        return TriangleMesh(vertices: vertices, indices: indices, materials: [0, 1])
    }

    private func makeMaterial(_ friction: Float, _ userId: UInt64) -> SurfaceMaterial {
        var m = SurfaceMaterial.defaultMaterial
        m.friction = friction
        m.userMaterialId = userId
        return m
    }

    @Test("mesh mover reports the per triangle material index")
    func worldMeshMaterials() {
        guard let mesh = makeTwoMaterialMesh() else {
            Issue.record("mesh creation failed")
            return
        }

        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.materials = [makeMaterial(0.2, 1), makeMaterial(0.8, 2)]
        _ = world.createMeshShape(body, shapeDef, mesh, scale: Vec3(1, 1, 1))

        world.step(1.0 / 60.0, subStepCount: 1)

        // Mover hanging just above the first triangle so its radius reaches 0.05 into the surface.
        let moverA = Capsule(center1: Vec3(-2, 0.15, 0), center2: Vec3(-2, 0.35, 0), radius: 0.2)

        var planesA: [PlaneResult] = []
        world.collideMover(Pos(), moverA) { _, planes in
            planesA.append(contentsOf: planes)
            return true
        }

        #expect(planesA.count == 1)
        #expect(planesA[0].plane.normal.y > 0.99)
        #expect(absFloat(planesA[0].plane.offset - 0.05) < 1e-4)
        #expect(absFloat(planesA[0].point.y) < 1e-4)
        #expect(planesA[0].materialIndex == 0)
        #expect(planesA[0].triangleIndex >= 0)
        #expect(planesA[0].triangleIndex < mesh.triangleCount)

        // Same mover over the second triangle.
        let moverB = Capsule(center1: Vec3(2, 0.15, 0), center2: Vec3(2, 0.35, 0), radius: 0.2)

        var planesB: [PlaneResult] = []
        world.collideMover(Pos(), moverB) { _, planes in
            planesB.append(contentsOf: planes)
            return true
        }

        #expect(planesB.count == 1)
        #expect(planesB[0].plane.normal.y > 0.99)
        #expect(planesB[0].materialIndex == 1)
        // The two movers hit different triangles.
        #expect(planesB[0].triangleIndex != planesA[0].triangleIndex)
    }

    @Test("a convex shape reports material index zero")
    func convexMaterialIndex() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.materials = [makeMaterial(0.5, 7)]
        _ = world.createHullShape(body, shapeDef, makeBoxHull(1, 0.5, 1))

        let mover = Capsule(center1: Vec3(0, 0.6, 0), center2: Vec3(0, 0.8, 0), radius: 0.2)

        var planes: [PlaneResult] = []
        world.collideMover(Pos(), mover) { _, found in
            planes.append(contentsOf: found)
            return true
        }

        #expect(planes.isEmpty == false)
        #expect(planes[0].materialIndex == 0)
    }
}

// MARK: - Mesh winding

extension MoverGeometryTests {
    @Test("mesh mover culls the backside of a triangle")
    func meshBackside() {
        guard let data = makeTwoMaterialMesh() else {
            Issue.record("mesh creation failed")
            return
        }
        let mesh = Mesh(data: data, scale: Vec3(1, 1, 1))

        // On the front of the left triangle, so a plane comes back.
        let above = Capsule(center1: Vec3(-2, 0.15, 0), center2: Vec3(-2, 0.35, 0), radius: 0.2)
        let front = collideMoverAndMesh(mesh, above, capacity: 4)
        #expect(front.count == 1)
        #expect(front[0].plane.normal.y > 0.99)
        #expect(front[0].childIndex == 0)
        #expect(front[0].triangleIndex >= 0)
        #expect(front[0].triangleIndex < data.triangleCount)

        // The same spot seen from behind the face is culled.
        let below = Capsule(center1: Vec3(-2, -0.35, 0), center2: Vec3(-2, -0.15, 0), radius: 0.2)
        #expect(collideMoverAndMesh(mesh, below, capacity: 4).isEmpty)
    }

    @Test("mirrored mesh scale keeps the front up and swaps the material")
    func meshMirroredScale() {
        guard let data = makeTwoMaterialMesh() else {
            Issue.record("mesh creation failed")
            return
        }
        // Reflecting the scale flips the triangle winding, and collision swaps it back, so the
        // front stays up. Local x = -2 maps onto the source right triangle, material 1.
        let mesh = Mesh(data: data, scale: Vec3(-1, 1, 1))

        let above = Capsule(center1: Vec3(-2, 0.15, 0), center2: Vec3(-2, 0.35, 0), radius: 0.2)
        let front = collideMoverAndMesh(mesh, above, capacity: 4)
        #expect(front.count == 1)
        #expect(front[0].plane.normal.y > 0.99)
        #expect(front[0].materialIndex == 1)

        let below = Capsule(center1: Vec3(-2, -0.35, 0), center2: Vec3(-2, -0.15, 0), radius: 0.2)
        #expect(collideMoverAndMesh(mesh, below, capacity: 4).isEmpty)
    }
}

// MARK: - Height field mover

extension MoverGeometryTests {
    /// Flat 3x3 vertex field at y = 0, one cell material per entry.
    private func makeFlatField(_ materials: [UInt8], clockwise: Bool) -> HeightField? {
        var definition = HeightFieldDef()
        definition.heights = Array(repeating: 0, count: 9)
        definition.materialIndices = materials
        definition.scale = Vec3(1, 1, 1)
        definition.countX = 3
        definition.countZ = 3
        definition.globalMinimumHeight = -1
        definition.globalMaximumHeight = 1
        definition.clockwiseWinding = clockwise
        return HeightField(definition)
    }

    @Test("height field mover culls the underside")
    func heightFieldBackside() {
        guard let field = makeFlatField([0, 0, 0, 0], clockwise: false) else {
            Issue.record("field creation failed")
            return
        }

        // Standing on the front (upper) face of the default winding.
        let above = Capsule(center1: Vec3(0.3, 0.15, 0.25), center2: Vec3(0.3, 0.35, 0.25), radius: 0.2)
        let front = collideMoverAndHeightField(field, above, capacity: 4)
        #expect(front.count == 1)
        #expect(front[0].plane.normal.y > 0.99)
        #expect(absFloat(front[0].plane.offset - 0.05) < 1e-4)

        // Under the surface is the back side and gets culled.
        let below = Capsule(
            center1: Vec3(0.3, -0.35, 0.25), center2: Vec3(0.3, -0.15, 0.25), radius: 0.2)
        #expect(collideMoverAndHeightField(field, below, capacity: 4).isEmpty)
    }

    @Test("height field mover reports triangle, child and material indices")
    func heightFieldReport() {
        guard let field = makeFlatField([1, 2, 0, 0], clockwise: false) else {
            Issue.record("field creation failed")
            return
        }

        // (0.3, 0.25) sits on the x + z <= 1 side of cell (0,0), which holds triangle 0.
        let first = Capsule(center1: Vec3(0.3, 0.15, 0.25), center2: Vec3(0.3, 0.35, 0.25), radius: 0.2)
        let a = collideMoverAndHeightField(field, first, capacity: 4)
        #expect(a.count == 1)
        #expect(a[0].triangleIndex == 0)
        #expect(a[0].childIndex == 0)
        #expect(a[0].materialIndex == 1)

        // (1.3, 0.3) sits on the x + z <= 2 side of cell (0,1), which holds triangle 2.
        let second = Capsule(center1: Vec3(1.3, 0.15, 0.3), center2: Vec3(1.3, 0.35, 0.3), radius: 0.2)
        let b = collideMoverAndHeightField(field, second, capacity: 4)
        #expect(b.count == 1)
        #expect(b[0].triangleIndex == 2)
        #expect(b[0].materialIndex == 2)
    }

    @Test("a clockwise height field faces down")
    func heightFieldClockwise() {
        guard let field = makeFlatField([0, 0, 0, 0], clockwise: true) else {
            Issue.record("field creation failed")
            return
        }

        // The winding is flipped, so the contactable side is underneath.
        let below = Capsule(
            center1: Vec3(0.3, -0.35, 0.25), center2: Vec3(0.3, -0.15, 0.25), radius: 0.2)
        let hit = collideMoverAndHeightField(field, below, capacity: 4)
        #expect(hit.count == 1)
        #expect(hit[0].plane.normal.y < -0.99)
        #expect(absFloat(hit[0].plane.offset - 0.05) < 1e-4)
        #expect(hit[0].triangleIndex == 0 || hit[0].triangleIndex == 1)

        let above = Capsule(center1: Vec3(0.3, 0.15, 0.25), center2: Vec3(0.3, 0.35, 0.25), radius: 0.2)
        #expect(collideMoverAndHeightField(field, above, capacity: 4).isEmpty)
    }
}

// MARK: - Body mover queries

extension MoverGeometryTests {
    @Test("body mover reports the base material for every convex shape")
    func bodyMaterialIndices() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        let body = world.createBody(def)

        // One shape of each convex type, spaced out along X.
        var shapeDef = ShapeDef()

        _ = world.createSphereShape(
            body, shapeDef, Sphere(center: Vec3(0, 0.5, 0), radius: 0.5))

        _ = world.createHullShape(
            body, shapeDef,
            makeTransformedBoxHull(0.5, 0.5, 0.5, Transform(Vec3(5, 0, 0), Quat.identity)))

        _ = world.createCapsuleShape(
            body, shapeDef,
            Capsule(center1: Vec3(9, 0, 0), center2: Vec3(11, 0, 0), radius: 0.3))

        let bodyTransform = Transform(Vec3(), Quat.identity)

        // Convex shapes report the base material as index 0.
        let movers = [
            Capsule(center1: Vec3(0, 1.15, 0), center2: Vec3(0, 1.35, 0), radius: 0.2),
            Capsule(center1: Vec3(5, 0.65, 0), center2: Vec3(5, 0.85, 0), radius: 0.2),
            Capsule(center1: Vec3(10, 0.45, 0), center2: Vec3(10, 0.65, 0), radius: 0.2),
        ]

        for (i, mover) in movers.enumerated() {
            let planes = world.bodyCollideMover(
                body, Pos(), mover, .defaultFilter, bodyTransform, capacity: 4)
            #expect(planes.count == 1, "mover \(i)")
            guard planes.count == 1 else { continue }
            #expect(planes[0].shapeId.index1 != 0)
            #expect(planes[0].result.plane.normal.y > 0.99, "mover \(i)")
            #expect(planes[0].result.materialIndex == 0, "mover \(i)")
            #expect(planes[0].result.childIndex == 0, "mover \(i)")
        }
    }

    @Test("body mover skips mesh and compound shapes")
    func bodySkipsMeshAndCompound() {
        guard let data = makeTwoMaterialMesh() else {
            Issue.record("mesh creation failed")
            return
        }

        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        let body = world.createBody(def)

        // A mesh and a compound under the same mover; only convex shapes answer.
        var shapeDef = ShapeDef()
        _ = world.createMeshShape(body, shapeDef, data, scale: Vec3(1, 1, 1))

        let planes = world.bodyCollideMover(
            body, Pos(),
            Capsule(center1: Vec3(-2, 0.15, 0), center2: Vec3(-2, 0.35, 0), radius: 0.2),
            .defaultFilter, Transform(Vec3(), Quat.identity), capacity: 4
        )

        // The mesh answers through the world query, not the body query.
        #expect(planes.isEmpty)
    }
}

// MARK: - Compound material remapping

extension MoverGeometryTests {
    /// The hull material and the two mesh materials are distinct, so the compound material table
    /// gets one slot per material. Children bake in hull then mesh order.
    @Test("compound mover remaps child material indices into the shared table")
    func worldCompoundMeshMaterials() {
        guard let mesh = makeTwoMaterialMesh() else {
            Issue.record("mesh creation failed")
            return
        }

        let meshMaterials = [makeMaterial(0.3, 101), makeMaterial(0.6, 202)]
        let hullMaterial = makeMaterial(0.9, 303)

        var definition = CompoundDef()

        var meshChild = CompoundMeshDef()
        meshChild.meshData = mesh
        meshChild.transform = .identity
        meshChild.scale = Vec3(1, 1, 1)
        meshChild.materials = meshMaterials
        definition.meshes.append(meshChild)

        var hullChild = CompoundHullDef()
        hullChild.hull = makeBoxHull(0.5, 0.5, 0.5)
        hullChild.transform = Transform(Vec3(-8, 0, 0), Quat.identity)
        hullChild.material = hullMaterial
        definition.hulls.append(hullChild)

        guard let compound = Compound(definition) else {
            Issue.record("compound creation failed")
            return
        }
        #expect(compound.materials.count == 3)
        #expect(compound.materials[0].userMaterialId == 303)
        #expect(compound.materials[1].userMaterialId == 101)
        #expect(compound.materials[2].userMaterialId == 202)

        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        let body = world.createBody(def)
        _ = world.createCompoundShape(body, ShapeDef(), compound)

        world.step(1.0 / 60.0, subStepCount: 1)

        // Mover on top of the hull child face at y = 0.5.
        let hullMover = Capsule(
            center1: Vec3(-8, 0.65, 0), center2: Vec3(-8, 0.85, 0), radius: 0.2)
        var hullPlanes: [PlaneResult] = []
        world.collideMover(Pos(), hullMover) { _, planes in
            hullPlanes.append(contentsOf: planes)
            return true
        }

        #expect(hullPlanes.count == 1)
        #expect(hullPlanes[0].plane.normal.y > 0.99)
        #expect(hullPlanes[0].childIndex == 0)
        #expect(hullPlanes[0].materialIndex == 0)
        #expect(compound.materials[Int(hullPlanes[0].materialIndex)].userMaterialId == 303)

        // The mesh reports triangle material 1, which the compound remaps through the child
        // material table to the shared slot of meshMaterials[1].
        let meshMover = Capsule(center1: Vec3(2, 0.15, 0), center2: Vec3(2, 0.35, 0), radius: 0.2)
        var meshPlanes: [PlaneResult] = []
        world.collideMover(Pos(), meshMover) { _, planes in
            meshPlanes.append(contentsOf: planes)
            return true
        }

        #expect(meshPlanes.count == 1)
        #expect(meshPlanes[0].plane.normal.y > 0.99)
        #expect(meshPlanes[0].childIndex == 1)
        #expect(meshPlanes[0].materialIndex == 2)
        #expect(compound.materials[Int(meshPlanes[0].materialIndex)].userMaterialId == 202)
    }

    @Test("mesh material indices clamp to the last valid slot")
    func worldMeshMaterialClamp() {
        guard let mesh = makeTwoMaterialMesh() else {
            Issue.record("mesh creation failed")
            return
        }

        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        let body = world.createBody(def)

        // Fewer shape materials than the mesh indexes: out of range indices clamp.
        var shapeDef = ShapeDef()
        shapeDef.materials = [makeMaterial(0.5, 42)]
        _ = world.createMeshShape(body, shapeDef, mesh, scale: Vec3(1, 1, 1))

        world.step(1.0 / 60.0, subStepCount: 1)

        // The mover over the second triangle would index material 1, which does not exist.
        let mover = Capsule(center1: Vec3(2, 0.15, 0), center2: Vec3(2, 0.35, 0), radius: 0.2)
        var planes: [PlaneResult] = []
        world.collideMover(Pos(), mover) { _, found in
            planes.append(contentsOf: found)
            return true
        }

        #expect(planes.count == 1)
        #expect(planes[0].materialIndex == 0)
    }
}
