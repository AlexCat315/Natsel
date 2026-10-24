// Scene builders ported from shared/benchmarks.c. Only the scenes exercised by the Swift
// benchmark harness live here.

import Foundation
import Natsel

// MARK: - Shared storage

/// Body counts for the last benchmark, mirroring the C harness counters report.
nonisolated(unsafe) var worldCounters = (body: Int32(0), shape: Int32(0), contact: Int32(0))

private func recordCounters(_ world: World) {
    worldCounters = (world.bodyCount, 0, world.contactCount)
}

// MARK: - joint_grid

func createJointGrid(_ world: World) {
    world.enableSleeping(false)

    let n = 100

    var bodyDef = BodyDef()
    bodyDef.enableSleep = false
    var shapeDef = ShapeDef()
    shapeDef.filter.categoryBits = 2
    shapeDef.filter.maskBits = ~2
    let sphere = Sphere(center: Vec3(), radius: 0.4)

    var jointDef = SphericalJointDef()
    var bodies = Array(repeating: BodyID.null, count: n * n)
    var index = 0

    for k in 0..<n {
        for i in 0..<n {
            let fk = Float(k)
            let fi = Float(i)

            bodyDef.type = i == 0 ? .staticBody : .dynamicBody
            bodyDef.position = Pos(fk, -fi, 0)

            let body = world.createBody(bodyDef)
            _ = world.createSphereShape(body, shapeDef, sphere)

            if i > 0 {
                jointDef.base.bodyIdA = bodies[index - 1]
                jointDef.base.bodyIdB = body
                jointDef.base.localFrameA.p = Vec3(0, -0.5, 0)
                jointDef.base.localFrameB.p = Vec3(0, 0.5, 0)
                _ = world.createSphericalJoint(jointDef)
            }

            if k > 0 {
                jointDef.base.bodyIdA = bodies[index - n]
                jointDef.base.bodyIdB = body
                jointDef.base.localFrameA.p = Vec3(0.5, 0, 0)
                jointDef.base.localFrameB.p = Vec3(-0.5, 0, 0)
                _ = world.createSphericalJoint(jointDef)
            }

            bodies[index] = body
            index += 1
        }
    }
    recordCounters(world)
}

// MARK: - large_pyramid

func createLargePyramid(_ world: World) {
    world.enableSleeping(false)

    let baseCount = 100

    do {
        var bodyDef = BodyDef()
        bodyDef.position = Pos(0, -1, 0)
        let ground = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(400, 1, 400))
    }

    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody

    var shapeDef = ShapeDef()
    shapeDef.density = 100

    let h: Float = 0.5
    let box = makeBoxHull(h, h, h)
    let shift = h

    for i in 0..<baseCount {
        let y = (2 * Float(i) + 1) * shift
        for j in i..<baseCount {
            let x = (Float(i) + 1) * shift + 2 * (Float(j - i)) * shift - h * Float(baseCount)
            bodyDef.position = Pos(x, y, 0)
            let body = world.createBody(bodyDef)
            _ = world.createHullShape(body, shapeDef, box)
        }
    }
    recordCounters(world)
}

// MARK: - many_pyramids

private func createSmallPyramid(_ world: World, baseCount: Int, extent: Float, centerX: Float, baseZ: Float) {
    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody
    bodyDef.enableSleep = false

    var shapeDef = ShapeDef()
    shapeDef.density = 100

    let box = makeBoxHull(extent, extent, extent)

    for i in 0..<baseCount {
        let y = (2 * Float(i) + 1) * extent
        for j in i..<baseCount {
            let x = (Float(i) + 1) * extent + 2 * Float(j - i) * extent + centerX - 0.5
            bodyDef.position = Pos(x, y, baseZ)
            let body = world.createBody(bodyDef)
            _ = world.createHullShape(body, shapeDef, box)
        }
    }
}

func createManyPyramids(_ world: World) {
    let baseCount = 10
    let extent: Float = 0.5
    let rowCount = 14
    let columnCount = 14
    let groundExtent = extent * Float(columnCount) * (Float(baseCount) + 1)

    do {
        var bodyDef = BodyDef()
        bodyDef.position = Pos(0, -1, 0)
        let ground = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(groundExtent, 1, groundExtent))
    }

    let baseWidth = 2 * extent * Float(baseCount)
    let baseZ = -groundExtent + 2 * extent
    let deltaZ = 2 * (groundExtent - 2 * extent) / Float(rowCount - 1)

    var z = baseZ
    for _ in 0..<rowCount {
        for j in 0..<columnCount {
            let centerX = -groundExtent + Float(j) * (baseWidth + 2 * extent) + 2 * extent
            createSmallPyramid(world, baseCount: baseCount, extent: extent, centerX: centerX, baseZ: z)
        }
        z += deltaZ
    }
    recordCounters(world)
}

// MARK: - washer

func createWasher(_ world: World) {
    var groundId = BodyID.null
    do {
        var bodyDef = BodyDef()
        bodyDef.position.y = -1
        groundId = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(groundId, shapeDef, makeBoxHull(60, 1, 60))
    }

    do {
        let motorSpeed: Float = 25

        var bodyDef = BodyDef()
        bodyDef.position = Pos(0, 21, 0)
        bodyDef.type = .kinematicBody
        bodyDef.angularVelocity = Vec3(0, 0, (scenePi / 180) * motorSpeed)
        bodyDef.linearVelocity = Vec3(0.001, -0.002, 0)

        let bodyId = world.createBody(bodyDef)
        var shapeDef = ShapeDef()

        let r0: Float = 14
        let r1: Float = 16
        let r2: Float = 18
        let nd = Vec3(0, 0, -10)
        let pd = Vec3(0, 0, 10)

        let angle = scenePi / 18
        let q = makeQuatFromAxisAngle(Vec3(0, 0, 1), angle)
        let qo = makeQuatFromAxisAngle(Vec3(0, 0, 1), 0.1 * angle)
        var u1 = Vec3(1, 0, 0)
        for i in 0..<36 {
            let u2: Vec3
            if i == 35 {
                u2 = Vec3(1, 0, 0)
            } else {
                u2 = rotateVector(q, u1)
            }

            do {
                let a1 = invRotateVector(qo, u1)
                let a2 = rotateVector(qo, u2)
                let p1 = mulAdd(nd, r1, a1)
                let p2 = mulAdd(nd, r2, a1)
                let p3 = mulAdd(nd, r1, a2)
                let p4 = mulAdd(nd, r2, a2)
                let p5 = mulAdd(pd, r1, a1)
                let p6 = mulAdd(pd, r2, a1)
                let p7 = mulAdd(pd, r1, a2)
                let p8 = mulAdd(pd, r2, a2)
                if let hull = createHull([p1, p2, p3, p4, p5, p6, p7, p8], maxVertexCount: 8) {
                    _ = world.createHullShape(bodyId, shapeDef, hull)
                }
            }

            if i % 9 == 0 {
                let p1 = mulAdd(nd, r0, u1)
                let p2 = mulAdd(nd, r1, u1)
                let p3 = mulAdd(nd, r0, u2)
                let p4 = mulAdd(nd, r1, u2)
                let p5 = mulAdd(pd, r0, u1)
                let p6 = mulAdd(pd, r1, u1)
                let p7 = mulAdd(pd, r0, u2)
                let p8 = mulAdd(pd, r1, u2)
                if let hull = createHull([p1, p2, p3, p4, p5, p6, p7, p8], maxVertexCount: 8) {
                    _ = world.createHullShape(bodyId, shapeDef, hull)
                }
            }

            u1 = u2
        }
    }

    let gridCount = 20
    let a: Float = 0.2

    let cube = makeBoxHull(a, a, a)
    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody
    var shapeDef = ShapeDef()

    var x = -2 * a * Float(gridCount)
    for _ in 0..<gridCount {
        var y = -2 * a * Float(gridCount) + 21
        for _ in 0..<gridCount {
            var z = -2 * a * Float(gridCount)
            for _ in 0..<gridCount {
                bodyDef.position = Pos(x, y, z)
                let bodyId = world.createBody(bodyDef)
                _ = world.createHullShape(bodyId, shapeDef, cube)
                z += 4 * a
            }
            y += 4 * a
        }
        x += 4 * a
    }
    recordCounters(world)
}

// MARK: - convex_pile

func createConvexPile(_ world: World) {
    do {
        var bodyDef = BodyDef()
        bodyDef.position = Pos(0, -1, 0)
        let ground = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(250, 1, 250))
    }

    let countX = 8
    let countZ = 8
    let layers = 80
    let amplitude: Float = 2
    let scatter = 2 * amplitude

    guard let convex = createComplexHull(amplitude) else { return }

    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody
    var shapeDef = ShapeDef()

    for layer in 0..<layers {
        for z in 0..<countZ {
            for x in 0..<countX {
                let posX = (Float(x) - 0.5 * Float(countX)) * scatter
                let posZ = (Float(z) - 0.5 * Float(countZ)) * scatter
                let posY = amplitude + 2 * amplitude * Float(layer)
                bodyDef.position = Pos(posX, posY, posZ)
                let bodyId = world.createBody(bodyDef)
                _ = world.createHullShape(bodyId, shapeDef, convex)
            }
        }
    }
    recordCounters(world)
}

// MARK: - spinner

nonisolated(unsafe) private var spinnerMesh: TriangleMesh?

private func createInwardCylinderMesh(radius: Float, halfDepth: Float, yCenter: Float, segmentCount: Int) -> TriangleMesh {
    let vertexCount = 2 * segmentCount + 2
    let triangleCount = 4 * segmentCount

    var vertices = Array(repeating: Vec3(), count: vertexCount)
    var indices = Array(repeating: Int32(0), count: 3 * triangleCount)

    for i in 0..<segmentCount {
        let angle = 2 * scenePi * Float(i) / Float(segmentCount)
        let cs = computeCosSin(angle)
        let x = radius * cs.cosine
        let y = radius * cs.sine + yCenter
        vertices[2 * i + 0] = Vec3(x, y, -halfDepth)
        vertices[2 * i + 1] = Vec3(x, y, halfDepth)
    }

    let centerNegative = 2 * segmentCount
    let centerPositive = 2 * segmentCount + 1
    vertices[centerNegative] = Vec3(0, yCenter, -halfDepth)
    vertices[centerPositive] = Vec3(0, yCenter, halfDepth)

    var index = 0
    for i in 0..<segmentCount {
        let j = (i + 1) % segmentCount

        indices[index] = Int32(2 * i + 0)
        indices[index + 1] = Int32(2 * i + 1)
        indices[index + 2] = Int32(2 * j + 1)
        index += 3

        indices[index] = Int32(2 * i + 0)
        indices[index + 1] = Int32(2 * j + 1)
        indices[index + 2] = Int32(2 * j + 0)
        index += 3

        indices[index] = Int32(centerNegative)
        indices[index + 1] = Int32(2 * i + 0)
        indices[index + 2] = Int32(2 * j + 0)
        index += 3

        indices[index] = Int32(centerPositive)
        indices[index + 1] = Int32(2 * j + 1)
        indices[index + 2] = Int32(2 * i + 1)
        index += 3
    }

    var options = MeshBuildOptions()
    options.identifyEdges = true
    return TriangleMesh(vertices: vertices, indices: indices, options: options)!
}

nonisolated(unsafe) var spinnerProbeBodies: [BodyID] = []

func createSpinner(_ world: World) {
    let radius: Float = 12
    let halfDepth: Float = 1
    let yCenter: Float = radius

    let groundId: BodyID
    do {
        var bodyDef = BodyDef()
        groundId = world.createBody(bodyDef)

        let mesh = createInwardCylinderMesh(radius: radius, halfDepth: halfDepth, yCenter: yCenter, segmentCount: 90)
        spinnerMesh = mesh

        var shapeDef = ShapeDef()
        shapeDef.baseMaterial.friction = 0.1
        _ = world.createMeshShape(groundId, shapeDef, mesh)
    }

    do {
        let halfLength = 0.5 * radius

        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(0, yCenter - halfLength, 0)
        bodyDef.enableSleep = false

        let spinnerId = world.createBody(bodyDef)
        spinnerProbeBodies = [groundId, spinnerId]

        let paddle = makeBoxHull(0.3, halfLength + 0.1, halfDepth + 0.1)
        var shapeDef = ShapeDef()
        shapeDef.baseMaterial.friction = 0
        _ = world.createHullShape(spinnerId, shapeDef, paddle)

        var jointDef = RevoluteJointDef()
        jointDef.base.bodyIdA = groundId
        jointDef.base.bodyIdB = spinnerId
        jointDef.base.localFrameA.p = Vec3(0, yCenter - halfLength, 0)
        jointDef.enableMotor = true
        jointDef.motorSpeed = 8
        jointDef.maxMotorTorque = .greatestFiniteMagnitude
        _ = world.createRevoluteJoint(jointDef)
    }

    let sphere = Sphere(center: Vec3(), radius: 0.2)

    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody

    var shapeDef = ShapeDef()
    shapeDef.baseMaterial.friction = 0.1
    shapeDef.baseMaterial.restitution = 0.1
    shapeDef.density = 0.25

    let bodyCount = 1500
    let spacing: Float = 0.55
    let yStart: Float = 2
    let d = yCenter - yStart
    let chordHalfWidth = (radius * radius - d * d).squareRoot()
    let xLimit = chordHalfWidth - 2 * spacing
    let zLimit = halfDepth - 2 * sphere.radius

    var x = -xLimit
    var y = yStart
    var z = -zLimit

    for _ in 0..<bodyCount {
        bodyDef.position = Pos(x, y, z)
        let bodyId = world.createBody(bodyDef)
        _ = world.createSphereShape(bodyId, shapeDef, sphere)

        z += spacing
        if z > zLimit {
            z = -zLimit
            x += spacing
            if x > xLimit {
                x = -xLimit
                y += spacing
            }
        }
    }
    recordCounters(world)
}

func destroySpinner() {
    spinnerMesh = nil
}

// MARK: - sleep

private let sleepPyramidCount = 10
nonisolated(unsafe) private var sleepBodyA = Array(repeating: BodyID.null, count: sleepPyramidCount)
nonisolated(unsafe) private var sleepBodyB = Array(repeating: BodyID.null, count: sleepPyramidCount)

private func createSleepPyramid(
    _ world: World, baseCount: Int, extent: Float, centerX: Float, index: Int
) {
    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody
    bodyDef.sleepThreshold = 1

    var shapeDef = ShapeDef()
    let box = makeBoxHull(extent, extent, extent)

    for i in 0..<baseCount {
        let y = (2 * Float(i) + 1) * extent
        for j in i..<baseCount {
            let x = (Float(i) + 1) * extent + 2 * Float(j - i) * extent + centerX
            bodyDef.position = Pos(x, y, 0)
            let bodyId = world.createBody(bodyDef)
            _ = world.createHullShape(bodyId, shapeDef, box)

            if i == 0 && j == 0 {
                sleepBodyA[index] = bodyId
            } else if i == 1 && j == 1 {
                sleepBodyB[index] = bodyId
            }
        }
    }
}

func createSleep(_ world: World) {
    sleepBodyA = Array(repeating: BodyID.null, count: sleepPyramidCount)
    sleepBodyB = Array(repeating: BodyID.null, count: sleepPyramidCount)

    let baseCount = 20
    let extent: Float = 0.5

    let baseWidth = 2 * extent * Float(baseCount)
    let pitch = baseWidth + 8 * extent
    let span = pitch * Float(sleepPyramidCount - 1)

    do {
        var bodyDef = BodyDef()
        bodyDef.position = Pos(0, -1, 0)
        let ground = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(0.5 * span + baseWidth, 1, 4))
    }

    for i in 0..<sleepPyramidCount {
        let centerX = -0.5 * span + Float(i) * pitch - 0.5 * baseWidth
        createSleepPyramid(world, baseCount: baseCount, extent: extent, centerX: centerX, index: i)
    }
    recordCounters(world)
}

func stepSleep(_ world: World, _ stepCount: Int) {
    for i in 0..<sleepPyramidCount {
        if world.bodyIsAwake(sleepBodyA[i]) == false {
            var jointDef = FilterJointDef()
            jointDef.base.bodyIdA = sleepBodyA[i]
            jointDef.base.bodyIdB = sleepBodyB[i]
            let jointId = world.createFilterJoint(jointDef)
            world.destroyJoint(Int32(truncatingIfNeeded: jointId.stored >> 32) - 1, true)
            break
        }
    }
}

// MARK: - trees

nonisolated(unsafe) private var treeMesh: TriangleMesh?

private func createTrees(_ world: World, scale: Int) {
    treeMesh = nil

    let tilt: Float = 0
    var bodyDef = BodyDef()
    bodyDef.position = Pos(0, 0, 0)
    bodyDef.rotation = makeQuatFromAxisAngle(Vec3(1, 0, 0), tilt)
    let groundId = world.createBody(bodyDef)

    let xCount = scale * 150
    let zCount = scale * 200

    let cellWidth = 1 / Float(scale)
    let amplitude: Float = 0.4
    let rowHz: Float = 0.05
    let columnHz: Float = 0.1

    guard let mesh = createWaveMesh(
        xCount: xCount, zCount: zCount, cellWidth: cellWidth,
        amplitude: amplitude, rowFrequency: rowHz, columnFrequency: columnHz
    ) else { return }
    treeMesh = mesh
    var shapeDef = ShapeDef()
    _ = world.createMeshShape(groundId, shapeDef, mesh)

    bodyDef.type = .dynamicBody
    bodyDef.sleepThreshold = 0.2
    bodyDef.rotation = Quat.identity

    let bodyCount = 50

    shapeDef.baseMaterial.friction = 0.9
    shapeDef.baseMaterial.rollingResistance = 0.05
    shapeDef.updateBodyMass = false
    shapeDef.density = 1

    let hullCount = 22
    var hulls: [Hull] = []

    var y: Float = 1
    var r: Float = 0.75
    let l: Float = 1.5
    for _ in 0..<hullCount {
        if let hull = createCylinder(l + 2 * r, r, y - r, 6) {
            hulls.append(hull)
        }
        y += l + 2 * r
        r = 0.95 * r
    }

    var angularVelocity: Float = -0.5
    var z: Float = -70
    let cs = computeCosSin(tilt)
    let yTilt = cs.sine / cs.cosine
    for bodyIndex in 0..<bodyCount {
        bodyDef.position = Pos(0, 1 - z * yTilt, z)
        let bodyId = world.createBody(bodyDef)

        for shapeIndex in 0..<22 {
            if shapeIndex < hulls.count {
                _ = world.createHullShape(bodyId, shapeDef, hulls[shapeIndex])
            }
        }

        let velocityScale = 0.5 + (0.5 * Float(bodyIndex)) / Float(bodyCount)
        world.applyMassFromShapes(bodyId)
        let center = world.bodyWorldCenter(bodyId)
        let omega = Vec3(0, 0, velocityScale * angularVelocity)
        let v = cross(omega, center - bodyDef.position)
        world.setBodyAngularVelocity(bodyId, omega)
        world.setBodyLinearVelocity(bodyId, v)

        z += 3
        angularVelocity = -angularVelocity
    }
    recordCounters(world)
}

func createTrees25(_ world: World) { createTrees(world, scale: 4) }
func createTrees50(_ world: World) { createTrees(world, scale: 2) }
func createTrees100(_ world: World) { createTrees(world, scale: 1) }

func destroyTrees() {
    treeMesh = nil
}

/// Divergence probe: the spinner paddle and its revolute motor alone.
func createPaddle(_ world: World) {
    let groundId = world.createBody(BodyDef())
    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody
    bodyDef.position = Pos(0, 6, 0)
    bodyDef.enableSleep = false
    let spinnerId = world.createBody(bodyDef)
    var shapeDef = ShapeDef()
    shapeDef.baseMaterial.friction = 0
    _ = world.createHullShape(spinnerId, shapeDef, makeBoxHull(0.3, 6.1, 1.1))
    var jointDef = RevoluteJointDef()
    jointDef.base.bodyIdA = groundId
    jointDef.base.bodyIdB = spinnerId
    jointDef.base.localFrameA.p = Vec3(0, 6, 0)
    jointDef.enableMotor = true
    jointDef.motorSpeed = 8
    jointDef.maxMotorTorque = .greatestFiniteMagnitude
    _ = world.createRevoluteJoint(jointDef)
}

/// B3_PI. `Float.pi` rounds toward zero; the C literal rounds to nearest.
let scenePi: Float = 3.14159265359
