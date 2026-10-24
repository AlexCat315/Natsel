// Scenes for the systems beyond the Box3D port: vehicles, cloth, and volumetric soft bodies.

import Natsel

nonisolated(unsafe) var benchmarkVehicles: [VehicleID] = []

private func makeTerrain(_ world: World) {
    let ground = world.createBody(BodyDef())
    if let mesh = createWaveMesh(xCount: 120, zCount: 120, cellWidth: 2, amplitude: 0.4, rowFrequency: 0.15, columnFrequency: 0.15) {
        _ = world.createMeshShape(ground, ShapeDef(), mesh)
    }
}

// MARK: - vehicles

/// 64 four wheel drive cars with anti-roll bars and cylinder wheel casts on wavy terrain.
func createVehicles(_ world: World) {
    makeTerrain(world)
    benchmarkVehicles = []
    for row in 0..<8 {
        for column in 0..<8 {
            var bodyDef = BodyDef()
            bodyDef.type = .dynamicBody
            bodyDef.position = Pos(Float(column - 4) * 12, 3, Float(row - 4) * 12)
            let body = world.createBody(bodyDef)
            var shapeDef = ShapeDef()
            shapeDef.density = 1500 / (8 * 0.9 * 2 * 0.3)
            _ = world.createHullShape(body, shapeDef, makeBoxHull(0.9, 0.3, 2))

            var def = VehicleDef()
            def.bodyId = body
            def.collisionTester = .cylinder(convexRadiusFraction: 0.5)
            def.maxPitchRollAngle = 60 * 0.0174533
            for (i, p) in [Vec3(0.9, -0.1, 1.5), Vec3(-0.9, -0.1, 1.5), Vec3(0.9, -0.1, -1.5), Vec3(-0.9, -0.1, -1.5)].enumerated() {
                var wheel = WheelSettings()
                wheel.position = p
                wheel.radius = 0.35
                wheel.width = 0.25
                wheel.suspensionMinLength = 0.1
                wheel.suspensionMaxLength = 0.4
                wheel.maxSteerAngle = i < 2 ? 0.5 : 0
                def.wheels.append(wheel)
            }
            def.antiRollBars = [VehicleAntiRollBar(leftWheel: 0, rightWheel: 1), VehicleAntiRollBar(leftWheel: 2, rightWheel: 3)]
            var controller = WheeledVehicleControllerSettings()
            controller.differentials = [
                VehicleDifferentialSettings(leftWheel: 0, rightWheel: 1, engineTorqueRatio: 0.5),
                VehicleDifferentialSettings(leftWheel: 2, rightWheel: 3, engineTorqueRatio: 0.5),
            ]
            def.controller = .wheeled(controller)
            benchmarkVehicles.append(world.createVehicle(def))
        }
    }
}

func stepVehicles(_ world: World, _ stepIndex: Int) {
    for (i, vehicle) in benchmarkVehicles.enumerated() {
        let steer: Float = (stepIndex / 120 + i) % 2 == 0 ? 0.4 : -0.4
        world.setVehicleDriverInput(vehicle, forward: 0.6, right: steer)
    }
}

func destroyVehicles() {
    benchmarkVehicles = []
}

// MARK: - cloth

/// Eight 40x40 cloths falling over static spheres and a pile of dynamic boxes.
func createCloth(_ world: World) {
    let ground = world.createBody(BodyDef())
    _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(50, 1, 50, Vec3(0, -1, 0)))
    let attributes = SoftBodyVertexAttributes(compliance: 1.0e-6, shearCompliance: 1.0e-5, bendCompliance: 1.0e-3)
    let settings = SoftBodySettings.makeCloth(columns: 40, rows: 40, spacing: 0.05, attributes: attributes, bendType: benchmarkEnvironment["NATSEL_BEND"] == "distance" ? .distance : .dihedral)

    for k in 0..<8 {
        let x = Float(k % 4) * 3 - 4.5
        let z = Float(k / 4) * 3 - 1.5
        let obstacle = world.createBody(BodyDef())
        _ = world.createSphereShape(obstacle, ShapeDef(), Sphere(center: Vec3(x, 0.6, z), radius: 0.6))

        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(x + 0.6, 1.6, z)
        let box = world.createBody(bodyDef)
        _ = world.createHullShape(box, ShapeDef(), makeBoxHull(0.15, 0.15, 0.15))

        var def = SoftBodyDef(settings: settings, position: Pos(x, 2, z))
        def.vertexRadius = 0.01
        def.enableSleep = false
        _ = world.createSoftBody(def)
    }
}

// MARK: - soft_cubes

/// 27 volumetric cubes dropped in a pile.
func createSoftCubes(_ world: World) {
    let ground = world.createBody(BodyDef())
    _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(50, 1, 50, Vec3(0, -1, 0)))
    let settings = SoftBodySettings.makeCube(count: 6, spacing: 0.1, compliance: 1.0e-5)
    for i in 0..<27 {
        let p = Pos(Float(i % 3) * 0.7 - 0.7, 1 + Float(i / 9) * 0.7, Float((i / 3) % 3) * 0.7 - 0.7)
        var def = SoftBodyDef(settings: settings, position: p)
        def.vertexRadius = 0.01
        def.enableSleep = false
        _ = world.createSoftBody(def)
    }
}
