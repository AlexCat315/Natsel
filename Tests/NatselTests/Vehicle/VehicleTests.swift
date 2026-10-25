import Testing
@testable import Natsel

/// A 1500 kg car on a flat static ground, z forward, y up, x left.
private func makeCar(
    _ world: World, tester: VehicleCollisionTester = .ray, position: Vec3 = Vec3(0, 1.2, 0),
    antiRoll: Bool = true, maxPitchRoll: Float = B3.pi
) -> (VehicleID, BodyID) {
    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody
    bodyDef.position = Pos(position)
    let body = world.createBody(bodyDef)

    let halfWidth: Float = 0.9
    let halfLength: Float = 2.0
    let halfHeight: Float = 0.25
    var shapeDef = ShapeDef()
    shapeDef.density = 1500 / (8 * halfWidth * halfLength * halfHeight)
    _ = world.createHullShape(body, shapeDef, makeBoxHull(halfWidth, halfHeight, halfLength))

    var def = VehicleDef()
    def.bodyId = body
    def.collisionTester = tester
    def.maxPitchRollAngle = maxPitchRoll
    let positions = [
        Vec3(halfWidth, -0.1, halfLength - 0.4), Vec3(-halfWidth, -0.1, halfLength - 0.4),
        Vec3(halfWidth, -0.1, -halfLength + 0.4), Vec3(-halfWidth, -0.1, -halfLength + 0.4),
    ]
    for (i, p) in positions.enumerated() {
        var wheel = WheelSettings()
        wheel.position = p
        wheel.radius = 0.35
        wheel.width = 0.25
        wheel.suspensionMinLength = 0.1
        wheel.suspensionMaxLength = 0.4
        wheel.maxSteerAngle = i < 2 ? 30 * B3.degToRad : 0
        wheel.maxHandBrakeTorque = i < 2 ? 0 : 4000
        def.wheels.append(wheel)
    }
    if antiRoll {
        def.antiRollBars = [VehicleAntiRollBar(leftWheel: 0, rightWheel: 1), VehicleAntiRollBar(leftWheel: 2, rightWheel: 3)]
    }
    var controller = WheeledVehicleControllerSettings()
    controller.differentials = [
        VehicleDifferentialSettings(leftWheel: 0, rightWheel: 1, engineTorqueRatio: 0.5),
        VehicleDifferentialSettings(leftWheel: 2, rightWheel: 3, engineTorqueRatio: 0.5),
    ]
    def.controller = .wheeled(controller)
    return (world.createVehicle(def), body)
}

private func makeGround(_ world: World, friction: Float = 1) -> BodyID {
    let ground = world.createBody(BodyDef())
    var shapeDef = ShapeDef()
    shapeDef.baseMaterial.friction = friction
    _ = world.createHullShape(ground, shapeDef, makeOffsetBoxHull(2000, 1, 2000, Vec3(0, -1, 0)))
    return ground
}

@Suite("Vehicles")
struct VehicleTests {
    @Test("car settles on its suspension")
    func settles() {
        let world = World()
        _ = makeGround(world)
        let (car, body) = makeCar(world)
        #expect(car.isNull == false)

        for _ in 0..<180 { world.step(1.0 / 60.0, subStepCount: 4) }

        let p = world.bodyPosition(body)
        #expect(abs(world.bodyLinearVelocity(body).y) < 0.05)
        // Chassis bottom is above the ground: the wheels carry the car.
        #expect(p.y - 0.25 > 0.15)
        var totalForce: Float = 0
        for w in 0..<4 {
            let state = world.vehicleWheelState(car, Int32(w))!
            #expect(state.hasContact)
            #expect(state.suspensionLength > 0.1 && state.suspensionLength < 0.4)
            totalForce += state.suspensionForce
        }
        // The suspension carries the weight.
        #expect(abs(totalForce - 1500 * 10) < 0.05 * 1500 * 10)
    }

    @Test("throttle accelerates, shifts gears, and brakes stop the car", arguments: [
        VehicleCollisionTester.ray, .sphere, .cylinder(convexRadiusFraction: 0.5),
    ])
    func drive(_ tester: VehicleCollisionTester) {
        let world = World()
        _ = makeGround(world)
        let (car, body) = makeCar(world, tester: tester)
        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }

        var maxGear: Int32 = 0
        for _ in 0..<600 {
            world.setVehicleDriverInput(car, forward: 1, right: 0)
            world.step(1.0 / 60.0, subStepCount: 4)
            maxGear = max(maxGear, world.vehicleDrivetrainState(car)!.currentGear)
        }
        let speed = world.vehicleForwardSpeed(car)
        #expect(speed > 15)
        #expect(maxGear >= 2)
        let rpm = world.vehicleDrivetrainState(car)!.engineRPM
        #expect(rpm >= 1000 && rpm <= 6000)
        // It drove straight.
        #expect(abs(world.bodyPosition(body).x) < 1)

        for _ in 0..<600 {
            world.setVehicleDriverInput(car, forward: 0, right: 0, brake: 1)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        #expect(abs(world.vehicleForwardSpeed(car)) < 0.1)
    }

    @Test("reverse input drives backward")
    func reverse() {
        let world = World()
        _ = makeGround(world)
        let (car, body) = makeCar(world)
        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
        let z0 = world.bodyPosition(body).z
        for _ in 0..<240 {
            world.setVehicleDriverInput(car, forward: -1, right: 0)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        #expect(world.vehicleDrivetrainState(car)!.currentGear < 0)
        #expect(world.bodyPosition(body).z < z0 - 5)
    }

    @Test("steering turns the car right")
    func steer() {
        let world = World()
        _ = makeGround(world)
        let (car, body) = makeCar(world)
        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
        for _ in 0..<240 {
            world.setVehicleDriverInput(car, forward: 0.5, right: 1)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        // With x to the left, a right turn moves toward negative x and yaws clockwise from above.
        #expect(world.bodyPosition(body).x < -2)
        #expect(world.bodyAngularVelocity(body).y < 0)
        #expect(world.vehicleWheelState(car, 0)!.steerAngle < 0)
    }

    @Test("pitch/roll limit keeps a tipped car upright")
    func pitchRoll() {
        let world = World()
        _ = makeGround(world)
        let (_, body) = makeCar(world, position: Vec3(0, 3, 0), maxPitchRoll: 30 * B3.degToRad)
        world.setBodyAngularVelocity(body, Vec3(0, 0, 8))
        for _ in 0..<240 { world.step(1.0 / 60.0, subStepCount: 4) }
        let up = rotateVector(world.bodyRotation(body), Vec3(0, 1, 0))
        #expect(up.y > computeCosSin(32 * B3.degToRad).cosine)
    }

    @Test("parked car with hand brake holds on a slope")
    func handBrake() {
        let world = World()
        let ground = world.createBody(BodyDef())
        var shapeDef = ShapeDef()
        shapeDef.baseMaterial.friction = 1
        let slope = makeQuatFromAxisAngle(Vec3(1, 0, 0), 10 * B3.degToRad)
        _ = world.createHullShape(ground, shapeDef, makeTransformedBoxHull(100, 1, 100, Transform(Vec3(0, -1, 0), slope)))
        let (car, body) = makeCar(world, position: Vec3(0, 1.4, 0))
        world.setBodyTransform(body, Pos(0, Float(1.4), 0), slope)
        for _ in 0..<60 {
            world.setVehicleDriverInput(car, forward: 0, right: 0, brake: 1, handBrake: 1)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        let p0 = world.bodyPosition(body)
        for _ in 0..<240 {
            world.setVehicleDriverInput(car, forward: 0, right: 0, brake: 1, handBrake: 1)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        #expect(distance(world.bodyPosition(body), p0) < 0.05)
    }

    @Test("car pushes a dynamic box it drives onto and the box feels the load")
    func dynamicGround() {
        let world = World()
        _ = makeGround(world)
        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(0, Float(0.25), 0)
        let platform = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        shapeDef.density = 100
        shapeDef.baseMaterial.friction = 1
        _ = world.createHullShape(platform, shapeDef, makeBoxHull(3, 0.25, 4))

        let (car, _) = makeCar(world, position: Vec3(0, 1.6, 0))
        for _ in 0..<120 { world.step(1.0 / 60.0, subStepCount: 4) }
        let state = world.vehicleWheelState(car, 0)!
        #expect(state.hasContact)
        #expect(state.contactBodyId == platform)
        #expect(world.bodyPosition(platform).y < 0.26)
    }

    @Test("vehicle sleeps when idle and wakes on input")
    func sleeping() {
        let world = World()
        _ = makeGround(world)
        let (car, body) = makeCar(world)
        for _ in 0..<300 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.isBodyAwake(body) == false)
        world.setVehicleDriverInput(car, forward: 1, right: 0)
        #expect(world.isBodyAwake(body))
        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.vehicleForwardSpeed(car) > 1)
    }

    @Test("destroying the chassis removes the vehicle")
    func destroyChassis() {
        let world = World()
        _ = makeGround(world)
        let (car, body) = makeCar(world)
        world.step(1.0 / 60.0, subStepCount: 4)
        world.destroyBody(body)
        world.step(1.0 / 60.0, subStepCount: 4)
        #expect(world.vehicleIsValid(car) == false)
        #expect(world.vehicleCount == 0)
    }

    @Test("tracked vehicle drives and turns on the spot")
    func tracked() {
        let world = World()
        _ = makeGround(world)
        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(0, Float(1.2), 0)
        let body = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        shapeDef.density = 8000 / (8 * 1.5 * 0.5 * 3)
        _ = world.createHullShape(body, shapeDef, makeBoxHull(1.5, 0.5, 3))

        var def = VehicleDef()
        def.bodyId = body
        var controller = TrackedVehicleControllerSettings()
        controller.engine.maxTorque = 3000
        controller.transmission.clutchStrength = 200
        controller.tracks[0].wheels = []
        controller.tracks[1].wheels = []
        for side in 0..<2 {
            let x: Float = side == 0 ? 1.4 : -1.4
            for k in 0..<5 {
                var wheel = WheelSettings()
                wheel.position = Vec3(x, -0.4, -2.4 + 1.2 * Float(k))
                wheel.radius = 0.4
                wheel.suspensionMinLength = 0.2
                wheel.suspensionMaxLength = 0.5
                wheel.suspensionFrequency = 1.5
                wheel.trackLateralFriction = 1
                controller.tracks[side].wheels.append(Int32(def.wheels.count))
                def.wheels.append(wheel)
            }
            controller.tracks[side].drivenWheel = controller.tracks[side].wheels[4]
        }
        def.controller = .tracked(controller)
        let tank = world.createVehicle(def)
        #expect(tank.isNull == false)

        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
        for _ in 0..<300 {
            world.setTrackedVehicleDriverInput(tank, forward: 1, leftRatio: 1, rightRatio: 1)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        #expect(world.vehicleForwardSpeed(tank) > 3)
        #expect(abs(world.bodyPosition(body).x) < 1)

        for _ in 0..<300 {
            world.setTrackedVehicleDriverInput(tank, forward: 0, leftRatio: 0, rightRatio: 0, brake: 1)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        let p = world.bodyPosition(body)
        for _ in 0..<240 {
            world.setTrackedVehicleDriverInput(tank, forward: 1, leftRatio: 1, rightRatio: -1)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        // Pivot turn: yaw rate without much travel.
        #expect(abs(world.bodyAngularVelocity(body).y) > 0.2)
        #expect(distance(world.bodyPosition(body), p) < 2)
    }

    @Test("motorcycle stays upright and leans into turns")
    func motorcycle() {
        let world = World()
        _ = makeGround(world)
        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(0, Float(0.9), 0)
        let body = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        shapeDef.density = 240 / (8 * 0.2 * 0.3 * 0.8)
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.2, 0.3, 0.8))

        var def = VehicleDef()
        def.bodyId = body
        var front = WheelSettings()
        front.position = Vec3(0, -0.2, 0.75)
        front.radius = 0.31
        front.suspensionMinLength = 0.1
        front.suspensionMaxLength = 0.3
        front.maxSteerAngle = 30 * B3.degToRad
        front.inertia = 0.2
        var rear = front
        rear.position = Vec3(0, -0.2, -0.75)
        rear.maxSteerAngle = 0
        def.wheels = [front, rear]
        var controller = MotorcycleControllerSettings()
        controller.wheeled.engine.maxTorque = 150
        controller.wheeled.engine.maxRPM = 9000
        controller.wheeled.transmission.shiftUpRPM = 7000
        controller.wheeled.differentials = [VehicleDifferentialSettings(leftWheel: -1, rightWheel: 1)]
        controller.wheeled.differentials[0].differentialRatio = 1.93 * 40 / 16
        def.controller = .motorcycle(controller)
        let bike = world.createVehicle(def)

        for _ in 0..<300 {
            world.setVehicleDriverInput(bike, forward: 0.6, right: 0)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        let up = rotateVector(world.bodyRotation(body), Vec3(0, 1, 0))
        #expect(up.y > 0.95)
        #expect(world.vehicleForwardSpeed(bike) > 5)

        for _ in 0..<60 {
            world.setVehicleDriverInput(bike, forward: 0.3, right: 0.3)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        let lean = world.vehicleDrivetrainState(bike)!.leanAngle
        #expect(lean > 2 * B3.degToRad)
        let up2 = rotateVector(world.bodyRotation(body), Vec3(0, 1, 0))
        #expect(up2.y > 0.5)
    }

    @Test("limited slip and curves")
    func helpers() {
        let curve = LinearCurve([Vec2(1, 2), Vec2(0, 0), Vec2(2, 1)])
        #expect(curve.evaluate(-1) == 0)
        #expect(curve.evaluate(0.5) == 1)
        #expect(curve.evaluate(1.5) == 1.5)
        #expect(curve.evaluate(9) == 1)

        let open = limitedSlipFractions(0.5, 10, 1, .greatestFiniteMagnitude)
        #expect(open.left == 0.5)
        let locked = limitedSlipFractions(0.5, 10, 1, 1.4)
        #expect(locked.left == 0 && locked.right == 1)
        let partial = limitedSlipFractions(0.5, 1.2, 1, 1.4)
        #expect(partial.left < 0.5 && partial.left > 0)
    }

    @Test("wheel transform follows suspension and steering")
    func wheelTransform() {
        let world = World()
        _ = makeGround(world)
        let (car, _) = makeCar(world)
        for _ in 0..<60 {
            world.setVehicleDriverInput(car, forward: 0, right: 1)
            world.step(1.0 / 60.0, subStepCount: 4)
        }
        let local = world.vehicleWheelLocalTransform(car, 0)!
        let state = world.vehicleWheelState(car, 0)!
        let settings = world.vehicleWheelSettings(car, 0)!
        #expect(abs(local.p.y - (settings.position.y - state.suspensionLength)) < 1.0e-4)
        // The axle (model x) is rotated by the steering angle about up.
        let axle = rotateVector(local.q, Vec3(1, 0, 0))
        #expect(abs(axle.y) < 1.0e-3)
        #expect(abs(atan2Approx(-axle.z, axle.x) - state.steerAngle) < 0.01)
    }

    @Test("debug draw shows wheels and contacts")
    func debugDraw() {
        let world = World()
        _ = makeGround(world)
        _ = makeCar(world)
        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }
        var segments = 0
        var points = 0
        var options = DebugDraw()
        options.drawShapes = false
        options.drawSegment = { _, _, _ in segments += 1 }
        options.drawPoint = { _, _, _ in points += 1 }
        world.draw(options)
        // Per wheel: suspension, 16 rim segments, spoke, two contact lines.
        #expect(segments >= 4 * 20)
        #expect(points >= 4)
        options.drawVehicles = false
        segments = 0
        world.draw(options)
        #expect(segments == 0)
    }

    @Test("parallel wheel casts match serial")
    func parallelCasts() {
        func run(_ workers: Int32) -> [Pos] {
            let world = World()
            world.workerCount = workers
            _ = makeGround(world)
            var cars: [(VehicleID, BodyID)] = []
            for k in 0..<10 {
                cars.append(makeCar(world, tester: .cylinder(convexRadiusFraction: 0.5), position: Vec3(Float(k) * 4, 1.2, 0)))
            }
            for step in 0..<120 {
                for (i, car) in cars.enumerated() {
                    world.setVehicleDriverInput(car.0, forward: 0.8, right: (step / 30 + i) % 2 == 0 ? 0.5 : -0.5)
                }
                world.step(1.0 / 60.0, subStepCount: 4)
            }
            return cars.map { world.bodyPosition($0.1) }
        }
        #expect(run(1) == run(4))
    }
}
