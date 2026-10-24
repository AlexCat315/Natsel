// Vehicle storage, creation, input, and inspection.

/// The scalar wheel settings used by the solver. Plain data, so copying a wheel never touches
/// reference counts; the tire curves live in `VehicleRecord.wheelSettings`.
struct WheelParameters: BitwiseCopyable {
    var position: Vec3
    var suspensionForcePoint: Vec3
    var enableSuspensionForcePoint: Bool
    var suspensionDirection: Vec3
    var steeringAxis: Vec3
    var wheelUp: Vec3
    var wheelForward: Vec3
    var suspensionMinLength: Float
    var suspensionMaxLength: Float
    var suspensionPreloadLength: Float
    var suspensionFrequency: Float
    var suspensionDampingRatio: Float
    var suspensionStiffness: Float
    var suspensionDamping: Float
    var radius: Float
    var width: Float
    var inertia: Float
    var angularDamping: Float
    var maxSteerAngle: Float
    var maxBrakeTorque: Float
    var maxHandBrakeTorque: Float
    var trackLongitudinalFriction: Float
    var trackLateralFriction: Float

    init(_ s: WheelSettings) {
        position = s.position
        suspensionForcePoint = s.suspensionForcePoint
        enableSuspensionForcePoint = s.enableSuspensionForcePoint
        suspensionDirection = s.suspensionDirection
        steeringAxis = s.steeringAxis
        wheelUp = s.wheelUp
        wheelForward = s.wheelForward
        suspensionMinLength = s.suspensionMinLength
        suspensionMaxLength = s.suspensionMaxLength
        suspensionPreloadLength = s.suspensionPreloadLength
        suspensionFrequency = s.suspensionFrequency
        suspensionDampingRatio = s.suspensionDampingRatio
        suspensionStiffness = s.suspensionStiffness
        suspensionDamping = s.suspensionDamping
        radius = s.radius
        width = s.width
        inertia = s.inertia
        angularDamping = s.angularDamping
        maxSteerAngle = s.maxSteerAngle
        maxBrakeTorque = s.maxBrakeTorque
        maxHandBrakeTorque = s.maxHandBrakeTorque
        trackLongitudinalFriction = s.trackLongitudinalFriction
        trackLateralFriction = s.trackLateralFriction
    }
}

/// Runtime state of one wheel: collision result, rotational state, and per-step solver data.
struct VehicleWheel: BitwiseCopyable {
    var settings: WheelParameters

    // Rotation
    var angularVelocity: Float = 0
    var rotationAngle: Float = 0
    var steerAngle: Float = 0

    // Collision
    var hasContact = false
    var contactBody: Int32 = B3.nullIndex
    var contactBodyId = BodyID()
    var contactShapeId = ShapeID()
    var contactPosition = Pos()
    var contactNormal = Vec3()
    var longitudinal = Vec3()
    var lateral = Vec3()
    var suspensionLength: Float = 0
    var surfaceFriction: Float = 0
    var userMaterialId: UInt64 = 0
    var longitudinalSlip: Float = 0
    var lateralSlip: Float = 0
    var longitudinalFriction: Float = 0
    var lateralFriction: Float = 0

    // Solver
    var groundIndex: Int32 = B3.nullIndex
    var groundInvMass: Float = 0
    var groundInvI = Matrix3()
    /// Chassis center of mass to the tire contact.
    var rA = Vec3()
    /// Chassis center of mass to the suspension force point.
    var rAs = Vec3()
    /// Ground center of mass to the tire contact.
    var rB = Vec3()
    var suspensionPosition: Float = 0
    var minLengthPosition: Float = 0
    var suspensionMass: Float = 0
    var longitudinalBodyK: Float = 0
    var lateralMass: Float = 0
    var suspensionSoftness = Softness()
    var suspensionImpulse: Float = 0
    var stopImpulse: Float = 0
    var longitudinalImpulse: Float = 0
    var lateralImpulse: Float = 0
    var brakeImpulse: Float = 0
    var maxBrakeImpulse: Float = 0
    /// Share of the clutch shaft speed, rebuilt every substep.
    var clutchCoefficient: Float = 0

    // Last step totals for inspection
    var totalSuspensionImpulse: Float = 0
    var totalLongitudinalImpulse: Float = 0
    var totalLateralImpulse: Float = 0

    init(_ settings: WheelSettings) {
        self.settings = WheelParameters(settings)
        suspensionLength = settings.suspensionMaxLength
    }

    var inverseInertia: Float { settings.inertia > 0 ? 1 / settings.inertia : 0 }
}

struct VehicleRecord {
    var id: Int32 = B3.nullIndex
    var generation: UInt16 = 0
    var userData: UnsafeMutableRawPointer?

    var bodyId = BodyID()
    var localUp = Vec3(0, 1, 0)
    var localForward = Vec3(0, 0, 1)
    var worldUp: Vec3?
    var maxPitchRollAngle: Float = B3.pi

    var wheels: [VehicleWheel] = []
    /// Full wheel settings including the tire curves.
    var wheelSettings: [WheelSettings] = []
    var antiRollBars: [VehicleAntiRollBar] = []
    var drivetrain: VehicleDrivetrain
    var tester = VehicleCollisionTester.ray
    var filter = QueryFilter.defaultFilter
    var cosMaxSlopeAngle: Float = 0
    var tireFriction: TireFrictionCallback?

    // Driver input
    var forwardInput: Float = 0
    var rightInput: Float = 0
    var brakeInput: Float = 0
    var handBrakeInput: Float = 0
    var leftTrackRatio: Float = 1
    var rightTrackRatio: Float = 1

    // Solver
    var active = false
    var chassisIndex: Int32 = B3.nullIndex
    var invMass: Float = 0
    var invI = Matrix3()
    var stepUp = Vec3(0, 1, 0)
    var pitchRollActive = false
    var pitchRollAxis = Vec3()
    var pitchRollMass: Float = 0
    var pitchRollChassisUp = Vec3()
    var pitchRollImpulse: Float = 0
    /// Throttle after automatic brake-on-reverse handling.
    var effectiveThrottle: Float = 0
    var effectiveBrake: Float = 0

    /// A free slot, also used as a placeholder while a record is moved out of the array.
    nonisolated(unsafe) static let empty = VehicleRecord(VehicleDef())

    init(_ def: VehicleDef) {
        bodyId = def.bodyId
        userData = def.userData
        localUp = normalize(def.up)
        localForward = normalize(def.forward)
        maxPitchRollAngle = def.maxPitchRollAngle
        wheelSettings = def.wheels.map(normalizedWheelSettings)
        wheels = wheelSettings.map { VehicleWheel($0) }
        antiRollBars = def.antiRollBars
        drivetrain = VehicleDrivetrain(def.controller, wheels: def.wheels)
        tester = def.collisionTester
        filter = def.filter
        cosMaxSlopeAngle = computeCosSin(clampFloat(def.maxSlopeAngle, 0, B3.pi)).cosine
    }
}

extension World {
    // MARK: Creation

    /// Create a vehicle constraint on a dynamic chassis. Returns a null id on invalid input.
    @discardableResult
    public func createVehicle(_ def: VehicleDef) -> VehicleID {
        b3Assert(locked == false)
        guard isValidBody(def.bodyId), body(def.bodyId).type == .dynamicBody else {
            return VehicleID()
        }
        guard def.wheels.isEmpty == false else { return VehicleID() }

        for wheel in def.wheels {
            guard wheel.radius > 0, wheel.suspensionMaxLength >= wheel.suspensionMinLength,
                wheel.suspensionMinLength >= 0, isValidVec3(wheel.position),
                lengthSquared(wheel.suspensionDirection) > 0, lengthSquared(wheel.wheelForward) > 0,
                lengthSquared(wheel.wheelUp) > 0
            else { return VehicleID() }
        }
        let wheelCount = Int32(def.wheels.count)
        for bar in def.antiRollBars {
            guard bar.leftWheel >= 0, bar.leftWheel < wheelCount, bar.rightWheel >= 0,
                bar.rightWheel < wheelCount, bar.leftWheel != bar.rightWheel
            else { return VehicleID() }
        }

        var record = VehicleRecord(def)

        let id = vehicleIdPool.alloc()
        if Int(id) == vehicles.count {
            vehicles.append(record)
        } else {
            record.generation = vehicles[Int(id)].generation &+ 1
            vehicles[Int(id)] = record
        }
        vehicles[Int(id)].id = id
        return VehicleID(index1: id + 1, world0: worldId, generation: vehicles[Int(id)].generation)
    }

    /// Destroy a vehicle. The chassis body is not affected.
    public func destroyVehicle(_ vehicleId: VehicleID) {
        guard vehicleIsValid(vehicleId) else { return }
        let index = Int(vehicleId.index1 - 1)
        let generation = vehicles[index].generation
        vehicles[index] = VehicleRecord.empty
        vehicles[index].generation = generation
        vehicleIdPool.free(Int32(index))
    }

    public func vehicleIsValid(_ vehicleId: VehicleID) -> Bool {
        guard vehicleId.world0 == worldId, vehicleId.index1 >= 1,
            Int(vehicleId.index1) <= vehicles.count
        else { return false }
        let record = vehicles[Int(vehicleId.index1 - 1)]
        return record.id != B3.nullIndex && record.generation == vehicleId.generation
    }

    /// Number of live vehicles.
    public var vehicleCount: Int32 { vehicleIdPool.idCount }

    @inline(__always)
    func vehicleIndex(_ vehicleId: VehicleID) -> Int? {
        vehicleIsValid(vehicleId) ? Int(vehicleId.index1 - 1) : nil
    }

    // MARK: Input

    /// Driver input for wheeled vehicles and motorcycles.
    /// - forward: -1 full reverse ... 1 full throttle. With the automatic gearbox, input against
    ///   the direction of travel brakes until the vehicle nearly stops.
    /// - right: -1 full left ... 1 full right.
    /// - brake, handBrake: 0 ... 1.
    public func setVehicleDriverInput(
        _ vehicleId: VehicleID, forward: Float, right: Float, brake: Float = 0, handBrake: Float = 0
    ) {
        guard let i = vehicleIndex(vehicleId) else { return }
        vehicles[i].forwardInput = clampFloat(forward, -1, 1)
        vehicles[i].rightInput = clampFloat(right, -1, 1)
        vehicles[i].brakeInput = clampFloat(brake, 0, 1)
        vehicles[i].handBrakeInput = clampFloat(handBrake, 0, 1)
        wakeVehicleIfDriven(i)
    }

    /// Driver input for tracked vehicles. The track ratios in [-1, 1] scale the drive to each track;
    /// opposite signs turn on the spot.
    public func setTrackedVehicleDriverInput(
        _ vehicleId: VehicleID, forward: Float, leftRatio: Float, rightRatio: Float, brake: Float = 0
    ) {
        guard let i = vehicleIndex(vehicleId) else { return }
        vehicles[i].forwardInput = clampFloat(forward, -1, 1)
        vehicles[i].leftTrackRatio = clampFloat(leftRatio, -1, 1)
        vehicles[i].rightTrackRatio = clampFloat(rightRatio, -1, 1)
        vehicles[i].brakeInput = clampFloat(brake, 0, 1)
        wakeVehicleIfDriven(i)
    }

    /// Manual gearbox control. Only used with `TransmissionMode.manual`.
    public func setVehicleTransmission(_ vehicleId: VehicleID, gear: Int32, clutchFriction: Float) {
        guard let i = vehicleIndex(vehicleId) else { return }
        vehicles[i].drivetrain.transmission.set(gear: gear, clutchFriction: clutchFriction)
    }

    private func wakeVehicleIfDriven(_ index: Int) {
        let record = vehicles[index]
        guard record.forwardInput != 0 || record.rightInput != 0, isValidBody(record.bodyId) else {
            return
        }
        let bodyIndex = self.bodyIndex(record.bodyId)
        if bodies[Int(bodyIndex)].setIndex >= SetType.firstSleepingSet.rawValue {
            wakeBody(bodyIndex)
        }
    }

    // MARK: Tuning

    /// The chassis body.
    public func vehicleBody(_ vehicleId: VehicleID) -> BodyID? {
        vehicleIndex(vehicleId).map { vehicles[$0].bodyId }
    }

    public func vehicleWheelCount(_ vehicleId: VehicleID) -> Int32 {
        vehicleIndex(vehicleId).map { Int32(vehicles[$0].wheels.count) } ?? 0
    }

    public func vehicleWheelSettings(_ vehicleId: VehicleID, _ wheel: Int32) -> WheelSettings? {
        guard let i = vehicleIndex(vehicleId), wheel >= 0, Int(wheel) < vehicles[i].wheels.count
        else { return nil }
        return vehicles[i].wheelSettings[Int(wheel)]
    }

    /// Replace a wheel's settings, for example to tune suspension or tires at run time.
    public func setVehicleWheelSettings(_ vehicleId: VehicleID, _ wheel: Int32, _ settings: WheelSettings) {
        guard let i = vehicleIndex(vehicleId), wheel >= 0, Int(wheel) < vehicles[i].wheels.count,
            settings.radius > 0, settings.suspensionMaxLength >= settings.suspensionMinLength
        else { return }
        let s = normalizedWheelSettings(settings)
        vehicles[i].wheelSettings[Int(wheel)] = s
        vehicles[i].wheels[Int(wheel)].settings = WheelParameters(s)
        if vehicles[i].drivetrain.kind == .motorcycle {
            vehicles[i].drivetrain.configureLeanGeometry(vehicles[i].wheelSettings)
        }
    }

    /// Replace the engine settings. The current engine speed is kept within the new range.
    public func setVehicleEngine(_ vehicleId: VehicleID, _ settings: VehicleEngineSettings) {
        guard let i = vehicleIndex(vehicleId) else { return }
        vehicles[i].drivetrain.engine.settings = settings
        vehicles[i].drivetrain.engine.clampSpeed()
    }

    /// Replace the gearbox settings. The current gear is kept when it still exists.
    public func setVehicleTransmissionSettings(_ vehicleId: VehicleID, _ settings: VehicleTransmissionSettings) {
        guard let i = vehicleIndex(vehicleId) else { return }
        let gear = vehicles[i].drivetrain.transmission.currentGear
        let clutch = vehicles[i].drivetrain.transmission.clutchFriction
        vehicles[i].drivetrain.transmission.settings = settings
        vehicles[i].drivetrain.transmission.set(gear: gear, clutchFriction: clutch)
    }

    public func setVehicleDifferentials(_ vehicleId: VehicleID, _ differentials: [VehicleDifferentialSettings]) {
        guard let i = vehicleIndex(vehicleId) else { return }
        let count = Int32(vehicles[i].wheels.count)
        vehicles[i].drivetrain.differentials = differentials.filter {
            $0.leftWheel < count && $0.rightWheel < count
        }
    }

    public func setVehicleAntiRollBars(_ vehicleId: VehicleID, _ bars: [VehicleAntiRollBar]) {
        guard let i = vehicleIndex(vehicleId) else { return }
        let count = Int32(vehicles[i].wheels.count)
        vehicles[i].antiRollBars = bars.filter {
            $0.leftWheel >= 0 && $0.leftWheel < count && $0.rightWheel >= 0 && $0.rightWheel < count
        }
    }

    /// World up used by the pitch/roll limit and motorcycle lean. Defaults to minus gravity.
    public func setVehicleWorldUp(_ vehicleId: VehicleID, _ up: Vec3?) {
        guard let i = vehicleIndex(vehicleId) else { return }
        vehicles[i].worldUp = up.map { normalize($0) }
    }

    public func setVehicleMaxPitchRollAngle(_ vehicleId: VehicleID, _ angle: Float) {
        guard let i = vehicleIndex(vehicleId) else { return }
        vehicles[i].maxPitchRollAngle = angle
    }

    public func setVehicleCollisionTester(_ vehicleId: VehicleID, _ tester: VehicleCollisionTester) {
        guard let i = vehicleIndex(vehicleId) else { return }
        vehicles[i].tester = tester
    }

    public func setVehicleTireFrictionCallback(_ vehicleId: VehicleID, _ callback: TireFrictionCallback?) {
        guard let i = vehicleIndex(vehicleId) else { return }
        vehicles[i].tireFriction = callback
    }

    // MARK: Inspection

    public func vehicleWheelState(_ vehicleId: VehicleID, _ wheel: Int32) -> VehicleWheelState? {
        guard let i = vehicleIndex(vehicleId), wheel >= 0, Int(wheel) < vehicles[i].wheels.count
        else { return nil }
        let w = vehicles[i].wheels[Int(wheel)]
        var state = VehicleWheelState()
        state.hasContact = w.hasContact
        state.contactBodyId = w.hasContact ? w.contactBodyId : BodyID()
        state.contactShapeId = w.hasContact ? w.contactShapeId : ShapeID()
        state.contactPosition = w.contactPosition
        state.contactNormal = w.contactNormal
        state.contactLongitudinal = w.longitudinal
        state.contactLateral = w.lateral
        state.suspensionLength = w.suspensionLength
        state.suspensionForce = w.totalSuspensionImpulse * inv_dt
        state.longitudinalForce = w.totalLongitudinalImpulse * inv_dt
        state.lateralForce = w.totalLateralImpulse * inv_dt
        state.longitudinalSlip = w.longitudinalSlip
        state.lateralSlip = w.lateralSlip
        state.angularVelocity = w.angularVelocity
        state.rotationAngle = w.rotationAngle
        state.steerAngle = w.steerAngle
        return state
    }

    public func vehicleDrivetrainState(_ vehicleId: VehicleID) -> VehicleDrivetrainState? {
        guard let i = vehicleIndex(vehicleId) else { return nil }
        let d = vehicles[i].drivetrain
        var state = VehicleDrivetrainState()
        state.engineRPM = d.engine.rpm
        state.currentGear = d.transmission.currentGear
        state.clutchFriction = d.transmission.clutchFriction
        state.isSwitchingGear = d.transmission.isSwitchingGear
        state.leanAngle = d.lean.currentLean
        if d.tracks.count >= 2 {
            state.trackAngularVelocity = Vec2(d.tracks[0].angularVelocity, d.tracks[1].angularVelocity)
        }
        return state
    }

    /// Chassis speed along the vehicle forward direction.
    public func vehicleForwardSpeed(_ vehicleId: VehicleID) -> Float {
        guard let i = vehicleIndex(vehicleId), isValidBody(vehicles[i].bodyId) else { return 0 }
        let record = vehicles[i]
        let forward = rotateVector(bodyRotation(record.bodyId), record.localForward)
        return dot(bodyLinearVelocity(record.bodyId), forward)
    }

    /// Wheel transform in the chassis frame. The returned rotation maps `wheelRight` to the axle and
    /// `wheelUp` to the wheel up, so a wheel model authored with those axes can be drawn with it.
    public func vehicleWheelLocalTransform(
        _ vehicleId: VehicleID, _ wheel: Int32, wheelRight: Vec3 = Vec3(1, 0, 0), wheelUp: Vec3 = Vec3(0, 1, 0)
    ) -> Transform? {
        guard let i = vehicleIndex(vehicleId), wheel >= 0, Int(wheel) < vehicles[i].wheels.count
        else { return nil }
        let w = vehicles[i].wheels[Int(wheel)]
        let s = w.settings

        // Model basis -> wheel basis (right = axle, up, forward).
        let axle = normalize(cross(s.wheelUp, s.wheelForward))
        let modelForward = normalize(cross(wheelRight, wheelUp))
        let model = Matrix3(normalize(wheelRight), normalize(wheelUp), modelForward)
        let wheelBasis = Matrix3(axle, s.wheelUp, s.wheelForward)
        let basis = makeQuatFromMatrix(mulMM(wheelBasis, transpose(model)))

        let steer = makeQuatFromAxisAngle(s.steeringAxis, w.steerAngle)
        let spin = makeQuatFromAxisAngle(axle, w.rotationAngle)
        let rotation = normalizeQuat(mulQuat(steer, mulQuat(spin, basis)))
        let position = mulAdd(s.position, w.suspensionLength, s.suspensionDirection)
        return Transform(position, rotation)
    }

    /// Wheel transform in world space. See `vehicleWheelLocalTransform`.
    public func vehicleWheelTransform(
        _ vehicleId: VehicleID, _ wheel: Int32, wheelRight: Vec3 = Vec3(1, 0, 0), wheelUp: Vec3 = Vec3(0, 1, 0)
    ) -> WorldTransform? {
        guard let local = vehicleWheelLocalTransform(vehicleId, wheel, wheelRight: wheelRight, wheelUp: wheelUp),
            let i = vehicleIndex(vehicleId), isValidBody(vehicles[i].bodyId)
        else { return nil }
        return mulWorldTransforms(bodyTransform(vehicles[i].bodyId), local)
    }

    public func vehicleUserData(_ vehicleId: VehicleID) -> UnsafeMutableRawPointer? {
        vehicleIndex(vehicleId).flatMap { vehicles[$0].userData }
    }
}

/// Unit length directions.
func normalizedWheelSettings(_ settings: WheelSettings) -> WheelSettings {
    var s = settings
    s.suspensionDirection = normalize(s.suspensionDirection)
    s.steeringAxis = normalize(s.steeringAxis)
    s.wheelUp = normalize(s.wheelUp)
    s.wheelForward = normalize(s.wheelForward)
    return s
}
