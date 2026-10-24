// Engine, gearbox, differentials, and tracks. The drivetrain owns the rotational degrees of freedom
// of the engine, the wheels, and the tracks. Ground contact couples them to the chassis in the
// vehicle solver; here they are coupled to each other.

let rpmToAngularVelocity: Float = B3.twoPi / 60
let angularVelocityToRPM: Float = 60 / B3.twoPi

// MARK: - Engine

struct VehicleEngine {
    var settings: VehicleEngineSettings

    /// Crankshaft angular velocity (rad/s).
    var angularVelocity: Float

    init(_ settings: VehicleEngineSettings) {
        self.settings = settings
        angularVelocity = settings.minRPM * rpmToAngularVelocity
    }

    var rpm: Float { angularVelocity * angularVelocityToRPM }

    /// Torque available at the current speed for a throttle in [0, 1].
    func torque(_ throttle: Float) -> Float {
        let span = settings.maxRPM - settings.minRPM
        let normalized = span > 0 ? clampFloat((rpm - settings.minRPM) / span, 0, 1) : 0
        return throttle * settings.maxTorque * settings.normalizedTorque.evaluate(normalized)
    }

    var inverseInertia: Float { settings.inertia > 0 ? 1 / settings.inertia : 0 }

    /// Keep the engine between idle and red line.
    mutating func clampSpeed() {
        let lower = settings.minRPM * rpmToAngularVelocity
        let upper = settings.maxRPM * rpmToAngularVelocity
        angularVelocity = clampFloat(angularVelocity, lower, upper)
    }
}

// MARK: - Transmission

struct VehicleTransmission {
    var settings: VehicleTransmissionSettings

    /// 0 neutral, positive forward gears, negative reverse gears.
    var currentGear: Int32 = 0
    var clutchFriction: Float = 1
    var gearSwitchTimeLeft: Float = 0
    var clutchReleaseTimeLeft: Float = 0
    var gearSwitchLatencyTimeLeft: Float = 0

    init(_ settings: VehicleTransmissionSettings) {
        self.settings = settings
    }

    var isSwitchingGear: Bool { gearSwitchTimeLeft > 0 }

    /// Gear ratio between engine and transmission output. Zero in neutral.
    var currentRatio: Float {
        if currentGear > 0 {
            let index = Int(currentGear) - 1
            return index < settings.gearRatios.count ? settings.gearRatios[index] : 0
        }
        if currentGear < 0 {
            let index = Int(-currentGear) - 1
            return index < settings.reverseGearRatios.count ? settings.reverseGearRatios[index] : 0
        }
        return 0
    }

    /// Manual control.
    mutating func set(gear: Int32, clutchFriction: Float) {
        let forwardCount = Int32(settings.gearRatios.count)
        let reverseCount = Int32(settings.reverseGearRatios.count)
        currentGear = clampInt(gear, -reverseCount, forwardCount)
        self.clutchFriction = clampFloat(clutchFriction, 0, 1)
    }

    /// Automatic gear selection and clutch release.
    mutating func update(_ dt: Float, engineRPM: Float, forwardInput: Float, canShiftUp: Bool) {
        guard settings.mode == .automatic else { return }

        let oldGear = currentGear
        let forwardCount = Int32(settings.gearRatios.count)
        let reverseCount = Int32(settings.reverseGearRatios.count)

        if currentGear == 0 || forwardInput * Float(currentGear) < 0 {
            // From neutral, or reversing direction: pick first or reverse from the input.
            if forwardInput > 0 {
                currentGear = forwardCount > 0 ? 1 : 0
            } else if forwardInput < 0 {
                currentGear = reverseCount > 0 ? -1 : 0
            } else {
                currentGear = 0
            }
        } else if gearSwitchLatencyTimeLeft == 0 {
            if canShiftUp && engineRPM > settings.shiftUpRPM {
                if currentGear < 0 {
                    if currentGear > -reverseCount { currentGear -= 1 }
                } else if currentGear < forwardCount {
                    currentGear += 1
                }
            } else if engineRPM < settings.shiftDownRPM {
                if currentGear < 0 {
                    // Without throttle the gearbox goes to neutral so the engine can idle.
                    let highest: Int32 = forwardInput != 0 ? -1 : 0
                    if currentGear < highest { currentGear += 1 }
                } else {
                    let lowest: Int32 = forwardInput != 0 ? 1 : 0
                    if currentGear > lowest { currentGear -= 1 }
                }
            }
        }

        if oldGear != currentGear {
            // Leaving neutral is instant, everything else takes the switch time.
            gearSwitchTimeLeft = oldGear != 0 ? settings.switchTime : 0
            clutchReleaseTimeLeft = settings.clutchReleaseTime
            gearSwitchLatencyTimeLeft = settings.switchLatency
            clutchFriction = 0
        } else if gearSwitchTimeLeft > 0 {
            gearSwitchTimeLeft = maxFloat(0, gearSwitchTimeLeft - dt)
            clutchFriction = 0
        } else if clutchReleaseTimeLeft > 0 {
            clutchReleaseTimeLeft = maxFloat(0, clutchReleaseTimeLeft - dt)
            let release = settings.clutchReleaseTime
            clutchFriction = release > 0 ? 1 - clutchReleaseTimeLeft / release : 1
        } else {
            clutchFriction = 1
            gearSwitchLatencyTimeLeft = maxFloat(0, gearSwitchLatencyTimeLeft - dt)
        }
    }
}

// MARK: - Torque bias

/// Torque fraction for the slower and faster output of a limited slip coupling. Below the ratio
/// limit the base split is kept; at the limit all torque goes to the slower output.
@inline(__always)
func limitedSlipFractions(
    _ baseLeft: Float, _ leftSpeed: Float, _ rightSpeed: Float, _ limitedSlipRatio: Float
) -> (left: Float, right: Float) {
    let baseRight = 1 - baseLeft
    guard limitedSlipRatio < .greatestFiniteMagnitude, limitedSlipRatio > 1 else {
        return (baseLeft, baseRight)
    }

    let omegaL = maxFloat(1.0e-3, absFloat(leftSpeed))
    let omegaR = maxFloat(1.0e-3, absFloat(rightSpeed))
    if omegaL > omegaR {
        let t = clampFloat((omegaL / omegaR - 1) / (limitedSlipRatio - 1), 0, 1)
        let left = baseLeft * (1 - t)
        return (left, 1 - left)
    } else if omegaR > omegaL {
        let t = clampFloat((omegaR / omegaL - 1) / (limitedSlipRatio - 1), 0, 1)
        let right = baseRight * (1 - t)
        return (1 - right, right)
    }
    return (baseLeft, baseRight)
}

// MARK: - Drivetrain

enum VehicleControllerKind: BitwiseCopyable {
    case wheeled
    case motorcycle
    case tracked
}

/// Scalar track settings; the wheel list lives in `VehicleDrivetrain.trackWheels`.
struct TrackParameters: BitwiseCopyable {
    var drivenWheel: Int32
    var inertia: Float
    var angularDamping: Float
    var maxBrakeTorque: Float
    var differentialRatio: Float

    init(_ s: VehicleTrackSettings) {
        drivenWheel = s.drivenWheel
        inertia = s.inertia
        angularDamping = s.angularDamping
        maxBrakeTorque = s.maxBrakeTorque
        differentialRatio = s.differentialRatio
    }
}

/// Track runtime state.
struct VehicleTrack: BitwiseCopyable {
    var settings: TrackParameters
    var angularVelocity: Float = 0
    var brakeImpulse: Float = 0
    var maxBrakeImpulse: Float = 0
    /// Radius of the driven wheel; track speed = angularVelocity * drivenRadius.
    var drivenRadius: Float = 1

    var inverseInertia: Float { settings.inertia > 0 ? 1 / settings.inertia : 0 }
}

/// Motorcycle lean runtime state.
struct VehicleLean {
    var settings = MotorcycleControllerSettings()
    var targetLean: Float = 0
    var currentLean: Float = 0
    var integral: Float = 0
    var wheelBase: Float = 0
    var frontWheel: Int32 = 0

    // Solver data
    var active = false
    var axis = Vec3()
    var mass: Float = 0
    var position: Float = 0
    var softness = Softness()
    var impulse: Float = 0
}

struct VehicleDrivetrain {
    var kind = VehicleControllerKind.wheeled
    var engine: VehicleEngine
    var transmission: VehicleTransmission
    var differentials: [VehicleDifferentialSettings] = []
    var differentialLimitedSlipRatio: Float = 1.4
    var tracks: [VehicleTrack] = []
    var trackWheels: [[Int32]] = []
    var lean = VehicleLean()

    /// The automatic gearbox opened the clutch for this step.
    var clutchOpen = false

    /// Wheel index -> track index for tracked vehicles, -1 otherwise.
    var wheelTrack: [Int32] = []

    init(_ settings: VehicleControllerSettings, wheels: [WheelSettings]) {
        switch settings {
        case .wheeled(let s):
            kind = .wheeled
            engine = VehicleEngine(s.engine)
            transmission = VehicleTransmission(s.transmission)
            differentials = s.differentials
            differentialLimitedSlipRatio = s.differentialLimitedSlipRatio
        case .motorcycle(let s):
            kind = .motorcycle
            engine = VehicleEngine(s.wheeled.engine)
            transmission = VehicleTransmission(s.wheeled.transmission)
            differentials = s.wheeled.differentials
            differentialLimitedSlipRatio = s.wheeled.differentialLimitedSlipRatio
            lean.settings = s
        case .tracked(let s):
            kind = .tracked
            engine = VehicleEngine(s.engine)
            transmission = VehicleTransmission(s.transmission)
            tracks = s.tracks.map { VehicleTrack(settings: TrackParameters($0)) }
            trackWheels = s.tracks.map(\.wheels)
        }

        wheelTrack = [Int32](repeating: -1, count: wheels.count)
        for t in tracks.indices {
            for w in trackWheels[t] where w >= 0 && Int(w) < wheels.count {
                wheelTrack[Int(w)] = Int32(t)
            }
            let driven = Int(tracks[t].settings.drivenWheel)
            if driven >= 0 && driven < wheels.count {
                tracks[t].drivenRadius = maxFloat(wheels[driven].radius, B3.linearSlop)
            }
        }

        if kind == .motorcycle {
            configureLeanGeometry(wheels)
        }
    }

    /// The steered wheel is the front; the wheel base is the forward distance to the farthest
    /// non-steered wheel.
    mutating func configureLeanGeometry(_ wheels: [WheelSettings]) {
        var front: Int32 = 0
        var bestSteer: Float = -1
        for (i, w) in wheels.enumerated() where w.maxSteerAngle > bestSteer {
            bestSteer = w.maxSteerAngle
            front = Int32(i)
        }
        var wheelBase: Float = 0
        if Int(front) < wheels.count {
            let f = wheels[Int(front)]
            let forward = normalize(f.wheelForward)
            for (i, w) in wheels.enumerated() where Int32(i) != front {
                wheelBase = maxFloat(wheelBase, absFloat(dot(f.position - w.position, forward)))
            }
        }
        lean.frontWheel = front
        lean.wheelBase = wheelBase
    }

    var isDriven: Bool { kind != .tracked ? !differentials.isEmpty : !tracks.isEmpty }
}
