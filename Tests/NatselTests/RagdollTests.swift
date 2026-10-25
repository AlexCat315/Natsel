import Testing
@testable import Natsel

/// A small humanoid: pelvis, spine, head, two arms and two legs of two parts each.
private func humanSkeleton() -> Skeleton {
    var skeleton = Skeleton()
    skeleton.addJoint("pelvis")
    skeleton.addJoint("spine", parentName: "pelvis")
    skeleton.addJoint("head", parentName: "spine")
    for side in ["L", "R"] {
        skeleton.addJoint("upperArm\(side)", parentName: "spine")
        skeleton.addJoint("lowerArm\(side)", parentName: "upperArm\(side)")
        skeleton.addJoint("thigh\(side)", parentName: "pelvis")
        skeleton.addJoint("calf\(side)", parentName: "thigh\(side)")
    }
    return skeleton
}

/// Standing bind pose, feet at y = 0 when the root offset is zero.
private func humanBindPose(_ skeleton: Skeleton, at origin: Vec3 = Vec3()) -> SkeletonPose {
    var pose = SkeletonPose(skeleton: skeleton)
    pose.rootOffset = Pos(origin)
    let down = makeQuatFromAxisAngle(.axisZ, -0.5 * Float.pi)
    let up = makeQuatFromAxisAngle(.axisZ, 0.5 * Float.pi)
    func set(_ name: String, _ p: Vec3, _ q: Quat) { pose.jointMatrices[skeleton.jointIndex(name)!] = Transform(p, q) }
    set("pelvis", Vec3(0, 1.0, 0), up)
    set("spine", Vec3(0, 1.25, 0), up)
    set("head", Vec3(0, 1.65, 0), up)
    for (side, x) in [("L", Float(-1)), ("R", Float(1))] {
        set("upperArm\(side)", Vec3(0.25 * x, 1.55, 0), down)
        set("lowerArm\(side)", Vec3(0.25 * x, 1.25, 0), down)
        set("thigh\(side)", Vec3(0.12 * x, 0.95, 0), down)
        set("calf\(side)", Vec3(0.12 * x, 0.5, 0), down)
    }
    pose.calculateJointStates()
    return pose
}

/// Parts along each joint's x axis (which points along the bone).
private func humanSettings(_ skeleton: Skeleton) -> RagdollSettings {
    func capsule(_ length: Float, _ radius: Float) -> RagdollShape {
        .capsule(Capsule(center1: Vec3(radius, 0, 0), center2: Vec3(length - radius, 0, 0), radius: radius))
    }
    var parts: [RagdollPart] = []
    for joint in skeleton.joints {
        var part = RagdollPart()
        part.jointFriction = 0.5
        switch joint.name {
        case "pelvis": part.shapes = [.box(halfExtents: Vec3(0.12, 0.15, 0.1), transform: Transform(Vec3(0.1, 0, 0), .identity))]
        case "spine":
            part.shapes = [capsule(0.38, 0.13)]
            part.joint = .swingTwist(coneAngle: 0.4, twistMin: -0.3, twistMax: 0.3)
        case "head":
            part.shapes = [.sphere(Sphere(center: Vec3(0.12, 0, 0), radius: 0.11))]
            part.joint = .swingTwist(coneAngle: 0.6, twistMin: -0.8, twistMax: 0.8)
        case let name where name.hasPrefix("upperArm"):
            part.shapes = [capsule(0.3, 0.05)]
            part.joint = .swingTwist(coneAngle: 1.2, twistMin: -0.5, twistMax: 0.5)
        case let name where name.hasPrefix("lowerArm"):
            part.shapes = [capsule(0.3, 0.045)]
            // Hinge about the joint's y axis, which is world x in the bind pose: the elbow bends forward.
            part.joint = .hinge(lower: -2, upper: 0)
            part.jointFrame = Transform(Vec3(), makeQuatFromAxisAngle(.axisZ, 0.5 * Float.pi))
        case let name where name.hasPrefix("thigh"):
            part.shapes = [capsule(0.45, 0.07)]
            part.joint = .pyramid(twist: -0.3...0.3, swingY: -0.4...0.4, swingZ: -1.2...0.4)
        default:
            part.shapes = [capsule(0.45, 0.055)]
            // The knee bends backward.
            part.joint = .hinge(lower: 0, upper: 2)
            part.jointFrame = Transform(Vec3(), makeQuatFromAxisAngle(.axisZ, 0.5 * Float.pi))
        }
        parts.append(part)
    }
    return RagdollSettings(skeleton: skeleton, parts: parts)
}

private func groundWorld(gravity: Vec3 = Vec3(0, -10, 0)) -> World {
    let world = World()
    world.gravity = gravity
    var def = BodyDef()
    def.position = Pos(0, -1, 0)
    _ = world.createHullShape(world.createBody(def), ShapeDef(), makeBoxHull(20, 1, 20))
    return world
}

private func rotationError(_ a: Quat, _ b: Quat) -> Float {
    getQuatAngle(invMulQuat(a, b).s < 0 ? negateQuat(invMulQuat(a, b)) : invMulQuat(a, b))
}

@Suite("Skeleton and ragdoll")
struct RagdollTests {
    @Test("skeleton structure and pose spaces")
    func skeleton() {
        let skeleton = humanSkeleton()
        #expect(skeleton.jointCount == 11)
        let calf = skeleton.jointIndex("calfL")!
        #expect(skeleton.isAncestor(skeleton.jointIndex("pelvis")!, of: calf))
        #expect(skeleton.isAncestor(skeleton.jointIndex("spine")!, of: calf) == false)

        let bind = humanBindPose(skeleton)
        var copy = SkeletonPose(skeleton: skeleton)
        copy.jointStates = bind.jointStates
        copy.calculateJointMatrices()
        for i in 0..<skeleton.jointCount {
            #expect(distance(copy.jointMatrices[i].p, bind.jointMatrices[i].p) < 1.0e-5)
            #expect(rotationError(copy.jointMatrices[i].q, bind.jointMatrices[i].q) < 1.0e-3)
        }
    }

    @Test("animation sampling, looping, and blending")
    func animation() {
        var skeleton = Skeleton()
        skeleton.addJoint("root")
        skeleton.addJoint("arm", parent: 0)
        let turn = makeQuatFromAxisAngle(.axisZ, 1)
        var animation = SkeletalAnimation(tracks: [
            .init(jointName: "arm", keyframes: [
                .init(time: 1, translation: Vec3(2, 0, 0), rotation: turn),
                .init(time: 0, translation: Vec3(0, 0, 0), rotation: .identity),
            ]),
            .init(jointName: "missing", keyframes: [.init(time: 0, translation: Vec3(), rotation: .identity)]),
        ])
        #expect(animation.duration == 1)
        var pose = SkeletonPose(skeleton: skeleton)
        animation.sample(0.5, &pose)
        #expect(distance(pose.jointStates[1].translation, Vec3(1, 0, 0)) < 1.0e-5)
        #expect(abs(getQuatAngle(pose.jointStates[1].rotation) - 0.5) < 2.0e-3)
        animation.sample(7, &pose)
        #expect(distance(pose.jointStates[1].translation, Vec3(2, 0, 0)) < 1.0e-5)

        animation.isLooping = true
        animation.sample(2.25, &pose)
        #expect(distance(pose.jointStates[1].translation, Vec3(0.5, 0, 0)) < 1.0e-5)

        // Half weight blends from the current state.
        animation.isLooping = false
        pose.jointStates[1] = SkeletonJointState(translation: Vec3(0, 4, 0), rotation: .identity)
        animation.sample(1, &pose, weight: 0.5)
        #expect(distance(pose.jointStates[1].translation, Vec3(1, 2, 0)) < 1.0e-5)
        pose.calculateJointMatrices()
        #expect(distance(pose.jointMatrices[1].p, Vec3(1, 2, 0)) < 1.0e-5)
    }

    @Test("mapping between a detailed and a ragdoll skeleton")
    func mapper() {
        // Low: pelvis -> chest -> head. High: pelvis -> spine1 -> spine2 -> chest -> head -> jaw.
        var low = Skeleton()
        low.addJoint("pelvis")
        low.addJoint("chest", parentName: "pelvis")
        low.addJoint("head", parentName: "chest")
        var high = Skeleton()
        high.addJoint("pelvis")
        high.addJoint("spine1", parentName: "pelvis")
        high.addJoint("spine2", parentName: "spine1")
        high.addJoint("chest", parentName: "spine2")
        high.addJoint("head", parentName: "chest")
        high.addJoint("jaw", parentName: "head")

        let lowNeutral = [Transform(Vec3(0, 1, 0), .identity), Transform(Vec3(0, 1.6, 0), .identity),
                          Transform(Vec3(0, 2, 0), .identity)]
        var highPose = SkeletonPose(skeleton: high)
        highPose.jointMatrices = [Transform(Vec3(0, 1, 0), .identity), Transform(Vec3(0, 1.2, 0), .identity),
                                  Transform(Vec3(0, 1.4, 0), .identity), Transform(Vec3(0, 1.6, 0), .identity),
                                  Transform(Vec3(0, 2, 0), .identity), Transform(Vec3(0, 1.95, 0.08), .identity)]
        highPose.calculateJointStates()
        let mapper = SkeletonMapper(low: low, lowNeutral: lowNeutral, high: high, highNeutral: highPose.jointMatrices)!
        #expect(mapper.mappings.count == 3)
        #expect(mapper.chains.count == 1 && mapper.chains[0].highIndices == [0, 1, 2, 3])
        #expect(mapper.unmapped == [5])

        // Bend the ragdoll at the pelvis: the chest swings to the side.
        var lowPose = SkeletonPose(skeleton: low)
        let bend = makeQuatFromAxisAngle(.axisZ, 0.6)
        lowPose.jointMatrices = [Transform(Vec3(0, 1, 0), bend),
                                 Transform(Vec3(0, 1, 0) + rotateVector(bend, Vec3(0, 0.6, 0)), bend),
                                 Transform(Vec3(0, 1, 0) + rotateVector(bend, Vec3(0, 1, 0)), bend)]
        lowPose.calculateJointStates()
        var mapped = SkeletonPose(skeleton: high)
        mapper.map(lowPose, animated: highPose, into: &mapped)
        #expect(distance(mapped.jointMatrices[3].p, lowPose.jointMatrices[1].p) < 1.0e-5)
        // The chain inner joints lie on the bent line between pelvis and chest.
        let direction = rotateVector(bend, .axisY)
        for (k, height) in [(1, Float(0.2)), (2, Float(0.4))] {
            #expect(distance(mapped.jointMatrices[k].p, Vec3(0, 1, 0) + height * direction) < 1.0e-4)
        }
        // The jaw keeps its animated offset from the head.
        let jawLocal = invMulTransforms(mapped.jointMatrices[4], mapped.jointMatrices[5])
        #expect(distance(jawLocal.p, Vec3(0, -0.05, 0.08)) < 1.0e-4)

        var back = SkeletonPose(skeleton: low)
        mapper.mapReverse(mapped, into: &back)
        for i in 0..<3 {
            #expect(distance(back.jointMatrices[i].p, lowPose.jointMatrices[i].p) < 1.0e-5)
        }
    }

    @Test("a ragdoll falls, settles, and keeps its joints together within limits")
    func fall() {
        let world = groundWorld()
        let skeleton = humanSkeleton()
        let settings = humanSettings(skeleton)
        let bind = humanBindPose(skeleton, at: Vec3(0, 0.5, 0))
        let ragdoll = Ragdoll(world: world, settings: settings, pose: bind)!
        #expect(ragdoll.bodies.count == 11)
        #expect(ragdoll.filterJoints.isEmpty == false)

        // The pose read back equals the creation pose.
        let read = ragdoll.pose()
        for i in 0..<skeleton.jointCount {
            #expect(distance(read.worldTransform(i).p, bind.worldTransform(i).p) < 1.0e-5)
        }

        ragdoll.applyLinearImpulse(Vec3(30, 0, 10))
        for _ in 0..<300 { world.step(1.0 / 60.0, subStepCount: 4) }
        let bounds = ragdoll.bounds
        #expect(bounds.lowerBound.y > -0.05)
        #expect(bounds.upperBound.y < 1.0)

        // Joints are connected: each child joint origin coincides with its anchor on the parent.
        let pose = ragdoll.pose()
        for (i, joint) in skeleton.joints.enumerated() where joint.parentIndex >= 0 {
            let sim = world.jointSim(ragdoll.joints[i])!
            let anchorA = transformWorldPoint(world.bodyTransform(ragdoll.bodies[joint.parentIndex]), sim.localFrameA.p)
            let anchorB = transformWorldPoint(world.bodyTransform(ragdoll.bodies[i]), sim.localFrameB.p)
            #expect(distance(anchorA, anchorB) < 0.02, "\(joint.name)")
            // Hinges stay hinges.
            if case .hinge(let lower, let upper) = settings.parts[i].joint {
                let rel = world.sixDOFJointPose(ragdoll.joints[i])!.q
                #expect(abs(rel.v.y) < 0.03 && abs(rel.v.z) < 0.03, "\(joint.name)")
                let angle = 2 * atan2Approx(rel.v.x, rel.s)
                #expect(angle > lower - 0.08 && angle < upper + 0.08, "\(joint.name) \(angle)")
            }
        }
        _ = pose
        #expect(ragdoll.isAwake == false)
    }

    @Test("motors drive the ragdoll to an animated pose")
    func motors() {
        let world = groundWorld(gravity: Vec3())
        let skeleton = humanSkeleton()
        let bind = humanBindPose(skeleton, at: Vec3(0, 2, 0))
        let ragdoll = Ragdoll(world: world, settings: humanSettings(skeleton), pose: bind)!

        // Target: elbow and knee bent inside their hinge ranges (about the joint's y axis), head turned.
        var target = bind
        let elbow = skeleton.jointIndex("lowerArmL")!
        let knee = skeleton.jointIndex("calfR")!
        let head = skeleton.jointIndex("head")!
        target.jointStates[elbow].rotation = mulQuat(target.jointStates[elbow].rotation, makeQuatFromAxisAngle(.axisY, -1))
        target.jointStates[knee].rotation = mulQuat(target.jointStates[knee].rotation, makeQuatFromAxisAngle(.axisY, 1))
        target.jointStates[head].rotation = mulQuat(target.jointStates[head].rotation, makeQuatFromAxisAngle(.axisX, 0.5))
        target.calculateJointMatrices()

        ragdoll.driveToPose(usingMotors: target, maxTorque: 200, hertz: 6)
        for _ in 0..<240 { world.step(1.0 / 60.0, subStepCount: 4) }
        let pose = ragdoll.pose()
        for joint in [elbow, knee, head] {
            #expect(rotationError(pose.jointStates[joint].rotation, target.jointStates[joint].rotation) < 0.05,
                    "\(skeleton.joints[joint].name) \(rotationError(pose.jointStates[joint].rotation, target.jointStates[joint].rotation)) \(rotationError(bind.jointStates[joint].rotation, target.jointStates[joint].rotation))")
        }

        ragdoll.disableMotors()
        let axis = world.sixDOFJointAxis(ragdoll.joints[elbow], .rotationX)!
        #expect(axis.motorMode == .off)
    }

    @Test("kinematic driving reaches the pose in one step")
    func kinematic() {
        let world = groundWorld(gravity: Vec3())
        let skeleton = humanSkeleton()
        let bind = humanBindPose(skeleton, at: Vec3(0, 2, 0))
        let ragdoll = Ragdoll(world: world, settings: humanSettings(skeleton), pose: bind)!
        var target = bind
        target.rootOffset = Pos(Float(0.3), Float(2.1), Float(-0.2))
        ragdoll.driveToPose(usingKinematics: target, timeStep: 1.0 / 60.0)
        world.step(1.0 / 60.0, subStepCount: 4)
        for i in 0..<skeleton.jointCount {
            #expect(distance(world.bodyPosition(ragdoll.bodies[i]), target.worldTransform(i).p) < 2.0e-3)
        }
    }

    @Test("mass stabilization and self collision options")
    func massAndCollision() {
        let skeleton = humanSkeleton()
        var settings = humanSettings(skeleton)
        let hand = skeleton.jointIndex("lowerArmL")!
        settings.parts[hand].shapeDef.density = 50_000
        settings.maxMassRatio = 0
        let loose = Ragdoll(world: groundWorld(), settings: settings, pose: humanBindPose(skeleton))!
        let upper = skeleton.jointIndex("upperArmL")!
        let rawRatio = loose.world.bodyMass(loose.bodies[hand]) / loose.world.bodyMass(loose.bodies[upper])
        #expect(rawRatio > 10)

        settings.maxMassRatio = 3
        let stable = Ragdoll(world: groundWorld(), settings: settings, pose: humanBindPose(skeleton))!
        for (i, joint) in skeleton.joints.enumerated() where joint.parentIndex >= 0 {
            let ratio = stable.world.bodyMass(stable.bodies[i]) / stable.world.bodyMass(stable.bodies[joint.parentIndex])
            #expect(ratio <= 3.001 && ratio >= 1 / 3.001, "\(joint.name)")
        }
        #expect(abs(stable.totalMass - loose.totalMass) < 1.0e-3 * loose.totalMass)

        settings.selfCollision = .none
        settings.collisionGroup = -7
        let isolated = Ragdoll(world: groundWorld(), settings: settings, pose: humanBindPose(skeleton))!
        #expect(isolated.filterJoints.isEmpty)
        let shape = isolated.world.bodies[Int(isolated.bodies[0].index1 - 1)].headShapeId
        #expect(isolated.world.shapes[Int(shape)].filter.groupIndex == -7)

        let world = stable.world
        let count = world.bodies.filter { $0.id != B3.nullIndex }.count
        stable.destroy()
        #expect(world.bodies.filter { $0.id != B3.nullIndex }.count == count - 11)
    }
}
