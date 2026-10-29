


@testable import Natsel

enum BoneId: Int, CaseIterable {
    case pelvis
    case spine01
    case spine03
    case neck
    case thighL
    case calfL
    case thighR
    case calfR
    case upperArmL
    case lowerArmL
    case upperArmR
    case lowerArmR
}

struct HumanBone {
    var bodyId = BodyID()
    var jointId = JointID()
    var parentIndex = -1
    var jointType = JointType.filterJoint
    var localFrameA = Transform()
    var localFrameB = Transform()
    var referenceFrame = Transform()
    var swingLimit: Float = 0
    var twistLimit = Vec2()
    var jointFriction: Float = 1
}

struct Human {
    var bones = Array(repeating: HumanBone(), count: BoneId.allCases.count)
    var filterJoints: [JointID] = []
    var frictionTorque: Float = 0
    var isSpawned = false
}

private func boneTransform(_ p: Vec3, _ q: Quat) -> Transform {
    Transform(p, q)
}

private func makeCapsule(_ c1: Vec3, _ c2: Vec3, _ radius: Float) -> Capsule {
    Capsule(center1: c1, center2: c2, radius: radius)
}

func createHuman(
    _ human: inout Human,
    _ world: World,
    _ position: Pos,
    _ frictionTorque: Float,
    _ hertz: Float,
    _ dampingRatio: Float,
    _ groupIndex: Int,
    _ userData: UnsafeMutableRawPointer?,
    _ colorize: Bool
) {
    precondition(human.isSpawned == false)

    for i in human.bones.indices {
        human.bones[i].bodyId = BodyID.null
        human.bones[i].jointId = JointID.null
        human.bones[i].jointFriction = 1
        human.bones[i].parentIndex = -1
    }
    human.filterJoints.removeAll()
    human.frictionTorque = frictionTorque

    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody
    bodyDef.userData = userData

    var shapeDef = ShapeDef()
    let defaultDensity = shapeDef.density

    func addBone(
        _ boneId: BoneId, _ parent: BoneId?, _ name: String, _ reference: Transform,
        _ group: Int, _ friction: Float, _ rolling: Float,
        capsules: [(Vec3, Vec3, Float)],
        jointType: JointType = .filterJoint,
        localFrameA: Transform = Transform(),
        localFrameB: Transform = Transform(),
        swingLimit: Float = 0, twistLimit: Vec2 = Vec2(),
        jointFriction: Float = 1,
        secondCapsuleDensity: Float? = nil
    ) {
        let index = boneId.rawValue
        human.bones[index].parentIndex = parent?.rawValue ?? -1
        bodyDef.name = name
        human.bones[index].referenceFrame = reference
        bodyDef.rotation = reference.q
        bodyDef.position = offsetPos(position, reference.p)
        human.bones[index].bodyId = world.createBody(bodyDef)

        shapeDef.filter.groupIndex = Int32(group)
        shapeDef.baseMaterial.friction = friction
        shapeDef.baseMaterial.rollingResistance = rolling

        for (capsuleIndex, capsule) in capsules.enumerated() {
            if capsuleIndex == 1, let density = secondCapsuleDensity {
                shapeDef.density = density
            }
            _ = world.createCapsuleShape(
                human.bones[index].bodyId, shapeDef,
                makeCapsule(capsule.0, capsule.1, capsule.2)
            )
        }
        shapeDef.density = defaultDensity

        human.bones[index].jointType = jointType
        human.bones[index].localFrameA = localFrameA
        human.bones[index].localFrameB = localFrameB
        human.bones[index].swingLimit = swingLimit
        human.bones[index].twistLimit = twistLimit
        human.bones[index].jointFriction = jointFriction
    }

    addBone(
        .pelvis, nil, "pelvis",
        boneTransform(Vec3(0.000000, 0.996219, -0.023868), Quat(Vec3(1, 0, 0), 0.000000)),
        0, 0.6, 0.1,
        capsules: [(Vec3(0.040000, 0.000001, 0.000000), Vec3(-0.040000, -0.000001, 0.000000), 0.150000)]
    )

    addBone(
        .spine01, .pelvis, "spine_01",
        boneTransform(Vec3(0.000000, 1.017288, -0.024882), Quat(Vec3(1, 0, 0), 0.000000)),
        -groupIndex, 0.5, 0.1,
        capsules: [(Vec3(0.029876, -0.146581, 0.006260), Vec3(-0.029876, -0.146574, 0.006260), 0.145663)],
        jointType: .sphericalJoint,
        localFrameA: boneTransform(
            Vec3(0.000000, -0.021069, 0.001014), Quat(Vec3(-0.642737, 0, 0), -0.766087)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.707107, 0, 0), -0.707107)
        ),
        swingLimit: 35 * B3.degToRad,
        twistLimit: Vec2(-17.5 * B3.degToRad, 17.5 * B3.degToRad)
    )

    addBone(
        .spine03, .spine01, "spine_03",
        boneTransform(Vec3(0.000000, 1.267766, -0.022320), Quat(Vec3(1, 0, 0), 0.000000)),
        0, 0.5, 0.1,
        capsules: [(Vec3(0.063996, -0.117434, -0.040199), Vec3(-0.063996, -0.117432, -0.040199), 0.165004)],
        jointType: .sphericalJoint,
        localFrameA: boneTransform(
            Vec3(0.000000, -0.250478, -0.002562), Quat(Vec3(-0.642766, 0, 0), -0.766063)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.707107, 0, 0), -0.707107)
        ),
        swingLimit: 35 * B3.degToRad,
        twistLimit: Vec2(-17.5 * B3.degToRad, 17.5 * B3.degToRad)
    )

    addBone(
        .neck, .spine03, "neck_01",
        boneTransform(Vec3(0.000000, 1.498783, 0.024462), Quat(Vec3(0.987879, 0, 0), 0.155228)),
        0, 0.2, 0.05,
        capsules: [(Vec3(0.000000, -0.212340, 0.000000), Vec3(0.000000, -0.087340, 0.000000), 0.110000)],
        jointType: .sphericalJoint,
        localFrameA: boneTransform(
            Vec3(0.000000, -0.231017, -0.046781), Quat(Vec3(-0.516157, -0.000002, 0.000003), -0.856494)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.707108, -0.000002, 0.000002), -0.707106)
        ),
        swingLimit: 30 * B3.degToRad,
        twistLimit: Vec2(-17.5 * B3.degToRad, 17.5 * B3.degToRad)
    )

    addBone(
        .thighL, .pelvis, "thigh_l",
        boneTransform(
            Vec3(0.092175, 0.971562, -0.011177),
            Quat(Vec3(0.471412, 0.529510, -0.499888), -0.497495)
        ),
        -groupIndex, 0.6, 0.1,
        capsules: [(Vec3(-0.047269, 0.000001, 0.000000), Vec3(-0.379769, 0.000005, 0.000000), 0.091539)],
        jointType: .sphericalJoint,
        localFrameA: boneTransform(
            Vec3(0.092175, 0.024656, -0.012691),
            Quat(Vec3(-0.460186, -0.365368, -0.637662), 0.498118)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.499997, -0.500003, 0.500002), 0.499998)
        ),
        swingLimit: 30 * B3.degToRad,
        twistLimit: Vec2(-10 * B3.degToRad, 10 * B3.degToRad)
    )

    addBone(
        .calfL, .thighL, "calf_l",
        boneTransform(
            Vec3(0.118720, 0.534541, -0.035362),
            Quat(Vec3(0.521188, 0.480597, -0.546380), -0.445935)
        ),
        0, 0.1, 0,
        capsules: [
            (Vec3(-0.445000, 0.000001, 0.000000), Vec3(0.005000, -0.000001, 0.000000), 0.080000),
            (Vec3(-0.456371, 0.129321, 0.000013), Vec3(-0.359977, 0.014394, 0.000013), 0.070000),
        ],
        jointType: .revoluteJoint,
        localFrameA: boneTransform(
            Vec3(-0.438494, -0.000175, 0.000000),
            Quat(Vec3(0.344866, -0.938652, 0.000002), -0.000008)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.008182, -0.999967, 0.000001), -0.000007)
        ),
        twistLimit: Vec2(-30 * B3.degToRad, 30 * B3.degToRad),
        secondCapsuleDensity: 0.5 * defaultDensity
    )

    addBone(
        .thighR, .pelvis, "thigh_r",
        boneTransform(
            Vec3(-0.092175, 0.971562, -0.011177),
            Quat(Vec3(-0.497495, 0.499888, 0.529510), -0.471412)
        ),
        -groupIndex, 0.6, 0.1,
        capsules: [(Vec3(0.047269, -0.000001, 0.000000), Vec3(0.379769, -0.000005, 0.000000), 0.092587)],
        jointType: .sphericalJoint,
        localFrameA: boneTransform(
            Vec3(-0.092175, 0.024656, -0.012691),
            Quat(Vec3(0.380666, 0.491846, 0.488644), -0.611889)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.500002, 0.499998, -0.499998), 0.500002)
        ),
        swingLimit: 30 * B3.degToRad,
        twistLimit: Vec2(-10 * B3.degToRad, 10 * B3.degToRad)
    )

    addBone(
        .calfR, .thighR, "calf_r",
        boneTransform(
            Vec3(-0.118720, 0.534541, -0.035362),
            Quat(Vec3(-0.445935, 0.546380, 0.480597), -0.521188)
        ),
        0, 0.1, 0,
        capsules: [
            (Vec3(0.445000, -0.000002, 0.000003), Vec3(-0.005000, 0.000000, 0.000003), 0.080000),
            (Vec3(0.369113, -0.006830, 0.000017), Vec3(0.465507, -0.121757, 0.000017), 0.070000),
        ],
        jointType: .revoluteJoint,
        localFrameA: boneTransform(
            Vec3(0.438494, 0.000175, 0.000000),
            Quat(Vec3(0.000007, 0.000005, -0.344860), -0.938654)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(0.000006, 0.000005, 0.008188), -0.999966)
        ),
        twistLimit: Vec2(-30 * B3.degToRad, 30 * B3.degToRad),
        secondCapsuleDensity: 0.5 * defaultDensity
    )

    addBone(
        .upperArmL, .spine03, "upperarm_l",
        boneTransform(
            Vec3(0.185817, 1.443630, 0.031306),
            Quat(Vec3(0.560220, -0.307179, 0.366233), -0.676512)
        ),
        0, 0.6, 0.1,
        capsules: [(Vec3(0.296038, 0.004386, -0.000794), Vec3(-0.016086, 0.004384, -0.016127), 0.065000)],
        jointType: .sphericalJoint,
        localFrameA: boneTransform(
            Vec3(0.185817, -0.175865, -0.053625),
            Quat(Vec3(-0.559466, -0.213784, -0.670199), 0.438323)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.500953, -0.502617, -0.509318), -0.486844)
        ),
        swingLimit: 45 * B3.degToRad,
        twistLimit: Vec2(-20 * B3.degToRad, 20 * B3.degToRad),
        jointFriction: 0.8
    )

    addBone(
        .lowerArmL, .upperArmL, "lowerarm_l",
        boneTransform(
            Vec3(0.347683, 1.193455, 0.029378),
            Quat(Vec3(0.406736, -0.493012, 0.089135), -0.763911)
        ),
        0, 0.3, 0.1,
        capsules: [(Vec3(0.312329, -0.008256, -0.016454), Vec3(0.033153, 0.008259, -0.002741), 0.060000)],
        jointType: .revoluteJoint,
        localFrameA: boneTransform(
            Vec3(0.297979, 0.000361, 0.000000),
            Quat(Vec3(0.916726, 0.399352, 0.000839), -0.011456)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(0.999922, -0.003857, 0.004659), -0.010908)
        ),
        twistLimit: Vec2(-35 * B3.degToRad, 35 * B3.degToRad),
        jointFriction: 0.06
    )

    addBone(
        .upperArmR, .spine03, "upperarm_r",
        boneTransform(
            Vec3(-0.185817, 1.443630, 0.031306),
            Quat(Vec3(0.676512, 0.366233, 0.307179), 0.560220)
        ),
        0, 0.6, 0.1,
        capsules: [(Vec3(-0.300237, -0.000001, -0.000319), Vec3(0.011887, 0.000001, 0.015013), 0.065000)],
        jointType: .sphericalJoint,
        localFrameA: boneTransform(
            Vec3(-0.185817, -0.175865, -0.053625),
            Quat(Vec3(0.213793, 0.559449, 0.438334), -0.670204)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.509319, 0.486843, 0.500954), -0.502617)
        ),
        swingLimit: 45 * B3.degToRad,
        twistLimit: Vec2(-20 * B3.degToRad, 20 * B3.degToRad),
        jointFriction: 0.8
    )

    addBone(
        .lowerArmR, .upperArmR, "lowerarm_r",
        boneTransform(
            Vec3(-0.347683, 1.193455, 0.029378),
            Quat(Vec3(0.763911, 0.089135, 0.493012), 0.406736)
        ),
        0, 0.3, 0.1,
        capsules: [(Vec3(-0.312649, -0.000003, 0.016471), Vec3(-0.032986, 0.000000, 0.002732), 0.060000)],
        jointType: .revoluteJoint,
        localFrameA: boneTransform(
            Vec3(-0.297979, -0.000361, 0.000000),
            Quat(Vec3(-0.405444, 0.914059, 0.010541), 0.000852)
        ),
        localFrameB: boneTransform(
            Vec3(0, 0, 0), Quat(Vec3(-0.002794, 0.999936, 0.010042), 0.004364)
        ),
        twistLimit: Vec2(-35 * B3.degToRad, 35 * B3.degToRad),
        jointFriction: 0.06
    )

    for i in 1..<BoneId.allCases.count {
        var bone = human.bones[i]
        let parent = human.bones[bone.parentIndex]

        bone.localFrameA.q = normalizeQuat(bone.localFrameA.q)
        bone.localFrameB.q = normalizeQuat(bone.localFrameB.q)

        if bone.jointType == .revoluteJoint {
            var def = RevoluteJointDef()
            def.base.bodyIdA = parent.bodyId
            def.base.bodyIdB = bone.bodyId
            def.base.localFrameA = bone.localFrameA
            def.base.localFrameB = bone.localFrameB
            def.enableLimit = true
            def.lowerAngle = bone.twistLimit.x
            def.upperAngle = bone.twistLimit.y
            def.enableSpring = hertz > 0
            def.hertz = hertz
            def.dampingRatio = dampingRatio
            def.enableMotor = true
            def.maxMotorTorque = bone.jointFriction * frictionTorque
            bone.jointId = world.createRevoluteJoint(def)
        } else if bone.jointType == .sphericalJoint {
            var def = SphericalJointDef()
            def.base.bodyIdA = parent.bodyId
            def.base.bodyIdB = bone.bodyId
            def.base.localFrameA = bone.localFrameA
            def.base.localFrameB = bone.localFrameB
            def.enableConeLimit = true
            def.coneAngle = bone.swingLimit
            def.enableTwistLimit = true
            def.lowerTwistAngle = bone.twistLimit.x
            def.upperTwistAngle = bone.twistLimit.y
            def.enableSpring = hertz > 0
            def.hertz = hertz
            def.dampingRatio = dampingRatio
            def.enableMotor = true
            def.maxMotorTorque = bone.jointFriction * frictionTorque
            bone.jointId = world.createSphericalJoint(def)
        }
        human.bones[i] = bone
    }

    // Disable some collisions.
    var filterDef = FilterJointDef()
    filterDef.base.bodyIdA = human.bones[BoneId.thighL.rawValue].bodyId
    filterDef.base.bodyIdB = human.bones[BoneId.thighR.rawValue].bodyId
    human.filterJoints.append(world.createFilterJoint(filterDef))
    filterDef.base.bodyIdA = human.bones[BoneId.neck.rawValue].bodyId
    filterDef.base.bodyIdB = human.bones[BoneId.upperArmL.rawValue].bodyId
    human.filterJoints.append(world.createFilterJoint(filterDef))
    filterDef.base.bodyIdA = human.bones[BoneId.neck.rawValue].bodyId
    filterDef.base.bodyIdB = human.bones[BoneId.upperArmR.rawValue].bodyId
    human.filterJoints.append(world.createFilterJoint(filterDef))

    human.isSpawned = true
}
