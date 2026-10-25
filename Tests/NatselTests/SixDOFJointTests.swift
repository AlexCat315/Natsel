import Testing
@testable import Natsel

private struct Rig {
    let world = World()
    let ground: BodyID
    let body: BodyID

    init(gravity: Vec3 = Vec3(0, -10, 0), offset: Vec3 = Vec3(1, 0, 0)) {
        world.gravity = gravity
        world.enableSleep = false
        ground = world.createBody(BodyDef())
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(offset)
        body = world.createBody(def)
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.2, 0.1, 0.1))
    }

    /// Joint at the origin, frame aligned with the world.
    func joint(_ configure: (inout SixDOFJointDef) -> Void) -> JointID {
        var def = SixDOFJointDef()
        def.base.bodyIdA = ground
        def.base.bodyIdB = body
        def.base.localFrameA = .identity
        def.base.localFrameB = Transform(subPos(.zero, world.bodyPosition(body)), .identity)
        configure(&def)
        return world.createSixDOFJoint(def)
    }

    func run(_ steps: Int) {
        for _ in 0..<steps { world.step(1.0 / 60.0, subStepCount: 4) }
    }
}

@Suite("Six DOF joint")
struct SixDOFJointTests {
    @Test("all axes locked behaves like a weld")
    func weld() {
        let rig = Rig()
        let joint = rig.joint { $0.lockAll() }
        #expect(rig.world.jointType(joint) == .sixDOFJoint)
        rig.run(120)
        #expect(distance(rig.world.bodyPosition(rig.body), Vec3(1, 0, 0)) < 0.01)
        let q = rig.world.bodyRotation(rig.body)
        #expect(abs(q.s) > 0.9995)
        #expect(rig.world.jointConstraintForce(joint).map { length($0) > 1 } == true)
    }

    @Test("slider with limits")
    func slider() {
        let rig = Rig(gravity: Vec3(10, -10, 0))
        let joint = rig.joint {
            $0.lockAll(except: [.translationX])
            $0[.translationX] = SixDOFAxisSettings(.limited, lower: -0.5, upper: 0.5)
        }
        rig.run(180)
        let pose = rig.world.sixDOFJointPose(joint)!
        #expect(abs(pose.p.x - 0.5) < 0.01)
        #expect(abs(pose.p.y) < 0.01 && abs(pose.p.z) < 0.01)
        #expect(abs(pose.q.s) > 0.9995)
        // Free it and it slides away.
        var free = rig.world.sixDOFJointAxis(joint, .translationX)!
        free.motion = .free
        rig.world.setSixDOFJointAxis(joint, .translationX, free)
        rig.run(30)
        #expect(rig.world.sixDOFJointPose(joint)!.p.x > 1)
    }

    @Test("hinge with a velocity motor")
    func hinge() {
        let rig = Rig()
        _ = rig.joint {
            $0.lockAll(except: [.rotationX])
            $0[.rotationX].motorMode = .velocity
            $0[.rotationX].motorTarget = 2
            $0[.rotationX].maxMotorForce = 500
        }
        rig.run(120)
        let w = rig.world.bodyAngularVelocity(rig.body)
        #expect(abs(w.x - 2) < 0.02)
        #expect(abs(w.y) < 0.02 && abs(w.z) < 0.02)
        #expect(distance(rig.world.bodyPosition(rig.body), Vec3(1, 0, 0)) < 0.01)
    }

    @Test("pyramid swing limit")
    func pyramid() {
        let rig = Rig()
        let limit: Float = 30 * B3.degToRad
        let joint = rig.joint {
            $0.lockAll(except: [.rotationZ])
            $0[.rotationZ] = SixDOFAxisSettings(.limited, lower: -limit, upper: limit)
        }
        rig.run(180)
        // The body hangs from the pivot toward -y and rests on the lower limit.
        let pose = rig.world.sixDOFJointPose(joint)!
        var angle: Float = 0
        let axis = getAxisAngle(&angle, pose.q)
        #expect(abs(abs(angle) - limit) < 0.02)
        #expect(abs(abs(axis.z) - 1) < 1.0e-3)
    }

    @Test("cone swing limit")
    func cone() {
        let rig = Rig()
        let angle: Float = 20 * B3.degToRad
        _ = rig.joint {
            $0.lockAll(except: [.rotationX, .rotationY, .rotationZ])
            $0[.rotationY].motion = .limited
            $0[.rotationZ].motion = .limited
            $0.swingLimit = .cone(angle: angle)
        }
        rig.world.setBodyAngularVelocity(rig.body, Vec3(0, 3, 4))
        var maxAngle: Float = 0
        for _ in 0..<180 {
            rig.world.step(1.0 / 60.0, subStepCount: 4)
            let x = rotateVector(rig.world.bodyRotation(rig.body), .axisX)
            maxAngle = max(maxAngle, atan2Approx(length(cross(.axisX, x)), x.x))
        }
        #expect(maxAngle < angle + 0.04)
        #expect(maxAngle > angle - 0.05)
    }

    @Test("position motors reach their targets")
    func positionMotors() {
        // Anchor at the center of mass so the translation and rotation springs do not couple.
        let rig = Rig(gravity: Vec3(), offset: Vec3())
        let target = makeQuatFromAxisAngle(Vec3(0, 0, 1), 0.25 * B3.pi)
        let joint = rig.joint {
            for axis in SixDOFAxis.allCases {
                $0[axis].motorMode = .position
                $0[axis].maxMotorForce = 1.0e5
                $0[axis].motorHertz = 4
                $0[axis].motorDampingRatio = 1
            }
            $0[.translationX].motorTarget = 0.3
            $0.targetRotation = target
        }
        rig.run(240)
        let pose = rig.world.sixDOFJointPose(joint)!
        #expect(distance(pose.p, Vec3(0.3, 0, 0)) < 0.01)
        #expect(abs(dotQuat(pose.q, target)) > 0.9995)

        rig.world.setSixDOFJointMotorTarget(joint, .translationX, -0.2)
        rig.world.setSixDOFJointTargetRotation(joint, .identity)
        rig.run(240)
        let back = rig.world.sixDOFJointPose(joint)!
        #expect(distance(back.p, Vec3(-0.2, 0, 0)) < 0.01)
        #expect(abs(back.q.s) > 0.9995)
    }

    @Test("friction holds a free axis against a weaker force")
    func friction() {
        let mass: Float = 0.4 * 0.2 * 0.2 * 1000
        for (friction, holds) in [(2 * mass * 10, true), (0.5 * mass * 10, false)] {
            let rig = Rig(gravity: Vec3(10, 0, 0))
            let joint = rig.joint {
                $0.lockAll(except: [.translationX])
                $0[.translationX].maxFriction = friction
            }
            rig.run(60)
            let x = rig.world.sixDOFJointPose(joint)!.p.x
            if holds {
                #expect(abs(x) < 1.0e-3)
            } else {
                #expect(x > 0.5)
            }
        }
    }

    @Test("destroying six DOF joints recycles their state")
    func lifecycle() {
        let rig = Rig()
        let first = rig.joint { $0.lockAll() }
        rig.run(2)
        rig.world.destroyJoint(first.index1 - 1, true)
        #expect(rig.world.sixDOFPool.idCount == 0)
        let second = rig.joint { $0.lockAll(except: [.rotationX]) }
        #expect(rig.world.sixDOFJointAxis(second, .rotationX)?.motion == .free)
        #expect(rig.world.sixDOFJointAxis(first, .rotationX) == nil)
        rig.world.destroyBody(rig.body)
        #expect(rig.world.sixDOFPool.idCount == 0)
    }
}
