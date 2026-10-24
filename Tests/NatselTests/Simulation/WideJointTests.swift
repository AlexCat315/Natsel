import Testing

@testable import Natsel

@Suite("Wide joint equivalence")
struct WideJointTests {
    @Test("point joints, angular fallbacks and disabled cached impulses agree", arguments: [false, true])
    func jointGrid(_ mixed: Bool) throws {
        func makeWorld(_ wide: Bool) -> (World, [JointID]) {
            let world = World()
            world.enableWideSolver = wide
            world.enableSleep = false
            let width = 10
            let height = 8
            var bodies: [BodyID] = []
            var joints: [JointID] = []
            var shape = ShapeDef()
            shape.filter.categoryBits = 2
            shape.filter.maskBits = ~2
            for x in 0..<width {
                for y in 0..<height {
                    var body = BodyDef()
                    body.type = y == 0 ? .staticBody : .dynamicBody
                    body.position = Pos(Float(x), -Float(y), 0)
                    body.angularVelocity = Vec3(0.03 * Float(x), 0.01 * Float(y), -0.02)
                    let id = world.createBody(body)
                    _ = world.createSphereShape(id, shape, Sphere(center: Vec3(), radius: 0.4))
                    var joint = SphericalJointDef()
                    joint.base.bodyIdB = id
                    joint.enableSpring = mixed && (x + y) % 11 == 0
                    joint.hertz = 2
                    joint.dampingRatio = 0.2
                    joint.enableMotor = mixed && (x + 2 * y) % 17 == 0
                    joint.maxMotorTorque = 0.4
                    joint.motorVelocity = Vec3(0.2, 0, 0.1)
                    if y > 0 {
                        joint.base.bodyIdA = bodies.last!
                        joint.base.localFrameA.p = Vec3(0, -0.5, 0)
                        joint.base.localFrameB.p = Vec3(0, 0.5, 0)
                        joints.append(world.createSphericalJoint(joint))
                    }
                    if x > 0 {
                        joint.base.bodyIdA = bodies[bodies.count - height]
                        joint.base.localFrameA.p = Vec3(0.5, 0, 0)
                        joint.base.localFrameB.p = Vec3(-0.5, 0, 0)
                        joints.append(world.createSphericalJoint(joint))
                    }
                    bodies.append(id)
                }
            }
            return (world, joints)
        }
        let (scalar, scalarIds) = makeWorld(false)
        let (wide, wideIds) = makeWorld(true)
        var sawWide = false
        var sawPartial = false
        for step in 0..<120 {
            if step == 40 {
                for (idA, idB) in zip(scalarIds, wideIds) {
                    _ = scalar.enableSphericalJointSpring(idA, false)
                    _ = wide.enableSphericalJointSpring(idB, false)
                    _ = scalar.enableSphericalJointMotor(idA, false)
                    _ = wide.enableSphericalJointMotor(idB, false)
                }
            }
            scalar.enableWarmStarting = step % 23 != 0
            wide.enableWarmStarting = scalar.enableWarmStarting
            scalar.step(1 / 60, subStepCount: 4)
            wide.step(1 / 60, subStepCount: 4)
            try #require(scalar.worldStateHash() == wide.worldStateHash(), "step \(step), mixed \(mixed)")
            for (idA, idB) in zip(scalarIds, wideIds) {
                let a = scalar.readJointSim(idA.index1 - 1).spherical.linearImpulse
                let b = wide.readJointSim(idB.index1 - 1).spherical.linearImpulse
                try #require(
                    a.x.bitPattern == b.x.bitPattern && a.y.bitPattern == b.y.bitPattern
                        && a.z.bitPattern == b.z.bitPattern,
                    "joint impulse step \(step)")
            }
            for list in wide.solverJoints.batches {
                sawWide = sawWide || !list.isEmpty
                sawPartial = sawPartial || list.contains { $0.laneCount < 4 }
            }
        }
        #expect(sawWide)
        #expect(sawPartial)
    }
}
