import Testing
@testable import Natsel

@Suite("Body")
struct BodyTests {
    @Test("body creation, forces, and mass match the C reference")
    func bodyReference() {
        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(1, 2, 3)
        let s1 = world.createBody(def)

        def = BodyDef()
        def.type = .kinematicBody
        def.position = Pos(0, 1, 0)
        let k1 = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 2, 3)
        def.rotation = makeQuatFromAxisAngle(Vec3(0, 0, 1), 0.5)
        def.linearVelocity = Vec3(0.5, -1.25, 2)
        def.angularVelocity = Vec3(0.1, 0.2, -0.3)
        def.linearDamping = 0.1
        def.angularDamping = 0.2
        def.gravityScale = 0.5
        let d1 = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(-2, 0, 0)
        def.isAwake = false
        let d2 = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(5, 5, 5)
        def.isEnabled = false
        let d3 = world.createBody(def)

        #expect(world.solverSets.count == 4)
        #expect(world.islands.count == 3)

        #expect(world.solverSets[0].bodySims.count == 1)
        #expect(world.solverSets[0].bodyStates.count == 0)
        #expect(world.solverSets[0].islandSims.count == 0)
        #expect(world.solverSets[1].bodySims.count == 1)
        #expect(world.solverSets[2].bodySims.count == 2)
        #expect(world.solverSets[2].bodyStates.count == 2)
        #expect(world.solverSets[2].islandSims.count == 2)
        #expect(world.solverSets[3].bodySims.count == 1)
        #expect(world.solverSets[3].islandSims.count == 1)

        // Static
        var body = world.body(s1)
        #expect(body.setIndex == 0)
        #expect(body.localIndex == 0)
        #expect(body.generation == 1)
        #expect(body.id == 0)
        #expect(body.islandId == B3.nullIndex)
        #expect(body.flags == 0x0000_6000)
        var sim = world.bodySimFor(s1)
        #expect(sim.transform.p == Pos(1, 2, 3))
        #expect(sim.transform.q.s.bitPattern == 0x3f80_0000)
        #expect(sim.center.x == 1)
        #expect(sim.invMass == 0)
        #expect(sim.minExtent == 100_000)
        #expect(sim.maxExtent.x == 0)
        #expect(sim.flags == 0x0000_6000)

        // Kinematic
        body = world.body(k1)
        #expect(body.setIndex == 2)
        #expect(body.localIndex == 0)
        #expect(body.islandId == 0)
        #expect(body.islandIndex == 0)
        #expect(body.flags == 0x0000_6000)

        // Dynamic
        body = world.body(d1)
        #expect(body.setIndex == 2)
        #expect(body.localIndex == 1)
        #expect(body.islandId == 1)
        #expect(body.flags == 0x0000_7000)
        sim = world.bodySimFor(d1)
        #expect(sim.transform.q.s.bitPattern == 0x3f77_f06b)
        #expect(world.bodyLinearVelocity(d1).x == 0.5)
        #expect(world.bodyAngularVelocity(d1).z.bitPattern == 0xbe99_999a)
        #expect(world.bodyState(body)!.deltaRotation.s.bitPattern == 0x3f80_0000)

        // Sleeping
        body = world.body(d2)
        #expect(body.setIndex == 3)
        #expect(body.localIndex == 0)
        #expect(body.islandId == 2)

        // Disabled
        body = world.body(d3)
        #expect(body.setIndex == 1)
        #expect(body.islandId == B3.nullIndex)

        // Forces
        world.applyBodyForce(d1, Vec3(10, 20, 30), Pos(Float(1.5), Float(2.5), Float(3.5)), false)
        world.applyBodyTorque(d1, Vec3(1, -2, 3), false)
        sim = world.bodySimFor(d1)
        #expect(sim.force.x.bitPattern == 0x4120_0000)
        #expect(sim.force.y.bitPattern == 0x41a0_0000)
        #expect(sim.torque.z.bitPattern == 0x4100_0000)

        // Impulse
        world.applyBodyLinearImpulse(d1, Vec3(1, 2, 3), Pos(Float(1.5), Float(2.5), Float(3.5)), false)
        #expect(world.bodyLinearVelocity(d1).x.bitPattern == 0x3f00_0000)
        #expect(world.bodyLinearVelocity(d1).y.bitPattern == 0xbfa0_0000)
        #expect(world.bodyAngularVelocity(d1).x.bitPattern == 0x3dcc_cccd)

        // Transform
        world.setBodyTransform(d1, Pos(4, 5, 6), Quat.identity)
        sim = world.bodySimFor(d1)
        #expect(sim.transform.p == Pos(4, 5, 6))
        #expect(sim.transform.q.s.bitPattern == 0x3f80_0000)
        #expect(sim.center == Pos(4, 5, 6))

        // Mass data
        var massData = MassData()
        massData.mass = 5
        massData.center = Vec3(0.1, 0.2, 0.3)
        massData.inertia = Matrix3(Vec3(1, 0, 0), Vec3(0, 2, 0), Vec3(0, 0, 3))
        world.setBodyMassData(d1, massData)

        let got = world.bodyMassData(d1)
        #expect(got.mass.bitPattern == 0x40a0_0000)
        #expect(got.center.x.bitPattern == 0x3dcc_cccd)
        #expect(got.inertia.cx.x.bitPattern == 0x3f80_0000)
        #expect(got.inertia.cy.y.bitPattern == 0x4000_0000)

        let center = world.bodyWorldCenter(d1)
        #expect(Float(center.x).bitPattern == 0x4083_3333)
        #expect(Float(center.y).bitPattern == 0x40a6_6666)
        #expect(Float(center.z).bitPattern == 0x40c9_999a)

        let inverse = world.bodyWorldInverseRotationalInertia(d1)
        #expect(inverse.cx.x.bitPattern == 0x3f80_0000)
        #expect(inverse.cy.y.bitPattern == 0x3f00_0000)

        // Destroy
        world.destroyBody(k1)
        #expect(world.solverSets.count == 4)
        #expect(world.bodies.count == 5)
        #expect(!world.isValidBody(k1))
        #expect(world.isValidBody(d1))
    }
}
