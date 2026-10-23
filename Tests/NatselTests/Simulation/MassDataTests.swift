import Testing
@testable import Natsel

// Explicit mass property edits, ported from test/test_body.c.
@Suite("Mass data")
struct MassDataTests {
    /// diag(2, 4, 8)
    private var diagInertia: Matrix3 {
        var m = Matrix3()
        m.cx = Vec3(2, 0, 0)
        m.cy = Vec3(0, 4, 0)
        m.cz = Vec3(0, 0, 8)
        return m
    }

    @Test("mass data round trips through every accessor")
    func roundTrip() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(5, -3, 2)
        let body = world.createBody(def)

        let center = Vec3(0.1, 0.2, 0.3)
        var massData = MassData()
        massData.mass = 3
        massData.center = center
        massData.inertia = diagInertia
        world.setBodyMassData(body, massData)

        #expect(absFloat(world.bodyMass(body) - 3) < 1e-6)
        #expect(absFloat(world.bodyInverseMass(body) - 1.0 / 3.0) < 1e-6)

        let md = world.bodyMassData(body)
        #expect(absFloat(md.mass - 3) < 1e-6)
        #expect(absFloat(md.center.x - center.x) < 1e-6)
        #expect(absFloat(md.center.y - center.y) < 1e-6)
        #expect(absFloat(md.center.z - center.z) < 1e-6)
        #expect(absFloat(md.inertia.cx.x - 2) < 1e-6)
        #expect(absFloat(md.inertia.cy.y - 4) < 1e-6)
        #expect(absFloat(md.inertia.cz.z - 8) < 1e-6)

        let localCenter = world.bodyLocalCenter(body)
        #expect(absFloat(localCenter.x - center.x) < 1e-6)
        #expect(absFloat(localCenter.y - center.y) < 1e-6)
        #expect(absFloat(localCenter.z - center.z) < 1e-6)

        let localInertia = world.bodyLocalRotationalInertia(body)
        #expect(absFloat(localInertia.cx.x - 2) < 1e-6)
        #expect(absFloat(localInertia.cy.y - 4) < 1e-6)
        #expect(absFloat(localInertia.cz.z - 8) < 1e-6)

        // Identity rotation, so the world inverse inertia is just the local inverse.
        let invWorld = world.bodyWorldInverseRotationalInertia(body)
        #expect(absFloat(invWorld.cx.x - 0.5) < 1e-5)
        #expect(absFloat(invWorld.cy.y - 0.25) < 1e-5)
        #expect(absFloat(invWorld.cz.z - 0.125) < 1e-5)
        #expect(absFloat(invWorld.cy.x) < 1e-5)
        #expect(absFloat(invWorld.cz.x) < 1e-5)
        #expect(absFloat(invWorld.cx.y) < 1e-5)
        #expect(absFloat(invWorld.cz.y) < 1e-5)
        #expect(absFloat(invWorld.cx.z) < 1e-5)
        #expect(absFloat(invWorld.cy.z) < 1e-5)

        // World center of mass is the body origin plus the local center under identity rotation.
        let worldCenter = world.bodyWorldCenter(body)
        #expect(absFloat(worldCenter.x - (5 + center.x)) < 1e-5)
        #expect(absFloat(worldCenter.y - (-3 + center.y)) < 1e-5)
        #expect(absFloat(worldCenter.z - (2 + center.z)) < 1e-5)
    }

    /// A 90 degree turn about z swaps the x and y principal moments, so diag(1/2, 1/4, 1/8) becomes
    /// diag(1/4, 1/2, 1/8).
    @Test("world inverse inertia is rotated into world space")
    func worldInertiaRotated() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        def.rotation = makeQuatFromAxisAngle(.axisZ, 0.5 * B3.pi)
        let body = world.createBody(def)

        var massData = MassData()
        massData.mass = 1
        massData.center = Vec3()
        massData.inertia = diagInertia
        world.setBodyMassData(body, massData)

        // The local inertia is stored untouched by the world transform.
        let localInertia = world.bodyLocalRotationalInertia(body)
        #expect(absFloat(localInertia.cx.x - 2) < 1e-6)
        #expect(absFloat(localInertia.cy.y - 4) < 1e-6)
        #expect(absFloat(localInertia.cz.z - 8) < 1e-6)

        let invWorld = world.bodyWorldInverseRotationalInertia(body)
        #expect(absFloat(invWorld.cx.x - 0.25) < 1e-4)
        #expect(absFloat(invWorld.cy.y - 0.5) < 1e-4)
        #expect(absFloat(invWorld.cz.z - 0.125) < 1e-4)
        #expect(absFloat(invWorld.cy.x) < 1e-4)
        #expect(absFloat(invWorld.cz.x) < 1e-4)
        #expect(absFloat(invWorld.cx.y) < 1e-4)
        #expect(absFloat(invWorld.cz.y) < 1e-4)
        #expect(absFloat(invWorld.cx.z) < 1e-4)
        #expect(absFloat(invWorld.cy.z) < 1e-4)
    }

    /// Fixed rotation leaves the mass intact but drives the whole angular inertia to zero.
    @Test("fixed rotation zeroes the inertia but keeps the mass")
    func fixedRotation() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        def.motionLocks.angularX = true
        def.motionLocks.angularY = true
        def.motionLocks.angularZ = true
        let body = world.createBody(def)

        var massData = MassData()
        massData.mass = 5
        massData.center = Vec3()
        massData.inertia = diagInertia
        world.setBodyMassData(body, massData)

        #expect(absFloat(world.bodyMass(body) - 5) < 1e-6)
        #expect(absFloat(world.bodyInverseMass(body) - 0.2) < 1e-6)

        let localInertia = world.bodyLocalRotationalInertia(body)
        #expect(absFloat(localInertia.cx.x) < 1e-6)
        #expect(absFloat(localInertia.cy.y) < 1e-6)
        #expect(absFloat(localInertia.cz.z) < 1e-6)

        let invWorld = world.bodyWorldInverseRotationalInertia(body)
        #expect(absFloat(invWorld.cx.x) < 1e-6)
        #expect(absFloat(invWorld.cy.y) < 1e-6)
        #expect(absFloat(invWorld.cz.z) < 1e-6)

        let md = world.bodyMassData(body)
        #expect(absFloat(md.inertia.cx.x) < 1e-6)
        #expect(absFloat(md.inertia.cy.y) < 1e-6)
        #expect(absFloat(md.inertia.cz.z) < 1e-6)
    }

    /// Zero mass and a zero tensor have zero determinant, so the inverses must collapse to zero
    /// rather than divide by it.
    @Test("zero mass and zero tensor collapse the inverses")
    func zeroMass() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)

        var massData = MassData()
        massData.mass = 0
        massData.center = Vec3()
        massData.inertia = Matrix3()
        world.setBodyMassData(body, massData)

        #expect(absFloat(world.bodyInverseMass(body)) < 1e-6)

        let localInertia = world.bodyLocalRotationalInertia(body)
        #expect(absFloat(localInertia.cx.x) < 1e-6)
        #expect(absFloat(localInertia.cy.y) < 1e-6)
        #expect(absFloat(localInertia.cz.z) < 1e-6)

        let invWorld = world.bodyWorldInverseRotationalInertia(body)
        #expect(absFloat(invWorld.cx.x) < 1e-6)
        #expect(absFloat(invWorld.cy.y) < 1e-6)
        #expect(absFloat(invWorld.cz.z) < 1e-6)
    }

    /// The stored linear velocity tracks the center of mass. Moving the center picks a different
    /// material point, so a spinning body must have its velocity re-referenced by
    /// omega x (newCenter - oldCenter).
    @Test("moving the center of mass re-references a spinning body's velocity")
    func consistentVelocity() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(7, 1, -4)
        let body = world.createBody(def)

        // Spin about the origin center, then shift the center of mass off the origin.
        let omega = Vec3(1, 2, 4)
        world.setBodyLinearVelocity(body, Vec3(1, -2, 3))
        world.setBodyAngularVelocity(body, omega)

        let center = Vec3(0.5, 0.25, 0.125)
        var massData = MassData()
        massData.mass = 3
        massData.center = center
        massData.inertia = diagInertia
        world.setBodyMassData(body, massData)

        // omega x center = (2*0.125 - 4*0.25, 4*0.5 - 1*0.125, 1*0.25 - 2*0.5)
        //               = (-0.75, 1.875, -0.75)
        let v = world.bodyLinearVelocity(body)
        #expect(absFloat(v.x - (1 - 0.75)) < 1e-6)
        #expect(absFloat(v.y - (-2 + 1.875)) < 1e-6)
        #expect(absFloat(v.z - (3 - 0.75)) < 1e-6)
    }
}
