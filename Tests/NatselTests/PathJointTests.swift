import Testing
@testable import Natsel

private struct PathRig {
    let world = World()
    let ground: BodyID
    let body: BodyID

    init(gravity: Vec3 = Vec3(0, -10, 0), position: Vec3) {
        world.gravity = gravity
        world.enableSleep = false
        ground = world.createBody(BodyDef())
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(position)
        body = world.createBody(def)
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.2, 0.1, 0.1))
    }

    func joint(_ path: ConstraintPath, _ configure: (inout PathJointDef) -> Void = { _ in }) -> JointID {
        var def = PathJointDef()
        def.base.bodyIdA = ground
        def.base.bodyIdB = body
        def.path = path
        configure(&def)
        return world.createPathJoint(def)
    }

    func run(_ steps: Int, _ each: () -> Void = {}) {
        for _ in 0..<steps {
            world.step(1.0 / 60.0, subStepCount: 4)
            each()
        }
    }
}

/// A circle of radius `r` in the xz plane, counter-clockwise seen from above.
private func circle(_ r: Float, _ points: Int = 12) -> ConstraintPath {
    let positions = (0..<points).map { i -> Vec3 in
        let a = 2 * Float.pi * Float(i) / Float(points)
        let cs = computeCosSin(a)
        return Vec3(r * cs.cosine, 0, -r * cs.sine)
    }
    return ConstraintPath(through: positions, isLooping: true)!
}

private func line(_ from: Vec3, _ to: Vec3) -> ConstraintPath {
    ConstraintPath(points: [
        PathPoint(position: from, tangent: to - from, normal: .axisY),
        PathPoint(position: to, tangent: to - from, normal: .axisY),
    ])!
}

@Suite("Path joint")
struct PathJointTests {
    @Test("path geometry: interpolation, closest point, and arc length")
    func geometry() {
        let straight = line(Vec3(0, 0, 0), Vec3(4, 0, 0))
        #expect(straight.maxFraction == 1)
        #expect(abs(straight.length - 4) < 1.0e-4)
        #expect(abs(straight.closestFraction(to: Vec3(1, 3, 0)) - 0.25) < 1.0e-4)
        #expect(straight.closestFraction(to: Vec3(-2, 0, 0)) == 0)
        #expect(straight.closestFraction(to: Vec3(9, 0, 0)) == 1)
        let frame = straight.point(at: 0.5)
        #expect(distance(frame.position, Vec3(2, 0, 0)) < 1.0e-5)
        #expect(distance(frame.tangent, .axisX) < 1.0e-5)
        #expect(distance(frame.normal, .axisY) < 1.0e-5)
        #expect(distance(frame.binormal, .axisZ) < 1.0e-5)

        let ring = circle(3, 16)
        #expect(ring.maxFraction == 16)
        // Catmull-Rom passes through its points; the loop length is close to 2 pi r.
        for i in 0..<16 {
            let p = ring.point(at: Float(i)).position
            #expect(abs(length(p) - 3) < 1.0e-4)
        }
        #expect(abs(ring.length - 2 * Float.pi * 3) < 0.05)
        // The search agrees with brute force sampling, for queries off the curve and on the seam.
        for query in [Vec3(3.6, 0.2, -0.5), Vec3(-1, 1, 2.5), Vec3(2.9, -0.3, 0.01), Vec3(0.1, 0, 0.1)] {
            let found = ring.closestFraction(to: query, hint: 8)
            var best = Float.greatestFiniteMagnitude
            for k in 0..<16000 {
                best = min(best, distance(ring.point(at: 16 * Float(k) / 16000).position, query))
            }
            #expect(distance(ring.point(at: found).position, query) <= best + 1.0e-4, "query \(query) found \(found)")
        }
        #expect(abs(ring.closestFraction(to: ring.point(at: 16.25).position) - 0.25) < 1.0e-3)
    }

    @Test("bead stays on a straight path and slides freely")
    func straight() {
        let rig = PathRig(gravity: Vec3(5, -10, 0), position: Vec3(0, 1, 0))
        let joint = rig.joint(line(Vec3(-10, 1, 0), Vec3(10, 1, 0)))
        rig.run(60) {
            let p = rig.world.bodyPosition(rig.body)
            #expect(abs(p.y - 1) < 0.01 && abs(p.z) < 0.01)
        }
        // Free sliding: x = 0.5 a t^2 = 2.5 m after one second.
        #expect(abs(rig.world.bodyPosition(rig.body).x - 2.5) < 0.1)
        let force = rig.world.jointConstraintForce(joint)!
        #expect(abs(force.y - 10 * rig.world.bodyMass(rig.body)) < 0.5)
        #expect(abs(force.x) < 0.1)
    }

    @Test("an open path stops the point at its ends")
    func endStop() {
        let rig = PathRig(gravity: Vec3(10, -10, 0), position: Vec3(0, 0, 0))
        let joint = rig.joint(line(Vec3(-1, 0, 0), Vec3(2, 0, 0)))
        rig.run(120)
        #expect(abs(rig.world.bodyPosition(rig.body).x - 2) < 0.01)
        #expect(rig.world.pathJointFraction(joint)! == 1)
        #expect(length(rig.world.bodyLinearVelocity(rig.body)) < 0.05)
    }

    @Test("friction holds the point until gravity beats it")
    func friction() {
        let rig = PathRig(gravity: Vec3(3, -10, 0), position: Vec3(0, 0, 0))
        let mass = rig.world.bodyMass(rig.body)
        let joint = rig.joint(line(Vec3(-5, 0, 0), Vec3(5, 0, 0))) { $0.maxFrictionForce = 4 * mass }
        rig.run(60)
        #expect(abs(rig.world.bodyPosition(rig.body).x) < 0.01)
        rig.world.setPathJointMaxFriction(joint, 2 * mass)
        rig.run(60)
        // Net acceleration 1 m/s^2: 0.5 m after one second.
        #expect(abs(rig.world.bodyPosition(rig.body).x - 0.5) < 0.05)
    }

    @Test("velocity motor drives around a loop and follows the path frame")
    func loopMotor() {
        let ring = circle(3)
        let rig = PathRig(gravity: Vec3(), position: Vec3(3, 0, 0))
        let joint = rig.joint(ring) {
            $0.motorMode = .velocity
            $0.targetVelocity = 4
            $0.maxMotorForce = 1000
            $0.rotationConstraint = .constrainToPath
        }
        let start = ring.point(at: 0)
        var last: Float = 0
        var wraps = 0
        rig.run(360) {
            let p = rig.world.bodyPosition(rig.body)
            #expect(abs(length(subPos(Pos(p.x, Float(0.0), p.z), .zero)) - 3) < 0.03 && abs(p.y) < 0.01)
            let f = rig.world.pathJointFraction(joint)!
            if f < last - 6 { wraps += 1 }
            last = f
            // The body turns with the path frame.
            let frame = ring.point(at: f)
            let q = rig.world.bodyRotation(rig.body)
            #expect(dot(rotateVector(q, start.tangent), frame.tangent) > 0.999)
            #expect(dot(rotateVector(q, start.normal), frame.normal) > 0.999)
        }
        #expect(abs(length(rig.world.bodyLinearVelocity(rig.body)) - 4) < 0.05)
        // 4 m/s for 6 s on a 18.85 m loop.
        #expect(wraps == 1)
    }

    @Test("position motor reaches a target fraction")
    func positionMotor() {
        let path = ConstraintPath(through: [Vec3(0, 0, 0), Vec3(2, 0, 1), Vec3(4, 0, 0), Vec3(6, 0, -1)])!
        let rig = PathRig(gravity: Vec3(), position: Vec3(0, 0, 0))
        let joint = rig.joint(path) {
            $0.motorMode = .position
            $0.targetFraction = 2.5
            $0.maxMotorForce = 200
            $0.motorHertz = 2
            $0.motorDampingRatio = 1
        }
        rig.run(240)
        #expect(abs(rig.world.pathJointFraction(joint)! - 2.5) < 0.01)
        #expect(distance(rig.world.bodyPosition(rig.body), path.point(at: 2.5).position) < 0.02)
        rig.world.setPathJointTargetFraction(joint, 0.5)
        rig.run(240)
        #expect(abs(rig.world.pathJointFraction(joint)! - 0.5) < 0.01)
    }

    @Test("rotation constraints")
    func rotations() {
        let ring = circle(3)
        func spin(_ rotation: PathRotationConstraint) -> (PathRig, JointID) {
            let rig = PathRig(gravity: Vec3(), position: Vec3(3, 0, 0))
            let joint = rig.joint(ring) {
                $0.rotationConstraint = rotation
                $0.motorMode = .velocity
                $0.targetVelocity = 3
                $0.maxMotorForce = 1000
            }
            rig.world.setBodyAngularVelocity(rig.body, Vec3(2, 3, 4))
            return (rig, joint)
        }

        // Fully constrained: the body keeps its orientation while it circles.
        do {
            let (rig, _) = spin(.fullyConstrained)
            rig.run(120) { #expect(abs(rig.world.bodyRotation(rig.body).s) > 0.999) }
        }
        // Hinged about a path axis: body B's copy of that axis (its direction at the start, since the
        // body starts unrotated) stays on the path axis while the body spins about it.
        let start = ring.point(at: 0)
        for (rotation, axis) in [(PathRotationConstraint.constrainAroundTangent, 0), (.constrainAroundNormal, 1),
                                 (.constrainAroundBinormal, 2)] {
            let (rig, joint) = spin(rotation)
            rig.run(120) {
                let frame = ring.point(at: rig.world.pathJointFraction(joint)!)
                let pathAxis = [frame.tangent, frame.normal, frame.binormal][axis]
                let bodyAxis = rotateVector(rig.world.bodyRotation(rig.body), [start.tangent, start.normal, start.binormal][axis])
                #expect(dot(bodyAxis, pathAxis) > 0.995, "\(rotation)")
            }
            // It still spins about that axis.
            #expect(length(rig.world.bodyAngularVelocity(rig.body)) > 0.5, "\(rotation)")
        }
    }

    @Test("path on a moving body and joint lifetime")
    func dynamicCarrier() {
        let world = World()
        world.gravity = Vec3(0, -10, 0)
        world.enableSleep = false
        var def = BodyDef()
        def.type = .dynamicBody
        let carrier = world.createBody(def)
        _ = world.createHullShape(carrier, ShapeDef(), makeBoxHull(2, 0.1, 0.5))
        def.position = Pos(Float(0.5), Float(0.3), 0)
        let bead = world.createBody(def)
        _ = world.createSphereShape(bead, ShapeDef(), Sphere(center: Vec3(), radius: 0.1))
        var joint = PathJointDef()
        joint.base.bodyIdA = carrier
        joint.base.bodyIdB = bead
        joint.base.localFrameA = Transform(Vec3(0, 0.3, 0), .identity)
        joint.path = line(Vec3(-2, 0, 0), Vec3(2, 0, 0))
        let id = world.createPathJoint(joint)
        world.setBodyAngularVelocity(carrier, Vec3(0, 0, 1))
        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
        // The bead stays on the line through the carrier's frame.
        let local = invTransformWorldPoint(world.bodyTransform(carrier), world.bodyPosition(bead))
        #expect(abs(local.y - 0.3) < 0.01 && abs(local.z) < 0.01)

        world.destroyJoint(id.index1 - 1, true)
        #expect(world.pathJointFraction(id) == nil)
        // The slot is reused.
        let again = world.createPathJoint(joint)
        #expect(world.pathJointFraction(again) != nil)
        #expect(world.pathJointPath(again) === joint.path)
    }
}
