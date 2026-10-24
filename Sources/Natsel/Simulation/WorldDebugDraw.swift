

// Debug draw (physics_world.c b3World_Draw).

extension World {
    /// Walk the world and emit debug draw callbacks for everything the caller asked to see.
    public func draw(_ options: DebugDraw, maskBits: UInt64 = .max) {
        // Culling query over all three trees. Each hit records its body for the per-body pass.
        var seenBodies = Set<Int32>()

        for tree in broadPhase.trees {
            _ = tree.query(options.drawingBounds, maskBits: maskBits, requireAllBits: false) {
                _, data in
                let shape = self.shapes[Int(data)]
                seenBodies.insert(shape.bodyId)
                self.drawShapeEntry(shape, options)
                return true
            }
        }

        // Per body extras: names, mass, sleep, joints, contacts, islands.
        for bodyId in seenBodies.sorted() {
            drawBodyExtras(bodyId, options)
        }

        drawJointExtrasPass(options)
        drawVehiclesPass(options)
        drawSoftBodiesPass(options)
    }

    // MARK: Shape

    private func drawShapeEntry(_ shape: Shape, _ options: DebugDraw) {
        if options.drawShapes {
            let color = debugColor(for: shape)

            if shape.userShape == nil, let factory = options.createDebugShape {
                let debugShape = DebugShape(
                    shapeId: ShapeID(
                        index1: shape.id + 1, world0: worldId, generation: shape.generation),
                    type: shape.type,
                    geometry: debugGeometry(for: shape)
                )
                var mutable = shape
                mutable.userShape = factory(debugShape)
                shapes[Int(shape.id)] = mutable
            }

            let cached = shapes[Int(shape.id)]
            if let userShape = cached.userShape, let drawShape = options.drawShape {
                drawShape(userShape, bodyTransformQuick(cached.bodyId), color)
            }
        }

        if options.drawBoundsOption, let drawBounds = options.drawBounds {
            drawBounds(fatAABBs[Int(shape.id)], DebugColors.gold)
        }
    }

    private func debugGeometry(for shape: Shape) -> DebugShapeGeometry {
        switch shape.geometry {
        case .capsule(let capsule):
            let p = UnsafeMutablePointer<Capsule>.allocate(capacity: 1)
            p.initialize(to: capsule)
            return .capsule(UnsafePointer(p))
        case .compound(let compound):
            return .compound(compound)
        case .heightField(let field):
            return .heightField(field)
        case .hull(let hull):
            // Hull is a view; expose its header for renderers that need the shape data.
            return .hull(UnsafePointer(hull.header))
        case .mesh(let mesh):
            let p = UnsafeMutablePointer<Mesh>.allocate(capacity: 1)
            p.initialize(to: mesh)
            return .mesh(UnsafePointer(p))
        case .sphere(let sphere):
            let p = UnsafeMutablePointer<Sphere>.allocate(capacity: 1)
            p.initialize(to: sphere)
            return .sphere(UnsafePointer(p))
        }
    }

    /// Hue carries the body state, material carries its energy. Diagnostic states keep a saturated
    /// hue so they pop.
    private func debugColor(for shape: Shape) -> HexColor {
        let material = shape.material(0)
        if material.customColor != 0 {
            // May already carry a packed material preset, pass through unchanged.
            return material.customColor
        }

        let body = bodies[Int(shape.bodyId)]
        let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]

        var rgb: HexColor
        var preset = DebugMaterial.default

        if body.type == .dynamicBody && body.mass == 0 {
            rgb = DebugColors.red
        } else if body.setIndex == SetType.disabledSet.rawValue {
            rgb = DebugColors.slateGray
        } else if shape.sensorIndex != B3.nullIndex {
            rgb = DebugColors.wheat
        } else if (sim.flags & BodyFlags.hadTimeOfImpact.rawValue) != 0 {
            rgb = DebugColors.lime
        } else if (sim.flags & BodyFlags.isFast.rawValue) != 0
            && body.setIndex == SetType.awakeSet.rawValue
        {
            rgb = DebugColors.turquoise
        } else if (body.flags & BodyFlags.isSpeedCapped.rawValue) != 0 {
            rgb = DebugColors.yellow
        } else if body.type == .staticBody {
            rgb = DebugColors.darkGray
            preset = .matte
        } else if body.type == .kinematicBody {
            if body.setIndex == SetType.awakeSet.rawValue {
                rgb = DebugColors.steelBlue
                preset = .metallic
            } else {
                rgb = DebugColors.lightSteelBlue
                preset = .matte
            }
        } else if body.setIndex == SetType.awakeSet.rawValue {
            rgb = DebugColors.tan
            preset = .soft
        } else {
            rgb = DebugColors.lightSlateGray
            preset = .dead
        }

        return makeDebugColor(rgb, preset)
    }

    // MARK: Per body

    private func drawBodyExtras(_ bodyId: Int32, _ options: DebugDraw) {
        let body = bodies[Int(bodyId)]
        if body.setIndex == SetType.disabledSet.rawValue { return }

        let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]

        if options.drawBodyNames, let name = names.find(body.nameId),
            let drawString = options.drawString
        {
            let transform = WorldTransform(sim.center, sim.transform.q)
            let p = transformWorldPoint(transform, Vec3(0.05, 0.05, 0.05))
            drawString(p, name, DebugColors.white)
        }

        if options.drawMass {
            let transform = WorldTransform(sim.center, sim.transform.q)
            options.drawTransform?(transform)
            if body.type == .dynamicBody, let drawString = options.drawString {
                let p = transformWorldPoint(transform, Vec3(0.05, 0.05, 0.05))
                drawString(p, String(format: "%.2f", body.mass), DebugColors.white)
            }
        }

        if options.drawSleep, let drawPoint = options.drawPoint {
            let colors = [
                DebugColors.blue, DebugColors.skyBlue, DebugColors.orange, DebugColors.red,
            ]
            var color = DebugColors.black
            if body.sleepThreshold > 0 {
                let ratio = body.sleepVelocity / body.sleepThreshold
                color = colors[Int(clampInt(Int32(ratio), 0, 3))]
            }
            drawPoint(sim.center, 10, color)
            if let drawString = options.drawString {
                let p = offsetPos(sim.center, Vec3(0.1, 0.1, 0.1))
                drawString(p, String(format: "  %.3f", body.sleepVelocity), color)
            }
        }

        if options.drawContacts, body.type == .dynamicBody,
            body.setIndex == SetType.awakeSet.rawValue
        {
            drawBodyContacts(bodyId, body, options)
        }
    }

    private func drawBodyContacts(_ bodyId: Int32, _ body: Body, _ options: DebugDraw) {
        let axisScale = 0.3 * getLengthUnitsPerMeter()
        var key = body.headContactKey

        while key != B3.nullIndex {
            let contactId = key >> 1
            let edgeIndex = Int(key & 1)
            let contact = contacts[Int(contactId)]
            key = contact.edges[edgeIndex].nextKey

            guard contact.setIndex == SetType.awakeSet.rawValue,
                contact.colorIndex != B3.nullIndex
            else { continue }

            let simA = solverSets[Int(bodies[Int(contact.edges[0].bodyId)].setIndex)]
                .bodySims[Int(bodies[Int(contact.edges[0].bodyId)].localIndex)]
            let simB = solverSets[Int(bodies[Int(contact.edges[1].bodyId)].setIndex)]
                .bodySims[Int(bodies[Int(contact.edges[1].bodyId)].localIndex)]

            for manifoldIndex in 0..<Int(contact.manifoldCount) {
                let manifold = contact.manifolds[manifoldIndex]
                let normal = manifold.normal

                // Average the anchors rather than the world points so the friction center stays
                // exact far from the origin.
                let contactCenter = options.drawAnchorA ? simA.center : simB.center

                for pointIndex in 0..<Int(manifold.pointCount) {
                    let mp = manifold.points[pointIndex]
                    let anchor = options.drawAnchorA ? mp.anchorA : mp.anchorB
                    let p = offsetPos(contactCenter, anchor)

                    if options.drawContactNormals, let drawSegment = options.drawSegment {
                        let p2 = offsetPos(p, axisScale * normal)
                        drawSegment(p, p2, DebugColors.lightGray)
                        options.drawString?(
                            p, String(format: "   %.2f", 100 * mp.separation),
                            DebugColors.white)
                    } else if options.drawContactForces, let drawSegment = options.drawSegment {
                        let invDt = inv_dt > 0 ? inv_dt : 60
                        let force = mp.totalNormalImpulse * invDt
                        let p2 = offsetPos(p, options.forceScale * force * normal)
                        drawSegment(p, p2, DebugColors.magenta)
                        options.drawString?(p, String(format: "  %.1f", force), DebugColors.white)
                    } else if options.drawContactFeatures, let drawString = options.drawString {
                        drawString(
                            p, String(format: "   %#X", mp.featureId), DebugColors.white)
                    }

                    if options.drawGraphColors, let drawPoint = options.drawPoint {
                        let size: Float = contact.colorIndex == overflowIndex ? 15 : 10
                        let idx = Int(contact.colorIndex)
                        let palette = DebugColors.graph
                        drawPoint(p, size, palette[idx % palette.count])
                    }
                }
            }
        }
    }

    // MARK: Joints

    private func drawJointExtrasPass(_ options: DebugDraw) {
        guard options.drawJoints, let drawSegment = options.drawSegment else { return }
        let drawPoint = options.drawPoint
        let scale = maxFloat(0.0001, options.jointScale)

        for index in 0..<Int32(joints.count) {
            let joint = joints[Int(index)]
            if joint.jointId == B3.nullIndex { continue }

            let bodyA = bodies[Int(joint.edges[0].bodyId)]
            let bodyB = bodies[Int(joint.edges[1].bodyId)]
            if bodyA.setIndex == SetType.disabledSet.rawValue
                || bodyB.setIndex == SetType.disabledSet.rawValue
            {
                continue
            }

            let sim = readJointSim(index)
            let xfA = bodyTransformQuick(joint.edges[0].bodyId)
            let xfB = bodyTransformQuick(joint.edges[1].bodyId)
            let pA = transformWorldPoint(xfA, sim.localFrameA.p)
            let pB = transformWorldPoint(xfB, sim.localFrameB.p)

            switch joint.type {
            case .distanceJoint:
                drawDistanceJoint(sim, xfA, xfB, pA, pB, drawSegment, drawPoint)

            case .revoluteJoint:
                drawRevoluteJoint(sim, xfA, xfB, scale, drawSegment, drawPoint)

            case .prismaticJoint:
                drawPrismaticJoint(sim, xfA, xfB, scale, drawSegment, drawPoint)

            case .sphericalJoint:
                drawSphericalJoint(sim, xfA, xfB, scale, drawSegment, drawPoint)

            case .wheelJoint:
                drawWheelJoint(sim, xfA, xfB, scale, drawSegment, drawPoint)

            case .weldJoint:
                let frameA = mulWorldTransforms(xfA, sim.localFrameA)
                let frameB = mulWorldTransforms(xfB, sim.localFrameB)
                let extents = Vec3(0.1 * scale, 0.05 * scale, 0.025 * scale)
                options.drawBox?(extents, frameA, DebugColors.darkOrange)
                options.drawBox?(extents, frameB, DebugColors.darkCyan)

            case .parallelJoint:
                let length = 0.1 * scale
                let frameA = mulWorldTransforms(xfA, sim.localFrameA)
                drawSegment(
                    frameA.p, offsetPos(frameA.p, length * rotateVector(frameA.q, .axisZ)),
                    DebugColors.green)
                let frameB = mulWorldTransforms(xfB, sim.localFrameB)
                drawSegment(
                    frameB.p, offsetPos(frameB.p, length * rotateVector(frameB.q, .axisZ)),
                    DebugColors.blue)

            case .motorJoint:
                drawSegment(pA, pB, DebugColors.plum)
                drawPoint?(pA, 8, DebugColors.lime)
                drawPoint?(pB, 8, DebugColors.plum)

            case .filterJoint:
                drawSegment(pA, pB, DebugColors.gold)

            case .gearJoint, .rackAndPinionJoint, .pulleyJoint, .sixDOFJoint, .pathJoint:
                drawCouplingJoint(sim, xfA, xfB, pA, pB, scale, drawSegment, drawPoint)
            }

            if options.drawGraphColors, let drawPoint {
                let idx = Int(joint.colorIndex >= 0 ? joint.colorIndex : overflowIndex)
                let palette = DebugColors.graph
                drawPoint(pA, 6, palette[idx % palette.count])
            }
        }
    }

    // MARK: Per type

    private func drawDistanceJoint(
        _ sim: JointSim, _ xfA: WorldTransform, _ xfB: WorldTransform,
        _ pA: Pos, _ pB: Pos,
        _ drawSegment: (Pos, Pos, HexColor) -> Void,
        _ drawPoint: ((Pos, Float, HexColor) -> Void)?
    ) {
        let joint = sim.distance
        let axis = normalize(subPos(pB, pA))

        if joint.minLength < joint.maxLength && joint.enableLimit {
            let pMin = offsetPos(pA, joint.minLength * axis)
            let pMax = offsetPos(pA, joint.maxLength * axis)

            if joint.minLength > B3.linearSlop {
                drawPoint?(pMin, 6, DebugColors.lightGreen)
            }
            if joint.maxLength < B3.huge {
                drawPoint?(pMax, 6, DebugColors.red)
            }
            if joint.minLength > B3.linearSlop && joint.maxLength < B3.huge {
                drawSegment(pMin, pMax, DebugColors.gray)
            }
        }

        drawSegment(pA, pB, DebugColors.white)
        drawPoint?(pA, 4, DebugColors.white)
        drawPoint?(pB, 4, DebugColors.white)

        if joint.hertz > 0 && joint.enableSpring {
            let pRest = offsetPos(pA, joint.length * axis)
            drawPoint?(pRest, 4, DebugColors.blue)
        }
    }

    /// Draw the local frame axes of a joint at world `frame`.
    private func drawFrameAxes(
        _ frame: WorldTransform, _ length: Float,
        _ drawSegment: (Pos, Pos, HexColor) -> Void
    ) {
        drawSegment(
            toVec3(frame.p), offsetPos(frame.p, length * rotateVector(frame.q, .axisX)),
            DebugColors.red)
        drawSegment(
            toVec3(frame.p), offsetPos(frame.p, length * rotateVector(frame.q, .axisY)),
            DebugColors.green)
        drawSegment(
            toVec3(frame.p), offsetPos(frame.p, length * rotateVector(frame.q, .axisZ)),
            DebugColors.blue)
    }

    /// Draw an angular wedge from lower to upper about the frame's z axis, plus a marker at the
    /// current twist.
    private func drawAngularWedge(
        _ frameA: WorldTransform, _ frameB: WorldTransform,
        _ lower: Float, _ upper: Float, _ radius: Float,
        _ drawSegment: (Pos, Pos, HexColor) -> Void
    ) {
        let sliceCount = 16

        let quatA = frameA.q
        var quatB = frameB.q
        if dotQuat(quatA, quatB) < 0 {
            // This keeps the twist angle in the range [-pi, pi].
            quatB = Quat(-quatB.v, -quatB.s)
        }
        let relQ = invMulQuat(quatA, quatB)

        var previous = Vec3()
        for index in 0...sliceCount {
            let t = Float(index) / Float(sliceCount)
            let alpha = lerpFloat(lower, upper, t)
            let cs = computeCosSin(alpha)
            let vertex = Vec3(radius * cs.cosine, radius * cs.sine, 0)

            if index == 0 {
                drawSegment(toVec3(frameA.p), transformWorldPoint(frameA, vertex), DebugColors.cyan)
            }
            if index == sliceCount {
                drawSegment(transformWorldPoint(frameA, vertex), toVec3(frameA.p), DebugColors.cyan)
            }
            if index > 0 {
                drawSegment(
                    transformWorldPoint(frameA, previous), transformWorldPoint(frameA, vertex),
                    DebugColors.cyan)
            }
            previous = vertex
        }

        // Marker at the current twist.
        let twist = getTwistAngle(relQ)
        let cs = computeCosSin(twist)
        let mark = Vec3(radius * cs.cosine, radius * cs.sine, 0)
        drawSegment(toVec3(frameA.p), transformWorldPoint(frameA, mark), DebugColors.yellow)
    }

    private func drawRevoluteJoint(
        _ sim: JointSim, _ xfA: WorldTransform, _ xfB: WorldTransform, _ scale: Float,
        _ drawSegment: (Pos, Pos, HexColor) -> Void,
        _ drawPoint: ((Pos, Float, HexColor) -> Void)?
    ) {
        let joint = sim.revolute
        let frameA = mulWorldTransforms(xfA, sim.localFrameA)
        let frameB = mulWorldTransforms(xfB, sim.localFrameB)

        drawFrameAxes(frameA, 0.1 * scale, drawSegment)

        if joint.enableLimit {
            drawAngularWedge(
                frameA, frameB, joint.lowerAngle, joint.upperAngle, 0.2 * scale, drawSegment)
        }
    }

    private func drawPrismaticJoint(
        _ sim: JointSim, _ xfA: WorldTransform, _ xfB: WorldTransform, _ scale: Float,
        _ drawSegment: (Pos, Pos, HexColor) -> Void,
        _ drawPoint: ((Pos, Float, HexColor) -> Void)?
    ) {
        let joint = sim.prismatic
        let frameA = mulWorldTransforms(xfA, sim.localFrameA)
        let frameB = mulWorldTransforms(xfB, sim.localFrameB)

        let matrix = makeMatrixFromQuat(frameA.q)
        let axis = matrix.cx
        let s = 0.2 * scale

        drawSegment(
            toVec3(frameA.p), offsetPos(frameA.p, s * matrix.cy), DebugColors.green)
        drawSegment(
            toVec3(frameA.p), offsetPos(frameA.p, s * matrix.cz), DebugColors.blue)

        if joint.enableLimit {
            let p1 = offsetPos(frameA.p, joint.lowerTranslation * axis)
            let p2 = offsetPos(frameA.p, joint.upperTranslation * axis)
            drawSegment(p1, p2, DebugColors.orange)
            drawPoint?(p1, 10, DebugColors.green)
            drawPoint?(p2, 10, DebugColors.red)
        } else {
            let p1 = offsetPos(frameA.p, -0.5 * scale * axis)
            let p2 = offsetPos(frameA.p, 0.5 * scale * axis)
            drawSegment(p1, p2, DebugColors.orange)
        }

        drawPoint?(frameB.p, 8, DebugColors.violet)
    }

    private func drawSphericalJoint(
        _ sim: JointSim, _ xfA: WorldTransform, _ xfB: WorldTransform, _ scale: Float,
        _ drawSegment: (Pos, Pos, HexColor) -> Void,
        _ drawPoint: ((Pos, Float, HexColor) -> Void)?
    ) {
        let joint = sim.sphericalJoint
        let frameA = mulWorldTransforms(xfA, sim.localFrameA)
        let frameB = mulWorldTransforms(xfB, sim.localFrameB)

        drawFrameAxes(frameA, 0.1 * scale, drawSegment)

        // The cone axis of body B.
        drawSegment(
            toVec3(frameB.p),
            offsetPos(frameB.p, 0.2 * scale * rotateVector(frameB.q, .axisZ)),
            DebugColors.orange)

        let sliceCount = 16

        if joint.enableTwistLimit {
            drawAngularWedge(
                frameA, frameB, joint.lowerTwistAngle, joint.upperTwistAngle, 0.1 * scale,
                drawSegment)
        }

        if joint.enableConeLimit {
            let radius = 0.1 * scale
            let cone = computeCosSin(joint.coneAngle)
            let coneRadius = radius * cone.sine
            let coneHeight = radius * cone.cosine

            var previous = Vec3()
            for index in 0...sliceCount {
                let phi = 2 * Float(index) / Float(sliceCount) * B3.pi
                let pcs = computeCosSin(phi)
                let vertex = Vec3(coneRadius * pcs.cosine, coneRadius * pcs.sine, coneHeight)

                drawSegment(
                    toVec3(frameA.p), transformWorldPoint(frameA, vertex), DebugColors.cyan)
                if index > 0 {
                    drawSegment(
                        transformWorldPoint(frameA, previous),
                        transformWorldPoint(frameA, vertex), DebugColors.cyan)
                }
                previous = vertex
            }
        }
    }

    private func drawWheelJoint(
        _ sim: JointSim, _ xfA: WorldTransform, _ xfB: WorldTransform, _ scale: Float,
        _ drawSegment: (Pos, Pos, HexColor) -> Void,
        _ drawPoint: ((Pos, Float, HexColor) -> Void)?
    ) {
        let joint = sim.wheel
        let frameA = mulWorldTransforms(xfA, sim.localFrameA)
        let frameB = mulWorldTransforms(xfB, sim.localFrameB)

        let matrixA = makeMatrixFromQuat(frameA.q)
        let matrixB = makeMatrixFromQuat(frameB.q)

        drawSegment(toVec3(frameA.p), toVec3(frameB.p), DebugColors.blue)

        if joint.enableSuspensionLimit {
            let lower = offsetPos(frameA.p, joint.lowerSuspensionLimit * matrixA.cx)
            let upper = offsetPos(frameA.p, joint.upperSuspensionLimit * matrixA.cx)
            let perp = matrixA.cy
            drawSegment(lower, upper, DebugColors.gray)
            drawSegment(
                offsetPos(lower, -0.1 * scale * perp), offsetPos(lower, 0.1 * scale * perp),
                DebugColors.green)
            drawSegment(
                offsetPos(upper, -0.1 * scale * perp), offsetPos(upper, 0.1 * scale * perp),
                DebugColors.red)
        } else {
            drawSegment(
                offsetPos(frameA.p, -1.0 * scale * matrixA.cx),
                offsetPos(frameA.p, 1.0 * scale * matrixA.cx), DebugColors.gray)
        }

        if joint.enableSteering && joint.enableSteeringLimit {
            let frame = WorldTransform(frameB.p, frameA.q)
            let radius = 0.5 * scale
            let sliceCount = 16
            let lower = joint.lowerSteeringLimit
            let upper = joint.upperSteeringLimit

            var cs = computeCosSin(lower)
            var previous = transformWorldPoint(
                frame, Vec3(0, -radius * cs.sine, radius * cs.cosine))

            for index in 0..<sliceCount {
                let t = (Float(index) + 1) / Float(sliceCount)
                let phi = lerpFloat(lower, upper, t)
                cs = computeCosSin(phi)
                let vertex = transformWorldPoint(
                    frame, Vec3(0, -radius * cs.sine, radius * cs.cosine))

                if index == 0 {
                    drawSegment(toVec3(frame.p), previous, DebugColors.cyan)
                }
                if index == sliceCount - 1 {
                    drawSegment(vertex, toVec3(frame.p), DebugColors.cyan)
                }
                drawSegment(previous, vertex, DebugColors.cyan)
                previous = vertex
            }
        }

        drawSegment(
            offsetPos(frameB.p, -0.5 * scale * matrixB.cz),
            offsetPos(frameB.p, 0.5 * scale * matrixB.cz), DebugColors.magenta)

        drawPoint?(frameA.p, 5, DebugColors.gray)
        drawPoint?(frameB.p, 5, DebugColors.dimGray)
    }
}
