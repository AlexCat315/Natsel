// Debug draw for vehicles and soft bodies.

extension World {
    func drawVehiclesPass(_ options: DebugDraw) {
        guard options.drawVehicles, let drawSegment = options.drawSegment, vehicleIdPool.idCount > 0 else { return }
        for record in vehicles where record.id != B3.nullIndex && isValidBody(record.bodyId) {
            let id = VehicleID(index1: record.id + 1, world0: worldId, generation: record.generation)
            let chassis = bodyTransform(record.bodyId)
            for (w, wheel) in record.wheels.enumerated() {
                let s = wheel.settings
                let attachment = transformWorldPoint(chassis, s.position)
                guard let transform = vehicleWheelTransform(id, Int32(w)) else { continue }
                let center = transform.p
                drawSegment(attachment, center, DebugColors.yellow)

                // Rim in the wheel plane. The local x axis of the wheel transform is the axle.
                let up = rotateVector(transform.q, Vec3(0, 1, 0))
                let forward = rotateVector(transform.q, Vec3(0, 0, 1))
                let segments = 16
                var previous = mulAdd(center, s.radius, up)
                for k in 1...segments {
                    let cs = computeCosSin(B3.twoPi * Float(k) / Float(segments))
                    let point = mulAdd(mulAdd(center, s.radius * cs.cosine, up), s.radius * cs.sine, forward)
                    drawSegment(previous, point, wheel.hasContact ? DebugColors.green : DebugColors.red)
                    previous = point
                }
                // Spoke shows the rotation.
                drawSegment(center, mulAdd(center, s.radius, up), DebugColors.white)

                if wheel.hasContact {
                    let scale = 0.3 * options.jointScale
                    drawSegment(wheel.contactPosition, mulAdd(wheel.contactPosition, scale, wheel.contactNormal), DebugColors.cyan)
                    drawSegment(wheel.contactPosition, mulAdd(wheel.contactPosition, scale, wheel.longitudinal), DebugColors.orange)
                    options.drawPoint?(wheel.contactPosition, 5, DebugColors.cyan)
                }
            }
        }
    }

    func drawSoftBodiesPass(_ options: DebugDraw) {
        guard options.drawSoftBodies, let drawSegment = options.drawSegment, softBodyIdPool.idCount > 0 else { return }
        for record in softBodies where record.id != B3.nullIndex {
            guard aabbOverlaps(record.bounds, options.drawingBounds) else { continue }
            let color = record.awake ? DebugColors.lightSteelBlue : DebugColors.gray
            record.positions.withUnsafeBufferPointer { x in
                for edge in record.settings.edges {
                    drawSegment(x[Int(edge.vertex.x)], x[Int(edge.vertex.y)], color)
                }
                if let drawPoint = options.drawPoint {
                    for i in 0..<x.count {
                        if record.invMasses[i] == 0 {
                            drawPoint(x[i], 5, DebugColors.red)
                        } else if record.hasContact[i] {
                            drawPoint(x[i], 3, DebugColors.orange)
                        }
                    }
                }
            }
        }
    }
}
