

// Sensors and their touch events (sensor.c).

/// A shape that overlaps a sensor.
struct SensorVisitor: BitwiseCopyable {
    var shapeId: Int32 = B3.nullIndex
    var generation: UInt16 = 0

    init() {}
}

/// A sensor records the shapes overlapping it across two frames.
struct Sensor {
    var shapeId: Int32 = B3.nullIndex

    /// Hits recorded during the current step (e.g. by CCD).
    var hits: [SensorVisitor] = []

    /// Overlaps from the previous frame.
    var overlaps1: [SensorVisitor] = []

    /// Overlaps from the current frame.
    var overlaps2: [SensorVisitor] = []

    init() {}
}

extension World {
    /// True when a sensor shape overlaps a visitor shape.
    func overlapSensor(
        _ sensorShape: Shape, _ sensorTransform: Transform, _ visitorShape: Shape,
        _ visitorTransform: Transform
    ) -> Bool {
        let proxy = makeShapeProxy(visitorShape)

        // Get the visitor shape in the frame of the sensor
        let relativeTransform = invMulTransforms(sensorTransform, visitorTransform)

        return proxy.withProxy { proxy in
            var localPoints = [Vec3](repeating: Vec3(), count: Int(B3.maxShapeCastPoints))
            let count = minInt(proxy.count, B3.maxShapeCastPoints)
            for i in 0..<Int(count) {
                localPoints[i] = transformPoint(relativeTransform, proxy.points![i])
            }

            return localPoints.withUnsafeMutableBufferPointer { buffer in
                let localProxy = ShapeProxy(
                    points: buffer.baseAddress, count: count, radius: proxy.radius
                )
                let identity = Transform(Vec3(), Quat.identity)

                switch sensorShape.geometry {
                case .sphere(let sphere):
                    return overlapSphere(sphere, identity, localProxy)
                case .capsule(let capsule):
                    return overlapCapsule(capsule, identity, localProxy)
                case .hull(let hull):
                    return overlapHull(hull, identity, localProxy)
                case .compound(let compound):
                    return overlapCompound(compound, identity, localProxy)
                case .heightField(let field):
                    return overlapHeightField(field, identity, localProxy)
                case .mesh(let mesh):
                    return overlapMesh(mesh, identity, localProxy)
                }
            }
        }
    }

    /// Query callback for one sensor.
    private func sensorQueryCallback(
        _ shapeId: Int32, _ sensor: inout Sensor, _ sensorShape: Shape, _ transform: Transform
    ) -> Bool {
        if shapeId == sensorShape.id {
            return true
        }

        let otherShape = shapes[Int(shapeId)]

        // Visitors must be convex.
        if isConvex(otherShape.type) == false {
            return true
        }

        // Are sensor events enabled on the other shape?
        if (otherShape.flags & ShapeFlags.enableSensorEvents.rawValue) == 0 {
            return true
        }

        // Skip shapes on the same body
        if otherShape.bodyId == sensorShape.bodyId {
            return true
        }

        // Check filter
        if shouldShapesCollide(sensorShape.filter, otherShape.filter) == false {
            return true
        }

        let otherTransform = toRelativeTransform(bodyTransformQuick(otherShape.bodyId), Pos())

        if overlapSensor(sensorShape, transform, otherShape, otherTransform) == false {
            return true
        }

        // Record the overlap
        var visitor = SensorVisitor()
        visitor.shapeId = shapeId
        visitor.generation = otherShape.generation
        sensor.overlaps2.append(visitor)

        return true
    }

    /// Update all sensors and publish begin/end touch events.
    func overlapSensors() {
        let sensorCount = sensors.count
        if sensorCount == 0 {
            return
        }

        var eventBits = BitSet(bitCapacity: UInt32(maxInt(Int32(sensorCount), 8)))
        eventBits.setBitCountAndClear(UInt32(maxInt(Int32(sensorCount), 8)))
        defer { eventBits.destroy() }

        for sensorIndex in 0..<sensorCount {
            var sensor = sensors[sensorIndex]
            let sensorShape = shapes[Int(sensor.shapeId)]

            // Swap overlap arrays
            swap(&sensor.overlaps1, &sensor.overlaps2)
            sensor.overlaps2.removeAll()

            // Append sensor hits
            sensor.overlaps2.append(contentsOf: sensor.hits)

            // Clear the hits
            sensor.hits.removeAll()

            let body = bodies[Int(sensorShape.bodyId)]
            if body.setIndex == SetType.disabledSet.rawValue
                || (sensorShape.flags & ShapeFlags.enableSensorEvents.rawValue) == 0
            {
                if sensor.overlaps1.isEmpty == false {
                    // This sensor is dropping all overlaps because it has been disabled.
                    eventBits.setBit(UInt32(sensorIndex))
                }
                sensors[sensorIndex] = sensor
                continue
            }

            let transform = toRelativeTransform(bodyTransformQuick(sensorShape.bodyId), Pos())

            let queryBounds = sensorShape.aabb
            for treeIndex in 0..<Int(BodyType.count) {
                _ = broadPhase.trees[treeIndex].query(
                    queryBounds, maskBits: sensorShape.filter.maskBits, requireAllBits: false
                ) { _, userData in
                    let otherShapeId = Int32(Int64(bitPattern: userData))
                    return self.sensorQueryCallback(
                        otherShapeId, &sensor, sensorShape, transform
                    )
                }
            }

            // Sort the overlaps to enable finding begin and end events.
            sensor.overlaps2.sort { $0.shapeId < $1.shapeId }

            // Remove duplicates from overlaps2.
            var uniqueCount = 0
            for i in 0..<sensor.overlaps2.count {
                if uniqueCount == 0
                    || sensor.overlaps2[i].shapeId != sensor.overlaps2[uniqueCount - 1].shapeId
                {
                    sensor.overlaps2[uniqueCount] = sensor.overlaps2[i]
                    uniqueCount += 1
                }
            }
            sensor.overlaps2.removeLast(sensor.overlaps2.count - uniqueCount)

            let count1 = sensor.overlaps1.count
            let count2 = sensor.overlaps2.count
            if count1 != count2 {
                eventBits.setBit(UInt32(sensorIndex))
            } else {
                for i in 0..<count1 {
                    if sensor.overlaps1[i].shapeId != sensor.overlaps2[i].shapeId
                        || sensor.overlaps1[i].generation != sensor.overlaps2[i].generation
                    {
                        eventBits.setBit(UInt32(sensorIndex))
                        break
                    }
                }
            }

            sensors[sensorIndex] = sensor
        }

        // Iterate sensors bits and publish events
        for sensorIndex in 0..<sensorCount where eventBits.getBit(UInt32(sensorIndex)) {
            let sensor = sensors[sensorIndex]
            let sensorShape = shapes[Int(sensor.shapeId)]
            let sensorId = ShapeID(
                index1: sensor.shapeId + 1, world0: worldId, generation: sensorShape.generation
            )

            let count1 = sensor.overlaps1.count
            let count2 = sensor.overlaps2.count
            let refs1 = sensor.overlaps1
            let refs2 = sensor.overlaps2

            // overlaps1 can have overlaps that end
            // overlaps2 can have overlaps that begin
            var index1 = 0
            var index2 = 0
            while index1 < count1 && index2 < count2 {
                let r1 = refs1[index1]
                let r2 = refs2[index2]
                if r1.shapeId == r2.shapeId {
                    if r1.generation < r2.generation {
                        // end
                        var event = SensorEndTouchEvent()
                        event.sensorShapeId = sensorId
                        event.visitorShapeId = ShapeID(
                            index1: r1.shapeId + 1, world0: worldId, generation: r1.generation
                        )
                        sensorEndEvents.pending.append(event)
                        index1 += 1
                    } else if r1.generation > r2.generation {
                        // begin
                        var event = SensorBeginTouchEvent()
                        event.sensorShapeId = sensorId
                        event.visitorShapeId = ShapeID(
                            index1: r2.shapeId + 1, world0: worldId, generation: r2.generation
                        )
                        sensorBeginEvents.append(event)
                        index2 += 1
                    } else {
                        // persisted
                        index1 += 1
                        index2 += 1
                    }
                } else if r1.shapeId < r2.shapeId {
                    // end
                    var event = SensorEndTouchEvent()
                    event.sensorShapeId = sensorId
                    event.visitorShapeId = ShapeID(
                        index1: r1.shapeId + 1, world0: worldId, generation: r1.generation
                    )
                    sensorEndEvents.pending.append(event)
                    index1 += 1
                } else {
                    // begin
                    var event = SensorBeginTouchEvent()
                    event.sensorShapeId = sensorId
                    event.visitorShapeId = ShapeID(
                        index1: r2.shapeId + 1, world0: worldId, generation: r2.generation
                    )
                    sensorBeginEvents.append(event)
                    index2 += 1
                }
            }

            while index1 < count1 {
                let r1 = refs1[index1]
                var event = SensorEndTouchEvent()
                event.sensorShapeId = sensorId
                event.visitorShapeId = ShapeID(
                    index1: r1.shapeId + 1, world0: worldId, generation: r1.generation
                )
                sensorEndEvents.pending.append(event)
                index1 += 1
            }

            while index2 < count2 {
                let r2 = refs2[index2]
                var event = SensorBeginTouchEvent()
                event.sensorShapeId = sensorId
                event.visitorShapeId = ShapeID(
                    index1: r2.shapeId + 1, world0: worldId, generation: r2.generation
                )
                sensorBeginEvents.append(event)
                index2 += 1
            }
        }
    }

    /// Create sensor storage for a sensor shape.
    func createSensor(_ shapeId: Int32) {
        var sensor = Sensor()
        sensor.shapeId = shapeId
        sensors.append(sensor)

        var shape = shapes[Int(shapeId)]
        shape.sensorIndex = Int32(sensors.count - 1)
        shapes[Int(shapeId)] = shape
    }

    /// Destroy the sensor for a shape, emitting end touch events for its overlaps.
    func destroySensor(_ shapeId: Int32) {
        var shape = shapes[Int(shapeId)]
        if shape.sensorIndex == B3.nullIndex {
            return
        }

        let sensorIndex = Int(shape.sensorIndex)
        let sensor = sensors[sensorIndex]

        for ref in sensor.overlaps2 {
            var event = SensorEndTouchEvent()
            event.sensorShapeId = ShapeID(
                index1: shape.id + 1, world0: worldId, generation: shape.generation
            )
            event.visitorShapeId = ShapeID(
                index1: ref.shapeId + 1, world0: worldId, generation: ref.generation
            )
            sensorEndEvents.pending.append(event)
        }

        let last = sensors.removeLast()
        if sensorIndex < sensors.count { sensors[sensorIndex] = last }

        // Fixup moved sensor
        if sensorIndex < sensors.count {
            let movedSensor = sensors[sensorIndex]
            var otherSensorShape = shapes[Int(movedSensor.shapeId)]
            otherSensorShape.sensorIndex = Int32(sensorIndex)
            shapes[Int(movedSensor.shapeId)] = otherSensorShape
        }

        shape.sensorIndex = B3.nullIndex
        shapes[Int(shapeId)] = shape
    }
}

extension World {
    /// Move CCD sensor hits into their sensors before the overlap pass.
    func acceptSensorHits() {
        for hit in taskContexts[0].sensorHits {
            let sensorShape = shapes[Int(hit.sensorId)]
            if sensorShape.sensorIndex == B3.nullIndex {
                continue
            }

            let visitorShape = shapes[Int(hit.visitorId)]
            var visitor = SensorVisitor()
            visitor.shapeId = hit.visitorId
            visitor.generation = visitorShape.generation
            sensors[Int(sensorShape.sensorIndex)].hits.append(visitor)
        }

        taskContexts[0].sensorHits.removeAll()
    }

    /// Buffered sensor begin/end touch events from the last step.
    public func sensorEvents() -> (begin: [SensorBeginTouchEvent], end: [SensorEndTouchEvent]) {
        (sensorBeginEvents, sensorEndEvents.published)
    }
}
