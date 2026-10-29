

// Shape creation and proxy management.

/// Content-keyed, reference counted hull store. Hulls with identical bytes share one instance.
final class HullDatabase {
    private struct Entry {
        var hull: Hull
        var refCount: Int
    }

    private var entries: [UInt64: [Entry]] = [:]

    /// Add a hull, cloning it on a miss. Returns the owned shared hull.
    func add(_ src: Hull) -> Hull? {
        let key = src.hash
        if var bucket = entries[key] {
            for i in 0..<bucket.count {
                if compareHullData(bucket[i].hull, src) {
                    bucket[i].refCount += 1
                    entries[key] = bucket
                    return bucket[i].hull
                }
            }
        }

        guard let clone = src.clone() else {
            return nil
        }

        entries[key, default: []].append(Entry(hull: clone, refCount: 1))
        return clone
    }

    /// Like `add` but takes ownership of a heap hull: inserted directly on a miss.
    func addOwned(_ owned: Hull) -> Hull {
        let key = owned.hash
        if var bucket = entries[key] {
            for i in 0..<bucket.count {
                if compareHullData(bucket[i].hull, owned) {
                    bucket[i].refCount += 1
                    entries[key] = bucket
                    return bucket[i].hull
                }
            }
        }

        entries[key, default: []].append(Entry(hull: owned, refCount: 1))
        return owned
    }

    /// Release a reference. The owned copy is freed when the count reaches zero.
    func remove(_ hull: Hull) {
        let key = hull.hash
        guard var bucket = entries[key] else {
            return
        }

        for i in 0..<bucket.count {
            if bucket[i].hull === hull {
                bucket[i].refCount -= 1
                if bucket[i].refCount <= 0 {
                    bucket.remove(at: i)
                }
                if bucket.isEmpty {
                    entries.removeValue(forKey: key)
                } else {
                    entries[key] = bucket
                }
                return
            }
        }
    }

    var count: Int {
        entries.values.reduce(0) { $0 + $1.count }
    }

    /// Opaque copy of the reference counts, for world state snapshots. Hulls are shared, not copied.
    var snapshot: Any { entries }

    func restore(_ snapshot: Any) {
        if let saved = snapshot as? [UInt64: [Entry]] { entries = saved }
    }
}

extension World {
    /// The per-shape AABB margin, a fraction of the shape extent capped at the global maximum.
    func computeShapeMargin(_ shape: Shape) -> Float {
        var margin: Float = 0

        switch shape.geometry {
        case .sphere(let sphere):
            margin = sphere.radius

        case .capsule(let capsule):
            margin = 0.5 * distance(capsule.center2, capsule.center1) + capsule.radius

        case .hull(let hull):
            let points = hull.points!
            var maxExtentSqr: Float = 0
            for i in 0..<Int(hull.vertexCount) {
                let distSqr = distanceSquared(points[i], hull.center)
                maxExtentSqr = maxFloat(maxExtentSqr, distSqr)
            }
            margin = maxExtentSqr.squareRoot()

        case .mesh, .heightField, .compound:
            // Non-convex shapes use the maximum margin.
            return B3.maxAABBMargin
        }

        return minFloat(B3.maxAABBMargin, B3.aabbMarginFraction * margin)
    }

    /// Compute the shape AABB and its fat AABB.
    func updateShapeAABBs(_ shape: inout Shape, _ fatAABB: inout AABB, _ transform: WorldTransform, _ proxyType: BodyType) {
        // Compute a bounding box with a speculative margin
        let speculativeDistance = B3.speculativeDistance
        let aabbMargin = shape.aabbMargin

        let aabb = computeFatShapeAABB(shape, transform, speculativeDistance)
        shape.aabb = aabb

        // Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
        let margin = proxyType == .staticBody ? speculativeDistance : aabbMargin
        fatAABB = aabbInflate(aabb, margin)
    }

    /// Create a shape's broad-phase proxy.
    func createShapeProxy(_ shape: inout Shape, _ type: BodyType, _ transform: WorldTransform, _ forcePairCreation: Bool) {
        b3Assert(shape.proxyKey == B3.nullIndex)

        var fatAABB = AABB()
        updateShapeAABBs(&shape, &fatAABB, transform, type)
        fatAABBs[Int(shape.id)] = fatAABB

        // Create proxies in the broad-phase.
        shape.proxyKey = broadPhase.createProxy(
            fatAABB, type: type, categoryBits: shape.filter.categoryBits, shapeIndex: shape.id,
            forcePairCreation: forcePairCreation
        )
        b3Assert(proxyTypeOf(shape.proxyKey).rawValue < BodyType.count)
    }

    // MARK: Creation

    private func createShapeInternal(
        _ bodyId: Int32,
        _ bodyTransform: WorldTransform,
        _ def: ShapeDef,
        _ geometry: ShapeGeometry,
        _ shapeType: ShapeType,
        _ shapeTransform: Transform,
        _ scale: Vec3,
        _ haveShapeTransform: Bool
    ) -> Int32? {
        let shapeId = shapeIdPool.alloc()

        if Int(shapeId) == shapes.count {
            shapes.append(Shape())
            fatAABBs.append(AABB())
        } else {
            b3Assert(shapes[Int(shapeId)].id == B3.nullIndex)
        }

        b3Assert(fatAABBs.count == shapes.count)

        var shape = shapes[Int(shapeId)]
        shape.geometry = geometry
        shape.type = shapeType

        switch shapeType {
        case .hullShape:
            if case .hull(let hull) = geometry {
                if haveShapeTransform {
                    // The transform and non-uniform scale are baked into fresh data, then shared.
                    guard let baked = cloneAndTransformHull(hull, shapeTransform, scale) else {
                        shapeIdPool.free(shapeId)
                        var failed = shapes[Int(shapeId)]
                        failed.id = B3.nullIndex
                        shapes[Int(shapeId)] = failed
                        return nil
                    }

                    let shared = hullDatabase.addOwned(baked)
                    shape.hull = shared
                } else {
                    if let shared = hullDatabase.add(hull) {
                        shape.hull = shared
                    }
                }
            }

        default:
            break
        }

        var body = bodies[Int(bodyId)]

        shape.id = shapeId
        shape.bodyId = body.id
        shape.density = def.density
        shape.explosionScale = def.explosionScale
        shape.filter = def.filter
        shape.userData = def.userData
        shape.userShape = nil
        shape.flags = 0
        shape.flags |= def.enableSensorEvents ? ShapeFlags.enableSensorEvents.rawValue : 0
        shape.flags |= def.enableContactEvents ? ShapeFlags.enableContactEvents.rawValue : 0
        shape.flags |= def.enableCustomFiltering ? ShapeFlags.enableCustomFiltering.rawValue : 0
        shape.flags |= def.enableHitEvents ? ShapeFlags.enableHitEvents.rawValue : 0
        shape.flags |= def.enablePreSolveEvents ? ShapeFlags.enablePreSolveEvents.rawValue : 0
        shape.flags |= def.enableSpeculativeContact ? ShapeFlags.enableSpeculative.rawValue : 0
        shape.proxyKey = B3.nullIndex
        shape.localCentroid = getShapeCentroid(shape)
        shape.aabbMargin = computeShapeMargin(shape)
        shape.aabb = AABB()
        fatAABBs[Int(shapeId)] = AABB()
        shape.nameId = names.add(def.name)
        shape.generation &+= 1

        if case .compound(let compound) = geometry {
            shape.materials = compound.materials
            compoundShapeCount += 1
        } else if def.materials.count > 1 {
            // Per triangle materials need a heap array.
            shape.materials = def.materials
        } else {
            // The common case is one material, stored inline with no allocation.
            shape.materials = [def.materials.first ?? def.baseMaterial]
        }

        shapes[Int(shapeId)] = shape

        if body.setIndex != SetType.disabledSet.rawValue {
            let proxyType = body.type
            let forcePairCreation = def.invokeContactCreation && shape.type != .compoundShape
            var mutableShape = shapes[Int(shapeId)]
            createShapeProxy(&mutableShape, proxyType, bodyTransform, forcePairCreation)
            shapes[Int(shapeId)] = mutableShape
            shape = mutableShape
        }

        // Add to shape doubly linked list
        if body.headShapeId != B3.nullIndex {
            var headShape = shapes[Int(body.headShapeId)]
            headShape.prevShapeId = shapeId
            shapes[Int(body.headShapeId)] = headShape
        }

        shape.prevShapeId = B3.nullIndex
        shape.nextShapeId = body.headShapeId
        shapes[Int(shapeId)] = shape
        body.headShapeId = shapeId
        body.shapeCount += 1
        bodies[Int(bodyId)] = body

        shape.sensorIndex = B3.nullIndex
        shapes[Int(shapeId)] = shape

        if def.isSensor {
            createSensor(shapeId)
        }

        return shapeId
    }

    private func createShape(
        _ bodyId: BodyID,
        _ def: ShapeDef,
        _ geometry: ShapeGeometry,
        _ shapeType: ShapeType,
        _ transform: Transform,
        _ scale: Vec3,
        _ haveShapeTransform: Bool
    ) -> ShapeID {
        b3Assert(isValidFloat(def.density) && def.density >= 0)

        guard isValidBody(bodyId) else {
            return ShapeID()
        }

        let index = bodyIndex(bodyId)
        var body = bodies[Int(index)]
        if body.type != .staticBody && (shapeType == .compoundShape || shapeType == .heightShape) {
            // Compound and height shapes must be on static bodies.
            return ShapeID()
        }

        let bodyTransform = bodySim(body).transform

        guard
            let shapeId = createShapeInternal(
                index, bodyTransform, def, geometry, shapeType, transform, scale, haveShapeTransform
            )
        else {
            return ShapeID()
        }

        body = bodies[Int(index)]
        if def.updateBodyMass {
            updateBodyMassData(index)
        } else if (body.flags & BodyFlags.dirtyMass.rawValue) == 0 {
            body.flags |= BodyFlags.dirtyMass.rawValue
            bodies[Int(index)] = body
            syncBodyFlags(index)
        }

        let shape = shapes[Int(shapeId)]
        return ShapeID(index1: shape.id + 1, world0: bodyId.world0, generation: shape.generation)
    }

    /// Create a sphere shape.
    public func createSphereShape(_ bodyId: BodyID, _ def: ShapeDef, _ sphere: Sphere) -> ShapeID {
        createShape(bodyId, def, .sphere(sphere), .sphereShape, Transform.identity, Vec3(1, 1, 1), false)
    }

    /// Create a capsule shape. Very short capsules become spheres.
    public func createCapsuleShape(_ bodyId: BodyID, _ def: ShapeDef, _ capsule: Capsule) -> ShapeID {
        b3Assert(isValidFloat(capsule.radius) && capsule.radius >= 0)

        if distance(capsule.center1, capsule.center2) < B3.minCapsuleLength {
            // A capsule that is too short becomes a sphere.
            let sphere = Sphere(center: 0.5 * (capsule.center1 + capsule.center2), radius: capsule.radius)
            return createShape(bodyId, def, .sphere(sphere), .sphereShape, Transform.identity, Vec3(1, 1, 1), false)
        }

        return createShape(bodyId, def, .capsule(capsule), .capsuleShape, Transform.identity, Vec3(1, 1, 1), false)
    }

    /// Create a hull shape.
    public func createHullShape(_ bodyId: BodyID, _ def: ShapeDef, _ hull: Hull) -> ShapeID {
        createShape(bodyId, def, .hull(hull), .hullShape, Transform.identity, Vec3(1, 1, 1), false)
    }

    /// Create a compound on a static body.
    public func createCompoundShape(_ bodyId: BodyID, _ def: ShapeDef, _ compound: Compound) -> ShapeID {
        createShape(bodyId, def, .compound(compound), .compoundShape, .identity, .one, false)
    }

    /// Create a height field on a static body.
    public func createHeightFieldShape(_ bodyId: BodyID, _ def: ShapeDef, _ field: HeightField) -> ShapeID {
        createShape(bodyId, def, .heightField(field), .heightShape, .identity, .one, false)
    }

    /// Create a mesh instance sharing immutable triangle geometry.
    public func createMeshShape(_ bodyId: BodyID, _ def: ShapeDef, _ data: TriangleMesh, scale: Vec3 = .one) -> ShapeID {
        createShape(bodyId, def, .mesh(Mesh(data: data, scale: safeScale(scale))), .meshShape,
                    .identity, scale, true)
    }

    /// Create a hull shape baked with a local transform and scale.
    func createTransformedHullShape(
        _ bodyId: BodyID,
        _ def: ShapeDef,
        _ hull: Hull,
        _ transform: Transform,
        _ scale: Vec3
    ) -> ShapeID {
        createShape(bodyId, def, .hull(hull), .hullShape, transform, scale, true)
    }

    /// Get a validated shape.
    func shape(_ id: ShapeID) -> Shape {
        b3Assert(shapeIsValid(id))
        return shapes[Int(id.index1 - 1)]
    }

    func shapeIsValid(_ id: ShapeID) -> Bool {
        if id.world0 != worldId {
            return false
        }
        if id.index1 < 1 || shapes.count < Int(id.index1) {
            return false
        }
        let shape = shapes[Int(id.index1 - 1)]
        if shape.id == B3.nullIndex {
            return false
        }
        return shape.generation == id.generation
    }

    func makeShapeID(_ shapeId: Int32) -> ShapeID {
        ShapeID(index1: shapeId + 1, world0: worldId, generation: shapes[Int(shapeId)].generation)
    }
}

// MARK: - Destruction

extension World {
    /// Destroy a shape on a body. This does not need to be called when destroying a body.
    func destroyShapeInternal(_ shapeId: Int32, _ wakeBodies: Bool) {
        var shape = shapes[Int(shapeId)]
        let bodyIndex = shape.bodyId
        var body = bodies[Int(bodyIndex)]

        if shape.type == .compoundShape {
            b3Assert(compoundShapeCount > 0)
            compoundShapeCount -= 1
        }

        // Remove the shape from the body's doubly linked list.
        if shape.prevShapeId != B3.nullIndex {
            var prevShape = shapes[Int(shape.prevShapeId)]
            prevShape.nextShapeId = shape.nextShapeId
            shapes[Int(shape.prevShapeId)] = prevShape
        }

        if shape.nextShapeId != B3.nullIndex {
            var nextShape = shapes[Int(shape.nextShapeId)]
            nextShape.prevShapeId = shape.prevShapeId
            shapes[Int(shape.nextShapeId)] = nextShape
        }

        if shapeId == body.headShapeId {
            body.headShapeId = shape.nextShapeId
        }

        body.shapeCount -= 1

        // Remove from broad-phase.
        destroyShapeProxy(&shape, broadPhase)

        // Commit shape-list changes before contact destruction can update the body.
        bodies[Int(bodyIndex)] = body
        var contactKey = body.headContactKey
        while contactKey != B3.nullIndex {
            let contactId = contactKey >> 1
            let edgeIndex = Int(contactKey & 1)
            let contact = contacts[Int(contactId)]
            contactKey = contact.edges[edgeIndex].nextKey
            if contact.shapeIdA == shapeId || contact.shapeIdB == shapeId {
                destroyContact(contactId, wakeBodies)
            }
        }
        body = bodies[Int(bodyIndex)]

        // Destroy the sensor and its touch events.
        destroySensor(shapeId)

        // Destroy every shape member from the allocation.
        destroyShapeAllocations(self, &shape)

        // Return shape to free list.
        shapeIdPool.free(shapeId)
        shape.id = B3.nullIndex
        shape.proxyKey = B3.nullIndex
        shapes[Int(shapeId)] = shape

        bodies[Int(bodyIndex)] = body
    }

    /// Destroy a shape.
    func destroyShape(_ id: ShapeID, updateBodyMass: Bool) {
        b3Assert(shapeIsValid(id))

        let shapeId = id.index1 - 1
        let shape = shapes[Int(shapeId)]

        // need to wake bodies because this might be a static body
        let wakeBodies = true

        let bodyIndex = shape.bodyId
        destroyShapeInternal(shapeId, wakeBodies)

        if updateBodyMass {
            updateBodyMassData(bodyIndex)
        }
    }
}
