// Closest-hit queries that skip one body and all sensors, and report the hit shape index and
// material. Vehicles, characters, and soft bodies use these to look at the world around themselves.

/// Closest hit of an internal query.
struct QueryHit {
    var hit = false
    var fraction: Float = 1
    var point = Pos()
    var normal = Vec3()
    var shapeIndex: Int32 = B3.nullIndex
    var materialIndex: Int32 = 0
}

extension World {
    /// True when an internal query should look at this shape.
    @inline(__always)
    func queryAcceptsShape(_ shape: borrowing Shape, _ filter: QueryFilter, _ ignoreBody: Int32) -> Bool {
        shape.bodyId != ignoreBody && shape.sensorIndex == B3.nullIndex
            && shouldQueryCollide(shape.filter, filter)
    }

    /// Closest ray hit from `origin` along `translation`. `accept` may reject a hit by normal.
    func closestRayHit(
        _ origin: Pos, _ translation: Vec3, _ filter: QueryFilter, ignoreBody: Int32,
        accept: (Vec3) -> Bool = { _ in true }
    ) -> QueryHit {
        var result = QueryHit()
        var input = RayCastInput()
        input.origin = toVec3(origin)
        input.translation = translation
        input.maxFraction = 1

        for tree in broadPhase.trees {
            _ = tree.rayCast(input, maskBits: filter.maskBits, requireAllBits: false) {
                queryInput, _, userData in
                let shapeIndex = Int(Int64(bitPattern: userData))
                let shape = self.shapes[shapeIndex]
                guard self.queryAcceptsShape(shape, filter, ignoreBody) else {
                    return queryInput.maxFraction
                }

                var localInput = queryInput
                localInput.origin = Vec3()
                let transform = toRelativeTransform(self.bodyTransformQuick(shape.bodyId), origin)
                let output = rayCastShape(shape, transform, localInput)
                guard output.hit, output.fraction < result.fraction, accept(output.normal) else {
                    return queryInput.maxFraction
                }

                result.hit = true
                result.fraction = output.fraction
                result.point = offsetPos(origin, output.point)
                result.normal = output.normal
                result.shapeIndex = Int32(shapeIndex)
                result.materialIndex = clampInt(output.materialIndex, 0, shape.materialCount - 1)
                return output.fraction
            }
            input.maxFraction = result.fraction
            if result.fraction == 0 { break }
        }
        return result
    }

    /// Closest shape cast hit. The proxy is relative to `origin`. Initially overlapping shapes
    /// (fraction zero) are skipped, as in `shapeCastClosest`.
    func closestShapeCastHit(
        _ origin: Pos, _ proxy: ShapeProxy, _ translation: Vec3, _ filter: QueryFilter,
        ignoreBody: Int32, canEncroach: Bool = false,
        accept: (Vec3) -> Bool = { _ in true }
    ) -> QueryHit {
        var result = QueryHit()
        guard let points = proxy.points, proxy.count > 0 else { return result }

        var shapeInput = ShapeCastInput()
        shapeInput.proxy = proxy
        shapeInput.translation = translation
        shapeInput.maxFraction = 1
        shapeInput.canEncroach = canEncroach

        var boxInput = BoxCastInput()
        boxInput.box = aabbOffset(makeAABB(points, proxy.count, proxy.radius), origin)
        boxInput.translation = translation
        boxInput.maxFraction = 1

        for tree in broadPhase.trees {
            _ = tree.boxCast(boxInput, maskBits: filter.maskBits, requireAllBits: false) {
                advancing, _, userData in
                let shapeIndex = Int(Int64(bitPattern: userData))
                let shape = self.shapes[shapeIndex]
                guard self.queryAcceptsShape(shape, filter, ignoreBody) else {
                    return advancing.maxFraction
                }

                var local = shapeInput
                local.maxFraction = advancing.maxFraction
                let transform = toRelativeTransform(self.bodyTransformQuick(shape.bodyId), origin)
                let output = shapeCastShape(shape, transform, local)
                guard output.hit, output.fraction > 0, output.fraction < result.fraction,
                    accept(output.normal)
                else { return advancing.maxFraction }

                result.hit = true
                result.fraction = output.fraction
                result.point = offsetPos(origin, output.point)
                result.normal = output.normal
                result.shapeIndex = Int32(shapeIndex)
                result.materialIndex = clampInt(output.materialIndex, 0, shape.materialCount - 1)
                return output.fraction
            }
            boxInput.maxFraction = result.fraction
            if result.fraction == 0 { break }
        }
        return result
    }

    /// Collision planes of a mover capsule (relative to `origin`) against the world, skipping one
    /// body and all sensors. Plane normals point toward the mover; offsets are penetration depths.
    func collideMoverIgnoring(
        _ origin: Pos, _ mover: Capsule, _ filter: QueryFilter, ignoreBody: Int32,
        _ callback: (_ shapeIndex: Int32, _ planes: [PlaneResult]) -> Void
    ) {
        let radius = Vec3(mover.radius, mover.radius, mover.radius)
        let relative = AABB(minVec3(mover.center1, mover.center2) - radius, maxVec3(mover.center1, mover.center2) + radius)
        let bounds = aabbOffset(relative, origin)
        var candidates: [Int32] = []
        for tree in broadPhase.trees {
            _ = tree.query(bounds, maskBits: filter.maskBits, requireAllBits: false) { _, data in
                let shapeIndex = Int32(truncatingIfNeeded: Int64(bitPattern: data))
                if self.queryAcceptsShape(self.shapes[Int(shapeIndex)], filter, ignoreBody) {
                    candidates.append(shapeIndex)
                }
                return true
            }
        }
        // Shape order keeps results independent of tree layout.
        candidates.sort()
        for shapeIndex in candidates {
            let shape = shapes[Int(shapeIndex)]
            let transform = toRelativeTransform(bodyTransformQuick(shape.bodyId), origin)
            let planes = Natsel.collideMover(shape, transform, mover, capacity: 64)
            if planes.isEmpty == false { callback(shapeIndex, planes) }
        }
    }

    /// True when the proxy (relative to `origin`) overlaps any accepted shape.
    func overlapsIgnoring(
        _ origin: Pos, _ proxy: ShapeProxy, _ filter: QueryFilter, ignoreBody: Int32,
        accept: (Int32) -> Bool = { _ in true }
    ) -> Bool {
        let bounds = aabbOffset(computeProxyAABB(proxy), origin)
        var found = false
        for tree in broadPhase.trees {
            _ = tree.query(bounds, maskBits: filter.maskBits, requireAllBits: false) { _, data in
                let shapeIndex = Int32(truncatingIfNeeded: Int64(bitPattern: data))
                let shape = self.shapes[Int(shapeIndex)]
                guard self.queryAcceptsShape(shape, filter, ignoreBody), accept(shapeIndex) else { return true }
                let transform = toRelativeTransform(self.bodyTransformQuick(shape.bodyId), origin)
                found = Natsel.overlapShape(shape, transform, proxy)
                return !found
            }
            if found { break }
        }
        return found
    }

    /// World space velocity of a point on a body. Static and sleeping bodies report zero.
    @inline(__always)
    func pointVelocity(_ bodyIndex: Int32, _ point: Pos) -> Vec3 {
        let record = bodies[Int(bodyIndex)]
        guard record.setIndex == SetType.awakeSet.rawValue else { return Vec3() }
        let set = Int(SetType.awakeSet.rawValue)
        let state = solverSets[set].bodyStates[Int(record.localIndex)]
        let center = solverSets[set].bodySims[Int(record.localIndex)].center
        return state.linearVelocity + cross(state.angularVelocity, subPos(point, center))
    }
}
