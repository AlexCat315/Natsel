

extension World {
    public func collideMover(_ origin: Pos, _ mover: Capsule, _ filter: QueryFilter = .defaultFilter,
                             _ callback: (ShapeID, [PlaneResult]) -> Bool) {
        let radius = Vec3(mover.radius, mover.radius, mover.radius)
        let relative = AABB(minVec3(mover.center1, mover.center2) - radius,
                            maxVec3(mover.center1, mover.center2) + radius)
        let bounds = aabbOffset(relative, origin)
        for tree in broadPhase.trees {
            _ = tree.query(bounds, maskBits: filter.maskBits, requireAllBits: false) { _, data in
                let shape = shapes[Int(data)]
                if !shouldQueryCollide(shape.filter, filter) { return true }
                let transform = toRelativeTransform(bodyTransformQuick(shape.bodyId), origin)
                let planes = Natsel.collideMover(shape, transform, mover, capacity: 64)
                if planes.isEmpty { return true }
                let id = ShapeID(index1: shape.id + 1, world0: worldId, generation: shape.generation)
                return callback(id, planes)
            }
        }
    }

    public func castMover(_ origin: Pos, _ mover: Capsule, _ translation: Vec3,
                          _ filter: QueryFilter = .defaultFilter,
                          shouldCollide: ((ShapeID) -> Bool)? = nil) -> Float {
        let centers = [mover.center1, mover.center2]
        return centers.withUnsafeBufferPointer { points in
            var shapeInput = ShapeCastInput()
            shapeInput.proxy = ShapeProxy(points: points.baseAddress, count: 2, radius: mover.radius)
            shapeInput.translation = translation
            shapeInput.maxFraction = 1
            shapeInput.canEncroach = mover.radius > 0
            var input = BoxCastInput()
            input.box = aabbOffset(makeAABB(points.baseAddress!, 2, mover.radius), origin)
            input.translation = translation
            input.maxFraction = 1
            var fraction: Float = 1
            for tree in broadPhase.trees {
                _ = tree.boxCast(input, maskBits: filter.maskBits, requireAllBits: false) { advancing, _, data in
                    let shape = shapes[Int(data)]
                    if !shouldQueryCollide(shape.filter, filter) { return fraction }
                    let id = ShapeID(index1: shape.id + 1, world0: worldId, generation: shape.generation)
                    if let shouldCollide, !shouldCollide(id) { return fraction }
                    var local = shapeInput
                    local.maxFraction = advancing.maxFraction
                    let transform = toRelativeTransform(bodyTransformQuick(shape.bodyId), origin)
                    let output = shapeCastShape(shape, transform, local)
                    if output.fraction == 0 { return fraction }
                    fraction = output.fraction
                    return fraction
                }
                if fraction == 0 { break }
                input.maxFraction = fraction
            }
            return fraction
        }
    }

    /// Cast a generic query shape through the world. The proxy is expressed in the query frame
    /// relative to `origin`.
    public func castShape(
        _ origin: Pos,
        _ proxy: ShapeProxy,
        _ translation: Vec3,
        _ filter: QueryFilter = .defaultFilter,
        hit: ((ShapeID, Pos, Vec3, Float) -> Float)? = nil
    ) -> Float {
        guard let points = proxy.points, proxy.count > 0 else { return 1 }

        var shapeInput = ShapeCastInput()
        shapeInput.proxy = proxy
        shapeInput.translation = translation
        shapeInput.maxFraction = 1
        shapeInput.canEncroach = proxy.radius > 0

        var input = BoxCastInput()
        input.box = aabbOffset(makeAABB(points, proxy.count, proxy.radius), origin)
        input.translation = translation
        input.maxFraction = 1

        var fraction: Float = 1
        for tree in broadPhase.trees {
            _ = tree.boxCast(input, maskBits: filter.maskBits, requireAllBits: false) { advancing, _, data in
                let shape = shapes[Int(data)]
                if !shouldQueryCollide(shape.filter, filter) { return fraction }
                var local = shapeInput
                local.maxFraction = advancing.maxFraction
                let transform = toRelativeTransform(bodyTransformQuick(shape.bodyId), origin)
                let output = shapeCastShape(shape, transform, local)
                if output.fraction == 0 { return fraction }

                let id = ShapeID(index1: shape.id + 1, world0: worldId, generation: shape.generation)
                if let hit {
                    let point = offsetPos(origin, output.point)
                    let next = hit(id, point, output.normal, output.fraction)
                    fraction = next
                    return next
                }
                fraction = output.fraction
                return fraction
            }
            if fraction == 0 { break }
            input.maxFraction = fraction
        }
        return fraction
    }
}
