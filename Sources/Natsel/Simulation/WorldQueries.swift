

// World queries (physics_world.c): overlap AABB, overlap shape, ray cast, shape cast.

/// Result of the closest shape cast.
public struct ShapeCastResult: Sendable {
    public var shapeId = ShapeID()
    public var point = Pos()
    public var normal = Vec3()
    public var fraction: Float = 0
    public var userMaterialId: UInt64 = 0
    public var triangleIndex: Int32 = 0
    public var childIndex: Int32 = 0
    public var hit = false

    public init() {}
}

extension World {
    /// Fast body transform lookup without locking.
    public func bodyTransformQuick(_ bodyId: Int32) -> WorldTransform {
        let body = bodies[Int(bodyId)]
        return solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)].transform
    }

    @inline(__always)
    private func queryShapeTransform(_ shape: Shape, _ origin: Pos) -> Transform {
        toRelativeTransform(bodyTransformQuick(shape.bodyId), origin)
    }

    /// Query the world for all shapes whose AABB overlaps the given box.
    public func overlapAABB(
        _ aabb: AABB,
        _ filter: QueryFilter = .defaultFilter,
        _ callback: (ShapeID) -> Bool
    ) -> TreeStats {
        var treeStats = TreeStats()

        for i in 0..<Int(BodyType.count) {
            let treeResult = broadPhase.trees[i].query(
                aabb, maskBits: filter.maskBits, requireAllBits: false
            ) { _, userData in
                let shapeId = Int32(Int64(bitPattern: userData))
                let shape = self.shapes[Int(shapeId)]

                if shouldQueryCollide(shape.filter, filter) == false {
                    return true
                }

                let id = ShapeID(
                    index1: shape.id + 1, world0: self.worldId, generation: shape.generation
                )
                return callback(id)
            }

            treeStats.nodeVisits += treeResult.nodeVisits
            treeStats.leafVisits += treeResult.leafVisits
        }

        return treeStats
    }

    /// Query the world for all shapes that overlap the given shape proxy placed at `origin`.
    public func overlapShape(
        _ origin: Pos,
        _ proxy: ShapeProxy,
        _ filter: QueryFilter = .defaultFilter,
        _ callback: (ShapeID) -> Bool
    ) -> TreeStats {
        var treeStats = TreeStats()

        let aabb = aabbOffset(computeProxyAABB(proxy), origin)

        for i in 0..<Int(BodyType.count) {
            let treeResult = broadPhase.trees[i].query(
                aabb, maskBits: filter.maskBits, requireAllBits: false
            ) { _, userData in
                let shapeId = Int32(Int64(bitPattern: userData))
                let shape = self.shapes[Int(shapeId)]

                if shouldQueryCollide(shape.filter, filter) == false {
                    return true
                }

                let transform = self.queryShapeTransform(shape, origin)
                if Natsel.overlapShape(shape, transform, proxy) == false {
                    return true
                }

                let id = ShapeID(
                    index1: shape.id + 1, world0: self.worldId, generation: shape.generation
                )
                return callback(id)
            }

            treeStats.nodeVisits += treeResult.nodeVisits
            treeStats.leafVisits += treeResult.leafVisits
        }

        return treeStats
    }

    /// Cast a ray through the world. The callback returns the new fraction, or -1 to skip the shape.
    public func castRay(
        _ origin: Pos,
        _ translation: Vec3,
        _ filter: QueryFilter = .defaultFilter,
        _ callback: (ShapeID, Pos, Vec3, Float, UInt64, Int32, Int32) -> Float
    ) -> TreeStats {
        var treeStats = TreeStats()
        var fraction: Float = 1
        var input = BoxCastInput()
        input.box = offsetAABB(AABB(Vec3(), Vec3()), origin)
        input.translation = translation
        input.maxFraction = 1

        for i in 0..<Int(BodyType.count) {
            let treeResult = broadPhase.trees[i].boxCast(
                input, maskBits: filter.maskBits, requireAllBits: false
            ) { queryInput, _, userData in
                let shapeId = Int32(Int64(bitPattern: userData))
                let shape = self.shapes[Int(shapeId)]

                if shouldQueryCollide(shape.filter, filter) == false {
                    return queryInput.maxFraction
                }

                let transform = self.queryShapeTransform(shape, origin)

                var localInput = RayCastInput()
                localInput.translation = translation
                localInput.maxFraction = queryInput.maxFraction
                let output = rayCastShape(shape, transform, localInput)

                if output.hit {
                    b3Assert(output.fraction <= queryInput.maxFraction)

                    let id = ShapeID(
                        index1: shape.id + 1, world0: self.worldId, generation: shape.generation
                    )
                    let point = offsetPos(origin, output.point)
                    let materialIndex = clampInt(output.materialIndex, 0, shape.materialCount - 1)
                    let userMaterialId = shape.material(materialIndex).userMaterialId

                    let newFraction = callback(
                        id, point, output.normal, output.fraction, userMaterialId,
                        output.triangleIndex, output.childIndex
                    )

                    // The user may return -1 to skip this shape
                    if 0 <= newFraction && newFraction <= 1 {
                        fraction = newFraction
                    }

                    return newFraction
                }

                return queryInput.maxFraction
            }

            treeStats.nodeVisits += treeResult.nodeVisits
            treeStats.leafVisits += treeResult.leafVisits

            if fraction == 0 {
                break
            }

            input.maxFraction = fraction
        }

        return treeStats
    }

    /// Cast a ray and return the closest hit.
    public func castRayClosest(
        _ origin: Pos,
        _ translation: Vec3,
        _ filter: QueryFilter = .defaultFilter
    ) -> RayResult {
        var result = RayResult()

        _ = castRay(origin, translation, filter) {
            shapeId, point, normal, fraction, userMaterialId, triangleIndex, childIndex in

            // Ignore initial overlap
            if fraction == 0 {
                return -1
            }

            result.shapeId = shapeId
            result.point = point
            result.normal = normal
            result.fraction = fraction
            result.userMaterialId = userMaterialId
            result.triangleIndex = triangleIndex
            result.childIndex = childIndex
            result.hit = true
            return fraction
        }

        return result
    }

    /// Cast a shape through the world.
    public func shapeCast(
        _ origin: Pos,
        _ input: ShapeCastInput,
        _ filter: QueryFilter = .defaultFilter,
        _ callback: (ShapeID, Pos, Vec3, Float, UInt64, Int32, Int32) -> Float
    ) -> TreeStats {
        var treeStats = TreeStats()
        var fraction: Float = 1

        let aabb = aabbOffset(computeProxyAABB(input.proxy), origin)

        for i in 0..<Int(BodyType.count) {
            var boxCast = BoxCastInput()
            boxCast.box = aabb
            boxCast.translation = input.translation
            boxCast.maxFraction = fraction

            let treeResult = broadPhase.trees[i].boxCast(
                boxCast, maskBits: filter.maskBits, requireAllBits: false
            ) { boxInput, _, userData in
                let shapeId = Int32(Int64(bitPattern: userData))
                let shape = self.shapes[Int(shapeId)]

                if shouldQueryCollide(shape.filter, filter) == false {
                    return boxInput.maxFraction
                }

                var localInput = input
                localInput.maxFraction = boxInput.maxFraction

                let transform = self.queryShapeTransform(shape, origin)
                let output = shapeCastShape(shape, transform, localInput)

                if output.hit {
                    let id = ShapeID(
                        index1: shape.id + 1, world0: self.worldId, generation: shape.generation
                    )
                    let materialIndex = clampInt(output.materialIndex, 0, shape.materialCount - 1)
                    let userMaterialId = shape.material(materialIndex).userMaterialId

                    let newFraction = callback(
                        id, offsetPos(origin, output.point), output.normal, output.fraction,
                        userMaterialId, output.triangleIndex, output.childIndex
                    )

                    // The user may return -1 to skip this shape
                    if 0 <= newFraction && newFraction <= 1 {
                        fraction = newFraction
                    }

                    return newFraction
                }

                return boxInput.maxFraction
            }

            treeStats.nodeVisits += treeResult.nodeVisits
            treeStats.leafVisits += treeResult.leafVisits

            if fraction == 0 {
                break
            }
        }

        return treeStats
    }

    /// Cast a shape and return the closest hit.
    public func shapeCastClosest(
        _ origin: Pos,
        _ input: ShapeCastInput,
        _ filter: QueryFilter = .defaultFilter
    ) -> ShapeCastResult {
        var result = ShapeCastResult()

        _ = shapeCast(origin, input, filter) {
            shapeId, point, normal, fraction, userMaterialId, triangleIndex, childIndex in

            // Ignore initial overlap
            if fraction == 0 {
                return -1
            }

            result.shapeId = shapeId
            result.point = point
            result.normal = normal
            result.fraction = fraction
            result.userMaterialId = userMaterialId
            result.triangleIndex = triangleIndex
            result.childIndex = childIndex
            result.hit = true
            return fraction
        }

        return result
    }
}
