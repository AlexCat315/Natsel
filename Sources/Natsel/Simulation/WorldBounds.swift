extension World {
    /// Tight world bounds without narrowing the absolute coordinates to float.
    public func shapeWorldAABB(_ id: ShapeID) -> WorldAABB {
        let shape = shapes[Int(id.index1 - 1)]
        let transform = bodyTransformQuick(shape.bodyId)
        return WorldAABB(localBounds: computeShapeAABB(shape, Transform(Vec3(), transform.q)), origin: transform.p)
    }

    public func bodyWorldAABB(_ id: BodyID) -> WorldAABB {
        let record = body(id)
        let transform = bodyTransformQuick(bodyIndex(id))
        var localBounds = AABB(Vec3(), Vec3())
        var shapeId = record.headShapeId
        var first = true
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            let bounds = computeShapeAABB(shape, Transform(Vec3(), transform.q))
            localBounds = first ? bounds : aabbUnion(localBounds, bounds)
            first = false
            shapeId = shape.nextShapeId
        }
        return WorldAABB(localBounds: localBounds, origin: transform.p)
    }

    public func softBodyWorldAABB(_ id: SoftBodyID) -> WorldAABB? {
        guard let i = softBodyIndex(id) else { return nil }
        return WorldAABB(localBounds: softBodies[i].bounds, origin: softBodies[i].origin)
    }
}

extension World {
    /// Query precise world bounds, rejecting conservative float-tree false positives.
    public func overlapAABB(
        _ bounds: WorldAABB, _ filter: QueryFilter = .defaultFilter,
        _ callback: (ShapeID) -> Bool
    ) -> TreeStats {
        overlapAABB(bounds.conservativeBounds, filter) { id in
            guard bounds.overlaps(self.shapeWorldAABB(id)) else { return true }
            return callback(id)
        }
    }
}
