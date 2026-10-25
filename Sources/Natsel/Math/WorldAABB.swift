/// Precise world bounds. `AABB` remains the compact, conservative broad-phase representation.
public struct WorldAABB: Sendable, Equatable, BitwiseCopyable {
    public var lowerBound: Pos
    public var upperBound: Pos

    public init(_ lowerBound: Pos, _ upperBound: Pos) {
        self.lowerBound = lowerBound
        self.upperBound = upperBound
    }

    public init(localBounds: AABB, origin: Pos) {
        lowerBound = offsetPos(origin, localBounds.lowerBound)
        upperBound = offsetPos(origin, localBounds.upperBound)
    }

    /// Outward rounding preserves all candidates when querying the compact spatial tree.
    public var conservativeBounds: AABB {
        AABB(Vec3(roundDownFloat(lowerBound.x), roundDownFloat(lowerBound.y), roundDownFloat(lowerBound.z)),
             Vec3(roundUpFloat(upperBound.x), roundUpFloat(upperBound.y), roundUpFloat(upperBound.z)))
    }

    public func contains(_ point: Pos) -> Bool {
        lowerBound.x <= point.x && point.x <= upperBound.x
            && lowerBound.y <= point.y && point.y <= upperBound.y
            && lowerBound.z <= point.z && point.z <= upperBound.z
    }

    public func overlaps(_ other: WorldAABB) -> Bool {
        lowerBound.x <= other.upperBound.x && other.lowerBound.x <= upperBound.x
            && lowerBound.y <= other.upperBound.y && other.lowerBound.y <= upperBound.y
            && lowerBound.z <= other.upperBound.z && other.lowerBound.z <= upperBound.z
    }
}

