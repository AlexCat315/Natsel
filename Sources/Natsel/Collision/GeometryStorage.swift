/// Shared lifetime for a geometry allocation and its immutable embedded geometry views.
private final class GeometryAllocation {
    let base: UnsafeMutableRawPointer
    let byteCount: Int32

    init(base: UnsafeMutableRawPointer, byteCount: Int32) {
        self.base = base
        self.byteCount = byteCount
    }
    deinit { b3Free(base, Int(byteCount)) }
}

struct GeometryStorage {
    private let allocation: GeometryAllocation
    private let offset: Int
    let byteCount: Int32
    var base: UnsafeMutableRawPointer { allocation.base + offset }

    init(takingOwnershipOf base: UnsafeMutableRawPointer, byteCount: Int32) {
        allocation = GeometryAllocation(base: base, byteCount: byteCount)
        offset = 0
        self.byteCount = byteCount
    }

    private init(allocation: GeometryAllocation, offset: Int, byteCount: Int32) {
        self.allocation = allocation
        self.offset = offset
        self.byteCount = byteCount
    }

    func slice(offset: Int, byteCount: Int32) -> GeometryStorage {
        precondition(offset >= 0 && byteCount >= 0 && offset + Int(byteCount) <= Int(self.byteCount))
        return GeometryStorage(allocation: allocation, offset: self.offset + offset, byteCount: byteCount)
    }
}
