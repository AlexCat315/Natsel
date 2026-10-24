// Contact metadata is POD. Variable-size data is owned by the world in stable slabs, so
// copying a contact handle never retains arrays or makes its manifold buffer shared.

struct ContactManifolds: RandomAccessCollection, MutableCollection, ExpressibleByArrayLiteral, Equatable {
    typealias Index = Int
    typealias Element = Manifold

    // The overwhelmingly common convex case needs no heap allocation.
    var first = Manifold()
    var extra: [Manifold] = []
    private(set) var count = 0

    init() {}
    init(_ manifold: Manifold) {
        first = manifold
        count = 1
    }
    init(arrayLiteral elements: Manifold...) {
        count = elements.count
        if count == 1 { first = elements[0] } else { extra = elements }
    }

    var startIndex: Int { 0 }
    var endIndex: Int { count }

    @inline(__always)
    subscript(_ index: Int) -> Manifold {
        get {
            precondition(index >= 0 && index < count)
            return count == 1 ? first : extra[index]
        }
        set {
            precondition(index >= 0 && index < count)
            if count == 1 { first = newValue } else { extra[index] = newValue }
        }
    }

    mutating func append(_ value: Manifold) {
        if count == 0 {
            first = value
        } else if count == 1 {
            extra = [first, value]
        } else {
            extra.append(value)
        }
        count += 1
    }

    mutating func removeAll(keepingCapacity: Bool = false) {
        count = 0
        extra.removeAll(keepingCapacity: keepingCapacity)
    }

    @inline(__always)
    mutating func withUnsafeMutableBufferPointer<R>(
        _ body: (inout UnsafeMutableBufferPointer<Manifold>) throws -> R
    ) rethrows -> R {
        if count == 1 {
            return try withUnsafeMutablePointer(to: &first) { pointer in
                var buffer = UnsafeMutableBufferPointer(start: pointer, count: 1)
                return try body(&buffer)
            }
        }
        return try extra.withUnsafeMutableBufferPointer(body)
    }

    @inline(__always)
    func withUnsafeBufferPointer<R>(_ body: (UnsafeBufferPointer<Manifold>) throws -> R) rethrows -> R {
        if count == 1 {
            return try withUnsafePointer(to: first) { pointer in
                try body(UnsafeBufferPointer(start: pointer, count: 1))
            }
        }
        return try extra.withUnsafeBufferPointer(body)
    }

    static func == (a: ContactManifolds, b: ContactManifolds) -> Bool {
        a.count == b.count && a.elementsEqual(b)
    }
}

struct ContactPayload {
    var manifolds = ContactManifolds()
    var meshContact = MeshContactState()
}

/// Reusable buffers for one worker's mesh narrow phase. The C reference bump-allocates the same
/// data from a per-worker arena; keeping it here avoids a fresh allocation per contact.
struct MeshContactScratch {
    var survivors: [MeshSurvivor] = []
    var manifolds: [MeshTriangleManifold] = []
    var points: [LocalManifoldPoint] = []

    /// Per-worker diagnostics. Keeping these off the World objects avoids an exclusivity check
    /// on every hull pair and contact.
    var satCalls: Int32 = 0
    var satCacheHits: Int32 = 0
    var recycledContacts: Int32 = 0

    /// Reused SAT and face-clipping slabs. `withUnsafeTemporaryAllocation` falls back to the heap
    /// above 1 KB, and these are 13 KB and 8 KB, so keeping them here removes a malloc/free from
    /// every hull pair.
    var satScratch: [UInt8] = []
    var faceScratch: [UInt8] = []

    /// Reused mesh-manifold selection and clustering storage.
    var selectedManifolds: [MeshTriangleManifold] = []
    var clusters: [MeshManifoldCluster] = []
    var consumedManifolds: [Bool] = []
    var projectedPoints: [MeshPatchPoint] = []
}

let satScratchByteCount = 25 * 132 * 4
let faceScratchByteCount = 2 * 64 * MemoryLayout<ClipVertex>.stride
    + 64 * MemoryLayout<LocalManifoldPoint>.stride

/// Slabs never move. Contact ids reuse the same initialized payload slot, while the pool
/// releases every non-POD field on world destruction, including slots of destroyed contacts.
final class ContactPayloadPool {
    private static let blockSize = 256
    private var blocks: [UnsafeMutablePointer<ContactPayload>] = []

    func initialize(_ index: Int) -> UnsafeMutablePointer<ContactPayload> {
        let blockIndex = index / Self.blockSize
        while blocks.count <= blockIndex {
            let block = UnsafeMutablePointer<ContactPayload>.allocate(capacity: Self.blockSize)
            block.initialize(repeating: ContactPayload(), count: Self.blockSize)
            blocks.append(block)
        }
        let result = blocks[blockIndex] + index % Self.blockSize
        result.pointee = ContactPayload()
        return result
    }

    deinit {
        for block in blocks {
            block.deinitialize(count: Self.blockSize)
            block.deallocate()
        }
    }
}

/// A borrowed view carries one pointer, so iteration and element access never copy the
/// inline manifold or allocate a Swift coroutine frame. Its owner is ContactPayloadPool.
struct ContactManifoldView: RandomAccessCollection, MutableCollection, Equatable {
    typealias Index = Int
    typealias Element = Manifold
    let payload: UnsafeMutablePointer<ContactPayload>

    var startIndex: Int { 0 }
    var endIndex: Int { count }
    var count: Int { payload.pointee.manifolds.count }

    @inline(__always)
    private var firstPointer: UnsafeMutablePointer<Manifold> {
        let offset =
            MemoryLayout<ContactPayload>.offset(of: \.manifolds)!
            + MemoryLayout<ContactManifolds>.offset(of: \.first)!
        return UnsafeMutableRawPointer(payload).advanced(by: offset).assumingMemoryBound(to: Manifold.self)
    }

    @inline(__always)
    subscript(_ index: Int) -> Manifold {
        unsafeAddress {
            precondition(index >= 0 && index < count)
            if count == 1 { return UnsafePointer(firstPointer) }
            return payload.pointee.manifolds.extra.withUnsafeBufferPointer { $0.baseAddress! + index }
        }
        nonmutating unsafeMutableAddress {
            precondition(index >= 0 && index < count)
            if count == 1 { return firstPointer }
            return payload.pointee.manifolds.extra.withUnsafeMutableBufferPointer { $0.baseAddress! + index }
        }
    }

    func removeAll(keepingCapacity: Bool = false) {
        payload.pointee.manifolds.removeAll(keepingCapacity: keepingCapacity)
    }

    func replace(with storage: ContactManifolds) { payload.pointee.manifolds = storage }

    @inline(__always)
    func withUnsafeMutableBufferPointer<R>(
        _ body: (inout UnsafeMutableBufferPointer<Manifold>) throws -> R
    ) rethrows -> R {
        if count == 1 {
            var buffer = UnsafeMutableBufferPointer(start: firstPointer, count: 1)
            return try body(&buffer)
        }
        return try payload.pointee.manifolds.extra.withUnsafeMutableBufferPointer(body)
    }

    var buffer: UnsafeBufferPointer<Manifold> {
        if count == 1 { return UnsafeBufferPointer(start: firstPointer, count: 1) }
        return payload.pointee.manifolds.extra.withUnsafeBufferPointer { $0 }
    }

    static func == (a: ContactManifoldView, b: ContactManifoldView) -> Bool {
        a.count == b.count && a.elementsEqual(b)
    }
}

extension Contact {
    var manifolds: ContactManifoldView { ContactManifoldView(payload: payload!) }

    var meshContact: MeshContactState {
        unsafeAddress {
            let offset = MemoryLayout<ContactPayload>.offset(of: \.meshContact)!
            return UnsafeRawPointer(payload!).advanced(by: offset).assumingMemoryBound(to: MeshContactState.self)
        }
        nonmutating unsafeMutableAddress {
            let offset = MemoryLayout<ContactPayload>.offset(of: \.meshContact)!
            return UnsafeMutableRawPointer(payload!).advanced(by: offset).assumingMemoryBound(to: MeshContactState.self)
        }
    }

    /// Public query views remain valid until the next world mutation, matching the C API.
    var manifoldBuffer: UnsafeBufferPointer<Manifold> {
        guard payload != nil else { return UnsafeBufferPointer(start: nil, count: 0) }
        return manifolds.buffer
    }
}
