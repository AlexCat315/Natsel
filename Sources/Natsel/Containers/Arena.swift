private let maxStackEntries = 32

struct StackEntry: BitwiseCopyable {
    var data: UnsafeMutableRawPointer?
    var size: Int32 = 0
    var usedMalloc: Bool = false
}

/// This is a stack-like arena allocator used for fast per step allocations.
/// You must nest allocate/free pairs. This allocator uses the heap if space is insufficient.
struct ArenaStack: BitwiseCopyable {
    var memory: UnsafeMutableRawPointer?
    var capacity: Int32 = 0
    var index: Int32 = 0

    var allocation: Int32 = 0
    var maxAllocation: Int32 = 0

    var entries = InlineArray<32, StackEntry>(repeating: StackEntry())
    var entryCount: Int32 = 0

    init() {}

    init(capacity: Int32) {
        b3Assert(capacity >= 0)
        self.capacity = capacity
        memory = b3Alloc(Int(capacity))
    }

    mutating func destroy() {
        b3Free(memory, Int(capacity))
    }

    mutating func alloc(_ size: Int32) -> UnsafeMutableRawPointer? {
        if entryCount == maxStackEntries {
            b3Assert(false)
            return nil
        }

        let alignedSize = ((size - 1) | (Int32(B3.alignment) - 1)) + 1

        var entry = StackEntry()
        entry.size = alignedSize
        if index + alignedSize > capacity {
            // fall back to the heap (undesirable)
            entry.data = b3Alloc(Int(alignedSize))
            entry.usedMalloc = true
        } else {
            entry.data = memory! + Int(index)
            entry.usedMalloc = false
            index += alignedSize
        }

        allocation += alignedSize
        if allocation > maxAllocation {
            maxAllocation = allocation
        }

        entries[Int(entryCount)] = entry
        entryCount += 1
        return entry.data
    }

    mutating func free(_ mem: UnsafeMutableRawPointer?) {
        let entryCount = self.entryCount
        b3Assert(entryCount > 0)
        let entry = entries[Int(entryCount - 1)]
        b3Assert(mem == entry.data)
        if entry.usedMalloc {
            b3Free(mem, Int(entry.size))
        } else {
            index -= entry.size
        }
        allocation -= entry.size
        self.entryCount -= 1
    }

    /// Grow the stack based on usage.
    mutating func grow() {
        // Stack must not be in use
        b3Assert(allocation == 0)

        if maxAllocation > capacity {
            b3Free(memory, Int(capacity))
            capacity = maxAllocation + maxAllocation / 2
            memory = b3Alloc(Int(capacity))
        }
    }
}

/// Heap-allocated fallback block tracked when an arena bump overflows.
struct OverflowBlock: BitwiseCopyable {
    var data: UnsafeMutableRawPointer?
    var size: Int32 = 0
}

/// Shared, heap-allocated state co-owned by every copy of an `Arena`.
/// `Arena` is passed by value so its bump pointer auto-restores on function return, but
/// overflow tracking and watermarks must persist across copies.
struct ArenaSharedState: BitwiseCopyable {
    var overflows = DynamicArray<OverflowBlock>()
    /// high water mark of the bump pointer this step
    var maxIndex: Int32 = 0
    /// total bytes in overflow blocks this step
    var overflowBytes: Int32 = 0
    /// all-time peak of (maxIndex + overflowBytes), survives sync
    var peakDemand: Int32 = 0
}

/// 16-byte alignment for SIMD + typical struct alignment.
private let arenaAlignment: Int32 = 16

struct Arena: BitwiseCopyable {
    var memory: UnsafeMutableRawPointer?
    var capacity: Int32 = 0
    var index: Int32 = 0
    var shared: UnsafeMutablePointer<ArenaSharedState>?

    init() {}

    init(capacity: Int32) {
        let c = capacity > 8 ? capacity : 8
        let shared = b3AllocTyped(ArenaSharedState.self, 1)!
        shared.initialize(to: ArenaSharedState())

        memory = b3Alloc(Int(c))
        self.capacity = c
        index = 0
        self.shared = shared
    }

    mutating func destroy() {
        if let shared {
            var i: Int32 = 0
            while i < shared.pointee.overflows.count {
                let block = shared.pointee.overflows[i]
                b3Free(block.data, Int(block.size))
                i += 1
            }
            shared.pointee.overflows.destroy()
            b3FreeTyped(shared, 1)
        }
        b3Free(memory, Int(capacity))
        self = Arena()
    }

    /// Heap-allocate an overflow block, register it in the shared state, return it.
    func overflowAlloc(_ size: Int32) -> UnsafeMutableRawPointer? {
        let shared = self.shared!
        let data = b3Alloc(Int(size))
        shared.pointee.overflows.push(OverflowBlock(data: data, size: size))
        shared.pointee.overflowBytes += size
        return data
    }

    /// Call between simulation steps. Frees this step's overflow blocks and grows the backing
    /// capacity if last step's demand (maxIndex + overflowBytes) exceeded it.
    mutating func sync() {
        let shared = self.shared!

        var i: Int32 = 0
        while i < shared.pointee.overflows.count {
            let block = shared.pointee.overflows[i]
            b3Free(block.data, Int(block.size))
            i += 1
        }
        shared.pointee.overflows.clear()

        let demand = shared.pointee.maxIndex + shared.pointee.overflowBytes
        if demand > shared.pointee.peakDemand {
            shared.pointee.peakDemand = demand
        }
        if demand > capacity {
            b3Free(memory, Int(capacity))
            let newCapacity = demand + demand / 2
            memory = b3Alloc(Int(newCapacity))
            capacity = newCapacity
        }

        index = 0
        shared.pointee.maxIndex = 0
        shared.pointee.overflowBytes = 0
    }

    @inline(__always)
    mutating func bump(_ size: Int32) -> UnsafeMutableRawPointer? {
        if size == 0 {
            return nil
        }

        let aligned = (index + (arenaAlignment - 1)) & ~(arenaAlignment - 1)

        if aligned + size > capacity {
            return overflowAlloc(size)
        }

        index = aligned + size
        if index > shared!.pointee.maxIndex {
            shared!.pointee.maxIndex = index
        }
        return memory! + Int(aligned)
    }

    /// Typed bump allocation of `count` uninitialized elements.
    @inline(__always)
    mutating func bump<T: BitwiseCopyable>(_ type: T.Type, _ count: Int32) -> UnsafeMutablePointer<T>? {
        bump(count * Int32(MemoryLayout<T>.stride))?.bindMemory(to: T.self, capacity: Int(count))
    }
}
