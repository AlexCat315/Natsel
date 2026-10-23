private let blockExponent: Int32 = 8
private let blockSize: Int32 = 1 << blockExponent
private let blockAlignment: Int32 = 16

struct Block: BitwiseCopyable {
    var memory: UnsafeMutableRawPointer?
}

/// Fixed-size element allocator with stable addresses and an intrusive free list.
struct BlockAllocator: BitwiseCopyable {
    var blocks = DynamicArray<Block>()
    var freeList: UnsafeMutableRawPointer?
    var elementSize: Int32 = 0
    var nextIndex: Int32 = 0
    var allocationCount: Int32 = 0

    init() {}

    /// Element must be large enough to hold a pointer. The element size is rounded up to the
    /// block alignment.
    init(elementSize: Int32, initialCount: Int32) {
        b3Assert(elementSize >= Int32(MemoryLayout<UnsafeMutableRawPointer>.size))

        // The free list stores a pointer inside each element, so the stride must keep every
        // element aligned.
        self.elementSize = ((elementSize - 1) | (blockAlignment - 1)) + 1

        if initialCount > 0 {
            let count = (initialCount + blockSize - 1) >> blockExponent
            blocks.resize(count)

            var i: Int32 = 0
            while i < blocks.count {
                blocks[i].memory = b3Alloc(Int(blockSize * self.elementSize))
                i += 1
            }
        }
    }

    mutating func destroy() {
        var i: Int32 = 0
        while i < blocks.count {
            b3Free(blocks[i].memory, Int(blockSize * elementSize))
            i += 1
        }

        blocks.destroy()
    }

    /// Returns one element of elementSize contiguous bytes. Address is stable until freed.
    mutating func allocate() -> UnsafeMutableRawPointer {
        allocationCount += 1

        // Pop from free list first
        if let element = freeList {
            freeList = element.load(as: UnsafeMutableRawPointer?.self)
            return element
        }

        let index = nextIndex
        nextIndex += 1

        let requiredBlockCount = (index >> blockExponent) + 1

        if requiredBlockCount > blocks.count {
            let oldCount = blocks.count
            blocks.resize(requiredBlockCount)

            var i = oldCount
            while i < requiredBlockCount {
                blocks[i].memory = b3Alloc(Int(blockSize * elementSize))
                i += 1
            }
        }

        let blockIndex = index >> blockExponent
        let blockOffset = index & (blockSize - 1)

        return blocks[blockIndex].memory! + Int(blockOffset * elementSize)
    }

    mutating func free(_ element: UnsafeMutableRawPointer) {
        b3Assert(allocationCount > 0)
        b3Assert(Int(bitPattern: element) & Int(blockAlignment - 1) == 0)

        allocationCount -= 1

        // Push onto free list
        element.storeBytes(of: freeList, as: UnsafeMutableRawPointer?.self)
        freeList = element
    }
}
