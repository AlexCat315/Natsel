/// Bit set provides fast operations on large arrays of bits.
struct BitSet: BitwiseCopyable {
    var bits: UnsafeMutablePointer<UInt64>?
    var blockCapacity: UInt32 = 0
    var blockCount: UInt32 = 0

    init() {}

    init(bitCapacity: UInt32) {
        blockCapacity = (bitCapacity + UInt32(MemoryLayout<UInt64>.size) * 8 - 1) / (UInt32(MemoryLayout<UInt64>.size) * 8)
        blockCount = 0
        bits = b3AllocTyped(UInt64.self, Int32(blockCapacity))
        bits?.initialize(repeating: 0, count: Int(blockCapacity))
    }

    mutating func destroy() {
        b3FreeTyped(bits, Int32(blockCapacity))
        blockCapacity = 0
        blockCount = 0
        bits = nil
    }

    mutating func setBitCountAndClear(_ bitCount: UInt32) {
        let blockCount = (bitCount + UInt32(MemoryLayout<UInt64>.size) * 8 - 1) / (UInt32(MemoryLayout<UInt64>.size) * 8)
        if blockCapacity < blockCount {
            destroy()
            let newBitCapacity = bitCount + (bitCount >> 1)
            self = BitSet(bitCapacity: newBitCapacity)
        }

        self.blockCount = blockCount
        bits?.update(repeating: 0, count: Int(blockCount))
    }

    mutating func grow(_ blockCount: UInt32) {
        b3Assert(blockCount > self.blockCount)
        if blockCount > blockCapacity {
            let oldCapacity = blockCapacity
            blockCapacity = blockCount + blockCount / 2
            let newBits = b3AllocTyped(UInt64.self, Int32(blockCapacity))!
            newBits.initialize(repeating: 0, count: Int(blockCapacity))
            b3Assert(bits != nil)
            newBits.update(from: bits!, count: Int(oldCapacity))
            b3FreeTyped(bits, Int32(oldCapacity))
            bits = newBits
        }

        self.blockCount = blockCount
    }

    func inPlaceUnion(_ setB: BitSet) {
        b3Assert(blockCount == setB.blockCount)
        let blockCount = self.blockCount
        var i: UInt32 = 0
        while i < blockCount {
            bits![Int(i)] |= setB.bits![Int(i)]
            i += 1
        }
    }

    @inline(__always)
    func setBit(_ bitIndex: UInt32) {
        let blockIndex = bitIndex / 64
        b3Assert(blockIndex < blockCount)
        bits![Int(blockIndex)] |= UInt64(1) << (bitIndex % 64)
    }

    @inline(__always)
    mutating func setBitGrow(_ bitIndex: UInt32) {
        let blockIndex = bitIndex / 64
        if blockIndex >= blockCount {
            grow(blockIndex + 1)
        }
        bits![Int(blockIndex)] |= UInt64(1) << (bitIndex % 64)
    }

    @inline(__always)
    func clearBit(_ bitIndex: UInt32) {
        let blockIndex = bitIndex / 64
        if blockIndex >= blockCount {
            return
        }
        bits![Int(blockIndex)] &= ~(UInt64(1) << (bitIndex % 64))
    }

    @inline(__always)
    func getBit(_ bitIndex: UInt32) -> Bool {
        let blockIndex = bitIndex / 64
        if blockIndex >= blockCount {
            return false
        }
        return (bits![Int(blockIndex)] & (UInt64(1) << (bitIndex % 64))) != 0
    }

    var byteCount: Int32 {
        Int32(blockCapacity) * Int32(MemoryLayout<UInt64>.size)
    }
}
