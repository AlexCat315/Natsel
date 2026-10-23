import Testing
@testable import Natsel

@Suite("Container storage")
struct StorageTests {
    @Test("bit set growth, union, clearing, and reset preserve block boundaries")
    func bitSetLifecycle() {
        var a = BitSet(bitCapacity: 64)
        var b = BitSet(bitCapacity: 64)
        defer { a.destroy(); b.destroy() }
        a.setBitCountAndClear(64)
        a.setBit(63)
        a.setBitGrow(128)
        #expect(a.blockCount == 3)
        #expect(a.getBit(63))
        #expect(a.getBit(128))
        #expect(!a.getBit(64))
        #expect(a.countSetBits() == 2)
        b.setBitCountAndClear(129)
        b.setBit(64)
        b.setBit(128)
        a.inPlaceUnion(b)
        #expect(a.countSetBits() == 3)
        a.clearBit(128)
        #expect(a.countSetBits() == 2)
        a.setBitCountAndClear(1)
        #expect(a.countSetBits() == 0)
        a.setBitCountAndClear(129)
        #expect(a.countSetBits() == 0)
        a.destroy()
        #expect(a.bits == nil)
        #expect(a.blockCount == 0)
        #expect(a.blockCapacity == 0)
    }

    @Test("hash set deletion repairs wraparound probe chains")
    func probeChains() {
        // All keys start in the final bucket, forcing probing to wrap through zero.
        let keys = (UInt64(1)...10_000).filter { keyHash($0) & 15 == 15 }.prefix(7)
        var set = HashSet(capacity: 16)
        defer { set.destroy() }
        for key in keys { do { let observed = !set.addKey(key); #expect(observed) } }
        for key in keys { do { let observed = set.addKey(key); #expect(observed) } }
        #expect(set.count == 7)
        for key in keys {
            do { let observed = set.removeKey(key); #expect(observed) }
            #expect(!set.containsKey(key))
            do { let observed = !set.removeKey(key); #expect(observed) }
            for remaining in keys where remaining > key {
                #expect(set.containsHashedKey(remaining, keyHash(remaining)))
            }
        }
        #expect(set.count == 0)
        for key: UInt64 in 1...1024 { do { let observed = !set.addKey(key); #expect(observed) } }
        #expect(set.capacity == 2048)
        for key: UInt64 in 1...1024 { #expect(set.containsKey(key)) }
        set.clear()
        #expect(set.count == 0)
        for key: UInt64 in 1...1024 { #expect(!set.containsKey(key)) }
    }

    @Test("ID recycling preserves the C last-in-first-out order")
    func idRecycling() {
        var pool = IDPool.create()
        defer { pool.destroy() }
        for id: Int32 in 0..<100 { do { let observed = pool.alloc() == id; #expect(observed) } }
        for id: Int32 in [17, 3, 99, 42] { pool.free(id) }
        #expect(pool.idCount == 96)
        #expect(pool.idCapacity == 100)
        for id: Int32 in [42, 99, 3, 17, 100] { do { let observed = pool.alloc() == id; #expect(observed) } }
        pool.destroy()
        do { let observed = pool.alloc() == 0; #expect(observed) }
    }

    @Test("dynamic array growth and swap removal preserve C indices")
    func dynamicArray() {
        var array = DynamicArray<Int32>()
        defer { array.destroy() }
        for value: Int32 in 0..<100 { array.push(value) }
        #expect(array.count == 100)
        #expect(array.capacity == 128)
        do { let observed = array.removeSwap(3) == 99; #expect(observed) }
        #expect(array[3] == 99)
        do { let observed = array.removeSwap(98) == B3.nullIndex; #expect(observed) }
        do { let observed = array.pop() == 97; #expect(observed) }
        let tail: [Int32] = [500, 501, 502]
        tail.withUnsafeBufferPointer { array.append($0.baseAddress!, Int32($0.count)) }
        do { let observed = array.pop() == 502; #expect(observed) }
        do { let observed = array.pop() == 501; #expect(observed) }
        do { let observed = array.pop() == 500; #expect(observed) }
        array.clear()
        #expect(array.count == 0)
        #expect(array.capacity == 128)
    }
}
