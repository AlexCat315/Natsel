import Testing
@testable import Natsel

@Suite("Base hash repeatability")
struct HashTests {
    @Test("rapidhash repeats across aligned and unaligned input boundaries")
    func rapidHashRepeatability() {
        let bytes = (0..<4098).map { UInt8(truncatingIfNeeded: $0 * 131 + 17) }
        let copy = Array(bytes.reversed().reversed())
        let unaligned = [UInt8(0)] + bytes
        let lengths = Array(0...256) + [257, 511, 512, 1023, 1024, 2047, 2048, 4095, 4096, 4097]
        bytes.withUnsafeBufferPointer { original in
            copy.withUnsafeBufferPointer { duplicate in
                unaligned.withUnsafeBufferPointer { shifted in
                    for length in lengths {
                        let expected = rapidhash(original.baseAddress!, length)
                        #expect(rapidhash(duplicate.baseAddress!, length) == expected)
                        #expect(rapidhash(shifted.baseAddress! + 1, length) == expected)
                        #expect(hash64NonZero(original.baseAddress!, Int32(length)) == (length == 0 ? 1 : expected))
                    }
                }
            }
        }
    }

    @Test("folded determinism hash preserves word order, tails, and seed")
    func determinismHash() {
        let bytes: [UInt8] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
        bytes.withUnsafeBytes { buffer in
            for seed: UInt32 in [0, 5381, .max] {
                for count in 0...bytes.count {
                    var expected = seed
                    var offset = 0
                    while offset + 8 <= count {
                        for half in 0..<2 {
                            let j = offset + half * 4
                            let word = UInt32(bytes[j]) | UInt32(bytes[j + 1]) << 8
                                | UInt32(bytes[j + 2]) << 16 | UInt32(bytes[j + 3]) << 24
                            expected = expected &* 33 &+ word
                        }
                        offset += 8
                    }
                    for byte in bytes[offset..<count] { expected = expected &* 33 &+ UInt32(byte) }
                    #expect(hash(seed, buffer.baseAddress!, Int32(count)) == expected)
                }
            }
        }
    }
}
