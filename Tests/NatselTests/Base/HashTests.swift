import Testing
@testable import Natsel

@Suite("Base hash C parity")
struct HashTests {
    @Test("rapidhash matches C across short, tail, and block boundaries")
    func rapidHashParity() {
        let bytes = (0..<4098).map { UInt8(truncatingIfNeeded: $0 * 131 + 17) }
        bytes.withUnsafeBufferPointer { buffer in
            let start = buffer.baseAddress!
            for (length, expected) in rapidHashReference.enumerated() {
                #expect(rapidhash(start, length) == expected, "length \(length)")
                #expect(hash64NonZero(start, Int32(length)) == (length == 0 ? 1 : expected))
            }
            for (length, expected) in rapidHashLongReference {
                #expect(rapidhash(start, length) == expected, "length \(length)")
            }
            // Identical content at an unaligned address must hash identically.
            var unaligned = [UInt8(0)] + Array(bytes.prefix(4097))
            unaligned.withUnsafeMutableBufferPointer { shifted in
                for (length, expected) in rapidHashLongReference {
                    #expect(rapidhash(shifted.baseAddress! + 1, length) == expected)
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
