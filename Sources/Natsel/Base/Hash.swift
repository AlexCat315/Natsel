// rapidhash V3 (compact, fast) - https://github.com/Nicoshev/rapidhash
// Copyright (C) 2025 Nicolas De Carli, BSD 2-Clause License.

private let rapidSecret: InlineArray<8, UInt64> = [
    0x2d35_8dcc_aa6c_78a5,
    0x8bb8_4b93_962e_acc9,
    0x4b33_a62e_d433_d4a3,
    0x4d5a_2da5_1de1_aa47,
    0xa076_1d64_78bd_642f,
    0xe703_7ed1_a0b4_28db,
    0x90ed_1765_281c_388c,
    0xaaaa_aaaa_aaaa_aaaa,
]

/// 64*64 -> 128 bit multiply. A receives the low bits and B the high bits.
@inline(__always)
private func rapidMum(_ a: inout UInt64, _ b: inout UInt64) {
    let r = a.multipliedFullWidth(by: b)
    a = r.low
    b = r.high
}

@inline(__always)
private func rapidMix(_ a: UInt64, _ b: UInt64) -> UInt64 {
    var a = a
    var b = b
    rapidMum(&a, &b)
    return a ^ b
}

@inline(__always)
private func rapidRead64(_ p: UnsafePointer<UInt8>) -> UInt64 {
    UInt64(littleEndian: UnsafeRawPointer(p).loadUnaligned(as: UInt64.self))
}

@inline(__always)
private func rapidRead32(_ p: UnsafePointer<UInt8>) -> UInt64 {
    UInt64(UInt32(littleEndian: UnsafeRawPointer(p).loadUnaligned(as: UInt32.self)))
}

func rapidhash(_ key: UnsafePointer<UInt8>, _ len: Int) -> UInt64 {
    let secret = rapidSecret
    var p = key
    var seed: UInt64 = 0
    seed ^= rapidMix(seed ^ secret[2], secret[1])
    var a: UInt64 = 0
    var b: UInt64 = 0
    var i = len
    if len <= 16 {
        if len >= 4 {
            seed ^= UInt64(len)
            if len >= 8 {
                let plast = p + len - 8
                a = rapidRead64(p)
                b = rapidRead64(plast)
            } else {
                let plast = p + len - 4
                a = rapidRead32(p)
                b = rapidRead32(plast)
            }
        } else if len > 0 {
            a = (UInt64(p[0]) << 45) | UInt64(p[len - 1])
            b = UInt64(p[len >> 1])
        } else {
            a = 0
            b = 0
        }
    } else {
        if len > 112 {
            var see1 = seed, see2 = seed
            var see3 = seed, see4 = seed
            var see5 = seed, see6 = seed
            repeat {
                seed = rapidMix(rapidRead64(p) ^ secret[0], rapidRead64(p + 8) ^ seed)
                see1 = rapidMix(rapidRead64(p + 16) ^ secret[1], rapidRead64(p + 24) ^ see1)
                see2 = rapidMix(rapidRead64(p + 32) ^ secret[2], rapidRead64(p + 40) ^ see2)
                see3 = rapidMix(rapidRead64(p + 48) ^ secret[3], rapidRead64(p + 56) ^ see3)
                see4 = rapidMix(rapidRead64(p + 64) ^ secret[4], rapidRead64(p + 72) ^ see4)
                see5 = rapidMix(rapidRead64(p + 80) ^ secret[5], rapidRead64(p + 88) ^ see5)
                see6 = rapidMix(rapidRead64(p + 96) ^ secret[6], rapidRead64(p + 104) ^ see6)
                p += 112
                i -= 112
            } while i > 112
            seed ^= see1
            see2 ^= see3
            see4 ^= see5
            seed ^= see6
            see2 ^= see4
            seed ^= see2
        }
        if i > 16 {
            seed = rapidMix(rapidRead64(p) ^ secret[2], rapidRead64(p + 8) ^ seed)
            if i > 32 {
                seed = rapidMix(rapidRead64(p + 16) ^ secret[2], rapidRead64(p + 24) ^ seed)
                if i > 48 {
                    seed = rapidMix(rapidRead64(p + 32) ^ secret[1], rapidRead64(p + 40) ^ seed)
                    if i > 64 {
                        seed = rapidMix(rapidRead64(p + 48) ^ secret[1], rapidRead64(p + 56) ^ seed)
                        if i > 80 {
                            seed = rapidMix(rapidRead64(p + 64) ^ secret[2], rapidRead64(p + 72) ^ seed)
                            if i > 96 {
                                seed = rapidMix(rapidRead64(p + 80) ^ secret[1], rapidRead64(p + 88) ^ seed)
                            }
                        }
                    }
                }
            }
        }
        a = rapidRead64(p + i - 16) ^ UInt64(i)
        b = rapidRead64(p + i - 8)
    }
    a ^= secret[1]
    b ^= seed
    rapidMum(&a, &b)
    return rapidMix(a ^ secret[7], b ^ secret[1] ^ UInt64(i))
}

func hash64NonZero(_ bytes: UnsafePointer<UInt8>, _ n: Int32) -> UInt64 {
    if n <= 0 {
        return 1
    }

    let h = rapidhash(bytes, Int(n))
    return h == 0 ? 1 : h
}

/// djb2 hash for determinism testing, folded 8 bytes per iteration to shorten the dependency
/// chain. Words are read little-endian so the value is identical across endianness.
public func hash(_ hash: UInt32, _ data: UnsafeRawPointer, _ count: Int32) -> UInt32 {
    let bytes = data.assumingMemoryBound(to: UInt8.self)
    let count = Int(count)
    var result = hash
    var i = 0

    while i + 8 <= count {
        let word = UInt64(littleEndian: data.loadUnaligned(fromByteOffset: i, as: UInt64.self))
        result = (result &<< 5) &+ result &+ UInt32(truncatingIfNeeded: word)
        result = (result &<< 5) &+ result &+ UInt32(truncatingIfNeeded: word >> 32)
        i += 8
    }

    while i < count {
        result = (result &<< 5) &+ result &+ UInt32(bytes[i])
        i += 1
    }

    return result
}

/// The initial value for `hash(_:_:_:)`.
public let hashInit: UInt32 = B3.hashInit
