@inline(__always) func ctz32(_ block: UInt32) -> UInt32 { UInt32(block.trailingZeroBitCount) }

@inline(__always) func clz32(_ value: UInt32) -> UInt32 { UInt32(value.leadingZeroBitCount) }

@inline(__always) func ctz64(_ block: UInt64) -> UInt32 { UInt32(block.trailingZeroBitCount) }

@inline(__always) func popCount64(_ block: UInt64) -> Int32 { Int32(block.nonzeroBitCount) }

@inline(__always) func isPowerOf2(_ x: Int32) -> Bool { (x & (x &- 1)) == 0 }

func boundingPowerOf2(_ x: Int32) -> Int32 {
    if x <= 1 {
        return 1
    }

    return 32 - Int32(clz32(UInt32(x) - 1))
}

func roundUpPowerOf2(_ x: Int32) -> Int32 {
    if x <= 1 {
        return 1
    }

    return 1 << (32 - Int32(clz32(UInt32(x) - 1)))
}

func lowerPowerOf2Exponent(_ x: Int32) -> Int32 {
    b3Assert(x > 0)
    let clz = Int32(clz32(UInt32(x)))

    // Position of most significant bit = floor(log2(M))
    return 31 - clz
}

/// Swap two values (B3_SWAP).
@inline(__always) func b3Swap<T>(_ x: inout T, _ y: inout T) { swap(&x, &y) }
