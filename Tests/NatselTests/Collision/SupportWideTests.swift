import Testing
@testable import Natsel

@Suite("SIMD support queries")
struct SupportWideTests {
    @Test("support values and embedded vertex indices match the scalar oracle", arguments: [1, 3, 4, 8, 12, 32, 64, 128])
    func support(_ count: Int) {
        var seed: UInt32 = 12345
        func random() -> Float {
            seed = seed &* 1664525 &+ 1013904223
            return Float(Int(seed % 2001) - 1000) / 64
        }
        var x = (0..<count).map { _ in random() }
        var y = (0..<count).map { _ in random() }
        var z = (0..<count).map { _ in random() }
        let mask: UInt32 = (1 << UInt32(hullBitCount)) - 1
        for pass in 0..<64 {
            let normal = pass == 0 ? Vec3() : Vec3(random(), random(), random())
            if pass == 32 {
                x = Array(repeating: x[0], count: count)
                y = Array(repeating: y[0], count: count)
                z = Array(repeating: z[0], count: count)
            }
            let bias: Float = 10000
            var minimum = Float.infinity
            for i in 0..<count {
                let d = normal.x * x[i] + normal.y * y[i] + normal.z * z[i]
                let value = bias - d
                let augmented = Float(bitPattern: (value.bitPattern & ~mask) | (UInt32(i) & mask))
                if augmented < minimum { minimum = augmented }
            }
            let index = Int(minimum.bitPattern & mask)
            let expected = normal.x * x[index] + normal.y * y[index] + normal.z * z[index]
            x.withUnsafeBufferPointer { bx in
                y.withUnsafeBufferPointer { by in
                    z.withUnsafeBufferPointer { bz in
                        let actual = getSupportWide(normal, bx.baseAddress!, by.baseAddress!, bz.baseAddress!, Int32(count), bias)
                        #expect(actual.vertexIndex == Int32(index))
                        #expect(actual.support.bitPattern == expected.bitPattern)
                    }
                }
            }
        }
    }
}
