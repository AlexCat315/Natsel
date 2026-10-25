@testable import Natsel

struct CollisionReferenceDigest {
    var value: UInt64 = 14695981039346656037
    mutating func add(_ value: Pos) { add(Float(value.x)); add(Float(value.y)); add(Float(value.z)) }
    mutating func add(_ word: UInt32) {
        for byte in 0..<4 {
            value ^= UInt64((word >> (8 * byte)) & 255)
            value = value &* 1099511628211
        }
    }
    mutating func add(_ scalar: Float) { add(scalar.bitPattern) }
    mutating func add(_ vector: Vec3) { add(vector.x); add(vector.y); add(vector.z) }
}


extension CollisionReferenceDigest {
    mutating func add(_ output: CastOutput) {
        add(output.normal); add(output.point); add(output.fraction)
        add(UInt32(bitPattern: output.iterations)); add(UInt32(bitPattern: output.triangleIndex))
        add(UInt32(bitPattern: output.childIndex)); add(UInt32(bitPattern: output.materialIndex))
        add(output.hit ? UInt32(1) : UInt32(0))
    }
    mutating func add(_ plane: PlaneResult) {
        add(plane.plane.normal); add(plane.plane.offset); add(plane.point)
        add(UInt32(bitPattern: plane.triangleIndex)); add(UInt32(bitPattern: plane.childIndex)); add(UInt32(bitPattern: plane.materialIndex))
    }
}
