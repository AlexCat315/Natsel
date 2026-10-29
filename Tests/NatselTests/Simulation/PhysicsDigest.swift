@testable import Natsel

struct PhysicsDigest {
    var value: UInt64 = 14695981039346656037
    mutating func add(_ value: Pos) {
        for scalar in [value.x, value.y, value.z] {
            add(UInt32(truncatingIfNeeded: scalar.bitPattern)); add(UInt32(truncatingIfNeeded: scalar.bitPattern >> 32))
        }
    }
    mutating func add(_ word: UInt32) {
        for byte in 0..<4 {
            value ^= UInt64((word >> (8 * byte)) & 255)
            value = value &* 1099511628211
        }
    }
    mutating func add(_ value: Float) { add(value.bitPattern) }
    mutating func add(_ value: Vec3) { add(value.x); add(value.y); add(value.z) }
    mutating func addContacts(_ world: World) {
        var count: UInt32 = 0
        for contact in world.contacts where contact.contactId != B3.nullIndex {
            count += 1
            add(contact.flags); add(UInt32(contact.manifoldCount))
            add(contact.friction); add(contact.restitution); add(contact.rollingResistance); add(contact.tangentVelocity)
            for manifold in contact.manifolds {
                add(manifold.normal); add(UInt32(manifold.pointCount))
                add(manifold.frictionImpulse); add(manifold.rollingImpulse); add(manifold.twistImpulse)
                for i in 0..<Int(manifold.pointCount) {
                    let point = manifold.points[i]
                    add(point.anchorA); add(point.anchorB); add(point.separation)
                    add(point.normalImpulse); add(point.totalNormalImpulse)
                    add(point.featureId); add(UInt32(bitPattern: point.triangleIndex)); add(point.persisted ? UInt32(1) : UInt32(0))
                }
            }
        }
        add(count)
    }
}
