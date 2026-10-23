// The broad-phase computes proxy pairs. This does not persist pairs; it reports potentially new
// pairs for the client to consume.

/// Proxy type is stored in the low two bits of the proxy key.
@inline(__always)
func proxyTypeOf(_ key: Int32) -> BodyType {
    BodyType(rawValue: key & 3)!
}

@inline(__always)
func proxyIdOfKey(_ key: Int32) -> Int32 {
    key >> 2
}

@inline(__always)
func proxyKey(_ id: Int32, _ type: BodyType) -> Int32 {
    (id << 2) | type.rawValue
}

/// A broad-phase with one dynamic tree per body type.
final class BroadPhase {
    /// One tree for each body type.
    let trees: [DynamicTree]

    /// The moved siblings gathered from the dynamic body tree.
    var movedSiblings: [Int32] = []

    /// Tracks shape pairs that have a contact.
    var pairSet: HashSet

    init(capacity: Capacity) {
        pairSet = HashSet(capacity: maxInt(32, 2 * capacity.contactCount))

        let staticCapacity = maxInt(16, capacity.staticShapeCount)
        let kinematicCapacity: Int32 = 16
        let dynamicCapacity = maxInt(16, capacity.dynamicShapeCount)

        trees = [
            DynamicTree(proxyCapacity: staticCapacity),
            DynamicTree(proxyCapacity: kinematicCapacity),
            DynamicTree(proxyCapacity: dynamicCapacity),
        ]
    }

    /// Create a proxy and return its key.
    @discardableResult
    func createProxy(
        _ aabb: AABB,
        type proxyBodyType: BodyType,
        categoryBits: UInt64,
        shapeIndex: Int32,
        forcePairCreation: Bool
    ) -> Int32 {
        b3Assert(proxyBodyType.rawValue >= 0 && proxyBodyType.rawValue < BodyType.count)

        let mark = proxyBodyType != .staticBody || forcePairCreation

        let proxyId = trees[Int(proxyBodyType.rawValue)].createProxyInternal(
            aabb, categoryBits: categoryBits, userData: UInt64(bitPattern: Int64(shapeIndex)),
            markMoved: mark
        )
        return proxyKey(proxyId, proxyBodyType)
    }

    /// Destroy a proxy.
    func destroyProxy(_ key: Int32) {
        let type = proxyTypeOf(key)
        let proxyId = proxyIdOfKey(key)

        b3Assert(type.rawValue >= 0 && type.rawValue < BodyType.count)
        trees[Int(type.rawValue)].destroyProxy(proxyId)
    }

    /// Move a proxy.
    func moveProxy(_ key: Int32, _ aabb: AABB) {
        let type = proxyTypeOf(key)
        let proxyId = proxyIdOfKey(key)

        trees[Int(type.rawValue)].moveProxyInternal(proxyId, aabb, true)
    }

    /// Get the shape index stored in a proxy.
    func getShapeIndex(_ key: Int32) -> Int32 {
        let type = proxyTypeOf(key)
        let proxyId = proxyIdOfKey(key)

        return Int32(truncatingIfNeeded: trees[Int(type.rawValue)].getUserData(proxyId))
    }

    /// Do two proxies overlap?
    func testOverlap(_ keyA: Int32, _ keyB: Int32) -> Bool {
        let typeA = Int(proxyTypeOf(keyA).rawValue)
        let proxyA = proxyIdOfKey(keyA)
        let typeB = Int(proxyTypeOf(keyB).rawValue)
        let proxyB = proxyIdOfKey(keyB)

        let aabbA = trees[typeA].getAABB(proxyA)
        let aabbB = trees[typeB].getAABB(proxyB)
        return aabbOverlaps(aabbA, aabbB)
    }

    /// Mark a proxy as moved. Serial use case.
    func markProxyMovedSerial(_ key: Int32) {
        let type = proxyTypeOf(key)
        let proxyId = proxyIdOfKey(key)
        trees[Int(type.rawValue)].markProxyMovedSerial(proxyId)
    }

    /// Mark a proxy as moved with a new AABB.
    func markProxyMoved(_ key: Int32, _ aabb: AABB) {
        let type = proxyTypeOf(key)
        let proxyId = proxyIdOfKey(key)
        trees[Int(type.rawValue)].markProxyMoved(proxyId, aabb)
    }

    /// Validate the dynamic and kinematic trees.
    func validate() {
        trees[Int(BodyType.dynamicBody.rawValue)].validate()
        trees[Int(BodyType.kinematicBody.rawValue)].validate()
    }

    /// Validate that no proxy has a moved flag.
    func validateNoMoved() {
        for j in 0..<Int(BodyType.count) {
            trees[j].validateNoMoved()
        }
    }

    /// Does any tree need a rebuild?
    var needsRebuild: Bool {
        trees[Int(BodyType.staticBody.rawValue)].needsRebuild
            || trees[Int(BodyType.kinematicBody.rawValue)].needsRebuild
            || trees[Int(BodyType.dynamicBody.rawValue)].needsRebuild
    }
}
