extension World {
    func emitCompoundPairs(_ compoundId: Int32, _ otherId: Int32, _ compound: Compound, _ keys: inout [UInt64]) {
        let other = shapes[Int(otherId)]
        let proxy = other.proxyKey
        let bounds = broadPhase.trees[Int(proxyTypeOf(proxy).rawValue)].getAABB(proxyIdOfKey(proxy))
        let transform = toRelativeTransform(bodyTransformQuick(shapes[Int(compoundId)].bodyId), .zero)
        let local = aabbTransform(invertTransform(transform), bounds)
        let start = keys.count
        queryCompound(compound, bounds: local) { child in
            let key = shapePairKey(compoundId, otherId, Int32(child))
            if !broadPhase.pairSet.containsKey(key) { keys.append(key) }
            return true
        }
        if keys.count > start && !shouldCreatePair(compoundId, otherId) {
            keys.removeSubrange(start...)
        }
    }
}
