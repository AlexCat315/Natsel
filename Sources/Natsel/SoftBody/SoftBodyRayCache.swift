/// A local-frame face BVH, built only when a queried soft body has enough faces to benefit.
/// It retains bounds and face indices, never copies or retains vertex storage.
final class SoftBodyRayCache {
    let generation: UInt16
    let revision: UInt64
    let tree: DynamicTree

    init(_ record: SoftBodyRecord) {
        generation = record.generation
        revision = record.spatialRevision
        tree = DynamicTree(proxyCapacity: Int32(record.settings.faces.count))
        for (index, face) in record.settings.faces.enumerated() {
            let a = record.positions[Int(face.vertex.x)]
            let b = record.positions[Int(face.vertex.y)]
            let c = record.positions[Int(face.vertex.z)]
            let bounds = AABB(minVec3(a, minVec3(b, c)), maxVec3(a, maxVec3(b, c)))
            _ = tree.createProxy(bounds, categoryBits: .max, userData: UInt64(index))
        }
        _ = tree.rebuild(fullBuild: true)
    }
}
