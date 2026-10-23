func bakeCompound(_ builder: CompoundBuilder) -> GeometryStorage {
    let layout = CompoundOffsets(builder)
    let base = b3AllocZero(layout.byteCount)!
    let header = base.bindMemory(to: CompoundData.self, capacity: 1)
    header.initialize(to: CompoundData())
    header.pointee.version = B3.compoundVersion
    header.pointee.byteCount = Int32(layout.byteCount)
    header.pointee.bounds = builder.tree.rootBounds
    header.pointee.treeHeight = builder.tree.height
    header.pointee.nodeOffset = Int32(layout.nodes)
    header.pointee.nodeCount = builder.tree.nodeEnd
    header.pointee.materialOffset = Int32(layout.materials)
    header.pointee.materialCount = Int32(builder.materials.values.count)
    header.pointee.capsuleOffset = Int32(layout.capsules)
    header.pointee.capsuleCount = Int32(builder.capsuleInstances.count)
    header.pointee.hullOffset = Int32(layout.hulls)
    header.pointee.hullCount = Int32(builder.hullInstances.count)
    header.pointee.sharedHullCount = Int32(builder.hulls.values.count)
    header.pointee.meshOffset = Int32(layout.meshes)
    header.pointee.meshCount = Int32(builder.meshInstances.count)
    header.pointee.sharedMeshCount = Int32(builder.meshes.values.count)
    header.pointee.sphereOffset = Int32(layout.spheres)
    header.pointee.sphereCount = Int32(builder.sphereInstances.count)
    func store<T: BitwiseCopyable>(_ values: [T], offset: Int) {
        if values.isEmpty { return }
        values.withUnsafeBufferPointer { buffer in
            (base + offset).bindMemory(to: T.self, capacity: values.count).initialize(from: buffer.baseAddress!, count: values.count)
        }
    }
    store(Array(builder.tree.nodes.prefix(Int(builder.tree.nodeEnd))), offset: layout.nodes)
    store(builder.materials.values, offset: layout.materials)
    store(builder.capsuleInstances, offset: layout.capsules)
    var hullInstances = builder.hullInstances
    for i in hullInstances.indices { hullInstances[i].hullOffset = UInt32(layout.hullBlobs[Int(hullInstances[i].hullOffset)]) }
    store(hullInstances, offset: layout.hulls)
    for (i, hull) in builder.hulls.values.enumerated() {
        (base + layout.hullBlobs[i]).copyMemory(from: hull.base, byteCount: Int(hull.byteCount))
    }
    var meshInstances = builder.meshInstances
    for i in meshInstances.indices {
        meshInstances[i].meshOffset = UInt32(layout.meshBlobs[Int(meshInstances[i].meshOffset)])
        meshInstances[i].materialOffset = UInt32(layout.meshMaterials) + meshInstances[i].materialOffset * 2
    }
    store(meshInstances, offset: layout.meshes)
    store(builder.meshMaterialMap, offset: layout.meshMaterials)
    for (i, mesh) in builder.meshes.values.enumerated() {
        (base + layout.meshBlobs[i]).copyMemory(from: mesh.base, byteCount: Int(mesh.byteCount))
    }
    store(builder.sphereInstances, offset: layout.spheres)
    return GeometryStorage(takingOwnershipOf: base, byteCount: Int32(layout.byteCount))
}
