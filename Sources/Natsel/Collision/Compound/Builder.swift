struct CompoundBuilder {
    let tree: DynamicTree
    var materials = CompoundInternTable<SurfaceMaterial>()
    var hulls = CompoundInternTable<Hull>()
    var meshes = CompoundInternTable<TriangleMesh>()
    var capsuleInstances: [PackedCapsule] = []
    var hullInstances: [PackedHullInstance] = []
    var meshInstances: [PackedMeshInstance] = []
    var sphereInstances: [PackedSphere] = []
    var meshMaterialMap: [UInt16] = []

    init?(_ definition: CompoundDef) {
        let count = definition.capsules.count + definition.hulls.count + definition.meshes.count + definition.spheres.count
        guard count > 0, Int64(count) < B3.maxChildShapes else { return nil }
        tree = DynamicTree(proxyCapacity: Int32(count))
        var childIndex: UInt64 = 0
        for item in definition.capsules {
            guard let materialIndex = internMaterial(item.material) else { return nil }
            var instance = PackedCapsule()
            instance.capsule = item.capsule
            instance.materialIndex = materialIndex
            capsuleInstances.append(instance)
            _ = tree.createProxy(computeCapsuleAABB(item.capsule, .identity), categoryBits: .max, userData: childIndex)
            childIndex += 1
        }
        for item in definition.hulls {
            guard let hull = item.hull, let materialIndex = internMaterial(item.material) else { return nil }
            _ = tree.createProxy(computeHullAABB(hull, item.transform), categoryBits: .max, userData: childIndex)
            childIndex += 1
            var instance = PackedHullInstance()
            instance.transform = item.transform
            instance.materialIndex = materialIndex
            instance.hullOffset = UInt32(hulls.insert(hull, bytes: geometryBytes(hull.base, hull.byteCount)))
            hullInstances.append(instance)
        }
        for item in definition.meshes {
            guard let mesh = item.meshData, !item.materials.isEmpty,
                  item.materials.count == Int(mesh.header.materialCount) else { return nil }
            _ = tree.createProxy(computeMeshAABB(mesh, item.transform, scale: item.scale), categoryBits: .max, userData: childIndex)
            childIndex += 1
            let start = meshMaterialMap.count
            for material in item.materials {
                guard let index = internMaterial(material) else { return nil }
                meshMaterialMap.append(index)
            }
            var instance = PackedMeshInstance()
            instance.transform = item.transform
            instance.scale = item.scale
            instance.meshOffset = UInt32(meshes.insert(mesh, bytes: geometryBytes(mesh.base, mesh.byteCount)))
            instance.materialOffset = UInt32(start)
            instance.materialCount = UInt32(item.materials.count)
            meshInstances.append(instance)
        }
        for item in definition.spheres {
            guard let materialIndex = internMaterial(item.material) else { return nil }
            var instance = PackedSphere()
            instance.sphere = item.sphere
            instance.materialIndex = materialIndex
            sphereInstances.append(instance)
            _ = tree.createProxy(computeSphereAABB(item.sphere, .identity), categoryBits: .max, userData: childIndex)
            childIndex += 1
        }
        _ = tree.rebuild(fullBuild: true)
    }

    private mutating func internMaterial(_ material: SurfaceMaterial) -> UInt16? {
        var material = material
        let bytes = withUnsafeBytes(of: &material) { Array($0) }
        let index = materials.insert(material, bytes: bytes)
        guard materials.values.count <= Int(UInt16.max) else { return nil }
        return UInt16(index)
    }
}

struct CompoundOffsets {
    var nodes: Int
    var materials: Int
    var capsules: Int
    var hulls: Int
    var hullBlobs: [Int] = []
    var meshes: Int
    var meshMaterials: Int
    var meshBlobs: [Int] = []
    var spheres: Int
    var byteCount: Int

    init(_ builder: CompoundBuilder) {
        func aligned(_ count: Int) -> Int { (count + 7) & ~7 }
        nodes = aligned(MemoryLayout<CompoundData>.stride)
        materials = nodes + aligned(Int(builder.tree.nodeEnd) * MemoryLayout<TreeNode>.stride)
        capsules = materials + aligned(builder.materials.values.count * MemoryLayout<SurfaceMaterial>.stride)
        hulls = capsules + aligned(builder.capsuleInstances.count * MemoryLayout<PackedCapsule>.stride)
        var offset = hulls + aligned(builder.hullInstances.count * MemoryLayout<PackedHullInstance>.stride)
        for hull in builder.hulls.values { hullBlobs.append(offset); offset += aligned(Int(hull.byteCount)) }
        meshes = offset
        meshMaterials = meshes + aligned(builder.meshInstances.count * MemoryLayout<PackedMeshInstance>.stride)
        offset = meshMaterials + aligned(builder.meshMaterialMap.count * MemoryLayout<UInt16>.stride)
        for mesh in builder.meshes.values { meshBlobs.append(offset); offset += aligned(Int(mesh.byteCount)) }
        spheres = offset
        byteCount = spheres + aligned(builder.sphereInstances.count * MemoryLayout<PackedSphere>.stride)
    }
}
