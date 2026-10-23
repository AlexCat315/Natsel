func validateCompoundBytes(_ bytes: [UInt8]) -> Bool {
    guard bytes.count >= MemoryLayout<CompoundData>.stride, bytes.count <= Int(Int32.max) else { return false }
    return bytes.withUnsafeBytes { raw in
        let header = raw.loadUnaligned(as: CompoundData.self)
        guard header.version == B3.compoundVersion, header.byteCount == bytes.count,
              header.nodeOffset >= MemoryLayout<CompoundData>.stride, header.nodeOffset & 7 == 0,
              header.nodeCount >= 2 else { return false }
        func region(_ offset: Int32, _ count: Int32, _ stride: Int) -> Bool {
            offset >= 0 && count >= 0 && Int64(offset) + Int64(count) * Int64(stride) <= bytes.count
        }
        guard region(header.nodeOffset, header.nodeCount, MemoryLayout<TreeNode>.stride),
              region(header.materialOffset, header.materialCount, MemoryLayout<SurfaceMaterial>.stride),
              region(header.capsuleOffset, header.capsuleCount, MemoryLayout<PackedCapsule>.stride),
              region(header.hullOffset, header.hullCount, MemoryLayout<PackedHullInstance>.stride),
              region(header.meshOffset, header.meshCount, MemoryLayout<PackedMeshInstance>.stride),
              region(header.sphereOffset, header.sphereCount, MemoryLayout<PackedSphere>.stride) else { return false }
        for i in 0..<Int(header.hullCount) {
            let instance = raw.loadUnaligned(fromByteOffset: Int(header.hullOffset) + i * MemoryLayout<PackedHullInstance>.stride, as: PackedHullInstance.self)
            let offset = Int(instance.hullOffset)
            guard offset >= 0, offset & 7 == 0, offset + MemoryLayout<HullData>.stride <= bytes.count else { return false }
            let hull = raw.loadUnaligned(fromByteOffset: offset, as: HullData.self)
            guard hull.version == B3.hullVersion, hull.byteCount >= MemoryLayout<HullData>.stride,
                  Int64(offset) + Int64(hull.byteCount) <= bytes.count else { return false }
        }
        for i in 0..<Int(header.meshCount) {
            let instance = raw.loadUnaligned(fromByteOffset: Int(header.meshOffset) + i * MemoryLayout<PackedMeshInstance>.stride, as: PackedMeshInstance.self)
            let offset = Int(instance.meshOffset)
            guard offset >= 0, offset & 7 == 0, instance.materialOffset >= 0, instance.materialCount > 0, offset + MemoryLayout<MeshData>.stride <= bytes.count,
                  Int64(instance.materialOffset) + Int64(instance.materialCount) * 2 <= bytes.count else { return false }
            let mesh = raw.loadUnaligned(fromByteOffset: offset, as: MeshData.self)
            guard mesh.version == B3.meshVersion, mesh.byteCount >= MemoryLayout<MeshData>.stride,
                  Int64(offset) + Int64(mesh.byteCount) <= bytes.count else { return false }
        }
        return true
    }
}
