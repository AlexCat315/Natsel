nonisolated(unsafe) public var meshTraceEnabled = false

extension World {
    func computeMeshManifolds(_ contactId: Int32, shapeA: Shape, transformA: WorldTransform,
                              shapeB: Shape, transformB: WorldTransform, isFast: Bool,
                              materialIndices: [UInt16]? = nil, _ scratch: UnsafeMutablePointer<MeshContactScratch>,
                              cs: inout ContactStore) -> Bool {
        guard let surface = TriangleSurface(shapeA.geometry) else { return false }
        var contact = cs.get(contactId)
        contact.meshContact.refresh(surface: surface, transform: transformA, bounds: shapeB.aabb)
        let triangles = collideMeshTriangles(&contact, surface: surface, transformA: transformA,
                                             shapeB: shapeB, transformB: transformB, isFast: isFast, scratch)
        let accepted = selectMeshManifolds(
            triangles, sphere: shapeB.type == .sphereShape, scratch)
        if meshTraceEnabled && contact.shapeIdA == 0 && contact.shapeIdB == 2 {
            func h(_ f: Float) -> String { String(f.bitPattern, radix: 16) }
            print("T count=\(contact.meshContact.triangles.count) ids=\(contact.meshContact.triangles.map { $0.triangleIndex })")
            for m in triangles { print("T tri \(m.triangleIndex) f=\(m.feature.rawValue) d=\(h(m.squaredDistance)) n=\(h(m.normal.x)) \(h(m.normal.y)) \(h(m.normal.z)) p=\(m.points.map { h($0.point.x) + "," + h($0.separation) })") }
            for m in accepted { print("T acc \(m.triangleIndex)") }
        }
        if accepted.isEmpty {
            contact.manifolds.removeAll()
            contact.manifoldCount = 0
            cs.set(contactId, contact)
            return false
        }
        let clusters = clusterMeshManifolds(accepted, scratch)
        let persisted = persistMeshManifolds(
            clusters, previous: contact.payload!.pointee.manifolds,
            transformA: transformA, transformB: transformB, scratch)
        contact.manifolds.replace(with: persisted)
        contact.manifoldCount = Int32(contact.manifolds.count)
        mixMeshMaterials(&contact, surface: surface, shapeA: shapeA, transformA: transformA,
                         shapeB: shapeB, transformB: transformB, materialIndices: materialIndices)
        cs.set(contactId, contact)
        return true
    }
}
