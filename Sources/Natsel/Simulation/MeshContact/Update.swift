extension World {
    func computeMeshManifolds(_ contactId: Int32, shapeA: Shape, transformA: WorldTransform,
                              shapeB: Shape, transformB: WorldTransform, isFast: Bool,
                              materialIndices: [UInt16]? = nil, cs: inout ContactStore) -> Bool {
        guard let surface = TriangleSurface(shapeA.geometry) else { return false }
        var contact = cs.get(contactId)
        contact.meshContact.refresh(surface: surface, transform: transformA, bounds: shapeB.aabb)
        let triangles = collideMeshTriangles(&contact, surface: surface, transformA: transformA,
                                             shapeB: shapeB, transformB: transformB, isFast: isFast)
        let accepted = selectMeshManifolds(triangles, sphere: shapeB.type == .sphereShape)
        if accepted.isEmpty {
            contact.manifolds.removeAll()
            contact.manifoldCount = 0
            cs.set(contactId, contact)
            return false
        }
        let clusters = clusterMeshManifolds(accepted)
        let persisted = persistMeshManifolds(clusters, previous: contact.payload!.pointee.manifolds,
                                             transformA: transformA, transformB: transformB)
        contact.manifolds.replace(with: persisted)
        contact.manifoldCount = Int32(contact.manifolds.count)
        mixMeshMaterials(&contact, surface: surface, shapeA: shapeA, transformA: transformA,
                         shapeB: shapeB, transformB: transformB, materialIndices: materialIndices)
        cs.set(contactId, contact)
        return true
    }
}
