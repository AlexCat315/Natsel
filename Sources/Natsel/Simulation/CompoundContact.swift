extension World {
    func updateCompoundContact(_ contactId: Int32, _ shapeA: Shape, _ xfA: WorldTransform,
                               _ shapeB: Shape, _ xfB: WorldTransform, _ isFast: Bool,
                               _ cs: inout ContactStore) -> Bool {
        guard case .compound(let compound) = shapeA.geometry else { return false }
        let child = compound.child(at: Int(cs.get(contactId).childIndex))
        var temporary = shapeA
        let touching: Bool
        switch child.geometry {
        case .capsule(let capsule):
            temporary.type = .capsuleShape
            temporary.geometry = .capsule(capsule)
            temporary.materials = [shapeA.material(Int32(child.materialIndices[0]))]
            if shapeB.type == .hullShape {
                touching = updateConvexContact(contactId, shapeB, xfB, temporary, xfA, true, &cs)
            } else {
                touching = updateConvexContact(contactId, temporary, xfA, shapeB, xfB, false, &cs)
            }
        case .hull(let hull):
            temporary.type = .hullShape
            temporary.geometry = .hull(hull)
            temporary.materials = [shapeA.material(Int32(child.materialIndices[0]))]
            touching = updateConvexContact(contactId, temporary, mulWorldTransforms(xfA, child.transform), shapeB, xfB, false, &cs)
        case .mesh(let mesh):
            temporary.type = .meshShape
            temporary.geometry = .mesh(mesh)
            touching = computeMeshManifolds(contactId, shapeA: temporary, transformA: mulWorldTransforms(xfA, child.transform),
                                             shapeB: shapeB, transformB: xfB, isFast: isFast,
                                             materialIndices: child.materialIndices, cs: &cs)
        case .sphere(let sphere):
            temporary.type = .sphereShape
            temporary.geometry = .sphere(sphere)
            temporary.materials = [shapeA.material(Int32(child.materialIndices[0]))]
            if shapeB.type == .capsuleShape || shapeB.type == .hullShape {
                touching = updateConvexContact(contactId, shapeB, xfB, temporary, xfA, true, &cs)
            } else {
                touching = updateConvexContact(contactId, temporary, xfA, shapeB, xfB, false, &cs)
            }
        }
        if case .mesh = child.geometry {
            var contact = cs.get(contactId)
            if touching && ((shapeA.flags | shapeB.flags) & ShapeFlags.enableHitEvents.rawValue) != 0 {
                contact.flags |= ContactFlags.simEnableHitEvent.rawValue
            } else {
                contact.flags &= ~ContactFlags.simEnableHitEvent.rawValue
            }
            cs.set(contactId, contact)
        }
        let offset = rotateVector(xfA.q, child.transform.p)
        var contact = cs.get(contactId)
        for i in 0..<Int(contact.manifoldCount) {
            for j in 0..<Int(contact.manifolds[i].pointCount) {
                contact.manifolds[i].points[j].anchorA = contact.manifolds[i].points[j].anchorA + offset
            }
        }
        cs.set(contactId, contact)
        return touching
    }
}
