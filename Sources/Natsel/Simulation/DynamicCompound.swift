/// A moving compound instantiated as convex fixtures on one body. Child ids follow the baked
/// Compound's child order, so query hits can be mapped back without changing existing ShapeID APIs.
/// Each child uses the world's broad phase, contact solver, sensors, and CCD.
public struct DynamicCompoundInstance: Sendable {
    public let bodyId: BodyID
    public let childShapeIds: [ShapeID]

    fileprivate init(bodyId: BodyID, childShapeIds: [ShapeID]) {
        self.bodyId = bodyId
        self.childShapeIds = childShapeIds
    }

    public func childIndex(of shape: ShapeID) -> Int? { childShapeIds.firstIndex(of: shape) }
}

extension World {
    /// Instantiate an immutable compound on a dynamic or kinematic body. Moving compounds must
    /// contain only convex children; meshes must be decomposed before instantiation. Invalid input
    /// leaves the body unchanged. Mass is computed once after all children are attached.
    public func createDynamicCompound(
        _ body: BodyID, _ definition: ShapeDef, _ compound: Compound
    ) -> DynamicCompoundInstance? {
        guard !locked, isValidBody(body), bodyType(body) != .staticBody else { return nil }
        let children = (0..<compound.childCount).map { compound.child(at: Int($0)) }
        guard children.allSatisfy({ if case .mesh = $0.geometry { return false }; return true }) else { return nil }
        var ids: [ShapeID] = []
        for child in children {
            var def = definition
            def.updateBodyMass = false
            def.baseMaterial = compound.materials[Int(child.materialIndices[0])]
            let id: ShapeID
            switch child.geometry {
            case .sphere(let sphere):
                id = createSphereShape(body, def, Sphere(center: transformPoint(child.transform, sphere.center), radius: sphere.radius))
            case .capsule(let capsule):
                id = createCapsuleShape(body, def, Capsule(center1: transformPoint(child.transform, capsule.center1),
                    center2: transformPoint(child.transform, capsule.center2), radius: capsule.radius))
            case .hull(let hull):
                id = createTransformedHullShape(body, def, hull, child.transform, .one)
            case .mesh:
                // Validated before attaching any child.
                return nil
            }
            guard shapeIsValid(id) else {
                for created in ids { destroyShape(created, updateBodyMass: false) }
                if definition.updateBodyMass { applyMassFromShapes(body) }
                return nil
            }
            ids.append(id)
        }
        if definition.updateBodyMass { applyMassFromShapes(body) }
        return DynamicCompoundInstance(bodyId: body, childShapeIds: ids)
    }

    public func dynamicCompoundIsValid(_ instance: DynamicCompoundInstance) -> Bool {
        isValidBody(instance.bodyId) && instance.childShapeIds.allSatisfy {
            shapeIsValid($0) && shapes[Int($0.index1 - 1)].bodyId == instance.bodyId.index1 - 1
        }
    }

    /// Replace the children atomically with respect to stepping. Validate and instantiate the new
    /// children before releasing the old ones, retaining unrelated fixtures on the same body.
    public func replaceDynamicCompound(
        _ instance: DynamicCompoundInstance, _ definition: ShapeDef, _ compound: Compound
    ) -> DynamicCompoundInstance? {
        guard !locked, dynamicCompoundIsValid(instance) else { return nil }
        var def = definition
        def.updateBodyMass = false
        guard let replacement = createDynamicCompound(instance.bodyId, def, compound) else { return nil }
        destroyDynamicCompound(instance, updateBodyMass: false)
        if definition.updateBodyMass { applyMassFromShapes(instance.bodyId) }
        return replacement
    }

    /// Release the remaining live children, including when some children were removed individually.
    public func destroyDynamicCompound(_ instance: DynamicCompoundInstance, updateBodyMass: Bool = true) {
        guard !locked, isValidBody(instance.bodyId) else { return }
        for id in instance.childShapeIds where shapeIsValid(id)
            && shapes[Int(id.index1 - 1)].bodyId == instance.bodyId.index1 - 1 {
            destroyShape(id, updateBodyMass: false)
        }
        if updateBodyMass { applyMassFromShapes(instance.bodyId) }
    }
}
