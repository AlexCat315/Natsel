extension World {
    func mixMeshMaterials(_ contact: inout Contact, surface: TriangleSurface, shapeA: Shape, transformA: WorldTransform,
                          shapeB: Shape, transformB: WorldTransform, materialIndices: [UInt16]? = nil) {
        let materialB = shapeB.material(0)
        var tangentA = Vec3.zero
        if shapeA.materialCount > 0 {
            var friction: Float = 0
            var restitution: Float = 0
            var sampleCount: Float = 0
            for manifold in contact.manifolds {
                for i in 0..<Int(manifold.pointCount) {
                    let triangleIndex = Int(manifold.points[i].triangleIndex)
                    let localIndex = surface.material(Int32(triangleIndex))
                    let mapped = materialIndices.map { Int32($0[Int(localIndex)]) } ?? localIndex
                    let materialIndex = clampInt(mapped, 0, shapeA.materialCount - 1)
                    let material = shapeA.material(materialIndex)
                    friction += frictionCallback(material.friction, material.userMaterialId, materialB.friction, materialB.userMaterialId)
                    restitution += restitutionCallback(material.restitution, material.userMaterialId, materialB.restitution, materialB.userMaterialId)
                    tangentA = tangentA + material.tangentVelocity
                    sampleCount += 1
                }
            }
            if sampleCount > 0 {
                let inverse: Float = 1 / sampleCount
                contact.friction = inverse * friction
                contact.restitution = inverse * restitution
                tangentA = inverse * tangentA
            }
        } else {
            let material = shapeA.material(0)
            contact.friction = frictionCallback(material.friction, material.userMaterialId, materialB.friction, materialB.userMaterialId)
            contact.restitution = restitutionCallback(material.restitution, material.userMaterialId, materialB.restitution, materialB.userMaterialId)
            tangentA = material.tangentVelocity
        }
        tangentA = rotateVector(transformA.q, tangentA)
        let radius: Float
        switch shapeB.geometry {
        case .sphere(let sphere): radius = sphere.radius
        case .capsule(let capsule): radius = capsule.radius
        case .hull(let hull): radius = hull.innerRadius
        default: radius = 0
        }
        contact.rollingResistance = materialB.rollingResistance * radius
        contact.tangentVelocity = tangentA - rotateVector(transformB.q, materialB.tangentVelocity)
    }
}
