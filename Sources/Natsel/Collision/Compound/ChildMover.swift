func collideMoverAndCompound(_ compound: Compound, _ mover: Capsule, capacity: Int) -> [PlaneResult] {
    guard capacity > 0 else { return [] }
    let radius = Vec3(mover.radius, mover.radius, mover.radius)
    let bounds = AABB(minVec3(mover.center1, mover.center2) - radius,
                      maxVec3(mover.center1, mover.center2) + radius)
    var results: [PlaneResult] = []
    queryCompound(compound, bounds: bounds) { index in
        let child = compound.child(at: index)
        let local = Capsule(center1: invTransformPoint(child.transform, mover.center1),
                            center2: invTransformPoint(child.transform, mover.center2), radius: mover.radius)
        var planes: [PlaneResult]
        var plane = PlaneResult()
        switch child.geometry {
        case .capsule(let capsule):
            planes = collideMoverAndCapsule(&plane, capsule, local) == 1 ? [plane] : []
        case .hull(let hull):
            planes = collideMoverAndHull(&plane, hull, local) == 1 ? [plane] : []
        case .mesh(let mesh):
            planes = collideMoverAndMesh(mesh, local, capacity: capacity - results.count)
        case .sphere(let sphere):
            planes = collideMoverAndSphere(&plane, sphere, local) == 1 ? [plane] : []
        }
        for var result in planes {
            result.plane.normal = rotateVector(child.transform.q, result.plane.normal)
            result.point = transformPoint(child.transform, result.point)
            result.childIndex = Int32(index)
            result.materialIndex = Int32(child.materialIndices[Int(result.materialIndex)])
            results.append(result)
        }
        return results.count < capacity
    }
    return results
}
