

func collideMover(_ shape: Shape, _ transform: Transform, _ mover: Capsule, capacity: Int) -> [PlaneResult] {
    guard capacity > 0 else { return [] }
    let local = Capsule(center1: invTransformPoint(transform, mover.center1),
                        center2: invTransformPoint(transform, mover.center2), radius: mover.radius)
    var plane = PlaneResult()
    var planes: [PlaneResult]
    switch shape.geometry {
    case .capsule(let capsule): planes = collideMoverAndCapsule(&plane, capsule, local) == 1 ? [plane] : []
    case .sphere(let sphere): planes = collideMoverAndSphere(&plane, sphere, local) == 1 ? [plane] : []
    case .hull(let hull): planes = collideMoverAndHull(&plane, hull, local) == 1 ? [plane] : []
    case .compound(let compound): planes = collideMoverAndCompound(compound, local, capacity: capacity)
    case .mesh(let mesh): planes = collideMoverAndMesh(mesh, local, capacity: capacity)
    case .heightField(let field): planes = collideMoverAndHeightField(field, local, capacity: capacity)
    }
    for i in planes.indices {
        planes[i].plane.normal = rotateVector(transform.q, planes[i].plane.normal)
        planes[i].point = transformPoint(transform, planes[i].point)
        planes[i].materialIndex = clampInt(planes[i].materialIndex, 0, shape.materialCount - 1)
    }
    return planes
}
