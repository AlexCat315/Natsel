public func queryCompound(_ compound: Compound, bounds: AABB, _ callback: (Int) -> Bool) {
    _ = compound.withTreeView { view in
        view.query(bounds) { _, data in callback(Int(data)) }
    }
}

func overlapCompound(_ compound: Compound, _ transform: Transform, _ proxy: ShapeProxy) -> Bool {
    var buffer = [Vec3](repeating: .zero, count: Int(B3.maxShapeCastPoints))
    return buffer.withUnsafeMutableBufferPointer { points in
        let local = makeLocalProxy(proxy, transform, points.baseAddress!)
        var touching = false
        let bounds = computeProxyAABB(local)
        _ = compound.withTreeView { view in
            view.query(bounds) { _, data in
                let child = compound.child(at: Int(data))
                switch child.geometry {
                case .capsule(let capsule): touching = overlapCapsule(capsule, child.transform, local)
                case .hull(let hull): touching = overlapHull(hull, child.transform, local)
                case .mesh(let mesh): touching = overlapMesh(mesh, child.transform, local)
                case .sphere(let sphere): touching = overlapSphere(sphere, child.transform, local)
                }
                return !touching
            }
        }
        return touching
    }
}

public func rayCastCompound(_ compound: Compound, _ input: RayCastInput) -> CastOutput {
    var result = CastOutput()
    _ = compound.withTreeView { view in
        view.rayCast(input) { input, _, data in
            let child = compound.child(at: Int(data))
            var local = input
            local.origin = invTransformPoint(child.transform, input.origin)
            local.translation = invRotateVector(child.transform.q, input.translation)
            var output: CastOutput
            switch child.geometry {
            case .capsule(let capsule): output = rayCastCapsule(capsule, local)
            case .hull(let hull): output = rayCastHull(hull, local)
            case .mesh(let mesh): output = rayCastMesh(mesh, local)
            case .sphere(let sphere): output = rayCastSphere(sphere, local)
            }
            if case .mesh = child.geometry {
                output.materialIndex = Int32(child.materialIndices[Int(output.materialIndex)])
            } else {
                output.materialIndex = Int32(child.materialIndices[0])
            }
            if output.hit {
                output.point = transformPoint(child.transform, output.point)
                output.normal = rotateVector(child.transform.q, output.normal)
                output.childIndex = Int32(data)
                result = output
                return output.fraction
            }
            return input.maxFraction
        }
    }
    return result
}

func shapeCastCompound(_ compound: Compound, _ input: ShapeCastInput) -> CastOutput {
    var result = CastOutput()
    if input.proxy.count == 0 { return result }
    var treeInput = BoxCastInput()
    treeInput.box = makeAABB(input.proxy.points!, input.proxy.count, input.proxy.radius)
    treeInput.translation = input.translation
    treeInput.maxFraction = input.maxFraction
    _ = compound.withTreeView { view in
        view.boxCast(treeInput) { advancing, _, data in
            let child = compound.child(at: Int(data))
            var points = [Vec3](repeating: .zero, count: Int(B3.maxShapeCastPoints))
            var output = points.withUnsafeMutableBufferPointer { points in
                var local = ShapeCastInput()
                local.proxy = makeLocalProxy(input.proxy, child.transform, points.baseAddress!)
                local.translation = invRotateVector(child.transform.q, input.translation)
                local.maxFraction = advancing.maxFraction
                local.canEncroach = input.canEncroach
                switch child.geometry {
                case .capsule(let capsule): return shapeCastCapsule(capsule, local)
                case .hull(let hull): return shapeCastHull(hull, local)
                case .mesh(let mesh): return shapeCastMesh(mesh, local)
                case .sphere(let sphere): return shapeCastSphere(sphere, local)
                }
            }
            if case .mesh = child.geometry {
                output.materialIndex = Int32(child.materialIndices[Int(output.materialIndex)])
            } else {
                output.materialIndex = Int32(child.materialIndices[0])
            }
            if output.hit {
                output.point = transformPoint(child.transform, output.point)
                output.normal = rotateVector(child.transform.q, output.normal)
                output.childIndex = Int32(data)
                result = output
                return output.fraction
            }
            return advancing.maxFraction
        }
    }
    return result
}
