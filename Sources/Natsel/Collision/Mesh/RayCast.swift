/// Triangle intersection. One is both the miss sentinel and the endpoint.
func intersectMeshRayTriangle(_ start: Vec3, _ delta: Vec3, _ a: Vec3, _ b: Vec3, _ c: Vec3) -> Float {
    let edge1 = c - b
    let edge2 = a - c
    let edge3 = b - a
    let middle1 = 0.5 * (b + c)
    let middle2 = 0.5 * (c + a)
    let middle3 = 0.5 * (a + b)
    let normal1 = cross(edge1, middle1 - start)
    let normal2 = cross(edge2, middle2 - start)
    let normal3 = cross(edge3, middle3 - start)
    if dot(normal1, delta) < 0 || dot(normal2, delta) < 0 || dot(normal3, delta) < 0 { return 1 }
    let normal = cross(b - a, c - a)
    let denominator = dot(normal, delta)
    if denominator >= 0 { return 1 }
    let fraction = dot(normal, a - start) / denominator
    if fraction <= 0 { return 1 }
    return minFloat(fraction, 1)
}

public func rayCastMesh(_ mesh: Mesh, _ input: RayCastInput) -> CastOutput {
    var best = CastOutput()
    best.fraction = input.maxFraction
    best.triangleIndex = B3.nullIndex
    guard let data = mesh.data else { return best }
    let scale = mesh.scale
    let inverseScale = Vec3(1 / scale.x, 1 / scale.y, 1 / scale.z)
    let ccw = scale.x * scale.y * scale.z > 0
    let start = inverseScale * input.origin
    let delta = inverseScale * input.translation
    var end = start + input.maxFraction * delta
    var rayMin = minVec3(start, end)
    var rayMax = maxVec3(start, end)
    var stack: [Int] = []
    var index = 0
    let nodes = data.nodes
    let triangles = data.triangles
    let vertices = data.vertices
    let materials = data.materialIndices
    while true {
        let node = nodes[index]
        let separation = maxVec3(rayMin - node.upperBound, node.lowerBound - rayMax)
        if separation.x <= 0 && separation.y <= 0 && separation.z <= 0
            && testBoundsRayOverlap(node.lowerBound, node.upperBound, start, delta) {
            if node.isLeaf {
                let offset = Int(node.triangleOffset)
                for triangleIndex in offset..<(offset + node.leafTriangleCount) {
                    let triangle = triangles[triangleIndex]
                    let a = scale * vertices[Int(triangle.index1)]
                    let b = scale * vertices[Int(ccw ? triangle.index2 : triangle.index3)]
                    let c = scale * vertices[Int(ccw ? triangle.index3 : triangle.index2)]
                    let alpha = intersectMeshRayTriangle(input.origin, input.translation, a, b, c)
                    if alpha < best.fraction {
                        best.normal = normalize(cross(b - a, c - a))
                        best.point = input.origin + alpha * input.translation
                        best.fraction = alpha
                        best.triangleIndex = Int32(triangleIndex)
                        best.materialIndex = Int32(materials[triangleIndex])
                        best.hit = true
                        end = start + alpha * delta
                        rayMin = minVec3(start, end)
                        rayMax = maxVec3(start, end)
                    }
                }
            } else {
                if getByIndex(delta, Int(node.data & 3)) > 0 {
                    stack.append(index + node.childOffset)
                    index += 1
                } else {
                    stack.append(index + 1)
                    index += node.childOffset
                }
                continue
            }
        }
        guard let next = stack.popLast() else { break }
        index = next
    }
    return best
}
