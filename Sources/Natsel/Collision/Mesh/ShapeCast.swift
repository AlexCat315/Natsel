func shapeCastMesh(_ mesh: Mesh, _ input: ShapeCastInput) -> CastOutput {
    var best = CastOutput()
    best.fraction = input.maxFraction
    best.triangleIndex = B3.nullIndex
    guard let data = mesh.data else { return best }
    let shapeBounds = makeAABB(input.proxy.points!, input.proxy.count, input.proxy.radius)
    let center = aabbCenter(shapeBounds)
    let extent = aabbExtents(shapeBounds)
    var end = center + input.maxFraction * input.translation
    var rayMin = minVec3(center, end)
    var rayMax = maxVec3(center, end)
    let scale = mesh.scale
    let inverse = Vec3(1 / scale.x, 1 / scale.y, 1 / scale.z)
    let ccw = scale.x * scale.y * scale.z > 0
    let start = inverse * center
    let delta = inverse * input.translation
    var localEnd = start + input.maxFraction * delta
    var localMin = minVec3(start, localEnd)
    var localMax = maxVec3(start, localEnd)
    let localExtent = absVec3(inverse) * extent
    let nodes = data.nodes
    let triangles = data.triangles
    let vertices = data.vertices
    let materials = data.materialIndices
    var stack: [Int] = []
    var index = 0
    while true {
        let node = nodes[index]
        let nodeMin = node.lowerBound - localExtent
        let nodeMax = node.upperBound + localExtent
        if boundsOverlap(nodeMin, nodeMax, localMin, localMax)
            && testBoundsRayOverlap(nodeMin, nodeMax, start, delta) {
            if node.isLeaf {
                let offset = Int(node.triangleOffset)
                for triangleIndex in offset..<(offset + node.leafTriangleCount) {
                    let triangle = triangles[triangleIndex]
                    let a = scale * vertices[Int(triangle.index1)]
                    let b = scale * vertices[Int(ccw ? triangle.index2 : triangle.index3)]
                    let c = scale * vertices[Int(ccw ? triangle.index3 : triangle.index2)]
                    let triangleMin = minVec3(a, minVec3(b, c)) - extent
                    let triangleMax = maxVec3(a, maxVec3(b, c)) + extent
                    if !boundsOverlap(triangleMin, triangleMax, rayMin, rayMax) { continue }
                    if signedVolume(a, b, c, center) < 0 { continue }
                    let trianglePoints = [Vec3.zero, b - a, c - a]
                    let output = trianglePoints.withUnsafeBufferPointer { points in
                        var pair = ShapeCastPairInput()
                        pair.proxyA = ShapeProxy(points: points.baseAddress, count: 3, radius: 0)
                        pair.proxyB = input.proxy
                        pair.transform = Transform(-a, .identity)
                        pair.maxFraction = best.fraction
                        pair.translationB = input.translation
                        pair.canEncroach = input.canEncroach
                        return shapeCast(pair)
                    }
                    if output.hit {
                        best = output
                        best.point = output.point + a
                        best.triangleIndex = Int32(triangleIndex)
                        best.materialIndex = Int32(materials[triangleIndex])
                        end = center + output.fraction * input.translation
                        rayMin = minVec3(center, end)
                        rayMax = maxVec3(center, end)
                        localEnd = start + output.fraction * delta
                        localMin = minVec3(start, localEnd)
                        localMax = maxVec3(start, localEnd)
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
