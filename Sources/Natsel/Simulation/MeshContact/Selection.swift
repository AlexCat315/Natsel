private struct MeshFeatureKeys {
    var keys = InlineArray<64, UInt64>(repeating: 0)
    var count = 0

    mutating func add(_ key: UInt64) -> Bool {
        for i in 0..<count where keys[i] == key { return false }
        if count == 64 { return true }
        keys[count] = key
        count += 1
        return true
    }

    func contains(_ key: UInt64) -> Bool {
        for i in 0..<count where keys[i] == key { return true }
        return false
    }
}

private func meshEdgeKey(_ a: Int32, _ b: Int32) -> UInt64 {
    UInt64(UInt32(bitPattern: minInt(a, b))) << 32 | UInt64(UInt32(bitPattern: maxInt(a, b)))
}

func selectMeshManifolds(_ manifolds: [MeshTriangleManifold], sphere: Bool) -> [MeshTriangleManifold] {
    var edges = MeshFeatureKeys()
    var vertices = MeshFeatureKeys()
    var accepted = InlineArray<512, Int32>(repeating: 0)
    var tentative = InlineArray<512, Int32>(repeating: 0)
    var acceptedCount = 0
    var tentativeCount = 0
    for (index, manifold) in manifolds.enumerated() {
        var accept = manifold.feature == .featureTriangleFace
        if manifold.feature == .featureHullFace {
            if dot(manifold.triangleNormal, manifold.normal) > 0.5 {
                accept = true
            } else {
                var separation = manifold.points.first!.separation
                for point in manifold.points.dropFirst() { separation = minFloat(separation, point.separation) }
                accept = separation < -2 * B3.linearSlop
            }
        }
        if accept {
            let v = manifold.vertices
            _ = edges.add(meshEdgeKey(v[0], v[1]))
            _ = edges.add(meshEdgeKey(v[1], v[2]))
            _ = edges.add(meshEdgeKey(v[2], v[0]))
            _ = vertices.add(UInt64(v[0])); _ = vertices.add(UInt64(v[1])); _ = vertices.add(UInt64(v[2]))
            accepted[acceptedCount] = Int32(index)
            acceptedCount += 1
        } else {
            tentative[tentativeCount] = Int32(index)
            tentativeCount += 1
        }
    }
    if sphere {
        quickSort(Int32(tentativeCount), less: { i, j in
            manifolds[Int(tentative[Int(i)])].squaredDistance
                < manifolds[Int(tentative[Int(j)])].squaredDistance
        }, swap: { i, j in tentative.swapAt(Int(i), Int(j)) })
        for tentativeIndex in 0..<tentativeCount {
            let index = Int(tentative[tentativeIndex])
            let m = manifolds[index]
            let v = m.vertices
            let edge1 = edges.add(meshEdgeKey(v[0], v[1]))
            let edge2 = edges.add(meshEdgeKey(v[1], v[2]))
            let edge3 = edges.add(meshEdgeKey(v[2], v[0]))
            let vertex1 = vertices.add(UInt64(v[0]))
            let vertex2 = vertices.add(UInt64(v[1]))
            let vertex3 = vertices.add(UInt64(v[2]))
            let collide: Bool
            switch m.feature {
            case .featureEdge1: collide = edge1
            case .featureEdge2: collide = edge2
            case .featureEdge3: collide = edge3
            case .featureVertex1: collide = vertex1
            case .featureVertex2: collide = vertex2
            case .featureVertex3: collide = vertex3
            default: b3Assert(false); collide = false
            }
            if collide {
                accepted[acceptedCount] = Int32(index)
                acceptedCount += 1
            }
        }
    } else {
        for tentativeIndex in 0..<tentativeCount {
            let index = Int(tentative[tentativeIndex])
            let m = manifolds[index]
            if m.flags & 0x77 == 0x77 { continue }
            let v = m.vertices
            if m.flags & 0x11 == 0x11 && edges.contains(meshEdgeKey(v[0], v[1])) { continue }
            if m.flags & 0x22 == 0x22 && edges.contains(meshEdgeKey(v[1], v[2])) { continue }
            if m.flags & 0x44 == 0x44 && edges.contains(meshEdgeKey(v[2], v[0])) { continue }
            accepted[acceptedCount] = Int32(index)
            acceptedCount += 1
        }
    }
    var result: [MeshTriangleManifold] = []
    result.reserveCapacity(acceptedCount)
    for i in 0..<acceptedCount {
        result.append(manifolds[Int(accepted[i])])
    }
    return result
}

struct MeshManifoldCluster {
    var normal: Vec3
    var triangleNormal: Vec3
    var points: [LocalManifoldPoint] = []
}

func clusterMeshManifolds(_ manifolds: [MeshTriangleManifold]) -> [MeshManifoldCluster] {
    var clusters: [MeshManifoldCluster] = []
    for manifold in manifolds {
        var selected: Int?
        for (index, cluster) in clusters.enumerated() {
            let manifoldAngle = dot(cluster.normal, manifold.normal)
            let triangleAngle = dot(cluster.triangleNormal, manifold.triangleNormal)
            if manifoldAngle <= 0.996 || triangleAngle <= 0.996 { continue }
            selected = index
            break
        }
        let index = selected ?? clusters.count
        if selected == nil {
            clusters.append(MeshManifoldCluster(normal: manifold.normal, triangleNormal: manifold.triangleNormal))
        }
        for var point in manifold.points {
            point.triangleIndex = manifold.triangleIndex
            clusters[index].points.append(point)
        }
    }
    for i in clusters.indices {
        clusters[i].points = reduceMeshCluster(clusters[i].points, normal: clusters[i].triangleNormal)
    }
    return clusters
}
