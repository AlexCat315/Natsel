func persistMeshManifolds(_ clusters: [MeshManifoldCluster], previous: [Manifold],
                          transformA: WorldTransform, transformB: WorldTransform) -> [Manifold] {
    var old = previous
    var consumed = [Bool](repeating: false, count: old.count)
    let matrix = makeMatrixFromQuat(transformB.q)
    let offset = subPos(transformB.p, transformA.p)
    var output: [Manifold] = []
    for cluster in clusters {
        var manifold = Manifold()
        manifold.pointCount = Int32(cluster.points.count)
        manifold.normal = mulMV(matrix, cluster.normal)
        let clusterNormal = mulMV(matrix, cluster.normal)
        var bestDot: Float = 0.995
        var bestIndex: Int?
        for j in old.indices {
            if consumed[j] { continue }
            let d = dot(old[j].normal, clusterNormal)
            if d > bestDot { bestIndex = j; bestDot = d }
        }
        if let bestIndex {
            manifold.frictionImpulse = old[bestIndex].frictionImpulse
            manifold.rollingImpulse = old[bestIndex].rollingImpulse
            manifold.twistImpulse = old[bestIndex].twistImpulse
            consumed[bestIndex] = true
        }
        for (j, source) in cluster.points.enumerated() {
            var target = ManifoldPoint()
            target.anchorB = mulMV(matrix, source.point)
            target.anchorA = target.anchorB + offset
            target.separation = source.separation - B3.meshRestOffset
            target.featureId = makeFeatureId(source.pair)
            target.triangleIndex = source.triangleIndex
            if let bestIndex {
                for k in 0..<Int(old[bestIndex].pointCount) {
                    let point = old[bestIndex].points[k]
                    if target.featureId == point.featureId && target.triangleIndex == point.triangleIndex {
                        target.normalImpulse = point.normalImpulse
                        target.persisted = true
                        old[bestIndex].points[k].triangleIndex = B3.nullIndex
                        break
                    }
                }
            }
            manifold.points[j] = target
        }
        output.append(manifold)
    }
    return output
}
