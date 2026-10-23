public func createWaveMesh(xCount: Int, zCount: Int, cellWidth: Float, amplitude: Float,
                           rowFrequency: Float, columnFrequency: Float) -> TriangleMesh? {
    guard xCount > 0, zCount > 0, cellWidth > 0 else { return nil }
    var vertices: [Vec3] = []
    let xWidth = cellWidth * Float(xCount)
    let zWidth = cellWidth * Float(zCount)
    let omegaZ = 2 * B3.pi * rowFrequency * cellWidth
    let omegaX = 2 * B3.pi * columnFrequency * cellWidth
    var x: Float = -0.5 * xWidth
    for ix in 0...xCount {
        let rowHeight = computeCosSin(omegaX * Float(ix)).sine
        var z: Float = -0.5 * zWidth
        for iz in 0...zCount {
            let columnHeight = computeCosSin(omegaZ * Float(iz)).sine
            let y = amplitude * rowHeight * columnHeight
            vertices.append(Vec3(x, y, z))
            z += cellWidth
        }
        x += cellWidth
    }
    var options = MeshBuildOptions()
    options.useMedianSplit = true
    options.identifyEdges = true
    return TriangleMesh(vertices: vertices, indices: meshGridIndices(xCount: xCount, zCount: zCount), options: options)
}

func meshGridIndices(xCount: Int, zCount: Int) -> [Int32] {
    var indices: [Int32] = []
    indices.reserveCapacity(6 * xCount * zCount)
    for ix in 0..<xCount {
        for iz in 0..<zCount {
            let i1 = Int32(iz + (zCount + 1) * ix)
            let i2 = i1 + 1
            let i3 = i2 + Int32(zCount + 1)
            let i4 = i3 - 1
            indices.append(contentsOf: [i1, i2, i3, i3, i4, i1])
        }
    }
    return indices
}

public func createTorusMesh(radialResolution: Int, tubularResolution: Int,
                            radius: Float, thickness: Float) -> TriangleMesh? {
    guard radialResolution >= 3, tubularResolution >= 3 else { return nil }
    var vertices: [Vec3] = []
    for radial in 0..<radialResolution {
        for tubular in 0..<tubularResolution {
            let u = Float(tubular) / Float(tubularResolution) * B3.twoPi
            let v = Float(radial) / Float(radialResolution) * B3.twoPi
            let csU = computeCosSin(u)
            let csV = computeCosSin(v)
            let x = (radius + thickness * csV.cosine) * csU.cosine
            let y = (radius + thickness * csV.cosine) * csU.sine
            let z = thickness * csV.sine
            vertices.append(Vec3(x, y, z))
        }
    }
    var indices: [Int32] = []
    for radial1 in 0..<radialResolution {
        let radial2 = (radial1 + 1) % radialResolution
        for tubular1 in 0..<tubularResolution {
            let tubular2 = (tubular1 + 1) % tubularResolution
            let i1 = Int32(radial1 * tubularResolution + tubular1)
            let i2 = Int32(radial1 * tubularResolution + tubular2)
            let i3 = Int32(radial2 * tubularResolution + tubular2)
            let i4 = Int32(radial2 * tubularResolution + tubular1)
            indices.append(contentsOf: [i1, i2, i3, i3, i4, i1])
        }
    }
    var options = MeshBuildOptions()
    options.identifyEdges = true
    return TriangleMesh(vertices: vertices, indices: indices, options: options)
}

private let boxMeshIndices: [Int32] = [
    0,1,3, 1,2,3, 0,4,1, 1,4,5, 0,3,7, 4,0,7,
    4,7,5, 6,5,7, 1,5,2, 6,2,5, 3,2,7, 6,7,2,
]

private func meshBoxVertices(center: Vec3, extent: Vec3) -> [Vec3] {
    let x = extent.x, y = extent.y, z = extent.z
    return [Vec3(x,y,z), Vec3(-x,y,z), Vec3(-x,-y,z), Vec3(x,-y,z),
            Vec3(x,y,-z), Vec3(-x,y,-z), Vec3(-x,-y,-z), Vec3(x,-y,-z)].map { $0 + center }
}

public func createBoxMesh(center: Vec3, extent: Vec3, identifyEdges: Bool = false) -> TriangleMesh? {
    var options = MeshBuildOptions()
    options.identifyEdges = identifyEdges
    return TriangleMesh(vertices: meshBoxVertices(center: center, extent: extent), indices: boxMeshIndices, options: options)
}

public func createHollowBoxMesh(center: Vec3, extent: Vec3) -> TriangleMesh? {
    var indices = boxMeshIndices
    for i in stride(from: 0, to: indices.count, by: 3) { indices.swapAt(i, i + 2) }
    var options = MeshBuildOptions()
    options.identifyEdges = true
    return TriangleMesh(vertices: meshBoxVertices(center: center, extent: extent), indices: indices, options: options)
}

public func createPlatformMesh(center: Vec3, height: Float, topWidth: Float, bottomWidth: Float) -> TriangleMesh? {
    let hb: Float = 0.5 * bottomWidth
    let ht: Float = 0.5 * topWidth
    let hy: Float = 0.5 * height
    let vertices = [Vec3(ht,hy,ht), Vec3(-ht,hy,ht), Vec3(-hb,-hy,hb), Vec3(hb,-hy,hb),
                    Vec3(ht,hy,-ht), Vec3(-ht,hy,-ht), Vec3(-hb,-hy,-hb), Vec3(hb,-hy,-hb)].map { $0 + center }
    var options = MeshBuildOptions()
    options.useMedianSplit = true
    options.identifyEdges = true
    return TriangleMesh(vertices: vertices, indices: boxMeshIndices, options: options)
}
