func heightFieldAdjacency(heights: [Float], materials: [UInt8], columns: Int, rows: Int, scale: Vec3) -> [UInt8] {
    func vertices(_ row: Int, _ column: Int, second: Bool) -> [Vec3] {
        let i11 = row * columns + column
        let i12 = i11 + 1
        let i21 = (row + 1) * columns + column
        let i22 = i21 + 1
        let x1 = Float(column), x2 = Float(column + 1)
        let z1 = Float(row), z2 = Float(row + 1)
        if second {
            return [scale * Vec3(x2, heights[i22], z2), scale * Vec3(x2, heights[i12], z1), scale * Vec3(x1, heights[i21], z2)]
        }
        return [scale * Vec3(x1, heights[i11], z1), scale * Vec3(x1, heights[i21], z2), scale * Vec3(x2, heights[i12], z1)]
    }
    func edgeFlags(_ separation: Float, _ cosine: Float, edge: Int) -> UInt8 {
        var flags: UInt8 = 0
        if separation > 0 || cosine > 0.9962 { flags |= UInt8(1 << edge) }
        if separation < 0 || cosine > 0.9962 { flags |= UInt8(0x10 << edge) }
        return flags
    }
    var flags = [UInt8](repeating: 0, count: 2 * (rows - 1) * (columns - 1))
    for row in 0..<(rows - 1) {
        for column in 0..<(columns - 1) {
            let cell = row * (columns - 1) + column
            if materials[cell] == 0xff { continue }
            let first = vertices(row, column, second: false)
            let second = vertices(row, column, second: true)
            let plane1 = makePlaneFromPoints(first[0], first[1], first[2])
            let plane2 = makePlaneFromPoints(second[0], second[1], second[2])
            let diagonal = edgeFlags(planeSeparation(plane1, second[0]), dot(plane1.normal, plane2.normal), edge: 1)
            var flags1 = diagonal
            var flags2 = diagonal
            if row > 0 && materials[(row - 1) * (columns - 1) + column] != 0xff {
                let v = vertices(row - 1, column, second: true)
                let n = normalize(cross(v[1] - v[0], v[2] - v[0]))
                flags1 |= edgeFlags(planeSeparation(plane1, v[1]), dot(plane1.normal, n), edge: 2)
            }
            if row + 1 < rows - 1 && materials[(row + 1) * (columns - 1) + column] != 0xff {
                let v = vertices(row + 1, column, second: false)
                let n = normalize(cross(v[1] - v[0], v[2] - v[0]))
                flags2 |= edgeFlags(planeSeparation(plane2, v[1]), dot(plane2.normal, n), edge: 2)
            }
            if column > 0 && materials[row * (columns - 1) + column - 1] != 0xff {
                let v = vertices(row, column - 1, second: true)
                let n = normalize(cross(v[1] - v[0], v[2] - v[0]))
                flags1 |= edgeFlags(planeSeparation(plane1, v[2]), dot(plane1.normal, n), edge: 0)
            }
            if column + 1 < columns - 1 && materials[row * (columns - 1) + column + 1] != 0xff {
                let v = vertices(row, column + 1, second: false)
                let n = normalize(cross(v[1] - v[0], v[2] - v[0]))
                flags2 |= edgeFlags(planeSeparation(plane2, v[2]), dot(plane2.normal, n), edge: 0)
            }
            flags[2 * cell] = flags1
            flags[2 * cell + 1] = flags2
        }
    }
    return flags
}
