extension HeightField {
    func corners(row: Int, column: Int) -> InlineArray<4, Vec3> {
        let i11 = row * columnCount + column
        let i12 = i11 + 1
        let i21 = (row + 1) * columnCount + column
        let i22 = i21 + 1
        let header = header
        let heights = compressedHeights
        let h11 = header.minHeight + header.heightScale * Float(heights[i11])
        let h12 = header.minHeight + header.heightScale * Float(heights[i12])
        let h21 = header.minHeight + header.heightScale * Float(heights[i21])
        let h22 = header.minHeight + header.heightScale * Float(heights[i22])
        let x1 = Float(column), x2 = Float(column + 1)
        let z1 = Float(row), z2 = Float(row + 1)
        return [scale * Vec3(x1, h11, z1), scale * Vec3(x2, h12, z1), scale * Vec3(x1, h21, z2), scale * Vec3(x2, h22, z2)]
    }

    func triangle(_ index: Int32) -> Triangle {
        let cell = Int(index) >> 1
        let row = cell / (columnCount - 1)
        let column = cell - row * (columnCount - 1)
        let v = corners(row: row, column: column)
        let i11 = Int32(row * columnCount + column)
        let i12 = i11 + 1
        let i21 = Int32((row + 1) * columnCount + column)
        let i22 = i21 + 1
        var triangle = Triangle()
        triangle.flags = Int32(edgeFlags[Int(index)])
        if index & 1 == 0 {
            triangle.vertices = [v[0], v[2], v[1]]
            triangle.i1 = i11; triangle.i2 = i21; triangle.i3 = i12
        } else {
            triangle.vertices = [v[3], v[1], v[2]]
            triangle.i1 = i22; triangle.i2 = i12; triangle.i3 = i21
        }
        if header.clockwise != 0 {
            let second = triangle.vertices[1]
            triangle.vertices[1] = triangle.vertices[2]
            triangle.vertices[2] = second
            swap(&triangle.i2, &triangle.i3)
            let flags = triangle.flags
            let edge1 = flags & 0x11
            let edge3 = flags & 0x44
            triangle.flags = (flags & ~0x55) | (edge1 << 2) | (edge3 >> 2)
        }
        return triangle
    }

    func material(triangle index: Int32) -> Int32 { Int32(materialIndices[Int(index) >> 1]) }
}

public func computeHeightFieldAABB(_ heightField: HeightField, _ transform: Transform) -> AABB {
    aabbTransform(transform, heightField.bounds)
}
