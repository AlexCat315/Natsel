struct HeightFieldRegion {
    var rows: Range<Int>
    var columns: Range<Int>

    init(_ field: HeightField, bounds: AABB) {
        let minRow = Int((bounds.lowerBound.z / field.scale.z).rounded(.down))
        let maxRow = Int((bounds.upperBound.z / field.scale.z).rounded(.down))
        let minColumn = Int((bounds.lowerBound.x / field.scale.x).rounded(.down))
        let maxColumn = Int((bounds.upperBound.x / field.scale.x).rounded(.down))
        let rowStart = max(0, minRow), rowEnd = min(field.rowCount - 1, maxRow + 1)
        let columnStart = max(0, minColumn), columnEnd = min(field.columnCount - 1, maxColumn + 1)
        rows = rowStart..<max(rowStart, rowEnd)
        columns = columnStart..<max(columnStart, columnEnd)
    }
}

func heightCellBounds(_ corners: InlineArray<4, Vec3>) -> AABB {
    AABB(minVec3(minVec3(corners[0], corners[1]), minVec3(corners[2], corners[3])),
         maxVec3(maxVec3(corners[0], corners[1]), maxVec3(corners[2], corners[3])))
}

func queryHeightField(_ field: HeightField, bounds: AABB, _ callback: (Vec3, Vec3, Vec3, Int32) -> Bool) {
    let region = HeightFieldRegion(field, bounds: bounds)
    for row in region.rows {
        for column in region.columns {
            let cell = row * (field.columnCount - 1) + column
            if field.materialIndices[cell] == 0xff { continue }
            let v = field.corners(row: row, column: column)
            if aabbOverlaps(bounds, heightCellBounds(v)) {
                let index = Int32(2 * cell)
                // Upstream height-field queries ignore callback return values.
                if field.header.clockwise != 0 {
                    _ = callback(v[0], v[1], v[2], index)
                    _ = callback(v[3], v[2], v[1], index + 1)
                } else {
                    _ = callback(v[0], v[2], v[1], index)
                    _ = callback(v[3], v[1], v[2], index + 1)
                }
            }
        }
    }
}
