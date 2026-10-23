public func createGridHeightField(rowCount: Int, columnCount: Int, scale: Vec3 = .one, makeHoles: Bool = false) -> HeightField? {
    guard rowCount >= 2, columnCount >= 2 else { return nil }
    var definition = heightFieldGridDefinition(rows: rowCount, columns: columnCount, scale: scale, holes: makeHoles)
    definition.heights = [Float](repeating: 0, count: rowCount * columnCount)
    return HeightField(definition)
}

public func createWaveHeightField(rowCount: Int, columnCount: Int, scale: Vec3 = .one,
                                  rowFrequency: Float, columnFrequency: Float, makeHoles: Bool = false) -> HeightField? {
    guard rowCount >= 2, columnCount >= 2 else { return nil }
    var definition = heightFieldGridDefinition(rows: rowCount, columns: columnCount, scale: scale, holes: makeHoles)
    let omegaZ = 2 * B3.pi * rowFrequency
    let omegaX = 2 * B3.pi * columnFrequency
    for row in 0..<rowCount {
        let rowHeight = computeCosSin(omegaZ * Float(row)).sine
        for column in 0..<columnCount {
            let columnHeight = computeCosSin(omegaX * Float(column)).sine
            definition.heights.append(rowHeight * columnHeight)
        }
    }
    return HeightField(definition)
}

private func heightFieldGridDefinition(rows: Int, columns: Int, scale: Vec3, holes: Bool) -> HeightFieldDef {
    var definition = HeightFieldDef()
    definition.countX = Int32(columns)
    definition.countZ = Int32(rows)
    definition.scale = scale
    definition.globalMinimumHeight = -256
    definition.globalMaximumHeight = 256
    for cell in 0..<((rows - 1) * (columns - 1)) {
        definition.materialIndices.append(holes && cell > 0 && cell % 16 == 0 ? 0xff : 0)
    }
    return definition
}
