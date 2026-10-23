/// Immutable, quantized terrain shared by height-field shapes.
public final class HeightField: @unchecked Sendable {
    let base: UnsafeMutableRawPointer
    public let byteCount: Int32

    public init?(_ definition: HeightFieldDef) {
        let columns = Int(definition.countX)
        let rows = Int(definition.countZ)
        guard columns >= 2, rows >= 2, definition.heights.count == columns * rows,
              definition.materialIndices.isEmpty || definition.materialIndices.count == (columns - 1) * (rows - 1),
              definition.scale.x > 0, definition.scale.y > 0, definition.scale.z > 0,
              isValidVec3(definition.scale), definition.globalMinimumHeight.isFinite,
              definition.globalMaximumHeight.isFinite, definition.globalMinimumHeight <= definition.globalMaximumHeight,
              definition.heights.allSatisfy(\.isFinite) else { return nil }
        let heightCount = columns * rows
        let cellCount = (columns - 1) * (rows - 1)
        let heightsOffset = alignHeightFieldBytes(MemoryLayout<HeightFieldData>.stride)
        let materialOffset = heightsOffset + alignHeightFieldBytes(2 * heightCount)
        let flagsOffset = materialOffset + alignHeightFieldBytes(cellCount)
        let size = flagsOffset + alignHeightFieldBytes(2 * cellCount)
        byteCount = Int32(size)
        base = b3AllocZero(size)!
        let header = base.bindMemory(to: HeightFieldData.self, capacity: 1)
        header.initialize(to: HeightFieldData())
        header.pointee.version = B3.heightFieldVersion
        header.pointee.byteCount = byteCount
        header.pointee.scale = definition.scale
        header.pointee.columnCount = definition.countX
        header.pointee.rowCount = definition.countZ
        header.pointee.heightsOffset = Int32(heightsOffset)
        header.pointee.materialOffset = Int32(materialOffset)
        header.pointee.flagsOffset = Int32(flagsOffset)
        header.pointee.clockwise = definition.clockwiseWinding ? 1 : 0
        header.pointee.minHeight = definition.globalMinimumHeight
        header.pointee.maxHeight = definition.globalMaximumHeight
        let height = maxFloat(definition.globalMaximumHeight - definition.globalMinimumHeight, B3.linearSlop)
        let heightScale = height / Float(UInt16.max)
        header.pointee.heightScale = heightScale
        let compressed = (base + heightsOffset).bindMemory(to: UInt16.self, capacity: heightCount)
        var lower = definition.globalMaximumHeight
        var upper = definition.globalMinimumHeight
        let inverse: Float = 1 / heightScale
        for i in 0..<heightCount {
            let clamped = clampFloat(definition.heights[i], definition.globalMinimumHeight, definition.globalMaximumHeight)
            let scaled = (clamped - definition.globalMinimumHeight) * inverse
            compressed[i] = UInt16(minFloat(scaled, Float(UInt16.max)))
            lower = minFloat(lower, clamped)
            upper = maxFloat(upper, clamped)
        }
        var decompressed: [Float] = []
        for i in 0..<heightCount { decompressed.append(definition.globalMinimumHeight + heightScale * Float(compressed[i])) }
        let materials = definition.materialIndices.isEmpty ? [UInt8](repeating: 0, count: cellCount) : definition.materialIndices
        materials.withUnsafeBufferPointer { buffer in
            (base + materialOffset).bindMemory(to: UInt8.self, capacity: cellCount).initialize(from: buffer.baseAddress!, count: cellCount)
        }
        header.pointee.aabb = AABB(Vec3(0, definition.scale.y * lower, 0),
            Vec3(definition.scale.x * Float(columns - 1), definition.scale.y * upper, definition.scale.z * Float(rows - 1)))
        let flags = heightFieldAdjacency(heights: decompressed, materials: materials, columns: columns, rows: rows, scale: definition.scale)
        flags.withUnsafeBufferPointer { buffer in
            (base + flagsOffset).bindMemory(to: UInt8.self, capacity: flags.count).initialize(from: buffer.baseAddress!, count: flags.count)
        }
        header.pointee.hash = hash64NonZero(base.assumingMemoryBound(to: UInt8.self), byteCount)
    }

    deinit { b3Free(base, Int(byteCount)) }
    var header: HeightFieldData { base.assumingMemoryBound(to: HeightFieldData.self).pointee }
    var compressedHeights: UnsafeBufferPointer<UInt16> {
        UnsafeBufferPointer(start: (base + Int(header.heightsOffset)).assumingMemoryBound(to: UInt16.self), count: columnCount * rowCount)
    }
    var materialIndices: UnsafeBufferPointer<UInt8> {
        UnsafeBufferPointer(start: (base + Int(header.materialOffset)).assumingMemoryBound(to: UInt8.self), count: (columnCount - 1) * (rowCount - 1))
    }
    var edgeFlags: UnsafeBufferPointer<UInt8> {
        UnsafeBufferPointer(start: (base + Int(header.flagsOffset)).assumingMemoryBound(to: UInt8.self), count: 2 * (columnCount - 1) * (rowCount - 1))
    }
    public var columnCount: Int { Int(header.columnCount) }
    public var rowCount: Int { Int(header.rowCount) }
    public var bounds: AABB { header.aabb }
    public var scale: Vec3 { header.scale }
    public var hash: UInt64 { header.hash }
}

private func alignHeightFieldBytes(_ count: Int) -> Int { (count + 7) & ~7 }
