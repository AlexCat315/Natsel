import Testing
@testable import Natsel

func referenceHeightField(_ scenario: Int) -> HeightField? {
    let scale = Vec3(0.75, 1.25, 0.5)
    if scenario < 2 { return createGridHeightField(rowCount: 6, columnCount: 7, scale: scale, makeHoles: scenario == 1) }
    if scenario < 4 { return createWaveHeightField(rowCount: 6, columnCount: 7, scale: scale, rowFrequency: 0.2, columnFrequency: 0.3, makeHoles: scenario == 3) }
    return HeightField(referenceHeightDefinition(clockwise: scenario == 5))
}

func referenceHeightDefinition(clockwise: Bool) -> HeightFieldDef {
    let scale = Vec3(0.75, 1.25, 0.5)
    var definition = HeightFieldDef()
    definition.heights = (0..<42).map { Float(($0 * 17) % 29) * 0.125 - 1.5 }
    definition.materialIndices = (0..<30).map { $0 % 11 == 0 ? 255 : UInt8($0 % 3) }
    definition.scale = scale
    definition.countX = 7
    definition.countZ = 6
    definition.globalMinimumHeight = -1
    definition.globalMaximumHeight = 1
    definition.clockwiseWinding = clockwise
    return definition
}

@Suite("Height-field storage C parity")
struct HeightFieldEncodingTests {
    @Test("quantization, holes, adjacency, winding, and blob hashes match C", arguments: 0..<6)
    func encoding(scenario: Int) throws {
        let field = try #require(referenceHeightField(scenario))
        let hashes: [UInt64] = [0x14d9aa9f80fe533d, 0xce39ce5fa2b53933, 0x40a4a0573d99d92a,
                                0x9b31114bbd76481d, 0x8524a23e0dfb3200, 0x390ef118f5e80b58]
        let triangles: [UInt64] = [0xed23bf018bbdebd9, 0x10aba877579c1c89, 0xb82233a8552f2530,
                                   0x5a6d821e7feaaeda, 0x1644abef38f7789f, 0x65de5cab8a9ab1a7]
        #expect(field.byteCount == 280)
        #expect(field.hash == hashes[scenario])
        var digest = CollisionReferenceDigest()
        for i: Int32 in 0..<60 {
            if field.material(triangle: i) == 255 { continue }
            let triangle = field.triangle(i)
            for j in 0..<3 { digest.add(triangle.vertices[j]) }
            digest.add(UInt32(bitPattern: triangle.i1)); digest.add(UInt32(bitPattern: triangle.i2))
            digest.add(UInt32(bitPattern: triangle.i3)); digest.add(UInt32(bitPattern: triangle.flags))
        }
        #expect(digest.value == triangles[scenario])
    }
}
