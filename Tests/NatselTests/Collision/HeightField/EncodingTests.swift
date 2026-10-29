import Testing
@testable import Natsel

func makeTestHeightField(_ scenario: Int) -> HeightField? {
    let scale = Vec3(0.75, 1.25, 0.5)
    if scenario < 2 { return createGridHeightField(rowCount: 6, columnCount: 7, scale: scale, makeHoles: scenario == 1) }
    if scenario < 4 { return createWaveHeightField(rowCount: 6, columnCount: 7, scale: scale, rowFrequency: 0.2, columnFrequency: 0.3, makeHoles: scenario == 3) }
    return HeightField(makeTestHeightDefinition(clockwise: scenario == 5))
}

func makeTestHeightDefinition(clockwise: Bool) -> HeightFieldDef {
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

@Suite("Height-field storage repeatability")
struct HeightFieldEncodingTests {
    @Test("quantization, holes, adjacency, winding, and blob hashes are repeatable", arguments: 0..<6)
    func encoding(scenario: Int) throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let field = try #require(makeTestHeightField(scenario))
            #expect(field.byteCount == 280)
            samples.append(UInt64(field.hash))
            var digest = CollisionDigest()
            for i: Int32 in 0..<60 {
                if field.material(triangle: i) == 255 { continue }
                let triangle = field.triangle(i)
                for j in 0..<3 { digest.add(triangle.vertices[j]) }
                digest.add(UInt32(bitPattern: triangle.i1)); digest.add(UInt32(bitPattern: triangle.i2))
                digest.add(UInt32(bitPattern: triangle.i3)); digest.add(UInt32(bitPattern: triangle.flags))
            }
            samples.append(UInt64(digest.value))
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
