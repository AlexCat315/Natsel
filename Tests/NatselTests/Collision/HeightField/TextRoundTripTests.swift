private import Foundation
import Testing
@testable import Natsel

@Suite("Height-field text format")
struct HeightFieldTextRoundTripTests {
    @Test("authored text and loaded terrain round-trip through the text format")
    func roundTrip() throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let definition = makeTestHeightDefinition(clockwise: true)
            let text = try #require(HeightFieldText.encode(definition))
            let bytes = Array(text.utf8)
            #expect(bytes.count == 655)
            let digest = bytes.withUnsafeBufferPointer { hash64NonZero($0.baseAddress!, Int32($0.count)) }
            samples.append(UInt64(digest))
            let decoded = try #require(HeightFieldText.decode(text))
            #expect(decoded == definition)
            let path = NSTemporaryDirectory() + "natsel-height-" + UUID().uuidString + ".txt"
            defer { try? FileManager.default.removeItem(atPath: path) }
            try definition.write(to: path)
            let field = try HeightField.load(from: path)
            samples.append(UInt64(field.hash))
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("truncated and invalid height data is rejected")
    func malformed() {
        #expect(HeightFieldText.decode("") == nil)
        #expect(HeightFieldText.decode("7 6 1 1 1 -1 1 0") == nil)
        #expect(HeightFieldText.decode("-7 6 1 1 1 -1 1 0") == nil)
        #expect(HeightField(HeightFieldDef()) == nil)
        var definition = makeTestHeightDefinition(clockwise: false)
        definition.heights[0] = .nan
        #expect(HeightField(definition) == nil)
        definition.heights[0] = 0
        definition.materialIndices.removeLast()
        #expect(HeightField(definition) == nil)
    }
}
