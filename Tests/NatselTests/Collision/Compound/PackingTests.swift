import Testing
@testable import Natsel

@Suite("Compound storage C parity")
struct CompoundPackingTests {
    @Test("packing, sharing, materials, and rebuilt tree match the complete C blob", arguments: 0..<6)
    func packing(scenario: Int) throws {
        let compound = try #require(referenceCompound(scenario))
        let sizes = [224,368,1072,1000,2344,4480]
        let hashes: [UInt64] = [0x2d78578595d49b96,0x9aa9f793cb549295,0xfe5041f9169e658c,
                               0x7fe9e8b02866865d,0x153c48b802554ad2,0xd731e71d260bee85]
        let bytes = compound.serializedBytes()
        #expect(bytes.count == sizes[scenario])
        let hash = bytes.withUnsafeBufferPointer { hash64NonZero($0.baseAddress!, Int32($0.count)) }
        #expect(hash == hashes[scenario])
        #expect(compound.header.treeHeight == [0,1,1,1,6,7][scenario])
        #expect(compound.header.sharedHullCount == [0,0,1,0,1,2][scenario])
        #expect(compound.header.sharedMeshCount == [0,0,0,1,1,1][scenario])
        let loaded = try #require(Compound(serializedBytes: bytes))
        #expect(loaded.serializedBytes() == bytes)
        #expect(loaded.childCount == compound.childCount)
        #expect(loaded.materials == compound.materials)
    }

    @Test("embedded geometry outlives its compound without retaining the compound object")
    func lifetime() throws {
        var compound: Compound? = try #require(referenceCompound(2))
        weak var parent = compound
        let child = compound!.child(at: 0)
        guard case .hull(let hull) = child.geometry else { Issue.record("expected hull"); return }
        let expected = hull.vertexPositions()
        compound = nil
        #expect(parent == nil)
        #expect(hull.vertexPositions() == expected)
    }

    @Test("invalid definitions and damaged compound headers are rejected")
    func rejected() throws {
        #expect(Compound(CompoundDef()) == nil)
        let compound = try #require(referenceCompound(4))
        var bytes = compound.serializedBytes()
        bytes[0] ^= 1
        #expect(Compound(serializedBytes: bytes) == nil)
        #expect(Compound(serializedBytes: Array(bytes.prefix(8))) == nil)
        var damaged = compound.serializedBytes()
        let hullOffset = Int(compound.header.hullOffset) + MemoryLayout<PackedHullInstance>.offset(of: \PackedHullInstance.hullOffset)!
        for i in 0..<4 { damaged[hullOffset + i] = 255 }
        #expect(Compound(serializedBytes: damaged) == nil)
        var definition = CompoundDef()
        definition.hulls = [CompoundHullDef()]
        #expect(Compound(definition) == nil)
    }
}
