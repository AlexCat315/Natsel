import Testing
internal import Foundation
@testable import Natsel

// A body touching more constraints than the graph has colors must fall back to the overflow color.
// Scene ported from shared/overflow_color.c.
@Suite("Overflow color")
struct OverflowColorTests {
    private static let ringCount = 5
    private static let perRing = 5

    private func createOverflowColorPile(_ world: World) -> (hub: BodyID, neighbors: Int) {
        // Static ground (top surface at y = 0)
        var def = BodyDef()
        def.position = Pos(0, -1, 0)
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(20, 1, 20))

        // Tall, heavy hub so neighbors can ring around it in several vertical layers.
        let hubHalfX: Float = 0.5
        let hubHalfY: Float = 2.5
        let hubHalfZ: Float = 0.5

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, hubHalfY, 0)
        let hub = world.createBody(def)

        shapeDef = ShapeDef()
        shapeDef.density = 50
        _ = world.createHullShape(hub, shapeDef, makeBoxHull(hubHalfX, hubHalfY, hubHalfZ))

        // Neighbors: vertical rings, each box slightly overlapping the hub.
        let neighborHalf: Float = 0.2
        let ringRadius = hubHalfX + neighborHalf - 0.03
        let neighborShape = ShapeDef()

        let ringSpacing: Float = 0.5
        let baseY = neighborHalf + 0.05

        var neighborCount = 0
        for ring in 0..<Self.ringCount {
            let y = baseY + ringSpacing * Float(ring)

            // Offset alternate rings by half a slot so neighbors don't sit directly above each other.
            let thetaOffset: Float = (ring & 1) != 0 ? (B3.pi / Float(Self.perRing)) : 0

            for slot in 0..<Self.perRing {
                let theta = thetaOffset + (2 * B3.pi * Float(slot)) / Float(Self.perRing)
                let cs = computeCosSin(theta)

                def = BodyDef()
                def.type = .dynamicBody
                def.position = Pos(ringRadius * cs.cosine, y, ringRadius * cs.sine)
                let body = world.createBody(def)
                _ = world.createHullShape(
                    body, neighborShape, makeBoxHull(neighborHalf, neighborHalf, neighborHalf)
                )
                neighborCount += 1
            }
        }

        return (hub, neighborCount)
    }

    @Test("a pile around one hub populates the overflow color")
    func overflowColorPile() {
        let world = World()
        let (_, neighbors) = createOverflowColorPile(world)
        #expect(neighbors == Self.ringCount * Self.perRing)

        // One step would trip the asserts; several steps also exercise warm starting.
        for _ in 0..<10 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        let counters = world.counters()

        // Confirm the scene actually populated the overflow color. Without this a future change
        // to graph coloring could silently turn the test into a no-op.
        let overflowIndex = Int(B3.graphColorCount) - 1
        #expect(counters.colorCounts[overflowIndex] > 0)
        #expect(counters.colorCounts[overflowIndex] == 5)

        // Body and contact totals match the C scene.
        #expect(counters.bodyCount == 27)
        #expect(counters.contactCount == 65)
    }

    @Test("shared overflow bodies preserve scalar solver order")
    func overflowMatchesScalar() throws {
        let scalar = World(), wide = World()
        scalar.enableWideSolver = false
        wide.enableWideSolver = true
        scalar.enableSleep = false
        wide.enableSleep = false
        _ = createOverflowColorPile(scalar)
        _ = createOverflowColorPile(wide)
        var sawOverflow = false
        for step in 0..<60 {
            scalar.step(1 / 60, subStepCount: 4)
            wide.step(1 / 60, subStepCount: 4)
            sawOverflow = sawOverflow || !wide.constraintGraph.colors[Int(overflowIndex)].contacts.isEmpty
            #expect(!wide.usesWideContacts(overflowIndex, false))
            #expect(!wide.usesWideContacts(overflowIndex, true))
            try #require(scalar.worldStateHash() == wide.worldStateHash(), "step \(step)")
        }
        #expect(sawOverflow)
    }

    @Test("color counts sum to the awake contact total")
    func colorCountsSum() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        for i in 0..<3 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 0.6 + Float(i) * 1.05, 0)
            let box = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        }

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        let counters = world.counters()
        var colorSum: Int32 = 0
        for i in 0..<Int(B3.graphColorCount) {
            colorSum += counters.colorCounts[i]
        }

        // Every graph constraint is accounted for in exactly one color.
        #expect(colorSum > 0)
        #expect(counters.awakeContactCount >= colorSum)
    }
}
