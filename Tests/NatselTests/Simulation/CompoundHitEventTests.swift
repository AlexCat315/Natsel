import Testing
@testable import Natsel

// Hit-event material lookup must respect the compound child that participated in the contact.
// Ported from test/test_world.c TestCompoundHitEvents.
@Suite("Compound hit events")
struct CompoundHitEventTests {
    private static let hullMaterialA: UInt64 = 11
    private static let hullMaterialB: UInt64 = 22
    private static let sphereMaterial: UInt64 = 99
    private static let hullCenterX: Float = 3

    private func makeCompound() -> Compound? {
        var definition = CompoundDef()

        var materialA = SurfaceMaterial.defaultMaterial
        materialA.userMaterialId = Self.hullMaterialA

        var materialB = SurfaceMaterial.defaultMaterial
        materialB.userMaterialId = Self.hullMaterialB

        var hullA = CompoundHullDef()
        hullA.hull = makeBoxHull(1, 1, 1)
        hullA.transform = Transform(Vec3(-Self.hullCenterX, 0, 0), Quat.identity)
        hullA.material = materialA
        definition.hulls.append(hullA)

        var hullB = CompoundHullDef()
        hullB.hull = makeBoxHull(1, 1, 1)
        hullB.transform = Transform(Vec3(Self.hullCenterX, 0, 0), Quat.identity)
        hullB.material = materialB
        definition.hulls.append(hullB)

        return Compound(definition)
    }

    private func run(side: Int) -> (hitCount: Int, materialA: UInt64, materialB: UInt64)? {
        guard let compound = makeCompound() else { return nil }

        let expectedHullMaterial =
            side == 0 ? Self.hullMaterialA : Self.hullMaterialB
        let spawnX: Float = side == 0 ? -Self.hullCenterX : Self.hullCenterX

        let world = World()
        world.hitEventThreshold = 1

        // Static body holds the compound
        var def = BodyDef()
        def.type = .staticBody
        let compoundBody = world.createBody(def)
        _ = world.createCompoundShape(compoundBody, ShapeDef(), compound)

        // Sphere driven straight down onto the chosen child
        def = BodyDef()
        def.type = .dynamicBody
        def.gravityScale = 0
        def.position = Pos(spawnX, 3, 0)
        def.linearVelocity = Vec3(0, -30, 0)
        let sphereBody = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.enableHitEvents = true
        shapeDef.baseMaterial.userMaterialId = Self.sphereMaterial
        _ = world.createSphereShape(sphereBody, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        var hitCount = 0
        var capturedA: UInt64 = 0
        var capturedB: UInt64 = 0

        for _ in 0..<30 {
            world.step(1.0 / 60.0, subStepCount: 4)

            let events = world.contactEvents()
            if events.hitEvents.isEmpty == false && hitCount == 0 {
                capturedA = events.hitEvents[0].userMaterialIdA
                capturedB = events.hitEvents[0].userMaterialIdB
            }
            hitCount += events.hitEvents.count
        }

        // Verify the struck child's material is reported, not materials[0].
        _ = expectedHullMaterial
        return (hitCount, capturedA, capturedB)
    }

    @Test("hit event reports the struck compound child material")
    func compoundChildMaterial() {
        for side in 0..<2 {
            let expected: UInt64 = side == 0 ? Self.hullMaterialA : Self.hullMaterialB
            guard let result = run(side: side) else {
                Issue.record("compound creation failed")
                return
            }

            #expect(result.hitCount >= 1)
            // Sphere material on one side
            #expect(result.materialA == Self.sphereMaterial || result.materialB == Self.sphereMaterial)
            // Struck compound child's material on the other side.
            #expect(result.materialA == expected || result.materialB == expected)
        }
    }
}
