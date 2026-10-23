import Testing
internal import Foundation
@testable import Natsel

// Falling ragdoll determinism scene from shared/determinism.c and shared/human.c.
@Suite("Ragdoll determinism C parity")
struct RagdollDeterminismTests {
    private static let gridSize: Float = 15
    private static let gridCount = 2
    private static let groupSize = 2

    private struct Ragdoll {
        var humans: [Human] = []
    }





    @Test("human dump")
    func humanDump() {
        let n = Int(ProcessInfo.processInfo.environment["NATSEL_RAG_STEPS"] ?? "0") ?? 0
        let world = World()
        var human = Human()
        createHuman(&human, world, Pos(0, 15, 0), 5, 1, 0.7, 0, nil, false)
        for _ in 0..<n { world.step(1.0 / 60.0, subStepCount: 4) }
        func bits(_ f: Float) -> String { String(format: "%08X", f.bitPattern) }
        for (b, bone) in human.bones.enumerated() {
            let xf = world.bodyTransform(bone.bodyId)
            print("HUM \(b) \(bits(xf.p.x)) \(bits(xf.p.y)) \(bits(xf.p.z)) \(bits(xf.q.v.x)) \(bits(xf.q.v.y)) \(bits(xf.q.v.z)) \(bits(xf.q.s))")
        }
    }

    @Test("two humans dump")
    func twoHumansDump() {
        let n = Int(ProcessInfo.processInfo.environment["NATSEL_RAG_STEPS"] ?? "0") ?? 0
        let sep = Float(ProcessInfo.processInfo.environment["NATSEL_RAG_SEP"] ?? "0.75") ?? 0.75
        let world = World()
        var humans = [Human(), Human()]
        var p = Pos(-13 + 15 * 0.5, 15, -15 + 15 * 0.5)
        for i in 0..<2 {
            createHuman(&humans[i], world, p, 5, 1, 0.7, 0, nil, false)
            p.x += sep
        }
        for _ in 0..<n { world.step(1.0 / 60.0, subStepCount: 4) }
        func bits(_ f: Float) -> String { String(format: "%08X", f.bitPattern) }
        for (h, human) in humans.enumerated() {
            for (b, bone) in human.bones.enumerated() {
                let xf = world.bodyTransform(bone.bodyId)
                print("HUM2 \(h) \(b) \(bits(xf.p.x)) \(bits(xf.p.y)) \(bits(xf.p.z)) \(bits(xf.q.v.x)) \(bits(xf.q.v.y)) \(bits(xf.q.v.z)) \(bits(xf.q.s))")
            }
        }
    }

    @Test("two humans contacts dump")
    func twoHumansContactsDump() {
        let world = World()
        var humans = [Human(), Human()]
        var p = Pos(-13 + 15 * 0.5, 15, -15 + 15 * 0.5)
        for i in 0..<2 {
            createHuman(&humans[i], world, p, 5, 1, 0.7, 0, nil, false)
            p.x += 0.75
        }
        world.step(0.0, subStepCount: 4)
        func bits(_ f: Float) -> String { String(format: "%08X", f.bitPattern) }
        print("CONTACTS \(world.contacts.count)")
        for (c, contact) in world.contacts.enumerated() {
            var line = "\(c) shapes=\(contact.shapeIdA),\(contact.shapeIdB) flags=\(String(format: "%08X", contact.flags)) manifolds=\(contact.manifoldCount)"
            for m in 0..<Int(contact.manifoldCount) {
                let manifold = contact.manifolds[m]
                line += " [n \(bits(manifold.normal.x)) \(bits(manifold.normal.y)) \(bits(manifold.normal.z)) pc\(manifold.pointCount)"
                for q in 0..<Int(manifold.pointCount) {
                    let point = manifold.points[q]
                    line += " a(\(bits(point.anchorA.x)) \(bits(point.anchorA.y)) \(bits(point.anchorA.z))) b(\(bits(point.anchorB.x)) \(bits(point.anchorB.y)) \(bits(point.anchorB.z))) s\(bits(point.separation)) f\(String(format: "%08X", point.featureId))"
                }
                line += "]"
            }
            print("CT \(line)")
        }
    }

    @Test("two humans islands dump")
    func twoHumansIslandsDump() {
        let world = World()
        var humans = [Human(), Human()]
        var p = Pos(-13 + 15 * 0.5, 15, -15 + 15 * 0.5)
        for i in 0..<2 {
            createHuman(&humans[i], world, p, 5, 1, 0.7, 0, nil, false)
            p.x += 0.75
        }
        print("ISLANDS \(world.islands.count)")
        for (i, island) in world.islands.enumerated() where island.setIndex != B3.nullIndex {
            let bodies = island.bodies.map(String.init).joined(separator: ",")
            let joints = island.joints.map { String($0.jointId) }.joined(separator: ",")
            let contacts = island.contacts.map { String($0.contactId) }.joined(separator: ",")
            print("ISL \(i) set=\(island.setIndex) bodies=\(bodies) joints=\(joints) contacts=\(contacts)")
        }
        for (c, color) in world.constraintGraph.colors.enumerated() {
            let joints = color.jointSims.map { String($0.jointId) }.joined(separator: ",")
            let convex = color.convexContacts.map(String.init).joined(separator: ",")
            let contacts = color.contacts.map { String($0.contactId) }.joined(separator: ",")
            print("COLOR \(c) joints=\(joints) contacts=\(convex)|\(contacts)")
        }
    }

    @Test("joint dump")
    func jointDump() {
        let n = Int(ProcessInfo.processInfo.environment["NATSEL_RAG_STEPS"] ?? "0") ?? 0
        let world = World()
        var humans = [Human(), Human()]
        var p = Pos(-13 + 15 * 0.5, 15, -15 + 15 * 0.5)
        for i in 0..<2 {
            createHuman(&humans[i], world, p, 5, 1, 0.7, 0, nil, false)
            p.x += 0.75
        }
        for _ in 0..<n { world.step(1.0 / 60.0, subStepCount: 4) }
        func b(_ f: Float) -> String { String(format: "%08X", f.bitPattern) }
        func v(_ x: Vec3) -> String { "\(b(x.x)) \(b(x.y)) \(b(x.z))" }
        func q(_ x: Quat) -> String { "\(b(x.v.x)) \(b(x.v.y)) \(b(x.v.z)) \(b(x.s))" }
        for (ci, color) in world.constraintGraph.colors.enumerated() {
            for (j, js) in color.jointSims.enumerated() {
                var line = "J c\(ci) i\(j) id\(js.jointId) type\(js.type.rawValue) a\(js.bodyIdA) b\(js.bodyIdB) fa.p \(v(js.localFrameA.p)) fa.q \(q(js.localFrameA.q)) fb.p \(v(js.localFrameB.p)) fb.q \(q(js.localFrameB.q)) imA \(b(js.invMassA)) imB \(b(js.invMassB)) hz \(b(js.constraintHertz)) dp \(b(js.constraintDampingRatio)) ft \(b(js.forceThreshold)) tt \(b(js.torqueThreshold))"
                if js.type == .sphericalJoint {
                    let s = js.sphericalJoint
                    line += " dc \(v(s.deltaCenter)) sa \(v(s.swingAxis)) tj \(v(s.twistJacobian)) li \(v(s.linearImpulse)) si \(v(s.springImpulse)) mi \(v(s.motorImpulse)) lo \(b(s.lowerTwistImpulse)) up \(b(s.upperTwistImpulse)) sw \(b(s.swingImpulse))"
                } else if js.type == .revoluteJoint {
                    let s = js.revoluteJoint
                    line += " li \(v(s.linearImpulse)) pi \(b(s.perpImpulse.x)) pj \(b(s.perpImpulse.y)) si \(b(s.springImpulse)) mi \(b(s.motorImpulse)) lo \(b(s.lowerImpulse)) up \(b(s.upperImpulse)) dc \(v(s.deltaCenter)) da \(b(s.deltaAngle))"
                }
                print("JD \(line)")
            }
        }
    }







    @Test("ragdoll dump")
    func ragdollDump() {
        let n = Int(ProcessInfo.processInfo.environment["NATSEL_RAG_STEPS"] ?? "0") ?? 0
        let world = World()
        let data = createFallingRagdolls(world)
        for _ in 0..<n { world.step(1.0 / 60.0, subStepCount: 4) }
        func bits(_ f: Float) -> String { String(format: "%08X", f.bitPattern) }
        for (g, group) in data.groups.enumerated() {
            for (k, human) in group.humans.enumerated() {
                for (b, bone) in human.bones.enumerated() {
                    let xf = world.bodyTransform(bone.bodyId)
                    print("RAG \(g) \(k) \(b) \(bits(xf.p.x)) \(bits(xf.p.y)) \(bits(xf.p.z)) \(bits(xf.q.v.x)) \(bits(xf.q.v.y)) \(bits(xf.q.v.z)) \(bits(xf.q.s))")
                }
            }
        }
        print("RAG awake=\(world.awakeBodyCount())")
    }

    @Test("falling ragdolls match the C sleep step and transform hash")
    func fallingRagdolls() {
        let world = World()
        let data = createFallingRagdolls(world)
        var hash: UInt32 = 0
        var sleepStep = 0
        var stepCount = 0
        var done = false
        for _ in 0..<500 where !done {
            world.step(1.0 / 60.0, subStepCount: 4)
            if hash == 0 && world.bodyEvents().isEmpty && world.awakeBodyCount() == 0 {
                hash = hashInit
                for i in 0..<Self.gridCount {
                    for j in 0..<Self.gridCount {
                        for k in 0..<Self.groupSize {
                            let human = data.groups[i * Self.gridCount + j].humans[k]
                            for bone in human.bones {
                                hash = hashTransform(hash, world.bodyTransform(bone.bodyId))
                            }
                        }
                    }
                }
                sleepStep = stepCount
            }
            stepCount += 1
            done = hash != 0
        }

        #expect(done)
        #expect(sleepStep == 291)
        #expect(hash == 0xFF4C_605B)
    }

    private struct Group {
        var humans: [Human] = Array(repeating: Human(), count: 2)
    }

    private struct FallingRagdollData {
        var groups: [Group] = Array(repeating: Group(), count: 4)
        var gridMesh: TriangleMesh
        var torusMesh: TriangleMesh
    }

    private func createFallingRagdolls(_ world: World) -> FallingRagdollData {
        let halfMeshGridRows = 4
        let meshGridCellWidth = Self.gridSize / Float(2 * halfMeshGridRows)
        let gridMesh = createGridMesh(
            xCount: 2 * halfMeshGridRows, zCount: 2 * halfMeshGridRows,
            cellWidth: meshGridCellWidth, identifyEdges: true
        )!
        let torusMesh = createTorusMesh(
            radialResolution: 16, tubularResolution: 16,
            radius: 0.25 * Self.gridSize, thickness: 1
        )!

        let span = Self.gridSize * Float(Self.gridCount)
        var bodyDef = BodyDef()
        let shapeDef = ShapeDef()
        var data = FallingRagdollData(groups: [], gridMesh: gridMesh, torusMesh: torusMesh)
        bodyDef.position.x = -0.5 * span + 0.5 * Self.gridSize

        for i in 0..<Self.gridCount {
            bodyDef.position.z = -0.5 * span + 0.5 * Self.gridSize
            for j in 0..<Self.gridCount {
                let body = world.createBody(bodyDef)
                _ = world.createMeshShape(body, shapeDef, gridMesh)
                _ = world.createMeshShape(body, shapeDef, torusMesh)

                var group = Group()
                var position = Pos(
                    -0.5 * span + Float(Self.gridSize) * (Float(j) + 0.5),
                    15,
                    -0.5 * span + Float(Self.gridSize) * (Float(i) + 0.5)
                )
                for k in 0..<Self.groupSize {
                    createHuman(
                        &group.humans[k], world, position, 5, 1, 0.7,
                        i * Self.gridCount + j, nil, false
                    )
                    position.x += 0.75
                }
                data.groups.append(group)

                bodyDef.position.z += Self.gridSize
            }
            bodyDef.position.x += Self.gridSize
        }

        return data
    }
}
