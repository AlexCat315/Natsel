import Testing
@testable import Natsel

@Suite("Ragdoll determinism repeatability")
struct RagdollDeterminismTests {
    private static let gridSize: Float = 15
    private static let gridCount = 2
    private static let groupSize = 2

    private struct Ragdoll {
        var humans: [Human] = []
    }





    @Test("falling ragdolls are repeatable")
    func fallingRagdolls() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
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
            samples.append(UInt64(sleepStep))
            samples.append(UInt64(hash))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
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
        bodyDef.position.x = Double(-0.5 * span + 0.5 * Self.gridSize)

        for i in 0..<Self.gridCount {
            bodyDef.position.z = Double(-0.5 * span + 0.5 * Self.gridSize)
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

                bodyDef.position.z += Double(Self.gridSize)
            }
            bodyDef.position.x += Double(Self.gridSize)
        }

        return data
    }
}
