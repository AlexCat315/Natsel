import Testing
@testable import Natsel

@Suite("World determinism repeatability")
struct WorldDeterminismTests {
    @Test("wave pile is repeatable")
    func wavePile() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            var scene = createWavePile(world)
            var stepCount = 0
            var sleepStep = 0
            var hash: UInt32 = 0
            var done = false
            for _ in 0..<500 where !done {
                world.step(1.0 / 60.0, subStepCount: 4)
                done = updateWavePile(world, &scene, stepCount: &stepCount, sleepStep: &sleepStep, hash: &hash)
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

    @Test("mesh drop is repeatable")
    func meshDrop() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            var scene = createMeshDrop(world, Pos())
            var stepCount = 0
            var sleepStep = 0
            var hash: UInt32 = 0
            var done = false
            for _ in 0..<400 where !done {
                world.step(1.0 / 60.0, subStepCount: 4)
                done = updateMeshDrop(world, &scene, stepCount: &stepCount, sleepStep: &sleepStep, hash: &hash)
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

    @Test("query spawn is repeatable")
    func querySpawn() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            var scene = createQuerySpawn(world)
            var random = SceneRandom(seed: 71_689)
            var stepCount = 0
            var sleepStep = 0
            var hash: UInt32 = 0
            var done = false
            for _ in 0..<1000 where !done {
                world.step(1.0 / 60.0, subStepCount: 4)
                done = updateQuerySpawn(
                    world, &scene, &random, stepCount: &stepCount, sleepStep: &sleepStep, hash: &hash
                )
            }
            #expect(done)
            #expect(scene.spawnCount == 50)
            samples.append(UInt64(sleepStep))
            samples.append(UInt64(hash))
            samples.append(UInt64(scene.queryHitCount))
            samples.append(UInt64(scene.queryHash))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
