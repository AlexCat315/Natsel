import Testing
@testable import Natsel

// Determinism scenarios from test/test_determinism.c and shared/determinism.c.
@Suite("World determinism C double parity")
struct WorldDeterminismTests {
    @Test("wave pile matches the C sleep step and transform hash")
    func wavePile() {
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
        #expect(sleepStep == 272)
        #expect(hash == 0xC8E5_F3D6)
    }

    @Test("mesh drop matches the C sleep step and transform hash")
    func meshDrop() {
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
        #expect(sleepStep == 218)
        #expect(hash == 0xD378_CF04)
    }

    @Test("query spawn matches the C state and query hashes")
    func querySpawn() {
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
        #expect(sleepStep == 242)
        #expect(hash == 0x4895_85E6)
        #expect(scene.queryHitCount == 59)
        #expect(scene.queryHash == 0x5B44_29DC)
    }
}
