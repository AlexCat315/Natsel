import Testing
@testable import Natsel

@Suite("Mover plane solver")
struct MoverPlaneSolverTests {
    @Test("parallel planes resolve in one pass per plane")
    func parallelPlanes() {
        var planes = [CollisionPlane](repeating: CollisionPlane(), count: 3)
        planes[0].plane.normal = Vec3(0, 0, 1)
        planes[0].plane.offset = 0.5
        planes[0].pushLimit = Float.greatestFiniteMagnitude
        planes[1].plane.normal = Vec3(0, 0, 1)
        planes[1].plane.offset = 1.0
        planes[1].pushLimit = Float.greatestFiniteMagnitude

        let result = solvePlanes(Vec3(), &planes)

        #expect(result.iterationCount == 2)
        #expect(absFloat(result.delta.z - 1.0) < 0.0055)
    }

    @Test("a deep target takes the full iteration budget")
    func gamePlanes() {
        // This scenario takes many iterations because the target is deep into the plane.
        var planes = [CollisionPlane](repeating: CollisionPlane(), count: 3)
        planes[0].plane.normal = Vec3(0, -0.23941046, 0.970918416)
        planes[0].plane.offset = 0.390724182
        planes[0].pushLimit = Float.greatestFiniteMagnitude
        planes[1].plane.normal = Vec3(0, 0, 1)
        planes[1].plane.offset = 1.49998093
        planes[1].pushLimit = Float.greatestFiniteMagnitude

        var target = Vec3(-2.5390625, 0, -73.6880798)
        planes[0].plane.offset -= dot(planes[0].plane.normal, target)
        planes[1].plane.offset -= dot(planes[1].plane.normal, target)
        target = Vec3()

        let result = solvePlanes(target, &planes)

        #expect(result.iterationCount == 20)
    }
}
