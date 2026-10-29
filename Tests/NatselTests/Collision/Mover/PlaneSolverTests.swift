import Testing
@testable import Natsel

@Suite("Mover plane solver repeatability")
struct PlaneSolverTests {
    @Test("ordered plane pushes and clipped vectors are repeatable")
    func repeatability() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            var digest = CollisionDigest()
            for i in 0..<1000 {
                var planes = (0..<(i % 8)).map { j in
                    var normal = Vec3(Float((i + j * 3) % 9 - 4), Float((i * 3 + j * 5) % 11 - 5), Float((i * 7 + j * 2) % 13 - 6))
                    if lengthSquared(normal) == 0 { normal.y = 1 }
                    return CollisionPlane(plane: Plane(normalize(normal), Float((i + j) % 17 - 8) * 0.2),
                                          pushLimit: j % 3 == 0 ? 0.15 : j % 3 == 1 ? Float.greatestFiniteMagnitude : 0,
                                          push: 3, clipVelocity: (i + j) % 2 == 0)
                }
                let target = Vec3(Float(i % 7 - 3) * 0.75, Float(i % 11 - 5) * 0.25, Float(i % 13 - 6) * 0.5)
                let result = solvePlanes(target, &planes)
                digest.add(result.delta); digest.add(UInt32(result.iterationCount))
                for plane in planes { digest.add(plane.push) }
                digest.add(clipVector(target, planes))
            }
            samples.append(UInt64(digest.value))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("parallel and game planes cover convergence cases")
    func convergence() {
        var planes = [CollisionPlane(plane: Plane(.axisZ, 0.5), pushLimit: .greatestFiniteMagnitude, push: 0, clipVelocity: false),
                      CollisionPlane(plane: Plane(.axisZ, 1), pushLimit: .greatestFiniteMagnitude, push: 0, clipVelocity: false)]
        let parallel = solvePlanes(.zero, &planes)
        #expect(parallel.iterationCount == 2)
        #expect(abs(parallel.delta.z - 1) < 0.0055)
        let target = Vec3(-2.5390625, 0, -73.6880798)
        planes[0].plane = Plane(Vec3(0, -0.23941046, 0.970918416), 0.390724182)
        planes[1].plane = Plane(.axisZ, 1.49998093)
        for i in planes.indices { planes[i].plane.offset -= dot(planes[i].plane.normal, target) }
        #expect(solvePlanes(.zero, &planes).iterationCount == 20)
    }
}
