import Testing
@testable import Natsel

@Suite("Upstream primitive mover cases")
struct PrimitivePlaneTests {
    @Test("sphere separated, touching, and deep overlap", arguments: 0..<3)
    func sphere(index: Int) {
        let movers = [Capsule(center1: Vec3(4, 3, 0), center2: Vec3(6, 3, 0), radius: 0.2),
                      Capsule(center1: Vec3(-1, 0.6, 0), center2: Vec3(1, 0.6, 0), radius: 0.2),
                      Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.2)]
        var plane = PlaneResult()
        let count = collideMoverAndSphere(&plane, Sphere(center: .zero, radius: 0.5), movers[index])
        #expect(count == (index == 0 ? 0 : 1))
        if index > 0 {
            #expect(isNormalized(plane.plane.normal))
            #expect(abs(plane.plane.offset - (index == 1 ? 0.1 : 0.7)) < 0.00001)
            if index == 1 { #expect(plane.plane.normal.y > 0.99) }
            if index == 2 { #expect(abs(plane.plane.normal.x) < 0.00001) }
        }
    }

    @Test("capsule separated, touching, crossing, and parallel overlap", arguments: 0..<4)
    func capsule(index: Int) {
        let movers = [Capsule(center1: Vec3(-1, 5, 0), center2: Vec3(1, 5, 0), radius: 0.2),
                      Capsule(center1: Vec3(-1, 0.4, 0), center2: Vec3(1, 0.4, 0), radius: 0.2),
                      Capsule(center1: Vec3(0, 0, -1), center2: Vec3(0, 0, 1), radius: 0.2),
                      Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.2)]
        var plane = PlaneResult()
        let shape = Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.3)
        let count = collideMoverAndCapsule(&plane, shape, movers[index])
        #expect(count == (index == 0 ? 0 : 1))
        if index > 0 {
            #expect(isNormalized(plane.plane.normal))
            #expect(abs(plane.plane.offset - (index == 1 ? 0.1 : 0.5)) < 0.00001)
            if index == 1 { #expect(plane.plane.normal.y > 0.99) }
            if index >= 2 { #expect(abs(plane.plane.normal.x) < 0.00001) }
            if index == 2 { #expect(abs(plane.plane.normal.z) < 0.00001) }
        }
    }

    @Test("hull separated, touching, and deep overlap", arguments: 0..<3)
    func hull(index: Int) {
        let movers = [Capsule(center1: Vec3(-0.3, 5, 0), center2: Vec3(0.3, 5, 0), radius: 0.2),
                      Capsule(center1: Vec3(-0.3, 0.6, 0), center2: Vec3(0.3, 0.6, 0), radius: 0.2),
                      Capsule(center1: Vec3(-0.2, 0, 0), center2: Vec3(0.2, 0, 0), radius: 0.1)]
        var plane = PlaneResult()
        #expect(collideMoverAndHull(&plane, makeBoxHull(0.5, 0.5, 0.5), movers[index]) == (index == 1 ? 1 : 0))
        if index == 1 {
            #expect(isNormalized(plane.plane.normal))
            #expect(plane.plane.normal.y > 0.99)
            #expect(abs(plane.plane.offset - 0.1) < 0.0001)
        }
    }
}
