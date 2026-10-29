import Testing
@testable import Natsel

@Suite("Hull")
struct HullTests {
    @Test("box hull has the expected topology and properties")
    func boxHullProperties() {
        let hull = makeBoxHull(1, 2, 3)

        #expect(hull.vertexCount == 8)
        #expect(hull.halfEdgeCount == 24)
        #expect(hull.faceCount == 6)

        #expect(hull.volume == 48)
        #expect(hull.surfaceArea == 88)
        #expect(hull.innerRadius == 1)

        #expect(hull.center == Vec3(0, 0, 0))
        #expect(hull.aabb.lowerBound == Vec3(-1, -2, -3))
        #expect(hull.aabb.upperBound == Vec3(1, 2, 3))

        #expect(isValidHull(hull))
    }

    @Test("box hull points use stable ordering")
    func boxHullPoints() {
        let hull = makeBoxHull(1, 2, 3)
        let points = hull.vertexPositions()

        #expect(points.count == 8)
        #expect(points[0] == Vec3(1, 2, 3))
        #expect(points[1] == Vec3(-1, 2, 3))
        #expect(points[2] == Vec3(-1, -2, 3))
        #expect(points[6] == Vec3(-1, -2, -3))
    }

    @Test("support vertex keeps the first maximum")
    func supportVertex() {
        let hull = makeBoxHull(1, 2, 3)

        #expect(findHullSupportVertex(hull, Vec3(1, 1, 1)) == 0)
        #expect(findHullSupportVertex(hull, Vec3(-1, -1, -1)) == 6)
        #expect(findHullSupportVertex(hull, Vec3(0, 0, -1)) == 4)
    }

    @Test("support face picks the aligned plane")
    func supportFace() {
        let hull = makeBoxHull(1, 2, 3)

        #expect(findHullSupportFace(hull, Vec3(1, 0, 0)) == 1)
        #expect(findHullSupportFace(hull, Vec3(0, -1, 0)) == 2)
    }

    @Test("hull content hash is stable and clones compare equal")
    func hullHash() {
        let a = makeBoxHull(1, 2, 3)
        let b = makeBoxHull(1, 2, 3)

        #expect(a.hash != 0)
        #expect(a.hash == b.hash)
        #expect(compareHullData(a, b))

        let clone = a.clone()
        #expect(clone != nil)
        #expect(clone!.hash == a.hash)
        #expect(compareHullData(a, clone!))
    }
}
