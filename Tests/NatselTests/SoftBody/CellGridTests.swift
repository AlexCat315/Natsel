import Testing
@testable import Natsel

@Suite("Soft contact cell grid")
struct CellGridTests {
    @Test("cell visits preserve ascending feature order with repeated and unordered inserts")
    func orderedFeatures() {
        var grid = CellGrid(cellSize: 1)
        let point = Vec3(0.25, 0.25, 0.25)
        for item: Int32 in [7, 2, -3, 2] { grid.insert(AABB(point, point), item) }
        grid.finish()
        var visited: [Int32] = []
        grid.visit(point) { visited.append($0) }
        #expect(visited == [-3, 2, 2, 7])
    }

    @Test("negative coordinates and exact cell boundaries keep all spanning features")
    func boundaries() {
        var grid = CellGrid(cellSize: 1)
        grid.insert(AABB(Vec3(-1, -1, -1), Vec3(1, 1, 1)), 4)
        grid.insert(AABB(Vec3(-0.001, -0.001, -0.001), .zero), 2)
        grid.insert(AABB(.zero, .zero), 1)
        grid.finish()
        func visit(_ p: Vec3) -> [Int32] {
            var result: [Int32] = []
            grid.visit(p) { result.append($0) }
            return result
        }
        #expect(visit(Vec3(-0.5, -0.5, -0.5)) == [2, 4])
        #expect(visit(.zero) == [1, 2, 4])
        #expect(visit(Vec3(1, 1, 1)) == [4])
        #expect(visit(Vec3(2, 2, 2)).isEmpty)
    }

    @Test("finishing again after insertion retains previous cells and feature order")
    func rebuilding() {
        var grid = CellGrid(cellSize: 1)
        grid.insert(AABB(.zero, .zero), 5)
        grid.finish()
        grid.insert(AABB(.zero, .zero), 3)
        let remote = Vec3(4, 5, 6)
        grid.insert(AABB(remote, remote), 1)
        grid.finish()
        var origin: [Int32] = []
        var distant: [Int32] = []
        grid.visit(.zero) { origin.append($0) }
        grid.visit(remote) { distant.append($0) }
        #expect(origin == [3, 5])
        #expect(distant == [1])
    }
}
