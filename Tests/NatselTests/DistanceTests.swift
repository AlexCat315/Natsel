import Testing
@testable import Natsel

@Suite("Distance")
struct DistanceTests {
    private func makeBox(_ hx: Float, _ hy: Float, _ hz: Float, _ offset: Vec3) -> [Vec3] {
        [
            Vec3(hx, hy, hz) + offset,
            Vec3(-hx, hy, hz) + offset,
            Vec3(-hx, -hy, hz) + offset,
            Vec3(hx, -hy, hz) + offset,
            Vec3(hx, hy, -hz) + offset,
            Vec3(-hx, hy, -hz) + offset,
            Vec3(-hx, -hy, -hz) + offset,
            Vec3(hx, -hy, -hz) + offset,
        ]
    }

    @Test("box-box distance is repeatable")
    func boxBoxDistance() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let a = makeBox(1, 1, 1, Vec3(0, 0, 0))
            let b = makeBox(1, 1, 1, Vec3(0, 0, 0))

            var cache = SimplexCache()
            let output = a.withUnsafeBufferPointer { pa in
                b.withUnsafeBufferPointer { pb in
                    var input = DistanceInput()
                    input.proxyA = ShapeProxy(points: pa.baseAddress, count: 8, radius: 0)
                    input.proxyB = ShapeProxy(points: pb.baseAddress, count: 8, radius: 0)
                    input.transform = Transform(Vec3(3, 0, 0), Quat.identity)
                    input.useRadii = false
                    return shapeDistance(input, &cache)
                }
            }

            samples.append(UInt64(output.distance.bitPattern))
            samples.append(UInt64(output.normal.x.bitPattern))
            #expect(output.normal.y.bitPattern == 0)
            #expect(output.normal.z.bitPattern == 0)

            #expect(output.pointA == Vec3(1, 1, 1))
            #expect(output.pointB == Vec3(2, 1, 1))

            #expect(output.iterations == 1)
            #expect(cache.count == 1)
            #expect(cache.indexA[0] == 0)
            #expect(cache.indexB[0] == 1)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("radii push the witness points to the surfaces")
    func pointRadii() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let a: [Vec3] = [Vec3(0, 0, 0)]
            let b: [Vec3] = [Vec3(0, 0, 0)]

            var cache = SimplexCache()
            let output = a.withUnsafeBufferPointer { pa in
                b.withUnsafeBufferPointer { pb in
                    var input = DistanceInput()
                    input.proxyA = ShapeProxy(points: pa.baseAddress, count: 1, radius: 1)
                    input.proxyB = ShapeProxy(points: pb.baseAddress, count: 1, radius: 1)
                    input.transform = Transform(Vec3(3, 0, 0), Quat.identity)
                    input.useRadii = true
                    return shapeDistance(input, &cache)
                }
            }

            samples.append(UInt64(output.distance.bitPattern))
            samples.append(UInt64(output.normal.x.bitPattern))
            samples.append(UInt64(output.pointA.x.bitPattern))
            samples.append(UInt64(output.pointB.x.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("overlapping boxes report zero distance")
    func overlap() {
        let a = makeBox(1, 1, 1, Vec3(0, 0, 0))
        let b = makeBox(1, 1, 1, Vec3(0, 0, 0))

        var cache = SimplexCache()
        let output = a.withUnsafeBufferPointer { pa in
            b.withUnsafeBufferPointer { pb in
                var input = DistanceInput()
                input.proxyA = ShapeProxy(points: pa.baseAddress, count: 8, radius: 0)
                input.proxyB = ShapeProxy(points: pb.baseAddress, count: 8, radius: 0)
                input.transform = Transform(Vec3(0.5, 0, 0), Quat.identity)
                input.useRadii = false
                return shapeDistance(input, &cache)
            }
        }

        #expect(output.distance == 0)
        #expect(output.normal == Vec3(0, 0, 0))
    }
}

@Suite("Shape cast")
struct ShapeCastTests {
    @Test("box cast is repeatable")
    func boxCast() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let a = [
                Vec3(1, 1, 1), Vec3(-1, 1, 1), Vec3(-1, -1, 1), Vec3(1, -1, 1),
                Vec3(1, 1, -1), Vec3(-1, 1, -1), Vec3(-1, -1, -1), Vec3(1, -1, -1),
            ]
            let b = a

            let output = a.withUnsafeBufferPointer { pa in
                b.withUnsafeBufferPointer { pb in
                    var input = ShapeCastPairInput()
                    input.proxyA = ShapeProxy(points: pa.baseAddress, count: 8, radius: 0)
                    input.proxyB = ShapeProxy(points: pb.baseAddress, count: 8, radius: 0)
                    input.transform = Transform(Vec3(3, 0, 0), Quat.identity)
                    input.translationB = Vec3(-5, 0, 0)
                    input.maxFraction = 1
                    input.canEncroach = false
                    return shapeCast(input)
                }
            }

            #expect(output.hit)
            samples.append(UInt64(output.fraction.bitPattern))
            #expect(output.normal == Vec3(1, 0, 0))
            #expect(output.point == Vec3(1, 1, 1))
            #expect(output.iterations == 2)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("sphere-like point pair cast is repeatable")
    func pointCast() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let a: [Vec3] = [Vec3(0, 0, 0)]
            let b: [Vec3] = [Vec3(0, 0, 0)]

            let output = a.withUnsafeBufferPointer { pa in
                b.withUnsafeBufferPointer { pb in
                    var input = ShapeCastPairInput()
                    input.proxyA = ShapeProxy(points: pa.baseAddress, count: 1, radius: 1)
                    input.proxyB = ShapeProxy(points: pb.baseAddress, count: 1, radius: 1)
                    input.transform = Transform(Vec3(5, 0, 0), Quat.identity)
                    input.translationB = Vec3(-10, 0, 0)
                    input.maxFraction = 1
                    input.canEncroach = false
                    return shapeCast(input)
                }
            }

            #expect(output.hit)
            samples.append(UInt64(output.fraction.bitPattern))
            samples.append(UInt64(output.normal.x.bitPattern))
            samples.append(UInt64(output.point.x.bitPattern))
            #expect(output.iterations == 2)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("receding cast misses")
    func recedingCast() {
        let a: [Vec3] = [Vec3(0, 0, 0)]
        let b: [Vec3] = [Vec3(0, 0, 0)]

        let output = a.withUnsafeBufferPointer { pa in
            b.withUnsafeBufferPointer { pb in
                var input = ShapeCastPairInput()
                input.proxyA = ShapeProxy(points: pa.baseAddress, count: 1, radius: 1)
                input.proxyB = ShapeProxy(points: pb.baseAddress, count: 1, radius: 1)
                input.transform = Transform(Vec3(5, 0, 0), Quat.identity)
                input.translationB = Vec3(10, 0, 0)
                input.maxFraction = 1
                input.canEncroach = false
                return shapeCast(input)
            }
        }

        #expect(!output.hit)
        #expect(output.fraction == 0)
    }
}

@Suite("Time of impact")
struct TimeOfImpactTests {
    private let box: [Vec3] = [
        Vec3(1, 1, 1), Vec3(-1, 1, 1), Vec3(-1, -1, 1), Vec3(1, -1, 1),
        Vec3(1, 1, -1), Vec3(-1, 1, -1), Vec3(-1, -1, -1), Vec3(1, -1, -1),
    ]

    private func run(_ sweepB: Sweep) -> TOIOutput {
        box.withUnsafeBufferPointer { pa in
            box.withUnsafeBufferPointer { pb in
                var input = TOIInput()
                input.proxyA = ShapeProxy(points: pa.baseAddress, count: 8, radius: 0)
                input.proxyB = ShapeProxy(points: pb.baseAddress, count: 8, radius: 0)
                input.sweepA = Sweep(localCenter: Vec3(), c1: Vec3(), c2: Vec3(), q1: Quat.identity, q2: Quat.identity)
                input.sweepB = sweepB
                input.maxFraction = 1
                return timeOfImpact(input)
            }
        }
    }

    @Test("translating box TOI is repeatable")
    func translate() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let output = run(Sweep(
                localCenter: Vec3(), c1: Vec3(5, 0, 0), c2: Vec3(0, 0, 0),
                q1: Quat.identity, q2: Quat.identity
            ))

            #expect(output.state == .hit)
            samples.append(UInt64(output.fraction.bitPattern))
            samples.append(UInt64(output.point.x.bitPattern))
            samples.append(UInt64(output.point.y.bitPattern))
            samples.append(UInt64(output.point.z.bitPattern))
            #expect(output.normal == Vec3(1, 0, 0))
            samples.append(UInt64(output.distance.bitPattern))
            #expect(output.distanceIterations == 2)
            #expect(output.pushBackIterations == 1)
            #expect(output.rootIterations == 2)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("receding box reports separated")
    func recede() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let output = run(Sweep(
                localCenter: Vec3(), c1: Vec3(5, 0, 0), c2: Vec3(10, 0, 0),
                q1: Quat.identity, q2: Quat.identity
            ))

            #expect(output.state == .separated)
            samples.append(UInt64(output.fraction.bitPattern))
            samples.append(UInt64(output.point.x.bitPattern))
            samples.append(UInt64(output.distance.bitPattern))
            #expect(output.distanceIterations == 1)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("rotating box TOI is repeatable")
    func rotate() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let q = makeQuatFromAxisAngle(Vec3(0, 0, 1), 0.785398163)
            let output = run(Sweep(
                localCenter: Vec3(), c1: Vec3(4, 0, 0), c2: Vec3(0, 0, 0),
                q1: Quat.identity, q2: q
            ))

            #expect(output.state == .hit)
            samples.append(UInt64(output.fraction.bitPattern))
            samples.append(UInt64(output.point.x.bitPattern))
            samples.append(UInt64(output.point.y.bitPattern))
            #expect(output.normal == Vec3(1, 0, 0))
            samples.append(UInt64(output.distance.bitPattern))
            #expect(output.distanceIterations == 2)
            #expect(output.rootIterations == 4)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
