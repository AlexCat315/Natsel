import Testing
@testable import Natsel

// Encroaching shape casts ported from test/test_distance.c. A unit box has a segment resting 2mm
// from its +x face: inside the cast target distance, but not overlapped.
@Suite("Shape cast encroach")
struct ShapeCastEncroachTests {
    private let boxA = [
        Vec3(-1, -1, 0), Vec3(1, -1, 0), Vec3(1, 1, 0), Vec3(-1, 1, 0),
    ]
    private let segmentB = [Vec3(1.002, -1, 0), Vec3(1.002, 1, 0)]

    /// A 5mm-gap variant used by the graded and max-fraction cases.
    private let gradedB = [Vec3(1.005, -1, 0), Vec3(1.005, 1, 0)]

    private func cast(
        _ bPoints: [Vec3], translation: Vec3, maxFraction: Float = 1
    ) -> CastOutput {
        var input = ShapeCastPairInput()
        input.transform = .identity
        input.translationB = translation
        input.maxFraction = maxFraction
        input.canEncroach = true

        var output = CastOutput()
        boxA.withUnsafeBufferPointer { a in
            bPoints.withUnsafeBufferPointer { b in
                input.proxyA = ShapeProxy(points: a.baseAddress, count: Int32(a.count), radius: 0)
                input.proxyB = ShapeProxy(points: b.baseAddress, count: Int32(b.count), radius: 0)
                output = shapeCast(input)
            }
        }
        return output
    }

    @Test("receding from the gap misses")
    func recede() {
        #expect(cast(segmentB, translation: Vec3(1, 0, 0)).hit == false)
    }

    @Test("sliding along the gap misses")
    func slide() {
        #expect(cast(segmentB, translation: Vec3(0, 2, 0)).hit == false)
    }

    @Test("closing the gap hits immediately")
    func closing() {
        let out = cast(segmentB, translation: Vec3(-1, 0, 0))
        #expect(out.hit)
        #expect(absFloat(out.fraction) < Float.ulpOfOne)
        #expect(absFloat(out.normal.x - 1) < 0.001)
    }

    /// A 5mm gap: closing consumes the gap down to the retained rest separation.
    @Test("a 5mm gap closes at the graded fraction")
    func graded() {
        let out = cast(gradedB, translation: Vec3(-1, 0, 0))
        #expect(out.hit)
        #expect(absFloat(out.fraction - 0.0025) < 0.0005)
        #expect(absFloat(out.normal.x - 1) < 0.001)
    }

    /// A 5mm gap closes at fraction 0.0025, so the max fraction gates the hit.
    @Test("max fraction gates the closed-gap hit")
    func maxFraction() {
        #expect(cast(gradedB, translation: Vec3(-1, 0, 0), maxFraction: 0.002).hit == false)

        let out = cast(gradedB, translation: Vec3(-1, 0, 0), maxFraction: 0.003)
        #expect(out.hit)
        #expect(absFloat(out.fraction - 0.0025) < 0.0005)
    }

    @Test("a mostly sideways sweep with no inward component misses")
    func graze() {
        // Moving purely sideways keeps the 2mm rest gap, so nothing closes.
        #expect(cast(segmentB, translation: Vec3(0, 4, 0)).hit == false)
        // Moving purely outward opens the gap further.
        #expect(cast(segmentB, translation: Vec3(2, 4, 0)).hit == false)
    }

    @Test("starting overlapped reports a zero fraction when encroaching is allowed")
    func overlap() {
        // Start the segment inside the box.
        let inside = [Vec3(0.5, -1, 0), Vec3(0.5, 1, 0)]
        let out = cast(inside, translation: Vec3(-1, 0, 0))
        #expect(out.hit)
        #expect(absFloat(out.fraction) < Float.ulpOfOne)
    }

    @Test("without encroaching the same gap is not closed")
    func noEncroach() {
        var input = ShapeCastPairInput()
        input.transform = .identity
        input.translationB = Vec3(-1, 0, 0)
        input.maxFraction = 1
        input.canEncroach = false

        var output = CastOutput()
        boxA.withUnsafeBufferPointer { a in
            segmentB.withUnsafeBufferPointer { b in
                input.proxyA = ShapeProxy(points: a.baseAddress, count: Int32(a.count), radius: 0)
                input.proxyB = ShapeProxy(points: b.baseAddress, count: Int32(b.count), radius: 0)
                output = shapeCast(input)
            }
        }

        // Closing the 2mm gap still registers; canEncroach only governs how close a cast may
        // travel *after* contact, not whether a real gap may be closed.
        #expect(output.hit)
    }
}
