import Testing
@testable import Natsel

@Suite("Shape ray edge cases")
struct ShapeRayEdgeTests {
    /// Capsule along x with radius 1.
    private let rayCapsule = Capsule(
        center1: Vec3(-2, 0, 0), center2: Vec3(2, 0, 0), radius: 1)

    private func checkHit(
        _ out: CastOutput, _ origin: Vec3, _ translation: Vec3,
        _ point: Vec3, _ normal: Vec3, _ fraction: Float, _ tol: Float
    ) -> String? {
        if !out.hit { return "no hit" }
        if absFloat(out.fraction - fraction) > tol { return "fraction \(out.fraction)" }
        if absFloat(out.point.x - point.x) > tol || absFloat(out.point.y - point.y) > tol
            || absFloat(out.point.z - point.z) > tol
        {
            return "point \(out.point)"
        }
        if absFloat(out.normal.x - normal.x) > tol || absFloat(out.normal.y - normal.y) > tol
            || absFloat(out.normal.z - normal.z) > tol
        {
            return "normal \(out.normal)"
        }

        // The reported point must also sit on the ray.
        let onRay = mulAdd(origin, out.fraction, translation)
        if distance(out.point, onRay) > tol { return "point not on ray" }
        return nil
    }

    @Test("coincident capsule centers collapse to a sphere")
    func degenerate() {
        // Coincident centers collapse to a sphere.
        let sphereLike = Capsule(center1: Vec3(), center2: Vec3(), radius: 1)
        var input = RayCastInput()
        input.origin = Vec3(-4, 0, 0)
        input.translation = Vec3(8, 0, 0)
        input.maxFraction = 1

        let out = rayCastCapsule(sphereLike, input)
        if let error = checkHit(
            out, input.origin, input.translation,
            Vec3(-1, 0, 0), Vec3(-1, 0, 0), 3.0 / 8.0, 1e-5
        ) {
            Issue.record("\(error)")
        }
    }

    @Test("max fraction straddles the side hit")
    func clip() {
        // The side hit occurs at fraction 1/3.
        var miss = RayCastInput()
        miss.origin = Vec3(0, 3, 0)
        miss.translation = Vec3(0, -6, 0)
        miss.maxFraction = 0.3
        #expect(rayCastCapsule(rayCapsule, miss).hit == false)

        var hit = RayCastInput()
        hit.origin = Vec3(0, 3, 0)
        hit.translation = Vec3(0, -6, 0)
        hit.maxFraction = 0.5
        let out = rayCastCapsule(rayCapsule, hit)
        #expect(out.hit)
        #expect(absFloat(out.fraction - 1.0 / 3.0) < 1e-5)
    }

    /// A ray within a hair of the capsule axis must still hit when it slowly converges onto the
    /// surface. The closest point solver is ill conditioned in this band, so this guards the near
    /// parallel fallback that intersects the infinite cylinder directly.
    @Test("near-parallel rays converging on the axis still hit")
    func parallel() {
        // Capsule along y. A long ray almost parallel to the axis drifts inward from just outside
        // the cylinder and dips through the far endcap.
        let axisY = Capsule(center1: Vec3(0, 0, 0), center2: Vec3(0, 10, 0), radius: 1)

        var first = RayCastInput()
        first.origin = Vec3(1.0001, 100, 0)
        first.translation = Vec3(-0.001, -200, 0)
        first.maxFraction = 1

        let outA = rayCastCapsule(axisY, first)
        #expect(outA.hit)
        if outA.hit {
            // The hit lands on the capsule surface and on the ray.
            let onSeg = pointToSegmentDistance(axisY.center1, axisY.center2, outA.point)
            #expect(absFloat(distance(outA.point, onSeg) - axisY.radius) < 1e-3)
            let onRay = mulAdd(first.origin, outA.fraction, first.translation)
            #expect(distance(outA.point, onRay) < 1e-3)
        }

        // Near parallel ray converging onto the x-axis capsule from far away.
        var second = RayCastInput()
        second.origin = Vec3(-1000, 1.0001, 0)
        second.translation = Vec3(2000, -0.001, 0)
        second.maxFraction = 1

        let outB = rayCastCapsule(rayCapsule, second)
        #expect(outB.hit)
        if outB.hit {
            let onSeg = pointToSegmentDistance(
                rayCapsule.center1, rayCapsule.center2, outB.point)
            #expect(absFloat(distance(outB.point, onSeg) - rayCapsule.radius) < 1e-3)
        }

        // Exactly parallel and outside the cylinder still misses.
        var parallel = RayCastInput()
        parallel.origin = Vec3(0, 3, 0)
        parallel.translation = Vec3(8, 0, 0)
        parallel.maxFraction = 1
        #expect(rayCastCapsule(rayCapsule, parallel).hit == false)
    }

    @Test("a ray starting inside the sphere hits at fraction zero")
    func sphereInterior() {
        let sphere = Sphere(center: Vec3(), radius: 1)

        var input = RayCastInput()
        input.origin = Vec3()
        input.translation = Vec3(1, 0, 0)
        input.maxFraction = 1

        let out = rayCastSphere(sphere, input)
        // Origin inside reports the origin with zero fraction.
        #expect(out.hit)
        #expect(out.fraction == 0)
        #expect(distance(out.point, input.origin) < Float.ulpOfOne)

        // A zero length ray while inside still reports the origin.
        var still = RayCastInput()
        still.origin = Vec3(0.5, 0, 0)
        still.translation = Vec3()
        still.maxFraction = 1
        let stillOut = rayCastSphere(sphere, still)
        #expect(stillOut.hit)
        #expect(distance(stillOut.point, still.origin) < Float.ulpOfOne)

        // A zero length ray outside misses.
        var outside = RayCastInput()
        outside.origin = Vec3(3, 0, 0)
        outside.translation = Vec3()
        outside.maxFraction = 1
        #expect(rayCastSphere(sphere, outside).hit == false)
    }

    @Test("a grazing ray reports a tangent hit")
    func sphereGraze() {
        let sphere = Sphere(center: Vec3(), radius: 1)

        // Travel along y = 1 exactly, so the ray is tangent to the sphere.
        var input = RayCastInput()
        input.origin = Vec3(-5, 1, 0)
        input.translation = Vec3(10, 0, 0)
        input.maxFraction = 1

        let out = rayCastSphere(sphere, input)
        #expect(out.hit)
        if out.hit {
            // The hit is at the top of the sphere.
            #expect(absFloat(out.point.x) < 1e-3)
            #expect(absFloat(out.point.y - 1) < 1e-3)
            #expect(absFloat(out.fraction - 0.5) < 1e-3)
        }
    }
}
