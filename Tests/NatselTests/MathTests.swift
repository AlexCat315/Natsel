import Testing
@testable import Natsel

@Suite("Math")
struct MathTests {
    @Test("atan2 approximation stays within tolerance")
    func atan2Accuracy() {
        #expect(atan2Approx(0, 0) == 0)
        #expect(abs(atan2Approx(1, 1) - 0.785398163) < 1e-4)
        #expect(abs(atan2Approx(-1, -1) + 2.35619449) < 1e-4)
    }

    @Test("cosine and sine pair is normalized")
    func cosSin() {
        let cs = computeCosSin(0.5)
        #expect(abs(cs.cosine * cs.cosine + cs.sine * cs.sine - 1) < 1e-6)
    }

    @Test("pi uses the C literal rounding")
    func piRounding() {
        #expect(B3.pi.bitPattern == 0x4049_0FDB)
    }
}
