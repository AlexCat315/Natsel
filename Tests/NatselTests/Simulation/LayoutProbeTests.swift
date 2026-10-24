import Testing
@testable import Natsel

@Suite("Layout probe")
struct LayoutProbeTests {
    @Test("SoA gathers rely on these offsets")
    func offsets() {
        #expect(MemoryLayout<ManifoldPoint>.offset(of: \.anchorA) == 0)
        #expect(MemoryLayout<ManifoldPoint>.offset(of: \.anchorB) == 12)
        #expect(MemoryLayout<ManifoldPoint>.offset(of: \.separation) == 24)
        #expect(MemoryLayout<ManifoldPoint>.offset(of: \.normalImpulse) == 28)
        let normal = MemoryLayout<Manifold>.offset(of: \.normal)!
        #expect(MemoryLayout<Manifold>.offset(of: \.twistImpulse) == normal + 12)
        #expect(MemoryLayout<Manifold>.offset(of: \.frictionImpulse) == normal + 16)
        #expect(MemoryLayout<Manifold>.offset(of: \.rollingImpulse) == normal + 28)
        #expect(MemoryLayout<Matrix3>.stride == 36)
        print("LAYOUT manifold.normal=\(normal) points=\(MemoryLayout<Manifold>.offset(of: \.points)!) mp.stride=\(MemoryLayout<ManifoldPoint>.stride) invInertia=\(MemoryLayout<BodySim>.offset(of: \.invInertiaWorld)!) invMass=\(MemoryLayout<BodySim>.offset(of: \.invMass)!) bodySim=\(MemoryLayout<BodySim>.stride)")
    }
}
