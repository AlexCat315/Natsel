import Testing
@testable import Natsel

@Suite("Solver storage")
struct SolverStorageTests {
    @Test("joint variants share compact POD storage")
    func jointLayout() {
        #expect(MemoryLayout<JointSim>.stride < 768)
        #expect(MemoryLayout<Contact>.stride < 256)
        print("LAYOUT JointSim=\(MemoryLayout<JointSim>.stride) Contact=\(MemoryLayout<Contact>.stride) Payload=\(MemoryLayout<ContactPayload>.stride)")
    }

    @Test("contact slabs preserve addresses and reset reused slots")
    func stableSlabs() {
        let pool = ContactPayloadPool()
        let first = pool.initialize(0)
        first.pointee.manifolds = ContactManifolds(Manifold())
        first.pointee.manifolds[0].points[0].normalImpulse = 7
        var contact = Contact()
        contact.payload = first
        let view = contact.manifoldBuffer
        for id in 1..<1025 { _ = pool.initialize(id) }
        #expect(view[0].points[0].normalImpulse == 7)
        first.pointee.manifolds[0].points[0].normalImpulse = 9
        #expect(view[0].points[0].normalImpulse == 9)
        #expect(pool.initialize(0) == first)
        #expect(first.pointee.manifolds.isEmpty)
    }

    @Test("inline and multiple manifold snapshots preserve value semantics")
    func manifoldSnapshots() {
        var manifolds = ContactManifolds(Manifold())
        var single = manifolds
        single[0].twistImpulse = 1
        #expect(manifolds[0].twistImpulse == 0)
        manifolds.append(Manifold())
        var multiple = manifolds
        multiple[1].twistImpulse = 2
        #expect(manifolds[1].twistImpulse == 0)
        manifolds.removeAll(keepingCapacity: true)
        manifolds.append(single[0])
        #expect(manifolds.count == 1)
        manifolds.withUnsafeMutableBufferPointer { $0[0].twistImpulse = 3 }
        #expect(manifolds[0].twistImpulse == 3)
        #expect(multiple[1].twistImpulse == 2)
    }
    @Test("default material mixers inline while custom callbacks preserve material ids")
    func materialMixing() {
        let world = World()
        #expect(world.mixFriction(0.25, 7, 1, 9) == 0.5)
        #expect(world.mixRestitution(0.25, 7, 1, 9) == 1)
        world.frictionCallback = { a, idA, b, idB in
            #expect(idA == 7 && idB == 9)
            return a + b
        }
        world.restitutionCallback = { a, idA, b, idB in
            #expect(idA == 7 && idB == 9)
            return a - b
        }
        #expect(world.mixFriction(0.25, 7, 1, 9) == 1.25)
        #expect(world.mixRestitution(0.25, 7, 1, 9) == -0.75)
        world.frictionCallback = nil
        world.restitutionCallback = nil
        #expect(world.mixFriction(0.25, 7, 1, 9) == 0.5)
    }

}
