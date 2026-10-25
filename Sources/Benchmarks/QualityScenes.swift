import Natsel

nonisolated(unsafe) private var querySoftBody = SoftBodyID.null
nonisolated(unsafe) private var softRayChecksum: Double = 0

/// A deformable surface with 7,938 faces queried 512 times per step. The BVH is rebuilt lazily
/// after each simulated frame. NATSEL_SOFT_QUERY_BVH=0 provides the linear-scan baseline.
func createSoftQueries(_ world: World) {
    world.gravity = .zero
    world.enableSoftBodyQueryAcceleration = benchmarkEnvironment["NATSEL_SOFT_QUERY_BVH"] != "0"
    var settings = SoftBodySettings.makeCloth(columns: 64, rows: 64, spacing: 0.1)
    for i in settings.vertices.indices { settings.vertices[i].invMass = 0 }
    var def = SoftBodyDef(settings: settings)
    def.enableSleep = false
    def.facesDoubleSided = true
    querySoftBody = world.createSoftBody(def)
    softRayChecksum = 0
}

func stepSoftQueries(_ world: World, _ step: Int) {
    for i in 0..<512 {
        let x = Double((i * 17 + step) % 101) * 0.06 - 3
        let z = Double((i * 31) % 101) * 0.06 - 3
        let hit = world.castRaySoftBody(querySoftBody, Pos(x, 2, z), Vec3(0, -4, 0))
        if hit.hit { softRayChecksum += Double(hit.fraction) + Double(hit.faceIndex) }
    }
}

func destroySoftQueries() {
    print("soft-ray checksum: \(softRayChecksum)")
    querySoftBody = .null
}

/// 2,000 separated soft bodies at a distant origin. Exercises precise grouping without creating
/// quadratic pairs from coarse absolute float bounds.
func createSparseSoftBodies(_ world: World) {
    world.gravity = .zero
    let settings = SoftBodySettings.makeCloth(columns: 2, rows: 2, spacing: 0.1)
    for i in 0..<2000 {
        var def = SoftBodyDef(settings: settings, position: Pos(1.0e12, Double(i) * 2, -1.0e12))
        def.enableSleep = false
        def.iterationCount = 1
        _ = world.createSoftBody(def)
    }
}
