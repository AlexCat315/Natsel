@testable import Natsel

func referenceCompound(_ scenario: Int) -> Compound? {
    let capsuleCounts = [0,2,0,0,2,6], hullCounts = [0,0,2,0,2,6]
    let meshCounts = [0,0,0,2,2,4], sphereCounts = [1,0,0,0,3,8]
    var a = SurfaceMaterial.defaultMaterial
    a.friction = 0.4; a.restitution = 0.1; a.rollingResistance = 0.05
    a.tangentVelocity = Vec3(0.125, 0, 0.25); a.userMaterialId = 5
    var b = a; b.friction = 0.7; b.userMaterialId = 6
    let materials = [a,b,a]
    let hulls = [makeBoxHull(0.5, 0.25, 0.75), makeBoxHull(0.4, 0.35, 0.6)]
    guard let mesh = createGridMesh(xCount: 2, zCount: 3, cellWidth: 0.5, materialCount: 3, identifyEdges: true) else { return nil }
    var definition = CompoundDef()
    for i in 0..<capsuleCounts[scenario] {
        var item = CompoundCapsuleDef()
        item.capsule = Capsule(center1: Vec3(Float(i) - 1, -0.5, 0.25), center2: Vec3(Float(i) - 1, 0.5, 0.25), radius: 0.2)
        item.material = materials[i % 2]
        definition.capsules.append(item)
    }
    for i in 0..<hullCounts[scenario] {
        var item = CompoundHullDef()
        item.hull = hulls[i % 3 == 2 ? 1 : 0]
        item.transform = Transform(Vec3(Float(i) * 0.75 - 1, 0.25, -0.5), makeQuatFromAxisAngle(.axisY, Float(i) * 0.1))
        item.material = materials[i % 2]
        definition.hulls.append(item)
    }
    for i in 0..<meshCounts[scenario] {
        var item = CompoundMeshDef()
        item.meshData = mesh; item.materials = materials
        item.transform = Transform(Vec3(Float(i) * 1.5, -0.5, 0.5), makeQuatFromAxisAngle(.axisY, Float(i) * 0.1))
        item.scale = Vec3(i % 2 == 0 ? 1 : -1, 1, 1.25)
        definition.meshes.append(item)
    }
    for i in 0..<sphereCounts[scenario] {
        var item = CompoundSphereDef()
        item.sphere = Sphere(center: Vec3(Float(i) * 0.5 - 1, 0.25, 1), radius: 0.3)
        item.material = materials[i % 2]
        definition.spheres.append(item)
    }
    return Compound(definition)
}
