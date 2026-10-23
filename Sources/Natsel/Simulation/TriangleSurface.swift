

/// Runtime boundary for the triangle operations shared by mesh and terrain contacts.
enum TriangleSurface {
    case mesh(Mesh)
    case heightField(HeightField)

    init?(_ geometry: ShapeGeometry) {
        switch geometry {
        case .mesh(let mesh): self = .mesh(mesh)
        case .heightField(let field): self = .heightField(field)
        default: return nil
        }
    }

    func query(_ bounds: AABB, _ callback: (Vec3, Vec3, Vec3, Int32) -> Bool) {
        switch self {
        case .mesh(let mesh): queryMesh(mesh, bounds: bounds, callback)
        case .heightField(let field): queryHeightField(field, bounds: bounds, callback)
        }
    }

    func triangle(_ index: Int32) -> Triangle {
        switch self {
        case .mesh(let mesh): mesh.triangle(index)
        case .heightField(let field): field.triangle(index)
        }
    }

    func material(_ index: Int32) -> Int32 {
        switch self {
        case .mesh(let mesh): Int32(mesh.data!.materialIndices[Int(index)])
        case .heightField(let field): field.material(triangle: index)
        }
    }
}
