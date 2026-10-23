extension Mesh {
    func triangle(_ index: Int32) -> Triangle {
        let data = data!
        let source = data.triangles[Int(index)]
        let flags = Int32(data.edgeFlags[Int(index)])
        let vertices = data.vertices
        var triangle = Triangle()
        triangle.vertices[0] = scale * vertices[Int(source.index1)]
        triangle.i1 = source.index1
        if scale.x * scale.y * scale.z < 0 {
            triangle.i2 = source.index3
            triangle.i3 = source.index2
            triangle.flags = (flags >> 4) & 7
        } else {
            triangle.i2 = source.index2
            triangle.i3 = source.index3
            triangle.flags = flags
        }
        triangle.vertices[1] = scale * vertices[Int(triangle.i2)]
        triangle.vertices[2] = scale * vertices[Int(triangle.i3)]
        return triangle
    }
}
