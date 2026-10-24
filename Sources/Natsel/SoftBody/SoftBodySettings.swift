// Shared soft body settings: the rest state of the particles and the constraints between them.
//
// A soft body is a set of particles connected by XPBD constraints: edges (stretch and shear),
// dihedral or distance bending, tetrahedral volume, long range attachments, skinning, and an
// optional internal pressure for closed surfaces. Settings are a value type whose arrays are shared
// by every instance created from them.

/// A particle in the rest configuration.
public struct SoftBodyVertex: Sendable, Equatable {
    public var position = Vec3()
    public var velocity = Vec3()
    /// Zero makes the vertex kinematic: it is moved only by the user or by skinning.
    public var invMass: Float = 1

    public init() {}

    public init(position: Vec3, velocity: Vec3 = Vec3(), invMass: Float = 1) {
        self.position = position
        self.velocity = velocity
        self.invMass = invMass
    }
}

/// A triangle of the soft body surface, counter clockwise when seen from outside.
public struct SoftBodyFace: Sendable, Equatable {
    public var vertex = SIMD3<UInt32>(0, 0, 0)

    public init(_ v0: UInt32, _ v1: UInt32, _ v2: UInt32) {
        vertex = SIMD3(v0, v1, v2)
    }

    public var isDegenerate: Bool {
        vertex.x == vertex.y || vertex.y == vertex.z || vertex.x == vertex.z
    }
}

/// Keeps two vertices at a rest length.
public struct SoftBodyEdge: Sendable, Equatable {
    public var vertex = SIMD2<UInt32>(0, 0)
    public var restLength: Float = 1
    /// Inverse stiffness (m/N). Zero is rigid.
    public var compliance: Float = 0

    public init(_ v0: UInt32, _ v1: UInt32, compliance: Float = 0) {
        vertex = SIMD2(v0, v1)
        self.compliance = compliance
    }
}

/// Keeps the angle between two triangles sharing the edge (v0, v1). The triangles are
/// (v0, v1, v2) and (v1, v0, v3).
public struct SoftBodyDihedralBend: Sendable, Equatable {
    public var vertex = SIMD4<UInt32>(0, 0, 0, 0)
    public var compliance: Float = 0
    /// Rest angle in radians.
    public var initialAngle: Float = 0

    public init(_ v0: UInt32, _ v1: UInt32, _ v2: UInt32, _ v3: UInt32, compliance: Float = 0) {
        vertex = SIMD4(v0, v1, v2, v3)
        self.compliance = compliance
    }
}

/// Keeps the volume of a tetrahedron.
public struct SoftBodyVolume: Sendable, Equatable {
    public var vertex = SIMD4<UInt32>(0, 0, 0, 0)
    /// Six times the signed rest volume.
    public var sixRestVolume: Float = 1
    public var compliance: Float = 0

    public init(_ v0: UInt32, _ v1: UInt32, _ v2: UInt32, _ v3: UInt32, compliance: Float = 0) {
        vertex = SIMD4(v0, v1, v2, v3)
        self.compliance = compliance
    }
}

/// Long range attachment: keeps a vertex within `maxDistance` of a kinematic anchor vertex. These
/// make long hanging cloth inextensible without extra iterations.
public struct SoftBodyLRA: Sendable, Equatable {
    public var anchor: UInt32 = 0
    public var vertex: UInt32 = 0
    public var maxDistance: Float = 0

    public init(anchor: UInt32, vertex: UInt32, maxDistance: Float = 0) {
        self.anchor = anchor
        self.vertex = vertex
        self.maxDistance = maxDistance
    }
}

/// Binds a vertex to an animated skeleton. Up to four joint weights.
public struct SoftBodySkinned: Sendable, Equatable {
    public var vertex: UInt32 = 0
    public var joints = SIMD4<UInt32>(0, 0, 0, 0)
    public var weights = SIMD4<Float>(1, 0, 0, 0)
    /// How far the vertex may move from its skinned position. Zero pins it to the skin.
    public var maxDistance: Float = 0
    /// A sphere behind the skinned surface the vertex may not enter, at
    /// `backStopDistance + backStopRadius` below the skinned position along the skinned normal.
    public var backStopDistance: Float = 0
    public var backStopRadius: Float = 0
    /// Faces around the vertex, used to compute the skinned normal.
    var normalFaces: [UInt32] = []

    public init(vertex: UInt32, joints: SIMD4<UInt32>, weights: SIMD4<Float>, maxDistance: Float = 0,
                backStopDistance: Float = 0, backStopRadius: Float = 0) {
        self.vertex = vertex
        self.joints = joints
        self.weights = weights
        self.maxDistance = maxDistance
        self.backStopDistance = backStopDistance
        self.backStopRadius = backStopRadius
    }
}

public enum SoftBodyLRAType: Sendable {
    case none
    /// Straight line distance to the nearest kinematic vertex.
    case euclidean
    /// Distance along the edges to the nearest kinematic vertex. Better for folded cloth.
    case geodesic
}

public enum SoftBodyBendType: Sendable {
    case none
    /// A distance constraint between the far vertices of adjacent triangles.
    case distance
    /// An angle constraint between adjacent triangles. Allows creases to keep their shape.
    case dihedral
}

/// Per vertex inputs for automatic constraint creation. The compliance of a constraint is the
/// largest compliance of its vertices.
public struct SoftBodyVertexAttributes: Sendable {
    /// Compliance of the triangle edges.
    public var compliance: Float = 0
    /// Compliance of quad diagonals. Infinity creates no shear constraints.
    public var shearCompliance: Float = 0
    /// Compliance of bending. Infinity creates no bend constraints.
    public var bendCompliance: Float = .infinity
    public var lraType = SoftBodyLRAType.none
    /// Scales the rest distance of the long range attachment, allowing a little stretch.
    public var lraMaxDistanceMultiplier: Float = 1

    public init() {}

    public init(compliance: Float, shearCompliance: Float = 0, bendCompliance: Float = .infinity,
                lraType: SoftBodyLRAType = .none, lraMaxDistanceMultiplier: Float = 1) {
        self.compliance = compliance
        self.shearCompliance = shearCompliance
        self.bendCompliance = bendCompliance
        self.lraType = lraType
        self.lraMaxDistanceMultiplier = lraMaxDistanceMultiplier
    }
}

public struct SoftBodySettings: Sendable {
    public var vertices: [SoftBodyVertex] = []
    public var faces: [SoftBodyFace] = []
    public var edges: [SoftBodyEdge] = []
    public var dihedralBends: [SoftBodyDihedralBend] = []
    public var volumes: [SoftBodyVolume] = []
    public var longRangeAttachments: [SoftBodyLRA] = []
    public var skinned: [SoftBodySkinned] = []
    /// Inverse bind pose of each skeleton joint, used with `skinned`.
    public var inverseBindMatrices: [Transform] = []

    /// Vertex index ranges grouped by spatial locality, used to batch collision detection.
    var patches: [Range<Int>] = []
    /// Vertex permutation applied by `optimize`, old index -> new index.
    public internal(set) var vertexRemap: [UInt32] = []

    public init() {}

    // MARK: Constraint generation

    /// Create edge, shear, bend, and long range attachment constraints from the faces.
    /// `attributes` has one entry per vertex, or a single entry used for all vertices.
    public mutating func createConstraints(
        _ attributes: [SoftBodyVertexAttributes], bendType: SoftBodyBendType = .distance,
        angleTolerance: Float = 0.13962634
    ) {
        guard attributes.isEmpty == false else { return }
        func attribute(_ v: UInt32) -> SoftBodyVertexAttributes {
            attributes.count == 1 ? attributes[0] : attributes[Int(v)]
        }

        struct EdgeKey: Hashable {
            var a: UInt32
            var b: UInt32
            init(_ x: UInt32, _ y: UInt32) {
                a = min(x, y)
                b = max(x, y)
            }
        }

        // Faces around each edge.
        var edgeFaces: [EdgeKey: [Int]] = [:]
        var edgeOrder: [EdgeKey] = []
        for (f, face) in faces.enumerated() where face.isDegenerate == false {
            let v = [face.vertex.x, face.vertex.y, face.vertex.z]
            for e in 0..<3 {
                let key = EdgeKey(v[e], v[(e + 1) % 3])
                if edgeFaces[key] == nil { edgeOrder.append(key) }
                edgeFaces[key, default: []].append(f)
            }
        }

        func opposite(_ face: SoftBodyFace, _ key: EdgeKey) -> UInt32 {
            for v in [face.vertex.x, face.vertex.y, face.vertex.z] where v != key.a && v != key.b {
                return v
            }
            return face.vertex.x
        }

        func position(_ v: UInt32) -> Vec3 { vertices[Int(v)].position }

        func faceNormal(_ face: SoftBodyFace) -> Vec3 {
            let a = position(face.vertex.x)
            let b = position(face.vertex.y)
            let c = position(face.vertex.z)
            return normalize(cross(b - a, c - a))
        }

        var existing = Set<EdgeKey>()
        for edge in edges { existing.insert(EdgeKey(edge.vertex.x, edge.vertex.y)) }

        func addEdge(_ a: UInt32, _ b: UInt32, _ compliance: Float) {
            guard compliance.isFinite, a != b else { return }
            let key = EdgeKey(a, b)
            guard existing.insert(key).inserted else { return }
            edges.append(SoftBodyEdge(a, b, compliance: compliance))
        }

        // A shared edge is a quad diagonal when it is the longest edge of both triangles and the
        // triangles are nearly coplanar.
        let cosTolerance = computeCosSin(angleTolerance).cosine
        var diagonals = Set<EdgeKey>()
        for key in edgeOrder {
            guard let list = edgeFaces[key], list.count == 2 else { continue }
            let f0 = faces[list[0]]
            let f1 = faces[list[1]]
            guard dot(faceNormal(f0), faceNormal(f1)) >= cosTolerance else { continue }
            let length = distanceSquared(position(key.a), position(key.b))
            var longest = true
            for (face, _) in [(f0, 0), (f1, 1)] {
                let o = opposite(face, key)
                if distanceSquared(position(o), position(key.a)) > length
                    || distanceSquared(position(o), position(key.b)) > length
                {
                    longest = false
                }
            }
            if longest { diagonals.insert(key) }
        }

        // Stretch edges, with quad diagonals treated as shear.
        for key in edgeOrder {
            let shear = diagonals.contains(key)
            let a = attribute(key.a)
            let b = attribute(key.b)
            let compliance = shear
                ? max(a.shearCompliance, b.shearCompliance) : max(a.compliance, b.compliance)
            addEdge(key.a, key.b, compliance)
        }

        // Shear across each quad, and bending across every other shared edge.
        for key in edgeOrder {
            guard let list = edgeFaces[key], list.count == 2 else { continue }
            let o0 = opposite(faces[list[0]], key)
            let o1 = opposite(faces[list[1]], key)
            guard o0 != o1 else { continue }
            if diagonals.contains(key) {
                let compliance = max(attribute(o0).shearCompliance, attribute(o1).shearCompliance)
                addEdge(o0, o1, compliance)
                continue
            }

            let compliance = max(
                max(attribute(key.a).bendCompliance, attribute(key.b).bendCompliance),
                max(attribute(o0).bendCompliance, attribute(o1).bendCompliance))
            guard compliance.isFinite else { continue }
            switch bendType {
            case .none:
                break
            case .distance:
                addEdge(o0, o1, compliance)
            case .dihedral:
                // Orient the shared edge as it appears in the first face: (v0, v1, v2).
                let f0 = faces[list[0]]
                let v = [f0.vertex.x, f0.vertex.y, f0.vertex.z]
                var e0 = key.a
                var e1 = key.b
                for i in 0..<3 where v[i] == key.b && v[(i + 1) % 3] == key.a {
                    e0 = key.b
                    e1 = key.a
                }
                dihedralBends.append(SoftBodyDihedralBend(e0, e1, o0, o1, compliance: compliance))
            }
        }

        // Long range attachments to the nearest kinematic vertex.
        let euclidean = (0..<vertices.count).filter { attribute(UInt32($0)).lraType == .euclidean }
        let geodesic = (0..<vertices.count).filter { attribute(UInt32($0)).lraType == .geodesic }
        let anchors = (0..<vertices.count).filter { vertices[$0].invMass == 0 }
        if anchors.isEmpty == false {
            for v in euclidean where vertices[v].invMass > 0 {
                var best = -1
                var bestDistance: Float = .greatestFiniteMagnitude
                for a in anchors {
                    let d = distanceSquared(vertices[v].position, vertices[a].position)
                    if d < bestDistance {
                        bestDistance = d
                        best = a
                    }
                }
                if best >= 0 {
                    let multiplier = attribute(UInt32(v)).lraMaxDistanceMultiplier
                    longRangeAttachments.append(SoftBodyLRA(
                        anchor: UInt32(best), vertex: UInt32(v), maxDistance: bestDistance.squareRoot() * multiplier))
                }
            }
            if geodesic.isEmpty == false {
                let (distances, nearest) = geodesicDistances(from: anchors)
                for v in geodesic where vertices[v].invMass > 0 && nearest[v] >= 0 {
                    let multiplier = attribute(UInt32(v)).lraMaxDistanceMultiplier
                    longRangeAttachments.append(SoftBodyLRA(
                        anchor: UInt32(nearest[v]), vertex: UInt32(v), maxDistance: distances[v] * multiplier))
                }
            }
        }

        calculateRestValues()
    }

    /// Multi source Dijkstra over the edges.
    private func geodesicDistances(from sources: [Int]) -> (distances: [Float], nearest: [Int]) {
        var adjacency = [[(Int, Float)]](repeating: [], count: vertices.count)
        for edge in edges {
            let a = Int(edge.vertex.x)
            let b = Int(edge.vertex.y)
            let d = distance(vertices[a].position, vertices[b].position)
            adjacency[a].append((b, d))
            adjacency[b].append((a, d))
        }

        var distances = [Float](repeating: .greatestFiniteMagnitude, count: vertices.count)
        var nearest = [Int](repeating: -1, count: vertices.count)
        var heap = BinaryHeap()
        for s in sources {
            distances[s] = 0
            nearest[s] = s
            heap.push(0, s)
        }
        while let (d, v) = heap.pop() {
            if d > distances[v] { continue }
            for (n, w) in adjacency[v] where d + w < distances[n] {
                distances[n] = d + w
                nearest[n] = nearest[v]
                heap.push(d + w, n)
            }
        }
        return (distances, nearest)
    }

    // MARK: Rest values

    /// Compute rest lengths, angles, volumes, and attachment distances from the vertex positions.
    /// Attachments with a positive distance keep it.
    public mutating func calculateRestValues() {
        func p(_ v: UInt32) -> Vec3 { vertices[Int(v)].position }
        for i in edges.indices {
            edges[i].restLength = distance(p(edges[i].vertex.x), p(edges[i].vertex.y))
        }
        for i in dihedralBends.indices {
            let b = dihedralBends[i]
            dihedralBends[i].initialAngle = dihedralAngle(p(b.vertex.x), p(b.vertex.y), p(b.vertex.z), p(b.vertex.w))
        }
        for i in volumes.indices {
            let t = volumes[i]
            let x0 = p(t.vertex.x)
            volumes[i].sixRestVolume = dot(p(t.vertex.y) - x0, cross(p(t.vertex.z) - x0, p(t.vertex.w) - x0))
        }
        for i in longRangeAttachments.indices where longRangeAttachments[i].maxDistance <= 0 {
            let a = longRangeAttachments[i]
            longRangeAttachments[i].maxDistance = distance(p(a.anchor), p(a.vertex))
        }
        calculateSkinnedNormalFaces()
    }

    mutating func calculateSkinnedNormalFaces() {
        guard skinned.isEmpty == false else { return }
        var facesOfVertex = [[UInt32]](repeating: [], count: vertices.count)
        for (f, face) in faces.enumerated() {
            for v in [face.vertex.x, face.vertex.y, face.vertex.z] {
                facesOfVertex[Int(v)].append(UInt32(f))
            }
        }
        for i in skinned.indices {
            skinned[i].normalFaces = facesOfVertex[Int(skinned[i].vertex)]
        }
    }

    // MARK: Optimization

    /// Reorder for the solver. Vertices are sorted along a space filling curve so collision can be
    /// batched in compact patches; edges are sorted outward from the kinematic vertices so stiffness
    /// propagates from the anchors in a single sweep. `vertexRemap` records the permutation.
    public mutating func optimize() {
        guard vertices.isEmpty == false else { return }
        let n = vertices.count

        // Morton order of the rest positions.
        var lower = vertices[0].position
        var upper = lower
        for v in vertices {
            lower = minVec3(lower, v.position)
            upper = maxVec3(upper, v.position)
        }
        let extent = maxVec3(upper - lower, Vec3(1.0e-6, 1.0e-6, 1.0e-6))
        func morton(_ p: Vec3) -> UInt32 {
            func expand(_ x: UInt32) -> UInt32 {
                var v = x & 0x3ff
                v = (v | (v << 16)) & 0x0300_00ff
                v = (v | (v << 8)) & 0x0300_f00f
                v = (v | (v << 4)) & 0x030c_30c3
                v = (v | (v << 2)) & 0x0924_9249
                return v
            }
            let q = (p - lower) * Vec3(1023 / extent.x, 1023 / extent.y, 1023 / extent.z)
            return expand(UInt32(q.x)) | (expand(UInt32(q.y)) << 1) | (expand(UInt32(q.z)) << 2)
        }
        let order = (0..<n).sorted { a, b in
            let ma = morton(vertices[a].position)
            let mb = morton(vertices[b].position)
            return ma != mb ? ma < mb : a < b
        }
        var remap = [UInt32](repeating: 0, count: n)
        for (newIndex, oldIndex) in order.enumerated() { remap[oldIndex] = UInt32(newIndex) }
        applyVertexPermutation(order: order, remap: remap)
        vertexRemap = vertexRemap.isEmpty ? remap : vertexRemap.map { remap[Int($0)] }

        // Edges sorted by graph distance from the kinematic vertices.
        var depth = [Int](repeating: Int.max, count: n)
        var adjacency = [[Int]](repeating: [], count: n)
        for e in edges {
            adjacency[Int(e.vertex.x)].append(Int(e.vertex.y))
            adjacency[Int(e.vertex.y)].append(Int(e.vertex.x))
        }
        var queue: [Int] = []
        for i in 0..<n where vertices[i].invMass == 0 {
            depth[i] = 0
            queue.append(i)
        }
        var head = 0
        while head < queue.count {
            let v = queue[head]
            head += 1
            for w in adjacency[v] where depth[w] == Int.max {
                depth[w] = depth[v] + 1
                queue.append(w)
            }
        }
        let edgeDepth = edges.map { min(depth[Int($0.vertex.x)], depth[Int($0.vertex.y)]) }
        let edgeOrder = edges.indices.sorted { a, b in
            edgeDepth[a] != edgeDepth[b] ? edgeDepth[a] < edgeDepth[b] : a < b
        }
        edges = edgeOrder.map { edges[$0] }

        buildPatches()
    }

    private mutating func applyVertexPermutation(order: [Int], remap: [UInt32]) {
        vertices = order.map { vertices[$0] }
        func r(_ v: UInt32) -> UInt32 { remap[Int(v)] }
        for i in faces.indices {
            faces[i].vertex = SIMD3(r(faces[i].vertex.x), r(faces[i].vertex.y), r(faces[i].vertex.z))
        }
        for i in edges.indices {
            edges[i].vertex = SIMD2(r(edges[i].vertex.x), r(edges[i].vertex.y))
        }
        for i in dihedralBends.indices {
            let v = dihedralBends[i].vertex
            dihedralBends[i].vertex = SIMD4(r(v.x), r(v.y), r(v.z), r(v.w))
        }
        for i in volumes.indices {
            let v = volumes[i].vertex
            volumes[i].vertex = SIMD4(r(v.x), r(v.y), r(v.z), r(v.w))
        }
        for i in longRangeAttachments.indices {
            longRangeAttachments[i].anchor = r(longRangeAttachments[i].anchor)
            longRangeAttachments[i].vertex = r(longRangeAttachments[i].vertex)
        }
        for i in skinned.indices {
            skinned[i].vertex = r(skinned[i].vertex)
        }
        calculateSkinnedNormalFaces()
    }

    /// Patches of consecutive vertices for collision batching.
    mutating func buildPatches() {
        let size = 32
        patches = stride(from: 0, to: vertices.count, by: size).map { $0..<min($0 + size, vertices.count) }
    }

    // MARK: Validation

    /// Check indices and values. Returns a description of the first problem found.
    public func validate() -> String? {
        let n = UInt32(vertices.count)
        guard n > 0 else { return "no vertices" }
        for v in vertices where !isValidVec3(v.position) || !isValidVec3(v.velocity) || !(v.invMass >= 0) {
            return "invalid vertex"
        }
        for f in faces where f.vertex.x >= n || f.vertex.y >= n || f.vertex.z >= n { return "face index out of range" }
        for e in edges where e.vertex.x >= n || e.vertex.y >= n || !(e.compliance >= 0) || e.vertex.x == e.vertex.y {
            return "invalid edge"
        }
        for b in dihedralBends where b.vertex.x >= n || b.vertex.y >= n || b.vertex.z >= n || b.vertex.w >= n
            || !(b.compliance >= 0)
        {
            return "invalid dihedral bend"
        }
        for t in volumes where t.vertex.x >= n || t.vertex.y >= n || t.vertex.z >= n || t.vertex.w >= n
            || !(t.compliance >= 0)
        {
            return "invalid volume"
        }
        for a in longRangeAttachments where a.anchor >= n || a.vertex >= n || !(a.maxDistance >= 0) {
            return "invalid long range attachment"
        }
        let jointCount = UInt32(inverseBindMatrices.count)
        for s in skinned {
            guard s.vertex < n else { return "skinned vertex out of range" }
            for k in 0..<4 where s.weights[k] > 0 && s.joints[k] >= jointCount {
                return "skinned joint out of range"
            }
        }
        return nil
    }

    // MARK: Factories

    /// A rectangular cloth in the xz plane centered on the origin.
    /// - pinned: returns true for grid coordinates that are kinematic.
    public static func makeCloth(
        columns: Int, rows: Int, spacing: Float, attributes: SoftBodyVertexAttributes = SoftBodyVertexAttributes(),
        bendType: SoftBodyBendType = .distance, pinned: (Int, Int) -> Bool = { _, _ in false }
    ) -> SoftBodySettings {
        var settings = SoftBodySettings()
        guard columns >= 2, rows >= 2 else { return settings }
        let offsetX = -0.5 * Float(columns - 1) * spacing
        let offsetZ = -0.5 * Float(rows - 1) * spacing
        for z in 0..<rows {
            for x in 0..<columns {
                let position = Vec3(offsetX + Float(x) * spacing, 0, offsetZ + Float(z) * spacing)
                settings.vertices.append(SoftBodyVertex(position: position, invMass: pinned(x, z) ? 0 : 1))
            }
        }
        func index(_ x: Int, _ z: Int) -> UInt32 { UInt32(x + z * columns) }
        for z in 0..<(rows - 1) {
            for x in 0..<(columns - 1) {
                // Faces point up (+y).
                settings.faces.append(SoftBodyFace(index(x, z), index(x, z + 1), index(x + 1, z + 1)))
                settings.faces.append(SoftBodyFace(index(x, z), index(x + 1, z + 1), index(x + 1, z)))
            }
        }
        settings.createConstraints([attributes], bendType: bendType)
        settings.optimize()
        return settings
    }

    /// A solid box of tetrahedra with `count` vertices per side.
    public static func makeCube(
        count: Int, spacing: Float, compliance: Float = 0, volumeCompliance: Float = 0
    ) -> SoftBodySettings {
        var settings = SoftBodySettings()
        guard count >= 2 else { return settings }
        let offset = -0.5 * Float(count - 1) * spacing
        func index(_ x: Int, _ y: Int, _ z: Int) -> UInt32 { UInt32(x + y * count + z * count * count) }
        for z in 0..<count {
            for y in 0..<count {
                for x in 0..<count {
                    let p = Vec3(offset + Float(x) * spacing, offset + Float(y) * spacing, offset + Float(z) * spacing)
                    settings.vertices.append(SoftBodyVertex(position: p))
                }
            }
        }

        var edgeSet = Set<UInt64>()
        func edge(_ a: UInt32, _ b: UInt32) {
            let key = UInt64(min(a, b)) << 32 | UInt64(max(a, b))
            if edgeSet.insert(key).inserted {
                settings.edges.append(SoftBodyEdge(a, b, compliance: compliance))
            }
        }

        // Five tetrahedra per cell, mirrored on alternate cells so the faces match.
        for z in 0..<(count - 1) {
            for y in 0..<(count - 1) {
                for x in 0..<(count - 1) {
                    var c = [UInt32]()
                    for k in 0..<8 {
                        c.append(index(x + (k & 1), y + ((k >> 1) & 1), z + ((k >> 2) & 1)))
                    }
                    let tets: [[Int]] = (x + y + z) % 2 == 0
                        ? [[0, 1, 2, 4], [1, 3, 2, 7], [1, 4, 5, 7], [2, 4, 7, 6], [1, 2, 4, 7]]
                        : [[0, 1, 3, 5], [0, 3, 2, 6], [0, 5, 4, 6], [3, 5, 6, 7], [0, 3, 6, 5]]
                    for t in tets {
                        var volume = SoftBodyVolume(c[t[0]], c[t[1]], c[t[2]], c[t[3]], compliance: volumeCompliance)
                        let p = { (i: UInt32) in settings.vertices[Int(i)].position }
                        let x0 = p(volume.vertex.x)
                        if dot(p(volume.vertex.y) - x0, cross(p(volume.vertex.z) - x0, p(volume.vertex.w) - x0)) < 0 {
                            volume.vertex = SIMD4(volume.vertex.y, volume.vertex.x, volume.vertex.z, volume.vertex.w)
                        }
                        settings.volumes.append(volume)
                        for i in 0..<4 {
                            for j in (i + 1)..<4 {
                                edge(c[t[i]], c[t[j]])
                            }
                        }
                    }
                }
            }
        }

        // Boundary faces, outward facing.
        let last = count - 1
        func quad(_ a: UInt32, _ b: UInt32, _ c: UInt32, _ d: UInt32) {
            settings.faces.append(SoftBodyFace(a, b, c))
            settings.faces.append(SoftBodyFace(a, c, d))
        }
        for i in 0..<last {
            for j in 0..<last {
                quad(index(i, j, 0), index(i, j + 1, 0), index(i + 1, j + 1, 0), index(i + 1, j, 0))
                quad(index(i, j, last), index(i + 1, j, last), index(i + 1, j + 1, last), index(i, j + 1, last))
                quad(index(0, i, j), index(0, i, j + 1), index(0, i + 1, j + 1), index(0, i + 1, j))
                quad(index(last, i, j), index(last, i + 1, j), index(last, i + 1, j + 1), index(last, i, j + 1))
                quad(index(i, 0, j), index(i + 1, 0, j), index(i + 1, 0, j + 1), index(i, 0, j + 1))
                quad(index(i, last, j), index(i, last, j + 1), index(i + 1, last, j + 1), index(i + 1, last, j))
            }
        }

        settings.calculateRestValues()
        settings.optimize()
        return settings
    }

    /// A closed sphere surface. Combine with `SoftBodyDef.pressure` for a balloon.
    public static func makeSphere(
        radius: Float, subdivisions: Int = 2, compliance: Float = 0, bendCompliance: Float = 1.0e-3,
        bendType: SoftBodyBendType = .dihedral
    ) -> SoftBodySettings {
        var settings = SoftBodySettings()

        // Icosahedron.
        let t: Float = (1 + Float(5).squareRoot()) / 2
        var points: [Vec3] = [
            Vec3(-1, t, 0), Vec3(1, t, 0), Vec3(-1, -t, 0), Vec3(1, -t, 0),
            Vec3(0, -1, t), Vec3(0, 1, t), Vec3(0, -1, -t), Vec3(0, 1, -t),
            Vec3(t, 0, -1), Vec3(t, 0, 1), Vec3(-t, 0, -1), Vec3(-t, 0, 1),
        ].map { normalize($0) }
        var triangles: [SIMD3<UInt32>] = [
            SIMD3(0, 11, 5), SIMD3(0, 5, 1), SIMD3(0, 1, 7), SIMD3(0, 7, 10), SIMD3(0, 10, 11),
            SIMD3(1, 5, 9), SIMD3(5, 11, 4), SIMD3(11, 10, 2), SIMD3(10, 7, 6), SIMD3(7, 1, 8),
            SIMD3(3, 9, 4), SIMD3(3, 4, 2), SIMD3(3, 2, 6), SIMD3(3, 6, 8), SIMD3(3, 8, 9),
            SIMD3(4, 9, 5), SIMD3(2, 4, 11), SIMD3(6, 2, 10), SIMD3(8, 6, 7), SIMD3(9, 8, 1),
        ]
        for _ in 0..<max(0, subdivisions) {
            var midpoints: [UInt64: UInt32] = [:]
            func midpoint(_ a: UInt32, _ b: UInt32) -> UInt32 {
                let key = UInt64(min(a, b)) << 32 | UInt64(max(a, b))
                if let m = midpoints[key] { return m }
                points.append(normalize(points[Int(a)] + points[Int(b)]))
                let m = UInt32(points.count - 1)
                midpoints[key] = m
                return m
            }
            var next: [SIMD3<UInt32>] = []
            for tri in triangles {
                let a = midpoint(tri.x, tri.y)
                let b = midpoint(tri.y, tri.z)
                let c = midpoint(tri.z, tri.x)
                next.append(SIMD3(tri.x, a, c))
                next.append(SIMD3(tri.y, b, a))
                next.append(SIMD3(tri.z, c, b))
                next.append(SIMD3(a, b, c))
            }
            triangles = next
        }

        settings.vertices = points.map { SoftBodyVertex(position: radius * $0) }
        settings.faces = triangles.map { SoftBodyFace($0.x, $0.y, $0.z) }
        settings.createConstraints(
            [SoftBodyVertexAttributes(compliance: compliance, shearCompliance: compliance, bendCompliance: bendCompliance)],
            bendType: bendType)
        settings.optimize()
        return settings
    }

    /// A chain of vertices along -y from the origin. The first vertex is kinematic when `pinned`.
    public static func makeRope(count: Int, spacing: Float, compliance: Float = 0, pinned: Bool = true) -> SoftBodySettings {
        var settings = SoftBodySettings()
        for i in 0..<max(count, 0) {
            settings.vertices.append(SoftBodyVertex(position: Vec3(0, -Float(i) * spacing, 0), invMass: i == 0 && pinned ? 0 : 1))
            if i > 0 {
                settings.edges.append(SoftBodyEdge(UInt32(i - 1), UInt32(i), compliance: compliance))
            }
        }
        if pinned {
            for i in 1..<max(count, 1) {
                settings.longRangeAttachments.append(SoftBodyLRA(anchor: 0, vertex: UInt32(i)))
            }
        }
        settings.calculateRestValues()
        settings.optimize()
        return settings
    }
}

// MARK: - Geometry helpers

/// Signed dihedral angle around the edge (x0, x1) between triangles (x0, x1, x2) and (x1, x0, x3).
/// Zero when the triangles are coplanar and unfolded.
@inline(__always)
func dihedralAngle(_ x0: Vec3, _ x1: Vec3, _ x2: Vec3, _ x3: Vec3) -> Float {
    let e = x1 - x0
    let n1 = cross(e, x2 - x0)
    let n2 = cross(x3 - x0, e)
    let len = length(e)
    guard len > 0 else { return 0 }
    return atan2Approx(dot(cross(n1, n2), e) / len, dot(n1, n2))
}

/// A small binary min heap of (distance, vertex).
private struct BinaryHeap {
    var items: [(Float, Int)] = []

    mutating func push(_ d: Float, _ v: Int) {
        items.append((d, v))
        var i = items.count - 1
        while i > 0 {
            let parent = (i - 1) / 2
            if items[parent].0 <= items[i].0 { break }
            items.swapAt(parent, i)
            i = parent
        }
    }

    mutating func pop() -> (Float, Int)? {
        guard let first = items.first else { return nil }
        let last = items.removeLast()
        if items.isEmpty == false {
            items[0] = last
            var i = 0
            while true {
                let l = 2 * i + 1
                let r = l + 1
                var smallest = i
                if l < items.count && items[l].0 < items[smallest].0 { smallest = l }
                if r < items.count && items[r].0 < items[smallest].0 { smallest = r }
                if smallest == i { break }
                items.swapAt(smallest, i)
                i = smallest
            }
        }
        return first
    }
}
