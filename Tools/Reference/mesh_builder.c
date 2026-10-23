// Test oracle only; never linked into Natsel.
// clang -O2 -DNDEBUG -ffp-contract=off -I <box3d>/src -I <box3d>/include mesh_builder.c <box3d-build>/src/libbox3d.a -o /tmp/mesh-reference
#include "mesh.c"
#include <stdio.h>

static uint64_t Digest(uint64_t hash, uint32_t word)
{
    for (int i = 0; i < 4; ++i) { hash ^= (word >> (8 * i)) & 255; hash *= 1099511628211ull; }
    return hash;
}
static uint64_t FloatDigest(uint64_t hash, float value)
{
    uint32_t word;
    memcpy(&word, &value, 4);
    return Digest(hash, word);
}
static void MeshRayReference(void)
{
    b3MeshData* data = b3CreateGridMesh(5, 4, 0.75f, 3, true);
    printf("grid hash 0x%016llx bytes %d\n", (unsigned long long)data->hash, data->byteCount);
    for (int scenario = 0; scenario < 3; ++scenario) {
        b3Vec3 scales[] = {{1,1,1}, {-2,0.5f,1.5f}, {1,-1,1}};
        b3Mesh mesh = {.data=data, .scale=scales[scenario]};
        uint64_t hash = 14695981039346656037ull;
        for (int i = 0; i < 80; ++i) {
            b3RayCastInput ray = {.origin = {(float)(i % 10) * 0.5f - 2.25f, i < 40 ? 2.0f : -2.0f, (float)(i % 8) * 0.5f - 1.75f},
                .translation = {0.125f, i < 40 ? -4.0f : 4.0f, -0.25f}, .maxFraction = i % 3 == 0 ? 0.5f : 1.0f};
            b3CastOutput out = b3RayCastMesh(&mesh, &ray);
            hash = FloatDigest(hash, out.normal.x); hash = FloatDigest(hash, out.normal.y); hash = FloatDigest(hash, out.normal.z);
            hash = FloatDigest(hash, out.point.x); hash = FloatDigest(hash, out.point.y); hash = FloatDigest(hash, out.point.z);
            hash = FloatDigest(hash, out.fraction); hash = Digest(hash, out.triangleIndex);
            hash = Digest(hash, out.materialIndex); hash = Digest(hash, out.hit);
        }
        printf("ray %d digest 0x%016llx\n", scenario, (unsigned long long)hash);
    }
    b3DestroyMesh(data);
}
static void MeshStorageReference(void)

{
    b3Vec3 vertices[] = {{0,0,0},{1,0,0},{0,0,1},{1,0,1},{0,1,0},{0,0,0},{2,0,0}};
    int32_t indices[] = {0,2,1,1,2,3,0,1,4,5,2,1,0,0,1,0,1,5,0,1,6};
    uint8_t materials[] = {0,1,2,3,4,5,6};
    for (int scenario = 0; scenario < 3; ++scenario) {
        b3MeshDef def = {.vertices=vertices,.indices=indices,.materialIndices=materials,.vertexCount=7,.triangleCount=7,
            .identifyEdges=scenario > 0,.weldVertices=scenario==2,.weldTolerance=0.001f,.clockWiseWinding=scenario==2};
        b3MeshData* mesh = b3CreateMesh(&def, NULL, 0);
        printf("mesh %d bytes %d hash 0x%016llx area %a degenerates %d triangles %d vertices %d materials %d flags", scenario,
            mesh->byteCount, (unsigned long long)mesh->hash, mesh->surfaceArea, mesh->degenerateCount, mesh->triangleCount, mesh->vertexCount, mesh->materialCount);
        const uint8_t* flags = b3GetMeshFlags(mesh);
        for (int i = 0; i < mesh->triangleCount; ++i) printf(" %02x", flags[i]);
        puts("");
        b3DestroyMesh(mesh);
    }
}
int main(void)

{
    for (int scenario = 0; scenario < 4; ++scenario) {
        int count = scenario < 2 ? 37 : (scenario == 2 ? 7 : 17);
        b3Primitive primitives[37];
        for (int i = 0; i < count; ++i) {
            b3Vec3 center = scenario < 2 ? (b3Vec3){(float)((i * 17) % 19) * 0.25f - 2.0f,
                (float)((i * 7) % 13) * 0.5f, (float)((i * 3) % 11) * 0.125f} : (b3Vec3){0, 0, 0};
            b3Vec3 extent = {0.125f, 0.25f, 0.0625f};
            primitives[i] = (b3Primitive){.aabb = {b3Sub(center, extent), b3Add(center, extent)}, .center = center, .triangleIndex = i};
        }
        b3Array(b3MeshNode) nodes = {0};
        int height = 0;
        b3BuildRecursive(&nodes, count, primitives, primitives, scenario == 1, &height);
        uint64_t hash = 14695981039346656037ull;
        for (int i = 0; i < count; ++i) hash = Digest(hash, primitives[i].triangleIndex);
        for (int i = 0; i < nodes.count; ++i) {
            b3MeshNode n = nodes.data[i];
            hash = FloatDigest(hash, n.lowerBound.x); hash = FloatDigest(hash, n.lowerBound.y); hash = FloatDigest(hash, n.lowerBound.z);
            uint32_t word; memcpy(&word, &n.data, 4); hash = Digest(hash, word);
            hash = FloatDigest(hash, n.upperBound.x); hash = FloatDigest(hash, n.upperBound.y); hash = FloatDigest(hash, n.upperBound.z);
            hash = Digest(hash, n.triangleOffset);
        }
        printf("scenario %d count %d nodes %d height %d digest 0x%016llx\n", scenario, count, nodes.count, height, (unsigned long long)hash);
        b3Array_Destroy(nodes);
    }
    MeshStorageReference();
    MeshRayReference();
    b3Vec3 source[] = {{-0.375f,0,0}, {0.375f,0,0}, {0,0,0}, {-0.75f,0,0}, {0.75f,0,0}, {0.125f,0.5f,-0.5f}, {0.25f,0.625f,-0.625f}, {0.375f,0.75f,-0.75f}};
    int32_t indices[] = {0,1,2,3,4,5,6,7};
    b3Vec3 vertices[8]; int32_t mapped[8];
    b3WeldData data = {.srcVertices=source,.srcIndices=indices,.dstVertices=vertices,.dstIndices=mapped,.vertexCount=8,.indexCount=8,.stride=sizeof(b3Vec3)};
    int count = b3WeldVertices(&data, 0.5f);
    printf("weld count %d mapping", count);
    for (int i = 0; i < 8; ++i) printf(" %d", mapped[i]);
    puts("");
}
