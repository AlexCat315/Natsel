// Test oracle only; never linked into Natsel.
// clang -O2 -DNDEBUG -ffp-contract=off -I <box3d>/src -I <box3d>/include mesh_queries.c <build>/src/libbox3d.a -o /tmp/mesh-queries
#include "mesh.c"
#include <stdio.h>

static uint64_t Word(uint64_t hash, uint32_t word) {
    for (int i=0;i<4;++i) { hash ^= (word >> (8*i)) & 255; hash *= 1099511628211ull; }
    return hash;
}
static uint64_t Scalar(uint64_t hash, float value) { uint32_t word; memcpy(&word,&value,4); return Word(hash,word); }
static uint64_t Vector(uint64_t hash, b3Vec3 value) { return Scalar(Scalar(Scalar(hash,value.x),value.y),value.z); }
static uint64_t Cast(uint64_t hash, b3CastOutput out) {
    hash=Vector(Vector(hash,out.normal),out.point); hash=Scalar(hash,out.fraction);
    hash=Word(hash,out.iterations); hash=Word(hash,out.triangleIndex); hash=Word(hash,out.childIndex);
    return Word(Word(hash,out.materialIndex),out.hit);
}
typedef struct Query { uint64_t hash; int count; int limit; } Query;
static bool Triangle(b3Vec3 a,b3Vec3 b,b3Vec3 c,int index,void* context) {
    Query* q=context;
    q->hash=Word(Vector(Vector(Vector(q->hash,a),b),c),index);
    return ++q->count < q->limit;
}
int main(void) {
    b3MeshData* geometries[]={b3CreateGridMesh(5,4,0.75f,3,true),
        b3CreateWaveMesh(5,4,0.75f,0.5f,0.3f,0.2f), b3CreateTorusMesh(7,9,1.25f,0.375f)};
    for(int i=0;i<3;++i) printf("geometry %d hash 0x%016llx bytes %d\n",i,(unsigned long long)geometries[i]->hash,geometries[i]->byteCount);
    b3Vec3 center={0.125f,-0.25f,0.5f}, extent={1.25f,0.75f,2.0f};
    b3MeshData* primitives[]={b3CreateBoxMesh(center,extent,true),b3CreateHollowBoxMesh(center,extent),b3CreatePlatformMesh(center,1.5f,2.0f,3.0f)};
    for(int i=0;i<3;++i) { printf("primitive %d hash 0x%016llx bytes %d\n",i,(unsigned long long)primitives[i]->hash,primitives[i]->byteCount); b3DestroyMesh(primitives[i]); }

    b3Vec3 scales[]={{1,1,1},{-2,0.5f,1.5f},{1,-1,1}};
    for(int scenario=0;scenario<9;++scenario) {
        b3Mesh mesh={geometries[scenario/3],scales[scenario%3]};
        uint64_t cast=14695981039346656037ull, overlap=cast, query=cast, mover=cast;
        b3Transform transform={{0.125f,-0.25f,0.5f},b3MakeQuatFromAxisAngle((b3Vec3){0,1,0},0.35f)};
        for(int i=0;i<80;++i) {
            float x=(float)(i%10)*0.5f-2.25f, z=(float)(i%8)*0.5f-1.75f;
            float y=i<40?2.0f:-2.0f;
            b3Vec3 points[]={{x,y,z},{x+0.125f,y+0.5f,z-0.25f}};
            b3ShapeCastInput input={.proxy={points,2,0.25f},.translation={0.125f,i<40?-4.0f:4.0f,-0.25f},
                .maxFraction=i%3==0?0.5f:1.0f,.canEncroach=i%2==0};
            cast=Cast(cast,b3ShapeCastMesh(&mesh,&input));
            points[0].y=(float)(i%5)*0.2f-0.3f; points[1].y=points[0].y+0.5f;
            overlap=Word(overlap,b3OverlapMesh(&mesh,transform,&input.proxy));
            b3Vec3 center={x,(float)(i%3)*0.3f-0.3f,z}, extent={0.375f,0.2f,0.375f};
            b3AABB bounds={b3Sub(center,extent),b3Add(center,extent)};
            Query q={query,0,i%4==0?3:100000};
            b3QueryMesh(&mesh,bounds,Triangle,&q); query=Word(q.hash,q.count);
            b3Capsule capsule={points[0],points[1],0.4f};
            b3PlaneResult planes[4];
            int count=b3CollideMoverAndMesh(planes,i%5,&mesh,&capsule);
            mover=Word(mover,count);
            for(int p=0;p<count;++p) {
                mover=Vector(mover,planes[p].plane.normal); mover=Scalar(mover,planes[p].plane.offset);
                mover=Vector(mover,planes[p].point); mover=Word(mover,planes[p].triangleIndex);
                mover=Word(mover,planes[p].childIndex); mover=Word(mover,planes[p].materialIndex);
            }
        }
        printf("scenario %d cast 0x%016llx overlap 0x%016llx query 0x%016llx mover 0x%016llx\n",scenario,
            (unsigned long long)cast,(unsigned long long)overlap,(unsigned long long)query,(unsigned long long)mover);
    }
    for(int i=0;i<3;++i) b3DestroyMesh(geometries[i]);
}
