// Test oracle only; never linked into Natsel.
// clang -O2 -DNDEBUG -ffp-contract=off -I <box3d>/src -I <box3d>/include height_queries.c <build>/src/libbox3d.a -o /tmp/height-queries
#include "height_field.c"
#include "height_fixture.h"
static uint64_t Word(uint64_t hash,uint32_t word) {
    for(int i=0;i<4;++i) { hash^=(word>>(8*i))&255; hash*=1099511628211ull; } return hash;
}
static uint64_t Scalar(uint64_t hash,float value) { uint32_t word; memcpy(&word,&value,4); return Word(hash,word); }
static uint64_t Vector(uint64_t hash,b3Vec3 value) { return Scalar(Scalar(Scalar(hash,value.x),value.y),value.z); }
static uint64_t Cast(uint64_t hash,b3CastOutput out) {
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
    for(int scenario=0;scenario<6;++scenario) {
        b3HeightFieldData* field=MakeReferenceHeightField(scenario);
        uint64_t rays=14695981039346656037ull,casts=rays,overlaps=rays,queries=rays,movers=rays;
        b3Transform transform={{0.125f,-0.25f,0.5f},b3MakeQuatFromAxisAngle((b3Vec3){0,1,0},0.35f)};
        for(int i=0;i<120;++i) {
            float x=(float)((i*7)%19)*0.375f-1.0f,z=(float)((i*11)%17)*0.25f-0.75f;
            float y=i<60?2.0f:-2.0f;
            b3Vec3 points[]={{x,y,z},{x+0.1f,y+0.5f,z-0.15f}};
            b3ShapeCastInput input={.proxy={points,2,(float)(i%3)*0.15f},
                .translation={(float)(i%4-2)*1.25f,i<60?-4.0f:4.0f,(float)(i%5-2)*0.75f},
                .maxFraction=i%3==0?0.35f:1.0f,.canEncroach=i%2==0};
            casts=Cast(casts,b3ShapeCastHeightField(field,&input));
            b3RayCastInput ray={points[0],input.translation,input.maxFraction};
            rays=Cast(rays,b3RayCastHeightField(field,&ray));
            points[0].y=(float)(i%7)*0.3f-1.0f; points[1].y=points[0].y+0.5f;
            overlaps=Word(overlaps,b3OverlapHeightField(field,transform,&input.proxy));
            b3Vec3 center={x,(float)(i%3)*0.3f-0.3f,z},extent={0.45f,0.3f,0.375f};
            b3AABB bounds={b3Sub(center,extent),b3Add(center,extent)};
            Query q={queries,0,i%4==0?3:100000};
            b3QueryHeightField(field,bounds,Triangle,&q); queries=Word(q.hash,q.count);
            b3Capsule capsule={points[0],points[1],0.4f}; b3PlaneResult planes[4];
            int count=b3CollideMoverAndHeightField(planes,1+i%4,field,&capsule);
            movers=Word(movers,count);
            for(int p=0;p<count;++p) {
                movers=Vector(movers,planes[p].plane.normal); movers=Scalar(movers,planes[p].plane.offset);
                movers=Vector(movers,planes[p].point); movers=Word(movers,planes[p].triangleIndex);
                movers=Word(movers,planes[p].childIndex); movers=Word(movers,planes[p].materialIndex);
            }
        }
        printf("scenario %d ray 0x%016llx cast 0x%016llx overlap 0x%016llx query 0x%016llx mover 0x%016llx\n",scenario,
            (unsigned long long)rays,(unsigned long long)casts,(unsigned long long)overlaps,(unsigned long long)queries,(unsigned long long)movers);
        b3DestroyHeightField(field);
    }
}
