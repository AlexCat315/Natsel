// Test oracle only; never linked into Natsel.
// clang -O2 -DNDEBUG -ffp-contract=off -I <box3d>/src -I <box3d>/include mesh_reduction.c <build>/src/libbox3d.a -o /tmp/mesh-reduction
#include "mesh_contact.c"
static uint64_t Word(uint64_t hash, uint32_t word) {
    for (int i=0;i<4;++i) { hash ^= (word >> (8*i)) & 255; hash *= 1099511628211ull; }
    return hash;
}
static uint64_t Scalar(uint64_t hash, float value) { uint32_t word; memcpy(&word,&value,4); return Word(hash,word); }
int main(void) {
    for(int scenario=0;scenario<3;++scenario) {
        uint64_t hash=14695981039346656037ull;
        for(int trial=0;trial<100;++trial) {
            int count=trial%24+1;
            b3Point2D points[24];
            for(int i=0;i<count;++i) {
                float x=(float)((i*17+trial*3)%19)-9.0f;
                float y=(float)((i*7+trial*5)%13)-6.0f;
                if(scenario==0) { x*=0.00001f; y*=0.00001f; }
                if(scenario==1) y=0;
                points[i]=(b3Point2D){{x,y},(float)((i*3+trial)%11)*-0.002f,i};
            }
            int result=b3CullPoints(points,count);
            hash=Word(hash,result);
            for(int i=0;i<result;++i) {
                hash=Scalar(hash,points[i].p.x); hash=Scalar(hash,points[i].p.y);
                hash=Scalar(hash,points[i].separation); hash=Word(hash,points[i].originalIndex);
            }
        }
        printf("scenario %d digest 0x%016llx\n",scenario,(unsigned long long)hash);
    }
}
