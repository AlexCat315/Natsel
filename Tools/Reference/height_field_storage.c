// Test oracle only; never linked into Natsel.
// clang -O2 -DNDEBUG -ffp-contract=off -I <box3d>/src -I <box3d>/include height_field_storage.c <build>/src/libbox3d.a -o /tmp/height-storage
#include "height_field.c"
#include "height_fixture.h"
static uint64_t Word(uint64_t hash,uint32_t word) {
    for(int i=0;i<4;++i) { hash^=(word>>(8*i))&255; hash*=1099511628211ull; } return hash;
}
static uint64_t Scalar(uint64_t hash,float value) { uint32_t word; memcpy(&word,&value,4); return Word(hash,word); }
static uint64_t Vector(uint64_t hash,b3Vec3 value) { return Scalar(Scalar(Scalar(hash,value.x),value.y),value.z); }
int main(void) {
    for(int scenario=0;scenario<6;++scenario) {
        b3HeightFieldData* field=MakeReferenceHeightField(scenario);
        uint64_t hash=14695981039346656037ull;
        for(int i=0;i<60;++i) {
            if(b3GetHeightFieldMaterial(field,i)==B3_HEIGHT_FIELD_HOLE) continue;
            b3Triangle t=b3GetHeightFieldTriangle(field,i);
            for(int j=0;j<3;++j) hash=Vector(hash,t.vertices[j]);
            hash=Word(hash,t.i1); hash=Word(hash,t.i2); hash=Word(hash,t.i3); hash=Word(hash,t.flags);
        }
        printf("scenario %d bytes %d hash 0x%016llx triangles 0x%016llx\n",scenario,field->byteCount,
            (unsigned long long)field->hash,(unsigned long long)hash);
        b3DestroyHeightField(field);
    }
}
