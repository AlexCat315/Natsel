// Test oracle only; never linked into Natsel.
// clang -O2 -DNDEBUG -ffp-contract=off -I <box3d>/src -I <box3d>/include height_text.c <build>/src/libbox3d.a -o /tmp/height-text
#include "core.h"
#include "box3d/collision.h"
#include <stdio.h>
int main(void) {
    float heights[42]; uint8_t materials[30];
    for(int i=0;i<42;++i) heights[i]=(float)((i*17)%29)*0.125f-1.5f;
    for(int i=0;i<30;++i) materials[i]=i%11==0?255:i%3;
    b3HeightFieldDef def={.heights=heights,.materialIndices=materials,.scale={0.75f,1.25f,0.5f},.countX=7,.countZ=6,
        .globalMinimumHeight=-1,.globalMaximumHeight=1,.clockwiseWinding=true};
    const char* path="/tmp/natsel-height-c-format.txt";
    b3DumpHeightData(&def,path);
    FILE* file=fopen(path,"r"); unsigned char bytes[4096]; int count=(int)fread(bytes,1,sizeof(bytes),file); fclose(file);
    b3HeightFieldData* field=b3LoadHeightField(path);
    printf("text bytes %d hash 0x%016llx loaded 0x%016llx\n",count,(unsigned long long)b3Hash64NonZero(bytes,count),(unsigned long long)field->hash);
    b3DestroyHeightField(field);
}
