// Test oracle only; never linked into Natsel.
// clang -O2 -DNDEBUG -ffp-contract=off -I <box3d>/src -I <box3d>/include compound_storage.c <build>/src/libbox3d.a -o /tmp/compound-storage
#include "compound.c"
#include "core.h"
#include "compound_fixture.h"
#include <stdio.h>
int main(void) {
    for(int scenario=0;scenario<6;++scenario) {
        b3CompoundData* compound=MakeReferenceCompound(scenario);
        printf("scenario %d bytes %d hash 0x%016llx height %d nodes %d materials %d hulls %d meshes %d\n",scenario,compound->byteCount,
            (unsigned long long)b3Hash64NonZero((const uint8_t*)compound,compound->byteCount),compound->treeHeight,
            compound->nodeCount,compound->materialCount,compound->sharedHullCount,compound->sharedMeshCount);
        b3DestroyCompound(compound);
    }
}
